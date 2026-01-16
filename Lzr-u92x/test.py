import serial
import struct
import numpy as np
import time
import cv2
import open3d as o3d
import multiprocessing as mp  # [FIX] Switched from threading to multiprocessing
import queue  # Still needed for Empty exceptions if used, but mp.Queue is primary
import os
import sys
import glob
from datetime import datetime
import statistics


# =====================================================
# CONFIGURATION
# =====================================================
def get_default_port():
    """Automatically selects the correct port based on OS."""
    if sys.platform.startswith('win'):
        return 'COM3'  # Windows: Check Device Manager if this fails
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # Check for typical USB serial adapters
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        if ports:
            return ports[0]
        return '/dev/ttyUSB0'  # Fallback
    return '/dev/ttyUSB0'


SERIAL_PORT = get_default_port()
BAUD_RATE = 921600

# Sensor Properties
START_ANGLE = -48.0
ANGULAR_RES = 0.3516

# Calibration / Background Subtraction
CALIBRATION_FRAMES = 4000
GRID_CELL_SIZE = 0.05  # Size of background grid cells (meters)

# Filtering Limits
MIN_RANGE_M = 0.10
MAX_RANGE_M = 2.25
MAX_NEIGHBOR_JUMP = 0.15

# Vehicle Logic
VEHICLE_SPEED_KMPH = 5
VEHICLE_SPEED_MPS = VEHICLE_SPEED_KMPH / 3.6
TRIGGER_THRESHOLD = 50  # Minimum points to consider it a vehicle
IDLE_TIMEOUT = 0.8  # Seconds of silence to consider vehicle passed

# --- Image Generation Parameters ---
GRID_RES = 0.005  # 5mm per pixel resolution
MAX_DIST_INTENSITY = 50.0

# 3D Rotation Correction (Degrees)
ROTATION_X_DEG = 25.0  # Pitch
ROTATION_Y_DEG = 0.0  # Roll
ROTATION_Z_DEG = 0.0  # Yaw

# Dynamic Frame Settings
FRAME_PADDING_M = 0.2
MIN_IMAGE_DIM_M = 1.0


# =====================================================
# HELPER: ROBUST MEDIAN (MAD)
# =====================================================
def robust_median(distances):
    """Calculates median ignoring outliers using Median Absolute Deviation."""
    if len(distances) < 5:
        return None
    median = statistics.median(distances)
    deviations = [abs(d - median) for d in distances]
    mad = statistics.median(deviations)
    if mad == 0:
        return median
    filtered = [d for d in distances if abs(d - median) <= 3 * mad]
    return statistics.median(filtered) if filtered else None


# =====================================================
# BACKGROUND WORKER (RUNS ON SEPARATE CORE)
# =====================================================
def background_worker(input_queue):
    """
    Standalone function that runs in a separate process.
    This prevents heavy math from blocking the serial reader.
    """
    print("🧵 Background Worker Process Started (PID: {})".format(os.getpid()))

    while True:
        try:
            # Block until data is available
            extracted_stack, ts = input_queue.get()

            # Safety Check
            if len(extracted_stack) < 30:
                print(f"⚠️ Vehicle {ts} Skipped: Too few points ({len(extracted_stack)})")
                continue

            folder = f"vehicle_{ts}"
            os.makedirs(folder, exist_ok=True)

            # ---------------------------------------------------------
            # 1. Cleaning (Statistical Outlier Removal)
            # ---------------------------------------------------------
            pcd = o3d.geometry.PointCloud()
            # Force float64 for stability on ARM (Raspberry Pi)
            pcd.points = o3d.utility.Vector3dVector(np.array(extracted_stack, dtype=np.float64))

            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=25, std_ratio=0.8)
            if len(pcd.points) == 0:
                print(f"⚠️ Vehicle {ts} Skipped: Empty after statistical cleanup")
                continue

            pcd, _ = pcd.remove_radius_outlier(nb_points=12, radius=0.06)
            if len(pcd.points) == 0:
                print(f"⚠️ Vehicle {ts} Skipped: Empty after radius cleanup")
                continue

            # ---------------------------------------------------------
            # 2. Standard Side View Transformation
            # ---------------------------------------------------------
            # Old Z (Time) -> New X (Length), Old Y (Height) -> New Y (Height)
            R_standard = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=np.float64)
            pts_side = (R_standard @ np.asarray(pcd.points).T).T

            # ---------------------------------------------------------
            # 3. Apply Full 3D Mounting Rotation Correction
            # ---------------------------------------------------------
            rad_x = np.radians(ROTATION_X_DEG)
            rad_y = np.radians(ROTATION_Y_DEG)
            rad_z = np.radians(ROTATION_Z_DEG)

            # Rotation Matrices
            Rx = np.array([
                [1, 0, 0],
                [0, np.cos(rad_x), -np.sin(rad_x)],
                [0, np.sin(rad_x), np.cos(rad_x)]
            ])
            Ry = np.array([
                [np.cos(rad_y), 0, np.sin(rad_y)],
                [0, 1, 0],
                [-np.sin(rad_y), 0, np.cos(rad_y)]
            ])
            Rz = np.array([
                [np.cos(rad_z), -np.sin(rad_z), 0],
                [np.sin(rad_z), np.cos(rad_z), 0],
                [0, 0, 1]
            ])

            # Combined Rotation: R_total = Rz * Ry * Rx
            R_total = Rz @ Ry @ Rx
            pts_side = (R_total @ pts_side.T).T

            # Save Rotated PCD
            pcd_side = o3d.geometry.PointCloud()
            pcd_side.points = o3d.utility.Vector3dVector(pts_side)
            o3d.io.write_point_cloud(f"{folder}/side_view.pcd", pcd_side)

            # ---------------------------------------------------------
            # 4. Dynamic Frame Calculation
            # ---------------------------------------------------------
            min_x, min_y = np.min(pts_side[:, :2], axis=0)
            max_x, max_y = np.max(pts_side[:, :2], axis=0)

            # Apply Padding
            min_x -= FRAME_PADDING_M
            max_x += FRAME_PADDING_M
            min_y -= FRAME_PADDING_M
            max_y += FRAME_PADDING_M

            # Ensure minimum frame size
            if (max_x - min_x) < MIN_IMAGE_DIM_M: max_x = min_x + MIN_IMAGE_DIM_M
            if (max_y - min_y) < MIN_IMAGE_DIM_M: max_y = min_y + MIN_IMAGE_DIM_M

            # Calculate dynamic image resolution
            width_m = max_x - min_x
            height_m = max_y - min_y
            x_bins = int(width_m / GRID_RES)
            y_bins = int(height_m / GRID_RES)

            # ---------------------------------------------------------
            # 5. Image Generation
            # ---------------------------------------------------------
            bev = np.zeros((y_bins, x_bins), dtype=np.float32)

            xi = ((pts_side[:, 0] - min_x) / GRID_RES).astype(np.int32)
            yi = ((pts_side[:, 1] - min_y) / GRID_RES).astype(np.int32)

            valid_indices = (xi >= 0) & (xi < x_bins) & (yi >= 0) & (yi < y_bins)
            xi = xi[valid_indices]
            yi = yi[valid_indices]
            v_points = pts_side[valid_indices]

            if len(v_points) > 0:
                dist = np.sqrt(v_points[:, 0] ** 2 + v_points[:, 1] ** 2)
                intensity = 1.0 - np.minimum(dist / MAX_DIST_INTENSITY, 1.0)

                bev[yi, xi] = intensity

                # Final Processing: Flip & Blur
                bev_img = (bev * 255).astype(np.uint8)
                bev_img = np.flipud(bev_img)
                bev_img = cv2.GaussianBlur(bev_img, (3, 3), 0)

                img_path = f"{folder}/side_view_image.png"
                cv2.imwrite(img_path, bev_img)
                print(f"✅ Vehicle {ts} Processed: {x_bins}x{y_bins} px -> {img_path}")
            else:
                print(f"⚠️ Vehicle {ts} Skipped: No points in generated frame")

        except Exception as e:
            print(f"❌ Error in background worker: {e}")
            import traceback
            traceback.print_exc()


# =====================================================
# MAIN SYSTEM CLASS
# =====================================================
class TollPlazaSystem:
    def __init__(self):
        print(f"🔌 Connecting to LiDAR on {SERIAL_PORT}...")
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        except serial.SerialException as e:
            print(f"❌ Serial Error: {e}")
            print("Troubleshooting: Check permissions (sudo chmod 666 /dev/ttyUSB0) or connection.")
            sys.exit(1)

        # [FIX] Use Multiprocessing Queue instead of Threading Queue
        self.processing_queue = mp.Queue()

        self.background_matrix = set()
        self.is_stacking = False
        self.current_full_stack = []
        self.current_extracted = []
        self.last_detection_time = time.time()
        self.start_capture_time = None

    def calibrate(self):
        print(f"⌛ Robust Zero Plane Training ({CALIBRATION_FRAMES} frames)...")
        os.makedirs("calibration", exist_ok=True)
        beam_history = {}
        frames = 0

        while frames < CALIBRATION_FRAMES:
            if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
                size_bytes = self.ser.read(2)
                if len(size_bytes) < 2: continue
                size = struct.unpack('<H', size_bytes)[0]
                body = self.ser.read(size)
                if len(body) < size: continue
                self.ser.read(2)  # Skip checksum

                if len(body) > 2 and struct.unpack('<H', body[:2])[0] == 50011:
                    data = body[3:]
                    for i in range(len(data) // 2):
                        d = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                        if d > 0:
                            beam_history.setdefault(i, []).append(d)
                    frames += 1
                    if frames % 500 == 0:
                        print(f"    Frames: {frames}/{CALIBRATION_FRAMES}")

        clean_zero_pts = []
        idxs = sorted(beam_history.keys())
        for j, idx in enumerate(idxs):
            robust_mm = robust_median(beam_history[idx])
            if robust_mm is None: continue

            dist_m = robust_mm / 1000.0
            if not (MIN_RANGE_M <= dist_m <= MAX_RANGE_M): continue

            angle = np.radians(START_ANGLE + idx * ANGULAR_RES)
            x, y = dist_m * np.cos(angle), dist_m * np.sin(angle)
            clean_zero_pts.append((x, y, 0.0))

            ix, iy = int(np.floor(x / GRID_CELL_SIZE)), int(np.floor(y / GRID_CELL_SIZE))
            self.background_matrix.add((ix, iy))

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(clean_zero_pts))
        o3d.io.write_point_cloud("calibration/lzr_zero_plane.pcd", pcd)
        print(f"✅ Robust Zero Plane Saved ({len(clean_zero_pts)} points)")

    def get_raw_frame(self):
        """Reads one full scan from the LiDAR."""
        if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
            size_bytes = self.ser.read(2)
            if len(size_bytes) < 2: return None
            size = struct.unpack('<H', size_bytes)[0]
            body = self.ser.read(size)
            if len(body) < size: return None
            self.ser.read(2)  # Checksum

            if len(body) > 2 and struct.unpack('<H', body[:2])[0] == 50011:
                data = body[3:]
                pts = []
                for i in range(len(data) // 2):
                    d_mm = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                    if 100 < d_mm < 3500:
                        dist = d_mm / 1000.0
                        ang = np.radians(START_ANGLE + i * ANGULAR_RES)
                        pts.append([dist * np.cos(ang), dist * np.sin(ang)])
                return pts
        return None

    def run_forever(self):
        # [FIX] Start Background Worker as a separate Process, not a Thread
        # This allows it to run on a different CPU core and bypass the GIL.
        p = mp.Process(target=background_worker, args=(self.processing_queue,))
        p.daemon = True  # Ensure it dies if main script dies
        p.start()

        self.calibrate()
        print("🚀 System Live - Waiting for vehicles...")

        while True:
            raw_xy = self.get_raw_frame()
            if not raw_xy: continue

            t = time.time()
            z = (t - self.start_capture_time) * VEHICLE_SPEED_MPS if self.is_stacking else 0
            raw_xyz = [[p[0], p[1], z] for p in raw_xy]

            extracted = []
            for p in raw_xy:
                ix, iy = int(np.floor(p[0] / GRID_CELL_SIZE)), int(np.floor(p[1] / GRID_CELL_SIZE))
                if (ix, iy) not in self.background_matrix:
                    extracted.append([p[0], p[1], z])

            # Trigger Logic
            if len(extracted) > TRIGGER_THRESHOLD:
                if not self.is_stacking:
                    self.is_stacking = True
                    self.start_capture_time = time.time()
                    self.current_full_stack = []
                    self.current_extracted = []
                    print("🚗 Vehicle Detected")

                self.current_full_stack.extend(raw_xyz)
                self.current_extracted.extend(extracted)
                self.last_detection_time = time.time()

            elif self.is_stacking and (time.time() - self.last_detection_time > IDLE_TIMEOUT):
                # Vehicle has passed
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                print(f"🏁 Vehicle {ts} queued for processing ({len(self.current_extracted)} points)")

                # [FIX] Send data to the separate process via Queue
                self.processing_queue.put((list(self.current_extracted), ts))

                self.is_stacking = False


if __name__ == "__main__":
    # Required for Multiprocessing support on some platforms
    mp.set_start_method('spawn', force=True)  # Safer for OpenCV/Open3D compatibility

    system = TollPlazaSystem()
    system.run_forever()
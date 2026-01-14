import serial
import struct
import numpy as np
import time
import cv2
import open3d as o3d
import threading
import queue
import os
from datetime import datetime
import statistics

# =====================================================
# CONFIGURATION
# =====================================================
SERIAL_PORT = '/dev/ttyUSB0'  # Change to '/dev/ttyUSB0' or '/dev/ttyACM0' for RPi
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

# [NEW] 3D Rotation Correction (Degrees)
# Adjust these values to compensate for physical mounting angles
ROTATION_X_DEG = 25.0  # Pitch (Tilt forward/backward)
ROTATION_Y_DEG = 0.0  # Roll  (Tilt left/right)
ROTATION_Z_DEG = 0.0  # Yaw   (Spin rotation)

# [NEW] Dynamic Frame Settings
FRAME_PADDING_M = 0.2  # Add 20cm empty space around the vehicle crop
MIN_IMAGE_DIM_M = 1.0  # Minimum image size (meters) to prevent errors on noise

processing_queue = queue.Queue()


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
# BACKGROUND PROCESSOR (3D ROTATION + DYNAMIC SIZE)
# =====================================================
def background_processor():
    print("🧵 Background Processor Started")
    while True:
        try:
            extracted_stack, ts = processing_queue.get()

            # Safety Check: Ensure enough points exist for statistical analysis
            if len(extracted_stack) < 30:
                print(f"⚠️ Vehicle {ts} Skipped: Too few points ({len(extracted_stack)})")
                processing_queue.task_done()
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
                processing_queue.task_done()
                continue

            pcd, _ = pcd.remove_radius_outlier(nb_points=12, radius=0.06)
            if len(pcd.points) == 0:
                print(f"⚠️ Vehicle {ts} Skipped: Empty after radius cleanup")
                processing_queue.task_done()
                continue

            # ---------------------------------------------------------
            # 2. Standard Side View Transformation
            # ---------------------------------------------------------
            # Map raw Lidar data to Side View:
            # Old Z (Time) -> New X (Length)
            # Old Y (Height) -> New Y (Height)
            # -Old X -> New Z (Depth)
            R_standard = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=np.float64)
            pts_side = (R_standard @ np.asarray(pcd.points).T).T

            # ---------------------------------------------------------
            # [NEW] 2.1 Apply Full 3D Mounting Rotation Correction
            # ---------------------------------------------------------
            # We construct a rotation matrix R_total = Rz * Ry * Rx
            rad_x = np.radians(ROTATION_X_DEG)
            rad_y = np.radians(ROTATION_Y_DEG)
            rad_z = np.radians(ROTATION_Z_DEG)

            # Pitch (X)
            Rx = np.array([
                [1, 0, 0],
                [0, np.cos(rad_x), -np.sin(rad_x)],
                [0, np.sin(rad_x), np.cos(rad_x)]
            ])

            # Roll (Y)
            Ry = np.array([
                [np.cos(rad_y), 0, np.sin(rad_y)],
                [0, 1, 0],
                [-np.sin(rad_y), 0, np.cos(rad_y)]
            ])

            # Yaw (Z)
            Rz = np.array([
                [np.cos(rad_z), -np.sin(rad_z), 0],
                [np.sin(rad_z), np.cos(rad_z), 0],
                [0, 0, 1]
            ])

            # Combined Rotation
            R_total = Rz @ Ry @ Rx

            # Apply rotation to all points
            pts_side = (R_total @ pts_side.T).T

            # Save Side View PCD (Rotated)
            pcd_side = o3d.geometry.PointCloud()
            pcd_side.points = o3d.utility.Vector3dVector(pts_side)
            o3d.io.write_point_cloud(f"{folder}/side_view.pcd", pcd_side)

            # ---------------------------------------------------------
            # [NEW] 3. Dynamic Frame Calculation
            # ---------------------------------------------------------
            # Find the min/max boundaries of the actual vehicle points
            min_x, min_y = np.min(pts_side[:, :2], axis=0)
            max_x, max_y = np.max(pts_side[:, :2], axis=0)

            # Apply Padding
            min_x -= FRAME_PADDING_M
            max_x += FRAME_PADDING_M
            min_y -= FRAME_PADDING_M
            max_y += FRAME_PADDING_M

            # Ensure minimum frame size (prevents crash on tiny artifacts)
            if (max_x - min_x) < MIN_IMAGE_DIM_M: max_x = min_x + MIN_IMAGE_DIM_M
            if (max_y - min_y) < MIN_IMAGE_DIM_M: max_y = min_y + MIN_IMAGE_DIM_M

            # Calculate dynamic image resolution (pixels)
            width_m = max_x - min_x
            height_m = max_y - min_y

            x_bins = int(width_m / GRID_RES)
            y_bins = int(height_m / GRID_RES)

            # ---------------------------------------------------------
            # 4. Image Generation
            # ---------------------------------------------------------
            bev = np.zeros((y_bins, x_bins), dtype=np.float32)

            # Vectorized projection relative to the dynamic origin (min_x, min_y)
            xi = ((pts_side[:, 0] - min_x) / GRID_RES).astype(np.int32)
            yi = ((pts_side[:, 1] - min_y) / GRID_RES).astype(np.int32)

            # Filter valid indices within the new dynamic grid
            valid_indices = (xi >= 0) & (xi < x_bins) & (yi >= 0) & (yi < y_bins)
            xi = xi[valid_indices]
            yi = yi[valid_indices]
            v_points = pts_side[valid_indices]

            if len(v_points) > 0:
                # Intensity based on distance (depth)
                dist = np.sqrt(v_points[:, 0] ** 2 + v_points[:, 1] ** 2)
                intensity = 1.0 - np.minimum(dist / MAX_DIST_INTENSITY, 1.0)

                # Assign to grid (Last point writes wins)
                bev[yi, xi] = intensity

                # Final Image Processing
                bev_img = (bev * 255).astype(np.uint8)
                bev_img = np.flipud(bev_img)  # Flip Y so ground is at bottom
                bev_img = cv2.GaussianBlur(bev_img, (3, 3), 0)

                img_path = f"{folder}/side_view_image.png"
                cv2.imwrite(img_path, bev_img)
                print(f"✅ Vehicle {ts} Processed: {x_bins}x{y_bins} px -> {img_path}")
            else:
                print(f"⚠️ Vehicle {ts} Skipped: No points in generated frame")

            processing_queue.task_done()

        except Exception as e:
            print(f"❌ Error in background processor: {e}")
            import traceback
            traceback.print_exc()
            processing_queue.task_done()


# =====================================================
# MAIN SYSTEM
# =====================================================
class TollPlazaSystem:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        except serial.SerialException as e:
            print(f"❌ Serial Error: {e}")
            print("Check connection and permissions (sudo chmod 666 /dev/ttyUSB0)")
            exit(1)

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
            # Read header
            if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
                size_bytes = self.ser.read(2)
                if len(size_bytes) < 2: continue
                size = struct.unpack('<H', size_bytes)[0]

                body = self.ser.read(size)
                if len(body) < size: continue

                # Checksum/Footer skip
                self.ser.read(2)

                # Verify Message ID (50011 for LZR-U921 specific output)
                if len(body) > 2 and struct.unpack('<H', body[:2])[0] == 50011:
                    data = body[3:]
                    for i in range(len(data) // 2):
                        d = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                        if d > 0:
                            beam_history.setdefault(i, []).append(d)
                    frames += 1
                    if frames % 500 == 0:
                        print(f"    Frames: {frames}/{CALIBRATION_FRAMES}")

        # Process Calibration Data
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

            # Add to background set
            ix, iy = int(np.floor(x / GRID_CELL_SIZE)), int(np.floor(y / GRID_CELL_SIZE))
            self.background_matrix.add((ix, iy))

        # Save Calibration
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
                    # Filter valid range immediately
                    if 100 < d_mm < 3500:
                        dist = d_mm / 1000.0
                        ang = np.radians(START_ANGLE + i * ANGULAR_RES)
                        pts.append([dist * np.cos(ang), dist * np.sin(ang)])
                return pts
        return None

    def run_forever(self):
        # Start the background processor in a separate thread
        t = threading.Thread(target=background_processor, daemon=True)
        t.start()

        self.calibrate()
        print("🚀 System Live - Waiting for vehicles...")

        while True:
            raw_xy = self.get_raw_frame()
            if not raw_xy: continue

            t = time.time()
            # Calculate Z (Time dimension) based on estimated speed
            z = (t - self.start_capture_time) * VEHICLE_SPEED_MPS if self.is_stacking else 0
            raw_xyz = [[p[0], p[1], z] for p in raw_xy]

            # Background Subtraction
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

                # Send deep copy to queue
                processing_queue.put((list(self.current_extracted), ts))

                self.is_stacking = False


if __name__ == "__main__":
    system = TollPlazaSystem()
    system.run_forever()

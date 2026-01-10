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
import sys

# =====================================================
# CONFIGURATION
# =====================================================
# Hardware
SERIAL_PORT = 'COM9'  # Adjust for your Pi
BAUD_RATE = 921600

# Sensor Geometry
START_ANGLE = -48.0
ANGULAR_RES = 0.3516

# [FIX 1] Increased Minimum Range to ignore dust on lens
MIN_RANGE_M = 0.15
MAX_RANGE_M = 2.25  # Extended slightly for road width

# Calibration
CALIBRATION_FRAMES = 2000
GRID_CELL_SIZE = 0.05
MAX_NEIGHBOR_JUMP = 0.15

# Physics (Speed)
# [FIX 2] Updated to 25 km/h to match road conditions
VEHICLE_SPEED_KMPH = 5
VEHICLE_SPEED_MPS = VEHICLE_SPEED_KMPH / 3.6

# Detection Logic
# [FIX 3] Stricter thresholds to stop ghost detections
TRIGGER_THRESHOLD = 40  # Ignores small noise clusters
REQUIRED_PERSISTENCE = 5  # Object must be seen in 5 frames to confirm
IDLE_TIMEOUT = 0.5  # Faster cutoff for road traffic

# Image Generation Parameters
X_IMG_RANGE = (-1, 1)
Y_IMG_RANGE = (-1, 1)
GRID_RES = 0.005  # 5mm per pixel
MAX_DIST_INTENSITY = 50.0

# Global Queues
# frame_queue: Raw data from Serial Thread -> Main Loop
# processing_queue: Extracted Object -> Image Generator Thread
frame_queue = queue.Queue()
processing_queue = queue.Queue()


# =====================================================
# HELPER: ROBUST MEDIAN (MAD)
# =====================================================
def robust_median(distances):
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
# WORKER: BACKGROUND IMAGE PROCESSOR
# =====================================================
def background_processor():
    print("🧵 Background Processor Started")
    while True:
        try:
            extracted_stack, ts = processing_queue.get()

            # Safety Check: Too few points crash Open3D
            if len(extracted_stack) < 30:
                print(f"⚠️ Vehicle {ts} Skipped: Too few points ({len(extracted_stack)})")
                processing_queue.task_done()
                continue

            folder = f"vehicle_{ts}"
            os.makedirs(folder, exist_ok=True)

            # 1. Point Cloud Cleaning
            pcd = o3d.geometry.PointCloud()
            # Force float64 for RPi stability
            pcd.points = o3d.utility.Vector3dVector(np.array(extracted_stack, dtype=np.float64))

            # Statistical Removal (Snow/Dust)
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=25, std_ratio=0.8)
            if len(pcd.points) == 0:
                processing_queue.task_done();
                continue

            # Radius Removal (Flying Pixels)
            pcd, _ = pcd.remove_radius_outlier(nb_points=12, radius=0.06)
            if len(pcd.points) == 0:
                processing_queue.task_done();
                continue

            # 2. Save Side View PCD
            # Rotate: Look from side
            R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=np.float64)
            pts_side = (R @ np.asarray(pcd.points).T).T

            pcd_side = o3d.geometry.PointCloud()
            pcd_side.points = o3d.utility.Vector3dVector(pts_side)
            o3d.io.write_point_cloud(f"{folder}/side_view.pcd", pcd_side)

            # 3. Generate Image
            mask = (
                    (pts_side[:, 0] >= X_IMG_RANGE[0]) & (pts_side[:, 0] <= X_IMG_RANGE[1]) &
                    (pts_side[:, 1] >= Y_IMG_RANGE[0]) & (pts_side[:, 1] <= Y_IMG_RANGE[1])
            )
            img_points = pts_side[mask]

            if len(img_points) > 0:
                x_bins = int((X_IMG_RANGE[1] - X_IMG_RANGE[0]) / GRID_RES)
                y_bins = int((Y_IMG_RANGE[1] - Y_IMG_RANGE[0]) / GRID_RES)
                bev = np.zeros((y_bins, x_bins), dtype=np.float32)

                # Map points to grid
                xi = ((img_points[:, 0] - X_IMG_RANGE[0]) / GRID_RES).astype(np.int32)
                yi = ((img_points[:, 1] - Y_IMG_RANGE[0]) / GRID_RES).astype(np.int32)

                valid = (xi >= 0) & (xi < x_bins) & (yi >= 0) & (yi < y_bins)
                xi, yi = xi[valid], yi[valid]

                # Intensity Mapping (Binary occupancy for clearer shape)
                bev[yi, xi] = 1.0

                # Convert to Image
                bev_img = (bev * 255).astype(np.uint8)
                bev_img = np.flipud(bev_img)

                # [REVERTED] Dilation Removed.
                # Only standard smoothing is applied.
                bev_img = cv2.GaussianBlur(bev_img, (3, 3), 0)

                cv2.imwrite(f"{folder}/side_view_image.png", bev_img)

            print(f"✅ Vehicle {ts} Saved")
            processing_queue.task_done()

        except Exception as e:
            print(f"❌ Error in Image Proc: {e}")
            processing_queue.task_done()


# =====================================================
# MAIN CLASS (THREADED READER)
# =====================================================
class TollPlazaSystem:
    def __init__(self):
        # Setup Serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.ser.reset_input_buffer()
        except Exception as e:
            print(f"❌ Serial Port Error: {e}")
            sys.exit(1)

        self.background_matrix = set()
        self.running = True

        # Detection State
        self.is_stacking = False
        self.current_full_stack = []
        self.current_extracted = []
        self.last_detection_time = time.time()
        self.start_capture_time = None

        # [FIX 4] Persistence Counters
        self.consecutive_triggers = 0
        self.pre_trigger_buffer = []

        # -------------------------------------------------

    # THREAD 1: SERIAL READER (High Speed)
    # -------------------------------------------------
    def serial_worker(self):
        print("⚡ Serial Worker Thread Live")
        internal_buffer = bytearray()

        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    # Read large chunks (up to 4KB) to avoid CPU bottleneck
                    chunk = self.ser.read(min(self.ser.in_waiting, 4096))
                    internal_buffer.extend(chunk)
                else:
                    time.sleep(0.001)
                    continue

                # Parse Buffer for Packets
                while len(internal_buffer) > 8:
                    header_idx = internal_buffer.find(b'\xfc\xfd\xfe\xff')

                    if header_idx == -1:
                        internal_buffer = internal_buffer[-3:]
                        break

                    if header_idx > 0:
                        internal_buffer = internal_buffer[header_idx:]

                    if len(internal_buffer) < 6: break

                    size = struct.unpack('<H', internal_buffer[4:6])[0]
                    total_len = 4 + 2 + size + 2

                    if len(internal_buffer) < total_len: break

                    packet = internal_buffer[:total_len]
                    internal_buffer = internal_buffer[total_len:]

                    # Extract Payload
                    body = packet[6:-2]
                    if len(body) >= 2 and struct.unpack('<H', body[:2])[0] == 50011:
                        # Push raw body to main loop queue
                        frame_queue.put(body[3:])

            except Exception as e:
                print(f"❌ Serial Error: {e}")
                time.sleep(1)

    # -------------------------------------------------
    # HELPER: PARSE BYTES
    # -------------------------------------------------
    def parse_raw_bytes_to_xy(self, data):
        pts = []
        for i in range(len(data) // 2):
            d_mm = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
            if 100 < d_mm < 3500:  # Filter 10cm to 3.5m
                dist = d_mm / 1000.0
                ang = np.radians(START_ANGLE + i * ANGULAR_RES)
                pts.append([dist * np.cos(ang), dist * np.sin(ang)])
        return pts

    # -------------------------------------------------
    # CALIBRATION
    # -------------------------------------------------
    def calibrate(self):
        print(f"⌛ Robust Calibration ({CALIBRATION_FRAMES} frames)...")
        # Flush old data
        while not frame_queue.empty(): frame_queue.get()

        frames = 0
        beam_history = {}

        while frames < CALIBRATION_FRAMES:
            try:
                # Read from Queue (filled by thread)
                raw_data = frame_queue.get(timeout=1.0)
                data = raw_data
                for i in range(len(data) // 2):
                    d = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                    if d > 0:
                        beam_history.setdefault(i, []).append(d)
                frames += 1
                if frames % 500 == 0: print(f"    {frames}/{CALIBRATION_FRAMES}")
            except queue.Empty:
                continue

        print("    Computing Median & Dilating...")
        idxs = sorted(beam_history.keys())
        for idx in idxs:
            robust_mm = robust_median(beam_history[idx])
            if robust_mm is None: continue
            dist_m = robust_mm / 1000.0

            if not (MIN_RANGE_M <= dist_m <= MAX_RANGE_M): continue

            angle = np.radians(START_ANGLE + idx * ANGULAR_RES)
            x, y = dist_m * np.cos(angle), dist_m * np.sin(angle)

            ix, iy = int(np.floor(x / GRID_CELL_SIZE)), int(np.floor(y / GRID_CELL_SIZE))

            # [FIX 5] Background Dilation
            # Prevents "Edge Jitter" false alarms
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    self.background_matrix.add((ix + dx, iy + dy))

        print(f"✅ Calibration Done. Background Cells: {len(self.background_matrix)}")

    # -------------------------------------------------
    # MAIN LOOP
    # -------------------------------------------------
    def run_forever(self):
        # Start Threads
        threading.Thread(target=background_processor, daemon=True).start()
        threading.Thread(target=self.serial_worker, daemon=True).start()

        time.sleep(1.0)
        self.calibrate()
        print("🚀 System Live (Persistence Mode Enabled)")

        while True:
            try:
                # Get latest frame from queue
                raw_data = frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            # Parse
            raw_xy = self.parse_raw_bytes_to_xy(raw_data)

            # Z Calculation
            t = time.time()
            if self.is_stacking:
                z = (t - self.start_capture_time) * VEHICLE_SPEED_MPS
            else:
                z = 0

            raw_xyz = [[p[0], p[1], z] for p in raw_xy]

            # Background Subtraction
            extracted = []
            for p in raw_xy:
                ix, iy = int(np.floor(p[0] / GRID_CELL_SIZE)), int(np.floor(p[1] / GRID_CELL_SIZE))

                # Robust Background Check
                is_background = False
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if (ix + dx, iy + dy) in self.background_matrix:
                            is_background = True
                            break
                    if is_background: break

                if not is_background:
                    extracted.append([p[0], p[1], z])

            # =========================================
            # [FIX 6] PERSISTENCE CHECK LOGIC
            # =========================================
            if len(extracted) > TRIGGER_THRESHOLD:
                # Potential Object Detected
                self.consecutive_triggers += 1

                # Buffer this frame so we don't lose the front of the car
                self.pre_trigger_buffer.append((raw_xyz, extracted))
                if len(self.pre_trigger_buffer) > 10:
                    self.pre_trigger_buffer.pop(0)

                # Only START recording if we see it for N consecutive frames
                if self.consecutive_triggers >= REQUIRED_PERSISTENCE:
                    if not self.is_stacking:
                        print("🚗 Vehicle Confirmed (Persistence Met)")
                        self.is_stacking = True
                        # Backdate start time to include the buffer
                        self.start_capture_time = time.time() - (0.02 * len(self.pre_trigger_buffer))
                        self.current_full_stack = []
                        self.current_extracted = []

                        # Add buffered frames
                        for b_xyz, b_ext in self.pre_trigger_buffer:
                            self.current_full_stack.extend(b_xyz)
                            self.current_extracted.extend(b_ext)
                        self.pre_trigger_buffer = []

                    # Standard Recording
                    self.current_full_stack.extend(raw_xyz)
                    self.current_extracted.extend(extracted)
                    self.last_detection_time = time.time()

            else:
                # No object seen in this frame
                self.consecutive_triggers = 0
                self.pre_trigger_buffer = []

                # If recording, check for timeout (End of Vehicle)
                if self.is_stacking and (time.time() - self.last_detection_time > IDLE_TIMEOUT):
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    processing_queue.put((list(self.current_extracted), ts))
                    self.is_stacking = False
                    print(f"🏁 Vehicle {ts} Finished")


if __name__ == "__main__":
    sys = TollPlazaSystem()
    try:
        sys.run_forever()
    except KeyboardInterrupt:
        sys.running = False
import serial
import struct
import numpy as np
import time
import cv2
import open3d as o3d
import threading
import queue
from datetime import datetime
from collections import Counter

# =====================================================
# CONFIGURATION
# =====================================================
SERIAL_PORT = 'COM8'
BAUD_RATE = 921600
START_ANGLE = -48.0
ANGULAR_RES = 0.3516

# --- CALIBRATION & ACCURACY ---
CALIBRATION_FRAMES = 100  # Set how many frames to "train" the zero plane
MIN_CONFIDENCE_RATIO = 0.9  # Point must appear in 80% of frames to be background
GRID_CELL_SIZE = 0.05  # Smaller cell size (5cm) for higher extraction accuracy

# --- VEHICLE STACKING ---
VEHICLE_SPEED_KMPH = 0.5
VEHICLE_SPEED_MPS = VEHICLE_SPEED_KMPH / 3.6  #
TRIGGER_THRESHOLD = 20  # Minimum points to consider it a vehicle
IDLE_TIMEOUT = 0.8  # Seconds of silence to end a vehicle profile
GRID_RES = 0.005  # 5mm per pixel for BEV image

processing_queue = queue.Queue()


class TollPlazaSystem:
    def __init__(self):
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        self.background_matrix = set()
        self.is_stacking = False
        self.current_stack = []
        self.last_detection_time = time.time()
        self.start_capture_time = None

    def calibrate(self):
        """Trains the background using a fixed number of frames for max accuracy."""
        print(f"⌛ Training Zero Plane using {CALIBRATION_FRAMES} frames... Do not move.")

        # Frequency map to track how often a grid cell is occupied
        cell_frequency = Counter()
        frames_captured = 0

        while frames_captured < CALIBRATION_FRAMES:
            raw_points = self.get_raw_frame()
            if raw_points:
                unique_cells_in_frame = set()
                for p in raw_points:
                    ix = int(np.floor(p[0] / GRID_CELL_SIZE))
                    iy = int(np.floor(p[1] / GRID_CELL_SIZE))
                    unique_cells_in_frame.add((ix, iy))

                for cell in unique_cells_in_frame:
                    cell_frequency[cell] += 1

                frames_captured += 1
                if frames_captured % 10 == 0:
                    print(f"   Progress: {frames_captured}/{CALIBRATION_FRAMES}")

        # Only add cells that were consistently present
        for cell, count in cell_frequency.items():
            if count >= (CALIBRATION_FRAMES * MIN_CONFIDENCE_RATIO):
                self.background_matrix.add(cell)

        print(f"✅ Calibration complete. Background Matrix size: {len(self.background_matrix)}")

    def get_raw_frame(self):
        """Reads and parses one full scan from the LZR-U921."""
        # Look for SYNC_HEADER b'\xfc\xfd\xfe\xff'
        if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
            size = struct.unpack('<H', self.ser.read(2))[0]
            body = self.ser.read(size)
            self.ser.read(2)

            if struct.unpack('<H', body[:2])[0] == 50011:
                data = body[3:]
                points = []
                for i in range(len(data) // 2):
                    d_mm = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                    # Range filter to remove ground/ceiling noise
                    if 100 < d_mm < 3500:
                        dist = d_mm / 1000.0
                        rad = np.radians(START_ANGLE + i * ANGULAR_RES)
                        points.append([dist * np.cos(rad), dist * np.sin(rad)])
                return points
        return None

    def run_forever(self):
        threading.Thread(target=background_processor, daemon=True).start()
        self.calibrate()
        print("🚀 System Live. Monitoring for vehicles...")

        while True:
            raw_points = self.get_raw_frame()
            if not raw_points: continue

            current_time = time.time()
            obj_points = []

            # Calculate Z based on time elapsed since vehicle entrance
            z_val = (current_time - self.start_capture_time) * VEHICLE_SPEED_MPS if self.is_stacking else 0

            for p in raw_points:
                ix = int(np.floor(p[0] / GRID_CELL_SIZE))
                iy = int(np.floor(p[1] / GRID_CELL_SIZE))

                # The core extraction logic
                if (ix, iy) not in self.background_matrix:
                    obj_points.append([p[0], p[1], z_val])

            # Logic to trigger stacking
            if len(obj_points) > TRIGGER_THRESHOLD:
                if not self.is_stacking:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] 🚗 Vehicle Detected")
                    self.is_stacking = True
                    self.current_stack = []
                    self.start_capture_time = time.time()

                self.current_stack.extend(obj_points)
                self.last_detection_time = time.time()

            elif self.is_stacking and (time.time() - self.last_detection_time > IDLE_TIMEOUT):
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                # Offload to background thread
                processing_queue.put((self.current_stack, ts))
                self.is_stacking = False
                print(f"🏁 Vehicle processing queued. Ready for next.")


# =====================================================
# BACKGROUND PROCESSING THREAD
# =====================================================
def background_processor():
    while True:
        data_packet = processing_queue.get()
        if data_packet is None: break

        points_list, ts = data_packet
        if len(points_list) < 50: continue

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points_list))

        # 1. Statistical Outlier Removal
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=25, std_ratio=0.8)

        # 2. Radius Outlier Removal
        pcd, _ = pcd.remove_radius_outlier(nb_points=12, radius=0.06)

        # 3. Save Cleaned PCD
        o3d.io.write_point_cloud(f"veh_{ts}.pcd", pcd)

        # 4. Save BEV Image
        pts = np.asarray(pcd.points)
        if len(pts) > 0:
            # Automatic image sizing based on actual vehicle length
            x_min, x_max = np.min(pts[:, 0]), np.max(pts[:, 0])
            z_max = np.max(pts[:, 2])

            w = int((x_max - x_min + 0.2) / GRID_RES)
            h = int((z_max + 0.2) / GRID_RES)

            bev = np.zeros((h, w), dtype=np.uint8)
            for x, y, z in pts:
                xi = int((x - x_min + 0.1) / GRID_RES)
                zi = int(z / GRID_RES)
                if 0 <= xi < w and 0 <= zi < h:
                    bev[zi, xi] = 255
            cv2.imwrite(f"veh_{ts}_BEV.png", np.flipud(bev))

        processing_queue.task_done()


if __name__ == "__main__":
    TollPlazaSystem().run_forever()
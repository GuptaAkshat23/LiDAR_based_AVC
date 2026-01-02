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
# CONFIGURATION (Replicated from your scripts)
# =====================================================
SERIAL_PORT = 'COM8'
BAUD_RATE = 921600
START_ANGLE = -48.0
ANGULAR_RES = 0.3516

# Calibration Settings
CALIBRATION_FRAMES = 100
MIN_CONFIDENCE_RATIO = 0.8
GRID_CELL_SIZE = 0.10  # Replicated from extract_profile.py

# Speed & ROI Settings (Replicated from image_generator.py)
VEHICLE_SPEED_KMPH = 2
VEHICLE_SPEED_MPS = VEHICLE_SPEED_KMPH / 3.6
X_RANGE = (-0, 2)  # Exact range from image_generator.py
Y_RANGE = (-0.5, 2)  # Exact range from image_generator.py
GRID_RES = 0.005  # Exact res from image_generator.py
MAX_DIST = 50.0  # Exact max_dist from image_generator.py

TRIGGER_THRESHOLD = 20
IDLE_TIMEOUT = 0.8

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
        """Trains background matrix using frequency logic for accuracy."""
        print(f"⌛ Calibrating Zero Plane ({CALIBRATION_FRAMES} frames)...")
        cell_frequency = Counter()
        frames_captured = 0
        while frames_captured < CALIBRATION_FRAMES:
            raw_points = self.get_raw_frame()
            if raw_points:
                unique_cells = set(
                    (int(np.floor(p[0] / GRID_CELL_SIZE)), int(np.floor(p[1] / GRID_CELL_SIZE))) for p in raw_points)
                for cell in unique_cells: cell_frequency[cell] += 1
                frames_captured += 1
        for cell, count in cell_frequency.items():
            if count >= (CALIBRATION_FRAMES * MIN_CONFIDENCE_RATIO):
                self.background_matrix.add(cell)
        print(f"✅ Background Matrix Ready: {len(self.background_matrix)} cells.")

    def get_raw_frame(self):
        """Standard LZR-U921 parsing."""
        if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
            size = struct.unpack('<H', self.ser.read(2))[0]
            body = self.ser.read(size)
            self.ser.read(2)
            if struct.unpack('<H', body[:2])[0] == 50011:
                data = body[3:]
                points = []
                for i in range(len(data) // 2):
                    d_mm = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                    if 100 < d_mm < 3500:  # Standard range filtering
                        dist = d_mm / 1000.0
                        rad = np.radians(START_ANGLE + i * ANGULAR_RES)
                        points.append([dist * np.cos(rad), dist * np.sin(rad)])
                return points
        return None

    def run_forever(self):
        threading.Thread(target=background_processor, daemon=True).start()
        self.calibrate()
        while True:
            raw_points = self.get_raw_frame()
            if not raw_points: continue
            current_time = time.time()
            obj_points = []

            # Use Speed-Corrected Z calculation
            z_val = (current_time - self.start_capture_time) * VEHICLE_SPEED_MPS if self.is_stacking else 0

            for p in raw_points:
                ix, iy = int(np.floor(p[0] / GRID_CELL_SIZE)), int(np.floor(p[1] / GRID_CELL_SIZE))
                if (ix, iy) not in self.background_matrix:
                    obj_points.append([p[0], p[1], z_val])

            if len(obj_points) > TRIGGER_THRESHOLD:
                if not self.is_stacking:
                    self.is_stacking = True
                    self.current_stack = []
                    self.start_capture_time = time.time()
                self.current_stack.extend(obj_points)
                self.last_detection_time = time.time()
            elif self.is_stacking and (time.time() - self.last_detection_time > IDLE_TIMEOUT):
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                processing_queue.put((self.current_stack, ts))
                self.is_stacking = False


# =====================================================
# REPLICATED PROCESSING LOGIC
# =====================================================
def background_processor():
    while True:
        data_packet = processing_queue.get()
        if data_packet is None: break
        points_list, ts = data_packet

        # 1. Convert to Open3D
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points_list))

        # 2. EXACT Outlier Removal Steps
        # Statistical Outlier Removal (SOR)
        pcd_clean, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
        pcd_clean = pcd.select_by_index(ind)
        # Radius Outlier Removal (ROR)
        pcd_final, ind = pcd_clean.remove_radius_outlier(nb_points=16, radius=0.05)
        pcd_final = pcd_clean.select_by_index(ind)

        # 3. Save PCD
        filename_pcd = f"veh_{ts}.pcd"
        o3d.io.write_point_cloud(filename_pcd, pcd_final)

        # 4. EXACT Image Generation Steps
        points = np.asarray(pcd_final.points)
        if len(points) > 0:
            # ROI Filter replicated from image_generator.py
            mask = (
                    (points[:, 0] >= X_RANGE[0]) & (points[:, 0] <= X_RANGE[1]) &
                    (points[:, 1] >= Y_RANGE[0]) & (points[:, 1] <= Y_RANGE[1])
            )
            points = points[mask]

            # Bins calculation
            x_bins = int((X_RANGE[1] - X_RANGE[0]) / GRID_RES)
            y_bins = int((Y_RANGE[1] - Y_RANGE[0]) / GRID_RES)
            bev = np.zeros((y_bins, x_bins), dtype=np.float32)

            # Intensity Projection
            for x, y, z in points:
                xi = int((x - X_RANGE[0]) / GRID_RES)
                yi = int((y - Y_RANGE[0]) / GRID_RES)
                if 0 <= xi < x_bins and 0 <= yi < y_bins:
                    dist = np.sqrt(x ** 2 + y ** 2)
                    intensity = 1.0 - min(dist / MAX_DIST, 1.0)
                    if intensity > bev[yi, xi]: bev[yi, xi] = intensity

            # Processing and Blurring
            bev = (bev * 255).astype(np.uint8)
            bev = np.flipud(bev)
            bev = cv2.GaussianBlur(bev, (3, 3), 0)

            cv2.imwrite(f"veh_{ts}_BEV.png", bev)
            print(f"✅ Processed: {filename_pcd} and Image.")

        processing_queue.task_done()


if __name__ == "__main__":
    TollPlazaSystem().run_forever()
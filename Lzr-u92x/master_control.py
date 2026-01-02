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
from collections import Counter

# =====================================================
# CONFIGURATION
# =====================================================
SERIAL_PORT = 'COM8'
BAUD_RATE = 921600
START_ANGLE = -48.0
ANGULAR_RES = 0.3516

# --- CALIBRATION & ACCURACY ---
CALIBRATION_FRAMES = 3000
MIN_CONFIDENCE_RATIO = 0.9
GRID_CELL_SIZE = 0.05

# --- VEHICLE STACKING ---
VEHICLE_SPEED_KMPH = 0.5
VEHICLE_SPEED_MPS = VEHICLE_SPEED_KMPH / 3.6
TRIGGER_THRESHOLD = 20
IDLE_TIMEOUT = 0.8
GRID_RES = 0.005

processing_queue = queue.Queue()


class TollPlazaSystem:
    def __init__(self):
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        self.background_matrix = set()
        self.background_points_raw = []  # For saving zero plane PCD
        self.is_stacking = False
        self.current_full_stack = []  # Includes EVERYTHING (for raw save)
        self.last_detection_time = time.time()
        self.start_capture_time = None

    def calibrate(self):
        """Trains the background and saves the Zero Plane file."""
        print(f"⌛ Training Zero Plane using {CALIBRATION_FRAMES} frames...")
        if not os.path.exists("calibration"): os.makedirs("calibration")

        cell_frequency = Counter()
        cell_coords = {}
        frames_captured = 0

        while frames_captured < CALIBRATION_FRAMES:
            raw_points = self.get_raw_frame()
            if raw_points:
                unique_cells_in_frame = set()
                for p in raw_points:
                    ix, iy = int(np.floor(p[0] / GRID_CELL_SIZE)), int(np.floor(p[1] / GRID_CELL_SIZE))
                    unique_cells_in_frame.add((ix, iy))
                    cell_coords[(ix, iy)] = [p[0], p[1], 0.0]

                for cell in unique_cells_in_frame:
                    cell_frequency[cell] += 1
                frames_captured += 1
                if frames_captured % 100 == 0:
                    print(f"   Progress: {frames_captured}/{CALIBRATION_FRAMES}")

        final_bg_pts = []
        for cell, count in cell_frequency.items():
            if count >= (CALIBRATION_FRAMES * MIN_CONFIDENCE_RATIO):
                self.background_matrix.add(cell)
                final_bg_pts.append(cell_coords[cell])

        # SAVE ZERO PLANE
        bg_pcd = o3d.geometry.PointCloud()
        bg_pcd.points = o3d.utility.Vector3dVector(np.array(final_bg_pts))
        o3d.io.write_point_cloud("calibration/lzr_zero_plane.pcd", bg_pcd)
        print(f"✅ Calibration complete. Saved to 'calibration/lzr_zero_plane.pcd'")

    def get_raw_frame(self):
        if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
            size = struct.unpack('<H', self.ser.read(2))[0]
            body = self.ser.read(size)
            self.ser.read(2)
            if struct.unpack('<H', body[:2])[0] == 50011:
                data, points = body[3:], []
                for i in range(len(data) // 2):
                    d_mm = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                    if 100 < d_mm < 3500:
                        dist = d_mm / 1000.0
                        rad = np.radians(START_ANGLE + i * ANGULAR_RES)
                        points.append([dist * np.cos(rad), dist * np.sin(rad)])
                return points
        return None

    def run_forever(self):
        threading.Thread(target=background_processor, daemon=True).start()
        self.calibrate()
        print("🚀 System Live. Monitoring...")

        while True:
            raw_scan_xy = self.get_raw_frame()
            if not raw_scan_xy: continue

            current_time = time.time()
            extracted_points = []
            z_val = (current_time - self.start_capture_time) * VEHICLE_SPEED_MPS if self.is_stacking else 0

            # Temporary list for the raw stacked profile (including background)
            raw_scan_xyz = [[p[0], p[1], z_val] for p in raw_scan_xy]

            for p in raw_scan_xy:
                ix, iy = int(np.floor(p[0] / GRID_CELL_SIZE)), int(np.floor(p[1] / GRID_CELL_SIZE))
                if (ix, iy) not in self.background_matrix:
                    extracted_points.append([p[0], p[1], z_val])

            if len(extracted_points) > TRIGGER_THRESHOLD:
                if not self.is_stacking:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] 🚗 Vehicle Detected")
                    self.is_stacking, self.current_full_stack, self.current_extracted = True, [], []
                    self.start_capture_time = time.time()

                self.current_full_stack.extend(raw_scan_xyz)
                self.current_extracted.extend(extracted_points)
                self.last_detection_time = time.time()

            elif self.is_stacking and (time.time() - self.last_detection_time > IDLE_TIMEOUT):
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                # Send both full stack and extracted stack to processor
                processing_queue.put((self.current_full_stack, self.current_extracted, ts))
                self.is_stacking = False
                print(f"🏁 Vehicle {ts} moved to processing.")


def background_processor():
    while True:
        data_packet = processing_queue.get()
        if data_packet is None: break
        full_stack, extracted_stack, ts = data_packet

        # Create folder for this vehicle
        folder = f"vehicle_{ts}"
        if not os.path.exists(folder): os.makedirs(folder)

        # 1. Save Raw Stacked Profile
        pcd_raw = o3d.geometry.PointCloud()
        pcd_raw.points = o3d.utility.Vector3dVector(np.array(full_stack))
        o3d.io.write_point_cloud(f"{folder}/1_raw_stacked.pcd", pcd_raw)

        # 2. Save Extracted Profile
        pcd_ext = o3d.geometry.PointCloud()
        pcd_ext.points = o3d.utility.Vector3dVector(np.array(extracted_stack))
        o3d.io.write_point_cloud(f"{folder}/2_extracted_object.pcd", pcd_ext)

        # 3. Outlier Removal
        pcd_clean, _ = pcd_ext.remove_statistical_outlier(nb_neighbors=25, std_ratio=0.8)
        pcd_clean, _ = pcd_clean.remove_radius_outlier(nb_points=12, radius=0.06)
        o3d.io.write_point_cloud(f"{folder}/3_cleaned_object.pcd", pcd_clean)

        # 4. Save BEV Image
        pts = np.asarray(pcd_clean.points)
        if len(pts) > 0:
            x_min, x_max, z_max = np.min(pts[:, 0]), np.max(pts[:, 0]), np.max(pts[:, 2])
            w, h = int((x_max - x_min + 0.2) / GRID_RES), int((z_max + 0.2) / GRID_RES)
            bev = np.zeros((h, w), dtype=np.uint8)
            for x, y, z in pts:
                xi, zi = int((x - x_min + 0.1) / GRID_RES), int(z / GRID_RES)
                if 0 <= xi < w and 0 <= zi < h: bev[zi, xi] = 255
            cv2.imwrite(f"{folder}/4_BEV_image.png", np.flipud(bev))

        print(f"✅ All stages saved in {folder}")
        processing_queue.task_done()


if __name__ == "__main__":
    TollPlazaSystem().run_forever()
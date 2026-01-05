import os

# CRITICAL: Limit threads BEFORE importing numpy or open3d to prevent memory crashes on Pi
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"

import faulthandler

faulthandler.enable()  # Diagnostic for exit code -11

import serial
import struct
import numpy as np
import time
import cv2
import open3d as o3d
import threading
import queue
from datetime import datetime
import statistics

# =====================================================
# CONFIGURATION
# =====================================================
# Updated for Linux Serial Port
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 921600

START_ANGLE = -48.0
ANGULAR_RES = 0.3516

CALIBRATION_FRAMES = 1000  # Reduced for Pi stability
GRID_CELL_SIZE = 0.05

MIN_RANGE_M = 0.10
MAX_RANGE_M = 3.5

VEHICLE_SPEED_MPS = 0.5 / 3.6
TRIGGER_THRESHOLD = 30  # Increased to filter noise
IDLE_TIMEOUT = 1.0

# Image Generation
X_IMG_RANGE = (-1, 1)
Y_IMG_RANGE = (-1, 1)
GRID_RES = 0.005
MAX_DIST_INTENSITY = 50.0

processing_queue = queue.Queue()


def robust_median(distances):
    if len(distances) < 5: return None
    median = statistics.median(distances)
    deviations = [abs(d - median) for d in distances]
    mad = statistics.median(deviations)
    if mad == 0: return median
    filtered = [d for d in distances if abs(d - median) <= 3 * mad]
    return statistics.median(filtered) if filtered else None


class TollPlazaSystem:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        except Exception as e:
            print(f"❌ Serial Error: {e}. Check if port is correct and permissions granted.")
            exit(1)
        self.background_matrix = set()
        self.is_stacking = False
        self.current_extracted = []
        self.last_detection_time = time.time()
        self.start_capture_time = None

    def calibrate(self):
        print(f"⌛ Calibrating Zero Plane ({CALIBRATION_FRAMES} frames)...")
        beam_history = {}
        frames = 0
        while frames < CALIBRATION_FRAMES:
            if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
                size = struct.unpack('<H', self.ser.read(2))[0]
                body = self.ser.read(size)
                self.ser.read(2)
                if struct.unpack('<H', body[:2])[0] == 50011:
                    data = body[3:]
                    for i in range(len(data) // 2):
                        d = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                        if d > 0: beam_history.setdefault(i, []).append(d)
                    frames += 1

        for idx, history in beam_history.items():
            val = robust_median(history)
            if val:
                dist = val / 1000.0
                angle = np.radians(START_ANGLE + idx * ANGULAR_RES)
                x, y = dist * np.cos(angle), dist * np.sin(angle)
                self.background_matrix.add((int(x / GRID_CELL_SIZE), int(y / GRID_CELL_SIZE)))
        print("✅ Calibration Complete")

    def run_forever(self):
        threading.Thread(target=background_processor, daemon=True).start()
        self.calibrate()
        print("🚀 System Live")
        while True:
            if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
                size = struct.unpack('<H', self.ser.read(2))[0]
                body = self.ser.read(size)
                self.ser.read(2)
                if struct.unpack('<H', body[:2])[0] == 50011:
                    data = body[3:]
                    t = time.time()
                    z = (t - self.start_capture_time) * VEHICLE_SPEED_MPS if self.is_stacking else 0

                    frame_pts = []
                    for i in range(len(data) // 2):
                        d_mm = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                        if 100 < d_mm < 3500:
                            dist = d_mm / 1000.0
                            ang = np.radians(START_ANGLE + i * ANGULAR_RES)
                            x, y = dist * np.cos(ang), dist * np.sin(ang)
                            if (int(x / GRID_CELL_SIZE), int(y / GRID_CELL_SIZE)) not in self.background_matrix:
                                frame_pts.append([x, y, z])

                    if len(frame_pts) > TRIGGER_THRESHOLD:
                        if not self.is_stacking:
                            self.is_stacking, self.start_capture_time = True, time.time()
                            self.current_extracted = []
                            print("🚗 Vehicle Detected")
                        self.current_extracted.extend(frame_pts)
                        self.last_detection_time = time.time()
                    elif self.is_stacking and (time.time() - self.last_detection_time > IDLE_TIMEOUT):
                        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                        processing_queue.put((list(self.current_extracted), ts))
                        self.is_stacking = False
                        print(f"🏁 Vehicle {ts} queued for processing")


def background_processor():
    while True:
        raw_data, ts = processing_queue.get()
        try:
            folder = f"vehicle_{ts}"
            os.makedirs(folder, exist_ok=True)
            pts = np.array(raw_data)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)

            # 1. Downsample first to save RAM and prevent SegFault
            pcd = pcd.voxel_down_sample(voxel_size=0.02)

            # 2. ARM-Optimized Cleaning
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=15, std_ratio=2.0)

            # 3. Side View Save
            R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
            pts_side = (R @ np.asarray(pcd.points).T).T
            pcd_side = o3d.geometry.PointCloud()
            pcd_side.points = o3d.utility.Vector3dVector(pts_side)
            o3d.io.write_point_cloud(f"{folder}/side_view.pcd", pcd_side)

            # 4. Image Generation
            x_bins, y_bins = int(2 / GRID_RES), int(2 / GRID_RES)
            bev = np.zeros((y_bins, x_bins), dtype=np.float32)
            for x, y, _ in pts_side:
                xi, yi = int((x + 1) / GRID_RES), int((y + 1) / GRID_RES)
                if 0 <= xi < x_bins and 0 <= yi < y_bins:
                    bev[yi, xi] = 1.0

            bev_img = (bev * 255).astype(np.uint8)
            cv2.imwrite(f"{folder}/side_view_image.png", np.flipud(bev_img))
            print(f"✅ Saved vehicle_{ts}")
        except Exception as e:
            print(f"⚠️ Processing Error: {e}")
        finally:
            processing_queue.task_done()


if __name__ == "__main__":
    TollPlazaSystem().run_forever()
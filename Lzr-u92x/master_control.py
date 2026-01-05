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
SERIAL_PORT = 'COM5'
BAUD_RATE = 921600

START_ANGLE = -48.0
ANGULAR_RES = 0.3516

CALIBRATION_FRAMES = 3000
GRID_CELL_SIZE = 0.05

MIN_RANGE_M = 0.10
MAX_RANGE_M = 3.5
MAX_NEIGHBOR_JUMP = 0.15

VEHICLE_SPEED_KMPH = 0.5
VEHICLE_SPEED_MPS = VEHICLE_SPEED_KMPH / 3.6
TRIGGER_THRESHOLD = 20
IDLE_TIMEOUT = 0.8

# Image Generation Parameters (from your Image Code)
X_IMG_RANGE = (-1, 1)
Y_IMG_RANGE = (-1, 1)
GRID_RES = 0.005  # 5mm per pixel
MAX_DIST_INTENSITY = 50.0

processing_queue = queue.Queue()


# =====================================================
# ROBUST MEDIAN (MAD)
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
# MAIN SYSTEM
# =====================================================
class TollPlazaSystem:
    def __init__(self):
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        self.background_matrix = set()
        self.is_stacking = False
        self.current_full_stack = []
        self.current_extracted = []
        self.last_detection_time = time.time()
        self.start_capture_time = None

    def calibrate(self):
        print(f"⌛ Robust Zero Plane Training ({CALIBRATION_FRAMES} frames)")
        os.makedirs("calibration", exist_ok=True)
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
                        if d > 0:
                            beam_history.setdefault(i, []).append(d)
                    frames += 1
                    if frames % 200 == 0:
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
        if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
            size = struct.unpack('<H', self.ser.read(2))[0]
            body = self.ser.read(size)
            self.ser.read(2)
            if struct.unpack('<H', body[:2])[0] == 50011:
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
        threading.Thread(target=background_processor, daemon=True).start()
        self.calibrate()
        print("🚀 System Live")
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

            if len(extracted) > TRIGGER_THRESHOLD:
                if not self.is_stacking:
                    self.is_stacking, self.start_capture_time = True, time.time()
                    self.current_full_stack, self.current_extracted = [], []
                    print("🚗 Vehicle Detected")
                self.current_full_stack.extend(raw_xyz)
                self.current_extracted.extend(extracted)
                self.last_detection_time = time.time()
            elif self.is_stacking and time.time() - self.last_detection_time > IDLE_TIMEOUT:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                processing_queue.put((self.current_extracted, ts))
                self.is_stacking = False
                print(f"🏁 Vehicle {ts} queued")


# =====================================================
# BACKGROUND PROCESSOR (Integrated Image Generation)
# =====================================================
def background_processor():
    while True:
        extracted_stack, ts = processing_queue.get()
        folder = f"vehicle_{ts}"
        os.makedirs(folder, exist_ok=True)

        # 1. Cleaning
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(extracted_stack))
        pcd, _ = pcd.remove_statistical_outlier(25, 0.8)
        pcd, _ = pcd.remove_radius_outlier(12, 0.06)

        # 2. Side View Transformation (Rotates for profile view)
        R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
        pts_side = (R @ np.asarray(pcd.points).T).T

        pcd_side = o3d.geometry.PointCloud()
        pcd_side.points = o3d.utility.Vector3dVector(pts_side)
        o3d.io.write_point_cloud(f"{folder}/side_view.pcd", pcd_side)

        # 3. Side View Image Generation (Using logic from your Image Code)
        if len(pts_side) > 0:
            # ROI Filter on the transformed side points
            mask = (
                    (pts_side[:, 0] >= X_IMG_RANGE[0]) & (pts_side[:, 0] <= X_IMG_RANGE[1]) &
                    (pts_side[:, 1] >= Y_IMG_RANGE[0]) & (pts_side[:, 1] <= Y_IMG_RANGE[1])
            )
            img_points = pts_side[mask]

            if len(img_points) > 0:
                x_bins = int((X_IMG_RANGE[1] - X_IMG_RANGE[0]) / GRID_RES)
                y_bins = int((Y_IMG_RANGE[1] - Y_IMG_RANGE[0]) / GRID_RES)
                bev = np.zeros((y_bins, x_bins), dtype=np.float32)

                for x, y, z in img_points:
                    xi = int((x - X_IMG_RANGE[0]) / GRID_RES)
                    yi = int((y - Y_IMG_RANGE[0]) / GRID_RES)

                    if 0 <= xi < x_bins and 0 <= yi < y_bins:
                        dist = np.sqrt(x ** 2 + y ** 2)
                        intensity = 1.0 - min(dist / MAX_DIST_INTENSITY, 1.0)
                        if intensity > bev[yi, xi]:
                            bev[yi, xi] = intensity

                # Final Image Processing
                bev_img = (bev * 255).astype(np.uint8)
                bev_img = np.flipud(bev_img)
                bev_img = cv2.GaussianBlur(bev_img, (3, 3), 0)

                cv2.imwrite(f"{folder}/side_view_image.png", bev_img)

        print(f"✅ Vehicle {ts} Processed: side_view.pcd and side_view_image.png saved.")
        processing_queue.task_done()


if __name__ == "__main__":
    TollPlazaSystem().run_forever()
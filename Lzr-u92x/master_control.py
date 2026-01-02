import serial
import struct
import numpy as np
import time
import cv2
import open3d as o3d
import threading
import queue
from datetime import datetime

# =====================================================
# CONFIGURATION
# =====================================================
SERIAL_PORT = 'COM8'  # Updated to match your new file
BAUD_RATE = 921600
START_ANGLE = -48.0
ANGULAR_RES = 0.3516
GRID_CELL_SIZE = 0.10
VEHICLE_SPEED_KMPH = 2  # Adjust based on toll plaza speed
VEHICLE_SPEED_MPS = VEHICLE_SPEED_KMPH / 3.6

TRIGGER_THRESHOLD = 15
IDLE_TIMEOUT = 1.0
GRID_RES = 0.005

processing_queue = queue.Queue()


def background_processor():
    """ Handles heavy CPU tasks without stopping the LiDAR capture. """
    while True:
        data_packet = processing_queue.get()
        if data_packet is None: break

        points_list, ts = data_packet
        print(f"\n⚙️ Processing vehicle {ts} ({len(points_list)} points)...")

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points_list))

        # Statistical Cleaning
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
        pcd = pcd.select_by_index(ind)

        # Radius Cleaning
        cl, ind = pcd.remove_radius_outlier(nb_points=16, radius=0.05)
        pcd_final = pcd.select_by_index(ind)

        # Save PCD and Image
        pcd_name = f"vehicle_{ts}.pcd"
        img_name = f"vehicle_{ts}_BEV.png"

        o3d.io.write_point_cloud(pcd_name, pcd_final)

        # Generate BEV Image
        points = np.asarray(pcd_final.points)
        if len(points) > 0:
            # Dynamic scaling based on captured data
            x_bins = int(3.0 / GRID_RES)
            z_max = np.max(points[:, 2]) if len(points) > 0 else 1.0
            y_bins = int((z_max + 0.5) / GRID_RES)

            bev = np.zeros((y_bins, x_bins), dtype=np.uint8)
            for x, y, z in points:
                xi = int(x / GRID_RES) + (x_bins // 2)
                zi = int(z / GRID_RES)  # Z is now the length of the vehicle
                if 0 <= xi < x_bins and 0 <= zi < y_bins:
                    bev[zi, xi] = 255
            cv2.imwrite(img_name, np.flipud(bev))

        print(f"✅ Saved {pcd_name} and {img_name}")
        processing_queue.task_done()


class TollPlazaSystem:
    def __init__(self):
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        self.background_matrix = set()
        self.is_stacking = False
        self.current_stack = []
        self.last_detection_time = time.time()
        self.start_capture_time = None

    def calibrate(self):
        """ Capture background for 5 seconds """
        print("⌛ Calibrating Zero Plane... Clear the lane.")
        end_time = time.time() + 5
        while time.time() < end_time:
            raw = self.get_raw_frame()
            if raw:
                for p in raw:
                    ix, iy = int(np.floor(p[0] / GRID_CELL_SIZE)), int(np.floor(p[1] / GRID_CELL_SIZE))
                    self.background_matrix.add((ix, iy))
        print(f"✅ Calibration complete ({len(self.background_matrix)} cells).")

    def get_raw_frame(self):
        """ Fast hex parsing """
        if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
            size = struct.unpack('<H', self.ser.read(2))[0]
            body = self.ser.read(size)
            self.ser.read(2)
            if struct.unpack('<H', body[:2])[0] == 50011:
                data = body[3:]
                points = []
                for i in range(len(data) // 2):
                    d_mm = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                    if 100 < d_mm < 3000:
                        dist = d_mm / 1000.0
                        rad = np.radians(START_ANGLE + i * ANGULAR_RES)
                        points.append([dist * np.cos(rad), dist * np.sin(rad)])
                return points
        return None

    def run_forever(self):
        threading.Thread(target=background_processor, daemon=True).start()
        self.calibrate()
        print("🚀 System Live. Monitoring traffic...")

        while True:
            raw_points = self.get_raw_frame()
            if not raw_points: continue

            # Filter
            obj_points = []
            current_time = time.time()

            # If we are stacking, calculate Z based on time since vehicle started
            z_val = (current_time - self.start_capture_time) * VEHICLE_SPEED_MPS if self.is_stacking else 0

            for p in raw_points:
                ix, iy = int(np.floor(p[0] / GRID_CELL_SIZE)), int(np.floor(p[1] / GRID_CELL_SIZE))
                if (ix, iy) not in self.background_matrix:
                    obj_points.append([p[0], p[1], z_val])

            # Trigger logic
            if len(obj_points) > TRIGGER_THRESHOLD:
                if not self.is_stacking:
                    print("🚗 Vehicle Detected - Stacking...")
                    self.is_stacking = True
                    self.current_stack = []
                    self.start_capture_time = time.time()

                self.current_stack.extend(obj_points)
                self.last_detection_time = time.time()

            elif self.is_stacking and (time.time() - self.last_detection_time > IDLE_TIMEOUT):
                ts = datetime.now().strftime("%H%M%S")
                processing_queue.put((self.current_stack, ts))
                self.is_stacking = False
                print("🏁 Vehicle passed. Data sent to background thread.")


if __name__ == "__main__":
    TollPlazaSystem().run_forever()
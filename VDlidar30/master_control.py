import serial
import struct
import numpy as np
import time
import open3d as o3d
import threading
import queue
import os
from datetime import datetime
import statistics
from scipy.spatial import cKDTree
import sys

# =====================================================
# CONFIGURATION
# =====================================================
SERIAL_PORT = 'COM3'
BAUD_RATE = 512000

# Directory Structure
BASE_DIR = "scan_data"
CALIB_DIR = os.path.join(BASE_DIR, "calibration")
RAW_DIR = os.path.join(BASE_DIR, "raw_stacks")
EXTRACTED_DIR = os.path.join(BASE_DIR, "extracted_objects")

for folder in [CALIB_DIR, RAW_DIR, EXTRACTED_DIR]:
    os.makedirs(folder, exist_ok=True)

# Processing Parameters
Z_INCREMENT_METERS = 0.0075  # Simulation of movement speed (Z-axis stretching)
DIST_THRESHOLD = 0.05  # 5cm tolerance for background changes
MIN_CLUSTER_POINTS = 30  # Minimum points to constitute a "real" object
CLUSTER_DISTANCE = 0.08  # DBSCAN Epsilon: Max distance between points in a cluster

# Calibration & Triggering
CALIBRATION_ROTATIONS = 120  # High sample count for accurate 360 background
ANGLE_BIN_SIZE = 0.5  # 0.5 degree resolution
TRIGGER_COUNT_THRESHOLD = 15  # Sensitivity: How many points distinct from BG trigger a capture?
IDLE_TIMEOUT = 2.0  # Time to wait after motion stops before saving

processing_queue = queue.Queue()


# =====================================================
# BACKGROUND PROCESSOR (Cleaning & Extraction)
# =====================================================
def background_processor(zero_plane_pcd_path):
    """
    Consumer Thread:
    1. Loads the Zero Plane (Background).
    2. Subtracts Background from the Raw Stack.
    3. Removes Noise (Statistical Outlier).
    4. Clusters remaining points to isolate the object.
    """
    print("   [Processor] Loading background model...")
    pcd_zero = o3d.io.read_point_cloud(zero_plane_pcd_path)
    if pcd_zero.is_empty():
        print("❌ [Processor] Error: Background file is empty.")
        return

    pts_zero = np.asarray(pcd_zero.points)
    # KDTree for ultra-fast 2D spatial queries
    ref_tree = cKDTree(pts_zero[:, 0:2])

    while True:
        # Wait for data from the main thread
        raw_stack_pts, ts = processing_queue.get()
        pts_stack = np.array(raw_stack_pts)

        if len(pts_stack) == 0:
            processing_queue.task_done()
            continue

        print(f"   [Processor] Processing Scan {ts} ({len(pts_stack)} points)...")

        # 1. Save Raw Stack (Backup)
        raw_pcd = o3d.geometry.PointCloud()
        raw_pcd.points = o3d.utility.Vector3dVector(pts_stack)
        o3d.io.write_point_cloud(os.path.join(RAW_DIR, f"raw_{ts}.pcd"), raw_pcd)

        # 2. Background Subtraction
        # We compare the (Y, Z) of the stack (Lidar sensor plane)
        # against the (X, Y) of the calibration (Background plane)
        # Note: In the stack, Y/Z are the sensor's scanning coordinates.
        distances, _ = ref_tree.query(pts_stack[:, 1:3], k=1)

        # Filter: Keep points that are FURTHER than DIST_THRESHOLD from the background
        object_indices = np.where(distances > DIST_THRESHOLD)[0]

        if len(object_indices) > MIN_CLUSTER_POINTS:
            extracted_pts = pts_stack[object_indices]

            pcd_obj = o3d.geometry.PointCloud()
            pcd_obj.points = o3d.utility.Vector3dVector(extracted_pts)

            # 3. Statistical Outlier Removal (Noise Reduction)
            # Removes stray points ("dust") that aren't part of a solid shape
            pcd_clean, _ = pcd_obj.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

            if len(pcd_clean.points) > MIN_CLUSTER_POINTS:
                # 4. DBSCAN Clustering
                # Groups points together; ignores isolated clusters
                labels = np.array(pcd_clean.cluster_dbscan(eps=CLUSTER_DISTANCE,
                                                           min_points=MIN_CLUSTER_POINTS))

                if len(labels) > 0 and np.any(labels >= 0):
                    # Select the largest cluster (assumed to be the main object)
                    counts = np.bincount(labels[labels >= 0])
                    largest_cluster_idx = np.argmax(counts)
                    final_pts = np.asarray(pcd_clean.points)[labels == largest_cluster_idx]

                    final_pcd = o3d.geometry.PointCloud()
                    final_pcd.points = o3d.utility.Vector3dVector(final_pts)

                    # Save Final
                    output_path = os.path.join(EXTRACTED_DIR, f"object_{ts}.pcd")
                    o3d.io.write_point_cloud(output_path, final_pcd)
                    print(f"✅ [Processor] Saved Clean Object: {output_path}")
                else:
                    print("⚠️ [Processor] Object removed during clustering (too scattered).")
            else:
                print("⚠️ [Processor] Object removed during noise filtering.")
        else:
            print("⚠️ [Processor] No substantial object found after subtraction.")

        processing_queue.task_done()


# =====================================================
# MAIN PIPELINE (Unfiltered 360 Detection)
# =====================================================
class PCDMasterPipeline:
    def __init__(self):
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        self.is_stacking = False
        self.current_stack = []
        self.scan_count = 0
        self.bg_tree = None
        self.buffer = b""

    def lidar_command(self, cmd):
        self.ser.setDTR(True)  # Wake up TG30
        self.ser.write(cmd)
        time.sleep(0.5)

    def get_packets(self):
        """Reads and parses TG30 packets continuously."""
        if self.ser.in_waiting:
            self.buffer += self.ser.read(self.ser.in_waiting)
            while len(self.buffer) > 10:
                # Check for Start Header (0xAA 0x55)
                if (self.buffer[0] == 0xAA and self.buffer[1] == 0x55):
                    lsn = self.buffer[3]
                    packet_len = 10 + 2 * lsn
                    if len(self.buffer) < packet_len: return None
                    packet = self.buffer[:packet_len]
                    self.buffer = self.buffer[packet_len:]
                    return self.parse_packet(packet)
                self.buffer = self.buffer[1:]
        return None

    def parse_packet(self, packet):
        """Decodes packet data without angular limits."""
        lsn = packet[3]
        fsa = struct.unpack('<H', packet[4:6])[0]
        lsa = struct.unpack('<H', packet[6:8])[0]
        angle_fsa = (fsa >> 1) / 64.0
        angle_lsa = (lsa >> 1) / 64.0
        diff = (angle_lsa - angle_fsa) % 360
        step = diff / (lsn - 1) if lsn > 1 else 0

        pts = []
        for i in range(lsn):
            dist_raw = struct.unpack('<H', packet[10 + i * 2:12 + i * 2])[0]
            dist_m = (dist_raw / 4.0) / 1000.0

            # PHYSICAL LIMIT ONLY: Ignore sensor blind spot (< 5cm)
            # We DO NOT cap the max distance. It sees as far as it can.
            if dist_m > 0.05:
                angle = (angle_fsa + step * i) % 360
                pts.append((angle, dist_m))
        return pts

    def create_pcd_zero_plane(self):
        """
        Creates a robust 360-degree background model using Median filtering.
        """
        print("⌛ Calibrating 360° Background... (Keep area clear)")
        self.lidar_command(b'\xA5\x60')

        # Bins for every 0.5 degrees
        polar_bins = {}
        frames, last_angle = 0, 0

        # Collect data
        while frames < CALIBRATION_ROTATIONS:
            data = self.get_packets()
            if data:
                for angle, dist in data:
                    b_idx = int(angle / ANGLE_BIN_SIZE)
                    polar_bins.setdefault(b_idx, []).append(dist)

                    # Count rotations
                    if angle < last_angle and (last_angle - angle > 180):
                        frames += 1
                        sys.stdout.write(f"\r   Progress: {frames}/{CALIBRATION_ROTATIONS} rotations")
                        sys.stdout.flush()
                    last_angle = angle

        print("\n   Computing statistical median...")
        clean_points = []
        for bin_idx, distances in polar_bins.items():
            if len(distances) > 5:
                # Median filters out moving objects (people walking by) during calib
                m_dist = statistics.median(distances)
                angle_rad = np.radians(bin_idx * ANGLE_BIN_SIZE)

                # Convert Polar -> Cartesian (2D)
                clean_points.append([m_dist * np.cos(angle_rad), m_dist * np.sin(angle_rad), 0.0])

        # Save Background PCD
        pcd_zero = o3d.geometry.PointCloud()
        pcd_zero.points = o3d.utility.Vector3dVector(np.array(clean_points))
        calib_file = os.path.join(CALIB_DIR, "zero_plane_reference.pcd")
        o3d.io.write_point_cloud(calib_file, pcd_zero)

        # Build Tree for live comparison
        self.bg_tree = cKDTree(np.array(clean_points)[:, 0:2])
        print(f"✅ Calibration Complete. Reference saved to: {calib_file}")
        return calib_file

    def run(self):
        calib_path = self.create_pcd_zero_plane()

        # Start the background processor thread
        threading.Thread(target=background_processor, args=(calib_path,), daemon=True).start()

        last_angle, last_detect_time = 0.0, time.time()
        print(f"🚀 System Live. Monitoring FULL 360° FOV...")

        try:
            while True:
                pts = self.get_packets()
                if not pts: continue

                points_above_bg = 0
                frame_xyz = []

                for angle, dist in pts:
                    # Rotation Counter (for Z-stacking)
                    if angle < last_angle and (last_angle - angle > 180):
                        self.scan_count += 1
                    last_angle = angle

                    # Convert to Cartesian
                    rad = np.radians(angle)
                    y_sens, z_sens = dist * np.cos(rad), dist * np.sin(rad)
                    x_world = self.scan_count * Z_INCREMENT_METERS

                    # ---------------------------------------------------------
                    # TRIGGER LOGIC: 360 DEGREE CHECK
                    # ---------------------------------------------------------
                    # Check if this point exists in our Background Model
                    dist_to_bg, _ = self.bg_tree.query([y_sens, z_sens], k=1)

                    # If point is significantly different from background (> 5cm)
                    # It counts towards the "Object Detected" threshold
                    if dist_to_bg > DIST_THRESHOLD:
                        points_above_bg += 1

                    frame_xyz.append([x_world, y_sens, z_sens])

                # ---------------------------------------------------------
                # STATE MACHINE
                # ---------------------------------------------------------
                # If we see enough anomalous points, we assume an object is moving
                if points_above_bg > TRIGGER_COUNT_THRESHOLD:
                    if not self.is_stacking:
                        print("\n🔴 Motion Detected! Recording...")
                        self.is_stacking = True
                        self.current_stack = []
                        self.scan_count = 0  # Reset Z-axis for new object

                    # Accumulate data
                    self.current_stack.extend(frame_xyz)
                    last_detect_time = time.time()

                # If silence persists, finish the capture
                elif self.is_stacking and (time.time() - last_detect_time > IDLE_TIMEOUT):
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    print(f"\n🏁 Capture Complete. Queuing {len(self.current_stack)} points...")
                    processing_queue.put((self.current_stack, ts))

                    self.is_stacking = False

                # Live Status Indicator
                sys.stdout.write(
                    f"\rStatus: {'RECORDING' if self.is_stacking else 'IDLE'} | Delta Pts: {points_above_bg:03}   ")
                sys.stdout.flush()

        except KeyboardInterrupt:
            print("\nStopping...")
            self.lidar_command(b'\xA5\x65')
            self.ser.close()


if __name__ == "__main__":
    PCDMasterPipeline().run()
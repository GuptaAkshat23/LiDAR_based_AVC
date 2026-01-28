import serial
import struct
import numpy as np
import time
import cv2
import open3d as o3d
import multiprocessing as mp  # Used for parallel processing (Capture vs. Processing)
import queue  # Used for handling queue exceptions
import os
import sys
import glob
from datetime import datetime
import statistics
import gc  # Garbage Collector interface: Used to prevent latency spikes during vehicle capture

"""
SYSTEM OVERVIEW:
----------------
This script captures LiDAR data from a BEA LZR-U921 sensor to create 3D vehicle profiles.
It is optimized for Single Board Computers (like Raspberry Pi) using the following techniques:
1. Multiprocessing: Capture runs on the main core; 3D processing runs on a separate core.
2. Virtual Clocking: Ignores system jitter by assigning fixed time slots to frames based on sensor FPS.
3. Garbage Collection Control: Disables Python GC during active vehicle capture to prevent frame drops.
4. Background Subtraction: Learns the "empty road" to isolate vehicle points.
"""

# =====================================================
# CONFIGURATION PARAMETERS
# =====================================================

# --- Serial Communication ---
SERIAL_PORT = '/dev/ttyUSB0'  # Port where the LZR sensor is connected
BAUD_RATE = 921600  # High baud rate required for high-frequency LiDAR data

# --- Sensor Timing & Logic ---
# The LZR-U921 scans at ~66Hz. To reconstruct a smooth 3D object from 2D slices,
# we treat every frame as occupying exactly 15.15ms (1/66), regardless of OS lag.
SENSOR_FPS = 66.0
FRAME_INTERVAL = 1.0 / SENSOR_FPS

# --- Sensor Physical Properties ---
START_ANGLE = -48.0  # The starting angle of the laser scan in degrees
ANGULAR_RES = 0.3516  # The angular step between each measurement point

# --- Calibration Settings ---
CALIBRATION_FRAMES = 4000  # Number of frames to read to learn the background (empty road)
GRID_CELL_SIZE = 0.05  # Grid size (in meters) for background discretization

# --- Data Filtering ---
MIN_RANGE_M = 0.10  # Ignore points closer than 10cm (sensor noise/cover)
MAX_RANGE_M = 2.25  # Ignore points further than 2.25m (ground/far walls)
MAX_NEIGHBOR_JUMP = 0.15  # Max distance between points to be considered continuous

# --- Vehicle Reconstruction Logic ---
VEHICLE_SPEED_KMPH = 5  # Assumed constant speed of vehicle through toll
VEHICLE_SPEED_MPS = VEHICLE_SPEED_KMPH / 3.6  # Convert km/h to m/s
TRIGGER_THRESHOLD = 50  # Minimum number of non-background points to start recording a car
IDLE_TIMEOUT = 0.8  # Seconds of silence (no vehicle points) before declaring the car has passed

# --- Image Output Settings ---
GRID_RES = 0.005  # Pixel resolution in meters for the generated image (5mm per pixel)
MAX_DIST_INTENSITY = 50.0  # Normalization factor for pixel intensity

# --- 3D Rotation Correction ---
# These angles correct the physical mounting tilt of the sensor.
ROTATION_X_DEG = 25.0
ROTATION_Y_DEG = 0.0
ROTATION_Z_DEG = 0.0

# --- Dynamic Frame Output ---
FRAME_PADDING_M = 0.2  # Add 20cm padding around the detected vehicle in the image
MIN_IMAGE_DIM_M = 1.0  # Minimum size of the output image in meters


# =====================================================
# HELPER: ROBUST MEDIAN (MAD)
# =====================================================
def robust_median(distances):
    """
    Calculates the median distance for a specific beam angle, filtering out noise.
    Uses Median Absolute Deviation (MAD) to ignore flickering measurements.

    Args:
        distances (list): List of distance measurements for a single angle.
    Returns:
        float: The filtered median distance, or None if data is insufficient.
    """
    if len(distances) < 5:
        return None

    # 1. Calculate standard median
    median = statistics.median(distances)

    # 2. Calculate deviations from the median
    deviations = [abs(d - median) for d in distances]
    mad = statistics.median(deviations)

    # 3. If variance is 0, return median
    if mad == 0:
        return median

    # 4. Filter out points that deviate more than 3x the MAD (outliers)
    filtered = [d for d in distances if abs(d - median) <= 3 * mad]

    return statistics.median(filtered) if filtered else None


# =====================================================
# BACKGROUND WORKER (CONSUMER PROCESS)
# =====================================================
def background_worker(input_queue):
    """
    Runs on a separate CPU core.
    Fetches raw 3D point clouds from the queue, cleans noise, performs rotation,
    and generates the final 2D image.
    """
    print("🧵 Background Worker Process Started (PID: {})".format(os.getpid()))

    while True:
        try:
            # Block until data is available in the queue
            extracted_stack, ts = input_queue.get()

            # Sanity Check: If the car is too small (fewer than 30 points), ignore it.
            if len(extracted_stack) < 30:
                print(f"⚠️ Vehicle {ts} Skipped: Too few points ({len(extracted_stack)})")
                continue

            folder = f"vehicle_{ts}"
            os.makedirs(folder, exist_ok=True)

            # -------------------------------------------------
            # STEP 1: Point Cloud Creation & Cleaning
            # -------------------------------------------------
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(extracted_stack, dtype=np.float64))

            # Statistical Outlier Removal: Removes points that are further away from their neighbors compared to the average.
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=25, std_ratio=0.8)
            if len(pcd.points) == 0:
                print(f"⚠️ Vehicle {ts} Skipped: Empty after statistical cleanup")
                continue

            # Radius Outlier Removal: Removes isolated points that don't have at least 12 neighbors within 6cm.
            pcd, _ = pcd.remove_radius_outlier(nb_points=12, radius=0.06)
            if len(pcd.points) == 0:
                print(f"⚠️ Vehicle {ts} Skipped: Empty after radius cleanup")
                continue

            # -------------------------------------------------
            # STEP 2: Coordinate System Transformation
            # -------------------------------------------------
            # Swap axes to align with standard Side View (Vehicle moving X, Height Z)
            R_standard = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=np.float64)
            pts_side = (R_standard @ np.asarray(pcd.points).T).T

            # -------------------------------------------------
            # STEP 3: Mounting Rotation Correction
            # -------------------------------------------------
            # Apply X, Y, Z rotations to correct for how the sensor is bolted to the pole.
            rad_x = np.radians(ROTATION_X_DEG)
            rad_y = np.radians(ROTATION_Y_DEG)
            rad_z = np.radians(ROTATION_Z_DEG)

            # Rotation Matrix for X-axis
            Rx = np.array([
                [1, 0, 0],
                [0, np.cos(rad_x), -np.sin(rad_x)],
                [0, np.sin(rad_x), np.cos(rad_x)]
            ])
            # Rotation Matrix for Y-axis
            Ry = np.array([
                [np.cos(rad_y), 0, np.sin(rad_y)],
                [0, 1, 0],
                [-np.sin(rad_y), 0, np.cos(rad_y)]
            ])
            # Rotation Matrix for Z-axis
            Rz = np.array([
                [np.cos(rad_z), -np.sin(rad_z), 0],
                [np.sin(rad_z), np.cos(rad_z), 0],
                [0, 0, 1]
            ])

            # Combine rotations
            R_total = Rz @ Ry @ Rx
            pts_side = (R_total @ pts_side.T).T

            # Save the fully corrected 3D Point Cloud for debugging/viewing
            pcd_side = o3d.geometry.PointCloud()
            pcd_side.points = o3d.utility.Vector3dVector(pts_side)
            o3d.io.write_point_cloud(f"{folder}/side_view.pcd", pcd_side)

            # -------------------------------------------------
            # STEP 4: Dynamic Framing (Bounding Box)
            # -------------------------------------------------
            # Find the min/max extents of the vehicle to crop the image efficiently
            min_x, min_y = np.min(pts_side[:, :2], axis=0)
            max_x, max_y = np.max(pts_side[:, :2], axis=0)

            # Apply padding
            min_x -= FRAME_PADDING_M
            max_x += FRAME_PADDING_M
            min_y -= FRAME_PADDING_M
            max_y += FRAME_PADDING_M

            # Enforce minimum image size
            if (max_x - min_x) < MIN_IMAGE_DIM_M: max_x = min_x + MIN_IMAGE_DIM_M
            if (max_y - min_y) < MIN_IMAGE_DIM_M: max_y = min_y + MIN_IMAGE_DIM_M

            # Calculate output image resolution
            width_m = max_x - min_x
            height_m = max_y - min_y
            x_bins = int(width_m / GRID_RES)
            y_bins = int(height_m / GRID_RES)

            # -------------------------------------------------
            # STEP 5: Image Generation (Projection)
            # -------------------------------------------------
            bev = np.zeros((y_bins, x_bins), dtype=np.float32)

            # Map float coordinates to integer pixel indices
            xi = ((pts_side[:, 0] - min_x) / GRID_RES).astype(np.int32)
            yi = ((pts_side[:, 1] - min_y) / GRID_RES).astype(np.int32)

            # Keep only points that fall inside the image bounds
            valid_indices = (xi >= 0) & (xi < x_bins) & (yi >= 0) & (yi < y_bins)
            xi = xi[valid_indices]
            yi = yi[valid_indices]
            v_points = pts_side[valid_indices]

            if len(v_points) > 0:
                # Calculate pixel intensity based on depth (creates a 3D effect in 2D)
                dist = np.sqrt(v_points[:, 0] ** 2 + v_points[:, 1] ** 2)
                intensity = 1.0 - np.minimum(dist / MAX_DIST_INTENSITY, 1.0)

                bev[yi, xi] = intensity

                # Convert to 8-bit image
                bev_img = (bev * 255).astype(np.uint8)
                bev_img = np.flipud(bev_img)  # Flip logic to match image coordinates
                bev_img = cv2.GaussianBlur(bev_img, (3, 3), 0)  # Smooth out gaps

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
# MAIN SYSTEM CLASS (PRODUCER PROCESS)
# =====================================================
class TollPlazaSystem:
    def __init__(self):
        print(f"🔌 Connecting to LiDAR on {SERIAL_PORT}...")
        try:
            # Initialize Serial Connection
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

            # [CRITICAL] Increase System Buffer
            # Linux buffers can be small. If we process data slower than it arrives,
            # the buffer overflows and we lose data. We manually request a larger buffer.
            try:
                self.ser.set_buffer_size(rx_size=100000, tx_size=100000)
            except AttributeError:
                pass  # This method might not exist on all platforms/versions, safe to ignore.

        except serial.SerialException as e:
            print(f"❌ Serial Error: {e}")
            print("Troubleshooting: Check permissions (sudo chmod 666 /dev/ttyUSB0) or connection.")
            sys.exit(1)

        # Initialize Processing Queue
        self.processing_queue = mp.Queue()

        # Runtime variables
        self.background_matrix = set()  # Stores grid cells considered "background"
        self.is_stacking = False  # Flag: True if a vehicle is currently passing
        self.current_full_stack = []  # Raw points of current vehicle
        self.current_extracted = []  # Background-subtracted points of current vehicle
        self.last_detection_time = time.time()
        self.start_capture_time = None

    def calibrate(self):
        """
        Reads N frames from the sensor to learn the background.
        Calculates the median distance for every angle to create a "Zero Plane".
        """
        print(f"⌛ Robust Zero Plane Training ({CALIBRATION_FRAMES} frames)...")
        os.makedirs("calibration", exist_ok=True)
        beam_history = {}
        frames = 0

        # Loop until we have enough calibration frames
        while frames < CALIBRATION_FRAMES:
            # Check for Header: 0xFC followed by 0xFD, 0xFE, 0xFF (LZR Protocol)
            if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
                size_bytes = self.ser.read(2)
                if len(size_bytes) < 2: continue
                size = struct.unpack('<H', size_bytes)[0]
                body = self.ser.read(size)
                if len(body) < size: continue
                self.ser.read(2)  # Read checksum/footer

                # Message ID Check (50011 is the standard LZR data message)
                if len(body) > 2 and struct.unpack('<H', body[:2])[0] == 50011:
                    data = body[3:]
                    # Parse distances (2 bytes per point)
                    for i in range(len(data) // 2):
                        d = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                        if d > 0:
                            beam_history.setdefault(i, []).append(d)
                    frames += 1
                    if frames % 500 == 0:
                        print(f"    Frames: {frames}/{CALIBRATION_FRAMES}")

        # Compute Robust Median for every beam
        clean_zero_pts = []
        idxs = sorted(beam_history.keys())
        for j, idx in enumerate(idxs):
            robust_mm = robust_median(beam_history[idx])
            if robust_mm is None: continue

            # Convert to meters and polar coordinates
            dist_m = robust_mm / 1000.0
            if not (MIN_RANGE_M <= dist_m <= MAX_RANGE_M): continue

            angle = np.radians(START_ANGLE + idx * ANGULAR_RES)
            x, y = dist_m * np.cos(angle), dist_m * np.sin(angle)
            clean_zero_pts.append((x, y, 0.0))

            # Discretize space: Map (x,y) to a grid cell ID for fast lookup
            ix, iy = int(np.floor(x / GRID_CELL_SIZE)), int(np.floor(y / GRID_CELL_SIZE))
            self.background_matrix.add((ix, iy))

        # Save calibration data
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(clean_zero_pts))
        o3d.io.write_point_cloud("calibration/lzr_zero_plane.pcd", pcd)
        print(f"✅ Robust Zero Plane Saved ({len(clean_zero_pts)} points)")

    def get_raw_frame(self):
        """
        Parses a single raw frame from the Serial buffer.
        Returns:
            list: [[x, y], [x, y]...] or None if failed.
        """
        # Look for the start bytes (LZR specific header)
        if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
            size_bytes = self.ser.read(2)
            if len(size_bytes) < 2: return None
            size = struct.unpack('<H', size_bytes)[0]
            body = self.ser.read(size)
            if len(body) < size: return None
            self.ser.read(2)

            if len(body) > 2 and struct.unpack('<H', body[:2])[0] == 50011:
                data = body[3:]
                pts = []
                # Convert raw bytes to distances and then to X,Y coordinates
                for i in range(len(data) // 2):
                    d_mm = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                    if 100 < d_mm < 3500:  # Simple range filter (10cm to 3.5m)
                        dist = d_mm / 1000.0
                        ang = np.radians(START_ANGLE + i * ANGULAR_RES)
                        pts.append([dist * np.cos(ang), dist * np.sin(ang)])
                return pts
        return None

    def run_forever(self):
        """
        Main loop.
        1. Launches background worker.
        2. Calibrates.
        3. Reads data continuously.
        4. Detects vehicles and sends data to worker.
        """
        # Start the background processing core
        p = mp.Process(target=background_worker, args=(self.processing_queue,))
        p.daemon = True
        p.start()

        self.calibrate()
        print("🚀 System Live - Waiting for vehicles...")

        # [VIRTUAL CLOCK LOGIC]
        # Real-time systems on non-RTOS (like Raspberry Pi Linux) suffer from jitter.
        # Instead of trusting `time.time()`, we increment time by 1/FPS for every frame received.
        # This ensures the vehicle reconstruction is smooth even if the OS pauses briefly.
        virtual_time = time.time()

        while True:
            # Get 2D scan slice
            raw_xy = self.get_raw_frame()
            if not raw_xy: continue

            # --- TIME ALLOTMENT ---
            if self.is_stacking:
                # If we are recording a car, we enforce strict timing (Virtual Clock)
                virtual_time += FRAME_INTERVAL
            else:
                # If we are idle, sync virtual time to real wall-clock time
                virtual_time = time.time()

            t = virtual_time

            # Calculate Z position (Travel axis) based on speed and time
            # Z = Speed * Time
            z = (t - self.start_capture_time) * VEHICLE_SPEED_MPS if self.is_stacking else 0
            raw_xyz = [[p[0], p[1], z] for p in raw_xy]

            # Background Subtraction
            extracted = []
            for p in raw_xy:
                # Check if the point falls into a grid cell marked as "background"
                ix, iy = int(np.floor(p[0] / GRID_CELL_SIZE)), int(np.floor(p[1] / GRID_CELL_SIZE))
                if (ix, iy) not in self.background_matrix:
                    extracted.append([p[0], p[1], z])

            # --- TRIGGER LOGIC ---
            # If enough non-background points are detected, start recording
            if len(extracted) > TRIGGER_THRESHOLD:
                if not self.is_stacking:
                    print("🚗 Vehicle Detected! Recording...")
                    self.is_stacking = True
                    self.start_capture_time = virtual_time  # Start Z-axis from 0 here

                    # [OPTIMIZATION] Disable Garbage Collection
                    # Python's GC causes "Stop-the-World" pauses. We disable it while
                    # the vehicle is passing to ensure we don't drop serial packets.
                    gc.disable()

                    self.current_full_stack = []
                    self.current_extracted = []

                self.current_full_stack.extend(raw_xyz)
                self.current_extracted.extend(extracted)

                # Update last detection time using REAL time to handle timeout logic
                self.last_detection_time = time.time()

            # --- COMPLETION LOGIC ---
            # If we were recording, but haven't seen points for IDLE_TIMEOUT seconds
            elif self.is_stacking and (time.time() - self.last_detection_time > IDLE_TIMEOUT):
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                print(f"🏁 Vehicle {ts} queued for processing ({len(self.current_extracted)} points)")

                # Send data to the background worker
                self.processing_queue.put((list(self.current_extracted), ts))

                self.is_stacking = False

                # [OPTIMIZATION] Re-enable Garbage Collection
                # Now that the high-speed event is over, we let Python clean up memory.
                gc.enable()
                gc.collect()


if __name__ == "__main__":
    # Ensure correct start method for multiprocessing (compatibility for Linux/Windows)
    mp.set_start_method('spawn', force=True)
    system = TollPlazaSystem()
    system.run_forever()
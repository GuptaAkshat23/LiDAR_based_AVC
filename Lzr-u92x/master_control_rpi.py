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
# CONFIGURATION CONSTANTS
# =====================================================
# Serial Communication Settings
# SERIAL_PORT: The file path to the USB device.
# On Raspberry Pi/Linux, this is typically '/dev/ttyUSB0' or '/dev/ttyACM0'.
# On Windows, this would be 'COMx' (e.g., 'COM3').
SERIAL_PORT = '/dev/ttyUSB0'

# BAUD_RATE: The speed of data transmission in bits per second.
# 921600 is a high-speed rate required for the LZR-U921 LiDAR to prevent data lag.
BAUD_RATE = 921600

# -----------------------------------------------------
# LiDAR Sensor Intrinsic Properties (Hardware Specific)
# -----------------------------------------------------
# START_ANGLE: The starting angle of the laser scan in degrees (from manufacturer datasheet).
START_ANGLE = -48.0

# ANGULAR_RES: The angular step size between each laser point in degrees.
ANGULAR_RES = 0.3516

# -----------------------------------------------------
# Calibration & Background Subtraction Parameters
# -----------------------------------------------------
# CALIBRATION_FRAMES: The number of empty road scans to capture on startup.
# These frames are averaged to create a 'Zero Plane' (background) to ignore later.
CALIBRATION_FRAMES = 4000

# GRID_CELL_SIZE: Resolution of the background grid in meters.
# The road is divided into 5cm x 5cm cells. If a point falls into a "background cell", it is ignored.
GRID_CELL_SIZE = 0.05

# -----------------------------------------------------
# Data Filtering Limits
# -----------------------------------------------------
# MIN_RANGE_M / MAX_RANGE_M: Distance limits.
# Points closer than 10cm or further than 2.25m (e.g., outside the lane) are discarded.
MIN_RANGE_M = 0.10
MAX_RANGE_M = 2.25

# MAX_NEIGHBOR_JUMP: Used for continuity checks (not actively used in main loop currently).
MAX_NEIGHBOR_JUMP = 0.15

# -----------------------------------------------------
# Vehicle Logic & Time-Stacking Reconstruction
# -----------------------------------------------------
# VEHICLE_SPEED_KMPH: Assumed average speed of vehicles passing the toll.
# This is crucial for 3D reconstruction. We map 'Time' to 'Distance' using this speed.
VEHICLE_SPEED_KMPH = 5
VEHICLE_SPEED_MPS = VEHICLE_SPEED_KMPH / 3.6  # Conversion to Meters per Second

# TRIGGER_THRESHOLD: Sensitivity setting.
# A frame must have at least 50 non-background points to define "Vehicle Present".
TRIGGER_THRESHOLD = 50

# IDLE_TIMEOUT: The "Gap" timer.
# If no vehicle points are seen for 0.8 seconds, the system assumes the vehicle has passed
# and stops recording.
IDLE_TIMEOUT = 0.8

# -----------------------------------------------------
# 2D Image Generation Settings
# -----------------------------------------------------
# GRID_RES: Resolution of the final output image (Meters per Pixel).
# 0.005 means each pixel represents 5mm of the vehicle.
GRID_RES = 0.005

# MAX_DIST_INTENSITY: Depth scaling factor.
# Used to determine pixel brightness based on how far the point is from the sensor.
MAX_DIST_INTENSITY = 50.0

# -----------------------------------------------------
# 3D Rotation Correction (Mounting Compensation)
# -----------------------------------------------------
# These values correct for the physical tilt of the sensor installation.
# ROTATION_X_DEG: Pitch correction (if sensor is tilted up/down).
ROTATION_X_DEG = 25.0
ROTATION_Y_DEG = 0.0
ROTATION_Z_DEG = 0.0

# -----------------------------------------------------
# Dynamic Image Sizing
# -----------------------------------------------------
# FRAME_PADDING_M: Adds empty space (margin) around the tightly cropped vehicle image.
FRAME_PADDING_M = 0.2

# MIN_IMAGE_DIM_M: Ensures the image is at least 1 meter wide/tall, preventing errors
# if only a tiny fragment (like a bird) triggers the sensor.
MIN_IMAGE_DIM_M = 1.0

# processing_queue: A thread-safe FIFO queue to pass data from the
# main Serial Reader thread to the Background Processor thread.
processing_queue = queue.Queue()


# =====================================================
# HELPER FUNCTION: ROBUST MEDIAN (MAD)
# =====================================================
def robust_median(distances):
    """
    Calculates a statistical median while ignoring outliers.
    Used during calibration to prevent moving objects (like a person walking by)
    from corrupting the background 'Zero Plane'.

    Uses 'Median Absolute Deviation' (MAD) logic:
    1. Calculate standard median.
    2. Find how much each point deviates from that median.
    3. Keep only points that are within 3x the typical deviation.
    """
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
# BACKGROUND PROCESSOR (WORKER THREAD)
# =====================================================
def background_processor():
    """
    This function runs in a parallel thread (Daemon).
    It waits for a complete vehicle's 3D data stack, cleans it, corrects rotation,
    and generates the final 2D image.

    Offloading this to a thread ensures the main loop never pauses reading from
    the serial port (preventing data loss/buffer overflow).
    """
    print("[INFO] Background Processor Started")
    while True:
        try:
            # Block and wait until a vehicle is placed in the queue
            extracted_stack, ts = processing_queue.get()

            # --- Validation Check ---
            # If the vehicle has too few points (noise), skip processing.
            if len(extracted_stack) < 30:
                print(f"[WARN] Vehicle {ts} Skipped: Too few points ({len(extracted_stack)})")
                processing_queue.task_done()
                continue

            # Create a unique folder for this vehicle event
            folder = f"vehicle_{ts}"
            os.makedirs(folder, exist_ok=True)

            # ---------------------------------------------------------
            # Step 1: Point Cloud Noise Removal
            # ---------------------------------------------------------
            # Create Open3D PointCloud object
            pcd = o3d.geometry.PointCloud()
            # Explicitly cast to float64 to prevent segmentation faults on ARM/Raspberry Pi CPUs
            pcd.points = o3d.utility.Vector3dVector(np.array(extracted_stack, dtype=np.float64))

            # Statistical Outlier Removal: Removes "dust" points that are far from neighbors
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=25, std_ratio=0.8)
            if len(pcd.points) == 0:
                print(f"[WARN] Vehicle {ts} Skipped: Empty after statistical cleanup")
                processing_queue.task_done()
                continue

            # Radius Outlier Removal: Removes isolated clusters that aren't dense enough
            pcd, _ = pcd.remove_radius_outlier(nb_points=12, radius=0.06)
            if len(pcd.points) == 0:
                print(f"[WARN] Vehicle {ts} Skipped: Empty after radius cleanup")
                processing_queue.task_done()
                continue

            # ---------------------------------------------------------
            # Step 2: Coordinate Transformation (Standardization)
            # ---------------------------------------------------------
            # Reorient axes to standard Side View:
            # Old Z (Time) -> Becomes X (Length of vehicle)
            # Old Y (Height) -> Becomes Y (Height of vehicle)
            # Old X (Depth) -> Becomes Z (Depth)
            R_standard = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=np.float64)
            pts_side = (R_standard @ np.asarray(pcd.points).T).T

            # ---------------------------------------------------------
            # Step 3: 3D Mounting Rotation Correction
            # ---------------------------------------------------------
            # Apply Pitch, Yaw, and Roll matrices to level the vehicle
            # based on the sensor's physical installation angle.
            rad_x = np.radians(ROTATION_X_DEG)
            rad_y = np.radians(ROTATION_Y_DEG)
            rad_z = np.radians(ROTATION_Z_DEG)

            # Rotation Matrix for X-axis (Pitch)
            Rx = np.array([
                [1, 0, 0],
                [0, np.cos(rad_x), -np.sin(rad_x)],
                [0, np.sin(rad_x), np.cos(rad_x)]
            ])

            # Rotation Matrix for Y-axis (Roll)
            Ry = np.array([
                [np.cos(rad_y), 0, np.sin(rad_y)],
                [0, 1, 0],
                [-np.sin(rad_y), 0, np.cos(rad_y)]
            ])

            # Rotation Matrix for Z-axis (Yaw)
            Rz = np.array([
                [np.cos(rad_z), -np.sin(rad_z), 0],
                [np.sin(rad_z), np.cos(rad_z), 0],
                [0, 0, 1]
            ])

            # Combine rotations: Z * Y * X order
            R_total = Rz @ Ry @ Rx

            # Apply rotation to all points in the cloud
            pts_side = (R_total @ pts_side.T).T

            # Save the corrected 3D Point Cloud file (.pcd) for debugging/visualization
            pcd_side = o3d.geometry.PointCloud()
            pcd_side.points = o3d.utility.Vector3dVector(pts_side)
            o3d.io.write_point_cloud(f"{folder}/side_view.pcd", pcd_side)

            # ---------------------------------------------------------
            # Step 4: Dynamic Frame Calculation (Cropping)
            # ---------------------------------------------------------
            # Find the minimum and maximum coordinates of the point cloud
            min_x, min_y = np.min(pts_side[:, :2], axis=0)
            max_x, max_y = np.max(pts_side[:, :2], axis=0)

            # Add padding margin
            min_x -= FRAME_PADDING_M
            max_x += FRAME_PADDING_M
            min_y -= FRAME_PADDING_M
            max_y += FRAME_PADDING_M

            # Enforce minimum image size constraints
            if (max_x - min_x) < MIN_IMAGE_DIM_M: max_x = min_x + MIN_IMAGE_DIM_M
            if (max_y - min_y) < MIN_IMAGE_DIM_M: max_y = min_y + MIN_IMAGE_DIM_M

            # Calculate the final image dimensions in pixels
            width_m = max_x - min_x
            height_m = max_y - min_y

            x_bins = int(width_m / GRID_RES)
            y_bins = int(height_m / GRID_RES)

            # ---------------------------------------------------------
            # Step 5: Image Generation (Projection)
            # ---------------------------------------------------------
            # Initialize empty black image
            bev = np.zeros((y_bins, x_bins), dtype=np.float32)

            # Map metric coordinates to pixel indices
            xi = ((pts_side[:, 0] - min_x) / GRID_RES).astype(np.int32)
            yi = ((pts_side[:, 1] - min_y) / GRID_RES).astype(np.int32)

            # Ensure indices stay within the image bounds
            valid_indices = (xi >= 0) & (xi < x_bins) & (yi >= 0) & (yi < y_bins)
            xi = xi[valid_indices]
            yi = yi[valid_indices]
            v_points = pts_side[valid_indices]

            if len(v_points) > 0:
                # Calculate depth intensity (Closer = Brighter)
                dist = np.sqrt(v_points[:, 0] ** 2 + v_points[:, 1] ** 2)
                intensity = 1.0 - np.minimum(dist / MAX_DIST_INTENSITY, 1.0)

                # Project points onto the 2D grid
                bev[yi, xi] = intensity

                # Post-processing: Convert to 8-bit image (0-255)
                bev_img = (bev * 255).astype(np.uint8)
                bev_img = np.flipud(bev_img)  # Flip vertically (Ground at bottom)
                bev_img = cv2.GaussianBlur(bev_img, (3, 3), 0)  # Smooth out gaps

                # Save final image
                img_path = f"{folder}/side_view_image.png"
                cv2.imwrite(img_path, bev_img)
                print(f"[SUCCESS] Vehicle {ts} Processed: {x_bins}x{y_bins} px -> {img_path}")
            else:
                print(f"[WARN] Vehicle {ts} Skipped: No points in generated frame")

            # Mark task as done in queue
            processing_queue.task_done()

        except Exception as e:
            print(f"[ERROR] In background processor: {e}")
            import traceback
            traceback.print_exc()
            processing_queue.task_done()


# =====================================================
# MAIN SYSTEM CLASS
# =====================================================
class TollPlazaSystem:
    def __init__(self):
        """
        Initializes the Serial Interface and system state variables.
        """
        try:
            # Open connection to the LiDAR sensor
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        except serial.SerialException as e:
            print(f"[CRITICAL] Serial Error: {e}")
            print("Check connection and permissions (sudo chmod 666 /dev/ttyUSB0)")
            exit(1)

        # 'background_matrix': Stores grid cells (x, y) that represent the static road.
        self.background_matrix = set()

        # 'is_stacking': State flag. True when a vehicle is currently passing.
        self.is_stacking = False

        # 'current_full_stack': Stores all 3D points for the current vehicle.
        self.current_full_stack = []

        # 'current_extracted': Stores only foreground (non-road) points.
        self.current_extracted = []

        # Timing variables for vehicle separation logic
        self.last_detection_time = time.time()
        self.start_capture_time = None

    def calibrate(self):
        """
        Runs on startup to learn the empty road environment.
        Captures 4000 frames and builds a statistical model of the background.
        """
        print(f"[INFO] Robust Zero Plane Training ({CALIBRATION_FRAMES} frames)...")
        os.makedirs("calibration", exist_ok=True)
        beam_history = {}
        frames = 0

        # --- Phase 1: Data Collection ---
        while frames < CALIBRATION_FRAMES:
            # Check for Sync Header (0xFC, 0xFD, 0xFE, 0xFF)
            if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
                # Read Payload Size
                size_bytes = self.ser.read(2)
                if len(size_bytes) < 2: continue
                size = struct.unpack('<H', size_bytes)[0]

                # Read Body
                body = self.ser.read(size)
                if len(body) < size: continue

                # Skip Checksum (2 bytes)
                self.ser.read(2)

                # Message ID 50011 indicates Measurement Data
                if len(body) > 2 and struct.unpack('<H', body[:2])[0] == 50011:
                    data = body[3:]
                    # Parse distances and store them per beam index
                    for i in range(len(data) // 2):
                        d = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                        if d > 0:
                            beam_history.setdefault(i, []).append(d)
                    frames += 1
                    if frames % 500 == 0:
                        print(f"    Frames: {frames}/{CALIBRATION_FRAMES}")

        # --- Phase 2: Statistical Processing ---
        clean_zero_pts = []
        idxs = sorted(beam_history.keys())

        for j, idx in enumerate(idxs):
            # Calculate Robust Median for this specific beam angle
            robust_mm = robust_median(beam_history[idx])
            if robust_mm is None: continue

            dist_m = robust_mm / 1000.0
            # Filter valid lane range
            if not (MIN_RANGE_M <= dist_m <= MAX_RANGE_M): continue

            # Convert Polar (Angle, Dist) to Cartesian (X, Y)
            angle = np.radians(START_ANGLE + idx * ANGULAR_RES)
            x, y = dist_m * np.cos(angle), dist_m * np.sin(angle)
            clean_zero_pts.append((x, y, 0.0))

            # Store this point's Grid Cell as "Background"
            ix, iy = int(np.floor(x / GRID_CELL_SIZE)), int(np.floor(y / GRID_CELL_SIZE))
            self.background_matrix.add((ix, iy))

        # Save Calibration Point Cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(clean_zero_pts))
        o3d.io.write_point_cloud("calibration/lzr_zero_plane.pcd", pcd)
        print(f"[SUCCESS] Robust Zero Plane Saved ({len(clean_zero_pts)} points)")

    def get_raw_frame(self):
        """
        Parses exactly one scan frame from the LiDAR serial stream.
        Returns: A list of [x, y] coordinates in meters.
        """
        # Sync Header Check
        if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
            size_bytes = self.ser.read(2)
            if len(size_bytes) < 2: return None
            size = struct.unpack('<H', size_bytes)[0]

            body = self.ser.read(size)
            if len(body) < size: return None

            self.ser.read(2)  # Checksum (Skipped)

            # Process Measurement Message (ID 50011)
            if len(body) > 2 and struct.unpack('<H', body[:2])[0] == 50011:
                data = body[3:]
                pts = []
                for i in range(len(data) // 2):
                    d_mm = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                    # Hardware Range Filter (0.1m to 3.5m)
                    if 100 < d_mm < 3500:
                        dist = d_mm / 1000.0
                        ang = np.radians(START_ANGLE + i * ANGULAR_RES)
                        # Polar to Cartesian
                        pts.append([dist * np.cos(ang), dist * np.sin(ang)])
                return pts
        return None

    def run_forever(self):
        """
        The main execution loop.
        1. Starts the background processing thread.
        2. Runs calibration.
        3. Loops infinitely to read sensor data and detect vehicles.
        """
        # Start the background processor thread (non-blocking)
        t = threading.Thread(target=background_processor, daemon=True)
        t.start()

        self.calibrate()
        print("[INFO] System Live - Waiting for vehicles...")

        while True:
            # Read one scan from sensor
            raw_xy = self.get_raw_frame()
            if not raw_xy: continue

            t_now = time.time()

            # Calculate Z-coordinate (Time Axis)
            # If we are currently recording a vehicle, Z increases based on speed.
            # If idle, Z is 0.
            z = (t_now - self.start_capture_time) * VEHICLE_SPEED_MPS if self.is_stacking else 0

            # Convert 2D frame to 3D slice
            raw_xyz = [[p[0], p[1], z] for p in raw_xy]

            # --- Background Subtraction ---
            extracted = []
            for p in raw_xy:
                # Convert point to grid coordinates
                ix, iy = int(np.floor(p[0] / GRID_CELL_SIZE)), int(np.floor(p[1] / GRID_CELL_SIZE))

                # Keep point only if it is NOT in the background matrix
                if (ix, iy) not in self.background_matrix:
                    extracted.append([p[0], p[1], z])

            # --- Vehicle State Machine ---

            # Case 1: Vehicle Detected (Trigger)
            if len(extracted) > TRIGGER_THRESHOLD:
                # If this is the first frame of a new vehicle:
                if not self.is_stacking:
                    self.is_stacking = True
                    self.start_capture_time = time.time()
                    self.current_full_stack = []
                    self.current_extracted = []
                    print("[EVENT] Vehicle Detected")

                # Accumulate data
                self.current_full_stack.extend(raw_xyz)
                self.current_extracted.extend(extracted)
                self.last_detection_time = time.time()

            # Case 2: Vehicle Passed (Timeout)
            # If we were recording, but haven't seen meaningful data for IDLE_TIMEOUT seconds:
            elif self.is_stacking and (time.time() - self.last_detection_time > IDLE_TIMEOUT):
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                print(f"[EVENT] Vehicle {ts} queued for processing ({len(self.current_extracted)} points)")

                # Send data copy to the background thread for processing
                processing_queue.put((list(self.current_extracted), ts))

                # Reset state for next vehicle
                self.is_stacking = False


if __name__ == "__main__":
    system = TollPlazaSystem()
    system.run_forever()
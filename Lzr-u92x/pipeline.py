import serial
import struct
import time
import os
import sys
import numpy as np
import open3d as o3d

# --- IMPORT YOUR ORIGINAL FILES ---
import stacked_reader  # Uses process_single_scan() logic
import extract_profile  # Uses perform_filtering() logic
import outlier_remover  # Uses remove_outliers() logic

# =====================================================
# CONFIGURATION
# =====================================================
SERIAL_PORT = 'COM4'
BAUD_RATE = 921600
ZERO_PLANE_FILE = "lzr_zero_plane_clean.pcd"
OUTPUT_FOLDER = "Extracted_Vehicles"

# Trigger Sensitivity
TRIGGER_THRESHOLD = 15  # Points differing from background to start
STOP_THRESHOLD = 5  # Points differing from background to stop
BUFFER_FRAMES = 10  # Frames to wait before saving


# =====================================================
# HELPER: LOAD BACKGROUND
# =====================================================
def load_background_grid(filename):
    """
    Uses the logic from 'extract_profile.py' to build the grid.
    We need this here to do the LIVE trigger check.
    """
    bg_matrix = set()
    print(f"Loading Background: {filename}")

    # Use extract_profile's reader to ensure compatibility
    points = extract_profile.read_pcd(filename)

    for p in points:
        # Use the same GRID_CELL_SIZE from your extract_profile file
        # We assume it is 0.10 based on your file
        ix = int(np.floor(p[0] / 0.10))
        iy = int(np.floor(p[1] / 0.10))
        bg_matrix.add((ix, iy))

    return bg_matrix


# =====================================================
# MAIN AUTOMATION LOOP
# =====================================================
def run_pipeline():
    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)

    # 1. Load Background for the Trigger
    if not os.path.exists(ZERO_PLANE_FILE):
        print("❌ Error: Zero plane file missing. Run zero_plane_recorder.py first.")
        return

    bg_matrix = load_background_grid(ZERO_PLANE_FILE)

    # 2. Open Serial Port
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"🚀 Pipeline Running on {SERIAL_PORT}. Waiting for vehicles...")
    except Exception as e:
        print(f"Error opening serial: {e}")
        return

    # State Variables
    is_recording = False
    vehicle_points = []
    stop_counter = 0
    start_time = 0

    while True:
        try:
            # --- A. READ DATA ---
            # We must handle the raw reading here to feed it to stacked_reader
            if ser.read(1) == b'\xfc' and ser.read(3) == b'\xfd\xfe\xff':
                size = struct.unpack('<H', ser.read(2))[0]
                body = ser.read(size)
                ser.read(2)  # Checksum

                cmd = struct.unpack('<H', body[:2])[0]

                if cmd == 50011:
                    # --- B. PROCESS FRAME ---
                    # 1. Get 2D slice for Trigger Check (Z=0)
                    # We use your stacked_reader's logic to parse bytes!
                    current_slice = stacked_reader.process_single_scan(body[3:], 0.0)

                    # 2. Check Distortion (Trigger)
                    active_count = 0
                    for p in current_slice:
                        ix = int(np.floor(p[0] / 0.10))
                        iy = int(np.floor(p[1] / 0.10))
                        if (ix, iy) not in bg_matrix:
                            active_count += 1

                    # --- C. STATE MACHINE ---

                    # CASE 1: Start Recording
                    if not is_recording and active_count > TRIGGER_THRESHOLD:
                        print("\n🚗 Trigger! Vehicle Detected.")
                        is_recording = True
                        start_time = time.time()
                        vehicle_points = []
                        stop_counter = 0

                    # CASE 2: Currently Recording
                    if is_recording:
                        # Calculate Speed-Corrected Z
                        elapsed = time.time() - start_time
                        # Use stacked_reader logic again with TRUE Z
                        # assuming speed 10 km/h (2.77 m/s)
                        z_val = elapsed * 2.77

                        real_points = stacked_reader.process_single_scan(body[3:], z_val)
                        vehicle_points.extend(real_points)

                        # Check if vehicle has left
                        if active_count < STOP_THRESHOLD:
                            stop_counter += 1
                            if stop_counter >= BUFFER_FRAMES:
                                print("✅ Vehicle Passed. Processing...")
                                process_vehicle(vehicle_points)
                                is_recording = False  # Reset
                                print("🟢 Ready for next vehicle.")
                        else:
                            stop_counter = 0  # Vehicle still here

        except KeyboardInterrupt:
            print("Stopping...")
            break


# =====================================================
# THE PROCESSING CHAIN
# =====================================================
def process_vehicle(points):
    """
    Saves the raw stack, runs Extraction, then runs Cleaning.
    """
    timestamp = time.strftime("%Y%m%d_%H%M%S")

    # 1. Save RAW Stack
    raw_filename = f"{OUTPUT_FOLDER}/Raw_{timestamp}.pcd"
    extract_profile.save_pcd(points, raw_filename)  # Use your existing save function

    # 2. Run EXTRACTION (Extract profile.py)
    # logic: background subtraction
    extracted_filename = f"{OUTPUT_FOLDER}/Extracted_{timestamp}.pcd"
    print("   -> Running Extraction...")

    # We call the function from your file!
    extract_profile.perform_filtering(ZERO_PLANE_FILE, raw_filename, extracted_filename)

    # 3. Run CLEANING (Outlier Remover.py)
    print("   -> Running Outlier Removal...")
    outlier_remover.remove_outliers(extracted_filename)

    # Cleanup (Optional: Delete the raw file to save space)
    if os.path.exists(raw_filename):
        os.remove(raw_filename)

    print(f"✨ Complete! Saved Clean Vehicle: {extracted_filename.replace('.pcd', '_cleaned.pcd')}")


if __name__ == "__main__":
    run_pipeline()
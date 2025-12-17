import serial
import time
import numpy as np
import datetime
import os
import signal
import sys


SERIAL_PORT = 'COM4'  # Change to your port
BAUD_RATE = 460800  # Standard LZR U921 baud rate
POINTS_PER_SCAN = 274  # Standard LZR resolution
FOV_DEGREES = 96.0  # Field of View
SAVE_DIRECTORY = "./scans"  # Folder where files will be saved

# MOVEMENT SIMULATION
# We assume a base speed, but we will PAUSE if the truck stops.
Z_INCREMENT_METERS = 0.05

# STOP DETECTION SENSITIVITY
# REDUCED THRESHOLD: 5.0mm
# The truck must be extremely still to trigger a pause.
# Even slow creeping will now be recorded as movement.
STOP_THRESHOLD_MM = 5.0

# Global buffers
all_points_buffer = []
scan_counter = 0
previous_distances = None  # To remember the last frame


# ==========================================
# PCD SAVING FUNCTION (COMBINED)
# ==========================================
def save_combined_pcd(points_list, output_folder):
    """
    Saves the accumulated list of (x, y, z) points into a single PCD file.
    """
    if not points_list:
        print("[WARNING] No points to save.")
        return

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_folder, f"lzr_smart_3d_{timestamp}.pcd")

    num_points = len(points_list)

    print(f"\n[SAVING] Writing {num_points} points to {filename}...")

    with open(filename, 'w') as f:
        # PCD Header
        f.write("# .PCD v.7 - LZR Smart Scan\n")
        f.write("VERSION .7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")
        for p in points_list:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print(f"[COMPLETE] File saved successfully.")


# ==========================================
# SMART PROCESSING FUNCTION
# ==========================================
def process_scan(distances_mm):
    global scan_counter, previous_distances

    # --- 1. STOP DETECTION LOGIC ---
    # Convert to numpy array for fast math
    current_dist = np.array(distances_mm)

    is_stopped = False

    if previous_distances is not None:
        # Calculate absolute difference between this frame and last frame
        diff = np.abs(current_dist - previous_distances)
        avg_change = np.mean(diff)

        # If the scene barely changed (less than 5mm), the truck is stopped
        if avg_change < STOP_THRESHOLD_MM:
            is_stopped = True
            # Optional: Uncomment to see when it pauses
            # sys.stdout.write(f"\r[PAUSED] Delta: {avg_change:.1f}mm < {STOP_THRESHOLD_MM}mm   ")
            # sys.stdout.flush()

    # Update history
    previous_distances = current_dist

    if is_stopped:
        return  # EXIT FUNCTION: Do not add points, do not increment Z

    # --- 2. STANDARD PROCESSING (If Moving) ---

    # Calculate Z (Length) based on counter
    current_length_z = scan_counter * Z_INCREMENT_METERS

    angles = np.linspace(
        np.radians(-FOV_DEGREES / 2),
        np.radians(FOV_DEGREES / 2),
        len(distances_mm)
    )

    for r, theta in zip(distances_mm, angles):
        r_meters = r / 1000.0

        if 0.1 < r_meters < 60.0:
            # POLAR TO CARTESIAN
            raw_x = r_meters * np.cos(theta)
            raw_y = r_meters * np.sin(theta)

            # SIDEWAYS MAPPING (ROTATION FIX)
            # X = Length (Time)
            # Y = Width (Distance)
            # Z = Height (Scan Angle)

            x_world = current_length_z
            y_world = abs(raw_x)
            z_world = raw_y

            all_points_buffer.append((x_world, y_world, z_world))

    scan_counter += 1

    if scan_counter % 10 == 0:
        sys.stdout.write(f"\r[STATUS] Scanning... Points: {len(all_points_buffer)} | Length: {current_length_z:.2f}m")
        sys.stdout.flush()


# ==========================================
# MAIN LOOP
# ==========================================
def run_recorder():
    ser = None
    print("--- LZR U921 SMART RECORDER ---")
    print(f"Stop Threshold: {STOP_THRESHOLD_MM} mm (High Sensitivity)")
    print("Feature: Sideways Rotation Fix applied.")
    print("Press Ctrl+C to save and exit.\n")

    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}. Waiting for data...")
        buffer = b""

        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer += chunk

                start_index = buffer.find(b'\x02')
                if start_index != -1:
                    buffer = buffer[start_index:]
                    EXPECTED_SIZE = (POINTS_PER_SCAN * 2) + 10

                    if len(buffer) >= EXPECTED_SIZE:
                        raw_payload = buffer[4: 4 + (POINTS_PER_SCAN * 2)]
                        distances = []
                        for i in range(0, len(raw_payload) - 1, 2):
                            high = raw_payload[i]
                            low = raw_payload[i + 1]
                            dist = (high << 8) + low
                            distances.append(dist)

                        if len(distances) == POINTS_PER_SCAN:
                            process_scan(distances)

                        buffer = buffer[EXPECTED_SIZE:]
            time.sleep(0.005)

    except serial.SerialException:
        print(f"[ERROR] Could not open {SERIAL_PORT}.")
    except KeyboardInterrupt:
        print("\n\n[STOP] Recording stopped.")
    finally:
        if ser and ser.is_open:
            ser.close()
        save_combined_pcd(all_points_buffer, SAVE_DIRECTORY)


if __name__ == "__main__":
    run_recorder()
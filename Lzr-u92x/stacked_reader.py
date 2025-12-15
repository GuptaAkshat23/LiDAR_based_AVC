import serial
import time
import numpy as np
import datetime
import os
import signal
import sys

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM4'  # Change to your port
BAUD_RATE = 460800  # Standard LZR U921 baud rate
POINTS_PER_SCAN = 274  # Standard LZR resolution
FOV_DEGREES = 96.0  # Field of View
SAVE_DIRECTORY = "./scans"  # Folder where files will be saved

# 3D VISUALIZATION SETTINGS
# How much to move the Z-axis for every scan frame (simulating movement)
# 0.05 = 5cm per scan. Adjust this based on how fast you move the sensor.
Z_INCREMENT_METERS = 0.05

# Global buffer to hold all points until we save
all_points_buffer = []
scan_counter = 0


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
    filename = os.path.join(output_folder, f"lzr_combined_3d_{timestamp}.pcd")

    num_points = len(points_list)

    print(f"\n[SAVING] Writing {num_points} points to {filename}...")

    with open(filename, 'w') as f:
        # PCD Header
        f.write("# .PCD v.7 - LZR U921 Combined 3D Scan\n")
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

        # PCD Data
        for p in points_list:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print(f"[COMPLETE] File saved successfully.")


# ==========================================
# PROCESSING FUNCTION
# ==========================================
def process_scan(distances_mm):
    global scan_counter

    # Calculate Z height for this specific slice
    current_z = scan_counter * Z_INCREMENT_METERS

    # Create angles array
    angles = np.linspace(
        np.radians(-FOV_DEGREES / 2),
        np.radians(FOV_DEGREES / 2),
        len(distances_mm)
    )

    valid_frame_points = 0

    for r, theta in zip(distances_mm, angles):
        r_meters = r / 1000.0

        # Filter valid range (adjust as needed)
        if 0.1 < r_meters < 60.0:
            x = r_meters * np.cos(theta)
            y = r_meters * np.sin(theta)
            z = current_z  # Use the incremented Z height

            all_points_buffer.append((x, y, z))
            valid_frame_points += 1

    scan_counter += 1
    # Print status every 10 scans so we know it's working
    if scan_counter % 10 == 0:
        print(f"Captured Scan #{scan_counter} | Total Points: {len(all_points_buffer)}")


# ==========================================
# MAIN READER LOOP
# ==========================================
def run_recorder():
    global scan_counter
    ser = None

    print("--- LZR U921 3D RECORDER ---")
    print(f"Z-Step per scan: {Z_INCREMENT_METERS} meters")
    print("Press Ctrl+C to stop recording and save the file.\n")

    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}. Waiting for data...")

        buffer = b""

        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer += chunk

                # --- PARSING LOGIC ---
                # Find start byte (Assuming 0x02 STX based on your previous code)
                start_index = buffer.find(b'\x02')

                if start_index != -1:
                    buffer = buffer[start_index:]

                    # Expected size: Header + Data (274*2) + Checksum/Footer
                    # Adjust '10' if your overhead is different
                    EXPECTED_SIZE = (POINTS_PER_SCAN * 2) + 10

                    if len(buffer) >= EXPECTED_SIZE:
                        # Extract payload (assuming offset 4)
                        raw_payload = buffer[4: 4 + (POINTS_PER_SCAN * 2)]

                        distances = []
                        for i in range(0, len(raw_payload) - 1, 2):
                            high = raw_payload[i]
                            low = raw_payload[i + 1]
                            dist = (high << 8) + low
                            distances.append(dist)

                        # Accumulate points if valid frame
                        if len(distances) == POINTS_PER_SCAN:
                            process_scan(distances)

                        # Clear processed buffer
                        buffer = buffer[EXPECTED_SIZE:]

            # Small sleep to prevent CPU hogging
            time.sleep(0.005)

    except serial.SerialException:
        print(f"[ERROR] Could not open {SERIAL_PORT}.")
    except KeyboardInterrupt:
        print("\n[STOP] Recording stopped by user.")
    finally:
        if ser and ser.is_open:
            ser.close()
        # SAVE THE FILE ON EXIT
        save_combined_pcd(all_points_buffer, SAVE_DIRECTORY)


if __name__ == "__main__":
    run_recorder()
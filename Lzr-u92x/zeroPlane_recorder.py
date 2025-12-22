import serial
import time
import numpy as np
import datetime
import os
import sys

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM5'  # <--- Make sure this matches the Port Finder result (COM4 or COM5)
BAUD_RATE = 460800  # <--- WARNING: Auto-Tuner found 921600. Change this if you see garbage.
POINTS_PER_SCAN = 274
FOV_DEGREES = 96.0
SAVE_DIRECTORY = "./scans"

# 3D SETTINGS
Z_INCREMENT_METERS = 0.05

# Buffers
all_points_buffer = []
scan_counter = 0


def save_combined_pcd(points_list, output_folder):
    if not points_list:
        print("\n[WARNING] No points to save.")
        return

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_folder, f"lzr_sideways_3d_{timestamp}.pcd")
    num_points = len(points_list)

    print(f"\n\n[SAVING] Writing {num_points} points to {filename}...")

    with open(filename, 'w') as f:
        f.write("# .PCD v.7 - LZR U921 Sideways Scan\n")
        f.write("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {num_points}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {num_points}\nDATA ascii\n")
        for p in points_list:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print(f"[COMPLETE] File saved successfully.")


def process_scan(distances_mm):
    global scan_counter
    current_length = scan_counter * Z_INCREMENT_METERS
    angles = np.linspace(np.radians(-FOV_DEGREES / 2), np.radians(FOV_DEGREES / 2), len(distances_mm))

    for r, theta in zip(distances_mm, angles):
        r_meters = r / 1000.0
        # Filter range: 0.1m to 60m
        if 0.1 < r_meters < 60.0:
            raw_x = r_meters * np.cos(theta)
            raw_y = r_meters * np.sin(theta)

            # Mapping: X=Time, Y=Width, Z=Height
            x_world = current_length
            y_world = abs(raw_x)
            z_world = raw_y

            all_points_buffer.append((x_world, y_world, z_world))

    scan_counter += 1


def run_recorder():
    global scan_counter
    print("--- LZR U921 RECORDER (WITH LIVE ANALYTICS) ---")
    print(f"Target: {SERIAL_PORT} @ {BAUD_RATE}")
    print("Press Ctrl+C to stop and save.\n")

    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
        print("Connected. Waiting for stream...")

        buffer = b""

        # ANALYTICS VARIABLES
        start_time = time.time()
        byte_counter = 0
        last_print = time.time()

        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer += chunk
                byte_counter += len(chunk)

                # --- PARSING ---
                start_index = buffer.find(b'\x02')
                if start_index != -1:
                    buffer = buffer[start_index:]
                    EXPECTED_SIZE = (POINTS_PER_SCAN * 2) + 10  # Approx frame size

                    if len(buffer) >= EXPECTED_SIZE:
                        # Extract Payload
                        raw_payload = buffer[4: 4 + (POINTS_PER_SCAN * 2)]

                        # Verify integrity (simple check)
                        if len(raw_payload) == (POINTS_PER_SCAN * 2):
                            distances = []
                            # Big Endian Decoding
                            for i in range(0, len(raw_payload) - 1, 2):
                                high = raw_payload[i]
                                low = raw_payload[i + 1]
                                dist = (high << 8) + low
                                distances.append(dist)

                            process_scan(distances)

                        # Move buffer forward
                        buffer = buffer[EXPECTED_SIZE:]

            # --- LIVE DASHBOARD (Updates every 0.2s) ---
            if time.time() - last_print > 0.2:
                elapsed = time.time() - start_time
                if elapsed > 0:
                    fps = scan_counter / elapsed
                    kbps = (byte_counter / 1024) / elapsed

                    # \r overwrites the current line
                    sys.stdout.write(
                        f"\r[REC] Frames: {scan_counter} | "
                        f"Points: {len(all_points_buffer)} | "
                        f"FPS: {fps:.1f} | "
                        f"Data: {kbps:.1f} KB/s   "
                    )
                    sys.stdout.flush()
                    last_print = time.time()

            time.sleep(0.001)

    except serial.SerialException:
        print(f"\n[ERROR] Could not open {SERIAL_PORT}.")
    except KeyboardInterrupt:
        print("\n\n[STOP] Recording stopped by user.")
    finally:
        if 'ser' in locals() and ser.is_open: ser.close()
        save_combined_pcd(all_points_buffer, SAVE_DIRECTORY)


if __name__ == "__main__":
    run_recorder()
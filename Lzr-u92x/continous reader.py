import serial
import struct
import numpy as np
import time
import datetime
import os

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM5'  # Change to your LZR port
BAUD_RATE = 921600  # LZR standard high speed
SAVE_DIRECTORY = "./scans"

# --- 3D STACKING SETTINGS ---
# Distance moved per single scan (e.g., 0.05m = 5cm)
# Unlike the 360 scanner, LZR scans very fast (60Hz).
# If moving slowly (walking), try 0.01 or 0.005.
STEP_INCREMENT_M = 0.01

all_points_buffer = []
scan_counter = 0


def save_combined_pcd(points_list, output_folder):
    if not points_list:
        print("[WARNING] No points to save.")
        return
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_folder, f"lzr_step_stacked_{timestamp}.pcd")

    num_points = len(points_list)
    print(f"\n[SAVING] Writing {num_points} points to {filename}...")

    with open(filename, 'w') as f:
        # Standard PCD Header
        f.write("# .PCD v.7 - LZR Step Stacked Scan\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {num_points}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {num_points}\nDATA ascii\n")
        for p in points_list:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
    print(f"[COMPLETE] File saved.")


def parse_lzr_packet(data_bytes, current_travel_x):
    """
    Parses LZR-U921 MDI message and returns points.
    Unlike TG30, LZR sends the full scan in one packet.
    """
    points = []
    num_values = len(data_bytes) // 2

    # LZR-U921 Angular Parameters
    start_angle = -48.0
    angular_res = 0.3516

    for i in range(num_values):
        # Read distance (Little Endian uint16)
        dist_mm = struct.unpack('<H', data_bytes[i * 2: i * 2 + 2])[0]

        # Filter noise/error
        if dist_mm < 10 or dist_mm > 6000:
            continue

        # Convert Polar -> Cartesian (Sensor Slice)
        angle_rad = np.radians(start_angle + (i * angular_res))
        dist_m = dist_mm / 1000.0

        # MAPPING:
        # Y, Z = The Sensor's 2D Slice
        # X    = The Travel Axis (determined by Scan Counter)

        y_sens = dist_m * np.cos(angle_rad)
        z_sens = dist_m * np.sin(angle_rad)

        # Add to list with the calculated Travel Position (X)
        points.append((current_travel_x, y_sens, z_sens))

    return points


def run_recorder():
    global scan_counter
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.reset_input_buffer()

        print(f"🚀 LZR-U921 Step-Based Recorder Started...")
        print(f"👉 Incrementing X by {STEP_INCREMENT_M}m per scan.")
        print("Press Ctrl+C to stop and save.")

        while True:
            # 1. Look for SYNC (FC FD FE FF)
            if ser.read(1) == b'\xfc':
                if ser.read(3) == b'\xfd\xfe\xff':

                    # 2. Read SIZE
                    size_bytes = ser.read(2)
                    if len(size_bytes) < 2: continue
                    msg_size = struct.unpack('<H', size_bytes)[0]

                    # 3. Read BODY
                    msg_body = ser.read(msg_size)
                    if len(msg_body) < msg_size: continue

                    # 4. Read Checksum
                    ser.read(2)

                    # 5. Process MDI Command (50011)
                    if len(msg_body) > 2:
                        cmd = struct.unpack('<H', msg_body[0:2])[0]
                        if cmd == 50011:
                            # --- LOGIC ADAPTATION ---
                            # In TG30, we waited for angle wrap.
                            # In LZR, receiving this message means 1 full scan is complete.

                            scan_counter += 1

                            # Calculate Travel Position (X)
                            current_travel_x = scan_counter * STEP_INCREMENT_M

                            # Extract Data (Skip CMD + PlaneID if present)
                            # Standard offset is 3 (CMD(2) + Plane(1))
                            raw_data = msg_body[3:]

                            new_points = parse_lzr_packet(raw_data, current_travel_x)
                            all_points_buffer.extend(new_points)

                            # Status update every 10 scans
                            if scan_counter % 10 == 0:
                                print(
                                    f"Scan Count: {scan_counter} | Travel: {current_travel_x:.2f}m | Total Points: {len(all_points_buffer)}")

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
        save_combined_pcd(all_points_buffer, SAVE_DIRECTORY)


if __name__ == "__main__":
    run_recorder()
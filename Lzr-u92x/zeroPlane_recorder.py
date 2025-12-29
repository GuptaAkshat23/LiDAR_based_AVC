import serial
import struct
import numpy as np
import time
import statistics
import os
import sys

# --- CONFIGURATION ---
SERIAL_PORT = 'COM4'  # Adjust this to your actual port
BAUD_RATE = 921600  # Standard LZR Baud
OUTPUT_FILENAME = "lzr_zero_plane.pcd"

# --- LZR SPECIFIC CONSTANTS ---
START_ANGLE = -48.0  # Degrees
ANGULAR_RES = 0.3516  # Degrees per step
SYNC_HEADER = b'\xfc\xfd\xfe\xff'

# --- GLOBAL STORAGE ---
# Stores lists of distances for each beam index
# Format: { beam_index_0: [d1, d2...], beam_index_1: [d1, d2...] }
beam_history = {}


def save_statistical_pcd(history, filename):
    """
    Calculates the MEDIAN distance for each beam index to create
    a noise-free background model and saves it to PCD.
    """
    if not history:
        print("❌ No data collected.")
        return

    print(f"\n\n🛑 Stopping... Analyzing data from {len(history)} beams...")
    print("   Computing Median (Statistical Background Model)... please wait.")

    clean_points = []

    # Sort by index to keep points ordered in the file
    sorted_indices = sorted(history.keys())

    for idx in sorted_indices:
        dist_list = history[idx]

        if dist_list:
            # 1. Compute Median Distance (Removes moving objects/noise)
            median_dist_mm = statistics.median(dist_list)

            # 2. Convert to Coordinates (Polar -> Cartesian)
            # Only keep points that are valid (e.g., > 0)
            if median_dist_mm > 0:
                angle_deg = START_ANGLE + (idx * ANGULAR_RES)
                angle_rad = np.radians(angle_deg)

                # Convert mm to meters
                dist_m = median_dist_mm / 1000.0

                x = dist_m * np.cos(angle_rad)
                y = dist_m * np.sin(angle_rad)

                # Z is always 0 for the zero plane
                clean_points.append((x, y, 0.0))

    if not clean_points:
        print("❌ Error: No valid points computed.")
        return

    # 3. Write PCD File
    header = (
        "# .PCD v0.7 - Zero Plane Data\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(clean_points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(clean_points)}\n"
        "DATA ascii\n"
    )

    try:
        with open(filename, 'w') as f:
            f.write(header)
            for p in clean_points:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
        print(f"✅ SUCCESS: Saved '{filename}' with {len(clean_points)} static points.")
    except Exception as e:
        print(f"❌ File Write Error: {e}")


def process_raw_data(data_bytes):
    """Parses raw bytes and adds distances to history bins."""
    # Based on your provided code: "Standard MDI starts after CMD... and optional Plane Number"
    # data_bytes input here should be the RAW distance data (msg_body[3:])

    num_values = len(data_bytes) // 2

    for i in range(num_values):
        # Read 2 bytes as unsigned short (distance in mm)
        dist_mm = struct.unpack('<H', data_bytes[i * 2: i * 2 + 2])[0]

        # Add to history for this specific beam index
        if i not in beam_history:
            beam_history[i] = []

        # Filter out 0 or error codes if necessary (usually 0 is no return)
        if dist_mm > 0:
            beam_history[i].append(dist_mm)


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"🚀 Connected to LZR-U921 on {SERIAL_PORT}")
        print("💡 Recording background... Keep the area clear if possible.")
        print("⌨️  Press CTRL+C to STOP and save the Zero Plane.")

        scan_count = 0

        while True:
            # 1. Look for Sync Header
            # We read byte by byte to ensure we don't miss alignment
            if ser.read(1) == b'\xfc':
                if ser.read(3) == b'\xfd\xfe\xff':

                    # 2. Read Message Size
                    size_bytes = ser.read(2)
                    if len(size_bytes) < 2: continue
                    msg_size = struct.unpack('<H', size_bytes)[0]

                    # 3. Read Message Body
                    msg_body = ser.read(msg_size)
                    if len(msg_body) < msg_size: continue

                    # 4. Read Footer (Checksum) - discard
                    ser.read(2)

                    # 5. Process Command 50011 (MDI)
                    cmd = struct.unpack('<H', msg_body[0:2])[0]
                    if cmd == 50011:
                        # Extract distance data
                        # [0:2] = CMD
                        # [2] = Plane Number (Optional, usually 1 byte)
                        # [3:] = Distance Data
                        raw_distances = msg_body[3:]

                        process_raw_data(raw_distances)

                        scan_count += 1
                        if scan_count % 10 == 0:
                            sys.stdout.write(f"\r[Recording] Scans Captured: {scan_count}")
                            sys.stdout.flush()

    except KeyboardInterrupt:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        # On CTRL+C, we trigger the save
        save_statistical_pcd(beam_history, OUTPUT_FILENAME)

    except Exception as e:
        print(f"\n❌ Error: {e}")
        if 'ser' in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
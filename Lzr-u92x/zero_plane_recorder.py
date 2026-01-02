import serial
import struct
import numpy as np
import time
import statistics
import os
import sys

# --- CONFIGURATION ---
SERIAL_PORT = 'COM4'
BAUD_RATE = 921600
OUTPUT_FILENAME = "lzr_zero_plane_clean.pcd"

# --- LZR U921 CONSTANTS ---
START_ANGLE = -48.0
ANGULAR_RES = 0.3516
SYNC_HEADER = b'\xfc\xfd\xfe\xff'

# --- FILTERING LIMITS (VERY IMPORTANT) ---
MIN_RANGE_M = 0.10     # Ignore anything closer than 10 cm
MAX_RANGE_M = 3.00     # Your box max dimension (adjust if needed)
MAX_NEIGHBOR_JUMP = 0.15  # 15 cm allowed jump between beams

beam_history = {}


# ----------------------------------------------------
# ROBUST STATISTICS (MAD FILTER)
# ----------------------------------------------------
def robust_median(distances):
    """
    Removes outliers using Median Absolute Deviation (MAD)
    """
    if len(distances) < 5:
        return None

    median = statistics.median(distances)
    deviations = [abs(d - median) for d in distances]
    mad = statistics.median(deviations)

    if mad == 0:
        return median

    # Keep points within 3 * MAD
    filtered = [d for d in distances if abs(d - median) <= 3 * mad]

    if not filtered:
        return None

    return statistics.median(filtered)


# ----------------------------------------------------
# SAVE CLEAN ZERO PLANE
# ----------------------------------------------------
def save_statistical_pcd(history, filename):
    print("\n🛑 Processing Zero Plane with Outlier Removal...")

    beam_points = {}

    # STEP 1: Robust median per beam
    for idx in sorted(history.keys()):
        dlist = history[idx]
        robust_mm = robust_median(dlist)

        if robust_mm is None:
            continue

        dist_m = robust_mm / 1000.0

        if dist_m < MIN_RANGE_M or dist_m > MAX_RANGE_M:
            continue

        angle_deg = START_ANGLE + idx * ANGULAR_RES
        angle_rad = np.radians(angle_deg)

        x = dist_m * np.cos(angle_rad)
        y = dist_m * np.sin(angle_rad)

        beam_points[idx] = (x, y, 0.0)

    # STEP 2: Neighbor consistency filtering
    clean_points = []
    sorted_idx = sorted(beam_points.keys())

    for i, idx in enumerate(sorted_idx):
        x, y, z = beam_points[idx]

        if i > 0 and i < len(sorted_idx) - 1:
            prev_dist = np.linalg.norm(beam_points[sorted_idx[i - 1]][:2])
            curr_dist = np.linalg.norm((x, y))
            next_dist = np.linalg.norm(beam_points[sorted_idx[i + 1]][:2])

            if (abs(curr_dist - prev_dist) > MAX_NEIGHBOR_JUMP and
                abs(curr_dist - next_dist) > MAX_NEIGHBOR_JUMP):
                continue  # isolated spike → remove

        clean_points.append((x, y, z))

    if not clean_points:
        print("❌ No clean points generated.")
        return

    # STEP 3: Write PCD
    header = (
        "# .PCD v0.7 - LZR U921 Clean Zero Plane\n"
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

    with open(filename, "w") as f:
        f.write(header)
        for p in clean_points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print(f"✅ CLEAN ZERO PLANE SAVED → {filename}")
    print(f"📊 Final Points: {len(clean_points)}")


# ----------------------------------------------------
# RAW DATA PARSING
# ----------------------------------------------------
def process_raw_data(data_bytes):
    num_values = len(data_bytes) // 2

    for i in range(num_values):
        dist_mm = struct.unpack('<H', data_bytes[i*2:i*2+2])[0]

        if dist_mm == 0:
            continue

        beam_history.setdefault(i, []).append(dist_mm)


# ----------------------------------------------------
# MAIN LOOP
# ----------------------------------------------------
def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("🚀 U921 Connected")
        print("📦 Recording zero plane (press CTRL+C to stop)")

        scan_count = 0

        while True:
            if ser.read(1) == b'\xfc':
                if ser.read(3) == b'\xfd\xfe\xff':

                    size = struct.unpack('<H', ser.read(2))[0]
                    body = ser.read(size)
                    ser.read(2)

                    cmd = struct.unpack('<H', body[:2])[0]

                    if cmd == 50011:
                        process_raw_data(body[3:])
                        scan_count += 1

                        if scan_count % 10 == 0:
                            sys.stdout.write(f"\rScans: {scan_count}")
                            sys.stdout.flush()

    except KeyboardInterrupt:
        ser.close()
        save_statistical_pcd(beam_history, OUTPUT_FILENAME)

    except Exception as e:
        print("❌ Error:", e)


if __name__ == "__main__":
    main()
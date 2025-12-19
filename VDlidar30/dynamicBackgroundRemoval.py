import serial
import math
import time
import struct
import sys
import os

# --- CONFIGURATION ---
PORT = 'COM3'  # <--- CHECK YOUR PORT
BAUD = 512000
BG_FILE = "scans/final_zero_plane.pcd"
OUTPUT_FOLDER = "scans_combined"

# -- ROBUSTNESS SETTINGS --
ANGLE_BIN_SIZE = 0.5  # 0.5 degree bins (720 total bins)
SEARCH_WINDOW = 2  # Check +/- 2 bins (approx +/- 1 degree) for jitter
NOISE_TOLERANCE = 0.10  # 10cm Tolerance (Anything closer than 10cm to wall is ignored)
MIN_DIST_M = 0.20  # Ignore points closer than 20cm to lidar
VOXEL_SIZE = 0.01  # 1cm grid snapping

# Global storage
background_bins = {}  # Format: { bin_index: distance_meters }
dynamic_points_set = set()


def get_bin_index(angle):
    """Converts angle to discrete bin index."""
    return int(angle / ANGLE_BIN_SIZE)


def load_background_model(filename):
    """Parses PCD to create a robust lookup table."""
    if not os.path.exists(filename):
        print(f"❌ Error: File '{filename}' not found!")
        return False

    print(f"📂 Loading Background: {filename}...")
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()

        is_data = False
        count = 0
        for line in lines:
            if line.startswith("DATA ascii"):
                is_data = True
                continue
            if not is_data: continue

            parts = line.strip().split()
            if len(parts) >= 2:
                x = float(parts[0])
                y = float(parts[1])

                # Convert to Polar
                dist_m = math.sqrt(x * x + y * y)
                angle_rad = math.atan2(y, x)
                angle_deg = math.degrees(angle_rad)
                if angle_deg < 0: angle_deg += 360

                # Store in bin (Keep the max distance found in this bin to represent the wall)
                b_idx = get_bin_index(angle_deg)
                if b_idx not in background_bins:
                    background_bins[b_idx] = dist_m
                else:
                    # If multiple points land in one bin, take the average or max
                    background_bins[b_idx] = (background_bins[b_idx] + dist_m) / 2
                count += 1

        print(f"✅ Background Loaded! Bins filled: {len(background_bins)}")
        return True
    except Exception as e:
        print(f"❌ Error parsing background: {e}")
        return False


def is_background(angle, dist_m):
    """
    Checks if the point belongs to the background using a Sliding Window.
    Returns True if the point is likely part of the wall.
    """
    center_bin = get_bin_index(angle)

    # Check neighbors (Handle wrap-around 0-360)
    # We look for the "closest match" in the background
    min_diff = 999.0

    for i in range(-SEARCH_WINDOW, SEARCH_WINDOW + 1):
        check_bin = center_bin + i

        # Handle 0-360 wrap
        if check_bin < 0: check_bin += 720
        if check_bin >= 720: check_bin -= 720

        bg_dist = background_bins.get(check_bin)

        if bg_dist:
            # Difference between Live Point and this Background Bin
            diff = dist_m - bg_dist

            # We only care if the point is BEHIND or ON the wall
            # If dist_m is 2.0m and bg is 2.0m -> diff is 0 (Background)
            # If dist_m is 1.0m and bg is 2.0m -> diff is -1.0 (Object)

            # We want to find the BEST fit.
            if abs(diff) < abs(min_diff):
                min_diff = diff

    # If the point is within tolerance of the wall (or behind it), it's background
    # Logic: If min_diff is > -0.10, it means it's not significantly closer than the wall
    if min_diff > -NOISE_TOLERANCE:
        return True  # It is the wall (or noise behind wall)

    return False  # It is significantly closer -> OBJECT


def save_combined_file(unique_points):
    """Saves the set of unique voxel points."""
    if not unique_points:
        print("\n⚠️ No object detected.")
        return

    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(OUTPUT_FOLDER, f"Object_{timestamp}.pcd")

    print(f"\n\n💾 Saving Object Scan: {filename}...")
    points_list = list(unique_points)

    # Simple Despeckling: Remove points with no neighbors (Optional optimization)
    # For now, we save everything to ensure we don't delete the truck.

    header = f"""# .PCD v0.7 - Object Scan
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {len(points_list)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points_list)}
DATA ascii
"""
    with open(filename, 'w') as f:
        f.write(header)
        for p in points_list:
            f.write(f"{p[0]:.4f} {p[1]:.4f} 0.000\n")

    print(f"✅ Saved {len(points_list)} points.")


def parse_packet(packet):
    try:
        lsn = packet[3]
        fsa = struct.unpack('<H', packet[4:6])[0]
        lsa = struct.unpack('<H', packet[6:8])[0]
        angle_fsa = (fsa >> 1) / 64.0
        angle_lsa = (lsa >> 1) / 64.0
        diff = angle_lsa - angle_fsa
        if diff < 0: diff += 360
        angle_step = diff / (lsn - 1) if lsn > 1 else 0

        parsed_points = []
        for i in range(lsn):
            dist_val = struct.unpack('<H', packet[10 + 2 * i: 12 + 2 * i])[0]
            distance_mm = dist_val / 4.0
            if distance_mm > 0:
                raw_angle = angle_fsa + (angle_step * i)
                parsed_points.append((raw_angle, distance_mm))
        return parsed_points
    except:
        return []


# --- MAIN LOOP ---
if __name__ == "__main__":
    if not load_background_model(BG_FILE):
        sys.exit(1)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        ser.setDTR(True)
        ser.write(b'\xA5\x60')
        time.sleep(1.0)
        ser.reset_input_buffer()

        print(f"🚀 Robust Object Tracker on {PORT}")
        print(f"🛡️  Wall Buffer: {NOISE_TOLERANCE * 100} cm")
        print(f"👀 Search Window: +/- {SEARCH_WINDOW} bins")
        print("⌨️  Press Ctrl+C to STOP and SAVE.\n")

        buffer = b''
        last_raw_angle = 0.0
        rotation_count = 0
        points_captured_this_frame = 0

        while True:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)

                while len(buffer) > 10:
                    if buffer[0] == 0xAA and buffer[1] == 0x55:
                        lsn = buffer[3]
                        packet_len = 10 + (2 * lsn)
                        if len(buffer) < packet_len: break

                        packet = buffer[:packet_len]
                        buffer = buffer[packet_len:]

                        new_data = parse_packet(packet)

                        for (angle, dist_mm) in new_data:
                            norm_angle = angle % 360.0
                            dist_m = dist_mm / 1000.0

                            # 1. Filter Blind Spots & Max Range
                            if dist_m < MIN_DIST_M or dist_m > 12.0: continue

                            # 2. ROBUST BACKGROUND CHECK
                            # If it returns True, it is the wall (or close to it)
                            if is_background(norm_angle, dist_m):
                                continue

                                # 3. If we are here, it is an OBJECT
                            ang_rad = math.radians(norm_angle)
                            x = dist_m * math.cos(ang_rad)
                            y = dist_m * math.sin(ang_rad)

                            # 4. Voxel Snap
                            x_snap = round(x / VOXEL_SIZE) * VOXEL_SIZE
                            y_snap = round(y / VOXEL_SIZE) * VOXEL_SIZE

                            dynamic_points_set.add((x_snap, y_snap))
                            points_captured_this_frame += 1

                            if (last_raw_angle - norm_angle) > 200:
                                rotation_count += 1
                                sys.stdout.write(
                                    f"\r🔄 Scans: {rotation_count} | 🚛 Object Points: {len(dynamic_points_set)}   ")
                                sys.stdout.flush()
                                points_captured_this_frame = 0

                            last_raw_angle = norm_angle
                    else:
                        buffer = buffer[1:]

    except KeyboardInterrupt:
        print("\n🛑 Stopping...")
        if 'ser' in locals() and ser.is_open:
            ser.write(b'\xA5\x65')
            ser.close()

        save_combined_file(dynamic_points_set)
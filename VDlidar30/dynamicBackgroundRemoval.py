import serial
import math
import time
import struct
import sys
import os

# --- CONFIGURATION ---
PORT = 'COM3'  # <--- CHECK PORT
BAUD = 512000
BG_FILE = "scans/final_zero_plane.pcd"
OUTPUT_FOLDER = "scans_combined"

# -- OPTIMIZATION SETTINGS --
ANGLE_BIN_SIZE = 0.25  # Reduced from 1.0 to 0.25 for higher precision
NOISE_TOLERANCE = 0.03  # Reduced from 0.20m to 0.03m (3cm) thanks to interpolation
VOXEL_SIZE = 0.005  # Snap points to nearest 5mm grid to reduce fuzz
MIN_DIST_M = 0.15  # Ignore points closer than 15cm (lidar blind spot)

# Global storage
background_map = {}
# Use a Set for the buffer to automatically remove exact duplicates
all_dynamic_points_set = set()


def get_bin_index(angle):
    """Converts angle to discrete bin index."""
    return int(angle / ANGLE_BIN_SIZE)


def load_background_model(filename):
    """Parses PCD to create a High-Res lookup table."""
    if not os.path.exists(filename):
        print(f"❌ Error: File '{filename}' not found!")
        return False

    print(f"📂 Loading High-Res Background: {filename}...")
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

                # Store in bin
                b_idx = get_bin_index(angle_deg)
                background_map[b_idx] = dist_m
                count += 1

        # Fill gaps: If a bin is empty, use the previous neighbor
        # This prevents crashes if the background scan had tiny gaps
        max_bins = int(360 / ANGLE_BIN_SIZE)
        for i in range(max_bins):
            if i not in background_map:
                # Look back up to 5 bins to fill gap
                for k in range(1, 6):
                    prev_idx = (i - k) % max_bins
                    if prev_idx in background_map:
                        background_map[i] = background_map[prev_idx]
                        break

        print(f"✅ Background Loaded! Points: {count}")
        return True
    except Exception as e:
        print(f"❌ Error parsing background: {e}")
        return False


def get_interpolated_bg_dist(angle):
    """
    Calculates expected background distance using Linear Interpolation.
    This allows us to compare exact angles rather than rough bins.
    """
    max_bins = int(360 / ANGLE_BIN_SIZE)

    # Exact float index
    idx_float = angle / ANGLE_BIN_SIZE

    idx_L = int(idx_float) % max_bins
    idx_R = (idx_L + 1) % max_bins

    dist_L = background_map.get(idx_L, 0)
    dist_R = background_map.get(idx_R, 0)

    if dist_L == 0 or dist_R == 0: return None  # Gap in background data

    # Calculate weight (residue)
    residue = idx_float - int(idx_float)

    # Weighted average
    expected_dist = dist_L * (1.0 - residue) + dist_R * residue
    return expected_dist


def save_combined_file(unique_points):
    """Saves the set of unique voxel points."""
    if not unique_points:
        print("\n⚠️ No object detected.")
        return

    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(OUTPUT_FOLDER, f"Clean_Scan_{timestamp}.pcd")

    print(f"\n\n💾 Saving Cleaned File: {filename}...")
    points_list = list(unique_points)
    print(f"📊 Total Voxelized Points: {len(points_list)}")

    header = f"""# .PCD v0.7 - Clean Voxel Scan
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
            # p is (x, y), add z=0
            f.write(f"{p[0]:.4f} {p[1]:.4f} 0.000\n")

    print("✅ Save Complete!")


def parse_packet(packet):
    # Standard RPLIDAR A1/A2 parser
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
            # Distance is in mm
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
        ser.write(b'\xA5\x60')  # Scan mode
        time.sleep(1.0)
        ser.reset_input_buffer()

        print(f"🚀 High-Res Tracker Running on {PORT}")
        print(f"🎯 Noise Tolerance: {NOISE_TOLERANCE * 100} cm")
        print("⌨️  Press Ctrl+C to SAVE result.\n")

        buffer = b''
        last_raw_angle = 0.0
        rotation_count = 0

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
                            # 1. Normalize
                            norm_angle = angle % 360.0
                            dist_m = dist_mm / 1000.0

                            # 2. Filter Blind Spots
                            if dist_m < MIN_DIST_M: continue

                            # 3. INTERPOLATED Background Check
                            bg_dist_m = get_interpolated_bg_dist(norm_angle)

                            if bg_dist_m:
                                # We only care if the object is CLOSER than the wall
                                # (dist_m < bg - tolerance)
                                # This removes "ghosts" behind the wall
                                if dist_m < (bg_dist_m - NOISE_TOLERANCE):
                                    # Convert to Cartesian
                                    ang_rad = math.radians(norm_angle)
                                    x = dist_m * math.cos(ang_rad)
                                    y = dist_m * math.sin(ang_rad)

                                    # 4. VOXEL GRID SNAP (The "Clarity" Fix)
                                    # Round to nearest VOXEL_SIZE (e.g., 5mm)
                                    # This forces scattered points into a single coordinate
                                    x_snap = round(x / VOXEL_SIZE) * VOXEL_SIZE
                                    y_snap = round(y / VOXEL_SIZE) * VOXEL_SIZE

                                    all_dynamic_points_set.add((x_snap, y_snap))

                            # UI Updates
                            if (last_raw_angle - norm_angle) > 200:
                                rotation_count += 1
                                sys.stdout.write(
                                    f"\r🔄 Rotations: {rotation_count} | 🧱 Voxel Points: {len(all_dynamic_points_set)}   ")
                                sys.stdout.flush()

                            last_raw_angle = norm_angle

                    else:
                        buffer = buffer[1:]

            time.sleep(0.0005)

    except KeyboardInterrupt:
        print("\n🛑 Stopping Hardware...")
        if 'ser' in locals() and ser.is_open:
            ser.write(b'\xA5\x65')
            ser.close()

        save_combined_file(all_dynamic_points_set)
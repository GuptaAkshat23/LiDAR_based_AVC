import serial
import math
import time
import struct
import sys
import statistics
import os

# --- CONFIGURATION ---
PORT = 'COM3'  # Adjust to your TG30 Port
BAUD = 512000  # TG30 Standard Baud Rate
ANGLE_BIN_SIZE = 0.5  # Resolution in degrees (0.5 deg bins = 720 bins total)
OUTPUT_FOLDER = "scans"  # Folder name to save files

# --- GLOBAL STORAGE ---
# Dictionary to store lists of distances for each angle bin
# Format: { bin_index: [dist1, dist2, dist3...], ... }
polar_bins = {}


def get_bin_index(angle):
    """Converts an angle (0-360) into a bin index based on resolution."""
    return int(angle / ANGLE_BIN_SIZE)


def save_single_zero_plane(bins, filename="final_zero_plane.pcd"):
    """
    Computes the MEDIAN distance for each bin to remove dynamic noise
    and saves a SINGLE clean Zero-Plane PCD file in the 'scans' folder.
    """
    clean_points = []

    print(f"\n\n🛑 Stopping... Analyzing {len(bins)} angle segments...")
    print("   Computing Median (Statistical Background Model)... please wait.")

    # 1. Ensure Output Folder Exists
    if not os.path.exists(OUTPUT_FOLDER):
        try:
            os.makedirs(OUTPUT_FOLDER)
            print(f"   Created folder: '{OUTPUT_FOLDER}'")
        except OSError as e:
            print(f"❌ Error creating folder: {e}")
            return

    # 2. Process Data
    for bin_idx, distances in bins.items():
        if len(distances) > 0:
            # Statistical Filter: Use Median to reject outliers
            median_dist = statistics.median(distances)

            # Reconstruct X, Y from the Bin Angle and Median Distance
            # We use the center angle of the bin
            angle_center = (bin_idx * ANGLE_BIN_SIZE) + (ANGLE_BIN_SIZE / 2)
            angle_rad = math.radians(angle_center)

            x = (median_dist / 1000.0) * math.cos(angle_rad)
            y = (median_dist / 1000.0) * math.sin(angle_rad)

            clean_points.append((x, y))

    if not clean_points:
        print("❌ Error: No valid points to save.")
        return

    # 3. Construct Full Path
    full_path = os.path.join(OUTPUT_FOLDER, filename)

    # 4. Write the PCD Header & Data
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {len(clean_points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(clean_points)}
DATA ascii
"""
    try:
        with open(full_path, 'w') as f:
            f.write(header)
            for p in clean_points:
                # Z is hardcoded to 0.000 (Zero Plane)
                f.write(f"{p[0]:.3f} {p[1]:.3f} 0.000\n")
        print(f"✅ SUCCESS: Saved '{full_path}' with {len(clean_points)} static points.")
    except Exception as e:
        print(f"❌ File Write Error: {e}")


def parse_packet(packet):
    """Decodes YDLidar packet. Returns list of (angle, distance_mm)."""
    try:
        lsn = packet[3]
        if lsn <= 0: return []

        fsa = struct.unpack('<H', packet[4:6])[0]
        lsa = struct.unpack('<H', packet[6:8])[0]

        angle_fsa = (fsa >> 1) / 64.0
        angle_lsa = (lsa >> 1) / 64.0

        diff = angle_lsa - angle_fsa
        if diff < 0: diff += 360

        angle_step = diff / (lsn - 1) if lsn > 1 else 0

        parsed_data = []

        for i in range(lsn):
            idx = 10 + (2 * i)  # TG30 Standard 2-byte distance
            dist_data = struct.unpack('<H', packet[idx:idx + 2])[0]
            distance = dist_data / 4.0

            if distance > 0:
                current_angle = angle_fsa + (angle_step * i)
                parsed_data.append((current_angle, distance))

        return parsed_data
    except:
        return []


# --- MAIN LOOP ---
if __name__ == "__main__":
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        ser.setDTR(True)
        ser.write(b'\xA5\x60')
        time.sleep(0.5)
        ser.reset_input_buffer()

        print(f"🚀 Building Statistical Zero Plane on {PORT}")
        print(f"📂 Output will be saved to: ./{OUTPUT_FOLDER}/")
        print("💡 Recording indefinitely. Press CTRL+C to stop and save.")
        print("-" * 50)

        buffer = b''
        previous_angle = 0.0
        frame_count = 0
        points_accumulated = 0

        while True:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)

                while len(buffer) > 10:
                    if buffer[0] == 0xAA and buffer[1] == 0x55:
                        lsn = buffer[3]
                        packet_len = 10 + (2 * lsn)

                        if len(buffer) < packet_len:
                            break

                        packet = buffer[:packet_len]
                        buffer = buffer[packet_len:]

                        # Get Data
                        new_data = parse_packet(packet)

                        for (angle, distance) in new_data:
                            # 1. Binning Logic
                            b_idx = get_bin_index(angle)
                            if b_idx not in polar_bins:
                                polar_bins[b_idx] = []
                            polar_bins[b_idx].append(distance)
                            points_accumulated += 1

                            # 2. Frame Detection (for UI only)
                            if angle < previous_angle and previous_angle > 300 and angle < 50:
                                frame_count += 1

                                # Live Status
                                sys.stdout.write(
                                    f"\r[Recording] Frames: {frame_count} | "
                                    f"Total Samples: {points_accumulated} "
                                )
                                sys.stdout.flush()

                            previous_angle = angle
                    else:
                        buffer = buffer[1:]

    except KeyboardInterrupt:
        # This block runs when you press CTRL+C
        if 'ser' in locals() and ser.is_open:
            ser.write(b'\xA5\x65')  # Stop command
            ser.close()

        # PROCESS AND SAVE DATA
        save_single_zero_plane(polar_bins)

    except Exception as e:
        print(f"\nError: {e}")
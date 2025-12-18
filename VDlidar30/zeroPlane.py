import serial
import math
import time
import struct
import sys
import statistics

# --- CONFIGURATION ---
PORT = 'COM3'  # Adjust to your TG30 Port
BAUD = 512000  # TG30 Standard Baud Rate
FRAMES_TO_COLLECT = 50  # How many frames to average before saving the final model
ANGLE_BIN_SIZE = 0.5  # Resolution in degrees (0.5 deg bins = 720 bins total)

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
    and saves a SINGLE clean Zero-Plane PCD file.
    """
    clean_points = []

    print(f"\n\nProcessing Statistical Model from {len(bins)} angle segments...")

    for bin_idx, distances in bins.items():
        if len(distances) > 5:  # Threshold: Ignore bins with too little data
            # Statistical Filter: Use Median to reject outliers (temporary obstacles/noise)
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

    # Write the PCD Header
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
    with open(filename, 'w') as f:
        f.write(header)
        for p in clean_points:
            # Z is hardcoded to 0.000 (Zero Plane)
            f.write(f"{p[0]:.3f} {p[1]:.3f} 0.000\n")

    print(f"✅ SUCCESS: Saved 'final_zero_plane.pcd' with {len(clean_points)} static points.")


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
                # Store pure polar data (Angle, Distance) for statistical processing
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
        print(f"🎯 Target: Collect {FRAMES_TO_COLLECT} frames for averaging...")

        buffer = b''
        previous_angle = 0.0
        frame_count = 0

        while frame_count < FRAMES_TO_COLLECT:
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
                            # 1. Binning Logic (Statistical Accumulation)
                            b_idx = get_bin_index(angle)
                            if b_idx not in polar_bins:
                                polar_bins[b_idx] = []
                            polar_bins[b_idx].append(distance)

                            # 2. Frame Detection
                            if angle < previous_angle and previous_angle > 300 and angle < 50:
                                frame_count += 1

                                # Progress Bar
                                percent = (frame_count / FRAMES_TO_COLLECT) * 100
                                sys.stdout.write(
                                    f"\r[Progress] {frame_count}/{FRAMES_TO_COLLECT} frames ({percent:.1f}%)")
                                sys.stdout.flush()

                                if frame_count >= FRAMES_TO_COLLECT:
                                    break

                            previous_angle = angle
                    else:
                        buffer = buffer[1:]

        # --- FINAL PROCESSING ---
        save_single_zero_plane(polar_bins)

        # Cleanup
        ser.write(b'\xA5\x65')
        ser.close()

    except KeyboardInterrupt:
        print("\n🛑 Interrupted! Saving what we have so far...")
        save_single_zero_plane(polar_bins)
        if 'ser' in locals() and ser.is_open:
            ser.write(b'\xA5\x65')
            ser.close()

    except Exception as e:
        print(f"\nError: {e}")
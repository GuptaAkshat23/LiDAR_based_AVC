import serial
import math
import time
import struct
import sys
import statistics
import os

# --- CONFIGURATION ---
PORT = 'COM3'  # Adjust to your RS-485 Adapter Port
BAUD = 460800  # Standard LZR-U921 Baud Rate
ANGLE_BIN_SIZE = 0.5  # Resolution in degrees for the output file
OUTPUT_FOLDER = "scans"

# --- LZR CONSTANTS ---
# Sync Header: 0xFFFEFDFC (Little Endian -> FC FD FE FF)
LZR_SYNC = b'\xFC\xFD\xFE\xFF'
CMD_MDI = 50011  # Command code for Measurement Data [cite: 320]
FOV_START = -48.0  # Starting angle in degrees [cite: 209]
FOV_END = 48.0  # Ending angle in degrees [cite: 216]
POINTS_PER_PLANE = 274  # Max points per plane [cite: 207]

# Calculate angular step: 96 degrees / 273 steps ~= 0.3516 degrees
ANGLE_STEP = (FOV_END - FOV_START) / (POINTS_PER_PLANE - 1)

# --- GLOBAL STORAGE ---
polar_bins = {}


def get_bin_index(angle):
    """Converts an angle into a bin index based on resolution."""
    return int(angle / ANGLE_BIN_SIZE)


def save_single_zero_plane(bins, filename="lzr_zero_plane.pcd"):
    """
    Computes the MEDIAN distance for each bin and saves a PCD file.
    """
    clean_points = []
    print(f"\n\n🛑 Stopping... Analyzing {len(bins)} angle segments...")
    print("   Computing Median (Statistical Background Model)... please wait.")

    if not os.path.exists(OUTPUT_FOLDER):
        try:
            os.makedirs(OUTPUT_FOLDER)
        except OSError as e:
            print(f"❌ Error creating folder: {e}")
            return

    for bin_idx, distances in bins.items():
        if len(distances) > 0:
            median_dist = statistics.median(distances)

            # Reconstruct X, Y from the Bin Angle and Median Distance
            angle_center = (bin_idx * ANGLE_BIN_SIZE) + (ANGLE_BIN_SIZE / 2)
            angle_rad = math.radians(angle_center)

            # Convert mm to meters
            x = (median_dist / 1000.0) * math.cos(angle_rad)
            y = (median_dist / 1000.0) * math.sin(angle_rad)

            # LZR is often mounted vertically or looking down; Z is 0 for this projection
            clean_points.append((x, y))

    if not clean_points:
        print("❌ Error: No valid points to save.")
        return

    full_path = os.path.join(OUTPUT_FOLDER, filename)

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
                f.write(f"{p[0]:.3f} {p[1]:.3f} 0.000\n")
        print(f"✅ SUCCESS: Saved '{full_path}' with {len(clean_points)} static points.")
    except Exception as e:
        print(f"❌ File Write Error: {e}")


def parse_lzr_packet(payload):
    """
    Decodes LZR-U921 payload.
    Assumes payload contains [CMD, Options..., Plane, MDI].
    We look for the MDI block at the end of the payload.
    """
    try:
        # 1. Check Command [cite: 319]
        # CMD is the first 2 bytes of the message
        cmd = struct.unpack('<H', payload[:2])[0]
        if cmd != CMD_MDI:
            return []

        # 2. Extract MDI Data
        # The MDI (distances) is always at the end of the message[cite: 301].
        # Length is 274 points * 2 bytes = 548 bytes.
        mdi_len = POINTS_PER_PLANE * 2

        if len(payload) < mdi_len + 2:  # +2 for CMD
            return []

        # Take the last 548 bytes as the distance data
        raw_distances = payload[-mdi_len:]

        parsed_data = []

        for i in range(POINTS_PER_PLANE):
            # Extract 2-byte distance (mm) [cite: 380]
            dist_mm = struct.unpack('<H', raw_distances[i * 2: i * 2 + 2])[0]

            # Calculate Angle (Curtain -48 to +48) [cite: 213]
            current_angle = FOV_START + (i * ANGLE_STEP)

            # LZR Max Range is 65m (65000mm). Filter invalid/max points if needed.
            if 0 < dist_mm < 65000:
                parsed_data.append((current_angle, dist_mm))

        return parsed_data

    except Exception as e:
        print(f"Parse Error: {e}")
        return []


def verify_checksum(message, received_chk):
    """
    Calculates checksum: Sum of all bytes of the message part[cite: 285].
    """
    calc_sum = sum(message) & 0xFFFF  # Keep it 16-bit
    return calc_sum == received_chk


# --- MAIN LOOP ---
if __name__ == "__main__":
    try:
        # Initialize Serial [cite: 236]
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        ser.reset_input_buffer()

        print(f"🚀 Connecting to LZR-U921 on {PORT} @ {BAUD} bps")
        print(f"📂 Output will be saved to: ./{OUTPUT_FOLDER}/")
        print("💡 Recording... Press CTRL+C to stop and save.")
        print("-" * 50)

        buffer = b''
        frame_count = 0
        points_accumulated = 0

        while True:
            # Read chunks of data
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)

            # Search for Sync Header FC FD FE FF
            if len(buffer) < 8:  # Min size for Header
                time.sleep(0.001)
                continue

            sync_idx = buffer.find(LZR_SYNC)

            if sync_idx == -1:
                # Keep last few bytes in case sync is split
                buffer = buffer[-3:]
                continue

            # Align buffer to Sync
            buffer = buffer[sync_idx:]

            if len(buffer) < 6:  # Need Sync(4) + Size(2)
                continue

            # Read Message Size [cite: 266]
            # SIZE is bytes 4-5 (Little Endian)
            msg_size = struct.unpack('<H', buffer[4:6])[0]

            # Total Frame = Sync(4) + Size(2) + Message(msg_size) + Checksum(2)
            total_len = 4 + 2 + msg_size + 2

            if len(buffer) < total_len:
                continue  # Wait for more data

            # Extract Full Packet
            packet = buffer[:total_len]
            buffer = buffer[total_len:]  # Advance buffer

            # Parse Components
            # Header = packet[0:6]
            message_body = packet[6: 6 + msg_size]
            checksum_bytes = packet[6 + msg_size: 6 + msg_size + 2]

            received_chk = struct.unpack('<H', checksum_bytes)[0]

            # Verify Integrity
            if verify_checksum(message_body, received_chk):

                # Decode Distances
                new_data = parse_lzr_packet(message_body)

                if new_data:
                    frame_count += 1
                    for (angle, distance) in new_data:
                        # Binning
                        b_idx = get_bin_index(angle)
                        if b_idx not in polar_bins:
                            polar_bins[b_idx] = []
                        polar_bins[b_idx].append(distance)
                        points_accumulated += 1

                    # Live Status
                    if frame_count % 10 == 0:
                        sys.stdout.write(
                            f"\r[LZR-U921] Frames: {frame_count} | "
                            f"Total Pts: {points_accumulated} "
                        )
                        sys.stdout.flush()
            else:
                # Checksum failed, just skip this packet
                pass

    except KeyboardInterrupt:
        if 'ser' in locals() and ser.is_open:
            ser.close()

        # PROCESS AND SAVE DATA
        save_single_zero_plane(polar_bins)

    except Exception as e:
        print(f"\nError: {e}")
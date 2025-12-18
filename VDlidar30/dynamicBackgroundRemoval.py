import serial
import math
import time
import struct
import sys
import os

# --- CONFIGURATION ---
PORT = 'COM3'  # Adjust to your TG30 Port
BAUD = 512000  # TG30 Baud Rate
BG_FILE = "scans/final_zero_plane.pcd"  # The static reference file
OUTPUT_FOLDER = "scans/dynamic"  # Where to save moving objects
ANGLE_BIN_SIZE = 0.5  # MUST match the size used to create the Zero Plane
NOISE_TOLERANCE = 0.20  # (Meters) Ignore changes smaller than 20cm
MIN_POINTS_TO_SAVE = 10  # Don't save empty frames (noise filtering)

# --- GLOBAL STORAGE ---
# Dictionary: { bin_index: distance_in_mm }
background_map = {}


def get_bin_index(angle):
    return int(angle / ANGLE_BIN_SIZE)


def load_background_model(filename):
    """Parses the Zero Plane PCD to create a lookup table for comparison."""
    if not os.path.exists(filename):
        print(f"❌ Error: Background file '{filename}' not found!")
        return False

    print(f"📂 Loading Background Model: {filename}...")
    count = 0
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()

        # Skip header, find DATA ascii
        is_data = False
        for line in lines:
            if line.startswith("DATA ascii"):
                is_data = True
                continue
            if not is_data: continue

            # Parse X, Y (Z is 0)
            parts = line.strip().split()
            if len(parts) >= 2:
                x = float(parts[0])
                y = float(parts[1])

                # Convert back to Polar (Angle, Dist) to populate lookup table
                dist_m = math.sqrt(x * x + y * y)
                angle_rad = math.atan2(y, x)
                angle_deg = math.degrees(angle_rad)
                if angle_deg < 0: angle_deg += 360

                # Map bin to distance
                b_idx = get_bin_index(angle_deg)
                background_map[b_idx] = dist_m * 1000.0  # Store in mm
                count += 1

        print(f"✅ Background Loaded! Reference points: {count}")
        return True
    except Exception as e:
        print(f"❌ Error parsing background: {e}")
        return False


def save_dynamic_frame(points, frame_id):
    """Saves ONLY the moving object points."""
    if len(points) < MIN_POINTS_TO_SAVE:
        return False

    filename = f"{OUTPUT_FOLDER}/move_{frame_id:04d}.pcd"

    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {len(points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA ascii
"""
    try:
        with open(filename, 'w') as f:
            f.write(header)
            for p in points:
                f.write(f"{p[0]:.3f} {p[1]:.3f} 0.000\n")
        return True
    except:
        return False


def parse_packet(packet):
    """Standard TG30 Packet Decoder"""
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
            idx = 10 + (2 * i)
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
    # 1. Setup Folders & Background
    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)

    if not load_background_model(BG_FILE):
        sys.exit(1)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        ser.setDTR(True)
        ser.write(b'\xA5\x60')
        time.sleep(0.5)
        ser.reset_input_buffer()

        print(f"🚀 Tracking Moving Objects on {PORT}")
        print(f"🎯 Threshold: Points changing > {NOISE_TOLERANCE * 100:.1f} cm are kept.")
        print("-" * 50)

        buffer = b''
        current_dynamic_points = []
        previous_angle = 0.0
        frame_count = 0

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

                            # --- BACKGROUND SUBTRACTION LOGIC ---
                            b_idx = get_bin_index(angle)

                            # Retrieve background distance (if it exists for this angle)
                            bg_dist_mm = background_map.get(b_idx)

                            is_moving_object = False

                            if bg_dist_mm:
                                # Calculate difference in METERS
                                delta_m = abs(dist_mm - bg_dist_mm) / 1000.0

                                # If the change is significant, it's an object!
                                if delta_m > NOISE_TOLERANCE:
                                    is_moving_object = True
                            else:
                                # If we have NO background data for this angle,
                                # you can decide to keep it or ignore it.
                                # Ignoring is safer to prevent noise.
                                pass

                            if is_moving_object:
                                angle_rad = math.radians(angle)
                                x = (dist_mm / 1000.0) * math.cos(angle_rad)
                                y = (dist_mm / 1000.0) * math.sin(angle_rad)
                                current_dynamic_points.append((x, y))

                            # --- FRAME LOGIC ---
                            if angle < previous_angle and previous_angle > 300 and angle < 50:
                                frame_count += 1
                                saved = save_dynamic_frame(current_dynamic_points, frame_count)

                                count_str = f"{len(current_dynamic_points)} pts" if saved else "Empty"
                                sys.stdout.write(f"\r[Tracking] Frame: {frame_count} | Object: {count_str}   ")
                                sys.stdout.flush()

                                current_dynamic_points = []

                            previous_angle = angle
                    else:
                        buffer = buffer[1:]

    except KeyboardInterrupt:
        if 'ser' in locals() and ser.is_open:
            ser.write(b'\xA5\x65')
            ser.close()
        print("\n🛑 Stopped.")
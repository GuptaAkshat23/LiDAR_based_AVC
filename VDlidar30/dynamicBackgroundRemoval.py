import serial
import math
import time
import struct
import sys
import os
import numpy as np

# --- CONFIGURATION ---
PORT = 'COM3'  # <--- CHECK YOUR PORT
BAUD = 512000
BG_FILE = "scans/final_zero_plane.pcd"  # Path to your Zero Plane file
OUTPUT_FOLDER = "scans_combined"  # Folder for the final file
ANGLE_BIN_SIZE = 1.0  # Bin size for lookup (Deg)
NOISE_TOLERANCE = 0.20  # 20cm threshold (ignore small jitters)

# Global storage for the background model
background_map = {}

# Global buffer to hold ALL object points found during the session
all_dynamic_points_buffer = []


def get_bin_index(angle):
    return int(angle / ANGLE_BIN_SIZE)


def load_background_model(filename):
    """Parses the Zero Plane PCD to create a lookup table."""
    if not os.path.exists(filename):
        print(f"❌ Error: Background file '{filename}' not found!")
        return False

    print(f"📂 Loading Background Model: {filename}...")
    count = 0
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()

        is_data = False
        for line in lines:
            if line.startswith("DATA ascii"):
                is_data = True
                continue
            if not is_data: continue

            parts = line.strip().split()
            if len(parts) >= 2:
                x = float(parts[0])
                y = float(parts[1])

                dist_m = math.sqrt(x * x + y * y)
                angle_rad = math.atan2(y, x)
                angle_deg = math.degrees(angle_rad)
                if angle_deg < 0: angle_deg += 360

                b_idx = get_bin_index(angle_deg)
                background_map[b_idx] = dist_m * 1000.0  # Store in mm
                count += 1

        print(f"✅ Background Loaded! Reference points: {count}")
        return True
    except Exception as e:
        print(f"❌ Error parsing background: {e}")
        return False


def save_combined_file(points):
    """Saves all accumulated points into ONE big PCD file."""
    if not points:
        print("\n⚠️ No moving objects were detected. Nothing to save.")
        return

    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(OUTPUT_FOLDER, f"Combined_Object_Scan_{timestamp}.pcd")

    print(f"\n\n💾 Saving Combined File: {filename}...")
    print(f"📊 Total Object Points: {len(points)}")

    header = f"""# .PCD v0.7 - Combined Dynamic Scan
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
    with open(filename, 'w') as f:
        f.write(header)
        for p in points:
            f.write(f"{p[0]:.3f} {p[1]:.3f} 0.000\n")

    print("✅ Save Complete!")


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
            distance = dist_val / 4.0

            if distance > 0:
                raw_angle = angle_fsa + (angle_step * i)
                parsed_points.append((raw_angle, distance))
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

        print(f"🚀 Combined Object Tracker Running on {PORT}")
        print("⚠️  Only points DIFFERENT from the background will be saved.")
        print("⌨️  Press Ctrl+C to STOP and SAVE the combined file.\n")
        print("🔴 Scanning...")

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
                            # 1. Normalize angle 0-360
                            norm_angle = angle % 360.0

                            # 2. Check Background
                            b_idx = get_bin_index(norm_angle)
                            bg_dist_mm = background_map.get(b_idx)

                            is_object = False

                            if bg_dist_mm:
                                delta_m = abs(dist_mm - bg_dist_mm) / 1000.0
                                # If difference > 20cm, it's an object
                                if delta_m > NOISE_TOLERANCE:
                                    is_object = True

                            # 3. Add to Main Buffer if it is an object
                            if is_object:
                                ang_rad = math.radians(norm_angle)
                                x = (dist_mm / 1000.0) * math.cos(ang_rad)
                                y = (dist_mm / 1000.0) * math.sin(ang_rad)
                                all_dynamic_points_buffer.append((x, y))

                            # 4. Simple Rotation Counter for UI
                            if (last_raw_angle - norm_angle) > 200:
                                rotation_count += 1
                                sys.stdout.write(
                                    f"\r🔄 Rotations: {rotation_count} | 📦 Object Points Collected: {len(all_dynamic_points_buffer)}   ")
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

        # --- THIS IS THE KEY CHANGE ---
        # Instead of saving frames in the loop, we save ONCE at the end
        save_combined_file(all_dynamic_points_buffer)
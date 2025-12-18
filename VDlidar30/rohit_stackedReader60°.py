import serial
import time
import numpy as np
import datetime
import os
import struct

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM3'  # Update to your TG30 port
BAUD_RATE = 512000  # TG30 standard baud rate
SAVE_DIRECTORY = "./scans"

# --- 60 DEGREE FILTER ---
MIN_ANGLE = -30.0
MAX_ANGLE = 30.0

# --- 3D STACKING SETTINGS ---
# Distance moved per full 360 rotation (e.g., 0.05m = 5cm)
Z_INCREMENT_METERS = 0.05

all_points_buffer = []
scan_counter = 0


def save_combined_pcd(points_list, output_folder):
    if not points_list:
        print("[WARNING] No points to save.")
        return
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_folder, f"tg30_stacked_3d_{timestamp}.pcd")

    num_points = len(points_list)
    print(f"\n[SAVING] Writing {num_points} points to {filename}...")

    with open(filename, 'w') as f:
        f.write("# .PCD v.7 - TG30 Stacked Scan\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {num_points}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {num_points}\nDATA ascii\n")
        for p in points_list:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
    print(f"[COMPLETE] File saved.")


def normalize_angle(angle):
    if angle > 180: return angle - 360
    return angle


def process_packet(packet):
    """Parses TG30 binary packet and returns 60-degree filtered points."""
    try:
        lsn = packet[3]
        fsa = struct.unpack('<H', packet[4:6])[0]
        lsa = struct.unpack('<H', packet[6:8])[0]
        angle_fsa = (fsa >> 1) / 64.0
        angle_lsa = (lsa >> 1) / 64.0

        diff = angle_lsa - angle_fsa
        if diff < 0: diff += 360
        angle_step = diff / (lsn - 1) if lsn > 1 else 0

        valid_points = []
        for i in range(lsn):
            dist_data = struct.unpack('<H', packet[10 + i * 2: 12 + i * 2])[0]
            distance = dist_data / 4.0  # TG30 Scale

            if distance > 0:
                raw_angle = angle_fsa + (angle_step * i)
                norm_angle = normalize_angle(raw_angle)

                # Filter for the 60-degree slice
                if MIN_ANGLE <= norm_angle <= MAX_ANGLE:
                    angle_rad = np.radians(norm_angle)

                    # Sensor Coordinates (2D slice)
                    # We treat distance as Width (Y) and angle as Height (Z)
                    # X will be our "Travel" (Stacking) axis
                    y_sens = (distance / 1000.0) * np.cos(angle_rad)
                    z_sens = (distance / 1000.0) * np.sin(angle_rad)

                    valid_points.append({
                        'raw_angle': raw_angle,
                        'y': y_sens,
                        'z': z_sens
                    })
        return valid_points
    except:
        return []


def run_recorder():
    global scan_counter
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.setDTR(True)
        ser.write(b'\xA5\x60')  # Start TG30
        time.sleep(0.5)
        ser.reset_input_buffer()

        print(f"🚀 TG30 3D Stacked Recorder Started (60° Window)...")

        buffer = b""
        last_raw_angle = 0.0

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

                        points = process_packet(packet)

                        for pt in points:
                            # Detect New Rotation to increment Z (Travel Axis)
                            if pt['raw_angle'] < last_raw_angle:
                                scan_counter += 1
                                if scan_counter % 20 == 0:
                                    print(f"Rotation Count: {scan_counter} | Points: {len(all_points_buffer)}")

                            # Map to 3D Space
                            # X = Progress (Travel), Y = Width, Z = Height
                            x_world = scan_counter * Z_INCREMENT_METERS
                            y_world = pt['y']
                            z_world = pt['z']

                            all_points_buffer.append((x_world, y_world, z_world))
                            last_raw_angle = pt['raw_angle']
                    else:
                        buffer = buffer[1:]
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if 'ser' in locals():
            ser.write(b'\xA5\x65')  # Stop TG30
            ser.close()
        save_combined_pcd(all_points_buffer, SAVE_DIRECTORY)


if __name__ == "__main__":
    run_recorder()
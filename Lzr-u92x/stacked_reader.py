import serial
import time
import numpy as np
import datetime
import os
import struct

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM5'  # Update this to your COM port
BAUD_RATE = 460800  # LZR-U921 Standard Baud
SAVE_DIRECTORY = "./scans"

# --- 3D STACKING SETTINGS ---
X_INCREMENT_METERS = 0.015

# --- SENSOR GEOMETRY ---
FOV_START_DEG = -48.0
FOV_END_DEG = 48.0
MAX_POINTS = 274  #

# LZR Sync Header: 0xFFFEFDFC
LZR_SYNC = b'\xFC\xFD\xFE\xFF'

all_points_buffer = []
scan_counter = 0


def save_combined_pcd(points_list, output_folder):
    """Saves accumulated 3D points to a PCD file."""
    if not points_list:
        print("[WARNING] No points to save.")
        return
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_folder, f"lzr_u921_stacked_{timestamp}.pcd")

    num_points = len(points_list)
    print(f"\n[SAVING] Writing {num_points} points to {filename}...")

    try:
        with open(filename, 'w') as f:
            f.write("# .PCD v.7\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {num_points}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {num_points}\nDATA ascii\n")
            for p in points_list:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
        print(f"[COMPLETE] File saved: {filename}")
    except Exception as e:
        print(f"[ERROR] Save failed: {e}")


def verify_checksum(payload, received_chk):
    """
    Calculates the 16-bit sum of the payload.
    Checks against both Little and Big Endian interpretations.
    """
    calc_sum = sum(payload) & 0xFFFF
    # Checksum is valid if it matches the received value
    return calc_sum == received_chk


def process_lzr_packet(payload):
    """
    Parses distances using Big-Endian and 14-bit masking.
    """
    valid_points = []
    try:
        # Command word is Big-Endian
        cmd = struct.unpack('>H', payload[:2])[0]
        if cmd != 50011:
            return []

        # Distance block is 548 bytes at the end of the payload
        mdi_size = MAX_POINTS * 2
        raw_data = payload[-mdi_size:]
        num_points = len(raw_data) // 2

        # Correct resolution to fix 'curved wall'
        step = (FOV_END_DEG - FOV_START_DEG) / (num_points - 1)

        for i in range(num_points):
            # Read 2-byte distance as Big-Endian
            raw_val = struct.unpack('>H', raw_data[i * 2: i * 2 + 2])[0]

            # Mask top 2 status bits to fix 'ghost points'
            dist_mm = raw_val & 0x3FFF

            if 50 < dist_mm < 10000:  # Ignore noise and max-range
                angle_deg = FOV_START_DEG + (i * step)
                angle_rad = np.radians(angle_deg)

                # Cartesian conversion
                y_sens = (dist_mm / 1000.0) * np.sin(angle_rad)
                z_sens = (dist_mm / 1000.0) * np.cos(angle_rad)

                valid_points.append({'y': y_sens, 'z': z_sens})
        return valid_points
    except:
        return []


def run_recorder():
    global scan_counter
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        ser.reset_input_buffer()

        print(f"🚀 LZR-U921 3D Recorder Started (Baud: {BAUD_RATE})")
        print("Press Ctrl+C to stop and save.")

        buffer = b""
        while True:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)

                sync_idx = buffer.find(LZR_SYNC)
                if sync_idx >= 0:
                    buffer = buffer[sync_idx:]

                    if len(buffer) < 8:  # Sync(4) + Size(2) + Payload... + Checksum(2)
                        continue

                    # Size field is usually Little-Endian
                    msg_size = struct.unpack('<H', buffer[4:6])[0]
                    total_packet_len = 4 + 2 + msg_size + 2

                    if len(buffer) < total_packet_len:
                        continue

                    packet = buffer[:total_packet_len]
                    buffer = buffer[total_packet_len:]

                    payload = packet[6: 6 + msg_size]

                    # Try Checksum as Little-Endian
                    chk_field_le = struct.unpack('<H', packet[6 + msg_size: 6 + msg_size + 2])[0]
                    # Try Checksum as Big-Endian
                    chk_field_be = struct.unpack('>H', packet[6 + msg_size: 6 + msg_size + 2])[0]

                    if verify_checksum(payload, chk_field_le) or verify_checksum(payload, chk_field_be):
                        points = process_lzr_packet(payload)
                        if points:
                            scan_counter += 1
                            x_world = scan_counter * X_INCREMENT_METERS
                            for pt in points:
                                all_points_buffer.append((x_world, pt['y'], pt['z']))

                            if scan_counter % 60 == 0:
                                print(f"Processing... {scan_counter} scans captured.")
                    else:
                        # Optional: print(f"Checksum Fail: Sum {sum(payload)&0xFFFF}")
                        pass
                else:
                    if len(buffer) > 4096:
                        buffer = buffer[-1024:]

    except KeyboardInterrupt:
        print("\nStopping and saving...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        save_combined_pcd(all_points_buffer, SAVE_DIRECTORY)


if __name__ == "__main__":
    run_recorder()
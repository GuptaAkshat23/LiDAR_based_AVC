import serial
import time
import numpy as np
import datetime
import os
import struct

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM3'  # Adjust to your RS-485 USB Adapter
BAUD_RATE = 460800  # LZR-U921 Standard High-Speed Baud
SAVE_DIRECTORY = "./scans"

# --- 3D STACKING SETTINGS ---
# Distance moved per scan line (LZR-U921 runs at 60Hz)
# Adjust this value based on your conveyor/vehicle speed.
# Example: 1 m/s speed / 60 scans/s = ~0.016 m/scan
X_INCREMENT_METERS = 0.015

# --- SENSOR GEOMETRY ---
FOV_START_DEG = -48.0  # LZR Start Angle [cite: 209]
FOV_END_DEG = 48.0  # LZR End Angle [cite: 216]
MAX_POINTS = 274  # Max points per plane

# LZR Sync Header: 0xFFFEFDFC (Little Endian -> FC FD FE FF)
LZR_SYNC = b'\xFC\xFD\xFE\xFF'

all_points_buffer = []
scan_counter = 0


def save_combined_pcd(points_list, output_folder):
    """Saves the accumulated 3D points to a PCD file."""
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
            f.write(
                "# .PCD v.7 - LZR-U921 Stacked Scan\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {num_points}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {num_points}\nDATA ascii\n")
            for p in points_list:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
        print(f"[COMPLETE] File saved.")
    except Exception as e:
        print(f"[ERROR] Could not save file: {e}")


def verify_checksum(payload, received_chk):
    """Sum of all bytes in the message part[cite: 285]."""
    calc_sum = sum(payload) & 0xFFFF
    return calc_sum == received_chk


def process_lzr_packet(payload):
    """
    Parses LZR-U921 MDI packet.
    Payload = CMD (2) + [Optional Headers] + Distances + [Optional Footer]
    """
    valid_points = []
    try:
        # CMD is the first 2 bytes (50011 for MDI) [cite: 320]
        cmd = struct.unpack('<H', payload[:2])[0]
        if cmd != 50011:
            return []

        # The Distance Data (MDI) is usually the last block.
        # Max length is 274 * 2 = 548 bytes[cite: 383].
        # We assume the last N bytes are the distances.
        # (Note: Requires LZR to be configured to send distances, which is default)

        # Heuristic: We look for the largest block of data that fits into 2-byte chunks
        # and corresponds to the max points (274).
        mdi_size = MAX_POINTS * 2

        if len(payload) < mdi_size + 2:
            # If payload is smaller than max points, assume entire payload after CMD is data
            # (Configurable via 'Number of distance values' parameter [cite: 920])
            raw_data = payload[2:]  # Skip CMD
        else:
            # Otherwise, grab the last 548 bytes (safest for default config)
            raw_data = payload[-mdi_size:]

        num_points = len(raw_data) // 2

        # Calculate angular resolution for this specific packet
        if num_points > 1:
            step = (FOV_END_DEG - FOV_START_DEG) / (num_points - 1)
        else:
            step = 0

        for i in range(num_points):
            # Distance is 2 bytes Little Endian in mm [cite: 380]
            dist_mm = struct.unpack('<H', raw_data[i * 2: i * 2 + 2])[0]

            if 10 < dist_mm < 65000:  # Filter noise and max range [cite: 202]
                # Calculate Angle
                angle_deg = FOV_START_DEG + (i * step)
                angle_rad = np.radians(angle_deg)

                # Convert to Cartesian (Sensor Frame)
                # Assuming sensor is mounted flat, scanning horizontally:
                # Y = Width, Z = Depth/Distance
                # (Adjust sin/cos based on your specific mounting rotation)
                y_sens = (dist_mm / 1000.0) * np.sin(angle_rad)
                z_sens = (dist_mm / 1000.0) * np.cos(angle_rad)

                valid_points.append({
                    'y': y_sens,
                    'z': z_sens
                })
        return valid_points

    except Exception as e:
        # print(f"Parse Error: {e}")
        return []


def run_recorder():
    global scan_counter
    try:
        # LZR uses RS-485. Ensure your adapter supports 460800 baud.
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        ser.reset_input_buffer()

        print(f"🚀 LZR-U921 3D Stacked Recorder Started...")
        print(f"   Mode: Curtain Scan (Stacking along X-axis)")
        print(f"   Baud: {BAUD_RATE}")

        buffer = b""

        while True:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)

                # 1. Search for SYNC Header
                sync_idx = buffer.find(LZR_SYNC)
                if sync_idx >= 0:
                    buffer = buffer[sync_idx:]  # Align to sync

                    # Need at least Header(4) + Size(2)
                    if len(buffer) < 6:
                        continue

                    # 2. Read Frame Size [cite: 266]
                    msg_size = struct.unpack('<H', buffer[4:6])[0]

                    # Total Frame = Sync(4) + Size(2) + Message(msg_size) + Chk(2) [cite: 254]
                    total_len = 4 + 2 + msg_size + 2

                    if len(buffer) < total_len:
                        continue  # Wait for full packet

                    # 3. Extract Packet
                    packet = buffer[:total_len]
                    buffer = buffer[total_len:]

                    # 4. Verify Checksum [cite: 284]
                    message_payload = packet[6: 6 + msg_size]
                    received_chk = struct.unpack('<H', packet[6 + msg_size:])[0]

                    if verify_checksum(message_payload, received_chk):
                        # 5. Parse Points
                        points = process_lzr_packet(message_payload)

                        if points:
                            scan_counter += 1
                            if scan_counter % 60 == 0:  # Approx once per second at 60Hz
                                print(f"Scan Lines: {scan_counter} | Total Points: {len(all_points_buffer)}")

                            # 6. Stack Points in 3D
                            # X = Motion (Time * Increment)
                            # Y = Sensor Width
                            # Z = Sensor Depth
                            x_world = scan_counter * X_INCREMENT_METERS

                            for pt in points:
                                all_points_buffer.append((x_world, pt['y'], pt['z']))
                    else:
                        print("Checksum Fail")
                else:
                    # Discard junk bytes if buffer gets too full
                    if len(buffer) > 2048:
                        buffer = buffer[-1024:]

            # Small sleep to prevent CPU hogging
            # time.sleep(0.0001)

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        save_combined_pcd(all_points_buffer, SAVE_DIRECTORY)


if __name__ == "__main__":
    run_recorder()
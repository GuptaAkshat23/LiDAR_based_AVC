import serial
import math
import time
import struct
import sys

# --- CONFIGURATION ---
PORT = 'COM3'  # Adjust to your TG30 Port
BAUD = 512000  # TG30 Standard Baud Rate
SAVE_TO_FILE = True  # Set to False if you only want to view stats


def save_pcd(points, frame_id):
    """Saves the scan as a Zero-Plane (Z=0) PCD file."""
    if len(points) < 50:
        return False  # Skip empty/noise frames

    filename = f"scan_{frame_id:04d}.pcd"

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
                # Z is hardcoded to 0.000 for the "Zero Plane" approach
                f.write(f"{p[0]:.3f} {p[1]:.3f} 0.000\n")
        return True
    except Exception as e:
        print(f"\nError saving file: {e}")
        return False


def parse_packet(packet):
    """Decodes YDLidar packet. Returns list of (angle, x, y)."""
    try:
        lsn = packet[3]
        if lsn <= 0: return []

        # FSA (First Sample Angle) and LSA (Last Sample Angle)
        fsa = struct.unpack('<H', packet[4:6])[0]
        lsa = struct.unpack('<H', packet[6:8])[0]

        angle_fsa = (fsa >> 1) / 64.0
        angle_lsa = (lsa >> 1) / 64.0

        diff = angle_lsa - angle_fsa
        if diff < 0: diff += 360

        angle_step = diff / (lsn - 1) if lsn > 1 else 0

        parsed_points = []

        for i in range(lsn):
            # TG30 Standard: 2 bytes for distance.
            # NOTE: If your TG30 is in Intensity Mode, this offset might need to be 3 bytes per point.
            idx = 10 + (2 * i)

            dist_data = struct.unpack('<H', packet[idx:idx + 2])[0]
            distance = dist_data / 4.0

            if distance > 0:
                current_angle = angle_fsa + (angle_step * i)
                angle_rad = math.radians(current_angle)

                # Project to Zero Plane (2D -> 3D)
                x = (distance / 1000.0) * math.cos(angle_rad)
                y = (distance / 1000.0) * math.sin(angle_rad)

                parsed_points.append((current_angle, x, y))

        return parsed_points
    except Exception:
        return []


# --- MAIN LOOP ---
if __name__ == "__main__":
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)

        # 1. Reset/Wake Up
        ser.setDTR(True)
        ser.write(b'\xA5\x60')  # Scan Command
        time.sleep(0.5)
        ser.reset_input_buffer()

        print(f"🚀 YDLidar TG30 Connected on {PORT}")
        print("Waiting for data stream...\n")

        buffer = b''
        current_scan_points = []
        previous_angle = 0.0
        frame_count = 0

        start_time = time.time()

        while True:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)

                # Process packets
                while len(buffer) > 10:
                    # Sync Header: AA 55
                    if buffer[0] == 0xAA and buffer[1] == 0x55:

                        # Packet Type Check (CT byte)
                        # type_byte = buffer[2]
                        lsn = buffer[3]

                        # Calculate Packet Length: Header(10) + Data(2 * LSN)
                        # Warning: If TG30 sends intensity, change 2 to 3 below
                        packet_len = 10 + (2 * lsn)

                        if len(buffer) < packet_len:
                            break  # Buffer incomplete, wait for more bytes

                        packet = buffer[:packet_len]
                        buffer = buffer[packet_len:]

                        # Parse
                        new_data = parse_packet(packet)

                        for (angle, x, y) in new_data:
                            # --- END OF CIRCLE DETECTION ---
                            # Angle wraps from ~359 to ~0
                            if angle < previous_angle and previous_angle > 300 and angle < 50:

                                # 1. Frame Complete
                                frame_count += 1
                                points_in_frame = len(current_scan_points)

                                # 2. Save Data
                                saved_status = "Skipped"
                                if SAVE_TO_FILE:
                                    if save_pcd(current_scan_points, frame_count):
                                        saved_status = "Saved"

                                # 3. Display Stats (Overwrites previous line)
                                sys.stdout.write(
                                    f"\r[Recording] Frames Taken: {frame_count} | "
                                    f"Points: {points_in_frame:04d} | "
                                    f"Status: {saved_status}   "
                                )
                                sys.stdout.flush()

                                # Reset for next frame
                                current_scan_points = []

                            current_scan_points.append((x, y))
                            previous_angle = angle

                    else:
                        # If header not found, slide buffer by 1 to find sync
                        buffer = buffer[1:]

    except KeyboardInterrupt:
        if 'ser' in locals() and ser.is_open:
            ser.write(b'\xA5\x65')  # Stop Command
            ser.close()
        print("\n\n🛑 Stopped by User.")
        print(f"Total Frames Captured: {frame_count}")

    except Exception as e:
        print(f"\nError: {e}")
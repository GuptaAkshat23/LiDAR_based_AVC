import serial
import math
import time
import struct

PORT = 'COM7'
BAUD = 512000


def save_pcd(points, filename):
    # Only save if we have a decent amount of data (e.g., > 100 points)
    if len(points) < 50:
        return

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
    with open(filename, 'w') as f:
        f.write(header)
        for p in points:
            f.write(f"{p[0]:.3f} {p[1]:.3f} 0.000\n")
    print(f"✅ Saved Full Scan: {filename} ({len(points)} points)")


def parse_packet(packet):
    """Decodes a single packet and returns a list of (angle, x, y) tuples"""
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

        parsed_points = []

        for i in range(lsn):
            idx = 10 + (2 * i)
            dist_data = struct.unpack('<H', packet[idx:idx + 2])[0]
            distance = dist_data / 4.0

            if distance > 0:
                current_angle = angle_fsa + (angle_step * i)
                angle_rad = math.radians(current_angle)

                # Convert to Meters
                x = (distance / 1000.0) * math.cos(angle_rad)
                y = (distance / 1000.0) * math.sin(angle_rad)

                # Return Angle too, so we can detect "Wrap Around"
                parsed_points.append((current_angle, x, y))

        return parsed_points
    except:
        return []


# --- MAIN LOOP ---
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)

    # 1. Wake Up
    ser.setDTR(True)
    ser.write(b'\xA5\x60')
    time.sleep(0.5)
    ser.reset_input_buffer()

    print(f"🚀 Collecting Full 360° Scans on {PORT}...")

    buffer = b''
    current_scan_points = []
    previous_angle = 0.0

    while True:
        if ser.in_waiting:
            buffer += ser.read(ser.in_waiting)

            # Process all valid packets in the buffer
            while len(buffer) > 10:
                if buffer[0] == 0xAA and buffer[1] == 0x55:
                    lsn = buffer[3]
                    packet_len = 10 + (2 * lsn)

                    if len(buffer) < packet_len:
                        break  # Wait for more data

                    packet = buffer[:packet_len]
                    buffer = buffer[packet_len:]

                    # Parse the packet
                    new_data = parse_packet(packet)

                    for (angle, x, y) in new_data:
                        # --- ROTATION LOGIC ---
                        # If angle jumps from High (350) to Low (10), we finished a circle
                        if angle < previous_angle and previous_angle > 300 and angle < 50:
                            # SAVE THE FULL CIRCLE
                            timestamp = time.strftime("%H%M%S")
                            save_pcd(current_scan_points, f"scan_full_{timestamp}.pcd")
                            current_scan_points = []  # Reset for next circle


                        current_scan_points.append((x, y))
                        previous_angle = angle

                else:
                    buffer = buffer[1:]

except KeyboardInterrupt:
    if 'ser' in locals():
        ser.write(b'\xA5\x65')
        ser.close()
    print("Stopped.")
import serial
import math
import time
import struct
import os
import sys

# =========================
# CONFIGURATION
# =========================
PORT = 'COM3'
BAUD = 512000

OUTPUT_FOLDER = "rc_car_output"

# --- OBJECT PARAMETERS ---
MIN_DIST_M = 0.10      # 10 cm
MAX_DIST_M = 0.50      # 50 cm (RC car only)
FRONT_FOV = 30.0       # ±15 degrees ONLY
VOXEL_SIZE = 0.02      # 2 cm
MAX_SCANS = 1          # 🔴 ONLY ONE ROTATION

# =========================
# GLOBAL STORAGE
# =========================
scan_points = []

# =========================
# SAVE PCD
# =========================
def save_pcd(points):
    if not points:
        print("⚠ No points to save")
        return

    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)

    ts = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(OUTPUT_FOLDER, f"RC_CAR_{ts}.pcd")

    header = f"""# .PCD v0.7
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

    with open(filename, "w") as f:
        f.write(header)
        for x, y in points:
            f.write(f"{x:.4f} {y:.4f} 0.000\n")

    print(f"\n✅ Saved {len(points)} points → {filename}")

# =========================
# PACKET PARSER
# =========================
def parse_packet(packet):
    try:
        lsn = packet[3]
        fsa = struct.unpack('<H', packet[4:6])[0]
        lsa = struct.unpack('<H', packet[6:8])[0]

        angle_fsa = (fsa >> 1) / 64.0
        angle_lsa = (lsa >> 1) / 64.0

        diff = angle_lsa - angle_fsa
        if diff < 0:
            diff += 360.0

        step = diff / (lsn - 1) if lsn > 1 else 0

        points = []
        for i in range(lsn):
            raw = struct.unpack('<H', packet[10 + 2*i:12 + 2*i])[0]
            dist_m = (raw / 4.0) / 1000.0
            if dist_m <= 0:
                continue

            angle = (angle_fsa + step * i) % 360.0
            points.append((angle, dist_m))

        return points
    except:
        return []

# =========================
# MAIN
# =========================
if __name__ == "__main__":

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        ser.write(b'\xA5\x60')   # Start LiDAR
        time.sleep(1.0)
        ser.reset_input_buffer()

        print("\n🚗 RC CAR SINGLE-SCAN CAPTURE")
        print("📏 Distance: 10–50 cm")
        print("🎯 Front view only")
        print("🔄 Capturing 1 scan ONLY\n")

        buffer = b''
        last_angle = 0.0
        scan_count = 0

        while True:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)

                while len(buffer) > 10:
                    # Accept AA55 and AB65 headers
                    if (buffer[0] == 0xAA and buffer[1] == 0x55) or \
                       (buffer[0] == 0xAB and buffer[1] == 0x65):

                        lsn = buffer[3]
                        packet_len = 10 + 2 * lsn
                        if len(buffer) < packet_len:
                            break

                        packet = buffer[:packet_len]
                        buffer = buffer[packet_len:]

                        points = parse_packet(packet)

                        for angle, dist_m in points:

                            # Convert to signed angle (-180 to +180)
                            if angle > 180:
                                angle -= 360

                            # FRONT VIEW ONLY
                            if abs(angle) > FRONT_FOV / 2:
                                continue

                            # DISTANCE FILTER
                            if dist_m < MIN_DIST_M or dist_m > MAX_DIST_M:
                                continue

                            # CARTESIAN
                            rad = math.radians(angle)
                            x = dist_m * math.cos(rad)
                            y = dist_m * math.sin(rad)

                            # VOXEL SNAP
                            x = round(x / VOXEL_SIZE) * VOXEL_SIZE
                            y = round(y / VOXEL_SIZE) * VOXEL_SIZE

                            scan_points.append((x, y))

                            # ROTATION DETECTION
                            if last_angle > 170 and angle < -170:
                                scan_count += 1
                                print(f"✅ Scan completed: {scan_count}")
                                if scan_count >= MAX_SCANS:
                                    raise KeyboardInterrupt

                            last_angle = angle

                    else:
                        buffer = buffer[1:]

    except KeyboardInterrupt:
        print("\n🛑 Capture finished")
        if ser.is_open:
            ser.write(b'\xA5\x65')
            ser.close()

        save_pcd(list(set(scan_points)))
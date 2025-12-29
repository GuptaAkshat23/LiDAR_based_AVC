import serial
import struct
import numpy as np
import sys
import time

# =====================================================
# CONFIGURATION
# =====================================================
SERIAL_PORT = 'COM4'
BAUD_RATE = 921600
OUTPUT_FILENAME = "u921_stacked_scans.pcd"

START_ANGLE = -48.0
ANGULAR_RES = 0.3516

MIN_RANGE_M = 0.10
MAX_RANGE_M = 3.00

Z_INCREMENT = 0.05   # 5 cm per scan
MAX_SCANS = 200     # set None to run until CTRL+C

SYNC_HEADER = b'\xfc\xfd\xfe\xff'

# =====================================================
# GLOBAL STORAGE
# =====================================================
stacked_points = []
scan_index = 0

# =====================================================
# PROCESS ONE SCAN (NO HISTORY)
# =====================================================
def process_single_scan(data_bytes, z_value):
    points = []

    num_values = len(data_bytes) // 2

    for i in range(num_values):
        dist_mm = struct.unpack('<H', data_bytes[i*2:i*2+2])[0]
        if dist_mm == 0:
            continue

        dist_m = dist_mm / 1000.0
        if dist_m < MIN_RANGE_M or dist_m > MAX_RANGE_M:
            continue

        angle_deg = START_ANGLE + i * ANGULAR_RES
        angle_rad = np.radians(angle_deg)

        x = dist_m * np.cos(angle_rad)
        y = dist_m * np.sin(angle_rad)

        points.append((x, y, z_value))

    return points

# =====================================================
# SAVE STACKED PCD
# =====================================================
def save_stacked_pcd(points, filename):
    if not points:
        print("❌ No stacked points to save")
        return

    header = (
        "# .PCD v0.7 - LZR U921 STACKED SCANS\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA ascii\n"
    )

    with open(filename, "w") as f:
        f.write(header)
        for p in points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print(f"\n✅ STACKED PCD SAVED → {filename}")
    print(f"📊 Total stacked points: {len(points)}")

# =====================================================
# MAIN LOOP
# =====================================================
def main():
    global scan_index

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("🚀 U921 Connected")
        print("📦 Stacking scans...")
        print("⌨️  CTRL+C to stop")

        while True:
            if ser.read(1) == b'\xfc' and ser.read(3) == b'\xfd\xfe\xff':

                size = struct.unpack('<H', ser.read(2))[0]
                body = ser.read(size)
                ser.read(2)

                cmd = struct.unpack('<H', body[:2])[0]

                if cmd == 50011:
                    z_value = scan_index * Z_INCREMENT
                    scan_points = process_single_scan(body[3:], z_value)

                    stacked_points.extend(scan_points)
                    scan_index += 1

                    sys.stdout.write(
                        f"\rScans stacked: {scan_index} | Points: {len(stacked_points)}"
                    )
                    sys.stdout.flush()

                    if MAX_SCANS and scan_index >= MAX_SCANS:
                        break

    except KeyboardInterrupt:
        print("\n⏹️ Stopping capture...")

    finally:
        if 'ser' in locals():
            ser.close()
        save_stacked_pcd(stacked_points, OUTPUT_FILENAME)

# =====================================================
if __name__ == "__main__":
    main()
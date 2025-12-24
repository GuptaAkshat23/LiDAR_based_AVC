import serial
import struct
import numpy as np
import time

# --- CONFIGURATION ---
SERIAL_PORT = 'COM5'  # Change to COMx for Windows
BAUD_RATE = 921600  # Default high-speed baud [cite: 80, 740]
SYNC_HEADER = b'\xfc\xfd\xfe\xff'  # OxFFFE FDFC in little-endian


def save_to_pcd(points, filename="output.pcd"):
    """Saves a list of (x, y, z) points to a PCD file format."""
    header = (
        "# .PCD v0.7 - Point Cloud Data\n"
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
    with open(filename, 'w') as f:
        f.write(header)
        for p in points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")


def parse_mdi(data_bytes):
    """Converts raw distance values (mm) to Cartesian (x, y) coordinates."""
    points = []
    num_values = len(data_bytes) // 2

    # Angular resolution is ~0.3516° [cite: 36]
    # Starting angle is -48° [cite: 38, 756]
    start_angle = -48.0
    angular_res = 0.3516

    for i in range(num_values):
        # Each distance is encoded via 2 bytes (little-endian) [cite: 209, 216]
        dist_mm = struct.unpack('<H', data_bytes[i * 2: i * 2 + 2])[0]

        # Calculate angle for the current spot
        angle_rad = np.radians(start_angle + (i * angular_res))

        # Convert Polar (dist, angle) to Cartesian (x, y)
        # Assuming 2D scan, Z is 0
        x = (dist_mm / 1000.0) * np.cos(angle_rad)
        y = (dist_mm / 1000.0) * np.sin(angle_rad)
        points.append((x, y, 0.0))

    return points


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to LZR-U921 on {SERIAL_PORT}")

        all_captured_points = []
        scans_to_capture = 60  # Capture 1 second of data at 60Hz

        while len(all_captured_points) < scans_to_capture:
            # 1. Look for SYNC [cite: 91]
            if ser.read(1) == b'\xfc':
                if ser.read(3) == b'\xfd\xfe\xff':
                    # 2. Read SIZE (2 bytes)
                    size_bytes = ser.read(2)
                    msg_size = struct.unpack('<H', size_bytes)[0]

                    # 3. Read Message (CMD + DATA) [cite: 83]
                    msg_body = ser.read(msg_size)

                    # 4. Read Footer (CHK - 2 bytes) [cite: 112]
                    ser.read(2)

                    # CMD 50011 indicates Measured Distance Information [cite: 149]
                    cmd = struct.unpack('<H', msg_body[0:2])[0]
                    if cmd == 50011:
                        # Extract distance data (offset depends on enabled options)
                        # Standard MDI starts after CMD (2 bytes)
                        # and optional Plane Number (1 byte) [cite: 198, 232]
                        raw_distances = msg_body[3:]
                        scan_points = parse_mdi(raw_distances)
                        all_captured_points.extend(scan_points)
                        print(f"Captured scan {len(all_captured_points) // 274}")

        save_to_pcd(all_captured_points, "lzr_scan.pcd")
        print("File 'lzr_scan.pcd' saved successfully.")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
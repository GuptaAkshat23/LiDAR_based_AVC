import serial
import struct
import numpy as np
import time
import os
import datetime

# --- CONFIGURATION ---
SERIAL_PORT = 'COM4'  # Change to COMx for Windows
BAUD_RATE = 921600  # Default high-speed baud
SYNC_HEADER = b'\xfc\xfd\xfe\xff'  # OxFFFE FDFC in little-endian

# Stacking Configuration
# X_INCREMENT controls the "speed" of the stack.
# 0.10 means 10cm of movement between every scan frame.
X_INCREMENT = 0.10


def save_to_pcd(points, filename="lzr_scan_stacked.pcd"):
    """Saves a list of (x, y, z) points to a PCD file format."""
    if not points:
        print("No points to save.")
        return

    # Generate a timestamped filename so we don't overwrite previous scans
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"lzr_scan_{timestamp}.pcd"

    print(f"Saving {len(points)} points to {filename}...")

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
    print("Save complete.")


def parse_mdi(data_bytes):
    """Converts raw distance values (mm) to Cartesian (sensor_x, sensor_y) coordinates."""
    points_2d = []
    num_values = len(data_bytes) // 2

    # Angular resolution is ~0.3516°
    # Starting angle is -48°
    start_angle = -48.0
    angular_res = 0.3516

    for i in range(num_values):
        # Each distance is encoded via 2 bytes (little-endian)
        dist_mm = struct.unpack('<H', data_bytes[i * 2: i * 2 + 2])[0]

        # Ignore noise / max range (filter out points < 5cm or > 20m)
        if dist_mm < 50 or dist_mm > 20000:
            continue

        # Calculate angle for the current spot
        angle_rad = np.radians(start_angle + (i * angular_res))

        # Convert Polar (dist, angle) to Cartesian (sensor_x, sensor_y)
        # matches your original code logic:
        # x = dist * cos(angle)
        # y = dist * sin(angle)
        sensor_x = (dist_mm / 1000.0) * np.cos(angle_rad)
        sensor_y = (dist_mm / 1000.0) * np.sin(angle_rad)

        points_2d.append((sensor_x, sensor_y))

    return points_2d


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to LZR-U921 on {SERIAL_PORT}")
        print("Starting continuous capture... Press Ctrl+C to stop and save.")

        all_captured_points = []
        scan_counter = 0

        while True:
            # 1. Look for SYNC
            if ser.read(1) == b'\xfc':
                if ser.read(3) == b'\xfd\xfe\xff':
                    # 2. Read SIZE (2 bytes)
                    size_bytes = ser.read(2)
                    msg_size = struct.unpack('<H', size_bytes)[0]

                    # 3. Read Message (CMD + DATA)
                    msg_body = ser.read(msg_size)

                    # 4. Read Footer (CHK - 2 bytes)
                    ser.read(2)

                    # CMD 50011 indicates Measured Distance Information
                    cmd = struct.unpack('<H', msg_body[0:2])[0]
                    if cmd == 50011:
                        # Extract distance data (offset depends on enabled options)
                        # Standard MDI starts after CMD (2 bytes)
                        # and optional Plane Number (1 byte)
                        raw_distances = msg_body[3:]

                        # Get the 2D points from the sensor
                        scan_2d = parse_mdi(raw_distances)

                        # --- STACKING LOGIC ---
                        # Calculate the 'Travel' position (World X)
                        # This simulates the vehicle moving through the toll plaza
                        travel_x = scan_counter * X_INCREMENT

                        for (sensor_x, sensor_y) in scan_2d:
                            # Map to 3D World Coordinates:
                            # World X = Travel Direction (Time)
                            # World Y = Sensor X (Width)
                            # World Z = Sensor Y (Height)
                            all_captured_points.append((travel_x, sensor_x, sensor_y))

                        scan_counter += 1

                        # Print status every 20 scans so you know it's working
                        if scan_counter % 20 == 0:
                            print(f"Captured {scan_counter} frames... Total points: {len(all_captured_points)}")

    except KeyboardInterrupt:
        print("\nStop signal received.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        # Save whatever we captured
        save_to_pcd(all_captured_points, "lzr_stacked_output.pcd")
        print("File saved successfully.")


if __name__ == "__main__":
    main()
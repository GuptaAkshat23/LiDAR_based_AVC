import serial
import time
import numpy as np
import datetime
import os

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM4'  # Check your device manager
BAUD_RATE = 460800
POINTS_PER_SCAN = 274

# !!! ADJUST THIS TO FLATTEN THE CURVE !!!
# Default is 96.0. If wall is curved, change this value.
FOV_DEGREES = 96.0

SAVE_DIRECTORY = "./scans"


# ==========================================
# PCD SAVING FUNCTION
# ==========================================
def save_pcd(distances_mm, output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = os.path.join(output_folder, f"lzr_scan_{timestamp}.pcd")

    # Generate angles based on the adjustable FOV
    angles = np.linspace(
        np.radians(-FOV_DEGREES / 2),
        np.radians(FOV_DEGREES / 2),
        len(distances_mm)
    )

    valid_points = []

    for r, theta in zip(distances_mm, angles):
        r_meters = r / 1000.0  # Convert mm to meters

        # Filter valid range (0.1m to 20m)
        # Increased upper limit slightly in case box is large
        if 0.1 < r_meters < 20.0:
            # Polar to Cartesian
            # We swap Sin/Cos logic if the sensor is mounted vertically vs horizontally
            # Standard Top-Down: X=Forward (Cos), Y=Left/Right (Sin)
            x = r_meters * np.cos(theta)
            y = r_meters * np.sin(theta)
            z = 0.0
            valid_points.append((x, y, z))

    # Save to file if we have data
    if len(valid_points) > 0:
        with open(filename, 'w') as f:
            f.write("# .PCD v.7 - LZR U921 Scan Data\n")
            f.write("VERSION .7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(valid_points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(valid_points)}\n")
            f.write("DATA ascii\n")

            for p in valid_points:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

        print(f"[SAVED] {filename} | Points: {len(valid_points)} | FOV Used: {FOV_DEGREES}")


# ==========================================
# MAIN READER LOOP
# ==========================================
def run_recorder():
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to LZR U921 on {SERIAL_PORT}")
        print(f"Current FOV Setting: {FOV_DEGREES} degrees")
        print("Waiting for data... (Press Ctrl+C to stop)")

        buffer = b""

        while True:
            if ser.in_waiting > 0:
                buffer += ser.read(ser.in_waiting)

                # Find Header (LZR usually starts with 0x02 STX)
                start_index = buffer.find(b'\x02')

                if start_index != -1:
                    buffer = buffer[start_index:]

                    # Expected size: Header(4) + Data(274*2) + Checksum(2) approx
                    required_size = 4 + (POINTS_PER_SCAN * 2) + 2

                    if len(buffer) >= required_size:
                        # Extract payload (skip header bytes, usually 4 bytes)
                        # Adjust '4' if your specific firmware version has a different header length
                        raw_payload = buffer[4: 4 + (POINTS_PER_SCAN * 2)]

                        distances = []
                        for i in range(0, len(raw_payload) - 1, 2):
                            high = raw_payload[i]
                            low = raw_payload[i + 1]
                            dist = (high << 8) + low
                            distances.append(dist)

                        if len(distances) == POINTS_PER_SCAN:
                            save_pcd(distances, SAVE_DIRECTORY)

                        # Remove processed frame from buffer
                        buffer = buffer[required_size:]

            time.sleep(0.01)

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except KeyboardInterrupt:
        print("\nStopping recorder...")
    finally:
        if ser and ser.is_open:
            ser.close()


if __name__ == "__main__":
    run_recorder()
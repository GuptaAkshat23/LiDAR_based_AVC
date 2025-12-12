import serial
import time
import numpy as np
import datetime
import binascii
import os

# ==========================================
# CONFIGURATION SETTINGS
# ==========================================
SERIAL_PORT = 'COM3'  # Change to your port (e.g., '/dev/ttyUSB0' on Linux)
BAUD_RATE = 460800  # Standard LZR U921 baud rate
POINTS_PER_SCAN = 274  # Standard LZR resolution
FOV_DEGREES = 96.0  # Field of View
SAVE_DIRECTORY = "./scans"  # Folder where files will be saved


# ==========================================
# PCD SAVING FUNCTION
# ==========================================
def save_pcd(distances_mm, output_folder):
    """
    Converts polar distances to Cartesian X/Y/Z and saves a PCD file.
    """
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # 1. Generate Timestamps for unique filename
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = os.path.join(output_folder, f"lzr_scan_{timestamp}.pcd")

    # 2. Math: Polar to Cartesian
    # Create an array of angles from -48 to +48 degrees
    angles = np.linspace(
        np.radians(-FOV_DEGREES / 2),
        np.radians(FOV_DEGREES / 2),
        len(distances_mm)
    )

    valid_points = []

    for r, theta in zip(distances_mm, angles):
        r_meters = r / 1000.0  # Convert mm to meters

        # Filter out error codes (0 or max value usually indicates error)
        if 0.1 < r_meters < 60.0:
            # Polar to Cartesian formulas
            x = r_meters * np.cos(theta)
            y = r_meters * np.sin(theta)
            z = 0.0  # 2D Sensor, so Z is flat
            valid_points.append((x, y, z))

    # 3. Write File
    with open(filename, 'w') as f:
        # PCD Header
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

        # PCD Data
        for p in valid_points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print(f"[SUCCESS] Saved {len(valid_points)} points to: {filename}")


# ==========================================
# MAIN READER LOOP
# ==========================================
def run_recorder():
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=1
        )
        print(f"Connected to LZR U921 on {SERIAL_PORT}")
        print("Waiting for data... (Press Ctrl+C to stop)")

        buffer = b""

        while True:
            # Read all available bytes
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer += chunk

                # --- PARSING LOGIC START ---
                # NOTE: This logic assumes the LZR standard frame structure.
                # You may need to adjust the 'start_byte' if your specific
                # firmware version uses a different header (e.g., 0x02 or 0xFF).

                # Look for a start byte (Example: 0x02 STX)
                start_index = buffer.find(b'\x02')

                if start_index != -1:
                    # Trim buffer to start at the header
                    buffer = buffer[start_index:]

                    # We expect 2 bytes per point * 274 points + Overhead (Header/Checksum)
                    # Approx frame size ~ 550-600 bytes
                    EXPECTED_SIZE = (POINTS_PER_SCAN * 2) + 10  # Estimate

                    if len(buffer) >= EXPECTED_SIZE:
                        # Extract the data payload (Assuming data starts at byte 4)
                        # Adjust '4' based on actual protocol documentation
                        raw_payload = buffer[4: 4 + (POINTS_PER_SCAN * 2)]

                        # Convert Raw Bytes to Integers (Distance in mm)
                        # LZR sends High Byte first, then Low Byte (Big Endian)
                        distances = []
                        for i in range(0, len(raw_payload) - 1, 2):
                            high_byte = raw_payload[i]
                            low_byte = raw_payload[i + 1]
                            dist = (high_byte << 8) + low_byte
                            distances.append(dist)

                        # --- TRIGGER SAVE ---
                        # Automatically save the valid frame
                        if len(distances) == POINTS_PER_SCAN:
                            save_pcd(distances, SAVE_DIRECTORY)

                        # Clear processed part of buffer
                        buffer = buffer[EXPECTED_SIZE:]

            time.sleep(0.01)

    except serial.SerialException as e:
        print(f"Error: Could not open port {SERIAL_PORT}. Check connection.")
    except KeyboardInterrupt:
        print("\nRecorder stopped by user.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    run_recorder()
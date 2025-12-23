import serial
import time
import numpy as np
import datetime
import os
import struct

# --- CONFIGURATION ---
SERIAL_PORT = 'COM4'  # CHECK YOUR COM PORT
BAUD_RATE = 460800
POINTS_PER_SCAN = 274
FOV_DEGREES = 96.0
STACK_SIZE = 100  # Frames per file
SIMULATED_SPEED = 0.05  # Meters per frame step
SAVE_DIRECTORY = "./raw_scans"

# Constants
SYNC_WORD = b'\xfc\xfd\xfe\xff'


def save_organized_pcd(frame_stack, output_folder, filename_prefix="scan"):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    timestamp = datetime.datetime.now().strftime("%H%M%S")
    filename = os.path.join(output_folder, f"{filename_prefix}_{timestamp}.pcd")

    # Prepare data arrays
    # An Organized Cloud has a Height (Stack Size) and Width (Points per Scan)
    width = POINTS_PER_SCAN
    height = len(frame_stack)
    total_points = width * height

    angles = np.linspace(np.radians(-FOV_DEGREES / 2), np.radians(FOV_DEGREES / 2), width)

    # Store XYZ string lines
    data_lines = []

    for frame_idx, distances_mm in enumerate(frame_stack):
        y_pos = frame_idx * SIMULATED_SPEED  # Y is movement direction

        for r_mm, theta in zip(distances_mm, angles):
            r_meters = r_mm / 1000.0

            # Keep ALL points. If invalid, set to NaN (Not a Number)
            if 0.1 < r_meters < 65.0:
                x = r_meters * np.sin(theta)
                z = r_meters * np.cos(theta)
                data_lines.append(f"{x:.4f} {y_pos:.4f} {z:.4f}")
            else:
                # IMPORTANT: We save "nan nan nan" to keep the grid alignment
                data_lines.append("nan nan nan")

    # Write Header for ORGANIZED Cloud
    with open(filename, 'w') as f:
        f.write("# .PCD v.7 - LZR Organized\n")
        f.write("VERSION .7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {width}\n")  # Columns
        f.write(f"HEIGHT {height}\n")  # Rows
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {total_points}\n")
        f.write("DATA ascii\n")
        f.write("\n".join(data_lines))

    print(f"[SAVED] Organized PCD: {filename}")


def parse_frame_distances(payload):
    if len(payload) < POINTS_PER_SCAN * 2: return [0] * POINTS_PER_SCAN
    return [struct.unpack('<H', payload[i:i + 2])[0] for i in range(0, POINTS_PER_SCAN * 2, 2)]


def run_capture():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to LZR U921 on {SERIAL_PORT}. Waiting for data...")

        buffer = b""
        current_stack = []

        while True:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)

                while SYNC_WORD in buffer:
                    parts = buffer.split(SYNC_WORD, 1)
                    if len(parts) > 1:
                        buffer = parts[1]
                        if len(buffer) < 4:
                            buffer = SYNC_WORD + buffer
                            break

                        msg_size = struct.unpack('<H', buffer[0:2])[0]
                        total_expected = 4 + msg_size + 2

                        if len(buffer) >= total_expected:
                            frame_data = buffer[:total_expected]
                            buffer = buffer[total_expected:]
                            payload = frame_data[4:-2]

                            dists = parse_frame_distances(payload)

                            # Ensure we always have exactly 274 points per row
                            if len(dists) == POINTS_PER_SCAN:
                                current_stack.append(dists)
                            else:
                                # Pad if error occurs to maintain sync
                                current_stack.append([0] * POINTS_PER_SCAN)

                            if len(current_stack) >= STACK_SIZE:
                                save_organized_pcd(current_stack, SAVE_DIRECTORY)
                                current_stack = []
                        else:
                            buffer = SYNC_WORD + buffer
                            break
                    else:
                        break
    except KeyboardInterrupt:
        if current_stack: save_organized_pcd(current_stack, SAVE_DIRECTORY, "partial")
        print("\nStopped.")


if __name__ == "__main__":
    run_capture()
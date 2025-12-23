import serial
import time
import numpy as np
import datetime
import os
import struct
import open3d as o3d

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM4'  # Ensure this matches your setup
BAUD_RATE = 460800
POINTS_PER_SCAN = 274  # U921 resolution
FOV_DEGREES = 96.0

# IMPORTANT: These must match your 'lzr_capture_organized.py' settings!
# This ensures the Zero file is the exact same shape as your Object file.
STACK_SIZE = 100
SIMULATED_SPEED = 0.05

# How many frames to read to calculate the average background?
CALIBRATION_FRAMES = 300

SYNC_WORD = b'\xfc\xfd\xfe\xff'


# ==========================================
# PARSING LOGIC (Same as Reader)
# ==========================================
def parse_frame_distances(payload):
    if len(payload) < POINTS_PER_SCAN * 2:
        return None
    return [struct.unpack('<H', payload[i:i + 2])[0] for i in range(0, POINTS_PER_SCAN * 2, 2)]


# ==========================================
# GENERATION LOGIC
# ==========================================
def save_extruded_zero_plane(avg_distances, output_filename):
    """
    Takes 1 "Perfect" row of distances and duplicates it STACK_SIZE times
    to create a full reference block compatible with the object scanner.
    """
    print(f"Generating Zero Reference from average data...")

    width = POINTS_PER_SCAN
    height = STACK_SIZE
    total_points = width * height

    angles = np.linspace(np.radians(-FOV_DEGREES / 2), np.radians(FOV_DEGREES / 2), width)

    # Pre-calculate X and Z for the single averaged row
    # (Since the sensor is stationary, X and Z don't change, only Y changes over time)
    base_row_x = []
    base_row_z = []

    for r_mm, theta in zip(avg_distances, angles):
        r_meters = r_mm / 1000.0

        if 0.1 < r_meters < 65.0:
            x = r_meters * np.sin(theta)
            z = r_meters * np.cos(theta)
            base_row_x.append(x)
            base_row_z.append(z)
        else:
            base_row_x.append(np.nan)
            base_row_z.append(np.nan)

    # Now generate the full file lines
    data_lines = []

    for frame_idx in range(height):
        # Calculate Y for this row (Time/Movement dimension)
        y_pos = frame_idx * SIMULATED_SPEED

        # Write the row
        for i in range(width):
            x = base_row_x[i]
            z = base_row_z[i]

            if np.isnan(x):
                data_lines.append("nan nan nan")
            else:
                # Note: We keep X and Z constant (perfect floor), only Y increments
                data_lines.append(f"{x:.4f} {y_pos:.4f} {z:.4f}")

    # Save to PCD
    with open(output_filename, 'w') as f:
        f.write("# .PCD v.7 - LZR Zero Reference\n")
        f.write("VERSION .7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {width}\n")
        f.write(f"HEIGHT {height}\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {total_points}\n")
        f.write("DATA ascii\n")
        f.write("\n".join(data_lines))

    print(f"SUCCESS! Zero Plane saved to: {output_filename}")


# ==========================================
# MAIN RECORDING LOOP
# ==========================================
def record_background():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to LZR U921 on {SERIAL_PORT}")
        print(f"Ensure the detection area is EMPTY.")
        print(f"Calibrating background using {CALIBRATION_FRAMES} frames...")

        buffer = b""
        accumulator = np.zeros(POINTS_PER_SCAN)
        counts = np.zeros(POINTS_PER_SCAN)
        frames_captured = 0

        while frames_captured < CALIBRATION_FRAMES:
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

                            if dists and len(dists) == POINTS_PER_SCAN:
                                # Convert to numpy for easier math
                                dist_arr = np.array(dists)

                                # Filter invalid (0 or max) before adding
                                valid_mask = (dist_arr > 100) & (dist_arr < 65000)

                                accumulator[valid_mask] += dist_arr[valid_mask]
                                counts[valid_mask] += 1

                                frames_captured += 1
                                if frames_captured % 50 == 0:
                                    print(f"Captured {frames_captured}/{CALIBRATION_FRAMES} frames...")
                        else:
                            buffer = SYNC_WORD + buffer
                            break
                    else:
                        break

        # Calculate Average
        # Avoid divide by zero if a point was never seen (set to 0)
        counts[counts == 0] = 1
        avg_distances = accumulator / counts

        # Save output
        save_extruded_zero_plane(avg_distances, "zero_ref.pcd")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    record_background()
import serial
import time
import numpy as np
import datetime
import os
import struct

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM4'  # Change to your port
BAUD_RATE = 460800  # U921 Standard [cite: 80]
POINTS_PER_SCAN = 274  # Max points per plane [cite: 36]
FOV_DEGREES = 96.0  # Field of View [cite: 36]
STACK_SIZE = 100  # How many frames to stack before saving one PCD file
SIMULATED_SPEED = 0.05  # Meters per frame (simulates vehicle moving past sensor)
SAVE_DIRECTORY = "./stacked_scans"

# Protocol Constants
SYNC_WORD = b'\xfc\xfd\xfe\xff'  # 0xFFFEFDFC in Little Endian


# ==========================================
# PCD SAVING FUNCTION (STACKED)
# ==========================================
def save_stacked_pcd(frame_stack, output_folder):
    """
    Saves a stack of frames as a single 3D Point Cloud.
    frame_stack: List of lists (or 2D array) containing distance_mm for each frame.
    """
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_folder, f"lzr_stack_{timestamp}.pcd")

    valid_points = []

    # Generate angles for one scan line (-48 to +48 degrees)
    angles = np.linspace(
        np.radians(-FOV_DEGREES / 2),
        np.radians(FOV_DEGREES / 2),
        POINTS_PER_SCAN
    )

    # --- STACKING LOOP ---
    # y_offset grows with each frame to simulate movement (AVC Logic)
    for frame_idx, distances_mm in enumerate(frame_stack):

        # Calculate Y position (Time dimension)
        y_pos = frame_idx * SIMULATED_SPEED

        for r_mm, theta in zip(distances_mm, angles):
            r_meters = r_mm / 1000.0

            # Filter valid range (U921 max is 65m [cite: 35], min detection ~2% remission at 10m [cite: 63])
            if 0.1 < r_meters < 65.0:
                # Polar to Cartesian conversion
                # X: Lateral position
                # Z: Height/Depth (Sensor scan plane)
                # Y: Longitudinal position (Stacked frames)
                x = r_meters * np.sin(theta)
                z = r_meters * np.cos(theta)

                valid_points.append((x, y_pos, z))

    # --- WRITE HEADER & DATA ---
    with open(filename, 'w') as f:
        f.write("# .PCD v.7 - LZR U921 Stacked Data\n")
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

    print(f"[SAVED] Stack of {len(frame_stack)} frames saved to: {filename} ({len(valid_points)} points)")


# ==========================================
# PARSER UTILS
# ==========================================
def parse_frame_distances(message_payload):
    """
    Extracts distance values from the Message part of the frame.
    According to protocol, distances are 2 bytes, Little Endian[cite: 86, 209].
    """
    distances = []

    # The payload might contain optional data (ID, Logs) before MDI.
    # We assume standard MDI mode here. If you enabled "Plane Number",
    # you might need to skip 1 byte.
    # For Raw MDI: Just read 2 bytes at a time.

    # Safety check for payload length
    if len(message_payload) < POINTS_PER_SCAN * 2:
        return None

    for i in range(0, POINTS_PER_SCAN * 2, 2):
        # Little Endian Unpacking (<H)
        dist_mm = struct.unpack('<H', message_payload[i:i + 2])[0]
        distances.append(dist_mm)

    return distances


# ==========================================
# MAIN STACKED READER
# ==========================================
def run_stacked_recorder():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to LZR U921 on {SERIAL_PORT}")
        print(f"Collecting stacks of {STACK_SIZE} frames...")

        buffer = b""
        current_stack = []

        while True:
            if ser.in_waiting > 0:
                buffer += ser.read(ser.in_waiting)

                # 1. Find SYNC Word
                while SYNC_WORD in buffer:
                    # Split buffer at SYNC to isolate the frame start
                    # The split removes the SYNC, so we handle the 'post-sync' data
                    parts = buffer.split(SYNC_WORD, 1)

                    # parts[0] is garbage before sync, parts[1] is potential frame
                    if len(parts) > 1:
                        buffer = parts[1]  # Keep the rest

                        # We need at least Header (4 bytes) to know the SIZE
                        # Header structure: SIZE (2 bytes) + CMD (2 bytes) [cite: 83]
                        if len(buffer) < 4:
                            # Not enough data yet, wait for next loop
                            buffer = SYNC_WORD + buffer  # Put SYNC back
                            break

                            # Read SIZE (First 2 bytes after SYNC) - Little Endian [cite: 95]
                        msg_size = struct.unpack('<H', buffer[0:2])[0]

                        # Total frame expected = Header(4) + Message(msg_size) + Footer(2) [cite: 83]
                        total_expected = 4 + msg_size + 2

                        if len(buffer) >= total_expected:
                            # Extract FULL Frame (excluding SYNC which is already stripped)
                            # Frame content: [SIZE_CMD_DATA_CHK]
                            frame_data = buffer[:total_expected]

                            # Remove this frame from main buffer
                            buffer = buffer[total_expected:]

                            # 2. Extract MDI (Data)
                            # Header is SIZE(2) + CMD(2) = 4 bytes offset
                            # Footer is CHK(2)
                            # Payload is in between
                            payload = frame_data[4:-2]

                            # 3. Parse Distances
                            distances = parse_frame_distances(payload)

                            if distances and len(distances) == POINTS_PER_SCAN:
                                current_stack.append(distances)

                                # 4. Check Stack Full
                                if len(current_stack) >= STACK_SIZE:
                                    save_stacked_pcd(current_stack, SAVE_DIRECTORY)
                                    current_stack = []  # Reset stack
                        else:
                            # Frame incomplete, wait for more bytes
                            buffer = SYNC_WORD + buffer  # Put SYNC back
                            break
                    else:
                        break

    except KeyboardInterrupt:
        print("\nStopping...")
        if len(current_stack) > 0:
            print("Saving partial stack...")
            save_stacked_pcd(current_stack, SAVE_DIRECTORY)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    run_stacked_recorder()
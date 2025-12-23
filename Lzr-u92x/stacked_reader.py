import serial
import time
import numpy as np
import datetime
import os
import struct
import sys

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM4'  # CHECK YOUR COM PORT
BAUD_RATE = 460800  # Standard for U921
POINTS_PER_SCAN = 274  # Resolution
FOV_DEGREES = 96.0
STACK_SIZE = 100  # How many frames per PCD file
SIMULATED_SPEED = 0.05  # Meters per frame
SAVE_DIRECTORY = "./raw_scans"

# Constants
SYNC_WORD = b'\xfc\xfd\xfe\xff'

# Analytics Config
UPDATE_INTERVAL = 0.5  # Update screen every 0.5 seconds


# ==========================================
# PCD SAVING (Unchanged)
# ==========================================
def save_organized_pcd(frame_stack, output_folder, filename_prefix="scan"):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    timestamp = datetime.datetime.now().strftime("%H%M%S")
    filename = os.path.join(output_folder, f"{filename_prefix}_{timestamp}.pcd")

    width = POINTS_PER_SCAN
    height = len(frame_stack)
    total_points = width * height

    angles = np.linspace(np.radians(-FOV_DEGREES / 2), np.radians(FOV_DEGREES / 2), width)
    data_lines = []

    for frame_idx, distances_mm in enumerate(frame_stack):
        y_pos = frame_idx * SIMULATED_SPEED

        for r_mm, theta in zip(distances_mm, angles):
            r_meters = r_mm / 1000.0
            if 0.1 < r_meters < 65.0:
                x = r_meters * np.sin(theta)
                z = r_meters * np.cos(theta)
                data_lines.append(f"{x:.4f} {y_pos:.4f} {z:.4f}")
            else:
                data_lines.append("nan nan nan")

    with open(filename, 'w') as f:
        f.write("# .PCD v.7 - LZR Organized\n")
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

    return filename


# ==========================================
# PARSER UTILS
# ==========================================
def parse_frame_distances(payload):
    if len(payload) < POINTS_PER_SCAN * 2: return [0] * POINTS_PER_SCAN
    return [struct.unpack('<H', payload[i:i + 2])[0] for i in range(0, POINTS_PER_SCAN * 2, 2)]


# ==========================================
# ANALYTICS DISPLAY FUNCTION
# ==========================================
def print_dashboard(start_time, total_frames, current_stack_len, saved_count, bad_frames, buffer_size, last_saved_file):
    uptime = time.time() - start_time
    avg_fps = total_frames / uptime if uptime > 0 else 0

    # Create a simple progress bar [======    ]
    bar_len = 20
    filled_len = int(bar_len * current_stack_len // STACK_SIZE)
    bar = '=' * filled_len + '-' * (bar_len - filled_len)

    # Clear screen (Windows 'cls' or Linux 'clear') - Optional, can be flickery
    # Instead, we will print using carriage return \r to overwrite lines if possible,
    # but for simple scrolling logs, a formatted block is safer.

    print(f"\n--- LZR U921 LIVE ANALYTICS ---")
    print(f"⏱  Uptime:       {int(uptime)} sec")
    print(f"⚡ Speed:        {avg_fps:.1f} FPS")
    print(f"📊 Total Frames: {total_frames}")
    print(f"💾 Files Saved:  {saved_count}")
    print(f"❌ Bad Frames:   {bad_frames} (Integrity Loss)")
    print(f"🧠 Buffer Load:  {buffer_size} bytes (Should correspond to baud speed)")
    print(f"📥 Stack Status: [{bar}] {current_stack_len}/{STACK_SIZE}")
    if last_saved_file:
        print(f"✅ Last Save:    {os.path.basename(last_saved_file)}")
    print("-------------------------------")


# ==========================================
# MAIN LOOP
# ==========================================
def run_capture():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to LZR U921 on {SERIAL_PORT} @ {BAUD_RATE} baud.")

        # Stats Variables
        start_time = time.time()
        last_update_time = time.time()
        total_frames = 0
        bad_frames = 0
        saved_files_count = 0
        last_file = None

        buffer = b""
        current_stack = []

        while True:
            # Check buffer
            if ser.in_waiting:
                # Read data
                new_data = ser.read(ser.in_waiting)
                buffer += new_data

                # PROCESSING LOOP
                while SYNC_WORD in buffer:
                    parts = buffer.split(SYNC_WORD, 1)
                    if len(parts) > 1:
                        buffer = parts[1]

                        # Wait for Header
                        if len(buffer) < 4:
                            buffer = SYNC_WORD + buffer
                            break

                        msg_size = struct.unpack('<H', buffer[0:2])[0]
                        total_expected = 4 + msg_size + 2

                        # Wait for Full Frame
                        if len(buffer) >= total_expected:
                            frame_data = buffer[:total_expected]
                            buffer = buffer[total_expected:]
                            payload = frame_data[4:-2]

                            dists = parse_frame_distances(payload)

                            # --- ANALYTICS: CHECK DATA INTEGRITY ---
                            if len(dists) == POINTS_PER_SCAN:
                                current_stack.append(dists)
                                total_frames += 1
                            else:
                                bad_frames += 1  # Frame was parsed but size was wrong
                                current_stack.append([0] * POINTS_PER_SCAN)  # Pad to keep alignment

                            # --- CHECK STACK FULL ---
                            if len(current_stack) >= STACK_SIZE:
                                last_file = save_organized_pcd(current_stack, SAVE_DIRECTORY)
                                saved_files_count += 1
                                current_stack = []
                        else:
                            # Incomplete frame
                            buffer = SYNC_WORD + buffer
                            break
                    else:
                        break

            # --- UPDATE DASHBOARD (Every 0.5 seconds) ---
            if time.time() - last_update_time > UPDATE_INTERVAL:
                print_dashboard(
                    start_time,
                    total_frames,
                    len(current_stack),
                    saved_files_count,
                    bad_frames,
                    len(buffer),  # Monitor if buffer is exploding
                    last_file
                )
                last_update_time = time.time()

    except KeyboardInterrupt:
        if current_stack:
            save_organized_pcd(current_stack, SAVE_DIRECTORY, "partial")
        print("\nStopped by user.")
    except Exception as e:
        print(f"\nCRITICAL ERROR: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    run_capture()
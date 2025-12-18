import serial
import time
import numpy as np
import os
import sys

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM5'
BAUD_RATE = 460800
POINTS_PER_SCAN = 274
EXPECTED_PACKET_SIZE = (POINTS_PER_SCAN * 2) + 10

OUTPUT_FOLDER = "zero_plane"
MODEL_FILENAME = "statistical_background.npz"

FRAMES_TO_CAPTURE = 500  # More frames = Better statistics


def run_recorder():
    print(f"--- 📊 STATISTICAL BACKGROUND RECORDER ---")
    print(f"Capturing {FRAMES_TO_CAPTURE} frames to model sensor noise...")

    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(1)

    # Buffer to hold all raw scans: Shape (N, 274)
    raw_scans = []
    serial_buf = b""

    try:
        while len(raw_scans) < FRAMES_TO_CAPTURE:
            if ser.in_waiting:
                serial_buf += ser.read(ser.in_waiting)

            while True:
                # Find Header (LZR standard 0x02)
                idx = serial_buf.find(b'\x02')
                if idx == -1:
                    serial_buf = b""
                    break

                if idx > 0:
                    serial_buf = serial_buf[idx:]

                if len(serial_buf) < EXPECTED_PACKET_SIZE:
                    break

                # Extract Payload
                payload = serial_buf[4: 4 + (POINTS_PER_SCAN * 2)]

                # Fast Numpy Decoding
                raw_bytes = np.frombuffer(payload, dtype=np.uint8)
                high = raw_bytes[0::2].astype(np.uint16)
                low = raw_bytes[1::2].astype(np.uint16)
                distances = (high << 8) | low

                if len(distances) == POINTS_PER_SCAN:
                    # Filter basic errors (0 or max range)
                    # We treat error codes as NaN so they don't mess up stats
                    distances = distances.astype(float)
                    distances[distances < 100] = np.nan
                    distances[distances > 60000] = np.nan

                    raw_scans.append(distances)
                    sys.stdout.write(f"\rCaptured: {len(raw_scans)}/{FRAMES_TO_CAPTURE}")
                    sys.stdout.flush()

                serial_buf = serial_buf[EXPECTED_PACKET_SIZE:]

    except KeyboardInterrupt:
        print("\nStopped early.")
    finally:
        ser.close()

    # --- PROCESSING ---
    print("\n\n🧠 Computing Statistical Model...")
    data_matrix = np.array(raw_scans)  # Shape (Frames, 274)

    # 1. Calculate Mean (The "Zero Plane") ignoring NaNs
    bg_mean = np.nanmean(data_matrix, axis=0)

    # 2. Calculate Std Dev (The "Noise Level") ignoring NaNs
    bg_std = np.nanstd(data_matrix, axis=0)

    # 3. Safety: Fill remaining NaNs (dead zones) with 0
    bg_mean = np.nan_to_num(bg_mean)
    bg_std = np.nan_to_num(bg_std)

    # 4. Save Compressed Model
    save_path = os.path.join(OUTPUT_FOLDER, MODEL_FILENAME)
    np.savez(save_path, mean=bg_mean, std=bg_std)

    print(f"✅ Model Saved: {save_path}")
    print(f"   Mean Profile Shape: {bg_mean.shape}")
    print(f"   Noise Profile Shape: {bg_std.shape}")
    print("   (Use this file in the remover script)")


if __name__ == "__main__":
    run_recorder()
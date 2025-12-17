import serial
import time
import numpy as np
import os
import sys

# ==========================================
# 🎯 CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM4'
BAUD_RATE = 460800
POINTS_PER_SCAN = 274
FOV_DEGREES = 96.0

# CALIBRATION DURATION
# 10 Seconds ensures we capture enough data to average out ALL noise.
DURATION_SEC = 10.0

OUTPUT_FOLDER = "zero_plane"
FILENAME_PCD = "master_zero_plane.pcd"
FILENAME_NPY = "master_zero_plane.npy"  # For Python to load later


# ==========================================

def save_static_pcd(distances, folder, filename):
    """Saves a single static profile as a PCD file for visualization"""
    filepath = os.path.join(folder, filename)

    # Generate angles
    angles = np.linspace(
        np.radians(-FOV_DEGREES / 2),
        np.radians(FOV_DEGREES / 2),
        len(distances)
    )

    points = []
    for r, theta in zip(distances, angles):
        r_meters = r / 1000.0
        if 0.1 < r_meters < 60.0:
            # Sideways Mapping (Static Z=0)
            # X = 0 (No movement time)
            # Y = Width (Distance)
            # Z = Height (Angle)
            raw_x = r_meters * np.cos(theta)
            raw_y = r_meters * np.sin(theta)

            points.append((0.0, abs(raw_x), raw_y))

    with open(filepath, 'w') as f:
        f.write("# .PCD v.7 - LZR MASTER ZERO PLANE\n")
        f.write("VERSION .7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for p in points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print(f"✅ Visual file saved: {filepath}")


def record_zero_plane():
    print("--- 🎯 RECORDING PERFECT ZERO PLANE ---")
    print(f"Time: {DURATION_SEC} seconds")
    print("👉 ACTION: CLEAR THE AREA. ENSURE NO MOVEMENT.")

    # Create folder
    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)
        print(f"📂 Created folder: {OUTPUT_FOLDER}")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("🔌 Connected. Warming up sensor...")
        time.sleep(1)  # Let serial stabilize

        scan_buffer = []
        start_time = time.time()

        # --- CAPTURE LOOP ---
        print("🔴 RECORDING STARTED...")
        while time.time() - start_time < DURATION_SEC:
            if ser.in_waiting:
                buffer = ser.read(ser.in_waiting)

                # Basic Parsing Logic
                idx = buffer.find(b'\x02')
                if idx != -1 and len(buffer) >= idx + (POINTS_PER_SCAN * 2) + 10:
                    payload = buffer[idx + 4: idx + 4 + (POINTS_PER_SCAN * 2)]
                    high = np.frombuffer(payload[0::2], dtype=np.uint8)
                    low = np.frombuffer(payload[1::2], dtype=np.uint8)
                    distances = (high.astype(np.uint16) << 8) + low

                    if len(distances) == POINTS_PER_SCAN:
                        scan_buffer.append(distances)
                        sys.stdout.write(f"\r⏳ Captured Frames: {len(scan_buffer)}")
                        sys.stdout.flush()

                    # Clear buffer processed part
                    # (In a simple script, flushing input is safer to stay real-time)
                    ser.reset_input_buffer()

        print("\n\n🛑 Recording Complete.")

        # --- PROCESSING FOR PERFECTION ---
        if len(scan_buffer) < 50:
            print("❌ Error: Not enough data captured. Check wiring.")
            return

        print("🧠 Calculating Statistical Median (Noise Removal)...")
        data_matrix = np.array(scan_buffer)

        # 1. MEDIAN FILTER
        # This removes transient noise (birds, dust, flicker)
        perfect_profile = np.median(data_matrix, axis=0)

        # 2. STANDARD DEVIATION CHECK (Quality Control)
        # We check how much the wall 'wobbled' during recording
        noise_level = np.std(data_matrix, axis=0)
        avg_noise = np.mean(noise_level)
        print(f"📊 Quality Report:")
        print(f"   Frames Used: {len(scan_buffer)}")
        print(f"   Average Sensor Jitter: {avg_noise:.2f} mm")

        if avg_noise > 50:
            print("⚠️ WARNING: High vibration detected! Mount might be loose.")
        else:
            print("✅ ZERO PLANE IS STABLE.")

        # --- SAVING ---
        # 1. Save as Numpy (For the actual recorder to load later)
        npy_path = os.path.join(OUTPUT_FOLDER, FILENAME_NPY)
        np.save(npy_path, perfect_profile)
        print(f"💾 Data file saved: {npy_path}")

        # 2. Save as PCD (For you to look at)
        save_static_pcd(perfect_profile, OUTPUT_FOLDER, FILENAME_PCD)

    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    record_zero_plane()
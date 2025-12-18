import serial
import time
import numpy as np
import os
import sys

# ==========================================
# 🔧 DENSE CALIBRATION TOOL
# ==========================================
SERIAL_PORT = 'COM5'
BAUD_RATE = 460800
POINTS_PER_SCAN = 274

# 🔴 UPDATED RANGE: Higher angles to fix the ')' shape
# We test from 90.0 up to 104.0
TEST_ANGLES = [90.0, 92.0, 94.0, 96.0, 98.0, 100.0, 102.0, 104.0]

# 🔴 DENSITY: How many frames to stack?
# 100 frames * 274 points = 27,400 points per file (Very visible)
FRAMES_TO_STACK = 100

OUTPUT_FOLDER = "fov_dense_test"


# ==========================================

def save_dense_pcd(all_frames, fov_deg, filename):
    filepath = os.path.join(OUTPUT_FOLDER, filename)

    # 1. Pre-calculate angles for this FOV
    angles = np.linspace(
        np.radians(-fov_deg / 2),
        np.radians(fov_deg / 2),
        POINTS_PER_SCAN
    )
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)

    points = []

    # 2. Process ALL frames (Don't average, keep everything)
    for distances in all_frames:
        # Filter valid data
        valid_mask = (distances > 100) & (distances < 20000)

        r_m = distances[valid_mask] / 1000.0

        # Apply math to this specific frame
        # X = Depth, Y = Width
        x = r_m * cos_a[valid_mask]
        y = r_m * sin_a[valid_mask]

        # Stack them at Z=0 so they form a thick line
        z = np.zeros_like(x)

        # Add to list
        frame_pts = np.column_stack((x, y, z))
        points.extend(frame_pts)

    # 3. Save HUGE file
    num_points = len(points)
    with open(filepath, 'w') as f:
        f.write(f"# .PCD v.7 - DENSE TEST {fov_deg}\n")
        f.write("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {num_points}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\nDATA ascii\n")
        for p in points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print(f"   👉 Generated: {filename} ({num_points} points)")


def run_tuning():
    print(f"--- 📐 DENSE FOV TUNER ---")
    print(f"Testing Angles: {TEST_ANGLES}")
    print(f"Capturing {FRAMES_TO_STACK} frames for maximum visibility...")

    if not os.path.exists(OUTPUT_FOLDER): os.makedirs(OUTPUT_FOLDER)

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(1)

        # --- 1. CAPTURE DATA (RAW) ---
        raw_frames = []
        serial_buf = b""
        start_time = time.time()

        print("📸 Recording Wall...")
        while len(raw_frames) < FRAMES_TO_STACK:
            if ser.in_waiting: serial_buf += ser.read(ser.in_waiting)

            while True:
                idx = serial_buf.find(b'\x02')
                if idx == -1:
                    serial_buf = b""
                    break
                if idx > 0: serial_buf = serial_buf[idx:]

                EXPECTED = (POINTS_PER_SCAN * 2) + 10
                if len(serial_buf) < EXPECTED: break

                payload = serial_buf[4: 4 + (POINTS_PER_SCAN * 2)]
                high = np.frombuffer(payload[0::2], dtype=np.uint8)
                low = np.frombuffer(payload[1::2], dtype=np.uint8)
                dists = (high.astype(np.uint16) << 8) + low

                if len(dists) == POINTS_PER_SCAN:
                    raw_frames.append(dists)
                    sys.stdout.write(f"\rCaptured: {len(raw_frames)}/{FRAMES_TO_STACK}")
                    sys.stdout.flush()

                serial_buf = serial_buf[EXPECTED:]

        ser.close()
        print("\n✅ Capture Complete. Processing...")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        return

    # --- 2. GENERATE FILES ---
    for angle in TEST_ANGLES:
        fname = f"dense_fov_{angle}.pcd"
        save_dense_pcd(raw_frames, angle, fname)

    print(f"\n📂 Files saved in '{OUTPUT_FOLDER}'")
    print("1. Open them in your visualizer.")
    print("2. You will see a THICK ribbon of points.")
    print("3. Find the one that is FLAT like a board, not curved like a banana.")


if __name__ == "__main__":
    run_tuning()
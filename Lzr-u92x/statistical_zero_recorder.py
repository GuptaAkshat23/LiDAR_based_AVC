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
FOV_DEGREES = 96.0
EXPECTED_PACKET_SIZE = (POINTS_PER_SCAN * 2) + 10

OUTPUT_FOLDER = "zero_plane"
FILENAME_PCD = "master_zero_plane.pcd"

FRAMES_TO_CAPTURE = 500  # High count for better stats


def run_recorder():
    print(f"--- 📊 STATISTICAL BACKGROUND RECORDER (PCD OUTPUT) ---")
    print(f"Capturing {FRAMES_TO_CAPTURE} frames...")

    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(1)

    raw_scans = []
    serial_buf = b""

    try:
        while len(raw_scans) < FRAMES_TO_CAPTURE:
            if ser.in_waiting:
                serial_buf += ser.read(ser.in_waiting)

            while True:
                idx = serial_buf.find(b'\x02')
                if idx == -1:
                    serial_buf = b""
                    break
                if idx > 0: serial_buf = serial_buf[idx:]

                if len(serial_buf) < EXPECTED_PACKET_SIZE:
                    break

                payload = serial_buf[4: 4 + (POINTS_PER_SCAN * 2)]
                raw = np.frombuffer(payload, dtype=np.uint8)
                high = raw[0::2].astype(np.uint16)
                low = raw[1::2].astype(np.uint16)
                distances = (high << 8) | low

                if len(distances) == POINTS_PER_SCAN:
                    d_float = distances.astype(float)
                    # Filter bad reads
                    d_float[d_float < 100] = np.nan
                    d_float[d_float > 60000] = np.nan

                    raw_scans.append(d_float)
                    sys.stdout.write(f"\rCaptured: {len(raw_scans)}/{FRAMES_TO_CAPTURE}")
                    sys.stdout.flush()

                serial_buf = serial_buf[EXPECTED_PACKET_SIZE:]

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()

    # --- PROCESSING ---
    print("\n\n🧠 Computing Statistics...")
    data = np.array(raw_scans)

    # 1. Compute Mean (The Wall Shape) & Std (The Noise)
    # Convert to METERS immediately for PCD
    bg_mean_mm = np.nanmean(data, axis=0)
    bg_std_mm = np.nanstd(data, axis=0)

    # Handle dead zones (NaN -> 0)
    bg_mean_mm = np.nan_to_num(bg_mean_mm)
    bg_std_mm = np.nan_to_num(bg_std_mm)

    bg_mean_m = bg_mean_mm / 1000.0
    bg_std_m = bg_std_mm / 1000.0

    # 2. Convert to Cartesian X/Y for PCD
    angles = np.linspace(
        np.radians(-FOV_DEGREES / 2),
        np.radians(FOV_DEGREES / 2),
        POINTS_PER_SCAN
    )

    points = []

    # We iterate 0 to 273
    for i in range(POINTS_PER_SCAN):
        r = bg_mean_m[i]
        noise = bg_std_m[i]
        theta = angles[i]

        if r > 0.05:  # Only save valid background points
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            z = 0.0
            # Store NOISE in Intensity field
            points.append((x, y, z, noise))

    # 3. Save PCD
    save_path = os.path.join(OUTPUT_FOLDER, FILENAME_PCD)
    with open(save_path, 'w') as f:
        f.write(f"# .PCD v.7 - Mean=XYZ, StdDev=Intensity\n")
        f.write("VERSION .7\n")
        f.write("FIELDS x y z intensity\n")  # Intensity holds the variance
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for p in points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {p[3]:.6f}\n")

    print(f"✅ PCD Saved: {save_path}")
    print("   Open this in Open3D/CloudCompare.")
    print("   - The dots show the Zero Plane.")
    print("   - The 'Intensity' color shows the Noise Level.")


if __name__ == "__main__":
    run_recorder()
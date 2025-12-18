import serial
import time
import numpy as np
import os
import sys
from scipy.signal import medfilt

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM5'
BAUD_RATE = 460800

POINTS_PER_SCAN = 274
FOV_DEGREES = 96.0

DURATION_SEC = 10.0

MIN_VALID_DISTANCE_MM = 50
MAX_VALID_DISTANCE_MM = 80000

EXPECTED_PACKET_SIZE = (POINTS_PER_SCAN * 2) + 10

OUTPUT_FOLDER = "zero_plane"
FILENAME_PCD = "master_zero_plane.pcd"
FILENAME_NPY = "master_zero_plane.npy"

MIN_FRAMES_REQUIRED = 30
SPATIAL_KERNEL = 5  # must be odd

# ==========================================


def save_static_pcd(distances, folder, filename):
    filepath = os.path.join(folder, filename)

    angles = np.linspace(
        np.radians(-FOV_DEGREES / 2),
        np.radians(FOV_DEGREES / 2),
        len(distances)
    )

    points = []
    for r, theta in zip(distances, angles):
        r_m = r / 1000.0
        if 0.05 < r_m < 80.0:
            x = r_m * np.cos(theta)
            y = r_m * np.sin(theta)
            points.append((0.0, abs(x), y))

    with open(filepath, 'w') as f:
        f.write("# .PCD v0.7 - MASTER ZERO PLANE\n")
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
            f.write(f"{p[0]:.5f} {p[1]:.5f} {p[2]:.5f}\n")

    print(f"✅ PCD saved: {filepath}")


def record_zero_plane():
    print("\n🎯 RECORDING MASTER ZERO PLANE")
    print("⚠️  ENSURE ABSOLUTE STILLNESS\n")

    os.makedirs(OUTPUT_FOLDER, exist_ok=True)

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.2)
    time.sleep(1.0)

    scan_buffer = []
    serial_buffer = b""

    total_packets = 0
    valid_packets = 0

    start_time = time.time()
    print("🔴 Recording...")

    while time.time() - start_time < DURATION_SEC:
        serial_buffer += ser.read(ser.in_waiting or 1)

        while True:
            start_idx = serial_buffer.find(b'\x02')
            if start_idx == -1:
                serial_buffer = b""
                break

            if start_idx > 0:
                serial_buffer = serial_buffer[start_idx:]

            if len(serial_buffer) < EXPECTED_PACKET_SIZE:
                break

            total_packets += 1

            payload = serial_buffer[4:4 + POINTS_PER_SCAN * 2]
            raw = np.frombuffer(payload, dtype=np.uint8)

            high = raw[0::2].astype(np.uint16)
            low = raw[1::2].astype(np.uint16)
            distances = (high << 8) | low

            valid_ratio = np.count_nonzero(
                (distances > MIN_VALID_DISTANCE_MM) &
                (distances < MAX_VALID_DISTANCE_MM)
            ) / POINTS_PER_SCAN

            if valid_ratio > 0.65:
                scan_buffer.append(distances)
                valid_packets += 1
                sys.stdout.write(f"\rFrames captured: {len(scan_buffer)}")
                sys.stdout.flush()

            serial_buffer = serial_buffer[EXPECTED_PACKET_SIZE:]

    ser.close()
    print("\n🛑 Capture complete.")

    print(f"📦 Total packets seen: {total_packets}")
    print(f"✅ Valid packets used: {valid_packets}")

    if len(scan_buffer) < MIN_FRAMES_REQUIRED:
        print("❌ Not enough clean data.")
        return

    # ==========================================
    # PROCESSING
    # ==========================================
    print("🧠 Computing robust zero plane...")

    data = np.array(scan_buffer)

    # Median profile
    median_profile = np.median(data, axis=0)

    # Median Absolute Deviation (robust noise metric)
    mad = np.median(np.abs(data - median_profile), axis=0)

    # Remove unstable angles
    stable_mask = mad < 40  # mm
    median_profile[~stable_mask] = np.nan
    median_profile = np.nanmedian(
        np.vstack([median_profile] * 3), axis=0
    )

    # Spatial smoothing
    median_profile = medfilt(median_profile, SPATIAL_KERNEL)

    # Save outputs
    npy_path = os.path.join(OUTPUT_FOLDER, FILENAME_NPY)
    np.save(npy_path, median_profile)
    print(f"💾 NPY saved: {npy_path}")

    save_static_pcd(median_profile, OUTPUT_FOLDER, FILENAME_PCD)

    print("\n🏁 ZERO PLANE CREATED SUCCESSFULLY")


if __name__ == "__main__":
    record_zero_plane()

import serial
import time
import numpy as np
import os
import sys
from scipy.signal import medfilt

# ==========================================
# USER SETTING
# ==========================================
CALIBRATION_MODE = "WALL"   # "WALL" or "FLOOR"

# ==========================================
# HARDWARE CONFIG
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
SPATIAL_KERNEL = 5  # odd

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
        if np.isnan(r):
            continue

        r_m = r / 1000.0
        if 0.05 < r_m < 80.0:
            x = r_m * np.cos(theta)
            y = r_m * np.sin(theta)
            points.append((0.0, abs(x), y))

    with open(filepath, 'w') as f:
        f.write("# .PCD v0.7 - ZERO PLANE\n")
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

    print(f"✅ PCD saved ({len(points)} points): {filepath}")


def record_zero_plane():
    print(f"\n🎯 ZERO PLANE CALIBRATION MODE: {CALIBRATION_MODE}")
    print("⚠️  ENSURE ABSOLUTE STILLNESS\n")

    os.makedirs(OUTPUT_FOLDER, exist_ok=True)

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.2)
    time.sleep(1.0)

    scan_buffer = []
    serial_buffer = b""

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
                sys.stdout.write(f"\rFrames: {len(scan_buffer)}")
                sys.stdout.flush()

            serial_buffer = serial_buffer[EXPECTED_PACKET_SIZE:]

    ser.close()
    print("\n🛑 Capture complete.")

    if len(scan_buffer) < MIN_FRAMES_REQUIRED:
        print("❌ Not enough clean data.")
        return

    # ==========================================
    # PROCESSING
    # ==========================================
    data = np.array(scan_buffer)
    median_profile = np.median(data, axis=0)

    mad = np.median(np.abs(data - median_profile), axis=0)

    if CALIBRATION_MODE == "WALL":
        mad_percentile = 70
        angle_mask = np.ones(POINTS_PER_SCAN, dtype=bool)

    elif CALIBRATION_MODE == "FLOOR":
        mad_percentile = 85

        angles = np.linspace(
            -FOV_DEGREES / 2,
            FOV_DEGREES / 2,
            POINTS_PER_SCAN
        )

        # reject shallow floor grazing rays
        angle_mask = np.abs(angles) < (FOV_DEGREES * 0.35)

    else:
        raise ValueError("CALIBRATION_MODE must be WALL or FLOOR")

    mad_threshold = np.percentile(mad[angle_mask], mad_percentile)

    stable_mask = (mad < mad_threshold) & angle_mask

    print(
        f"MAD → min:{mad.min():.1f} "
        f"mean:{mad.mean():.1f} "
        f"max:{mad.max():.1f}"
    )
    print(f"Using MAD threshold: {mad_threshold:.1f} mm")
    print(f"Stable angles: {np.sum(stable_mask)}/{POINTS_PER_SCAN}")

    median_profile[~stable_mask] = np.nan
    median_profile = np.nanmedian(
        np.vstack([median_profile] * 3), axis=0
    )

    if np.all(np.isnan(median_profile)):
        print("❌ All points rejected — fallback to raw median")
        median_profile = np.median(data, axis=0)

    median_profile = medfilt(median_profile, SPATIAL_KERNEL)

    np.save(os.path.join(OUTPUT_FOLDER, FILENAME_NPY), median_profile)
    save_static_pcd(median_profile, OUTPUT_FOLDER, FILENAME_PCD)

    print("\n🏁 ZERO PLANE CREATED SUCCESSFULLY")


if __name__ == "__main__":
    record_zero_plane()

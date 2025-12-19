import serial
import time
import numpy as np
import os
import sys
import statistics
import datetime

# ==========================================
# CONFIGURATION
# ==========================================
PORT = 'COM5'  # Adjust to your LZR Port
BAUD = 460800  # LZR Standard Baud Rate
POINTS_PER_SCAN = 274  # Fixed resolution for LZR
FOV_DEGREES = 96.0  # Field of View
OUTPUT_FOLDER = "zero_plane"
FILENAME = "final_zero_plane.pcd"

# ==========================================
# GLOBAL STORAGE
# ==========================================
# Unlike TG30 which has random angles, LZR has fixed indices [0..273].
# We create a list of lists to store history for each beam.
# polar_history[0] stores all distances ever measured by Beam 0.
polar_history = [[] for _ in range(POINTS_PER_SCAN)]


# ==========================================
# SAVE FUNCTION (MEDIAN FILTER)
# ==========================================
def save_statistical_zero_plane():
    print(f"\n\n🛑 Stopping... Analyzing history for {POINTS_PER_SCAN} beams...")
    print("   Computing Median (Statistical Background Model)... please wait.")

    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)

    clean_points = []

    # Generate Angles (Theta) for the fixed indices
    angles = np.linspace(
        np.radians(-FOV_DEGREES / 2),
        np.radians(FOV_DEGREES / 2),
        POINTS_PER_SCAN
    )

    # PROCESS EACH BEAM INDIVIDUALLY
    for i in range(POINTS_PER_SCAN):
        history = polar_history[i]

        if len(history) > 5:
            # 1. Statistical Median (The Magic Step)
            # Rejects cars, birds, and electrical noise.
            median_dist_mm = statistics.median(history)

            # Filter valid range (ignore dead beams)
            if 50 < median_dist_mm < 60000:
                r_meters = median_dist_mm / 1000.0
                theta = angles[i]

                # 2. Convert to Coordinates (Sideways Profile)
                # Matches your LZR setup: Y=Width, Z=Height, X=0 (Flat)

                # Note: 'abs' on width to ensure positive coordinates
                y_width = abs(r_meters * np.cos(theta))
                z_height = r_meters * np.sin(theta)

                clean_points.append((0.0, y_width, z_height))

    if not clean_points:
        print("❌ Error: No valid points to save.")
        return

    # SAVE PCD
    full_path = os.path.join(OUTPUT_FOLDER, FILENAME)

    try:
        with open(full_path, 'w') as f:
            f.write(f"# .PCD v.7 - LZR STATISTICAL ZERO PLANE\n")
            f.write("VERSION .7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(clean_points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(clean_points)}\n")
            f.write("DATA ascii\n")
            for p in clean_points:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

        print(f"✅ SUCCESS: Saved '{full_path}' with {len(clean_points)} static points.")

        # ALSO SAVE NUMPY (Useful for your background subtraction script)
        # We reconstruct the full array (filling missing spots with 0) for the .npy file
        npy_path = os.path.join(OUTPUT_FOLDER, "master_zero_plane.npy")

        full_profile = []
        for i in range(POINTS_PER_SCAN):
            if len(polar_history[i]) > 5:
                full_profile.append(statistics.median(polar_history[i]))
            else:
                full_profile.append(0.0)

        np.save(npy_path, np.array(full_profile))
        print(f"✅ SUCCESS: Saved '{npy_path}' for subtraction tool.")

    except Exception as e:
        print(f"❌ File Write Error: {e}")


# ==========================================
# MAIN LOOP
# ==========================================
def run_recorder():
    ser = None
    frame_count = 0

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"🚀 Building Statistical Zero Plane on {PORT}")
        print(f"📂 Output will be saved to: ./{OUTPUT_FOLDER}/")
        print("💡 Recording indefinitely. Press CTRL+C to stop and save.")
        print("-" * 50)

        buffer = b""

        while True:
            if ser.in_waiting > 0:
                buffer += ser.read(ser.in_waiting)

                # LZR Parsing Logic
                while True:
                    # Find Start Byte
                    start_idx = buffer.find(b'\x02')
                    if start_idx == -1:
                        buffer = b""
                        break

                    if start_idx > 0:
                        buffer = buffer[start_idx:]

                    EXPECTED_SIZE = (POINTS_PER_SCAN * 2) + 10

                    if len(buffer) < EXPECTED_SIZE:
                        break  # Wait for more data

                    # Extract Packet
                    payload = buffer[4: 4 + (POINTS_PER_SCAN * 2)]
                    high = np.frombuffer(payload[0::2], dtype=np.uint8)
                    low = np.frombuffer(payload[1::2], dtype=np.uint8)
                    distances = (high.astype(np.uint16) << 8) + low

                    # Store Data Logic
                    if len(distances) == POINTS_PER_SCAN:
                        for i, dist in enumerate(distances):
                            # Append distance to that specific beam's history
                            polar_history[i].append(dist)

                        frame_count += 1
                        sys.stdout.write(
                            f"\r[Recording] Frames: {frame_count} | Samples per Beam: {len(polar_history[137])}")
                        sys.stdout.flush()

                    # Move buffer forward
                    buffer = buffer[EXPECTED_SIZE:]

    except KeyboardInterrupt:
        # User pressed Ctrl+C
        if ser: ser.close()
        save_statistical_zero_plane()

    except Exception as e:
        print(f"\n❌ Error: {e}")


if __name__ == "__main__":
    run_recorder()
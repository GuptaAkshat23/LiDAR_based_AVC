import serial
import time
import numpy as np

# --- FINAL CONFIRMED SETTINGS ---
SERIAL_PORT = 'COM4'
BAUD_RATE = 921600  # <--- UPDATED
POINTS_PER_SCAN = 274
CALIBRATION_FRAMES = 100
FOV_DEGREES = 96.0


def save_pcd(distances, filename):
    print(f"\n[GENERATING] Creating {filename}...")
    angles = np.linspace(np.radians(-FOV_DEGREES / 2), np.radians(FOV_DEGREES / 2), POINTS_PER_SCAN)
    valid_points = []

    for r_mm, theta in zip(distances, angles):
        # Filter: 10cm to 10m (Room Mode)
        if 100 < r_mm < 10000:
            r_m = r_mm / 1000.0
            # Sideways Mapping
            x = 0
            y = r_m * np.cos(theta)
            z = r_m * np.sin(theta)
            valid_points.append((x, y, z))

    num = len(valid_points)
    with open(filename, 'w') as f:
        f.write("# .PCD v.7 - Background\n")
        f.write("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {num}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {num}\nDATA ascii\n")
        for p in valid_points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
    print(f"[SUCCESS] Saved '{filename}'.")


def record_background():
    print("--- STEP 1: CALIBRATION (921600 / BIG ENDIAN) ---")
    print("Keep lane EMPTY.")

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    buffer = b""
    frames = []

    try:
        while len(frames) < CALIBRATION_FRAMES:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)
                while True:
                    idx = buffer.find(b'\x02')
                    if idx == -1:
                        buffer = buffer[-5:]
                        break
                    if len(buffer) - idx < 558: break

                    raw_data = buffer[idx + 4: idx + 4 + 548]
                    dist_list = []

                    for i in range(0, len(raw_data) - 1, 2):
                        # BIG ENDIAN CONFIRMED
                        high = raw_data[i]
                        low = raw_data[i + 1]
                        val = (high << 8) + low
                        dist_list.append(val)

                    if len(dist_list) == POINTS_PER_SCAN:
                        frames.append(dist_list)
                        print(f"Captured {len(frames)} frames", end='\r')
                    buffer = buffer[idx + 558:]
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    if frames:
        bg = np.median(np.array(frames), axis=0)
        save_pcd(bg, "background.pcd")
        np.save("internal_zero_plane.npy", bg)


if __name__ == "__main__":
    record_background()
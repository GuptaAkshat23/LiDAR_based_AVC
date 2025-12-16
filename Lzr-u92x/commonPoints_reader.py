import serial
import time
import numpy as np
import datetime
import os
import sys

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM4'  # Change to your port
BAUD_RATE = 460800  # Standard LZR U921 baud rate
POINTS_PER_SCAN = 274  # Standard LZR resolution
FOV_DEGREES = 96.0  # Field of View
SAVE_DIRECTORY = "./scans"

# --- FILTERING SETTINGS ---
# The size of the grid cell to group points (in meters)
# 0.05 = 5cm.
GRID_RESOLUTION = 0.05

# 0.20 = 20%.
# A point must appear in 20% of the frames to be saved.
# (e.g., if you capture 100 frames, the point must appear in at least 20 of them)
MIN_PERSISTENCE_RATIO = 0.20

# Global storage for grid counting
# Format: { (grid_x, grid_y): hit_count }
spatial_grid = {}
total_frames_captured = 0


# ==========================================
# PCD SAVING FUNCTION (FILTERED)
# ==========================================
def save_filtered_pcd(grid_data, total_scans, output_folder):
    """
    Filters the grid based on persistence (20% threshold) and saves valid points.
    """
    if not grid_data:
        print("[WARNING] No data collected.")
        return

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Calculate the minimum hits required (20% of total scans)
    min_hits_required = int(total_scans * MIN_PERSISTENCE_RATIO)

    # Safety check: ensure we require at least 1 hit
    if min_hits_required < 1:
        min_hits_required = 1

    print(f"\n[FILTERING] Total Scans: {total_scans}")
    print(f"[FILTERING] Threshold: {MIN_PERSISTENCE_RATIO * 100}%")
    print(f"[FILTERING] A point must have been seen {min_hits_required} times to be saved.")

    final_points = []

    # Iterate through grid and keep only common points
    for (ix, iy), count in grid_data.items():
        if count >= min_hits_required:
            # Convert grid index back to real world meters (center of cell)
            x = ix * GRID_RESOLUTION
            y = iy * GRID_RESOLUTION
            z = 0.0  # Flattened plane
            final_points.append((x, y, z))

    if not final_points:
        print("[WARNING] No points passed the filter.")
        return

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_folder, f"lzr_filtered_20percent_{timestamp}.pcd")
    num_points = len(final_points)

    print(f"[SAVING] Writing {num_points} filtered points to {filename}...")

    with open(filename, 'w') as f:
        f.write("# .PCD v.7 - LZR U921 20% Persistence Filter\n")
        f.write("VERSION .7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")
        for p in final_points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print(f"[COMPLETE] File saved successfully.")


# ==========================================
# PROCESSING FUNCTION
# ==========================================
def process_scan(distances_mm):
    global total_frames_captured

    # Generate angles
    angles = np.linspace(
        np.radians(-FOV_DEGREES / 2),
        np.radians(FOV_DEGREES / 2),
        len(distances_mm)
    )

    # Set to track cells seen in THIS specific frame
    seen_cells_this_frame = set()

    for r, theta in zip(distances_mm, angles):
        r_meters = r / 1000.0

        # Basic range filter
        if 0.1 < r_meters < 60.0:
            # Convert to X, Y
            x = r_meters * np.cos(theta)
            y = r_meters * np.sin(theta)

            # Convert to Grid Coordinates (integer buckets)
            ix = int(round(x / GRID_RESOLUTION))
            iy = int(round(y / GRID_RESOLUTION))

            cell_key = (ix, iy)

            # Only count a cell once per frame!
            if cell_key not in seen_cells_this_frame:
                seen_cells_this_frame.add(cell_key)

                # Increment the persistent counter for this cell
                if cell_key in spatial_grid:
                    spatial_grid[cell_key] += 1
                else:
                    spatial_grid[cell_key] = 1

    total_frames_captured += 1

    if total_frames_captured % 10 == 0:
        print(f"Scanning... Frame #{total_frames_captured} | Tracking {len(spatial_grid)} unique locations")


# ==========================================
# MAIN READER LOOP
# ==========================================
def run_recorder():
    ser = None
    print("--- LZR U921 FILTER (20%) ---")
    print(f"Grid Resolution: {GRID_RESOLUTION * 100} cm")
    print(f"Persistence Threshold: {MIN_PERSISTENCE_RATIO * 100}%")
    print("Filter active: Points must appear in at least 1/5 of scans.")
    print("Press Ctrl+C to stop and save.\n")

    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}. Collecting data...")

        buffer = b""

        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer += chunk

                start_index = buffer.find(b'\x02')

                if start_index != -1:
                    buffer = buffer[start_index:]
                    EXPECTED_SIZE = (POINTS_PER_SCAN * 2) + 10

                    if len(buffer) >= EXPECTED_SIZE:
                        raw_payload = buffer[4: 4 + (POINTS_PER_SCAN * 2)]

                        distances = []
                        for i in range(0, len(raw_payload) - 1, 2):
                            high = raw_payload[i]
                            low = raw_payload[i + 1]
                            dist = (high << 8) + low
                            distances.append(dist)

                        if len(distances) == POINTS_PER_SCAN:
                            process_scan(distances)

                        buffer = buffer[EXPECTED_SIZE:]

            time.sleep(0.005)

    except serial.SerialException:
        print(f"[ERROR] Could not open {SERIAL_PORT}.")
    except KeyboardInterrupt:
        print("\n[STOP] Recording stopped.")
    finally:
        if ser and ser.is_open:
            ser.close()
        save_filtered_pcd(spatial_grid, total_frames_captured, SAVE_DIRECTORY)


if __name__ == "__main__":
    run_recorder()
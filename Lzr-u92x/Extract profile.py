import numpy as np
import os

# --- CONFIGURATION ---
ZERO_PLANE_FILE = "zero_plane.npy"
RAW_STACK_FILE = "raw_vehicle_stack.npy"
OUTPUT_PCD = "room_test_profile.pcd"

# PHYSICAL SETTINGS
FOV_DEGREES = 96.0
X_INCREMENT_METERS = 0.05

# ROOM SPECIFIC SETTINGS
MAX_RANGE_METERS = 20.0  # Hard limit for your room test
TOLERANCE_MM = 100  # 10cm sensitivity (Better for room testing)


def extract_profile():
    print("--- STEP 3: PROCESSING (ROOM MODE) ---")

    # 1. Load Data
    try:
        zero_plane = np.load(ZERO_PLANE_FILE)
        raw_stack = np.load(RAW_STACK_FILE)
        print(f"Loaded Background (Zero Plane): {zero_plane.shape}")
        print(f"Loaded Recording (Raw Stack):   {raw_stack.shape}")
    except FileNotFoundError:
        print("Error: Files not found. Run Step 1 and Step 2 first!")
        return

    final_points = []

    # Pre-calculate angles
    angles = np.linspace(np.radians(-FOV_DEGREES / 2), np.radians(FOV_DEGREES / 2), 274)

    # 2. Iterate through every time-slice
    for time_idx, frame in enumerate(raw_stack):

        # --- A. RANGE FILTER ---
        # Convert to meters for checking range
        frame_meters = frame / 1000.0

        # Create a mask for points within 20m
        valid_range_mask = (frame_meters > 0.1) & (frame_meters < MAX_RANGE_METERS)

        # --- B. BACKGROUND FILTER (Zero Plane) ---
        # Calculate difference from the wall/background
        diff = np.abs(frame - zero_plane)

        # Logic:
        # 1. Point must be within 20m
        # 2. Point must be DIFFERENT from the wall (> 10cm change)
        # 3. Point must be CLOSER than the wall (frame < zero_plane)
        is_object = valid_range_mask & (diff > TOLERANCE_MM) & (frame < zero_plane)

        # Extract valid object data
        object_ranges = frame_meters[is_object]
        object_angles = angles[is_object]

        # --- C. COORDINATE MAPPING ---
        if len(object_ranges) > 0:
            # X = Time (Movement)
            x_coords = np.full(len(object_ranges), time_idx * X_INCREMENT_METERS)

            # Y = Width/Depth (Distance from sensor)
            y_coords = object_ranges * np.cos(object_angles)

            # Z = Height (Vertical position)
            z_coords = object_ranges * np.sin(object_angles)

            # Stack and add to list
            points = np.column_stack((x_coords, y_coords, z_coords))
            final_points.extend(points)

    save_pcd(final_points, OUTPUT_PCD)


def save_pcd(points, filename):
    num = len(points)
    print(f"\n[RESULT] Extracted {num} points within 20m range.")

    if num == 0:
        print("Warning: No object detected. (Did you move in front of the sensor?)")
        return

    with open(filename, 'w') as f:
        f.write("# .PCD v.7 - Room Test Profile\n")
        f.write("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {num}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {num}\nDATA ascii\n")
        for p in points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print(f"Saved to: {os.path.abspath(filename)}")


if __name__ == "__main__":
    extract_profile()
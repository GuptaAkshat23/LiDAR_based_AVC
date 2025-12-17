import open3d as o3d
import numpy as np
import os
import tkinter as tk
from tkinter import filedialog

# ==========================================
# CONFIGURATION
# ==========================================
# Path to the numpy file created by 'record_zero_plane.py'
ZERO_PLANE_PATH = "./zero_plane/master_zero_plane.npy"

# THRESHOLD
# How much closer must the object be to be kept?
# 0.15 = 15cm. (Removes wall wobble and noise)
DIFFERENCE_THRESHOLD = 0.15

# SENSOR SPECS (Must match your Reader Code)
FOV_DEGREES = 96.0
POINTS_PER_SCAN = 274


# ==========================================

def pick_files():
    root = tk.Tk()
    root.withdraw()
    file_paths = filedialog.askopenfilenames(
        title="Select Raw Truck PCD Files",
        filetypes=[("Point Cloud Data", "*.pcd")]
    )
    return root.tk.splitlist(file_paths)


def subtract_background():
    # 1. Load Zero Plane
    if not os.path.exists(ZERO_PLANE_PATH):
        print(f"❌ Error: Could not find '{ZERO_PLANE_PATH}'")
        print("   Please run 'record_zero_plane.py' first!")
        return

    print("🧠 Loading Master Zero Plane...")
    zero_plane_dists = np.load(ZERO_PLANE_PATH)  # Array of 274 float distances

    # 2. Select Files
    files = pick_files()
    if not files: return

    # 3. Create Output Folder
    first_dir = os.path.dirname(files[0])
    out_dir = os.path.join(first_dir, "background_removed")
    if not os.path.exists(out_dir): os.makedirs(out_dir)

    print(f"🔪 Processing {len(files)} files...")

    for file_path in files:
        filename = os.path.basename(file_path)

        # Load Point Cloud
        pcd = o3d.io.read_point_cloud(file_path)
        if pcd.is_empty(): continue

        points = np.asarray(pcd.points)  # [X, Y, Z]
        original_count = len(points)

        # --- THE MATH: REVERSE ENGINEER POLAR COORDS ---
        # 1. Calculate R (Distance from sensor) based on Y and Z
        # Recall: Y was Width, Z was Height in your Sideways setup
        # R = sqrt(y^2 + z^2)
        r_calculated = np.sqrt(points[:, 1] ** 2 + points[:, 2] ** 2)

        # 2. Calculate Angle (Theta)
        # Theta = arctan2(Z, Y) (Note: Check orientation of your mount)
        # We need to map this angle to the Index [0...273] of the zero plane
        theta_calculated = np.arctan2(points[:, 2], points[:, 1])

        # 3. Map Angle to Zero Plane Index
        # Angle range is -48 to +48 degrees (in radians)
        fov_rad = np.radians(FOV_DEGREES)
        start_angle = -fov_rad / 2

        # Index = (Angle - StartAngle) / Step
        angle_step = fov_rad / (POINTS_PER_SCAN - 1)
        indices = ((theta_calculated - start_angle) / angle_step).astype(int)

        # Clip indices to be safe (0 to 273)
        indices = np.clip(indices, 0, POINTS_PER_SCAN - 1)

        # --- THE SUBTRACTION ---
        # Get the background distance for every single point's angle
        background_r_for_points = zero_plane_dists[indices] / 1000.0  # Convert mm to m

        # Compare: Is the Point closer than the Background?
        # Logic: (Background - Point) > Threshold
        diff = background_r_for_points - r_calculated

        # Create Keep Mask
        mask = diff > DIFFERENCE_THRESHOLD

        # Apply Mask
        clean_pcd = pcd.select_by_index(np.where(mask)[0])

        # --- STATISTICAL CLEANING (Optional Polish) ---
        # Even after subtraction, edge noise might remain. A light SOR clean helps.
        if len(clean_pcd.points) > 10:
            clean_pcd, _ = clean_pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)

        # Save
        if len(clean_pcd.points) > 0:
            out_name = f"no_bg_{filename}"
            o3d.io.write_point_cloud(os.path.join(out_dir, out_name), clean_pcd)
            print(f"✅ {filename}: {original_count} -> {len(clean_pcd.points)} pts (Saved)")
        else:
            print(f"⚠️ {filename}: Empty after subtraction (Only background found).")

    print("\n🎉 Background Subtraction Complete.")
    print(f"📂 Check folder: {out_dir}")


if __name__ == "__main__":
    subtract_background()
import open3d as o3d
import numpy as np
import os
import tkinter as tk
from tkinter import filedialog

# ==========================================
# CONFIGURATION
# ==========================================
ZERO_PCD_PATH = "./zero_plane/master_zero_plane.pcd"

# NOISE SENSITIVITY
# 4.0 = Strict (Needs to be 4x the noise level away to count as object)
# 3.0 = Loose (More detail, potential noise)
SIGMA_THRESHOLD = 4.0
MIN_BUFFER_M = 0.05  # 5cm minimal clearance

FOV_DEGREES = 96.0
POINTS_PER_SCAN = 274


def load_statistical_pcd(path):
    """Parses the special PCD where Intensity = StdDev"""
    print("🧠 Loading Statistical PCD...")
    pcd = o3d.io.read_point_cloud(path)

    # Open3D loads 'intensity' into colors if not specified,
    # but for pure data handling, we might need to parse manually or access tensor.
    # However, since we wrote simple ASCII, we can assume perfect order [0..N].

    points = np.asarray(pcd.points)

    # RECOVERY:
    # We need the 'Intensity' column. Open3D often hides it.
    # A robust way for our custom ASCII file is to just read lines if Open3D fails,
    # but let's assume we map by Index.

    # Calculate Mean R from XYZ
    bg_mean = np.linalg.norm(points, axis=1)  # Sqrt(x^2 + y^2)

    # For StdDev, we need to read the 4th column from the file directly
    # because Open3D sometimes ignores non-color intensities in legacy mode.
    bg_std = []
    with open(path, 'r') as f:
        for line in f:
            if line.startswith("DATA"): break
        for line in f:
            parts = line.split()
            if len(parts) >= 4:
                bg_std.append(float(parts[3]))  # The intensity column

    bg_std = np.array(bg_std)

    # Ensure sizes match (handling potential header/data mismatches)
    if len(bg_std) != len(bg_mean):
        print(f"⚠️ Warning: Size mismatch. Mean: {len(bg_mean)}, Std: {len(bg_std)}")
        # Fallback pad/trim
        min_len = min(len(bg_mean), len(bg_std))
        bg_mean = bg_mean[:min_len]
        bg_std = bg_std[:min_len]

    return bg_mean, bg_std


def subtract_background():
    if not os.path.exists(ZERO_PCD_PATH):
        print("❌ Zero plane PCD not found.")
        return

    # 1. Load the Model
    bg_mean_arr, bg_std_arr = load_statistical_pcd(ZERO_PCD_PATH)

    # 2. Select Files
    root = tk.Tk()
    root.withdraw()
    file_paths = filedialog.askopenfilenames(title="Select Raw PCDs", filetypes=[("PCD", "*.pcd")])
    if not file_paths: return

    out_dir = os.path.join(os.path.dirname(file_paths[0]), "dynamic_filtered")
    if not os.path.exists(out_dir): os.makedirs(out_dir)

    # Pre-calc constants
    fov_rad = np.radians(FOV_DEGREES)
    start_angle = -fov_rad / 2
    angle_step = fov_rad / (POINTS_PER_SCAN - 1)

    for fpath in file_paths:
        pcd = o3d.io.read_point_cloud(fpath)
        if pcd.is_empty(): continue
        points = np.asarray(pcd.points)

        # Calculate Polar Coords for the Current Frame
        r_curr = np.sqrt(points[:, 1] ** 2 + points[:, 2] ** 2)
        theta_curr = np.arctan2(points[:, 2], points[:, 1])

        # Map to Index [0..273] to look up the background stats
        indices = ((theta_curr - start_angle) / angle_step).astype(int)
        indices = np.clip(indices, 0, len(bg_mean_arr) - 1)

        # Retrieve expected Mean and Noise for these specific angles
        exp_mean = bg_mean_arr[indices]
        exp_noise = bg_std_arr[indices]

        # --- DYNAMIC FILTER LOGIC ---
        # Threshold adapts: Is this beam shaky? If so, be lenient.
        thresholds = (exp_noise * SIGMA_THRESHOLD) + MIN_BUFFER_M

        diff = exp_mean - r_curr
        mask = diff > thresholds

        clean_pcd = pcd.select_by_index(np.where(mask)[0])

        fname = os.path.basename(fpath)
        if len(clean_pcd.points) > 0:
            o3d.io.write_point_cloud(os.path.join(out_dir, "dyn_" + fname), clean_pcd)
            print(f"✅ {fname}: Saved (Dynamic Filter)")
        else:
            print(f"⚠️ {fname}: Empty")


if __name__ == "__main__":
    subtract_background()
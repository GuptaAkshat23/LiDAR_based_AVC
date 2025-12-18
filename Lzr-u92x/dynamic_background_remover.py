import open3d as o3d
import numpy as np
import os
import tkinter as tk
from tkinter import filedialog

# ==========================================
# CONFIGURATION
# ==========================================
MODEL_PATH = "./zero_plane/statistical_background.npz"

# SENSITIVITY SETTINGS
# Sigma Multiplier: How many std devs must the object be away from background?
# 3.0 covers 99.7% of noise. Increase to 4.0 or 5.0 if still noisy.
SIGMA_THRESHOLD = 4.0

# Minimum fixed buffer (meters) to handle sensor resolution limits
MIN_BUFFER_M = 0.05  # 5cm min distance required

FOV_DEGREES = 96.0
POINTS_PER_SCAN = 274


def subtract_dynamic_background():
    # 1. Load Statistical Model
    if not os.path.exists(MODEL_PATH):
        print("❌ Model not found. Run 'statistical_zero_recorder.py' first.")
        return

    print("🧠 Loading Statistical Background Model...")
    model = np.load(MODEL_PATH)
    bg_mean = model['mean'] / 1000.0  # Convert mm to Meters
    bg_std = model['std'] / 1000.0  # Convert mm to Meters

    # 2. Select Files
    root = tk.Tk()
    root.withdraw()
    file_paths = filedialog.askopenfilenames(title="Select Raw PCDs", filetypes=[("PCD", "*.pcd")])
    if not file_paths: return

    out_dir = os.path.join(os.path.dirname(file_paths[0]), "dynamic_filtered")
    if not os.path.exists(out_dir): os.makedirs(out_dir)

    print(f"Processing {len(file_paths)} files...")

    # Pre-calculate angular constants
    fov_rad = np.radians(FOV_DEGREES)
    start_angle = -fov_rad / 2
    angle_step = fov_rad / (POINTS_PER_SCAN - 1)

    for fpath in file_paths:
        pcd = o3d.io.read_point_cloud(fpath)
        if pcd.is_empty(): continue

        points = np.asarray(pcd.points)  # [X(Time), Y(Width), Z(Height)]

        # --- REVERSE MAPPING ---
        # Assuming your saved PCDs are: X=Travel, Y=Width, Z=Height (Sensor Frame Y, Z)
        # We need R (Distance) and Theta (Angle)

        # Calculate R (Euclidean distance on the Y-Z plane)
        r_current = np.sqrt(points[:, 1] ** 2 + points[:, 2] ** 2)

        # Calculate Theta
        theta_current = np.arctan2(points[:, 2], points[:, 1])

        # Map to Array Indices [0...273]
        indices = ((theta_current - start_angle) / angle_step).astype(int)
        indices = np.clip(indices, 0, POINTS_PER_SCAN - 1)

        # --- THE DYNAMIC FILTER ---
        # 1. Get the expected background dist and noise for these points
        exp_bg = bg_mean[indices]
        exp_noise = bg_std[indices]

        # 2. Calculate the dynamic threshold for each individual point
        # Threshold = (Noise_Factor * Sigma) + Min_Buffer
        dynamic_thresh = (exp_noise * SIGMA_THRESHOLD) + MIN_BUFFER_M

        # 3. Compare
        # Object is valid if: (Background - Current) > Threshold
        diff = exp_bg - r_current
        mask = diff > dynamic_thresh

        # --- APPLY & SAVE ---
        clean_pcd = pcd.select_by_index(np.where(mask)[0])

        # Optional: Final cleanup for flying pixels
        if len(clean_pcd.points) > 10:
            clean_pcd, _ = clean_pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=1.5)

        fname = os.path.basename(fpath)
        if len(clean_pcd.points) > 0:
            o3d.io.write_point_cloud(os.path.join(out_dir, "dyn_" + fname), clean_pcd)
            print(f"✅ {fname}: {len(clean_pcd.points)} pts kept")
        else:
            print(f"⚠️ {fname}: Fully filtered (Empty)")

    print("\nDone. Check the 'dynamic_filtered' folder.")


if __name__ == "__main__":
    subtract_dynamic_background()
import open3d as o3d
import numpy as np
import os
import copy
import tkinter as tk
from tkinter import filedialog

# ==========================================
# CONFIGURATION
# ==========================================
# We will normalize the truck length (Z-Axis) to be exactly 1.0 "Unit"
# This makes speed variations irrelevant for the AI model.
TARGET_LENGTH = 1.0
OUTPUT_FOLDER = "normalized_scans"


# ==========================================

def pick_files():
    root = tk.Tk()
    root.withdraw()
    file_paths = filedialog.askopenfilenames(
        title="Select Distorted Truck Scans",
        filetypes=[("Point Cloud Data", "*.pcd")]
    )
    return root.tk.splitlist(file_paths)


def normalize_clouds():
    files = pick_files()
    if not files: return

    # Create output folder
    first_dir = os.path.dirname(files[0])
    out_dir = os.path.join(first_dir, OUTPUT_FOLDER)
    if not os.path.exists(out_dir): os.makedirs(out_dir)

    print(f"📏 Normalizing {len(files)} files to Standard Length {TARGET_LENGTH}...")

    for file_path in files:
        filename = os.path.basename(file_path)

        # 1. Load Data
        pcd = o3d.io.read_point_cloud(file_path)
        if pcd.is_empty(): continue

        points = np.asarray(pcd.points)

        # 2. Find Current Dimensions (Bounding Box)
        # Z is the Length axis in your setup
        z_vals = points[:, 2]
        z_min = np.min(z_vals)
        z_max = np.max(z_vals)
        current_length = z_max - z_min

        if current_length == 0: continue

        print(f"   Processing: {filename}")
        print(f"   -> Original Length (Distorted): {current_length:.2f} units")

        # 3. THE MATH: Normalize Z
        # Formula: Z_new = (Z_old - Z_min) / (Current_Length) * Target_Length

        # Create a deep copy so we don't mess up original
        norm_pcd = copy.deepcopy(pcd)
        norm_points = np.asarray(norm_pcd.points)

        # Apply scaling ONLY to Z axis (Index 2)
        norm_points[:, 2] = (norm_points[:, 2] - z_min) / current_length * TARGET_LENGTH

        # Update points
        norm_pcd.points = o3d.utility.Vector3dVector(norm_points)

        print(f"   -> Normalized to range [0.0 - {TARGET_LENGTH}]")

        # 4. Save
        out_name = f"norm_{filename}"
        o3d.io.write_point_cloud(os.path.join(out_dir, out_name), norm_pcd)

    print("\n✅ Normalization Complete.")
    print(f"📂 Saved in: {out_dir}")
    print("👉 Use these files for Training the AI Model.")


if __name__ == "__main__":
    normalize_clouds()
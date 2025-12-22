import open3d as o3d
import numpy as np
import os
import tkinter as tk
from tkinter import filedialog

# ================================
# CONFIGURATION
# ================================
DISTANCE_THRESHOLD = 0.03  # 3 cm (tune: 0.02–0.05 for TG30)
VOXEL_SIZE = 0.01          # 1 cm downsampling (optional but recommended)

# ================================
# FILE PICKER
# ================================
def pick_pcd_file(title):
    root = tk.Tk()
    root.withdraw()
    return filedialog.askopenfilename(
        title=title,
        filetypes=[("PCD files", "*.pcd")]
    )

# ================================
# LOAD & PREPROCESS
# ================================
def load_pcd(path):
    pcd = o3d.io.read_point_cloud(path)
    if len(pcd.points) == 0:
        raise RuntimeError(f"Empty point cloud: {path}")
    return pcd

# ================================
# MAIN SUBTRACTION LOGIC
# ================================
def subtract_background(stacked_pcd, zero_pcd):
    # Downsample (important for stability)
    stacked_ds = stacked_pcd.voxel_down_sample(VOXEL_SIZE)
    zero_ds = zero_pcd.voxel_down_sample(VOXEL_SIZE)

    zero_tree = o3d.geometry.KDTreeFlann(zero_ds)
    stacked_pts = np.asarray(stacked_ds.points)

    object_points = []

    for pt in stacked_pts:
        [_, _, dist2] = zero_tree.search_knn_vector_3d(pt, 1)
        if len(dist2) == 0:
            object_points.append(pt)
        else:
            if np.sqrt(dist2[0]) > DISTANCE_THRESHOLD:
                object_points.append(pt)

    object_pcd = o3d.geometry.PointCloud()
    object_pcd.points = o3d.utility.Vector3dVector(object_points)

    return object_pcd

# ================================
# SAVE RESULT
# ================================
def save_pcd(pcd, stacked_path):
    folder = os.path.dirname(stacked_path)
    base = os.path.splitext(os.path.basename(stacked_path))[0]

    output_name = f"{base}_OBJECT_ONLY.pcd"
    output_path = os.path.join(folder, output_name)

    o3d.io.write_point_cloud(output_path, pcd)
    print(f"✅ Saved subtracted PCD: {output_path}")

# ================================
# MAIN
# ================================
def main():
    print("📂 Select STACKED profile PCD")
    stacked_path = pick_pcd_file("Select STACKED profile (.pcd)")
    if not stacked_path:
        print("❌ No stacked file selected")
        return

    print("📂 Select ZERO profile PCD")
    zero_path = pick_pcd_file("Select ZERO profile (.pcd)")
    if not zero_path:
        print("❌ No zero file selected")
        return

    print("🔄 Loading point clouds...")
    stacked_pcd = load_pcd(stacked_path)
    zero_pcd = load_pcd(zero_path)

    print("🧮 Subtracting background...")
    object_pcd = subtract_background(stacked_pcd, zero_pcd)

    if len(object_pcd.points) == 0:
        print("⚠ No object points detected — try increasing DISTANCE_THRESHOLD")
        return

    save_pcd(object_pcd, stacked_path)

    print(f"🎯 Object points: {len(object_pcd.points)}")

    # Optional visualization
    o3d.visualization.draw_geometries([object_pcd], window_name="Object Only")

if __name__ == "__main__":
    main()
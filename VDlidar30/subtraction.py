import open3d as o3d
import numpy as np
from scipy.spatial import cKDTree
import tkinter as tk
from tkinter import filedialog
import os

# ================= CONFIGURATION =================
# Tuning Parameters
DIST_THRESHOLD = 0.03  # 3cm margin for background subtraction
MIN_CLUSTER_POINTS = 50  # Minimum points to be considered a real object
CLUSTER_DISTANCE = 0.05  # Max distance between points in a cluster


# =================================================

def select_file(title_text):
    """Opens a file explorer window to select a PCD file."""
    root = tk.Tk()
    root.withdraw()  # Hide the main empty window
    print(f"📂 Waiting for user to select: {title_text}...")

    file_path = filedialog.askopenfilename(
        title=title_text,
        filetypes=[("Point Cloud Data", "*.pcd"), ("All Files", "*.*")]
    )

    if not file_path:
        print("❌ Selection cancelled.")
        return None

    print(f"   -> Selected: {os.path.basename(file_path)}")
    return file_path


def main():
    # ========================================================
    # STEP 0: FILE SELECTION
    # ========================================================
    print("--- STEP 0: FILE SELECTION ---")

    # 1. Select Zero Plane
    zero_plane_path = select_file("Select the ZERO PLANE file (Background)")
    if not zero_plane_path: return

    # 2. Select Stacked Reader
    stacked_path = select_file("Select the STACKED READER file (Object Scan)")
    if not stacked_path: return

    # Generate Output Filename automatically based on the input name
    output_dir = os.path.dirname(stacked_path)
    base_name = os.path.splitext(os.path.basename(stacked_path))[0]
    output_file = os.path.join(output_dir, f"EXTRACTED_{base_name}.pcd")

    print(f"\n🔹 Loading files...")
    pcd_zero = o3d.io.read_point_cloud(zero_plane_path)
    pcd_stack = o3d.io.read_point_cloud(stacked_path)

    if pcd_zero.is_empty() or pcd_stack.is_empty():
        print("❌ Error: One of the files is empty or corrupted.")
        return

    # ========================================================
    # STEP 1: PRE-PROCESSING & DENOISING
    # ========================================================
    print("\n🔹 Step 1: Denoising (Statistical Outlier Removal)...")
    pcd_zero, _ = pcd_zero.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd_stack, _ = pcd_stack.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # ========================================================
    # STEP 2: REGISTRATION (ALIGNMENT)
    # Mapping ZeroPlane(X,Y) -> Stacked(Y,Z)
    # ========================================================
    print("🔹 Step 2: Alignment...")
    pts_zero = np.asarray(pcd_zero.points)
    pts_stack = np.asarray(pcd_stack.points)

    # Zero Plane: X, Y become the reference 2D profile
    reference_profile_2d = pts_zero[:, 0:2]

    # Stacked Reader: Y, Z are the cross-section
    stack_cross_section_2d = pts_stack[:, 1:3]

    # ========================================================
    # STEP 3: DISTANCE-BASED SEGMENTATION (SUBTRACTION)
    # ========================================================
    print("🔹 Step 3: Background Subtraction...")
    tree = cKDTree(reference_profile_2d)

    # Calculate distance from every scan point to the nearest wall point
    distances, _ = tree.query(stack_cross_section_2d, k=1)

    # Keep points that are far away from the wall (> threshold)
    object_indices = np.where(distances > DIST_THRESHOLD)[0]

    if len(object_indices) == 0:
        print("⚠️ No object detected. All points matched the background.")
        return

    extracted_points = pts_stack[object_indices]
    pcd_object = o3d.geometry.PointCloud()
    pcd_object.points = o3d.utility.Vector3dVector(extracted_points)

    print(f"   -> Retained {len(extracted_points)} points (Object).")
    print(f"   -> Discarded {len(pts_stack) - len(extracted_points)} points (Background).")

    # ========================================================
    # STEP 4: POST-PROCESSING (CLUSTERING)
    # ========================================================
    print("🔹 Step 4: Clustering (Cleaning artifacts)...")
    labels = np.array(
        pcd_object.cluster_dbscan(eps=CLUSTER_DISTANCE, min_points=MIN_CLUSTER_POINTS, print_progress=False))

    if len(labels) == 0:
        print("⚠️ Only noise found.")
        return

    # Find largest cluster (The Object)
    counts = np.bincount(labels[labels >= 0])
    largest_cluster_idx = np.argmax(counts)

    final_object_indices = np.where(labels == largest_cluster_idx)[0]
    final_object_points = np.asarray(pcd_object.points)[final_object_indices]

    final_pcd = o3d.geometry.PointCloud()
    final_pcd.points = o3d.utility.Vector3dVector(final_object_points)

    # ========================================================
    # SAVE AND VISUALIZE
    # ========================================================
    o3d.io.write_point_cloud(output_file, final_pcd)
    print(f"\n✅ SUCCESS: Saved to: {output_file}")

    print("   Opening visualization window...")
    final_pcd.paint_uniform_color([0, 1, 0])  # Paint Green
    o3d.visualization.draw_geometries([final_pcd],
                                      window_name="Extracted Object",
                                      width=800, height=600)


if __name__ == "__main__":
    main()
import open3d as o3d
import tkinter as tk
from tkinter import filedialog


def remove_outliers():
    # 1. Ask user for the PCD file
    root = tk.Tk()
    root.withdraw()  # Hide the main tkinter window
    file_path = filedialog.askopenfilename(
        title="Select your Point Cloud file",
        filetypes=[("PCD files", "*.pcd"), ("All files", "*.*")]
    )

    if not file_path:
        print("No file selected. Exiting.")
        return

    # 2. Load the point cloud
    print(f"Loading: {file_path}")
    pcd = o3d.io.read_point_cloud(file_path)

    # Optional: Downsample to speed up processing if the cloud is very dense
    # pcd = pcd.voxel_down_sample(voxel_size=0.02)

    # 3. Statistical Outlier Removal (SOR)
    # This removes points that are further away from their neighbors compared to the average.
    # nb_neighbors: How many neighbors to analyze for each point
    # std_ratio: Lower values remove more points (stricter)
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
    pcd_clean = pcd.select_by_index(ind)

    # 4. Radius Outlier Removal (ROR)
    # This removes points that have fewer than 'nb_points' in a given 'radius'
    # Great for removing isolated "floating" noise
    cl, ind = pcd_clean.remove_radius_outlier(nb_points=16, radius=0.05)
    pcd_final = pcd_clean.select_by_index(ind)

    print("Outlier removal complete.")

    # 5. Visualize the result
    # The original points will be in Red, the cleaned points in Cyan
    pcd.paint_uniform_color([1, 0, 0])  # Red (Original)
    pcd_final.paint_uniform_color([0, 1, 1])  # Cyan (Cleaned)

    print("Showing result: Red = Original, Cyan = Cleaned")
    o3d.visualization.draw_geometries([pcd_final], window_name="Cleaned Point Cloud")

    # 6. Save the cleaned file
    save_path = file_path.replace(".pcd", "_cleaned.pcd")
    o3d.io.write_point_cloud(save_path, pcd_final)
    print(f"Cleaned file saved as: {save_path}")


if __name__ == "__main__":
    remove_outliers()
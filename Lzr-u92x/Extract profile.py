import tkinter as tk
from tkinter import filedialog, messagebox
import numpy as np
import open3d as o3d
import os


def subtract_scans():
    # Setup GUI
    root = tk.Tk()
    root.withdraw()

    # 1. Select the OBJECT File (The scan with the car)
    print("Select SIGNAL file (Car/Object)...")
    sig_path = filedialog.askopenfilename(
        title="Select Signal File",
        filetypes=[("PCD", "*.pcd")]
    )
    if not sig_path: return

    # 2. Select the ZERO PLANE File (The extruded background)
    print("Select ZERO PLANE file...")
    zero_path = filedialog.askopenfilename(
        title="Select Zero Plane File",
        filetypes=[("PCD", "*.pcd")]
    )
    if not zero_path: return

    try:
        print("Loading clouds...")
        # Load clouds and keep 'nan' values so the grid stays aligned
        sig_pcd = o3d.io.read_point_cloud(sig_path, remove_nan_points=False)
        zero_pcd = o3d.io.read_point_cloud(zero_path, remove_nan_points=False)

        # Convert to Numpy Arrays (Rows x Columns x 3)
        sig_pts = np.asarray(sig_pcd.points)
        zero_pts = np.asarray(zero_pcd.points)

        # CHECK: Are they the same size?
        if sig_pts.shape != zero_pts.shape:
            messagebox.showerror("Error",
                                 f"Shape Mismatch!\n\n"
                                 f"Signal Points: {sig_pts.shape[0]}\n"
                                 f"Zero Points:   {zero_pts.shape[0]}\n\n"
                                 f"Both files must use the exact same STACK_SIZE (e.g., 100)."
                                 )
            return

        print("Computing difference...")

        # --- THE MAGIC LOGIC ---
        # We calculate the distance between the Car Scan and the Empty Floor.
        # axis=1 means we check X, Y, and Z differences for every point.
        differences = sig_pts - zero_pts

        # Calculate the "Magnitude" of the difference (Distance in meters)
        # This tells us how far the point moved from the zero state.
        dist_diff = np.linalg.norm(differences, axis=1)

        # THRESHOLD: How much change counts as an object?
        # 0.05 meters = 5 cm. Any change less than 5cm is considered "Noise/Floor".
        threshold = 0.05

        # Create the Final Output
        # We take the ORIGINAL Signal points...
        final_pts = sig_pts.copy()

        # ...and we set the "Floor Points" to NaN (Invisible)
        # Logic: If the difference is SMALL, it's floor -> Delete it.
        # Logic: If the difference is BIG, it's car -> Keep it.
        final_pts[dist_diff < threshold] = np.nan

        # Save Result
        print("Saving result...")
        out_pcd = o3d.geometry.PointCloud()
        out_pcd.points = o3d.utility.Vector3dVector(final_pts)

        save_path = filedialog.asksaveasfilename(
            title="Save Clean Output",
            initialfile="clean_object_scan.pcd",
            defaultextension=".pcd"
        )

        if save_path:
            o3d.io.write_point_cloud(save_path, out_pcd)
            messagebox.showinfo("Success", f"Clean file saved!\n\nFloor points removed.")

    except Exception as e:
        messagebox.showerror("Error", str(e))
        print(f"Error: {e}")


if __name__ == "__main__":
    subtract_scans()
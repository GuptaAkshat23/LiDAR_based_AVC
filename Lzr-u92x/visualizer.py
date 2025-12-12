import open3d as o3d
import numpy as np
import glob
import os
import tkinter as tk
from tkinter import filedialog


class LZRU921Viewer:
    def __init__(self):
        # 1. Pick a file to start
        selected_path = self.pick_file()
        if not selected_path:
            return

        # 2. Find all other PCD files in that directory
        directory = os.path.dirname(selected_path)
        # Assuming your files are named like 'lzr_scan_...'
        search_pattern = os.path.join(directory, "*.pcd")
        self.files = sorted(glob.glob(search_pattern))

        if not self.files:
            print("❌ No files found.")
            return

        # Normalize paths for cross-platform safety
        selected_path = os.path.normpath(selected_path)
        self.files = [os.path.normpath(f) for f in self.files]

        # Find index of selected file
        try:
            self.index = self.files.index(selected_path)
        except ValueError:
            self.index = 0

        print(f"📂 Visualizing: {os.path.basename(self.files[self.index])}")

        # 3. Setup Open3D Window
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="LZR-U921 Viewer", width=1024, height=768)

        # 4. Visual Styling (Matches your request)
        opt = self.vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])  # PURE BLACK
        opt.point_size = 3.0  # Slightly larger for LZR visibility
        opt.show_coordinate_frame = False

        # 5. Load the first point cloud
        self.pcd = o3d.io.read_point_cloud(self.files[self.index])
        self.style_pcd(self.pcd)
        self.vis.add_geometry(self.pcd)

        # Add a small red sphere at (0,0,0) to represent the sensor itself
        self.origin = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        self.origin.paint_uniform_color([1, 0, 0])  # Red
        self.vis.add_geometry(self.origin)

        # 6. Bind Keys
        self.vis.register_key_callback(262, self.load_next)  # Right Arrow
        self.vis.register_key_callback(263, self.load_prev)  # Left Arrow

        print("Controls: Left/Right Arrow to switch files. Mouse to Rotate/Zoom.")

        # 7. Set initial view to Top-Down (Helpful for 2D Lidar)
        ctr = self.vis.get_view_control()
        ctr.set_zoom(0.8)

        self.vis.run()
        self.vis.destroy_window()

    def pick_file(self):
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.askopenfilename(
            title="Select a LZR-U921 Scan",
            filetypes=[("Point Cloud Data", "*.pcd")]
        )
        return file_path

    def style_pcd(self, pcd):
        # LZR U921 specific styling (Cyan/Aqua for high contrast on black)
        pcd.paint_uniform_color([0, 1, 1])

    def update_geometry(self):
        new_file = self.files[self.index]
        print(f"-> {os.path.basename(new_file)}")

        new_pcd = o3d.io.read_point_cloud(new_file)
        self.style_pcd(new_pcd)

        # CRITICAL FIX:
        # We must remove the old geometry and add the new one.
        # Just updating .colors or .points is risky if point counts differ
        # (e.g. if one scan filtered out errors and has 270 points vs 274).
        self.vis.remove_geometry(self.pcd, reset_bounding_box=False)
        self.pcd = new_pcd
        self.vis.add_geometry(self.pcd, reset_bounding_box=False)

        self.vis.poll_events()
        self.vis.update_renderer()

    def load_next(self, vis):
        if self.index < len(self.files) - 1:
            self.index += 1
            self.update_geometry()
        else:
            print("End of sequence.")

    def load_prev(self, vis):
        if self.index > 0:
            self.index -= 1
            self.update_geometry()
        else:
            print("Start of sequence.")


if __name__ == "__main__":
    LZRU921Viewer()
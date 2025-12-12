import open3d as o3d
import numpy as np
import glob
import os
import tkinter as tk
from tkinter import filedialog



class MinimalLiDARViewer:
    def __init__(self):
        # 1. Select a file first to identify the correct folder
        selected_path = self.pick_file()
        if not selected_path:
            return

        # 2. Automatically find all other .pcd files in that SAME folder
        # (This fixes bugs if your script and data are in different places)
        directory = os.path.dirname(selected_path)
        search_pattern = os.path.join(directory, "*.pcd")
        self.files = sorted(glob.glob(search_pattern))

        if not self.files:
            print("❌ No files found.")
            return

        # 3. Find the index of the file you clicked
        # We normalize paths to make sure Windows/Linux slashes match
        selected_path = os.path.normpath(selected_path)
        self.files = [os.path.normpath(f) for f in self.files]

        try:
            self.index = self.files.index(selected_path)
        except ValueError:
            self.index = 0

        print(f"📂 Visualizing: {os.path.basename(self.files[self.index])}")

        # 4. Setup Window
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="LiDAR Viewer", width=1024, height=768)

        # 5. Minimalist Render Options
        opt = self.vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])  # PURE BLACK
        opt.point_size = 2.0  # Fine detail
        opt.show_coordinate_frame = False  # Force hide axis

        # 6. Load Data
        self.pcd = o3d.io.read_point_cloud(self.files[self.index])
        self.style_pcd(self.pcd)
        self.vis.add_geometry(self.pcd)

        # NOTE: Coordinate Axis code has been deleted.

        # 7. Controls
        self.vis.register_key_callback(262, self.load_next)  # Right Arrow
        self.vis.register_key_callback(263, self.load_prev)  # Left Arrow

        print("controls: Left/Right Arrow to switch files.")
        self.vis.run()
        self.vis.destroy_window()

    def pick_file(self):
        """Opens dialog to pick the first file"""
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.askopenfilename(
            title="Select a LiDAR Scan",
            filetypes=[("Point Cloud Data", "*.pcd")]
        )
        return file_path

    def style_pcd(self, pcd):
        """Paint points Cyan for high contrast against black"""
        pcd.paint_uniform_color([0, 1, 1])

    def update_geometry(self):
        """Swaps data without closing window"""
        new_file = self.files[self.index]
        print(f"-> {os.path.basename(new_file)}")

        new_pcd = o3d.io.read_point_cloud(new_file)
        self.style_pcd(new_pcd)

        # Update points in-place (Fastest method)
        self.pcd.points = new_pcd.points
        self.pcd.colors = new_pcd.colors

        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

    def load_next(self, vis):
        if self.index < len(self.files) - 1:
            self.index += 1
            self.update_geometry()

    def load_prev(self, vis):
        if self.index > 0:
            self.index -= 1
            self.update_geometry()


if __name__ == "__main__":
    MinimalLiDARViewer()
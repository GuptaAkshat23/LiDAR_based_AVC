import open3d as o3d
import numpy as np
import glob
import os
import tkinter as tk
from tkinter import filedialog


class MinimalLiDARViewer:
    def __init__(self):
        # 1. Open File Dialog to pick the first file
        selected_path = self.pick_file()
        if not selected_path:
            return

        # 2. Smart Folder Detection
        # Automatically find all other .pcd files in the same folder
        directory = os.path.dirname(selected_path)
        search_pattern = os.path.join(directory, "*.pcd")
        self.files = sorted(glob.glob(search_pattern))

        if not self.files:
            print("❌ No files found.")
            return

        # 3. Find the index of the selected file
        selected_path = os.path.normpath(selected_path)
        self.files = [os.path.normpath(f) for f in self.files]

        try:
            self.index = self.files.index(selected_path)
        except ValueError:
            self.index = 0

        print(f"📂 Visualizing: {os.path.basename(self.files[self.index])}")

        # 4. Setup Visualization Window
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="Universal LiDAR Viewer", width=1024, height=768)

        # 5. Render Options (Black Background for High Contrast)
        opt = self.vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])  # PURE BLACK
        opt.point_size = 2.0  # Fine detail size
        opt.show_coordinate_frame = False  # Hide the colorful axis

        # 6. Load the Initial Point Cloud
        self.pcd = o3d.io.read_point_cloud(self.files[self.index])
        self.style_pcd(self.pcd)
        self.vis.add_geometry(self.pcd)

        # 7. Register Controls (Arrow Keys)
        self.vis.register_key_callback(262, self.load_next)  # Right Arrow
        self.vis.register_key_callback(263, self.load_prev)  # Left Arrow

        print("\n🎮 CONTROLS:")
        print("   [Right Arrow] : Next Scan")
        print("   [Left Arrow]  : Previous Scan")
        print("   [Mouse Drag]  : Rotate View")

        self.vis.run()
        self.vis.destroy_window()

    def pick_file(self):
        """Opens a standard Windows file picker"""
        root = tk.Tk()
        root.withdraw()  # Hide the main window
        file_path = filedialog.askopenfilename(
            title="Select a LiDAR Scan (.pcd)",
            filetypes=[("Point Cloud Data", "*.pcd")]
        )
        return file_path

    def style_pcd(self, pcd):
        """Paint points Cyan (Blue-Green) to pop against black background"""
        pcd.paint_uniform_color([0, 1, 1])

    def update_geometry(self):
        """Swaps the data shown in the window"""
        new_file = self.files[self.index]
        print(f"➡️ Switching to: {os.path.basename(new_file)}")

        new_pcd = o3d.io.read_point_cloud(new_file)
        self.style_pcd(new_pcd)

        # --- IMPORTANT FIX ---
        # Update both the points AND the colors
        self.pcd.points = new_pcd.points
        self.pcd.colors = new_pcd.colors

        # Tell the visualizer the data changed
        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

    def load_next(self, vis):
        if self.index < len(self.files) - 1:
            self.index += 1
            self.update_geometry()
        else:
            print("End of list.")

    def load_prev(self, vis):
        if self.index > 0:
            self.index -= 1
            self.update_geometry()
        else:
            print("Start of list.")


if __name__ == "__main__":
    MinimalLiDARViewer()
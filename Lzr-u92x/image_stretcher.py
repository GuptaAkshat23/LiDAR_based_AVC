import cv2
import numpy as np
import tkinter as tk
from tkinter import filedialog
import matplotlib.pyplot as plt
def expand_right_half():

    root = tk.Tk()
    root.withdraw()  # Hide the main window
    file_path = filedialog.askopenfilename(
        title="Select Image to Expand",
        filetypes=[("Image files", "*.png;*.jpg;*.jpeg;*.bmp")]
    )
    if not file_path:
        print("No image selected.")
        return

    # 2. Load Image
    img = cv2.imread(file_path)
    if img is None:
        print("Error: Could not load image.")
        return

    height, width = img.shape[:2]
    midpoint = width // 2

    # 3. Split Image
    left_part = img[:, :midpoint]
    right_part = img[:, midpoint:]

    # 4. Expand Right Half
    # "Expand by factor of .25" means new size is 1.0 + 0.25 = 1.25x
    expansion_factor = 0.30
    scale_multiplier = 1.0 + expansion_factor

    old_r_width = right_part.shape[1]
    new_r_width = int(old_r_width * scale_multiplier)

    # Resize the right part (Stretch width, keep height same)
    right_expanded = cv2.resize(right_part, (new_r_width, height), interpolation=cv2.INTER_LINEAR)

    # 5. Stitch Together
    final_img = np.hstack((left_part, right_expanded))

    # 6. Show Result
    # Convert BGR (OpenCV) to RGB (Matplotlib) for display
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    final_rgb = cv2.cvtColor(final_img, cv2.COLOR_BGR2RGB)

    plt.figure(figsize=(12, 6))

    plt.subplot(1, 2, 1)
    plt.title("Original Image")
    plt.imshow(img_rgb)
    plt.axis("off")

    plt.subplot(1, 2, 2)
    plt.title(f"Expanded Right Side (+{int(expansion_factor * 100)}%)")
    plt.imshow(final_rgb)
    plt.axis("off")

    plt.tight_layout()
    plt.show()

    # Optional: Save the result
    save_path = "expanded_image.png"
    cv2.imwrite(save_path, final_img)
    print(f"Saved expanded image to: {save_path}")


if __name__ == "__main__":
    expand_right_half()
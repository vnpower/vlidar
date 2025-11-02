import tkinter as tk
from PIL import Image, ImageDraw, ImageTk
import time

# Tọa độ trung tâm vòng tròn
w, h = 500, 300
radius = 20
thickness = 4

# Tạo cửa sổ overlay
root = tk.Tk()
root.overrideredirect(True)  # Không viền
root.attributes("-topmost", True)  # Luôn trên cùng
root.attributes("-transparentcolor", "white")  # Màu nền trong suốt

# Đặt vị trí cửa sổ
root.geometry(f"{radius*2+thickness*2}x{radius*2+thickness*2}+{w-radius-thickness}+{h-radius-thickness}")

# Tạo ảnh vòng tròn
size = radius*2 + thickness*2
image = Image.new("RGBA", (size, size), (255, 255, 255, 0))
draw = ImageDraw.Draw(image)
draw.ellipse(
    [thickness, thickness, size-thickness, size-thickness],
    outline="red", width=thickness
)
photo = ImageTk.PhotoImage(image)

# Hiển thị ảnh trong canvas
canvas = tk.Canvas(root, width=size, height=size, bg="white", highlightthickness=0)
canvas.pack()
canvas.create_image(0, 0, anchor=tk.NW, image=photo)

# Hiển thị trong 1 giây
root.after(1000, root.destroy)
root.mainloop()
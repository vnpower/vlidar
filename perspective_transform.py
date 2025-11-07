# Phép biến đổi phối cảnh (Perspective Transform)
import cv2
import numpy as np  
import math
import pyautogui  # Thêm ở đầu file nếu chưa có

# --- Hàm tính tọa độ điểm ảnh trên máy chiếu từ vị trí trên sàn ---
def floor_to_projector_pixel(
    floor_points: list,  # [(x1, y1), (x2, y2), (x3, y3), (x4, y4)] trên sàn, đơn vị cm
    projector_resolution: tuple,  # (width, height) của máy chiếu, ví dụ (800, 600)
    floor_pos: tuple  # (X, Y) vị trí trên sàn cần tìm
) -> tuple:
    """
    Trả về (x, y) pixel trên máy chiếu tương ứng với vị trí (X, Y) trên sàn.
    floor_points: 4 điểm tứ giác trên sàn (theo chiều kim đồng hồ, đơn vị cm)
    projector_resolution: (width, height) của máy chiếu
    floor_pos: (X, Y) trên sàn
    """
    # 4 điểm góc máy chiếu (theo chiều kim đồng hồ)
    proj_pts = np.array([
        [0, 0],
        [0, projector_resolution[1] - 1],
        [projector_resolution[0] - 1, projector_resolution[1] - 1],
        [projector_resolution[0] - 1, 0]
    ], dtype=np.float32)
    floor_pts = np.array(floor_points, dtype=np.float32)
    # Ma trận biến đổi phối cảnh: từ sàn về máy chiếu
    M = cv2.getPerspectiveTransform(floor_pts, proj_pts)
    # Chuyển vị trí (X, Y) về dạng (1, 1, 2)
    src = np.array([[floor_pos]], dtype=np.float32)
    dst = cv2.perspectiveTransform(src, M)
    x, y = dst[0][0]
    return (x, y)

# --- Kiểm tra nếu floor_pos nằm ngoài tứ giác floor_points ---
def is_point_in_floor_polygon(floor_points: list, floor_pos: tuple) -> bool:
    """
    Kiểm tra nếu vị trí floor_pos nằm trong tứ giác floor_points.
    floor_points: 4 điểm tứ giác trên sàn (theo chiều kim đồng hồ, đơn vị cm)
    floor_pos: (X, Y) trên sàn
    """
    contour = np.array(floor_points, dtype=np.float32)
    test = cv2.pointPolygonTest(contour, floor_pos, False)
    return test >= 0

# --- Hàm chuyển đổi từ (r, theta) sang (x, y) cho 4 đỉnh tứ giác ---
def polar_to_cartesian_list(polar_list, angle_unit='degree'):
    """
    Chuyển danh sách [(r1, theta1), ...] sang [(x1, y1), ...].
    angle_unit: 'degree' hoặc 'radian'.
    Góc theta tính từ trục tung (Oy), chiều dương ngược kim đồng hồ.
    """
    cartesian = []
    for r, theta in polar_list:
        if angle_unit == 'degree':
            theta_rad = math.radians(theta)
        else:
            theta_rad = theta
        x = r * math.sin(theta_rad)
        y = r * math.cos(theta_rad)
        cartesian.append((x, y))
    return cartesian

# --- Hàm tổng hợp: Tìm toạ độ điểm ảnh từ điểm trên sàn M(R, Theta) với 4 đỉnh tứ giác cho dưới dạng (r, theta) ---
def projector_pixel_from_polar(
    quad_polar,  # [(r1, theta1), (r2, theta2), (r3, theta3), (r4, theta4)]
    projector_resolution,  # (width, height)
    M_polar,  # (R, Theta)
    angle_unit='degree'
):
    # Chuyển 4 đỉnh sang (x, y)
    quad_cartesian = polar_to_cartesian_list(quad_polar, angle_unit)
    # Chuyển điểm M sang (x, y)
    M_xy = polar_to_cartesian_list([M_polar], angle_unit)[0]
    # Kiểm tra nếu điểm M nằm trong tứ giác
    if not is_point_in_floor_polygon(quad_cartesian, M_xy):
        print(f"Cảnh báo: Điểm M{M_polar} nằm ngoài tứ giác trên sàn!")
        return None
    # Tìm toạ độ điểm ảnh
    px_py = floor_to_projector_pixel(quad_cartesian, projector_resolution, M_xy)
    return px_py

def click_on_extended_monitor(px, py, main_screen_width=1920, main_screen_height=1080, extended_offset_x=None, extended_offset_y=0):
    """
    Click chuột trái tại (px, py) trên màn hình mở rộng.
    Nếu extended_offset_x=None, mặc định màn hình mở rộng nằm bên phải màn hình chính.
    """
    if extended_offset_x is None:
        # Mặc định màn hình mở rộng nằm bên phải màn hình chính
        extended_offset_x = main_screen_width
    global_x = int(extended_offset_x + px)
    global_y = int(extended_offset_y + py)
    print(f"Click chuột trái tại ({global_x}, {global_y}) trên extended monitor!")
    pyautogui.click(x=global_x, y=global_y, button='left')

# --- Ví dụ sử dụng ---
if __name__ == "__main__":
    # Định nghĩa 4 điểm tứ giác trên sàn (ngược chiều kim đồng hồ, đơn vị cm)
    # Ví dụ:
    A = (80, 50)
    B = (90, 150)
    C = (-90, 150)
    D = (-80, 50)
    floor_points = [A, B, C, D]
    projector_resolution = (800, 600)
    # Vị trí trên sàn cần tìm (X, Y)
    X, Y = 0, 50
    # Kiểm tra nếu vị trí nằm trong tứ giác
    if is_point_in_floor_polygon(floor_points, (X, Y)):
        px, py = floor_to_projector_pixel(floor_points, projector_resolution, (X, Y))
        print(f"Vị trí ({X}, {Y}) trên sàn ứng với pixel ({px:.2f}, {py:.2f}) trên máy chiếu.")
    else:
        print(f"Cảnh báo: Vị trí ({X}, {Y}) nằm ngoài tứ giác trên sàn!")
    
    # Nhập khoảng cách và góc cho 4 đỉnh (theo độ)
    polar_points = [
        (100, 10),  # r1, theta1
        (120, 45),  # r2, theta2
        (110, 100), # r3, theta3
        (90, 160)   # r4, theta4
    ]
    cartesian_points = polar_to_cartesian_list(polar_points, angle_unit='degree')
    print("Tọa độ các đỉnh tứ giác:")
    for i, (x, y) in enumerate(cartesian_points):
        print(f"Đỉnh {i+1}: ({x:.2f}, {y:.2f})")

    # 4 đỉnh tứ giác dưới dạng (r, theta)
    quad_polar = [
        (100, 270),
        (100, 90),
        (100*math.sqrt(2), 45),
        (100*math.sqrt(2), 315)
    ]
    projector_resolution = (800, 600)
    # Điểm trên sàn M(R, Theta)
    M_polar = (50, 0)
    px_py = projector_pixel_from_polar(quad_polar, projector_resolution, M_polar, angle_unit='degree')
    if px_py is not None:
        print(f"Điểm M{M_polar} trên sàn ứng với pixel ({px_py[0]:.2f}, {px_py[1]:.2f}) trên máy chiếu.")
        # Click chuột trái tại vị trí pixel đó trên màn hình
        screen_x, screen_y = int(px_py[0]), int(px_py[1])
        print(f"Click chuột trái tại ({screen_x}, {screen_y}) trên màn hình!")
        pyautogui.click(x=screen_x, y=screen_y, button='left')
    else:
        print(f"Điểm M{M_polar} nằm ngoài tứ giác trên sàn!")

    # Ví dụ sử dụng click_on_extended_monitor
    px, py = 400, 300  # Vị trí muốn click trên extended monitor (tọa độ local của extended monitor)
    # click_on_extended_monitor(px, py, main_screen_width=1920, main_screen_height=1080)
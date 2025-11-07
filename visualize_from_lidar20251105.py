import cv2
import numpy as np
import math
from rplidarc1 import RPLidar
import asyncio
import time
import threading

# Tỉ lệ chuyển đổi từ mm sang pixel (mm/pixel)
# SCALE_MM_PER_PIXEL = float(input("Nhập tỉ lệ (mm/pixel, ví dụ 10): "))
SCALE_MM_PER_PIXEL = 3

def read_lidar_data(lidar: RPLidar):

    data = []

    try:
        lidar_data = lidar.full_scan_map
        for distance, angle in lidar_data.items():
            if distance is None or angle is None:
                continue
            # Nếu angle là tuple, chỉ lấy phần tử đầu tiên hoặc cảnh báo
            if isinstance(angle, tuple):
                print(f"Warning: angle is tuple at distance={distance}: {angle}")
                if len(angle) > 0:
                    angle = angle[0]
                else:
                    continue
            try:
                distance_mm = float(distance)
                angle_deg = float(angle)
                data.append((distance_mm, angle_deg))
            except (ValueError, IndexError, TypeError) as e:
                print(f"Error parsing distance={distance}, angle={angle}: {e}")
                continue
    except (ValueError, IndexError) as e:
        pass
    
    return data


async def main(lidar: RPLidar):
    async def _run_scan_safe():
        """Start the scan but catch synchronous errors from simple_scan.

        If `lidar.simple_scan()` raises before returning a coroutine (e.g.
        because the device returned an unexpected response descriptor), we
        catch the exception here, log it, and set the stop_event so other
        tasks can shut down cleanly instead of being cancelled by the
        TaskGroup failing to create the task.
        """
        try:
            coro = lidar.simple_scan(export_scan_map=True)
            # If simple_scan returned a coroutine, await it
            if asyncio.iscoroutine(coro):
                await coro

        except Exception as e:
            print(f"Scan start failed: {e}")
            try:
                lidar.stop_event.set()
            except Exception:
                pass


        # Load background image
        bg_image = cv2.imread("bg1920x1080.png")
        if bg_image is None:
            print("Warning: bg1920x1080.png not found, using black background")
            bg_image = np.zeros((1080, 1920, 3), dtype=np.uint8)

        # Đọc dữ liệu LIDAR
        try:
            lidar_data = read_lidar_data(lidar)
        except FileNotFoundError:

            return
        except ValueError as e:

            return
        
        # Tạo bản sao của background để vẽ
        display_image = bg_image.copy()
        
        # Thống kê để báo cáo phạm vi dữ liệu
        min_dist = float('inf')
        max_dist = 0
        points_drawn = 0
        
        # Vẽ tất cả các điểm
        for distance_mm, angle_deg in lidar_data:
            # Cập nhật thống kê
            min_dist = min(min_dist, distance_mm)
            max_dist = max(max_dist, distance_mm)
            
            # Chuyển từ tọa độ cực (distance_mm, angle_deg) sang tọa độ Cartesian (x_mm, y_mm)
            x_mm = distance_mm * math.sin(math.radians(angle_deg))
            y_mm = distance_mm * math.cos(math.radians(angle_deg))
            
            # Chuyển sang pixel với tỉ lệ SCALE_MM_PER_PIXEL và tâm tại (960,540)
            x_px = int(960 + x_mm / SCALE_MM_PER_PIXEL)
            y_px = int(540 - y_mm / SCALE_MM_PER_PIXEL)  # trừ vì y tăng xuống dưới trong ảnh
            
            # Vẽ điểm nếu nằm trong khung hình
            if 0 <= x_px < 1920 and 0 <= y_px < 1080:
                cv2.circle(display_image, (x_px, y_px), 2, (0, 255, 0), -1)  # Màu xanh lá
                points_drawn += 1
        
        # Vẽ điểm tâm LIDAR
        cv2.circle(display_image, (960, 540), 5, (0, 0, 255), -1)  # Màu đỏ
        
        # Vẽ chú thích thông tin
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        text_color = (255, 255, 255)  # Màu trắng
        info_lines = [
            f"Points: {points_drawn}/{len(lidar_data)} displayed",
            f"Range: {min_dist:.0f}mm to {max_dist:.0f}mm",
            f"Scale: {SCALE_MM_PER_PIXEL}mm/pixel",
            "Green: LIDAR points",
            "Red: LIDAR center (960,540)",
            "Yellow: 0° reference line"
        ]
        
        # Vẽ đường thẳng góc 0 độ (theo trục y âm)
        # Đường thẳng dài 2000mm
        line_length_px = int(2000 / SCALE_MM_PER_PIXEL)  # Chuyển 2000mm sang pixel
        start_point = (960, 540)  # Tâm LIDAR
        end_point = (960, 540 - line_length_px)  # Điểm cuối của đường thẳng
        cv2.line(display_image, start_point, end_point, (0, 255, 255), 2)  # Màu vàng
        
        y = 30
        for line in info_lines:
            cv2.putText(display_image, line, (20, y), font, font_scale, text_color, 1)
            y += 25
        
        # Vẽ tứ giác ABCD
        quad_points = [
            (943, 302),   # A
            (1750, 329),  # B
            (1750, 31),   # C
            (943, 58)     # D
        ]
        
        # Chuyển đổi các điểm từ tọa độ cực sang pixel
        quad_pixels = []
        for dist_mm, angle_deg in quad_points:
            # Chuyển từ tọa độ cực sang Cartesian (mm)
            x_mm = dist_mm * math.sin(math.radians(angle_deg))
            y_mm = dist_mm * math.cos(math.radians(angle_deg))
            
            # Chuyển sang pixel với tỉ lệ SCALE_MM_PER_PIXEL và tâm tại (960,540)
            x_px = int(960 + x_mm / SCALE_MM_PER_PIXEL)
            y_px = int(540 - y_mm / SCALE_MM_PER_PIXEL)
            quad_pixels.append((x_px, y_px))
        
        # Vẽ tứ giác bằng các đoạn thẳng màu xanh dương
        for i in range(4):
            cv2.line(display_image, quad_pixels[i], quad_pixels[(i + 1) % 4], (255, 0, 0), 2)
        
        # Ghi chữ A, B, C, D tại các đỉnh
        labels = ['A', 'B', 'C', 'D']
        for (x, y), label in zip(quad_pixels, labels):
            cv2.putText(display_image, label, (x + 10, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        # Thêm thông tin về tứ giác vào chú thích
        info_lines.append("Blue: Quadrilateral ABCD")
        info_lines.append("A(943mm,302°) B(1750mm,329°)")
        info_lines.append("C(1750mm,31°) D(943mm,58°)")
        
        # Hiển thị ảnh
        cv2.imshow("LIDAR Data Visualization", display_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    async def wait_and_stop(t, event: asyncio.Event):
        """
        Wait for a specified time and then set an event to stop the scan.

        Args:
            t (float): The time to wait in seconds.
            event (asyncio.Event): The event to set when the time has elapsed.
        """
        print("Start wait for end event")
        await asyncio.sleep(t)
        print("Setting stop event")
        event.set()
        print(lidar.full_scan_map)

    async with asyncio.TaskGroup() as tg:
        tg.create_task(wait_and_stop(10, lidar.stop_event))
        tg.create_task(_run_scan_safe())

def polar_to_pixel(distance_mm, angle_deg, scale_mm_per_pixel=3, center=(960, 540)):
    x_mm = distance_mm * math.sin(math.radians(angle_deg))
    y_mm = distance_mm * math.cos(math.radians(angle_deg))
    x_px = int(center[0] + x_mm / scale_mm_per_pixel)
    y_px = int(center[1] - y_mm / scale_mm_per_pixel)
    return x_px, y_px

def draw_lidar_frame(full_scan_map, bg_image, scale_mm_per_pixel=3, center=(960, 540)):
    display_image = bg_image.copy()
    points_drawn = 0
    min_dist = float('inf')
    max_dist = 0
    # Vẽ các điểm LIDAR
    for key, value in full_scan_map.items():
        if value is None or len(value) < 4:
            continue
        quality, distance_mm, angle_deg, _ = value
        if distance_mm is None or angle_deg is None:
            continue
        try:
            distance_mm = float(distance_mm)
            angle_deg = float(angle_deg)
        except Exception:
            continue
        min_dist = min(min_dist, distance_mm)
        max_dist = max(max_dist, distance_mm)
        x_px, y_px = polar_to_pixel(distance_mm, angle_deg, scale_mm_per_pixel, center)
        if 0 <= x_px < 1920 and 0 <= y_px < 1080:
            cv2.circle(display_image, (x_px, y_px), 2, (0, 255, 0), -1)
            points_drawn += 1
    # Vẽ tâm LIDAR
    cv2.circle(display_image, center, 5, (0, 0, 255), -1)
    # Vẽ tứ giác ABCD
    quad_points = [
        (943, 302),   # A
        (1750, 329),  # B
        (1750, 31),   # C
        (943, 58)     # D
    ]
    quad_pixels = [polar_to_pixel(dist, ang, scale_mm_per_pixel, center) for dist, ang in quad_points]
    for i in range(4):
        cv2.line(display_image, quad_pixels[i], quad_pixels[(i + 1) % 4], (255, 0, 0), 2)
    labels = ['A', 'B', 'C', 'D']
    for (x, y), label in zip(quad_pixels, labels):
        cv2.putText(display_image, label, (x + 10, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    # Vẽ đường thẳng qua tâm lidar góc 0 độ
    line_length_mm = 2000
    x_end, y_end = polar_to_pixel(line_length_mm, 0, scale_mm_per_pixel, center)
    cv2.line(display_image, center, (x_end, y_end), (0, 255, 255), 2)
    # Thông tin
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    text_color = (255, 255, 255)
    info_lines = [
        f"Points: {points_drawn}/{len(full_scan_map)} displayed",
        f"Range: {min_dist:.0f}mm to {max_dist:.0f}mm",
        f"Scale: {scale_mm_per_pixel}mm/pixel",
        "Green: LIDAR points",
        "Red: LIDAR center (960,540)",
        "Blue: Quadrilateral ABCD",
        "Yellow: 0° reference line"
    ]
    y = 30
    for line in info_lines:
        cv2.putText(display_image, line, (20, y), font, font_scale, text_color, 1)
        y += 25
    return display_image

def visualize_lidar_continuous(lidar, scale_mm_per_pixel=3, center=(960, 540), bg_path="bg1920x1080.png"):
    bg_image = cv2.imread(bg_path)
    if bg_image is None:
        print(f"Warning: {bg_path} not found, using black background")
        bg_image = np.zeros((1080, 1920, 3), dtype=np.uint8)
    def update_loop():
        while True:
            frame = draw_lidar_frame(lidar.full_scan_map, bg_image, scale_mm_per_pixel, center)
            cv2.imshow("LIDAR Live Visualization", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            time.sleep(0.5)
        cv2.destroyAllWindows()
    t = threading.Thread(target=update_loop)
    t.daemon = True
    t.start()

if __name__ == "__main__":
    # logging.basicConfig(level=0)
    # lidar = RPLidar("/dev/ttyUSB0", 460800) # Linux/Mac
    lidar = RPLidar("\\\\.\\COM6", 460800) # Comm port 6 on Windows
    try:
        # Run the test that reads one sample and saves mapped pixel
        # test_exam(lidar)
        # Then run main scan routine
        asyncio.run(main(lidar))
        visualize_lidar_continuous(lidar, scale_mm_per_pixel=3, center=(960, 540), bg_path="bg1920x1080.png")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    
    finally:
        time.sleep(1)
        lidar.reset()
import cv2
import numpy as np
import math
from rplidarc1 import RPLidar
import asyncio
import time
import threading

async def main(lidar: RPLidar):
    """Run the LIDAR scan and the visualization concurrently.

    The scan (`lidar.simple_scan(export_scan_map=True)`) runs in a safe
    coroutine `_run_scan_safe`. The visualization runs in
    `visualize_task` which updates the OpenCV window every 0.5s. Both
    tasks are created inside an asyncio.TaskGroup so they run concurrently
    in the main thread (required for cv2.imshow on many platforms).
    Press ESC in the OpenCV window to stop the scan and exit.
    """
    async def _run_scan_safe():
        """Start the scan but catch synchronous errors from simple_scan.

        If `lidar.simple_scan()` raises before returning a coroutine we
        catch the exception and set the stop_event so visualization can
        exit cleanly.
        """
        try:
            coro = lidar.simple_scan(export_scan_map=True)
            if asyncio.iscoroutine(coro):
                await coro
        except Exception as e:
            print(f"Scan start failed: {e}")
            try:
                lidar.stop_event.set()
            except Exception:
                pass

    async def visualize_task():
        """Coroutine that updates the OpenCV window every 0.5s.

        Uses `draw_lidar_frame` to render the current `lidar.full_scan_map`.
        If the user presses ESC the function sets `lidar.stop_event` to
        request the scan to stop and then returns.
        """
        bg_image = cv2.imread("bg1920x1080.png")
        if bg_image is None:
            print("Warning: bg1920x1080.png not found, using black background")
            bg_image = np.zeros((1080, 1920, 3), dtype=np.uint8)

        window_name = "LIDAR Live Visualization"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        try:
            while not getattr(lidar, 'stop_event', asyncio.Event()).is_set():
                frame = draw_lidar_frame(lidar.full_scan_map, bg_image, scale_mm_per_pixel=3, center=(960, 540))
                cv2.imshow(window_name, frame)
                # Check key without blocking; keep responsive
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    try:
                        lidar.stop_event.set()
                    except Exception:
                        pass
                    break
                await asyncio.sleep(0.5)
        finally:
            cv2.destroyAllWindows()

    async with asyncio.TaskGroup() as tg:
        tg.create_task(_run_scan_safe())
        tg.create_task(visualize_task())


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
    for value in full_scan_map.values():
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

# Thêm vào cuối file để tự động chạy visualization sau khi main kết thúc
if __name__ == "__main__":
    # lidar = RPLidar("/dev/ttyUSB0", 460800) # Linux/Mac
    lidar = RPLidar("\\\\.\\COM6", 460800) # Comm port 6 on Windows
    try:
        # Start scan + visualization concurrently
        asyncio.run(main(lidar))
    except KeyboardInterrupt:
        pass
    finally:
        time.sleep(1)
        try:
            lidar.reset()
        except Exception:
            pass
import math
import numpy as np
import cv2
import time
import asyncio
from rplidarc1 import RPLidar
import pyautogui

def polar_to_cartesian(distance_mm, angle_deg):
    """Chuyển từ tọa độ cực (distance_mm, angle_deg) sang tọa độ Cartesian (x_mm, y_mm)."""
    angle_rad = angle_deg * math.pi / 180.0
    x_mm = - distance_mm * math.sin(angle_rad)
    y_mm = distance_mm * math.cos(angle_rad)
    return x_mm, y_mm

def get_perspective_transform_matrix(w:int = 1920, h:int = 1080):
    A=(943, 302)   # A (distance_mm, angle_deg)
    B=(1750, 329)  # B (distance_mm, angle_deg)
    C=(1750, 31)   # C (distance_mm, angle_deg)
    D=(943, 58)    # D (distance_mm, angle_deg)

    # Độ phân giải màn hình
    # w = 1920 # chiều rộng pixel
    # h = 1080 # chiều cao pixel
    TopLeft=[0,0] # Toạ độ pixel của điểm A sau khi chuyển đổi
    BottomLeft=[0,h-1] # Toạ độ pixel của điểm B sau khi chuyển
    BottomRight=[w-1,h-1] # Toạ độ pixel của điểm C sau khi chuyển đổi
    TopRight=[w-1,0] # Toạ độ pixel của điểm D

    quad_polar_points = [A, B, C, D]
    quad_cartesian_points = [polar_to_cartesian(distance_mm, angle_deg) for distance_mm, angle_deg in quad_polar_points]
    
    A = (580, 500)
    B = (710, 1450)
    C = (-710, 1450)
    D = (-580, 500)
    quad_cartesian_points = [A, B, C, D]

    quad_pixel_points = [TopLeft, BottomLeft, BottomRight, TopRight]
    source_points = np.array(quad_cartesian_points, dtype=np.float32)
    destination_points = np.array(quad_pixel_points, dtype=np.float32)
    MW = cv2.getPerspectiveTransform(source_points, destination_points)
    return MW
    
def detect_objects_in_lidar_data(full_scan_map, theta_scan):
    detected_pixel_points = []
    for value in full_scan_map.values():
        if value is None or len(value) < 4:
            continue
        quality, distance_mm, angle_deg, time = value
        if distance_mm is None or angle_deg is None:
            continue
        try:
            distance_mm = float(distance_mm)
            angle_deg = float(angle_deg)
        except Exception:
            continue
        if 0 <= angle_deg <= theta_scan or (360 - theta_scan) <= angle_deg < 360:
            M_cartesian = np.array([[polar_to_cartesian(distance_mm, angle_deg)]], dtype=np.float32)  # shape (1,1,2)
            x_px, y_px = cv2.perspectiveTransform(M_cartesian, MW)[0][0]
            if 0 <= x_px < w and 0 <= y_px < h:
                detected_pixel_points.append((int(x_px), int(y_px)))
    return detected_pixel_points

def cluster_detected_points(detected_pixel_points, distance_threshold=80):
    if not detected_pixel_points:
        return []
    clusters=[]
    n = len(detected_pixel_points)
    unassigned = set(range(n))
    while unassigned:
        i = unassigned.pop()
        cluster = [i]
        changed = True
        while changed:
            changed = False
            to_check = list(unassigned)
            for j in to_check:
                fits = True
                jx, jy = detected_pixel_points[j]
                for m in cluster:
                    mx, my = detected_pixel_points[m]
                    dx = jx - mx
                    dy = jy - my
                    if dx * dx + dy * dy >= (distance_threshold * distance_threshold):
                        fits = False
                        break
                if fits:
                    cluster.append(j)
                    unassigned.remove(j)
                    changed = True
        if len(cluster) >= 5:
            clusters.append(cluster)
    return clusters

async def main(lidar: RPLidar):

    async def _run_scan_safe():
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
        bg_image = cv2.imread("bg1920x1080.png")
        if bg_image is None:
            print("Warning: bg1920x1080.png not found, using black background")
            bg_image = np.zeros((h, w, 3), dtype=np.uint8)

        window_name = "LIDAR Live Visualization"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        try:
            while not getattr(lidar, 'stop_event', asyncio.Event()).is_set():
                display_image = bg_image.copy()
                detected_pixel_points = detect_objects_in_lidar_data(lidar.full_scan_map, theta_scan=80)
                clusters = cluster_detected_points(detected_pixel_points, distance_threshold=100)
                cluster_color = (255, 255, 0)  # vàng nhạt
                for ci, cluster in enumerate(clusters):
                    if not cluster:
                        continue
                    pts_px = np.array([[detected_pixel_points[idx][0], detected_pixel_points[idx][1]] for idx in cluster], dtype=np.float32)
                    if pts_px.shape[0] == 1:
                        cx, cy = pts_px[0]
                        radius = 8
                    else:
                        (cx, cy), radius = cv2.minEnclosingCircle(pts_px)
                    center_int = (int(cx), int(cy))
                    cv2.circle(display_image, center_int, int(radius) + 4, cluster_color, 2)
                    # Ghi nhãn số cụm
                    # cv2.putText(display_image, f"C{ci+1}", (center_int[0] + 6, center_int[1] + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, cluster_color, 2)
                
                cv2.imshow(window_name, display_image)
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

    async def stop_timer(t, event: asyncio.Event):
        await asyncio.sleep(t)
        event.set()

    async def mouse_task():
        while not getattr(lidar, 'stop_event', asyncio.Event()).is_set():
            detected_pixel_points = detect_objects_in_lidar_data(lidar.full_scan_map, theta_scan=80)
            # for point in detected_pixel_points:
            #     pyautogui.moveTo(point[0] + 1920, point[1] + 5)
            #     pyautogui.click()
            
            clusters = cluster_detected_points(detected_pixel_points, distance_threshold=80)
            for ci, cluster in enumerate(clusters):
                if not cluster:
                    continue
                pts_px = np.array([[detected_pixel_points[idx][0], detected_pixel_points[idx][1]] for idx in cluster], dtype=np.float32)
                
                if pts_px.shape[0] == 1:
                    cx, cy = pts_px[0]
                    radius = 8
                else:
                    (cx, cy), radius = cv2.minEnclosingCircle(pts_px)
                center_int = (int(cx), int(cy))
                # Di chuyển chuột đến tâm cụm + dịch một khoảng 10px
                pyautogui.moveTo(center_int[0] + 1920, center_int[1] + 15)
                pyautogui.click()
                
            await asyncio.sleep(0.1)

    async with asyncio.TaskGroup() as tg:
        tg.create_task(stop_timer(120, lidar.stop_event))
        tg.create_task(_run_scan_safe())
        # tg.create_task(visualize_task())
        tg.create_task(mouse_task())

if __name__ == "__main__":
    w, h = 1920, 1080
    MW = get_perspective_transform_matrix()
    theta_scan = 80 
    pyautogui.FAILSAFE = False

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
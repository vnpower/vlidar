from rplidarc1 import RPLidar
import logging
import asyncio
import time
import math
import cv2
import numpy as np
from sklearn.cluster import DBSCAN
import tkinter as tk
from PIL import Image, ImageDraw, ImageTk
import threading
import queue


class ScanVisualizer(threading.Thread):
    def __init__(self, lidar, left_detected_points, right_detected_points):
        super().__init__()
        self.lidar = lidar
        self.left_detected_points = left_detected_points
        self.right_detected_points = right_detected_points
        self.stop_event = threading.Event()
        # Load background image
        self.bg_image = cv2.imread("bg1920x1080.png")
        if self.bg_image is None:
            self.bg_image = np.zeros((1080, 1920, 3), dtype=np.uint8)
    
    def run(self):
        while not self.stop_event.is_set() and not self.lidar.stop_event.is_set():
            # Create a copy of background
            display_image = self.bg_image.copy()
            
            # Draw all scan points
            # Create a thread-safe copy of the scan map
            if self.lidar.full_scan_map:
                scan_map_copy = dict(self.lidar.full_scan_map)
                for angle, scan_data in scan_map_copy.items():
                    if scan_data is not None and len(scan_data) > 2 and scan_data[1] is not None:
                        distance_mm = scan_data[1]
                        angle_deg = scan_data[2]
                        
                        # Convert to cartesian coordinates (mm)
                        x_mm = distance_mm * math.sin(math.radians(angle_deg))
                        y_mm = distance_mm * math.cos(math.radians(angle_deg))
                        
                        # Convert to pixel coordinates (2mm/pixel, center at 960,540)
                        x_px = int(960 + x_mm / 2)
                        y_px = int(540 - y_mm / 2)
                        
                        # Check if point is in detected points (use pixel proximity against pixel-detected lists)
                        color = (0, 255, 0)  # Default green
                        try:
                            # self.left_detected_points/self.right_detected_points are lists of (px,py) floats
                            is_detected = False
                            for p in self.left_detected_points:
                                if p is None:
                                    continue
                                try:
                                    px_f, py_f = p
                                except Exception:
                                    continue
                                if px_f == -1:
                                    continue
                                if abs(px_f - x_px) <= 2 and abs(py_f - y_px) <= 2:
                                    is_detected = True
                                    break
                            if not is_detected:
                                for p in self.right_detected_points:
                                    if p is None:
                                        continue
                                    try:
                                        px_f, py_f = p
                                    except Exception:
                                        continue
                                    if px_f == -1:
                                        continue
                                    if abs(px_f - x_px) <= 2 and abs(py_f - y_px) <= 2:
                                        is_detected = True
                                        break
                            if is_detected:
                                color = (0, 0, 255)
                        except Exception:
                            # If anything goes wrong fall back to default color
                            color = (0, 255, 0)
                        
                        # Draw if within bounds
                        if 0 <= x_px < 1920 and 0 <= y_px < 1080:
                            cv2.circle(display_image, (x_px, y_px), 2, color, -1)
            
            # Draw LIDAR center
            cv2.circle(display_image, (960, 540), 5, (255, 255, 255), -1)
            
            # Show image
            cv2.imshow("LIDAR Scan Visualization", display_image)
            cv2.waitKey(1)
            
            # Wait for next update
            time.sleep(0.3)
    
    def stop(self):
        self.stop_event.set()
        cv2.destroyAllWindows()

class OverlayThread:
    """Run a persistent Tk overlay in a background thread and accept draw commands via a queue.

    The overlay window is borderless, topmost, and uses white as a transparent color.
    Send a list of clusters to draw where each cluster is a dict: {'pts': [(x,y),...], 'color': (r,g,b)}
    Coordinates must be in screen/projector pixels.
    """
    def __init__(self, size=(1920, 1080)):
        self.width, self.height = size
        self._q = queue.Queue()
        self._thread = None
        self._stop_event = threading.Event()

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        # put a sentinel to wake the queue
        try:
            self._q.put_nowait(None)
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def send_clusters(self, clusters):
        # clusters: list of {'pts': [(x,y),...], 'color': (r,g,b)}
        try:
            self._q.put_nowait(clusters)
        except Exception:
            pass

    def _run(self):
        try:
            root = tk.Tk()
            root.overrideredirect(True)
            root.attributes('-topmost', True)
            try:
                root.attributes('-transparentcolor', 'white')
            except Exception:
                pass
            # Position full-screen on primary monitor/projector
            try:
                root.geometry(f"{self.width}x{self.height}+0+0")
            except Exception:
                pass

            canvas = tk.Canvas(root, width=self.width, height=self.height, bg='white', highlightthickness=0)
            canvas.pack()

            # Keep a reference to the current PhotoImage
            canvas._photo = None

            def poll_queue():
                if self._stop_event.is_set():
                    try:
                        root.destroy()
                    except Exception:
                        pass
                    return
                try:
                    item = self._q.get_nowait()
                except queue.Empty:
                    root.after(50, poll_queue)
                    return

                if item is None:
                    root.after(50, poll_queue)
                    return

                clusters = item
                # Draw overlay image
                img = Image.new('RGBA', (self.width, self.height), (255,255,255,255))
                draw = ImageDraw.Draw(img)
                for c in clusters:
                    pts = c.get('pts', [])
                    color = c.get('color', (255,0,0))
                    if not pts:
                        continue
                    # draw polygon outline
                    try:
                        draw.polygon(pts, outline=color)
                    except Exception:
                        pass
                    # draw center
                    cx = int(sum([p[0] for p in pts]) / len(pts))
                    cy = int(sum([p[1] for p in pts]) / len(pts))
                    draw.ellipse((cx-6, cy-6, cx+6, cy+6), fill=color)

                photo = ImageTk.PhotoImage(img)
                canvas._photo = photo
                canvas.create_image(0, 0, anchor=tk.NW, image=photo)

                root.after(50, poll_queue)

            root.after(50, poll_queue)
            root.mainloop()
        except Exception:
            return


def _show_overlay_window(pts_px, duration_ms=1000):
    """
    Create a small transparent overlay window showing the cluster hull and center.
    pts_px: list of (x,y) pixel coordinates in screen/projector space.
    duration_ms: how long to show the overlay (milliseconds).
    """
    try:
        if not pts_px:
            return

        xs = [p[0] for p in pts_px]
        ys = [p[1] for p in pts_px]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        pad = 20
        width = maxx - minx + pad * 2
        height = maxy - miny + pad * 2

        # Create an RGBA image with white background (white will be made transparent)
        img = Image.new('RGBA', (width, height), (255, 255, 255, 255))
        draw = ImageDraw.Draw(img)

        # Translate points into local image coordinates
        local_pts = [((x - minx) + pad, (y - miny) + pad) for (x, y) in pts_px]

        # Draw filled polygon and center
        try:
            draw.polygon(local_pts, outline=(255, 0, 0), fill=None)
        except Exception:
            pass

        # Compute center and draw
        cx = int(sum([p[0] for p in local_pts]) / len(local_pts))
        cy = int(sum([p[1] for p in local_pts]) / len(local_pts))
        draw.ellipse((cx-6, cy-6, cx+6, cy+6), fill=(255,0,0))

        # Convert to PhotoImage
        root = tk.Tk()
        root.overrideredirect(True)
        root.attributes('-topmost', True)
        # Use white as transparent color
        try:
            root.attributes('-transparentcolor', 'white')
        except Exception:
            # Some platforms may not support transparentcolor; continue anyway
            pass

        # Position window at screen coordinates (minx-pad, miny-pad)
        geom_x = minx - pad
        geom_y = miny - pad
        # Ensure geometry values are integers
        try:
            root.geometry(f"{width}x{height}+{int(geom_x)}+{int(geom_y)}")
        except Exception:
            root.geometry(f"{width}x{height}+0+0")

        canvas = tk.Canvas(root, width=width, height=height, bg='white', highlightthickness=0)
        canvas.pack()

        photo = ImageTk.PhotoImage(img)
        # Keep a reference to prevent GC
        canvas._photo = photo
        canvas.create_image(0, 0, anchor=tk.NW, image=photo)

        root.after(duration_ms, root.destroy)
        root.mainloop()
    except Exception:
        # Overlay is a convenience; failures should not crash the main loop
        return

async def main(lidar: RPLidar):
    # Start scan visualizer thread (pass pixel-based detected lists so visualization can mark them)
    scan_viz = ScanVisualizer(lidar, left_detected_px_points, right_detected_px_points)
    scan_viz.start()
    
    try:
        async with asyncio.TaskGroup() as tg:
            tg.create_task(stop_timer(30, lidar.stop_event))
            # tg.create_task(detect_foot(0.5))
            tg.create_task(scan_foot(0.5))
            tg.create_task(continuous_safe_scan(lidar))
    finally:
        scan_viz.stop()
        scan_viz.join()

async def stop_timer(t, event: asyncio.Event):
    await asyncio.sleep(t)
    event.set()
    # print(lidar.full_scan_map)

async def scan_foot(interval: float = 0.5, theta_D: int = 45, theta_A: int = 315):
    # Use integer theta values computed in main (int_theta_A/int_theta_D) to match array sizes
    while not lidar.stop_event.is_set():
        await asyncio.sleep(interval)
        if lidar.full_scan_map:
            for i in range(int_theta_A*3, 360*3):
                scan_data = lidar.full_scan_map.get(i)
                # index into left_attention_map initialized with int_theta_A
                left_attention_map[i - int_theta_A*3] = scan_data
                if scan_data is not None and len(scan_data) > 2 and scan_data[1] is not None:
                    distance_mm = scan_data[1]
                    angle_deg = 360 - scan_data[2]
                    distance_max = yK / math.cos(math.pi * angle_deg / 180)
                    distance_min = yH / math.cos(math.pi * angle_deg / 180)
                    if (distance_mm < distance_max) and (distance_mm > distance_min):  # nếu khoảng cách thuộc phạm vi thì lưu vào mảng detected points
                        left_detected_points[i - int_theta_A*3] = (distance_mm * math.sin(math.radians(angle_deg)), distance_mm * math.cos(math.radians(angle_deg)))
                        px, py = floor_to_projector_pixel(projection_area_floor_points, projector_resolution, left_detected_points[i - int_theta_A*3])
                        left_detected_px_points[i - int_theta_A*3] = (px, py)
            for i in range(0, int_theta_D*3):
                scan_data = lidar.full_scan_map.get(i)
                right_attention_map[i] = scan_data
                if scan_data is not None and len(scan_data) > 2 and scan_data[1] is not None:
                    distance_mm = scan_data[1]
                    angle_deg = scan_data[2]
                    distance_max = yK / math.cos(math.pi * angle_deg / 180)
                    distance_min = yH / math.cos(math.pi * angle_deg / 180)
                    if (distance_mm < distance_max) and (distance_mm > distance_min):  # nếu khoảng cách thuộc phạm vi thì lưu vào mảng detected points
                        right_detected_points[i] = (distance_mm * math.sin(math.radians(angle_deg)), distance_mm * math.cos(math.radians(angle_deg)))
                        px, py = floor_to_projector_pixel(projection_area_floor_points, projector_resolution, right_detected_points[i])
                        right_detected_px_points[i] = (px, py)

async def detect_foot(interval: float = 0.5, theta_D: int = 45, theta_A: int = 315):
    # Start persistent overlay (uses global `projector_resolution`)
    overlay = OverlayThread(size=projector_resolution)
    overlay.start()
    try:
        while not lidar.stop_event.is_set():
            await asyncio.sleep(interval)
            if lidar.full_scan_map:
                for i in range(int_theta_A*3, 360*3):
                    scan_data = lidar.full_scan_map.get(i)
                    left_attention_map[i - int_theta_A*3] = scan_data
                    if scan_data is not None and len(scan_data) > 2 and scan_data[1] is not None:
                        distance_mm = scan_data[1]
                        angle_deg = 360 - scan_data[2]
                        distance_max = yK / math.cos(math.pi * angle_deg / 180)
                        distance_min = yH / math.cos(math.pi * angle_deg / 180)
                        if (distance_mm < distance_max) and (distance_mm > distance_min):  # nếu khoảng cách thuộc phạm vi thì lưu vào mảng detected points
                            left_detected_points[i - int_theta_A*3] = (distance_mm * math.sin(math.radians(angle_deg)), distance_mm * math.cos(math.radians(angle_deg)))
                            px, py = floor_to_projector_pixel(projection_area_floor_points, projector_resolution, left_detected_points[i - int_theta_A*3])
                            left_detected_px_points[i - int_theta_A*3] = (px, py)
                for i in range(0, int_theta_D*3):
                    scan_data = lidar.full_scan_map.get(i)
                    right_attention_map[i] = scan_data
                    if scan_data is not None and len(scan_data) > 2 and scan_data[1] is not None:
                        distance_mm = scan_data[1]
                        angle_deg = scan_data[2]
                        distance_max = yK / math.cos(math.pi * angle_deg / 180)
                        distance_min = yH / math.cos(math.pi * angle_deg / 180)
                        if (distance_mm < distance_max) and (distance_mm > distance_min):  # nếu khoảng cách thuộc phạm vi thì lưu vào mảng detected points
                            right_detected_points[i] = (distance_mm * math.sin(math.radians(angle_deg)), distance_mm * math.cos(math.radians(angle_deg)))
                            px, py = floor_to_projector_pixel(projection_area_floor_points, projector_resolution, right_detected_points[i])
                            right_detected_px_points[i] = (px, py)
                # --- Cluster left_detected_px_points and send to overlay ---
                valid_left_px = [p for p in left_detected_px_points if p[0] != -1]
                clusters_to_send = []
                if len(valid_left_px) > 0:
                    pts = np.array(valid_left_px, dtype=float)
                    if len(pts) >= 3:
                        clustering = DBSCAN(eps=30.0, min_samples=3).fit(pts)
                        labels = clustering.labels_
                    else:
                        labels = np.array([-1] * len(pts))

                    colors = [(255,0,0), (0,255,0), (0,0,255), (255,255,0), (255,0,255), (0,255,255)]
                    unique_labels = set(labels)
                    for lbl in unique_labels:
                        if lbl == -1:
                            continue
                        cluster_pts = pts[labels == lbl]
                        if cluster_pts.shape[0] == 0:
                            continue
                        pts_px = [(int(round(x)), int(round(y))) for x, y in cluster_pts]
                        clusters_to_send.append({'pts': pts_px, 'color': colors[lbl % len(colors)]})


                
                # Send clusters to overlay (non-blocking)
                try:
                    overlay.send_clusters(clusters_to_send)
                except Exception:
                    pass
    except asyncio.CancelledError:
        # Task cancelled during shutdown
        pass
    finally:
        # Stop overlay thread cleanly
        try:
            overlay.stop()
            cv2.destroyAllWindows()
        except Exception:
            pass

async def continuous_safe_scan(lidar: RPLidar):
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

if __name__ == "__main__":
    logging.basicConfig(level=0)
    # lidar = RPLidar("/dev/ttyUSB0", 460800) # Linux/Mac
    lidar = RPLidar("\\\\.\\COM6", 460800) # Comm port 6 on Windows
    w, h = 1920, 1080
    projector_resolution = (w, h)
    xA, yA = 800, 500 # mm
    A = (xA, yA)
    xB, yB = 900, 1500 # mm
    B = (xB, yB)
    xC, yC = -900, 1500 # mm
    C = (xC, yC)
    xD, yD = -800, 500 # mm
    D = (xD, yD)
    projection_area_floor_points = [A, B, C, D]
    xH, yH = 0, (yA + yD) / 2  # mm
    H = (xH, yH)
    xK, yK = 0, (yB + yC) / 2  # mm
    K = (xK, yK)
    left_projection_area = [A, B, K, H]
    right_projection_area = [H, K, C, D]
    theta_D = math.atan(-xD/yD) * 180 / math.pi
    theta_A = 360 + math.atan(xA/yA) * 180 / math.pi

    # integer degree counts used for array sizes (convert floats to ints)
    int_theta_D = int(round(theta_D))
    int_theta_A = int(round(theta_A))

    # Initialize arrays with proper sizes and default values
    # Use integer-degree conversions (multiply by 3 because maps use 1/3-degree resolution)
    left_attention_map = [None] * (360*3 - int_theta_A*3)  # left sector (int_theta_A to 360)
    left_detected_points = [(-1, 0)] * (360*3 - int_theta_A*3)
    left_detected_px_points = [(-1, 0)] * (360*3 - int_theta_A*3)
    right_attention_map = [None] * (int_theta_D*3)  # right sector (0 to int_theta_D)
    right_detected_points = [(-1, 0)] * (int_theta_D*3)
    right_detected_px_points = [(-1, 0)] * (int_theta_D*3)

    M = (1, 700)
    px, py = floor_to_projector_pixel(projection_area_floor_points, projector_resolution, M)
    print(f"Pixel trên máy chiếu của điểm M{M} là: ({px: 0.2f}, {py: 0.2f})")    

    try:
        asyncio.run(main(lidar))
    except KeyboardInterrupt:
        pass
    
    finally:
        time.sleep(1)
        lidar.reset()
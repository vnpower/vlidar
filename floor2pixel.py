"""floor2pixel.py

Compute projector pixel coordinates for a floor point M(x,y) given the
correspondence between four floor points A,B,C,D and projector pixels.

Mapping uses a 3x3 homography H so that [u, v, 1]^T ~ H [x, y, 1]^T.

Example mapping (used in __main__):
  A (x1, y1) -> (0,0)        top-left
  B (x2, y2) -> (0,600)      bottom-left
  C (x3, y3) -> (800,600)    bottom-right
  D (x4, y4) -> (800,0)      top-right

The file provides:
 - compute_homography(src, dst) -> H
 - map_point(H, x, y) -> (u, v)
 - a small runnable test when executed directly
"""

from __future__ import annotations
import numpy as np
import sys
from typing import Sequence, Tuple
import cv2
import math
import pyautogui  # Thêm ở đầu file nếu chưa có
import time
from datetime import datetime


def compute_homography(src_pts: Sequence[Tuple[float, float]], dst_pts: Sequence[Tuple[float, float]]) -> np.ndarray:
    """
    Compute 3x3 homography H mapping src_pts -> dst_pts.

    src_pts and dst_pts must be sequences of four (x,y) pairs, corresponding
    in the same order (A,B,C,D). Returns H as a 3x3 numpy array.
    """
    if len(src_pts) != 4 or len(dst_pts) != 4:
        raise ValueError("src_pts and dst_pts must each contain exactly 4 points")

    A = []
    b = []
    for (x, y), (u, v) in zip(src_pts, dst_pts):
        A.append([x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y])
        b.append(u)
        A.append([0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y])
        b.append(v)

    A = np.array(A, dtype=float)   # shape (8,8)
    b = np.array(b, dtype=float)   # shape (8,)

    # Solve A h = b for the 8 unknowns h11..h32 (h33 set to 1)
    # Use least-squares if A is near-singular for numerical robustness
    try:
        h = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        # fallback to least-squares solution
        h, *_ = np.linalg.lstsq(A, b, rcond=None)

    H = np.array([
        [h[0], h[1], h[2]],
        [h[3], h[4], h[5]],
        [h[6], h[7], 1.0]
    ], dtype=float)
    return H


def map_point(H: np.ndarray, x: float, y: float) -> Tuple[float, float]:
    """Apply homography H to floor point (x,y) and return pixel (u,v).

    Raises ZeroDivisionError if the mapped homogeneous w coordinate is zero.
    """
    p = H @ np.array([x, y, 1.0], dtype=float)
    w = p[2]
    if np.isclose(w, 0.0):
        raise ZeroDivisionError("Homography maps point to infinity (w ~ 0)")
    u = float(p[0] / w)
    v = float(p[1] / w)
    return u, v


def point_in_polygon(point: Tuple[float, float], polygon: Sequence[Tuple[float, float]]) -> bool:
    """Determine if a 2D point is inside a polygon using the ray casting algorithm.

    Returns True if the point is inside or on an edge, False otherwise.
    """
    x, y = point
    pts = list(polygon)
    inside = False
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        # Check if point is exactly on a vertex
        if np.isclose(x, x1) and np.isclose(y, y1):
            return True
        # Check if point is on the edge segment
        if (min(x1, x2) <= x <= max(x1, x2)) and (min(y1, y2) <= y <= max(y1, y2)):
            # cross product zero -> collinear
            if np.isclose((x2 - x1) * (y - y1) - (y2 - y1) * (x - x1), 0.0):
                return True
        # Ray casting test
        intersect = ((y1 > y) != (y2 > y)) and (
            x < (x2 - x1) * (y - y1) / (y2 - y1 + 0.0) + x1
        )
        if intersect:
            inside = not inside
    return inside

# --- Hàm chuyển đổi từ (r, theta) sang (x, y) cho 4 đỉnh tứ giác ---
def polar_to_cartesian_list(polar_list, angle_unit='degree'):
    """
    Chuyển danh sách [(r1, theta1), ...] sang [(x1, y1), ...].
    angle_unit: 'degree' hoặc 'radian'.
    Góc theta tính từ trục tung (Oy), chiều quay theo chiều kim đồng hồ.
    """
    cartesian = []
    for r, theta in polar_list:
        if angle_unit == 'degree':
            # theta_rad = math.radians(theta)
            theta_rad = 2*math.pi - math.radians(theta) # Đổi chiều quay
        else:
            # theta_rad = theta
            theta_rad = 2*math.pi - theta  # Đổi chiều quay
        x = r * math.sin(theta_rad)
        y = r * math.cos(theta_rad)
        cartesian.append((x, y))
    return cartesian


def _example1():
     
    x1, y1 = 1.0, 1.0       # A
    x2, y2 = 1.2, 2.5       # B
    x3, y3 = -1.2, 2.5      # C
    x4, y4 = -1.0, 1.0      # D

    # Provide points in the given A,B,C,D order then enforce CCW with A as first
    src = [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
    dst = [(0.0, 0.0), (0.0, 600.0), (800.0, 600.0), (800.0, 0.0)]
    # H = compute_homography(src, dst)
    H = compute_homography(src, dst)
    xm, ym = 0.6, 2.5
    u_m, v_m = map_point(H, xm, ym)
    print(f"Floor point M({xm:.6f}, {ym:.6f}) -> pixel (u,v) = ({u_m:.6f}, {v_m:.6f})")

def _example2():

    # Nhập khoảng cách và góc cho 4 đỉnh (theo độ)
    polar_points = [
        (1.141, 45),  # r1, theta1
        (2.773, 25.64),  # r2, theta2
        (2.773, 334.36), # r3, theta3
        (1.141, 315)   # r4, theta4
    ]
    cartesian_points = polar_to_cartesian_list(polar_points, angle_unit='degree')
    
    dst = [(0.0, 0.0), (0.0, 600.0), (800.0, 600.0), (800.0, 0.0)]
    H = compute_homography(cartesian_points, dst)
    M = (2, 350)
    (xm, ym) = polar_to_cartesian_list([M], angle_unit='degree')[0]
    u_m, v_m = map_point(H, xm, ym)
    print(f"Floor point M({xm:.6f}, {ym:.6f}) -> pixel (u,v) = ({u_m:.6f}, {v_m:.6f})")

def _example3():

    # Nhập khoảng cách và góc cho 4 đỉnh (theo độ)
    polar_points = [
        (1.141, 45),  # r1, theta1
        (2.773, 25.64),  # r2, theta2
        (2.773, 334.36), # r3, theta3
        (1.141, 315)   # r4, theta4
    ]
    cartesian_points = polar_to_cartesian_list(polar_points, angle_unit='degree')
    
    dst = [(0.0, 0.0), (0.0, 600.0), (800.0, 600.0), (800.0, 0.0)]
    H = compute_homography(cartesian_points, dst)
    M = (4, 350)
    (xm, ym) = polar_to_cartesian_list([M], angle_unit='degree')[0]
    inside = point_in_polygon((xm, ym), cartesian_points)
    if inside:
        u_m, v_m = map_point(H, xm, ym)
        print(f"Floor point M({xm:.6f}, {ym:.6f}) is inside the quadrilateral")
        print(f"Mapped pixel (u,v) = ({u_m:.6f}, {v_m:.6f})")
    else:
        print(f"Floor point M({xm:.6f}, {ym:.6f}) is outside the quadrilateral; skipping mapping")


def _example4():

    # Nhập khoảng cách và góc cho 4 đỉnh (theo độ)
    polar_points = [
        (1.141, 45),  # r1, theta1
        (2.773, 25.64),  # r2, theta2
        (2.773, 334.36), # r3, theta3
        (1.141, 315)   # r4, theta4
    ]
    cartesian_points = polar_to_cartesian_list(polar_points, angle_unit='degree')

    dst = [(0.0, 0.0), (0.0, 600.0), (800.0, 600.0), (800.0, 0.0)]
    H = compute_homography(cartesian_points, dst)

    # Try to get one scan sample from RPLidar; fallback to simulated sample
    def read_one_lidar_sample(timeout: float = 5.0):
        try:
            import asyncio
            from rplidarc1.scanner import RPLidar

            async def _read_once(lidar: RPLidar, timeout_s: float):
                try:
                    coro = lidar.simple_scan_timestamp(make_return_dict=False)
                except Exception:
                    return None
                task = asyncio.create_task(coro)
                try:
                    item = await asyncio.wait_for(lidar.output_queue.get(), timeout=timeout_s)
                    try:
                        lidar.stop_event.set()
                    except Exception:
                        pass
                    return item
                except Exception:
                    return None
                finally:
                    try:
                        task.cancel()
                    except Exception:
                        pass

            lidar = RPLidar()
            return asyncio.run(_read_once(lidar, timeout))
        except Exception:
            return None

    sample = read_one_lidar_sample(timeout=5.0)
    if sample is None:
        print('No LIDAR sample available; using simulated example M')
        M = (2, 350)  # (r meters, theta degrees)
    else:
        # Robust parsing of sample dict
        try:
            if isinstance(sample, dict):
                angle = sample.get('a_deg') or sample.get('angle') or sample.get('a')
                dist = sample.get('d_mm') or sample.get('distance') or sample.get('d') or sample.get('dist')
            else:
                # Unexpected type
                angle = None
                dist = None

            if angle is None:
                angle = 0.0
            angle = float(angle)

            if dist is None:
                d_val = 0.0
            else:
                d_val = float(dist)
                # Convert mm -> meters if value large
                if d_val > 50:  # >50 meters unlikely; assume mm
                    d_val = d_val / 1000.0

            M = (d_val, angle)
        except Exception:
            print('Unexpected LIDAR sample format; using simulated example M')
            M = (2, 350)

    # Convert to Cartesian, check inside, map and save
    (xm, ym) = polar_to_cartesian_list([M], angle_unit='degree')[0]
    inside = point_in_polygon((xm, ym), cartesian_points)
    if not inside:
        print(f"Floor point M({xm:.6f}, {ym:.6f}) is outside the quadrilateral; skipping mapping")
        return

    u_m, v_m = map_point(H, xm, ym)
    ts = time.time()
    iso = datetime.fromtimestamp(ts).isoformat()
    line = f"{iso},{ts:.6f},{u_m:.6f},{v_m:.6f}\n"
    try:
        with open('m_pixel.txt', 'a', encoding='utf-8') as f:
            f.write(line)
    except Exception as e:
        print('Failed to write pixel to file:', e)
        return

    print(f"Floor point M({xm:.6f}, {ym:.6f}) is inside the quadrilateral")
    print(f"Mapped pixel (u,v) = ({u_m:.6f}, {v_m:.6f}) and saved to m_pixel.txt")
if __name__ == '__main__':
    try:
        _example4()
    except Exception as e:
        print('Error during example run:', e, file=sys.stderr)
        raise

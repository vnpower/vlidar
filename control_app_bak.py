import pprint
import json
from rplidarc1 import RPLidar
import asyncio
import logging
import time
import math
import numpy as np
from typing import Sequence, Tuple

"""
This module provides an example of how to use the RPLidar class to perform
a simple scan and process the results.
"""

async def main(lidar: RPLidar):
    """
    Main coroutine that demonstrates how to use the RPLidar class.

    This function creates three tasks:
    1. A task to wait for a specified time and then stop the scan
    2. A task to print the scan results as they are received
    3. A task to perform the scan itself

    After the scan is complete, it prints the final scan results and resets the device.

    Args:
        lidar (RPLidar): An initialized RPLidar instance.
    """
    async def _run_scan_safe():
        """Start the scan but catch synchronous errors from simple_scan.

        If `lidar.simple_scan()` raises before returning a coroutine (e.g.
        because the device returned an unexpected response descriptor), we
        catch the exception here, log it, and set the stop_event so other
        tasks can shut down cleanly instead of being cancelled by the
        TaskGroup failing to create the task.
        """
        try:
            coro = lidar.simple_scan_timestamp(make_return_dict=True)
            # If simple_scan returned a coroutine, await it
            if asyncio.iscoroutine(coro):
                await coro
        except Exception as e:
            print(f"Scan start failed: {e}")
            try:
                lidar.stop_event.set()
            except Exception:
                pass

    async with asyncio.TaskGroup() as tg:
        tg.create_task(wait_and_stop(10, lidar.stop_event))
        # Pass lidar.output_dict so the printer can augment it with timestamps
        # tg.create_task(queue_printer(lidar.output_queue, lidar.stop_event, lidar.output_dict))
        tg.create_task(_run_scan_safe())

    if lidar.output_dict:
        pprint.pp(sorted(lidar.output_dict.items()))
        # Save the aggregated output_dict to a text file in JSON format
        try:
            # Sort the output_dict by key (ascending) and write as a dict
            sorted_items = sorted(lidar.output_dict.items(), key=lambda kv: kv[0])
            sorted_dict = {k: v for k, v in sorted_items}
            with open("lidar_output.txt", "w", encoding="utf-8") as f:
                json.dump(sorted_dict, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"Failed to write lidar.output_dict to file: {e}")

    lidar.reset()

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


if __name__ == "__main__":
    logging.basicConfig(level=0)
    # lidar = RPLidar("/dev/ttyUSB0", 460800) # Linux/Mac
    lidar = RPLidar("\\\\.\\COM6", 460800) # Comm port 6 on Windows

    try:
        asyncio.run(main(lidar))
    except KeyboardInterrupt:
        pass
    finally:
        time.sleep(1)
        lidar.reset()
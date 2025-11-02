import pprint
import json
from rplidarc1 import RPLidar
import asyncio
import logging
import time
from datetime import datetime
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
            coro = lidar.simple_scan(make_return_dict=True)
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
        # Periodic saver: every 0.5s dump lidar.output_dict (if any) to file
        async def periodic_task(interval: float = 0.5):
            try:
                while not lidar.stop_event.is_set():
                    if lidar.output_dict:
                        # helper to parse various sample formats into (r_meters, angle_deg)
                        def parse_polar(sample):
                            if sample is None:
                                return None
                            # tuple/list like (r, angle)
                            if isinstance(sample, (list, tuple)) and len(sample) >= 2:
                                try:
                                    r = float(sample[0])
                                    a = float(sample[1])
                                    # convert mm->m if value looks like mm
                                    if r > 50:
                                        r = r / 1000.0
                                    return (r, a)
                                except Exception:
                                    return None
                            # dict-like sample
                            if isinstance(sample, dict):
                                try:
                                    a = sample.get('a_deg') or sample.get('angle') or sample.get('a') or sample.get('ang')
                                    d = sample.get('d_mm') or sample.get('distance') or sample.get('d') or sample.get('dist') or sample.get('r')
                                    if a is None and d is None:
                                        return None
                                    a = float(a) if a is not None else 0.0
                                    d = float(d) if d is not None else 0.0
                                    if d > 50:
                                        d = d / 1000.0
                                    return (d, a)
                                except Exception:
                                    return None
                            # unsupported type
                            return None

                        # scan forward sector
                        for i in range(3 * 45):
                            if i in lidar.output_dict:
                                sample = lidar.output_dict[i]
                                polar = parse_polar(sample)
                                if polar is None:
                                    continue
                                r_m, angle_deg = polar
                                try:
                                    (xm, ym) = polar_to_cartesian_list([(r_m, angle_deg)], angle_unit='degree')[0]
                                except Exception:
                                    continue
                                inside = point_in_polygon((xm, ym), cartesian_points)
                                if inside:
                                    u_m, v_m = map_point(H, xm, ym)
                                    print(f"Floor point M({xm:.6f}, {ym:.6f}) is inside the quadrilateral")
                                    print(f"Mapped pixel (u,v) = ({u_m:.6f}, {v_m:.6f})")
                                else:
                                    print(f"Floor point M({xm:.6f}, {ym:.6f}) is outside the quadrilateral; skipping mapping")

                        # scan trailing sector
                        for i in range(315 * 3, 360 * 3):
                            if i in lidar.output_dict:
                                sample = lidar.output_dict[i]
                                polar = parse_polar(sample)
                                if polar is None:
                                    continue
                                r_m, angle_deg = polar
                                try:
                                    (xm, ym) = polar_to_cartesian_list([(r_m, angle_deg)], angle_unit='degree')[0]
                                except Exception:
                                    continue
                                inside = point_in_polygon((xm, ym), cartesian_points)
                                if inside:
                                    u_m, v_m = map_point(H, xm, ym)
                                    print(f"Floor point M({xm:.6f}, {ym:.6f}) is inside the quadrilateral")
                                    print(f"Mapped pixel (u,v) = ({u_m:.6f}, {v_m:.6f})")
                                else:
                                    print(f"Floor point M({xm:.6f}, {ym:.6f}) is outside the quadrilateral; skipping mapping")

                    await asyncio.sleep(interval)
            except asyncio.CancelledError:
                # Task cancelled during shutdown
                pass

        tg.create_task(periodic_task(0.5))
        tg.create_task(_run_scan_safe())


    # Final flush (once): write output_dict if present
    if lidar.output_dict:
        try:
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
    print(lidar.output_dict)


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

def test_exam(lidar: RPLidar | None = None):

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
    # Helper: read one sample from lidar (async) and return a sample dict or None
    def read_one_lidar_sample(timeout: float = 5.0):
        if lidar is None:
            return None
        try:
            import asyncio

            async def _read_once():
                try:
                    coro = lidar.simple_scan_timestamp(make_return_dict=False)
                except Exception:
                    return None
                # start the scan coroutine
                task = asyncio.create_task(coro)
                try:
                    item = await asyncio.wait_for(lidar.output_queue.get(), timeout=timeout)
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

            return asyncio.run(_read_once())
        except Exception:
            return None

    # Try to read a single LIDAR sample; fall back to a simulated sample if unavailable
    sample = read_one_lidar_sample(timeout=5.0)
    if sample is None:
        print('No LIDAR sample available; using simulated example M')
        M = (4, 350)  # (r meters, theta degrees)
    else:
        # parse sample (robust to common key names)
        try:
            if isinstance(sample, dict):
                angle = sample.get('a_deg') or sample.get('angle') or sample.get('a')
                dist = sample.get('d_mm') or sample.get('distance') or sample.get('d') or sample.get('dist')
            else:
                angle = None
                dist = None
            if angle is None:
                angle = 0.0
            angle = float(angle)
            if dist is None:
                d_val = float(sample.get('d', 0)) if isinstance(sample, dict) else 0.0
            else:
                d_val = float(dist)
                if d_val > 50:  # assume mm if large
                    d_val = d_val / 1000.0
            M = (d_val, angle)
        except Exception:
            print('Unexpected LIDAR sample format; using simulated example M')
            M = (4, 350)

    (xm, ym) = polar_to_cartesian_list([M], angle_unit='degree')[0]
    inside = point_in_polygon((xm, ym), cartesian_points)
    if not inside:
        print(f"Floor point M({xm:.6f}, {ym:.6f}) is outside the quadrilateral; skipping mapping")
        return

    u_m, v_m = map_point(H, xm, ym)
    # Use current time for scan timestamp; if sample contains a timestamp you can use it
    ts = time.time()
    iso = datetime.fromtimestamp(ts).isoformat()
    line = f"{iso},{ts:.6f},{u_m:.6f},{v_m:.6f}\n"
    try:
        with open('m_pixel.txt', 'a', encoding='utf-8') as f:
            f.write(line)
    except Exception as e:
        print(f'Failed to write pixel to file: {e}')
        return

    print(f"Floor point M({xm:.6f}, {ym:.6f}) is inside the quadrilateral")
    print(f"Mapped pixel (u,v) = ({u_m:.6f}, {v_m:.6f}) and saved to m_pixel.txt")

if __name__ == "__main__":
    logging.basicConfig(level=0)
    # lidar = RPLidar("/dev/ttyUSB0", 460800) # Linux/Mac
    lidar = RPLidar("\\\\.\\COM6", 460800) # Comm port 6 on Windows
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

    try:
        # Run the test that reads one sample and saves mapped pixel
        # test_exam(lidar)
        # Then run main scan routine
        asyncio.run(main(lidar))
    except KeyboardInterrupt:
        pass
    finally:
        time.sleep(1)
        lidar.reset()
import pprint
import json
from rplidarc1 import RPLidar
import asyncio
import logging
import time

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


async def queue_printer(q: asyncio.Queue, event: asyncio.Event, output_dict: dict | None = None):
    """
    Print scan results from a queue until a stop event is set.

    This coroutine continuously reads scan results from the queue and prints them.
    If the queue has fewer than 10 items, it sleeps briefly to allow more data
    to accumulate.

    Args:
        q (asyncio.Queue): The queue containing scan results.
        event (asyncio.Event): An event that signals when to stop printing.
    """
    print("Start queue listener")
    while not event.is_set():
        if q.qsize() < 10:
            print("Printer sleeping for more data")
            await asyncio.sleep(0.1)
        out = await q.get()
        # If the producer signals end-of-stream with None, break out
        if out is None:
            print("Received end-of-stream sentinel; exiting printer.")
            break
        # Ensure there's a timestamp for this record. If parser already provided
        # one (e.g., under key 'ts' or 'timestamp') keep it; otherwise add now.
        ts = None
        if isinstance(out, dict):
            ts = out.get("ts") or out.get("timestamp")
        if not ts:
            ts = time.time()
            if isinstance(out, dict):
                out["ts"] = ts

        print(out)

        # If an output_dict was provided, update it with distance+timestamp for the angle
        try:
            if output_dict is not None and isinstance(out, dict):
                a = out.get("a_deg")
                d = out.get("d_mm")
                if a is not None:
                    # store a dict with distance and timestamp so final dump includes ts
                    output_dict[a] = {"d_mm": d, "ts": ts}
        except Exception as e:
            print(f"Failed to update output_dict with timestamp: {e}")

    # Append the output dict as a JSON line to scan_output.txt
        try:
            with open("scan_output.txt", "a", encoding="utf-8") as f:
                f.write(json.dumps(out, ensure_ascii=False) + "\n")
        except Exception as e:
            # Don't crash printer on file errors; log to console
            print(f"Failed to write out to file: {e}")



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

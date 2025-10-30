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
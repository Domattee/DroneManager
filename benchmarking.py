import os
import psutil
import asyncio
import time

from dronemanager.dronemanager import DroneManager
from dronemanager.drone import DroneMAVSDK

import logging

PIDs = []

async def main():
    global PIDs
    PIDs.append(os.getpid())
    dm = DroneManager(DroneMAVSDK, log_to_console=True, console_log_level=logging.INFO)
    await dm.connect_to_drone("tom", None, None, "udp://172.23.39.82:18570", log_telemetry=True, telemetry_frequency=10)
    PIDs.append(dm.drones["tom"]._server_process.pid)
    await drone_back_and_forth("tom", dm)

    await dm.connect_to_drone("jerry", None, None, "udp://172.23.39.82:18571", log_telemetry=True, telemetry_frequency=10)
    PIDs.append(dm.drones["jerry"]._server_process.pid)
    await asyncio.gather(*[drone_back_and_forth(drone, dm) for drone in ["tom", "jerry"]])

    await dm.connect_to_drone("spike", None, None, "udp://172.23.39.82:18572", log_telemetry=True, telemetry_frequency=10)
    PIDs.append(dm.drones["spike"]._server_process.pid)
    await asyncio.gather(*[drone_back_and_forth(drone, dm) for drone in ["tom", "jerry", "spike"]])
    await dm.close()
    PIDs = []

async def drone_back_and_forth(drone, dm):
    await dm.arm(drone)
    await dm.takeoff(drone)
    await dm.fly_to(drone, local=[10, 10, -3])
    await dm.fly_to(drone, local=[0, 0, -3])
    await dm.land(drone)
    await dm.disarm(drone)

def check_cpu():
    global PIDs
    pid_dict: dict[int: psutil.Process] = {}
    while True:
        for pid in PIDs:
            if pid not in pid_dict:
                pid_dict[pid] = psutil.Process(pid=pid)
        for pid, p in pid_dict.items():
            p: psutil.Process
            p.memory_full_info()
            p.cpu_percent()
            # TODO: Collect and display information
        time.sleep(0.01)

if __name__ == "__main__":
    asyncio.run(main())
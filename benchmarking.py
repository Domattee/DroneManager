import os
from multiprocessing import Process, Event
import psutil
import yappi
from psutil import NoSuchProcess
import asyncio
import time
import matplotlib.pyplot as plt

from dronemanager.core import DroneManager
from dronemanager.app import DroneApp
from dronemanager.drone import DroneMAVSDK

import pyinstrument

import logging

FILE_PATH = "benchmarking_data.csv"
log_telemetry = False
with_app = False


async def measure_longterm():
    pid = os.getpid()
    stop_cpu_checker = Event()
    profile_process = Process(target=check_cpu, args=(pid, stop_cpu_checker))
    profile_process.start()
    try:
        dm = DroneManager(DroneMAVSDK, log_to_console=True, console_log_level=logging.INFO)
        if with_app:
            app = DroneApp(dm, logger=dm.logger)
            asyncio.get_running_loop().run_in_executor(None, app.run)
        await dm.connect_to_drone("tom", None, None, "udp://:14540", log_telemetry=log_telemetry,
                                  telemetry_frequency=10)
        await dm.connect_to_drone("jerry", None, None, "udp://:14541", log_telemetry=log_telemetry,
                                  telemetry_frequency=10)
        await dm.connect_to_drone("spike", None, None, "udp://:14542", log_telemetry=log_telemetry,
                                  telemetry_frequency=10)
        await asyncio.sleep(3)
        await asyncio.gather(*[drone_back_and_forth_timed(drone, dm, 1800) for drone in ["tom", "jerry", "spike"]])
    finally:
        if with_app:
            try:
                await app.command_screen.exit()  # Will probably throw an error about being in the wrong loop
            except:
                pass
        else:
            await dm.close()
        stop_cpu_checker.set()
        profile_process.join()

async def measure_cpu():
    pid = os.getpid()
    stop_cpu_checker = Event()
    profile_process = Process(target=check_cpu, args=(pid, stop_cpu_checker))
    profile_process.start()
    try:
        dm = DroneManager(DroneMAVSDK, log_to_console=True, console_log_level=logging.INFO)
        if with_app:
            app = DroneApp(dm, logger=dm.logger)
            asyncio.get_running_loop().run_in_executor(None, app.run)
        await dm.connect_to_drone("tom", None, None, "udp://:14540", log_telemetry=log_telemetry, telemetry_frequency=10)
        await drone_back_and_forth("tom", dm)

        await dm.connect_to_drone("jerry", None, None, "udp://:14541", log_telemetry=log_telemetry, telemetry_frequency=10)
        await asyncio.gather(*[drone_back_and_forth(drone, dm) for drone in ["tom", "jerry"]])

        await dm.connect_to_drone("spike", None, None, "udp://:14542", log_telemetry=log_telemetry, telemetry_frequency=10)
        await asyncio.gather(*[drone_back_and_forth(drone, dm) for drone in ["tom", "jerry", "spike"]])
    finally:
        if with_app:
            try:
                await app.command_screen.exit()  # Will probably throw an error about being in the wrong loop
            except:
                pass
        else:
            await dm.close()
        stop_cpu_checker.set()
        profile_process.join()


async def profile_multidrone_yappi():
    try:
        yappi.start()
        dm = DroneManager(DroneMAVSDK, log_to_console=True, console_log_level=logging.INFO)
        await dm.connect_to_drone("tom", None, None, "udp://:14540", log_telemetry=log_telemetry, telemetry_frequency=10)
        await drone_back_and_forth("tom", dm)
        yappi.stop()
        stats = yappi.get_func_stats()
        stats.save("yappi_1.prof", "pstat")
        yappi.clear_stats()

        yappi.start()
        await dm.connect_to_drone("jerry", None, None, "udp://:14541", log_telemetry=log_telemetry, telemetry_frequency=10)
        await asyncio.gather(*[drone_back_and_forth(drone, dm) for drone in ["tom", "jerry"]])
        yappi.stop()
        stats = yappi.get_func_stats()
        stats.save("yappi_2.prof", "pstat")
        yappi.clear_stats()


        yappi.start()
        await dm.connect_to_drone("spike", None, None, "udp://:14542", log_telemetry=log_telemetry, telemetry_frequency=10)
        await asyncio.gather(*[drone_back_and_forth(drone, dm) for drone in ["tom", "jerry", "spike"]])
        yappi.stop()
        stats = yappi.get_func_stats()
        stats.save("yappi_3.prof", "pstat")
        yappi.clear_stats()
    finally:
        await dm.close()


async def profile_multidrone():
    try:
        profiler1 = pyinstrument.Profiler()
        with profiler1:
            dm = DroneManager(DroneMAVSDK, log_to_console=True, console_log_level=logging.INFO)
            await dm.connect_to_drone("tom", None, None, "udp://:14540", log_telemetry=log_telemetry, telemetry_frequency=10)
            await drone_back_and_forth("tom", dm)
        with open("profile1.html", "w") as f:
            f.write(profiler1.output_html())

        profiler2 = pyinstrument.Profiler()
        with profiler2:
            await dm.connect_to_drone("jerry", None, None, "udp://:14541", log_telemetry=log_telemetry, telemetry_frequency=10)
            await asyncio.gather(*[drone_back_and_forth(drone, dm) for drone in ["tom", "jerry"]])
        with open("profile2.html", "w") as f:
            f.write(profiler2.output_html())

        profiler3 = pyinstrument.Profiler()
        with profiler3:
            await dm.connect_to_drone("spike", None, None, "udp://:14542", log_telemetry=log_telemetry, telemetry_frequency=10)
            await asyncio.gather(*[drone_back_and_forth(drone, dm) for drone in ["tom", "jerry", "spike"]])
        with open("profile3.html", "w") as f:
            f.write(profiler3.output_html())
    finally:
        await dm.close()


async def drone_back_and_forth(drone, dm):
    await dm.arm(drone)
    await dm.takeoff(drone)
    await dm.fly_to(drone, local=[10, 10, -3], tol=0.4)
    await dm.fly_to(drone, local=[0, 0, -3], tol=0.4)
    await dm.land(drone)
    await asyncio.sleep(3)
    await dm.disarm(drone)
    await dm.change_flightmode(drone, "hold")


async def drone_back_and_forth_timed(drone, dm, duration):
    await dm.arm(drone)
    await dm.takeoff(drone)
    start_time = time.time()
    while time.time() - start_time < duration:
        await dm.fly_to(drone, local=[10, 10, -3], tol=0.4)
        await dm.fly_to(drone, local=[0, 0, -3], tol=0.4)
    await dm.land(drone)
    await asyncio.sleep(3)
    await dm.disarm(drone)
    await dm.change_flightmode(drone, "hold")


def check_cpu(pid, stopping: Event):
    dt = 0.1
    usages = []
    #logger = logging.getLogger("manager")
    counter = 0
    p = psutil.Process(pid=pid)
    while not stopping.is_set():
        cpu_frame = 0
        mem_frame = 0
        try:
            mem_frame += p.memory_full_info().uss / 1e6
            cpu_frame += p.cpu_percent()
        except NoSuchProcess:
            continue
        timer = counter * dt
        usages.append((timer, cpu_frame, mem_frame))
        time.sleep(dt)
        counter += 1
        #if counter % 100 == 0:
        #    print("\t".join([f"{pid}:{max(usages[pid][1])}:{max(usages[pid][2])}" for pid in usages]))
    with open(FILE_PATH, mode="wt", encoding="utf8") as f:
        f.write("time, cpu, mem\n")
        for item in usages:
            f.write(f"{item[0]}, {item[1]}, {item[2]}\n")
    print("avg cpu", sum([usages[i][1] for i in range(len(usages))]) / len(usages),
          "avg mem", sum([usages[i][2] for i in range(len(usages))]) / len(usages))


def make_plots():
    t = []
    cpus = []
    mems = []
    cpu_emas = []
    ema_factor = 0.9
    with open(FILE_PATH, mode="rt", encoding="utf8") as f:
        for line in f.readlines():
            splits = line.strip().split(", ")
            if splits[0] == "time":
                continue
            stamp = float(splits[0])
            cpu = float(splits[1])
            mem = float(splits[2])
            t.append(stamp)
            cpus.append(cpu)
            mems.append(mem)
            if len(cpu_emas) == 0:
                cpu_ema = cpu
            else:
                cpu_ema = cpu_emas[-1] * ema_factor + cpu * (1 - ema_factor)
            cpu_emas.append(cpu_ema)
    fig, (cpu_ax, mem_ax) = plt.subplots(2, 1, figsize=(10, 5), sharex=True, constrained_layout=True)
    cpu_ax: plt.Axes
    cpu_ax.grid(visible=True)
    cpu_ax.set_ylabel("Single core CPU Usage [%]")
    mem_ax.set_ylabel("Memory Usage [Mb]")
    mem_ax.set_xlabel("Time [s]")
    #colors = plt.colormaps["tab10"].colors
    cpu_ax.plot(t, cpu_emas, color="blue", label="Average")
    cpu_ax.fill_between(t, 0, cpus, color="blue", alpha=0.2, label="Actual")
    mem_ax.plot(t, mems, color="blue")
    cpu_ax.legend()
    plt.show()


if __name__ == "__main__":
    #asyncio.run(profile_multidrone())
    #asyncio.run(profile_multidrone_yappi())
    #asyncio.run(measure_cpu())
    #asyncio.run(measure_longterm())
    make_plots()
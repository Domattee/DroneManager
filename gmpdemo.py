import asyncio
import traceback

from dronecontrol.dronemanager import DroneManager
from dronecontrol.drone import DroneMAVSDK

# TODO: Turn into mission


async def main():
    # Connect to drone
    try:
        dm = DroneManager(DroneMAVSDK)
        result = await dm.connect_to_drone("tom", None, None, "udp://:14540", 10)
        key = None
        while key not in ["y", "Y", "n", "N"]:
            key = input("Connected to drone, press y to continue or n to abort")
            if key in ["y", "Y"]:
                break
            if key in ["n", "N"]:
                return

        # Set area fence
        dm.set_fence("tom", -5.5, 5.5, -2.5, 2.5, 3)

        # Takeoff drone and fly to start position
        print("Taking off and flying to starting position")
        await dm.arm("tom")
        await dm.takeoff("tom", altitude=1.5)
        await dm.fly_to("tom", local=[-5, 0, -1.5], yaw=0)
        key = None
        while key not in ["y", "Y", "n", "N"]:
            key = input("Connected to drone, press y to continue or n to abort")
            if key in ["y", "Y"]:
                break
            if key in ["n", "N"]:
                return
        # fly to end position.
        await dm.fly_to("tom", local=[5, 0, -1.5], yaw=0)
        await dm.land("tom")
    except Exception:
        print(traceback.format_exception())


if __name__ == "__main__":
    asyncio.run(main())

import asyncio
import math

from collections import defaultdict

from mavsdk.gimbal import GimbalError
from mavsdk.gimbal import ControlMode as MAVControlMode
from mavsdk.gimbal import GimbalMode as MAVGimbalMode
from mavsdk.gimbal import SendMode as MAVSendMode

from dronecontrol.plugin import Plugin
from dronecontrol.utils import relative_gps

# TODO: Big refactor, swap control management to gimbal class, better gimbal presence checking
# TODO: Request gimbal information messages
# TODO: Check how MAVSDK python wants this to work. With gimbal_id != 0, MAVSDK sends a message with ID 1000 and the FC returns Failure 3 "unsupported".
# TODO: The multiple ID issue seems to be a bug in MAVSDK python. Sometimes the device id in the command 1000 is the component id, then returns failure 2 (invalid), sometimes it is an int (returns failure 3)

ControlMode = MAVControlMode
GimbalMode = MAVGimbalMode
SendMode = MAVSendMode


class GimbalPlugin(Plugin):
    PREFIX = "gimbal"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "add": self.add_gimbals,
            "remove": self.remove_gimbal,
            "take": self.take_control,
            "release": self.release_control,
            "set": self.set_gimbal_angles,
            "point": self.point_gimbal_at,
            "mode": self.set_gimbal_mode,
            "status": self.status,
        }
        self.background_functions = [
        ]
        self.gimbals: dict[str, Gimbal] = {}  # Dictionary with drone names as keys and gimbals as values

    async def start(self):
        self.logger.debug("Starting Gimbal plugin...")
        await super().start()
        for drone in self.dm.drones:
            await self.add_gimbals(drone)

    async def close(self):
        """ Removes all gimbals """
        await super().close()
        coros = [self.remove_gimbal(drone) for drone in self.gimbals]
        await asyncio.gather(*coros)

    def check_has_gimbal(self, drone):
        if drone not in self.gimbals:
            self.logger.warning(f"No drone with gimbal {drone}!")
            return False
        return True

    async def add_gimbals(self, drone: str):
        """ Add Gimbals from/for a given drone to the plugin"""
        self.logger.info(f"Adding gimbal to drone {drone}")
        try:
            drone_object = self.dm.drones[drone]
            self.gimbals[drone] = Gimbal(self.logger, drone_object)
        except Exception as e:
            self.logger.warning(repr(e))

    async def remove_gimbal(self, drone: str):
        """ Remove a gimbal from the plugin"""
        self.logger.info(f"Removing gimbal to drone {drone}")
        gimbal = self.gimbals.pop(drone)
        await gimbal.close()
        del gimbal

    async def status(self, drone: str):
        if self.check_has_gimbal(drone):
            self.gimbals[drone].log_status()

    async def take_control(self, drone: str, gimbal_id: int = None):
        if self.check_has_gimbal(drone):
            await self.gimbals[drone].take_control(gimbal_id)

    async def release_control(self, drone: str, gimbal_id: int = None):
        if self.check_has_gimbal(drone):
            await self.gimbals[drone].release_control(gimbal_id)

    async def set_gimbal_angles(self, drone: str, roll: float, pitch: float, yaw: float, gimbal_id: int = None):
        if self.check_has_gimbal(drone):
            try:
                self.logger.debug(f"Setting gimbal angles for gimbal {gimbal_id} on {drone} to {roll, pitch, yaw}")
                return await self.gimbals[drone].set_gimbal_angles(gimbal_id, roll, pitch, yaw)
            except Exception as e:
                self.logger.error("Couldn't set angles due to an exception!")
                self.logger.debug(repr(e), exc_info=True)
                return False
        return False

    async def point_gimbal_at(self, drone: str, x1: float, x2: float, x3: float, relative: bool = False, gimbal_id: int = None):
        if self.check_has_gimbal(drone):
            if relative:
                return await self.gimbals[drone].point_gimbal_at_relative(gimbal_id, x1, x2, x3)
            else:
                return await self.gimbals[drone].point_gimbal_at(gimbal_id, x1, x2, x3)
        return False

    async def set_gimbal_mode(self, drone: str, mode: str, gimbal_id: int = None):
        if self.check_has_gimbal(drone):
            return await self.gimbals[drone].set_gimbal_mode(gimbal_id, mode)
        return False


class Gimbal:

    def __init__(self, logger, drone):
        self.logger = logger
        self.drone = drone

        self.gimbal_list = set()  # Currently not used
        self.gimbal_id = None
        self.roll: dict[int, float] = {}
        self.pitch: dict[int, float] = {}
        self.yaw: dict[int, float] = {}
        self.mode: dict[int, GimbalMode] = {}
        self.primary_control: dict[int, tuple[float, float]] = {}
        self.secondary_control: dict[int, tuple[float, float]] = {}
        self._running_tasks = set()
        self._start_background_tasks()

    def _start_background_tasks(self):
        self._running_tasks.add(asyncio.create_task(self._check_gimbal_attitude()))
        self._running_tasks.add(asyncio.create_task(self._check_gimbal_control()))
        #self._running_tasks.add(asyncio.create_task(self._check_connected_gimbals()))

    async def close(self):
        try:
            await self.release_control(None)
        except Exception as e:
            self.logger.warning("Exception while closing gimbal object, check logs")
            self.logger.debug(repr(e), exc_info=True)
        for task in self._running_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()

    async def _check_connected_gimbals(self):
        async for gimballist in self.drone.system.gimbal.gimbal_list():
            new_gimbals = [gimbal.gimbal_id for gimbal in gimballist.gimbals]
            for gimbal_id in new_gimbals:
                if gimbal_id not in self.gimbal_list:
                    self._add_gimbal(gimbal_id)
            for cur_gimbal_id in self.gimbal_list:
                if cur_gimbal_id not in new_gimbals:
                    self._remove_gimbal(cur_gimbal_id)

    def _add_gimbal(self, gimbal_id):
        self.roll[gimbal_id] = math.nan
        self.pitch[gimbal_id] = math.nan
        self.yaw[gimbal_id] = math.nan
        self.primary_control[gimbal_id] = (math.nan, math.nan)
        self.secondary_control[gimbal_id] = (math.nan, math.nan)
        self.mode[gimbal_id] = GimbalMode.YAW_FOLLOW
        self.gimbal_list.add(gimbal_id)

    def _remove_gimbal(self, gimbal_id):
        self.roll.pop(gimbal_id)
        self.pitch.pop(gimbal_id)
        self.yaw.pop(gimbal_id)
        self.primary_control.pop(gimbal_id)
        self.secondary_control.pop(gimbal_id)
        self.gimbal_list.remove(gimbal_id)

    async def _check_gimbal_attitude(self):
        async for attitude in self.drone.system.gimbal.attitude():
            gimbal_id = attitude.gimbal_id
            rpy = attitude.euler_angle_forward
            self.gimbal_id = gimbal_id
            self.roll[gimbal_id] = rpy.roll_deg
            self.pitch[gimbal_id] = rpy.pitch_deg
            self.yaw[gimbal_id] = rpy.yaw_deg

    async def _check_gimbal_control(self):
        async for gimbal_control in self.drone.system.gimbal.control_status():
            gimbal_id = gimbal_control.gimbal_id
            self.gimbal_id = gimbal_id
            self.primary_control[gimbal_id] = (gimbal_control.sysid_primary_control, gimbal_control.compid_primary_control)
            self.secondary_control[gimbal_id] = (gimbal_control.sysid_secondary_control, gimbal_control.compid_secondary_control)

    def log_status(self):
        gimbal_id = self.gimbal_id
        self.logger.info(f"Gimbal {gimbal_id} control P:{self.primary_control[gimbal_id]}, "
                             f"S: {self.secondary_control[gimbal_id]}, Roll: {self.roll[gimbal_id]}, "
                             f"Pitch: {self.pitch[gimbal_id]}, Yaw: {self.yaw[gimbal_id]}")

    async def take_control(self, gimbal_id):
        if gimbal_id is None:
            gimbal_id = self.gimbal_id
        self.logger.info(f"Taking control over gimbal {gimbal_id}")
        await self._error_wrapper(self.drone.system.gimbal.take_control, gimbal_id, ControlMode.PRIMARY)

    async def release_control(self, gimbal_id):
        if gimbal_id is None:
            gimbal_id = self.gimbal_id
        self.logger.info(f"Releasing control over gimbal {gimbal_id}")
        await self._error_wrapper(self.drone.system.gimbal.release_control, gimbal_id)

    async def point_gimbal_at(self, gimbal_id, lat, long, amsl):
        if gimbal_id is None:
            gimbal_id = self.gimbal_id
        return await self._error_wrapper(self.drone.system.gimbal.set_roi_location, gimbal_id, lat, long, amsl)

    async def point_gimbal_at_relative(self, gimbal_id, x, y, z):
        if gimbal_id is None:
            gimbal_id = self.gimbal_id
        lat, long, amsl = relative_gps(x, y, z, *self.drone.position_global[:3])
        return await self.point_gimbal_at(gimbal_id, lat, long, amsl)

    async def set_gimbal_angles(self, gimbal_id, pitch, yaw):
        if gimbal_id is None:
            gimbal_id = self.gimbal_id
        await self._error_wrapper(self.drone.system.gimbal.set_angles, gimbal_id, 0, pitch, yaw,
                                  self.mode.get(gimbal_id, GimbalMode.YAW_FOLLOW), SendMode.ONCE)

    async def set_gimbal_rates(self, gimbal_id, pitch_rate, yaw_rate):
        if gimbal_id is None:
            gimbal_id = self.gimbal_id
        await self._error_wrapper(self.drone.system.gimbal.set_angular_rates, 0, pitch_rate, yaw_rate,
                                  self.mode.get(gimbal_id, GimbalMode.YAW_FOLLOW), SendMode.ONCE)

    async def set_gimbal_mode(self, gimbal_id, mode):
        assert mode in ["follow", "lock"]
        if gimbal_id is None:
            gimbal_id = self.gimbal_id
        if mode == "follow":
            self.mode[gimbal_id] = GimbalMode.YAW_FOLLOW
        elif mode == "lock":
            self.mode[gimbal_id] =GimbalMode.YAW_LOCK

    async def _error_wrapper(self, func, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except GimbalError as e:
            self.logger.error(f"GimbalError: {e._result.result_str}")
            return False
        return True

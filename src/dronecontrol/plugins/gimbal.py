import asyncio
import math

import mavsdk.gimbal
from mavsdk.gimbal import GimbalError
from mavsdk.gimbal import ControlMode as MAVControlMode
from mavsdk.gimbal import GimbalMode as MAVGimbalMode
from mavsdk.gimbal import SendMode as MAVSendMode

from dronecontrol.plugin import Plugin
from dronecontrol.utils import relative_gps

# TODO: Big refactor, swap control management to gimbal class, better gimbal presence checking
# TODO: Specific gimbal type for our POS gremsy that doesn't properly implement mavlink

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

    def gimbal_incontrol(self, drone, gimbal_id):
        return self.check_has_gimbal(drone) and self.gimbals[drone].in_control(gimbal_id)

    async def add_gimbals(self, drone: str):
        """ Add Gimbals from/for a given drone to the plugin"""
        self.logger.info(f"Adding gimbal to drone {drone}")
        try:
            drone_object = self.dm.drones[drone]
            self.gimbals[drone] = Gimbal(self.logger, self.dm, drone_object)
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
            res = await self.gimbals[drone].take_control(gimbal_id)
            if res:
                self.logger.info(f"Took control over gimbal {gimbal_id} for {drone}")
            else:
                self.logger.info(f"Couldn't take control over gimbal on {drone}!")


    async def release_control(self, drone: str, gimbal_id: int = None):
        if self.check_has_gimbal(drone):
            self.logger.info(f"Releasing control over gimbal {gimbal_id}")
            await self.gimbals[drone].release_control(gimbal_id)

    async def set_gimbal_angles(self, drone: str, pitch: float, yaw: float, gimbal_id: int = None):
        if self.check_has_gimbal(drone):
            try:
                return await self.gimbals[drone].set_gimbal_angles(gimbal_id, pitch, yaw)
            except Exception as e:
                self.logger.error("Couldn't set angles due to an exception!")
                self.logger.debug(repr(e), exc_info=True)
                return False
        return False

    async def set_gimbal_rate(self, drone: str, pitch_rate: float, yaw_rate: float, gimbal_id: int = None):
        if self.check_has_gimbal(drone):
            try:
                self.logger.debug(f"Setting gimbal rates for gimbal {gimbal_id} on {drone} to {pitch_rate, yaw_rate}")
                return await self.gimbals[drone].set_gimbal_angles(gimbal_id, pitch_rate, yaw_rate)
            except Exception as e:
                self.logger.error("Couldn't set angular rates due to an exception!")
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

    def __init__(self, logger, dm, drone):
        self.logger = logger
        self.dm = dm
        self.drone = drone

        self.gimbal_list: set[int] = set()
        self.roll: dict[int, float] = {}
        self.pitch: dict[int, float] = {}
        self.yaw: dict[int, float] = {}
        self.mode: dict[int, GimbalMode] = {}
        self.primary_control: dict[int, tuple[float, float]] = {}
        self.secondary_control: dict[int, tuple[float, float]] = {}
        self._add_gimbal(0)
        self._running_tasks = set()
        self.update_rate = 5  # How often we request updates on control and attitude
        self._start_background_tasks()

    def _start_background_tasks(self):
        self._running_tasks.add(asyncio.create_task(self._check_gimbal_attitude_cont()))
        self._running_tasks.add(asyncio.create_task(self._check_gimbal_control_cont()))
#        self._running_tasks.add(asyncio.create_task(self._check_connected_gimbals()))

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
            self.logger.debug(f"Found gimbals: {new_gimbals}")
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
        while True:
            try:
                for gimbal_id in self.gimbal_list:
                    self._running_tasks.add(asyncio.create_task(self._check_gimbal_attitude_gimbal(gimbal_id)))
                await asyncio.sleep(1/self.update_rate)
            except Exception as e:
                self.logger.warning(f"Exception in the gimbal attitude check function! See logs for details.")
                self.logger.debug(repr(e), exc_info=True)

    async def _check_gimbal_attitude_cont(self):
        gimbal_id = self._gimbal_id_check(None)
        async for attitude in self.drone.system.gimbal.attitude():
            rpy = attitude.euler_angle_forward
            self.roll[gimbal_id] = rpy.roll_deg
            self.pitch[gimbal_id] = rpy.pitch_deg
            self.yaw[gimbal_id] = rpy.yaw_deg
            self.logger.debug(f"Desired {gimbal_id} actual {attitude.gimbal_id}, eulers: {rpy.roll_deg, rpy.pitch_deg, rpy.yaw_deg}, ")

    async def _check_gimbal_attitude_gimbal(self, gimbal_id):
        attitude = await self.drone.system.gimbal.get_attitude(gimbal_id)
        attitude: mavsdk.gimbal.Attitude
        rpy = attitude.euler_angle_forward
        self.roll[gimbal_id] = rpy.roll_deg
        self.pitch[gimbal_id] = rpy.pitch_deg
        self.yaw[gimbal_id] = rpy.yaw_deg
        self.logger.debug(f"Desired {gimbal_id} actual {attitude.gimbal_id}, "
                          f"eulers: {rpy.roll_deg, rpy.pitch_deg, rpy.yaw_deg}, "
                          f"eulers NED: {attitude.euler_angle_north.roll_deg, attitude.euler_angle_north.pitch_deg, attitude.euler_angle_north.yaw_deg}, "
                          f"quat: {attitude.quaternion_forward.__dict__},"
                          f"quat ned {attitude.quaternion_north.__dict__}")

    async def _check_gimbal_control_cont(self):
        gimbal_id = self._gimbal_id_check(None)
        async for gimbal_control in self.drone.system.gimbal.control_status():
            self.primary_control[gimbal_id] = (gimbal_control.sysid_primary_control,
                                               gimbal_control.compid_primary_control)
            self.secondary_control[gimbal_id] = (gimbal_control.sysid_secondary_control,
                                                 gimbal_control.compid_secondary_control)
            self.logger.debug(f"{gimbal_id, gimbal_control.sysid_primary_control}")

    async def _check_gimbal_control(self):
        while True:
            try:
                for gimbal_id in self.gimbal_list:
                    self._running_tasks.add(asyncio.create_task(self._check_gimbal_control_gimbal(gimbal_id)))
                await asyncio.sleep(1/self.update_rate)
            except Exception as e:
                self.logger.warning(f"Exception in the gimbal control check function! See logs for details.")
                self.logger.debug(repr(e), exc_info=True)

    async def _check_gimbal_control_gimbal(self, gimbal_id):
        gimbal_control = await self.drone.system.gimbal.get_control_status(gimbal_id)
        self.primary_control[gimbal_id] = (gimbal_control.sysid_primary_control, gimbal_control.compid_primary_control)
        self.secondary_control[gimbal_id] = (gimbal_control.sysid_secondary_control, gimbal_control.compid_secondary_control)
        self.logger.debug(f"{gimbal_id, gimbal_control.sysid_primary_control}")

    def log_status(self):
        for gimbal_id in self.gimbal_list:
            self.logger.info(f"Gimbal {gimbal_id} control P:{self.primary_control[gimbal_id]}, "
                             f"S: {self.secondary_control[gimbal_id]}, Roll: {self.roll[gimbal_id]}, "
                             f"Pitch: {self.pitch[gimbal_id]}, Yaw: {self.yaw[gimbal_id]}")

    def _gimbal_id_check(self, gimbal_id):
        if gimbal_id is None and len(self.gimbal_list) == 1:
            return list(self.gimbal_list)[0]
        elif gimbal_id is None:
            self.logger.warning("Missing gimbal id argument for drone with multiple gimbals!")
            raise RuntimeError()
        else:
            return gimbal_id

    def in_control(self, gimbal_id: int | None):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        return self.primary_control[gimbal_id] == (self.dm.system_id, self.dm.component_id)

    async def take_control(self, gimbal_id: int | None):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        self.logger.info(f"Taking control of gimbal {gimbal_id}")
        return await self._error_wrapper(self.drone.system.gimbal.take_control, gimbal_id, ControlMode.PRIMARY)

    async def release_control(self, gimbal_id: int | None):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        return await self._error_wrapper(self.drone.system.gimbal.release_control, gimbal_id)

    async def point_gimbal_at(self, gimbal_id: int | None, lat, long, amsl):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        if not self.in_control(gimbal_id):
            self.logger.warning("Trying to point a gimbal we don't control, might not work!")
        return await self._error_wrapper(self.drone.system.gimbal.set_roi_location, gimbal_id, lat, long, amsl)

    async def point_gimbal_at_relative(self, gimbal_id: int | None, x, y, z):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        if not self.in_control(gimbal_id):
            self.logger.warning("Trying to point a gimbal we don't control, might not work!")
        lat, long, amsl = relative_gps(x, y, z, *self.drone.position_global[:3])
        return await self.point_gimbal_at(gimbal_id, lat, long, amsl)

    async def set_gimbal_angles(self, gimbal_id: int | None, pitch, yaw):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        if not self.in_control(gimbal_id):
            self.logger.warning("Trying to point a gimbal we don't control, might not work!")
        self.logger.debug(f"Setting gimbal angles for gimbal {gimbal_id} to {pitch, yaw}")
        return await self._error_wrapper(self.drone.system.gimbal.set_angles, gimbal_id, 0, pitch, yaw,
                                  self.mode.get(gimbal_id, GimbalMode.YAW_FOLLOW), SendMode.ONCE)

    async def set_gimbal_rates(self, gimbal_id: int | None, pitch_rate, yaw_rate):
        gimbal_id = self._gimbal_id_check(gimbal_id)
        if not self.in_control(gimbal_id):
            self.logger.warning("Trying to point a gimbal we don't control, might not work!")
        return await self._error_wrapper(self.drone.system.gimbal.set_angular_rates, 0, pitch_rate, yaw_rate,
                                  self.mode.get(gimbal_id, GimbalMode.YAW_FOLLOW), SendMode.ONCE)

    async def set_gimbal_mode(self, gimbal_id: int | None, mode):
        assert mode in ["follow", "lock"]
        gimbal_id = self._gimbal_id_check(gimbal_id)
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

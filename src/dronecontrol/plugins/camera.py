""" Plugin for controlling MAVSDK Cameras """
import asyncio

import mavsdk.camera

from dronecontrol.plugin import Plugin

# TODO: Check if a drone supports camera commands when adding cameras
# TODO: Probably want to set in the add command what type of camera, and then have classes for each specific camera,
#  i.e. Workswell, the raspberry pi ones, etc. to allow specific commands, such as swapping between vis and ir
# TODO: Work with the parameters and the param file somehow, have to get and parse the xmls


class CameraPlugin(Plugin):
    PREFIX = "cam"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "add": self.add_cameras,
            "remove": self.remove_camera,
            "status": self.status,
            "settings": self.get_settings,
            "photo": self.take_picture,
            "start": self.start_video,
            "stop": self.stop_video,
            "zoom": self.set_zoom,
            "swap": self.swap_ir_vis,
        }
        self.background_functions = [
        ]
        self.cameras: dict[str, Camera] = {}  # Dictionary with drone names as keys and gimbals as values

    async def start(self):
        self.logger.debug("Starting Camera plugin...")
        await super().start()
        for drone in self.dm.drones:
            await self.add_cameras(drone)

    async def close(self):
        """ Removes all cameras """
        await super().close()
        coros = [self.remove_camera(drone) for drone in self.cameras]
        await asyncio.gather(*coros)

    def check_has_camera(self, drone):
        if drone not in self.cameras:
            self.logger.warning(f"No drone with camera {drone}!")
            return False
        return True

    async def add_cameras(self, drone: str, camera_id: int = 100):
        """ Add cameras from/for a given drone to the plugin"""
        self.logger.info(f"Adding camera to drone {drone}")
        try:
            drone_object = self.dm.drones[drone]
            self.cameras[drone] = Camera(self.logger, self.dm, drone_object, camera_id=camera_id)
        except Exception as e:
            self.logger.warning(repr(e))

    async def remove_camera(self, drone: str):
        """ Remove a camera from the plugin"""
        self.logger.info(f"Removing camera from drone {drone}")
        camera = self.cameras.pop(drone)
        await camera.close()
        del camera

    async def status(self, drone: str):
        if self.check_has_camera(drone):
            self.cameras[drone].log_status()

    async def get_settings(self, drone: str):
        if self.check_has_camera(drone):
            return await self.cameras[drone].get_settings()
        return False

    async def take_picture(self, drone: str):
        if self.check_has_camera(drone):
            return await self.cameras[drone].take_picture()
        return False

    async def start_video(self, drone: str):
        if self.check_has_camera(drone):
            return await self.cameras[drone].start_video()
        return False

    async def stop_video(self, drone: str):
        if self.check_has_camera(drone):
            return await self.cameras[drone].stop_video()
        return False

    async def set_zoom(self, drone: str, zoom: float):
        if self.check_has_camera(drone):
            return await self.cameras[drone].set_zoom(zoom)
        return False

    async def swap_ir_vis(self, drone: str):
        if self.check_has_camera(drone):
            return await self.cameras[drone].swap_ir_vis()
        return False


class Camera:

    def __init__(self, logger, dm, drone, camera_id: int = 100):
        self.logger = logger
        self.dm = dm
        self.drone = drone

        self.camera_id = camera_id
        self._running_tasks = set()
        self._start_background_tasks()
        self._request_cam_info()

    def _start_background_tasks(self):
        self._running_tasks.add(asyncio.create_task(self._capture_info_updates()))

    async def _capture_info_updates(self):
        self.drone.system.camera: mavsdk.camera.Camera
        async for capture_info in self.drone.system.camera.capture_info():
            capture_info: mavsdk.camera.CaptureInfo
            self.logger.info(f"Capture update: Camera {capture_info.component_id} {'succeeded' if capture_info.is_success else 'failed'} with photo at {capture_info.time_utc_us}: {capture_info.file_url}")

    async def close(self):
        for task in self._running_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()

    def log_status(self):
        camera_id = self.camera_id
        self.logger.info(f"Camera {camera_id}")

    async def take_picture(self, ir=True, vis=True):
        flags = 0
        if ir:
            flags += 1
        if vis:
            flags += 8
        self.drone.mav_conn.send_cmd_long(target_component=100, cmd=2000, param3=1.0, param5=int(flags))

    async def start_video(self, ir=True, vis=True):
        flags = 0
        if ir:
            flags += 2
        if vis:
            flags += 4
        self.drone.mav_conn.send_cmd_long(target_component=self.camera_id, cmd=2500, param1=int(flags), param2=2)

    async def stop_video(self):
        self.drone.mav_conn.send_cmd_long(target_component=self.camera_id, cmd=2501)

    def _request_cam_info(self):
        self.drone.mav_conn.request_message(target_component=self.camera_id, message_id=259)

    async def get_settings(self):
        # TODO: Wait and handle answers, have to work in mavpassthrough for that
        # TODO: In MAVPassthrough, three "listen to" functions:
        #  Single response, with matching check, async, returns when matching message returns, with timeout option
        #  Multi response, with either timeout or number of messages, with matching, returns when timeout or number of messages is complete
        #  Indefinite response
        # TODO: Handle here for checks on if we have all the messages we want or not
        # TODO: Changing and reading setting values in the CLI
        self.drone.mav_conn.send_param_ext_request_list(self.drone.mav_conn.drone_system, self.camera_id)
        #self.drone.mav_conn.request_message(target_component=self.camera_id, message_id=260, param1=1)

    async def set_zoom(self, zoom):
        self.drone.mav_conn.send_cmd_long(target_component=self.camera_id, cmd=203, param2=zoom, param5=0)

    async def swap_ir_vis(self):
        self.drone.mav_conn.send_cmd_long(target_component=self.camera_id, cmd=530, param6=1)

    async def _error_wrapper(self, func, *args, **kwargs):
        try:
            res = await func(*args, **kwargs)
        except mavsdk.camera.CameraError as e:
            self.logger.error(f"CameraError: {e._result.result_str}")
            return False
        return res

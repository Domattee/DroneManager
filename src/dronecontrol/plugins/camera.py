""" Plugin for controlling MAVSDK Cameras """
import asyncio

import mavsdk.camera

from dronecontrol.plugin import Plugin

# TODO: Check if a drone supports camera commands when adding cameras
# TODO: Probably want to set in the add command what type of camera, and then have classes for each specific camera,
#  i.e. Workswell, the raspberry pi ones, etc. to allow specific commands, such as swapping to ir


class CameraPlugin(Plugin):
    PREFIX = "cam"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "add": self.add_cameras,
            "remove": self.remove_camera,
            "status": self.status,
            "list": self.list_cameras,
            "settings": self.get_settings,
            "photo": self.take_picture,
            "start": self.start_video,
            "stop": self.stop_video,
            "zoom": self.set_zoom,
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

    async def add_cameras(self, drone: str):
        """ Add cameras from/for a given drone to the plugin"""
        self.logger.info(f"Adding camera to drone {drone}")
        try:
            drone_object = self.dm.drones[drone]
            self.cameras[drone] = Camera(self.logger, self.dm, drone_object)
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

    async def list_cameras(self, drone: str):
        if self.check_has_camera(drone):
            return await self.cameras[drone].list_cameras()

    async def get_settings(self, drone: str, camera_id: int = None):
        if self.check_has_camera(drone):
            return await self.cameras[drone].get_settings(camera_id)

    async def take_picture(self, drone: str, camera_id: int = None):
        if self.check_has_camera(drone):
            return await self.cameras[drone].take_picture(camera_id)

    async def start_video(self, drone: str, camera_id: int = None):
        if self.check_has_camera(drone):
            return await self.cameras[drone].start_video(camera_id)

    async def stop_video(self, drone: str, camera_id: int = None):
        if self.check_has_camera(drone):
            return await self.cameras[drone].stop_video(camera_id)

    async def set_zoom(self, drone: str, zoom: float, camera_id: int = None):
        if self.check_has_camera(drone):
            return await self.cameras[drone].set_zoom(zoom, camera_id)


class Camera:

    def __init__(self, logger, dm, drone):
        self.logger = logger
        self.dm = dm
        self.drone = drone

        self.camera_list = set()  # Currently not used
        self.camera_id = None
        self.cameras = set()
        self._running_tasks = set()
        self._start_background_tasks()

    def _start_background_tasks(self):
        #self._running_tasks.add(asyncio.create_task(self._check_gimbal_attitude()))
        self._running_tasks.add(asyncio.create_task(self._capture_info_updates()))
        self._running_tasks.add(asyncio.create_task(self._check_connected_cameras()))

    async def _check_connected_cameras(self):
        async for camera_list in self.drone.system.camera.camera_list():
            for camera in camera_list:
                self.camera_id = camera.component_id
                self.cameras.add((camera.component_id, camera.model_name,
                                  camera.horizontal_resolution_px, camera.vertical_resolution_px))

    async def _capture_info_updates(self):
        self.drone.system.camera: mavsdk.camera.Camera
        async for capture_info in self.drone.system.camera.capture_info():
            capture_info: mavsdk.camera.CaptureInfo
            self.logger.info(f"Capture update: Camera {capture_info.component_id} {'succeeded' if capture_info.is_success else 'failed'} with photo at {capture_info.time_utc_us}")

    async def close(self):
        for task in self._running_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()

    def log_status(self):
        camera_id = self.camera_id
        self.logger.info(f"Camera {camera_id}")

    async def list_cameras(self):
        for camera in self.cameras:
            self.logger.info(f"Connected camera: {camera}")

    async def take_picture(self, camera_id: int = None, ir=True, vis=True):
        if camera_id is None:
            camera_id = self.camera_id
        return await self._error_wrapper(self.drone.system.camera.take_photo, camera_id)
        #flags = 0
        #if ir:
        #    flags += 1
        #if vis:
        #    flags += 8
        #await self.mav_conn.send_cmd_long(target_system=self.drone_system_id, target_component=100, cmd=2000,
        #                                  param3=1.0,
        #                                  param5=int(flags),
        #                                  )

    async def start_video(self, camera_id: int = None, ir=True, vis=True):
        if camera_id is None:
            camera_id = self.camera_id
        return await self._error_wrapper(self.drone.system.camera.start_video, camera_id)
        #flags = 0
        #if ir:
        #    flags += 2
        #if vis:
        #    flags += 4
        #await self.mav_conn.send_cmd_long(target_system=self.drone_system_id, target_component=100, cmd=2500,
        #                                  param1=int(flags),
        #                                  param2=2,
        #                                  )

    async def stop_video(self, camera_id: int = None):
        if camera_id is None:
            camera_id = self.camera_id
        return await self._error_wrapper(self.drone.system.camera.stop_video, camera_id)
        #await self.mav_conn.send_cmd_long(target_system=self.drone_system_id, target_component=100, cmd=2501, )

    async def get_settings(self, camera_id: int = None):
        if camera_id is None:
            camera_id = self.camera_id
        cur_settings = await self._error_wrapper(self.drone.system.camera.get_current_settings, camera_id)
        if cur_settings:
            for setting in cur_settings:
                self.logger.info(f"Setting {setting.setting_id} set to {setting.option}, {setting.is_range}. "
                                 f"Description: {setting.setting_description}")
        #await self.mav_conn.send_cmd_long(target_system=self.drone_system_id, target_component=100, cmd=521, )
        #await self.mav_conn.send_cmd_long(target_system=self.drone_system_id, target_component=100, cmd=522, param1=1)

    async def set_zoom(self, zoom, camera_id: int = None):
        if camera_id is None:
            camera_id = self.camera_id
        return await self._error_wrapper(self.drone.system.camera.zoom_range, camera_id, zoom)
        #await self.mav_conn.send_cmd_long(target_system=self.drone_system_id, target_component=100, cmd=203,
        #                                  param2=zoom,
        #                                  param5=0)

    async def _error_wrapper(self, func, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except mavsdk.camera.CameraError as e:
            self.logger.error(f"GimbalError: {e._result.result_str}")
            return False
        return True

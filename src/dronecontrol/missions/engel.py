""" Mission for ENGEL data collection
Capture images and combine with weather and position info from drone, storing them
Functions to retake the same position as in a previous image and take another image.
"""
import asyncio
import numpy as np

from dronecontrol.navigation.core import Waypoint, WayPointType
from dronecontrol.plugins.mission import Mission
from dronecontrol.sensors.ecowitt import WeatherData


class ENGELCaptureInfo:

    def __init__(self, time_utc, lat, long, amsl, drone_att, gimbal_att, cam_file, weather_data: WeatherData):
        self.time_utc = time_utc
        self.lat = lat
        self.long = long
        self.amsl = amsl
        self.drone_att = drone_att
        self.gimbal_att = gimbal_att
        self.file_on_cam = cam_file
        self.weather_data = weather_data


class ENGELDataMission(Mission):
    """ Data collection mission for ENGEL

    """

    DEPENDENCIES = ["gimbal", "camera", "sensor.ecowitt"]

    def __init__(self, dm, logger, name="engel"):
        super().__init__(dm, logger, name)
        mission_cli_commands = {
            "connect", self.connect,
            "capture", self.do_capture,
            "save", self.save_to_file,
        }
        self.cli_commands.update(mission_cli_commands)
        self.weather_sensor = None
        self.launch_point: Waypoint | None = None  # A dictionary with latitude and longitude and amsl values
        self.rtl_height = 10  # Height above launch point for return
        self.captures: list[ENGELCaptureInfo] = []
        self.background_functions = [
        ]
        self.drone_name = None

    async def connect(self):
        """ Connect to the Leitstand sensor"""
        connected = await self.dm.ecowitt.connect("192.168.1.41")
        if connected:
            self.weather_sensor = self.dm.ecowitt

    async def do_capture(self):
        """ Capture an image and store relevant data. """
        try:
            drone = self.drones.items()[0]
            cur_gps = drone.position_global
            cur_drone_att = drone.attitude
            gimbal = self.dm.gimbal[self.drone_name]
            cur_gimbal_att = np.asarray([gimbal.roll, gimbal.pitch, gimbal.yaw, gimbal.yaw_absolute])
            weather_data = await self.weather_sensor.get_data()

            # TODO: Do a capture info for each image captured, i.e. check image capture info somehow
            # TODO: Maybe create a callback that is active for some time after a do_capture call?
            self.captures.append(ENGELCaptureInfo(weather_data=weather_data))
        except Exception as e:
            self.logger.warning("Exception in the capturing function!")
            self.logger.debug(repr(e), exc_info=True)


    async def save_to_file(self):
        """ Save all capture information to a file, images will have to be downloaded separately anyway. """
        pass
        # TODO: All of it

    async def reset(self):
        """ Land back at launch points"""
        for drone in self.drones:
            await self.dm.fly_to(drone, waypoint=self.launch_point)
            await self.dm.land(drone)
            await asyncio.sleep(0.5)
            await self.dm.disarm(drone)

    async def status(self):
        """ Print information, such as how many captures we have taken"""
        self.logger.info(f"Drones {self.drones}. {len(self.captures)} captures")

    async def add_drones(self, names: list[str]):
        """ Adds camera and gimbal objects and stores current position for rtl"""
        if len(names) + len(self.drones) > 1:
            self.logger.warning("This mission only supports single drones!")
            return False
        self.logger.info(f"Adding drone {names} to ENGEL!")
        for name in names:
            try:
                gimbal_ok = await self.dm.gimbal.add_gimbals(name)
                cam_ok = await self.dm.camera.add_camera(name)
                rtl_pos = self.dm.drones[name].position_global
                cur_yaw = self.dm.drones[name].attitude[2]
                if gimbal_ok and cam_ok:
                    self.drones[name] = self.dm.drones[name]
                    self.drone_name = name
                    rtl_pos[2] += self.rtl_height
                    self.launch_point = Waypoint(WayPointType.POS_GLOBAL, gps=rtl_pos, yaw=cur_yaw)
                    self.logger.info(f"Added drone {name} to mission!")
                    return True
                else:
                    self.logger.info(f"Couldn't add {name} to mission: Gimbal OK {gimbal_ok}, Cam OK {cam_ok}")
                    return False
            except KeyError:
                self.logger.error(f"No drone named {name}")
        return False

    async def remove_drones(self, names: list[str]):
        """ Removes camera and gimbal objects """
        for name in names:
            try:
                self.drones.pop(name)
                self.launch_point = None
                self.drone_name = None
                await self.dm.gimbal.remove_gimbal(name)
                await self.dm.camera.remove_camera(name)
            except KeyError:
                self.logger.error(f"No drone named {name}")

    async def mission_ready(self, drone: str):
        return drone in self.drones

""" Mission for ENGEL data collection
Capture images and combine with weather and position info from drone, storing them
Functions to retake the same position as in a previous image and take another image.
"""
import asyncio
import numpy as np
import json
import datetime
import os

import dronecontrol.plugins.camera
from dronecontrol.navigation.core import Waypoint, WayPointType
from dronecontrol.plugins.mission import Mission
from dronecontrol.sensors.ecowitt import WeatherData
from dronecontrol.utils import LOG_DIR


class ENGELCaptureInfo:

    def __init__(self, time_utc, lat, long, amsl, drone_att, gimbal_att, gimbal_absolute, cam_file, weather_data: WeatherData):
        self.timestamp = time_utc
        self.lat = lat
        self.long = long
        self.amsl = amsl
        self.drone_att = drone_att
        self.gimbal_att = gimbal_att
        self.gimbal_yaw_absolute = gimbal_absolute
        self.file_on_cam = cam_file
        self.weather_data = weather_data

    def to_json_dict(self):
        # TODO: All of it
        pass

    def from_json_dict(self, json_dict):
        # TODO: All of it
        pass


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
            "configure", self.configure_cam,
        }
        self.cli_commands.update(mission_cli_commands)
        self.weather_sensor = None
        self.launch_point: Waypoint | None = None  # A dictionary with latitude and longitude and amsl values
        self.rtl_height = 10  # Height above launch point for return
        self.captures: list[ENGELCaptureInfo] = []
        self.background_functions = [
        ]
        self.drone_name = None

        self.capturing = False  # Set to True while a capture is in process to only allow a single capture at a time
        self.max_capture_duration = 2  # Time in seconds after capture command that we listen for capture info messages.

    async def connect(self):
        """ Connect to the Leitstand sensor"""
        connected = await self.dm.ecowitt.connect("192.168.1.41")
        if connected:
            self.weather_sensor = self.dm.ecowitt

    async def configure_cam(self):
        """ Set parameters for our camera (workswell wiris enterprise)"""
        # TODO: All of it
        pass

    async def do_capture(self):
        """ Capture an image and store relevant data. """
        try:
            if self.capturing:
                self.logger.warning("Already doing a capture, skipping")
                return False


            gimbal = self.dm.gimbal.gimbals[self.drone_name]
            camera = self.dm.camera.cameras[self.drone_name]

            # Send capture command
            self.capturing = True
            res = await camera.take_picture()

            # If denied: log, return False
            if not res:
                self.logger.warning("Engel capture failed as take photo command was denied")
                self.capturing = False
                return False
            # If accepted: Collect metadata, listen for capture_info messages for CAMERA_IMAGE_CAPTURED using callback on mav_conn
            else:
                # TODO:
                # Check CAMERA_IMAGE_CAPTURED messages for capture_result and save info if success:
                # time_utc, microseconds since epoch
                # lat, degree with 7 figures after decimal
                # lon, degree with 7 figures after decimal
                # alt, in mm
                # file_url, str
                # metadata from drone
                drone = self.drones.items()[0]
                cur_drone_att = drone.attitude
                cur_gimbal_att = np.asarray([gimbal.roll, gimbal.pitch, gimbal.yaw])
                weather_data = await self.weather_sensor.get_data()


            self.captures.append(ENGELCaptureInfo(drone_att=cur_drone_att, gimbal_att=cur_gimbal_att,
                                                  gimbal_absolute=gimbal.yaw_absolute, weather_data=weather_data))
            self.capturing = False
            return True
        except Exception as e:
            self.logger.warning("Exception in the capturing function!")
            self.logger.debug(repr(e), exc_info=True)
            return False


    async def save_to_file(self):
        """ Save all capture information to a file, images will have to be downloaded separately anyway. """
        timestamp = datetime.datetime.now(datetime.UTC)
        capture_info_file_name = f"engel_captures_{timestamp.hour}{timestamp.minute}{timestamp.second}-{timestamp.day}-{timestamp.month}-{timestamp.year}"
        capture_file_path = os.path.join(LOG_DIR, capture_info_file_name)
        with (open(capture_file_path, "wt") as f):
            output = [capture.to_json_dict() for capture in self.captures]
            json.dump(output, f)

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

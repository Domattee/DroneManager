""" Mission for ENGEL data collection
Capture images and combine with weather and position info from drone, storing them
Functions to retake the same position as in a previous image and take another image.
"""
import asyncio
import numpy as np
import json
import datetime
import os

from dronecontrol.navigation.core import Waypoint, WayPointType
from dronecontrol.plugins.mission import Mission
from dronecontrol.sensors.ecowitt import WeatherData
from dronecontrol.utils import LOG_DIR


class ENGELCaptureInfo:

    def __init__(self, time_utc, lat, long, amsl, drone_att, gimbal_att, gimbal_absolute, cam_file, weather_data: WeatherData):
        self.time = time_utc
        self.lat = lat
        self.long = long
        self.amsl = amsl
        self.drone_att = drone_att
        self.gimbal_att = gimbal_att
        self.gimbal_yaw_absolute = gimbal_absolute
        self.file_on_camera = cam_file
        self.weather_data = weather_data

    def to_json_dict(self):
        out_dict = self.__dict__.copy()
        out_dict["time"] = self.time.isoformat()
        out_dict["drone_att"] = self.drone_att.tolist()
        out_dict["gimbal_att"] = self.gimbal_att.tolist()
        out_dict["weather_data"] = self.weather_data.to_json_dict()
        return out_dict

    @classmethod
    def from_json_dict(cls, json_dict):
        time_utc = datetime.datetime.fromisoformat(json_dict["time"])
        drone_att = np.asarray(json_dict["drone_att"])
        gimbal_att = np.asarray(json_dict["gimbal_att"])
        weather_data = WeatherData.from_json_dict(json_dict["weather_data"])
        return cls(time_utc=time_utc, lat=json_dict["lat"], long=json_dict["long"], amsl=json_dict["amsl"],
                   drone_att=drone_att, gimbal_att=gimbal_att, gimbal_absolute=json_dict["gimbal_yaw_absolute"],
                   cam_file=json_dict["file_on_camera"], weather_data=weather_data)


class ENGELDataMission(Mission):
    """ Data collection mission for ENGEL

    """

    DEPENDENCIES = ["gimbal", "camera", "sensor.ecowitt"]

    def __init__(self, dm, logger, name="engel"):
        super().__init__(dm, logger, name)
        mission_cli_commands = {
            "connect", self.connect,
            "capture", self.do_capture,
            "save", self.save_captures_to_file,
            "load", self.load_captures_from_file,
            "configure", self.configure_cam,
            "replay", self.replay_captures,
        }
        self.cli_commands.update(mission_cli_commands)
        self.weather_sensor = None
        self.launch_point: Waypoint | None = None  # A dictionary with latitude and longitude and amsl values
        self.rtl_height = 10  # Height above launch point for return
        self.background_functions = [
        ]
        self.drone_name = None

        # Information on previous and current captures
        self.capturing = False  # Set to True while a capture is in process to only allow a single capture at a time
        self.max_capture_duration = 3  # Time in seconds after capture command that we listen for capture info messages.
        self.captures: list[ENGELCaptureInfo] = []  # Images taken this session
        self.loaded_captures: list[ENGELCaptureInfo] = []  # Images taken during a previous session, intended to be replayed

        # TODO: Postprocessing to load all the camera images and store them with EngelCaptureInfo somehow

    async def connect(self):
        """ Connect to the Leitstand sensor"""
        connected = await self.dm.ecowitt.connect("192.168.1.41")
        if connected:
            self.weather_sensor = self.dm.ecowitt

    async def configure_cam(self):
        """ Set parameters for our camera (workswell wiris enterprise)"""
        # TODO: All of it
        # Parameters to set:
        # Zoom, which images, thermal range maybe?
        pass

    async def _imaged_captured_callback(self, msg):
        """ Check CAMERA_IMAGE_CAPTURED messages for capture_result and save info if success, log failure otherwise

        This message contains this info:
        time_utc, milliseconds since epoch or boot (unfortunately boot for our camera). Used
        lat, latitude in degrees as integer with 7 figures after decimal
        lon, longitude in degrees as integer with 7 figures after decimal
        alt, amsl in mm
        file_url, str

        :param msg:
        :return:
        """
        if msg.capture_result == 1:
            if msg.time_utc < 1e12:  # Assume this is reporting time since boot if too small
                time_stamp = datetime.datetime.now(datetime.UTC)  # TODO: Maybe add timesync or something to estimate message delay?
            else:
                time_stamp = datetime.datetime.fromtimestamp(msg.time_utc / 1e3, datetime.UTC)
            lat = msg.lat / 1e7
            long = msg.lon / 1e7
            amsl = msg.alt / 1e3
            file_url = msg.file_url
            drone = self.drones.items()[0]
            gimbal = self.dm.gimbal.gimbals[self.drone_name]
            cur_drone_att = drone.attitude
            cur_gimbal_att = np.asarray([gimbal.roll, gimbal.pitch, gimbal.yaw])
            weather_data = await self.weather_sensor.last_data

            self.logger.debug("Captured image, saving info...")
            self.captures.append(ENGELCaptureInfo(time_utc=time_stamp, lat=lat, long=long, amsl=amsl, drone_att=cur_drone_att, gimbal_att=cur_gimbal_att, gimbal_absolute=gimbal.yaw_absolute, cam_file=file_url, weather_data=weather_data))
        else:
            self.logger.warning("Camera reports failure to capture image!")
            self.logger.debug(msg.to_dict())

    async def do_capture(self):
        """ Capture an image and store relevant data. """
        # TODO: Store current camera settings as well
        try:
            if self.capturing:
                self.logger.warning("Already doing a capture, skipping")
                return False

            camera = self.dm.camera.cameras[self.drone_name]

            # Make sure weather sensor has grabbed latest data
            await self.weather_sensor.get_data()

            # Send capture command
            self.capturing = True
            res = await camera.take_picture()

            # If command denied: log, return False
            if not res:
                self.logger.warning("Engel capture failed as take photo command was denied")
                self.capturing = False
                return False
            # If accepted: Collect metadata, listen for capture_info messages for CAMERA_IMAGE_CAPTURED using callback on mav_conn
            else:
                # Add callback, wait capture duration, remove callback
                # TODO: We should know how many images the camera will take after the configure call, maybe just wait for all of those.
                # Alternatively, use the known number to check if we missed a message or if some image didn't capture
                mav_conn = self.dm.drones[self.drone_name].mav_conn
                mav_conn.add_drone_message_callback(263, self._imaged_captured_callback)
                await asyncio.sleep(self.max_capture_duration)
                mav_conn.remove_drone_message_callback(263, self._imaged_captured_callback)
                self.capturing = False
                return True
        except Exception as e:
            self.logger.warning("Exception in the capturing function!")
            self.logger.debug(repr(e), exc_info=True)
            self.capturing = False
            return False

    async def replay_captures(self):
        """ Function to take the position from previous captures saved to file and capture them all again."""
        # TODO: All of it
        # For each loaded capture: Fly to position, optionally refine position, take new capture
        # Currently just prints loaded info for debug purposes
        for capture in self.loaded_captures:
            self.logger.info(capture.to_json_dict())

    async def save_captures_to_file(self):
        """ Save all capture information to a file, images will have to be downloaded separately anyway. """
        timestamp = datetime.datetime.now(datetime.UTC)
        capture_info_file_name = f"engel_captures_{timestamp.hour}{timestamp.minute}{timestamp.second}-{timestamp.day}-{timestamp.month}-{timestamp.year}.json"
        capture_file_path = os.path.join(LOG_DIR, capture_info_file_name)
        with open(capture_file_path, "wt") as f:
            output = [capture.to_json_dict() for capture in self.captures]
            json.dump(output, f)

    async def load_captures_from_file(self, filename: str):
        """ Load capture information from a file for the purpose of replaying it. """
        file_path = os.path.join(LOG_DIR, filename)
        with open(file_path, "rt") as f:
            json_list = json.load(f)
            captures = [ENGELCaptureInfo.from_json_dict(capture_dict) for capture_dict in json_list]
            self.loaded_captures = captures

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

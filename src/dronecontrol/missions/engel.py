""" Mission for ENGEL data collection
Capture images and combine with weather and position info from drone, storing them
Functions to retake the same position as in a previous image and take another image.
"""
import asyncio
import pathlib

import numpy as np
import json
import datetime
import time
import os
import shutil

from dronecontrol.navigation.core import Waypoint, WayPointType
from dronecontrol.plugins.mission import Mission
from dronecontrol.sensors.ecowitt import WeatherData
from dronecontrol.plugins.camera import CameraParameter, Camera
from dronecontrol.plugins.gimbal import Gimbal
from dronecontrol.utils import LOG_DIR


CAPTURE_DIR = os.path.join(LOG_DIR, "engel_data_captures")
os.makedirs(CAPTURE_DIR, exist_ok=True)


class EngelImageInfo:

    def __init__(self, time_utc, gps: np.ndarray, drone_att: np.ndarray, gimbal_att: np.ndarray, gimbal_absolute, cam_file: str):
        self.time = time_utc
        self.gps = gps
        self.drone_att = drone_att
        self.gimbal_att = gimbal_att
        self.gimbal_yaw_absolute = gimbal_absolute
        self.file_location = cam_file

    def to_json_dict(self):
        return {
            "time": self.time.isoformat(),
            "gps": self.gps.tolist(),
            "drone_att": self.drone_att.tolist(),
            "gimbal_att": self.gimbal_att.tolist(),
            "gimbal_yaw_absolute": self.gimbal_yaw_absolute,
            "file_location": self.file_location,
        }

    @classmethod
    def from_json_dict(cls, json_dict):
        img_time = datetime.datetime.fromisoformat(json_dict["time"])
        gps = np.asarray(json_dict["gps"])
        drone_att = np.asarray(json_dict["drone_att"])
        gimbal_att = np.asarray(json_dict["gimbal_att"])
        gimbal_yaw = json_dict["gimbal_yaw_absolute"]
        cam_file = json_dict["file_location"]
        return cls(img_time, gps, drone_att, gimbal_att, gimbal_yaw, cam_file)

class ENGELCaptureInfo:

    def __init__(self, images: list[EngelImageInfo], weather_data: WeatherData, camera_parameters: list[CameraParameter]):
        self.images = images
        self.weather_data = weather_data
        self.camera_parameters = camera_parameters
        self.capture_id = time.time_ns()
        self.reference_id = None  # Base reference image for replay captures, None if not replay capture

    def to_json_dict(self):
        out_dict = {
            "images": [image.to_json_dict() for image in self.images],
            "weather_data": self.weather_data.to_json_dict(),
            "camera_parameters": [param.to_json_dict() for param in self.camera_parameters],
            "capture_id": self.capture_id,
            "reference_id": self.reference_id,
        }
        return out_dict

    @classmethod
    def from_json_dict(cls, json_dict):
        images = [EngelImageInfo.from_json_dict(image_dict) for image_dict in json_dict["images"]]
        weather_data = WeatherData.from_json_dict(json_dict["weather_data"])
        cam_params = [CameraParameter.from_json_dict(entry) for entry in json_dict["camera_parameters"]]
        out = cls(images, weather_data=weather_data, camera_parameters=cam_params)
        out.capture_id = json_dict["capture_id"]
        out.reference_id = json_dict["reference_id"]
        return out


class ENGELDataMission(Mission):
    """ Data collection mission for ENGEL

    """

    DEPENDENCIES = ["gimbal", "camera", "sensor.ecowitt"]

    def __init__(self, dm, logger, name="engel"):
        super().__init__(dm, logger, name)
        mission_cli_commands = {
            "connect": self.connect,
            "capture": self.do_capture,
            "save": self.save_captures_to_file,
            "load": self.load_captures_from_file,
            "configure": self.configure_cam,
            "replay": self.replay_captures,
            "transfer": self.transfer,
        }
        self.cli_commands.update(mission_cli_commands)
        self.weather_sensor = None
        self.launch_point: Waypoint | None = None  # A dictionary with latitude and longitude and amsl values
        self.rtl_height = 10  # Height above launch point for return
        self.background_functions = [
        ]
        self.drone_name = None
        self.gimbal: Gimbal | None = None
        self.camera: Camera | None = None

        # Information on previous and current captures
        self.capturing = False  # Set to True while a capture is in process to only allow a single capture at a time
        self.max_capture_duration = 5  # Time in seconds after capture command that we listen for capture info messages.
        self.captures: list[ENGELCaptureInfo] = []  # Images taken this session
        self.loaded_captures: list[ENGELCaptureInfo] = []  # Images taken during a previous session, intended to be replayed
        self.loaded_file: str | None = None
        self._current_capture: ENGELCaptureInfo | None = None

    async def connect(self):
        """ Connect to the Leitstand sensor"""
        connected = await self.dm.ecowitt.connect("192.168.1.41")
        if connected:
            self.weather_sensor = self.dm.ecowitt

    async def configure_cam(self):
        """ Set parameters for our camera (Workswell WIRIS enterprise), won't work with others"""
        # Standard Parameter Set:
        # IMG_RAD_TIFF      1
        # IMG_RAD_JPEG      0
        # IMG_IR_SUPER      "Off"
        # IMG_SCREEN        0
        # IMG_VIS           1
        # IMG_VHR           1
        # RANGE_TYPE        "Manual"
        # RANGE_MAX         40.0
        # RANGE_MIN         10.0
        # MAIN_CAM          "Visible"
        # ZOOM_THERMO_I     1.0
        # ZOOM_VISIBLE_I    1.0
        self.logger.info("Setting camera parameters to default...")
        await self.camera.set_parameter("IMG_RAD_TIFF", True)
        await self.camera.set_parameter("IMG_RAD_JPEG", False)
        await self.camera.set_parameter("IMG_IR_SUPER", self.camera.parse_param_value("IMG_IR_SUPER", "Off"))
        await self.camera.set_parameter("IMG_SCREEN", False)
        await self.camera.set_parameter("IMG_VIS", True)
        await self.camera.set_parameter("IMG_VHR", True)
        await self.camera.set_parameter("RANGE_TYPE", self.camera.parse_param_value("RANGE_TYPE", "Manual"))
        await self.camera.set_parameter("RANGE_MAX", 40.0)
        await self.camera.set_parameter("RANGE_MIN", 10.0)
        await self.camera.set_parameter("MAIN_CAM", self.camera.parse_param_value("MAIN_CAM", "Visible"))
        await self.camera.set_parameter("ZOOM_THERMO_I", self.camera.parse_param_value("ZOOM_THERMO_I", "1.0"))
        await self.camera.set_parameter("ZOOM_VISIBLE_I", self.camera.parse_param_value("ZOOM_VISIBLE_I", "1.0"))

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
                time_stamp = datetime.datetime.now(datetime.UTC)
            else:
                time_stamp = datetime.datetime.fromtimestamp(msg.time_utc / 1e3, datetime.UTC)
            gps = np.asarray([msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1e3])
            file_url = msg.file_url
            drone = self.drones[self.drone_name]
            cur_drone_att = drone.attitude
            cur_gimbal_att = np.asarray([self.gimbal.roll, self.gimbal.pitch, self.gimbal.yaw])
            if file_url in [self._current_capture.images[i].file_location for i in range(len(self._current_capture.images))]:
                self.logger.debug("Camera saved over image it just took")
            else:
                self.logger.debug("Captured image, saving info...")
                self._current_capture.images.append(EngelImageInfo(time_stamp, gps, cur_drone_att,
                                                                   cur_gimbal_att, self.gimbal.yaw_absolute, file_url))
        else:
            self.logger.warning("Camera reports failure to capture image!")
            self.logger.debug(msg.to_dict())

    async def do_capture(self, reference_capture: ENGELCaptureInfo | None = None):
        """ Capture an image and store relevant data. """
        try:
            if self.capturing:
                self.logger.warning("Already doing a capture, skipping")
                return False

            # Make sure weather sensor has grabbed latest data
            if self.weather_sensor:
                weather_data = await self.weather_sensor.get_data()
            else:
                self.logger.warning(f"No Weather sensor, using dummy data!")
                weather_data = WeatherData()

            cam_params = list(self.camera.parameters.values())

            # Send capture command
            self.capturing = True
            capture = ENGELCaptureInfo([], weather_data, cam_params)
            # If we got a reference capture this is a replay capture, and we add the old id to this one
            if reference_capture is not None:
                capture.reference_id = reference_capture.capture_id

            self._current_capture = capture
            res = await self.camera.take_picture()

            # If command denied: log, return False
            if not res:
                self.logger.warning("Engel capture failed as take photo command was denied")
                self.capturing = False
                self._current_capture = None
                return False
            # If accepted: Collect metadata, listen for capture_info messages for CAMERA_IMAGE_CAPTURED using callback on mav_conn
            else:
                # Add callback, wait capture duration, remove callback
                # TODO: We should know how many images the camera will take after the configure call, maybe just wait for all of those.
                # TODO: Request images that didn't arrive using image index
                # TODO: Directly associate images with the corresponding reference image somehow, instead of the larger "capture"
                mav_conn = self.dm.drones[self.drone_name].mav_conn
                mav_conn.add_drone_message_callback(263, self._imaged_captured_callback)
                await asyncio.sleep(self.max_capture_duration)
                mav_conn.remove_drone_message_callback(263, self._imaged_captured_callback)
                self.capturing = False
                self._current_capture = None
                if len(capture.images) > 0:
                    self.logger.info(f"Captured {len(capture.images)} images!")
                    self.captures.append(capture)
                else:
                    self.logger.warning(f"No images captured! (Maybe capture duration is too short?)")
                return True
        except Exception as e:
            self.logger.warning("Exception in the capturing function!")
            self.logger.debug(repr(e), exc_info=True)
            self.capturing = False
            self._current_capture = None
            return False

    async def set_camera_parameters(self, params: list[CameraParameter]):
        # Go through a list of camera parameters and adjust the connected camera parameters to match
        for parameter in params:
            if self.camera.parameters[parameter.name].value != parameter.value:
                await self.camera.set_parameter(parameter.name, parameter.value)

    async def replay_captures(self):
        """ Function to take the position from previous captures saved to file and capture them all again."""
        # For each loaded capture: Set camera parameters, fly to position, optionally refine position, take new capture
        # Currently just prints loaded info for debug purposes
        for capture in self.loaded_captures:
            try:
                reference_image = capture.images[0]
                # Use "visible" as reference image for now. TODO: Figure out if this is best, might have to do screenshots instead if comparison happens against live feed
                for image in capture.images:
                    if "visible" in image.file_location:
                        reference_image = image

                # Set camera parameters
                cam_set_task = asyncio.create_task(self.set_camera_parameters(capture.camera_parameters))
                self._running_tasks.add(cam_set_task)
                # Fly to position and point gimbal
                # TODO: Have to correct gimbal attitude not just for drone pitch but also roll, depending on relative angle between gimbal yaw and drone yaw
                if self.drones[self.drone_name].is_armed and self.drones[self.drone_name].in_air:
                    # Fly to position
                    await self.dm.fly_to(self.drone_name, gps=reference_image.gps, yaw=reference_image.drone_att[2])
                # Point gimbal
                target_g_pitch = reference_image.gimbal_att[1] + reference_image.drone_att[1] - self.dm.drones[self.drone_name].attitude[1]
                target_g_yaw = reference_image.gimbal_yaw_absolute
                await self.gimbal.set_gimbal_mode("lock")
                await self.gimbal.set_gimbal_angles(target_g_pitch, target_g_yaw)
                while abs(self.gimbal.pitch - target_g_pitch) < 0.25 and abs(self.gimbal.yaw_absolute - target_g_yaw) < 0.25:
                    await asyncio.sleep(0.1)
                    target_g_pitch = reference_image.gimbal_att[1] + reference_image.drone_att[1] - self.dm.drones[self.drone_name].attitude[1]  # Recompute pitch target for possible drone change in pitch
                    # This doesn't reliably work for some reason. TODO: Figure out why
                # Refine position and gimbal attitude based on previous image
                # TODO: Integrate from other repo, more eval on simulation first
                # New capture
                await cam_set_task
                # Wait at least 3 seconds to make sure that gimbal is in position
                await asyncio.sleep(3)
                await self.do_capture(capture)
                # TODO: Check for replays that didn't work
            except Exception as e:
                self.logger.warning(f"Exception with replay for capture {capture.capture_id}")
                self.logger.debug(repr(e), exc_info=True)

    async def transfer(self, drive_letter: str):
        """ Load images from camera and do assorted metadata processing.

        Loads images from camera and stores them in a folder named after their capture ID. The capture information file
        is also rewritten to account for this. This is intended to be done after flights with the camera directly
        attached to the computer.

        :param drive_letter: Drive letter of the camera
        :return:
        """
        for capture in self.loaded_captures:
            # Create directory in capture folder
            img_dir = os.path.join(CAPTURE_DIR, str(capture.capture_id))
            os.makedirs(img_dir, exist_ok=True)
            for image in capture.images:
                cam_path = image.file_location
                if cam_path.startswith("/mnt/ssd/"):
                    cam_file_dir = pathlib.Path(cam_path[9:])
                    image_file_name = cam_file_dir.name
                    mounted_path = pathlib.Path(f"{drive_letter}:").resolve().joinpath(cam_file_dir)
                    if mounted_path.exists():
                        # Move images from camera to capture folder
                        local_img_path = pathlib.Path(img_dir).joinpath(image_file_name)
                        shutil.move(mounted_path, local_img_path)
                        # Change directory in capture
                        image.file_location = str(local_img_path)
                    else:
                        self.logger.warning(f"File {mounted_path} on camera doesn't exist!")
        await self._save_captures_to_file(self.loaded_captures, filename=self.loaded_file)

    async def _save_captures_to_file(self, captures, filename: str = None):
        """ Save all capture information to a file, images will have to be downloaded separately anyway. """
        if filename is None:
            timestamp = datetime.datetime.now(datetime.UTC)
            capture_info_file_name = f"engel_captures_{timestamp.hour}{timestamp.minute}{timestamp.second}-{timestamp.day}-{timestamp.month}-{timestamp.year}.json"
        else:
            capture_info_file_name = filename
        capture_file_path = os.path.join(CAPTURE_DIR, capture_info_file_name)
        self.logger.info(f"Saving info to file {capture_file_path}")
        with open(capture_file_path, "wt") as f:
            output = [capture.to_json_dict() for capture in captures]
            json.dump(output, f)

    async def save_captures_to_file(self, filename: str = None):
        return await self._save_captures_to_file(self.captures, filename)

    async def load_captures_from_file(self, filename: str):
        """ Load capture information from a file for the purpose of replaying it. """
        file_path = os.path.join(CAPTURE_DIR, filename)
        with open(file_path, "rt") as f:
            json_list = json.load(f)
            captures = [ENGELCaptureInfo.from_json_dict(capture_dict) for capture_dict in json_list]
            self.loaded_captures = captures
            self.loaded_file = filename
            self.logger.info(f"Loaded past captures from file {file_path}")

    async def reset(self):
        """ Clear capture info """
        # Resets variables as if the mission was just loaded. Useful for replay testing.
        self.captures = []
        self.loaded_captures = []
        self.loaded_file = None

    async def status(self):
        """ Print information, such as how many captures we have taken"""
        self.logger.info(f"Drones {self.drones}. {len(self.captures)} current, {len(self.loaded_captures)} old captures.")

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
                    self.gimbal = self.dm.gimbal.gimbals[name]
                    self.camera = self.dm.camera.cameras[name]
                    rtl_pos[2] += self.rtl_height
                    self.launch_point = Waypoint(WayPointType.POS_GLOBAL, gps=rtl_pos, yaw=cur_yaw)
                    await self.gimbal.take_control()
                    await self.gimbal.set_gimbal_mode("lock")
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
                self.gimbal = None
                self.camera = None
                await self.dm.gimbal.remove_gimbal(name)
                await self.dm.camera.remove_camera(name)
            except KeyError:
                self.logger.error(f"No drone named {name}")

    async def mission_ready(self, drone: str):
        return drone in self.drones

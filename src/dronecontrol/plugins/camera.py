""" Plugin for controlling MAVSDK Cameras """
import asyncio
import os

import requests
from lxml import etree

import mavsdk.camera

from dronecontrol.plugin import Plugin
from dronecontrol.utils import CACHE_DIR

# TODO: Overlooked the mavsdk param plugin, check if that makes the camera plugin work properly


class CameraPlugin(Plugin):
    PREFIX = "cam"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "add": self.add_cameras,
            "remove": self.remove_camera,
            "status": self.status,
            "settings": self.parameters,
            "set-setting": self.set_parameter,
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

    async def add_cameras(self, drone: str, camera_id: int = 100):
        """ Add cameras from/for a given drone to the plugin"""
        self.logger.info(f"Adding camera to drone {drone}")
        try:
            drone_object = self.dm.drones.get(drone, None)
            if drone_object:
                new_cam = Camera(self.logger, self.dm, drone_object, camera_id=camera_id)
                success = await new_cam.start()
                if success:
                    self.cameras[drone] = new_cam
                else:
                    await new_cam.close()
            else:
                self.logger.warning(f"No drone named {drone}")
        except Exception as e:
            self.logger.warning("Couldn't add the camera to the drone due to an exception!")
            self.logger.debug(repr(e), exc_info=True)

    async def remove_camera(self, drone: str):
        """ Remove a camera from the plugin"""
        self.logger.info(f"Removing camera from drone {drone}")
        camera = self.cameras.pop(drone)
        await camera.close()
        del camera

    async def status(self, drone: str):
        if self.check_has_camera(drone):
            self.cameras[drone].log_status()

    async def parameters(self, drone: str):
        if self.check_has_camera(drone):
            self.logger.info(f"Showing parameters for camera {self.cameras[drone].camera_id} on {drone}")
            return await self.cameras[drone].print_parameters()
        return False

    async def set_parameter(self, drone: str, param_name: str, param_value: str):
        if self.check_has_camera(drone):
            await self.cameras[drone].set_parameter(param_name, param_value)

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


class ParameterOption:

    def __init__(self, name, value, excludes):
        self.name = name  # For display
        self.value = value  # This is used internally
        self.excludes = excludes  # These parameters are rendered irrelevant if this option is set


class CameraParameter:

    def __init__(self, name, param_type, default, control: bool, description: str, updates: list[str],
                 options: list[ParameterOption], min_value: float | None, max_value: float | None,
                 step_size: float | None):
        self.name = name  # Name of the parameter as str, params will be referred to with this
        self.value = default  # Value of the parameter
        self.param_type = param_type  # Mavlink parameter type
        self.default = default  # Default value
        self.control = control
        self.description = description  # Human readable description

        # A list of other parameters that might get updated when this parameter is changed. Changing this parameter
        # should also reqest updates for these parameters.
        self.updates = updates

        self._options: dict[int, ParameterOption] = {option.value: option for option in options}
        self._options_by_name: dict[str, ParameterOption] = {option.name: option for option in options}

        # Indicates a range of possible options, equivalent to a series of options with name = value and no excludes
        self.min = min_value
        self.max = max_value
        self.step_size = step_size

    @property
    def is_range(self):
        return self.min is not None and self.max is not None

    @property
    def is_bool(self):
        return self.param_type == "bool"

    @property
    def is_option(self):
        return bool(self._options)

    def check_option_valid(self, value):
        if self.is_bool:
            return isinstance(value, bool)
        if self.is_range:
            if self.step_size is None:
                return self.min < value < self.max
            else:
                n_steps = (self.max - self.min) // self.step_size
                return value in [self.min + self.step_size * i for i in range(int(n_steps))]
        else:
            return value in self._options

    def get_option_by_name(self, option_name):
        return self._options_by_name.get(option_name, None)

    def get_option_by_value(self, option_value):
        return self._options.get(option_value)

    def get_options(self):
        return list(self._options.values())


class Camera:

    def __init__(self, logger, dm, drone, camera_id: int = 100):
        self.logger = logger
        self.dm = dm
        self.drone = drone

        self.camera_id = camera_id
        self._running_tasks = set()
        self._start_background_tasks()

        self.vendor_name = None
        self.model_name = None
        self.parameters: dict[str, CameraParameter] = {}

    def _start_background_tasks(self):
        self._running_tasks.add(asyncio.create_task(self._capture_info_updates()))

    async def _capture_info_updates(self):
        async for capture_info in self.drone.system.camera.capture_info():
            self.logger.info(f"Capture update: Camera {capture_info.component_id} "
                             f"{'succeeded' if capture_info.is_success else 'failed'} with photo at "
                             f"{capture_info.time_utc_us}: {capture_info.file_url}")

    async def start(self):
        have_cam = await self.init_cam_info()
        if have_cam:
            self._get_cam_params()
            return True
        return False

    async def close(self):
        self.drone.mav_conn.remove_drone_message_callback(322, self._listen_param_updates)
        for task in self._running_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()

    @property
    def params_loaded(self):
        return len(self.parameters) != 0

    def log_status(self):
        camera_id = self.camera_id
        self.logger.info(f"Camera {camera_id}, parameters {'not ' if self.params_loaded else ''}loaded")

    async def take_picture(self,):
        res = await self.drone.mav_conn.send_cmd_long(target_component=100, cmd=2000, param3=1.0)
        if not res:
            self.logger.warning("Couldn't take picture!")

    async def start_video(self):
        res = await self.drone.mav_conn.send_cmd_long(target_component=self.camera_id, cmd=2500, param2=1)
        if not res:
            self.logger.warning("Couldn't start video!")

    async def stop_video(self):
        res = await self.drone.mav_conn.send_cmd_long(target_component=self.camera_id, cmd=2501)
        if not res:
            self.logger.warning("Couldn't stop video!")

    async def set_zoom(self, zoom):
        res = await self.drone.mav_conn.send_cmd_long(target_component=self.camera_id, cmd=203, param2=zoom, param5=0)
        if not res:
            self.logger.warning("Couldn't set the zoom!")

    async def _error_wrapper(self, func, *args, **kwargs):
        try:
            res = await func(*args, **kwargs)
        except mavsdk.camera.CameraError as e:
            self.logger.error(f"CameraError: {e._result.result_str}")
            return False
        return res

    async def init_cam_info(self):
        cam_info = await self.drone.mav_conn.get_message(target_component=self.camera_id, message_id=259)
        if not cam_info:
            self.logger.warning("No camera found!")
        else:
            self.vendor_name = bytearray(cam_info.vendor_name).rstrip(b"\x00").decode("ascii")
            self.model_name = bytearray(cam_info.model_name).rstrip(b"\x00").decode("ascii")
            cam_def_uri = cam_info.cam_definition_uri
            cam_def_version = cam_info.cam_definition_version
            try:
                xml_str = await self._get_cam_definition(cam_def_uri, cam_def_version)
                if xml_str:
                    self.logger.info(xml_str)
                    self._parse_cam_definition(xml_str)
                    return True
            except Exception as e:
                self.logger.warning("Couldn't get the camera definition XML due to an exception!")
                self.logger.debug(repr(e), exc_info=True)
        return False

    async def _get_cam_definition(self, uri, version) -> str | None:
        cached_path = self._xml_cache_filepath(uri, version)
        if cached_path.exists():
            xml_str = await asyncio.get_running_loop().run_in_executor(None, self._load_xml, uri, version)
        else:
            self.logger.debug("Camera definition file not cached, downloading...")
            xml_str = await asyncio.get_running_loop().run_in_executor(None, self._download_xml, uri)
            if xml_str:
                await asyncio.get_running_loop().run_in_executor(None, self._save_xml, xml_str, uri, version)
        return xml_str

    def _download_xml(self, uri):
        try:
            response = requests.get(uri, verify=False)
            return response.text
        except requests.RequestException:
            self.logger.warning("Couldn't download the camera definition, probably due to no internet.")
            return None

    def _save_xml(self, xml_str, uri, version):
        filepath = self._xml_cache_filepath(uri, version)
        self.logger.debug(f"Saving camera definition xml {filepath}")
        os.makedirs(filepath.parent, exist_ok=True)
        with open(filepath, "wt", encoding="utf-8") as f:
            f.write(xml_str)

    def _load_xml(self, uri, version):
        try:
            filepath = self._xml_cache_filepath(uri, version)
            self.logger.debug(f"Loading camera definition xml {filepath}")
            with open(filepath, "rb") as f:
                xml_str = f.read()
            return xml_str
        except OSError as e:
            self.logger.error("Couldn't load the cached camera definition.")
            self.logger.debug(repr(e), exc_info=True)
            return None

    def _xml_cache_filepath(self, uri, version):
        pathsafe_uri = uri.replace("/", "_").replace("\\", "_").replace(":", "_").replace("?", "_")
        filename = f"{version}_{pathsafe_uri}.xml"
        cache_dir = CACHE_DIR.joinpath("camera_definitions")
        return cache_dir.joinpath(filename)

    def _parse_cam_definition(self, cam_def_xml: str):
        root = etree.fromstring(cam_def_xml)
        params = root.findall(".//parameter")
        for param in params:
            name = param.get("name")
            param_type = param.get("type")
            default = param.get("default")
            control = param.get("control") is None
            min_val = param.get("min")
            max_val = param.get("max")
            step_size = param.get("step")
            description = param.find("description").text

            options = []
            options_element = param.find("options")
            if options_element:
                option_name = options_element.get("name")
                option_value = options_element.get("value")
                excludes = []
                exclusions_element = options_element.find("exclusions")
                if exclusions_element:
                    for exclude_element in exclusions_element:
                        excludes.append(exclude_element.text)
                options.append(ParameterOption(option_name, option_value, excludes))

            updates = []
            updates_element = param.find("updates")
            if updates_element:
                for update_element in updates_element:
                    updates.append(update_element.text)

            self.parameters[name] = CameraParameter(name=name, param_type=param_type, default=default, control=control,
                                                    description=description, updates=updates, options=options,
                                                    min_value=min_val, max_value=max_val, step_size=step_size)

    def _get_cam_params(self):
        self.drone.mav_conn.send_param_ext_request_list(self.drone.mav_conn.drone_system, self.camera_id)
        self.drone.mav_conn.add_drone_message_callback(322, self._listen_param_updates)
        # TODO: Check that we have all the params loaded

    async def _listen_param_updates(self, msg):
        if msg.get_srcComponent() == self.camera_id and msg.get_srcSystem() == self.drone.mav_conn.drone_system:
            param_name = msg.param_id
            param_value = msg.param_value
            if param_name in self.parameters:
                self.parameters[param_name].value = param_value

    async def print_parameters(self):
        if len(self.parameters) == 0:
            self.logger.info("No parameters loaded, trying to download...")
            await self.init_cam_info()
        for param_name, parameter in self.parameters.items():
            if parameter.is_option:
                info_string = "".join([option.name for option in parameter.get_options()])
                param_value = parameter.get_option_by_value(parameter.value).name
            elif parameter.is_bool:
                info_string = "True or False"
                param_value = parameter.value
            elif parameter.is_range:
                if parameter.step_size:
                    info_string = f"Range between {parameter.min} and {parameter.max} " \
                                  f"with step size {parameter.step_size}"
                else:
                    info_string = f"Range between {parameter.min} and {parameter.max}"
                param_value = parameter.value
            else:
                info_string = "Invalid option somewhow"
                param_value = "You found an edge case for the parameter type, congratulations!"
            self.logger.info(f"Parameter {param_name}: {param_value}. {parameter.description}\n"
                             f"\tOptions: {info_string}")

    async def set_parameter(self, param_name, param_value: str):
        # Send the change param message and request updates for any params in the updates list

        if param_name not in self.parameters:
            self.logger.warning(f"Don't have a parameter {param_name}!")
            return False

        # Determine the type based on the parameter
        parameter = self.parameters[param_name]

        if parameter.is_option:  # Treat it as a string option name, get value from the option object
            parsed_value = parameter.get_option_by_name(param_value).value

        elif parameter.is_bool:  # Treat it as a string, get appropriate bool value
            if param_value in ["True", "true", "1"]:
                parsed_value = True
            elif param_value in ["False", "false", "0"]:
                parsed_value = False
            else:
                parsed_value = None

        else:  # Try to parse it into a float
            try:
                parsed_value = float(param_value)
            except ValueError:
                parsed_value = None

        if not parameter.check_option_valid(parsed_value):
            self.logger.warning(f"Value {parsed_value} is invalid for {param_name}")
            return False

        self.drone.mav_conn.send_param_ext_set(self.camera_id, param_name, parsed_value, parameter.param_type)
        ack_msg = await self.drone.mav_conn.listen_message(324, self.camera_id)
        if ack_msg.param_result != 0:
            self.logger.warning("Couldn't set parameter!")
            return False

        # Request updates on any params that might have changed.
        for updated_param_name in parameter.updates:
            self.drone.mav_conn.send_param_ext_request_read(self.camera_id, updated_param_name)
        return True

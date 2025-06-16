""" Plugin and ABC for external sensors, such as weather sensors."""
import asyncio
import abc
import importlib
import inspect
from pathlib import Path

from dronecontrol.plugin import Plugin

# TODO: A check which sensors could actually be connected. Check functions must return quick or be executed in
#  another process.


class SensorPlugin(Plugin):
    PREFIX = "sensor"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "load": self.load,
            "status": self.status,
        }
        self.sensors: dict[str, Sensor] = {}

    async def close(self):
        """ Stop all missions.

        :return:
        """
        await asyncio.gather(*[sensor.close() for sensor in list(self.sensors.values())])
        await super().close()

    def sensor_options(self):
        # Go through every file in mission folder
        _base_dir = Path(__file__).parent.parent
        _plugin_dir = _base_dir.joinpath("sensors")
        modules = [name.stem for name in _plugin_dir.iterdir()
                   if name.is_file() and name.suffix == ".py" and not name.stem.startswith("_")]
        return modules

    def _get_mission_class(self, module) -> None | type:
        try:
            plugin_mod = importlib.import_module("." + module, "dronecontrol.missions")
            plugin_classes = [member[1] for member in inspect.getmembers(plugin_mod, inspect.isclass)
                              if issubclass(member[1], Sensor)
                              and not member[1] is Sensor]  # Strict subclass check
            if len(plugin_classes) != 1:
                return None
            return plugin_classes[0]
        except ImportError as e:
            self.logger.error(f"Couldn't load mission {module} due to a python import error!")
            self.logger.debug(repr(e), exc_info=True)

    async def load(self, mission_module: str, name: str | None = None):
        """ Load a new sensor, which work like plugins with the name taking the role of the prefix.

        :return:
        """
        sensor = await self.dm.load_plugin(mission_module, name, self.sensor_options(), self._get_mission_class)
        if sensor:
            self.sensors[name] = sensor
        return sensor

    async def status(self):
        """ Status of running missions and missions that could be loaded."""
        self.logger.info("Status of currently connected sensors:")
        for sensor in self.sensors.values():
            await sensor.status()
        if len(self.sensors) == 0:
            self.logger.info("No connected sensors!")
        self.logger.info(f"Available sensor plugins for loading: {self.sensor_options()}")


class Sensor(Plugin, abc.ABC):
    PREFIX = "YOUDIDSOMETHINGWRONG"

    def __init__(self, dm, logger, name="YOUDIDSOMETHINGWRONG"):
        super().__init__(dm, logger, name)
        self.PREFIX = name
        self.cli_commands = {
            "connect": self.connect,
            "status": self.status,
            "disconnect": self.disconnect,
            "reconnect": self.reconnect,
        }

        # Connection arguments for convenient reconnect.
        self.connect_args = None
        self.connect_kwargs = None

    async def start(self):
        """ This function is called when the sensor plugin is loaded to automatically start any background functions.
        """
        await super().start()

    async def close(self):
        """ This function must end all running asyncio tasks. By default, all tasks in self._running_tasks are
        cancelled."""
        await self.disconnect()
        await super().close()

    @abc.abstractmethod
    async def connect(self, *args, **kwargs):
        """ Connect to a sensor."""
        self.connect_args = args
        self.connect_kwargs = kwargs

    @abc.abstractmethod
    async def status(self):
        """ Should write information about the current status of the sensor to the logger under INFO."""
        pass

    @abc.abstractmethod
    async def disconnect(self):
        """ Disconnect from a sensor. Should handle any socket clearing etc."""
        pass

    async def reconnect(self):
        await self.disconnect()
        await self.connect(*self.connect_args, **self.connect_kwargs)

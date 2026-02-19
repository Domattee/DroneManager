import abc
import asyncio
import collections
import enum
from pathlib import Path
import importlib
import inspect

from dronemanager.plugin import Plugin


class MissionPlugin(Plugin):
    PREFIX = "mission"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "load": self.load,
            "status": self.status,
        }
        self.missions: dict[str, Mission] = {}

    async def close(self):
        """ Stop all missions.

        :return:
        """
        await asyncio.gather(*[mission.close() for mission in list(self.missions.values())])
        await super().close()

    def mission_options(self):
        # Go through every file in mission folder
        _base_dir = Path(__file__).parent.parent
        _plugin_dir = _base_dir.joinpath("missions")
        modules = [name.stem for name in _plugin_dir.iterdir()
                   if name.is_file() and name.suffix == ".py" and not name.stem.startswith("_")]
        return modules

    def _get_mission_class(self, module) -> None | type:
        try:
            plugin_mod = importlib.import_module("." + module, "dronemanager.missions")
            plugin_classes = [member[1] for member in inspect.getmembers(plugin_mod, inspect.isclass)
                              if issubclass(member[1], Mission)
                              and not member[1] is Mission]  # Strict subclass check
            if len(plugin_classes) != 1:
                return None
            return plugin_classes[0]
        except ImportError as e:
            self.logger.error(f"Couldn't load mission {module} due to a python import error!")
            self.logger.debug(repr(e), exc_info=True)

    async def load(self, mission_module: str, name: str | None = None):
        """ Load a new mission, which work like plugins with the name taking the role of the prefix.
        """
        mission = await self.dm.load_plugin(mission_module, name, self.mission_options(), self._get_mission_class)
        if mission:
            self.missions[mission.name] = mission
        return mission

    async def status(self):
        """ Status of running missions and missions that could be loaded."""
        self.logger.info("Status of running missions:")
        for mission in self.missions.values():
            await mission.status()
        if len(self.missions) == 0:
            self.logger.info("No running missions!")
        self.logger.info(f"Available missions for loading: {self.mission_options()}")


class MissionStage(enum.Enum):
    pass


class FlightArea(abc.ABC):
    def __init__(self, *args, **kwargs):
        pass

    @property
    @abc.abstractmethod
    def x_min(self):
        pass

    @property
    @abc.abstractmethod
    def x_max(self):
        pass

    @property
    @abc.abstractmethod
    def y_min(self):
        pass

    @property
    @abc.abstractmethod
    def y_max(self):
        pass

    @property
    @abc.abstractmethod
    def alt_max(self):
        pass

    def boundary_list(self):
        return [self.x_min, self.x_max, self.y_min, self.y_max, self.alt_max]


class Mission(Plugin, abc.ABC):
    PREFIX = "YOUDIDSOMETHINGWRONG"

    def __init__(self, dm, logger, name="YOUDIDSOMETHINGWRONG"):
        super().__init__(dm, logger, name)
        self.PREFIX = name

        # CLI commands for these coroutines are generated automatically based on the signature and type hints.
        # They must be coroutines!
        self.cli_commands = {
            "reset": self.reset,
            "status": self.status,
            "add": self.add_drones,
            "remove": self.remove_drones,
        }

        # These attributes may be used by other part of the software, for example to determine the window size for a map
        self.current_stage: MissionStage | None = None  # For missions with multiple stages
        self.flight_area: FlightArea | None = None  # For missions with a defined flight area
        self.drones = collections.OrderedDict()  # The drones participating in the mission

        # A dictionary with any other information that might be useful for other parts of the library. The external
        # plugin automatically shares the information here.
        self.additional_info = {}

    async def start(self):
        """ This function is called when the mission is loaded to start all the necessary processes asynchronously.

        It is NOT a "start this mission" function. By default, launches any background processes, like starting a
        plugin."""
        await super().start()

    async def close(self):
        """ Shutdown function for the script. It should end any running tasks and clear any resources.

        By default, it cancels any tasks tracked in self._running_tasks."""
        await super().close()

    @abc.abstractmethod
    async def reset(self):
        """ Resets the mission back to the initial position.

        Keep safety in mind when this requires moving drones."""
        raise NotImplementedError

    @abc.abstractmethod
    async def status(self):
        """ Should write information about the current status of the mission to the logger under INFO."""
        raise NotImplementedError

    @abc.abstractmethod
    async def add_drones(self, names: list[str]):
        """ Add drones to the mission. Implementations should check that the drones are capable and meet mission
        requirements."""
        raise NotImplementedError

    @abc.abstractmethod
    async def remove_drones(self, names: list[str]):
        """ Remove drones from the mission. Implementations must take measures to prevent missions from running with
        too few drones"""
        raise NotImplementedError

    @abc.abstractmethod
    async def mission_ready(self, drone: str):
        """ Check whether any given drone is ready to keep going, i.e. is still connected etc. """
        raise NotImplementedError

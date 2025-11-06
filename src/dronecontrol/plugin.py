""" Class for extra, loadable plugins.

Plugins extend the functionality of DroneManager or Drone Classes by providing extra functions. They can also register
their own commands to the CLI.
"""
import asyncio
from abc import ABC


# TODO: Figure out scheduling
#   Have to interact with drone queues ("Move to position X, then turn gimbal, then move to position Y)
#   BUT, also want to perform plugin actions immediately mid flight without killing other drone tasks, (except if we do)
# TODO: Figure out how to do help strings for plugins, choices, store_true, etc... general CLI information.
# TODO: Move a bunch of the plugin handling from DroneManager to here.

class Plugin(ABC):
    """ Generic plugin class.

    The attribute cli_commands is called by the DroneManager CLI (and could be called by other UIs) to populate their
    interfaces. This is a dictionary with coroutines as values and human-readable names as keys. In DroneManager
    the names are used together with the class prefix to determine the command input on the command line, while the
    signature of the function is used to populate the CLI parser.
    The attribute background_functions should list coroutines that will run indefinitely, for example those
    polling for status updates from a camera. They will be started during construction of the class object, usually
    when the module is loaded. Note that these must be coroutines.

    A common kwarg is "name", for plugins of which multiple copies may be loaded, in which case the name acts as the
    unique identifier.

    """

    PREFIX = "abc"
    DEPENDENCIES = []  # Other plugin dependencies, which should be loaded (and are by dronemanager) before this one.
    # Nested dependencies can also be done with a dot notation, i.e. "mission.uam". The load function of the first
    # plugin will be used to load the second, i.e. we would load the plugin "mission" and then use the mission.load to
    # load the subplugin uam.
    # TODO: Proper dependency management

    def __init__(self, dm, logger, name, *args, **kwargs):
        self.dm: "dronecontrol.dronemanager.DroneManager" = dm
        self.logger = logger.getChild(self.__class__.__name__)
        self.name = name
        self.cli_commands = {}
        self.background_functions = []
        self._running_tasks = set()

    def start_background_functions(self):
        for coro in self.background_functions:
            self._running_tasks.add(asyncio.create_task(coro))

    async def start(self):
        """ Starts any background functions."""
        self.start_background_functions()

    async def close(self):
        """ Ends all running tasks functions."""
        for task in self._running_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()

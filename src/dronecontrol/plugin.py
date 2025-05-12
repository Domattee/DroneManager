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

class Plugin(ABC):
    """ Generic plugin class.

    The attribute cli_commands is called by the DroneManager CLI (and could be called by other UIs) to populate their
    interfaces. This is a dictionary with coroutines as values and human-readable names as keys. In DroneManager
    the names are used together with the class prefix to determine the command input on the command line, while the
    signature of the function is used to populate the CLI parser.
    The attribute background_functions should list coroutines that will run indefinitely, for example those
    polling for status updates from a camera. They will be started during construction of the class object, usually
    when the module is loaded. Note that these must be coroutines.

    """

    PREFIX = "abs"

    def __init__(self, dm, logger):
        self.dm: "dronecontrol.dronemanager.DroneManager" = dm
        self.logger = logger.getChild(self.__class__.__name__)
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

""" Class for extra, loadable plugins.

Plugins extend the functionality of DroneManager or Drone Classes by providing extra functions. They can also register
their own commands to the CLI.
"""
import abc
import asyncio
import shutil
from collections.abc import Coroutine
import importlib.util
import inspect
import pathlib
import sys

import dronemanager.core
from dronemanager.utils import DM_INSTALL_DIR


# TODO: Figure out scheduling
#   Have to interact with drone queues ("Move to position X, then turn gimbal, then move to position Y)
#   BUT, also want to perform plugin actions immediately mid flight without killing other drone tasks, (except if we do)
# TODO: Figure out how to do help strings for plugins, choices, store_true, etc... general CLI information.
# TODO: Move a bunch of the plugin handling from DroneManager to here.

class Plugin(abc.ABC):
    """ Generic plugin class.

    The attribute :py:attr:`cli_commands` is called by the DroneManager CLI (and could be called by other UIs) to
    populate their interfaces. This is a dictionary with coroutines as values and human-readable names as keys. In
    DroneManager the names are used together with the class prefix to determine the command input on the command line,
    while the signature of the function is used to populate the CLI parser.
    The attribute :py:attr:`background_functions` should list coroutines that will run indefinitely, for example those
    polling for status updates from a camera. They will be started during construction of the class object, usually
    when the module is loaded. Note that these must be coroutines.

    There is a basic dependency structure for plugins. The attribute :py:attr:`~dronemanager.plugin.Plugin.DEPENDENCIES`
    can be used to list other plugins by their names, on which this plugin depends. These are loaded before this one is.
    The list supports a single-entry deep dot-notation, i.e. "sensor.ecowitt" specifies that we depend on the ecowitt
    plugin, which requires the sensor plugin and should be loaded using their loading functions.

    A common kwarg is "name", for plugins of which multiple copies may be loaded, in which case the name acts as the
    unique identifier.

    Attributes:
        dm (dronemanager.core.DroneManager): The DroneManager instance connected to this plugin.
        logger (logging.Logger): The parent logger. A child logger with the name of the class is created below this.
        name (str): The name for this instance of the plugin.
        cli_commands (dict[str, Callable]): A dictionary with input strings as keys and the associated coroutines as
          values. The coroutine should be bare, i.e. ``coro`` instead of ``coro(args)``.
        background_functions (list[Coroutine]): A list with coroutines which will be launched automatically once the
          plugin has loaded. These coroutines should be complete, i.e. ``coro(args)`` and not ``coro``.
    """

    PREFIX: str = "abc"
    """(class attribute) The prefix for the CLI commands."""
    DEPENDENCIES: list[str] = []
    """(class attribute) Other plugins that this plugin depends on."""

    def __init__(self, dm, logger, name, *args, **kwargs):
        self.dm = dm
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
        while len(self._running_tasks) > 0:
            task = self._running_tasks.pop()
            if isinstance(task, asyncio.Task):
                task.cancel()


class MetaPlugin(Plugin, abc.ABC):
    """Plugin Class for plugins that define other plugin types, such as missions or sensors.

    """

    EXAMPLE_DIR: pathlib.Path | None = None
    """Directory in the source tree with example modules."""

    USER_DIR: pathlib.Path | None = None
    """Directory in the DroneManager install directory where the subplugins should be located."""

    VALID_CLASS_SUFFIX: str = ""
    """Valid subplugins must have class names ending with this string."""

    NAMESPACE: str = ""
    """Modules with subplugins have this prepended to their import to reduce collisions."""

    SUBTYPE: type = Plugin
    """The type that subplugins must subclass to be valid"""

    def __init__(self, dm: "dronemanager.core.DroneManager", logger, name, *args, **kwargs):
        super().__init__(dm, logger, name, *args, **kwargs)
        self._loaded = set()
        self._first_time_setup()
        self.on_load_coros = set()
        self.on_unload_coros = set()

    @property
    def loaded(self):
        return self._loaded

    def _first_time_setup(self):
        # If we have a user dir and an example dir, but user dir doesn't exist, copy examples over to the user dir
        if self.USER_DIR is not None and self.EXAMPLE_DIR is not None and not self.USER_DIR.exists():
            shutil.copytree(self.EXAMPLE_DIR, self.USER_DIR)
        # If we have a user dir, but no example dir, just create it.
        elif self.USER_DIR is not None and self.EXAMPLE_DIR is None:
            self.USER_DIR.mkdir(exist_ok=True)

    def plugin_options(self):
        # List every potential plugin file or directory in the directory
        modules = [name.stem for name in self.USER_DIR.iterdir()
                   if (name.is_file() and name.suffix == ".py" and not name.stem.startswith("_"))
                   or name.is_dir() and name.joinpath("__init__.py").is_file()]
        return modules

    # TODO: If plugin is a package, find main file and get plugin class out of it
    # TODO: Define that whole process

    def import_plugin_module(self, module: str):
        module_path = self.USER_DIR.joinpath(module)
        module_path_file = self.USER_DIR.joinpath(module + ".py")

        module_name = f"{self.NAMESPACE}.{module}"

        is_file = module_path_file.is_file()
        is_package = module_path.is_dir() and module_path.joinpath("__init__.py").is_file()

        if is_file and is_package:
            raise ValueError(
                f"Plugin {module} is ambiguous: both {module_path.as_posix()} and "
                f"{module_path_file.as_posix()} exist."
            )

        if not is_file and not is_package:
            raise ValueError(f"Plugin {module} was not found. Plugins of this type must be in {self.USER_DIR.as_posix()}")

        if module_name in sys.modules:
            raise ValueError(f"Plugin name {module} is already in use. Please rename the plugin."
            )

        if is_file:
            spec = importlib.util.spec_from_file_location(module_name, module_path_file)
        else:
            spec = importlib.util.spec_from_file_location(
                module_name,
                module_path.joinpath("__init__.py"),
                submodule_search_locations=[module_path.as_posix()],
            )

        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module

        try:
            spec.loader.exec_module(module)
        except Exception:
            sys.modules.pop(module_name, None)
            raise

        return module

    def get_plugin_class(self, module: str) -> type[Plugin]:
        try:
            plugin_mod = self.import_plugin_module(module)
            plugin_classes = [member[1] for member in inspect.getmembers(plugin_mod, inspect.isclass)
                              if issubclass(member[1], self.SUBTYPE)  # Must be appropriate subtype
                              and not member[1] in [self.SUBTYPE, MetaPlugin]  # Must strictly be subclass
                              and member[1].__name__.endswith(self.VALID_CLASS_SUFFIX)]  # Only consider marked classes
            if len(plugin_classes) != 1:
                raise RuntimeWarning(f"Too many or too few plugin classes in the module {module}!")
            return plugin_classes[0]
        except ImportError as e:
            self.logger.error(f"Couldn't load plugin {module} due to a python import error!")
            self.logger.debug(repr(e), exc_info=True)

    async def load(self, module: str, name: str | None = None):
        plugin = None
        if name is None:
            name = module
        try:
            # Basic checks that we can even try to load this plugin
            if hasattr(self.dm, name):
                raise RuntimeError(
                    f"Can't load plugin {module} with name {name} due to possible name "
                    f"collision with an existing attribute! Rename the plugin.")
            if name in self._loaded:
                self.logger.warning(f"Plugin {name} already loaded!")
                return False
            if module not in self.plugin_options():
                self.logger.warning(f"No plugin '{module}' found!")
                return False

            self.logger.info(f"Loading plugin {module} as {name}...")
            plugin = None
            try:
                plugin_class: type[Plugin] = self.get_plugin_class(module)
                if not plugin_class:
                    self.logger.error(
                        f"Module {module} contains no or multiple plugins, which is currently not supported!")
                    return False
                for dependency in plugin_class.DEPENDENCIES:
                    deps = dependency.split(".")
                    if len(deps) == 2:
                        metadep, dep = deps
                        if metadep not in self.dm.plugins:
                            meta_plugin = await self.dm.load(metadep)
                        else:
                            meta_plugin = getattr(self.dm, metadep)
                        await meta_plugin.load(dep)
                    elif len(deps) == 1:
                        if dependency not in self.dm.plugins:
                            await self.dm.load(dependency)
                    else:
                        self.logger.warning("Nested dependencies are only supported to the first level, i.e. one dot.")
                if name in self.dm.config.plugin_settings:
                    kwargs = self.dm.config.plugin_settings[name]
                else:
                    kwargs = {}
                plugin = plugin_class(self.dm, self.logger, name, **kwargs)
                setattr(self, name, plugin)
                self._loaded.add(name)
                await plugin.start()
            except Exception as e:
                self.logger.error(f"Couldn't load plugin {name} due to an exception: {repr(e)}!")
                self.logger.debug(repr(e), exc_info=True)
                if plugin is not None:
                    await plugin.close()
                if hasattr(self, name):
                    delattr(self, name)
                if name in self._loaded:
                    self._loaded.remove(name)
                return False
            self.logger.debug(f"Performing callbacks for plugin loading...")
            for func in self.on_load_coros:
                res = await asyncio.create_task(func(name, plugin))
                if isinstance(res, Exception):
                    self.logger.warning(f"Couldn't perform a callback {func} for this plugin due to an exception {repr(res)}!")
            self.logger.info(f"Completed loading Plugin {name}!")
        except Exception as e:
            self.logger.error(repr(e), exc_info=True)
        return plugin

    async def unload(self, name):
        if name not in self._loaded:
            self.logger.warning(f"No plugin named {name} loaded!")
            return False
        self.logger.info(f"Unloading plugin {name}")
        self._loaded.remove(name)
        plugin = getattr(self, name)
        unload_tasks = set()
        for func in self.on_unload_coros:
            unload_tasks.add(func(name, plugin))
        await asyncio.gather(*unload_tasks, return_exceptions=True)
        await plugin.close()
        delattr(self, name)
        return True


class PluginLoader(MetaPlugin):

    EXAMPLE_DIR = pathlib.Path(__file__).parent.joinpath("plugins")

    USER_DIR = DM_INSTALL_DIR.joinpath("plugins")

    VALID_CLASS_SUFFIX = "Plugin"

    NAMESPACE = "plugins"

    SUBTYPE = Plugin

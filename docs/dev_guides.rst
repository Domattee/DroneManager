Developer Guide
===============

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none


Conceptually, the DroneManager software is split into three large components.

The first handles core connection and command functions for a single drone. This component contains the
:py:mod:`Drone <dronemanager.drone>` and :py:mod:`MAVLink <dronemanager.mavpassthrough>` modules, as well as the
navigation functions.

The second expands this for multiple drones and makes them available to plugins. This component handles the bulk of
the logic. The main class is :py:class:`DroneManager <dronemanager.dronemanager.DroneManager>`, but it also contains
all plugins and missions. This is the core component of the library, and should be the entry point for most use cases.

The last component handles user interactions, i.e. the terminal interface. It consists of only the
:py:mod:`App <dronemanager.app>` and :py:mod:`<dronecontrol.widgets>` modules.

.. image:: imgs/implementationchart.svg

We use the python built-in ``logging`` library for all the log handling. By default, this includes the log pane on the
terminal interface and a series of files in the log directory. If you are not using the terminal interface, the argument
```log_to_console`` can be used to print messages to console instead.

.. note::

   This software makes extensive use of `asyncio library <https://docs.python.org/3/library/asyncio.html>`__. While
   multi-threading is possible with it, it is not automatically multithreaded. Care must be taken to offload
   CPU-intensive tasks to other threads or processes, or execution of the core tasks can be blocked.


Creating your own plugin
------------------------

Plugins exist to extend the functionality of DroneManager in a straightforward way. They define a list of commands that
they make available for the user interface, a list of background functions that should run continuously while the plugin
is running. They can also have dependencies on other plugins, which are then loaded automatically when the plugin is
loaded.

They are implemented as special classes in modules. Each module in the ``plugins`` folder is inspected for potential
classes. There can one plugin per module. The name of the module defines the name of the plugin. For the command-line,
they additionally provide a prefix, which is prepended to the commands to prevent collisions, i.e. multiple plugins can
have a ``connect`` command. For plugin ``abc``, this command becomes ``abc-connect``. Loaded plugins are accessible
from the DroneManager object by their name.

Below is a short guide on creating your own plugin.

Start with a python file named ``yourplugin.py`` with the following contents, and put it into the ``plugins``
directory::

   from dronemanager.plugin import Plugin

   class YourPlugin(Plugin):

       PREFIX = "test"

       def __init__(self, dm, logger, name):
           super().__init__(dm, logger, name)
           self.background_functions = [
           ]
           self.cli_commands = {
           }

This plugin doesn't do anything yet, but you can already load it, by typing ``load yourplugin``. The name of the file
defines the name of the plugin. You can have multiple classes in the file, but only one plugin class, which are
subclasses of ``Plugin`` and have names that end with "Plugin", as in the example.

To add a command to the CLI, it needs to be added to the ``cli_commands`` dictionary. The key should be a string, which
becomes the text that is entered into the CLI, while the value is the coroutine that will be executed. For example::

   class YourPlugin(Plugin):

       PREFIX = "test"

       def __init__(self, dm, logger, name):
           super().__init__(dm, logger, name)
           self.background_functions = [
           ]
           self.cli_commands = {
              "echo": self.str_echo,
           }

       async def str_echo(self, input_string: str):
           self.logger.info(input_string)

Restart DroneManager, load the plugin again and then type ``test-echo Hello!`` in the command line. It should now also
print ``Hello!`` in the log pane. The CLI commands are compounds of the plugin prefix, "test" in this case, and the
dictionary key, to prevent name collisions.

Everytime you change the code in a plugin, you will have to reload DroneManager.

Note that a CLI command must be a coroutine, not a normal function. Commands can have arguments, as in the example.
These should have type-hints, as we use them to define the parser. If you type ``test-echo What a nice day!``, you will
get an error, as the parser only expects one argument. Typing ``test-echo "What a nice day!"`` will print the message.

If an argument has a default value, it becomes an optional parameter to the CLI. List hints are also supported. Changing
the type hint to ``list[str]`` and typing ``test-echo What a nice day!`` will print ``["What", "a", "nice", day!"]``

Plugins can define dependencies through a class attribute::

   class YourPlugin(Plugin):

       PREFIX = "test"
       DEPENDENCIES = ["sensor"]

These plugins are loaded automatically when we try to load our plugin now. Dependencies also support a dot notation::

   DEPENDENCIES = ["sensor.ecowitt"]

This is necessary when you are dependent on one of the specialised plugins, currently only missions or sensors, as they
have their own loading procedures.

Finally, plugins can define background functions that are started and stopped automatically when the plugin is loaded
or closed::

   import asyncio
   from dronemanager.plugin import Plugin

   class YourPlugin(Plugin):

       PREFIX = "test"
       DEPENDENCIES = ["sensor.ecowitt"]

       def __init__(self, dm, logger, name):
           super().__init__(dm, logger, name)
           self.background_functions = [
               self.get_weather_data(),
           ]
           self.cli_commands = {
               "echo": self.str_echo,
           }

       async def str_echo(self, input_string: str):
           self.logger.info(input_string)

       async def get_weather_data(self):
           while True:
               await self.dm.ecowitt.get_data()
               await asyncio.sleep(1)

In this example, we regularly try to access the ``get_data`` function of the ecowitt sensor plugin. Since you're likely
not connected to such a sensor, you will get error messages in the log pane instead.
Like CLI commands, background functions must be coroutines, but note the different syntax with the parenthesis.


.. _guide_mission:

Creating custom missions
------------------------

Missions fundamentally work very similar to plugins, with a few key changes. They must be subclasses of the :class:`Mission`
abstract base class, they go into a folder "missions" and they have a new attribute ``name``, which takes the role of
the prefix used for plugins. Unlike with plugins, this is an instance attribute, allowing multiple instances of the same
mission class.
Loading a mission is done with ``mission-load <file> <name?>``, where <file> is the name of the file with the mission,
similar to plugins. The name argument is optional, by default the name of the file is used. The mission plugin must be
loaded first (done at startup by default).

Compared to plugins, there are 5 functions which missions must provide, as they used by other parts of the software.
They also come with three suggested attributes: A "current_stage", for whatever state the mission is currently in, a
"flight_area" and "drones", an ordered dictionary of the drones participating in this mission. Usage of these attributes
is completely optional, but if they are set, then information about them is sent out over the UDP plugin by default.


Navigation function guide
-------------------------

TODO: Explanation of the navigation system and how the components interact


Writing documentation
---------------------

We use Google-style docstrings. Most IDEs can be set up to configure
which style of docstring stub is generated.
Type hints go into the signature, not the docstring.

For classes, everything goes into the class docstring, except the arguments
for __init__, which go into the __init__ docstring.

Sphinx uses the class hierarchy to try and find a docstring when a class
overrides a member of its parent class. This can lead to errors when the
docstring of the parent class doesn't meet the formatting standards. In
this case, the subclass should provide its own docstring, referencing the
parent class when necessary.

For general formatting, see the sphinx documentation, they have examples
of Google-style docstrings as well.

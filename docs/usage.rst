Usage
=====

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none


Quick Start
-----------

A command ``dm`` is set up as part of the installation. Typing this command into a terminal will start a new
DroneManager instance and terminal interface.

The interface is split into three components:

- A log pane on the left that shows messages, confirmations and warnings.
- A status pane on the right that shows key information for any connected drones, such as the position.
- A command line on the bottom, which is the main way to interact with drone manager.

.. image:: imgs/tool-screenshot.png

In one terminal, activate the venv and start up DroneManager by typing ``dm``.
In a separate terminal, start up a PX4 SITL drone by moving to the PX4 SITL root directory and typing
``make px4-sitl gz_x500``.

.. note::
  This guide assumes that you also installed the PX4 SITL environment so we can use simulated drones.

To connect to the simulated drone with DroneManager, click on the command line at the bottom and type::

   connect tom udp://127.0.0.1:14540

This creates a new drone object, assigns it the name ``tom`` and tries to connect to a MAVLink Node with the given
connection string. You can provide any name, it will be used to identify the drone in other commands.

.. note::
   Typing any command with ``--help`` or ``-h`` prints a help string for that command.
   Also, a reference sheet with all default commands and their options is at the :ref:`bottom of this page <com_ref>`.

.. note::
   As a convenience feature, we provide a config file in which drone names and their connection string as well as other
   parameters may be saved. When entered with a name from this config file, the connect function will use the parameters
   from the file unless overridden in the command line.

After a short moment, you should see text pop in on the status pane showing information for our newly connected drone.

To arm the drone, type::

   arm tom

You should see the arm state of the drone switch. A few moments later, the drone should automatically disarm.

To take-off, arm the drone again and then type::

   takeoff tom

The drone should take-off to about 2 meters above the ground.

To move the drone to a coordinate (in the local coordinate system) type::

   flyto tom 10 0 -2 -90

This flies the drone to a position 10m north of the takeoff point at about the same altitude, facing west.
Note that the local coordinate system uses the NED convention, i.e. up is negative.

To move the drone by a certain amount::

   move tom -5 0 0 0

which moves the drone 5m toward the south of its current position, keeping everything else the same.

You can also fly the drone to a GPS coordinate::

   flytogps tom <lat> <long> <amsl> <yaw>

Land the drone with::

   land tom

After landing, the drone should auto-disarm. If not, you can type ``disarm tom``.

Flying a drone with these commands by hand can be quite tedious, as you have to eye-ball distances and facings. We
also support game pads. The ``controllers`` plugin is loaded by default. If you connect a DualShock 4 you can use it to
fly the drone with no further text commands.

Controls:

+-----------------------+-----------------------+
| Input                 | Action                |
+=======================+=======================+
| PS Button             | Take/release control  |
+-----------------------+-----------------------+
| Circle                | Arm                   |
+-----------------------+-----------------------+
| X                     | Disarm                |
+-----------------------+-----------------------+
| D-pad up              | Take-off              |
+-----------------------+-----------------------+
| D-pad down            | Land                  |
+-----------------------+-----------------------+
| Right stick           | Horizontal movement   |
+-----------------------+-----------------------+
| Left stick vertical   | Vertical movement     |
+-----------------------+-----------------------+
| Left stick horizontal | Yaw                   |
+-----------------------+-----------------------+

.. note::
   We currently only support the DualShock 4, as that is the only controller we have access to. However, we use pygame
   for the game pad support, so many more are supported in principle. You will have to make your own keybinds however.
   See the documentation of the controller plugin for information.


That's it for the core controls. However, for your research you will probably want to test some algorithms or use real
drones in some workflow. DroneManager enables this by making these core commands and more also available
programmatically.

A key feature to enable repeatable work is our "mission" concept. In principle, they are just fancy scripts. They can
do anything you could put into a python script, but can also readily make use of safety features we have developed,
automatically generate commands for the terminal interface, while still allowing manual control as well.
In the next section, we take a look at an example mission with multiple drones.

See :ref:`the mission guide <guide_mission>` for more information on setting up your own.

Example mission
---------------

TODO: UAM, whole setup in gazebo

Configuration file
------------------

TODO: Config file


Drone parameters
----------------

TODO: Short guide to Parameters


Common Issues
-------------

Fundamentally, our software works by sending commands to the flight controller (or another piece of hardware on the
drone), which then executes those commands. We've set up and tested our software to work with default PX4 and Ardupilot
configurations out-of-the-box, but if the flight controller doesn't respond as our software expects it too,
there isn't much we can do.
For example, depending on configuration of the flight controller, the "home" position might be the take-off position,
or it might be the coordinate origin point, usually set at boot-up for the local coordinate system. Imagine booting up
a drone on a desk, then carrying it to the take-off point, then setting up your laptop and notes on the same desk. When
you tell the drone to return-to-base, it might very well try to land on your laptop.
When working with real drones you should always test the actual behaviour in a safe environment before risking hardware
(or lives!).

Connection Issues
^^^^^^^^^^^^^^^^^

TODO: Ports, Mavlink awkwardness
TODO: Firewalls

Coordinate Systems
^^^^^^^^^^^^^^^^^^

With PX4 and Ardupilot there are two main coordinate systems:

- Global: Latitude, longitude and altitude above mean sea level.
- Local: A ``(x,y,z)`` cartesian coordinate system assuming a flat earth but with arbitrary axes and origin, usually
  initialized at boot-up.

Both come with their own downsides and idiosyncrasies. Note that the coordinate system used is independent of how the
drone determines its position. For example, a drone might use GNSS positioning but can still keep a local coordinate
system.

The global coordinate system requires a GNSS signal, making it generally inaccessible indoors, and often has poor
accuracy, which can make coordinating multiple drones difficult and be a challenge for reproducibility as well.

The local coordinate system can change significantly depending on the configuration of the firmware.
The standard configurations have the axes aligned with north, east and down and the origin set wherever the flight
controller first got some kind absolute position fix, usually shortly after boot-up. So if you have multiple drones
and you are setting them up in different locations, they will have different local coordinate systems.
With an external visual tracking system for positioning, the tracking system might use a forward, right and up
convention, while the drone expects forward, right and down.
With visual odometry instead of an absolute system, the axes might be aligned with the orientation of the drone at
boot-up.

Offboard mode
^^^^^^^^^^^^^

TODO: All of it


.. _com_ref:

Command reference
-----------------

The syntax used in this reference is as follows:

- ``<Parameters>`` are mandatory positional parameters.
- ``<Parameters?>`` are optional positional parameters.
- ``-p`` are boolean flags.
- ``-p <parameter: defaultValue>`` are optional parameters with a flag to indicate that they are being supplied. Usually,
  these have a default value.

Many commands can be "scheduled" by adding the flag `-s`, which means that the drone will finish any previous commands
before proceeding to the scheduled command. Multiple commands can be scheduled at once. Entering a command without
scheduling clears the schedule, i.e. the drone will cancel whatever it is doing and execute the new command immediately.

A command is considered "complete" when some condition is met, depending on the command, or when it errors out, either
because of an exception or because the flight controller denied the command, for example when trying to arm a drone
without a GPS fix (if the flight controller is configured to require a GPS fix for arming). Commands for multiple
drones complete independently, i.e. if you schedule a takeoff and a move for two drones, but one of the drones doesn't
reach the takeoff altitude, the other one will still start its move once its takeoff has completed.

You can also add `-h` or `--help` to print a help string, either for the whole interface or a specific command.

This list only contains the "core" commands that belong to DroneManager directly. For commands added by plugins, see
the documentation for the plugins.

TODO: Command list for non-plugin commands

+-------------------------+--------------------------+--------------------------+
| command                 | Arguments                | Description              |
+=========================+==========================+==========================+
| command -flag -s        | -flag: Toggles something | A dummy command line     |
|                         | -s: Schedule this command| with a long description  |
+-------------------------+--------------------------+--------------------------+
| command -flag -s        | -flag: Toggles something | A dummy command line     |
|                         | -s: Schedule this command| with a long description  |
+-------------------------+--------------------------+--------------------------+


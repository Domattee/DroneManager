Usage
=====

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none


Quick Start
-----------

A command called "dm" is set up as part of the installation. Typing this command into a terminal will start a new
DroneManager instance and terminal interface.

The interface is split into three components:
 - A log pane on the left that shows messages, confirmations and warnings.
 - A status pane on the right that shows key information for any connected drones, such as the position.
 - A command line on the bottom, which is the main way to interact with drone manager.

.. image:: imgs/tool-screenshot.png

In a terminal, start up DroneManager.
In a separate terminal, start up a PX4 SITL drone by moving to the PX4 SITL root directory and typing

.. note::
  This guide assumes that you also installed the PX4 SITL environment so we can use simulated drones.

To connect to the drone, type::

   connect tom udp://127.0.0.1:14540

This creates a new drone object, assigns it the name ``tom`` and tries to connect to a MAVLink Node at the given
IP and port. You can provide any name, it will be used to identify the drone in other commands. For details about all
the commands and their options, see the command reference. TODO

To arm the drone, type::

   arm tom

You should see the arm state of the drone switch. After a short moment, the drone should automatically disarm.


TODO - Usage quick start: Drone setup / connect / arm / takeoff / fly / land

TODO: Note on WSL IP when connecting.

TODO: Controller

Related set of functions or repeatable stuff: Write a mission

TODO: Config file

New features: Write a plugin

Safety features (fence)

Example mission
---------------

TODO: UAM, whole setup in gazebo

Traps
-----

TODO:
Coordinate systems
offboard

Drone parameters
----------------

TODO: Short guide to Parameters

Command reference
-----------------

TODO: General command structure


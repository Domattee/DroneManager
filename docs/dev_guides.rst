Developer Guide
===============

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none


Conceptually, the DroneManager software is split into three large components.

The first handles core connection and command functions for a single drone. This component contains the
:py:mod:`Drone <dronecontrol.drone>` and :py:mod:`MAVLink <dronecontrol.mavpassthrough>` modules, as well as the
navigation functions.

The second expands this for multiple drones and makes them available to plugins. This component handles the bulk of
the logic. The main class is :py:class:`DroneManager <dronecontrol.dronemanager.DroneManager>`, but it also contains
all plugins and missions.

The last component handles user interactions, i.e. the terminal interface. It consists of only the
:py:mod:`App <dronecontrol.app>` and :py:mod:`<dronecontrol.widgets>` modules.

TODO: GRAPH

TODO:
    - Two entry points for code: Drone or DroneManager
    - Recommended: DroneManager
    - New functionality: Plugins
    - Missions to collect functions that all relate to some workflow

.. note::

   TODO: Short bit on asyncio and its traps


Navigation function guide
-------------------------

TODO: Explanation of the navigation system and how the components interact

Creating your own plugin
------------------------

TODO: Example process of making a plugin, using existing as example

Creating your own mission
-------------------------

TODO: Example process of making a mission, using existing as example

Example missions
----------------

TODO: Explanation of UAM, holodeck functions

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

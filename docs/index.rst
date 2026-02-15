.. DroneManager documentation master file, created by
   sphinx-quickstart on Thu Dec 11 12:23:47 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

DroneManager documentation
==========================

DroneManager consists of two main components: The DroneManager library itself and a terminal application that uses
it. It can be used to connect to and command multiple drones using either game pads or text controls.

TODO: expand this note, point to usage guide and dev guides

.. note::

   This project is under active development.


.. toctree::
   :maxdepth: 2
   :caption: Contents:

   installation
   usage
   dev_guides
   holodeck
   api/index

Holodeck
--------

Holodeck is an example for the integration of DroneManager with modern simulation environments like Unity. It allows Users to control a real (or simulated) drone with a Playstation Controller via DroneManager while experiencing the virtual flight in a Unity Environment with enhanced perception through VR-Glasses (e.g. Meta-Quest Pro).

See the :doc:`Holodeck documentation <holodeck>` for more information.

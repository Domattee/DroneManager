# DroneControl

A package to connect to and control multiple drones.

## Table of Contents:

- [Installation](#installation)
- [Usage](#usage)
  - [Terminal Interface](#terminal-interface)
  - [Configuration file](#configuration-file)
  - [Plugins](#plugins)
- [Examples](#examples)
  - [Holodeck](#holodeck)
  - [UAM](#uam-demo)

## Installation

1. Clone this repository, 
2. Move into the root directory of the repository 
3. On Windows only: ` python windows_setup.py `
4. Install with pip:
```
pip install --upgrade pip
pip install -e .
```


As part of the installation a command called ```dm``` is installed, which starts the terminal interface. 
Alternatively you can run the app.py script.

> [!NOTE]  
> The additional script on windows is required to install MSVC and the MAVSDK Server binary, which isn't shipped with the installation.

## Usage

### Terminal interface

There are a large number of possible commands. The basic ones are listed below. Feedback is provided in the log pane.
Exception information, such as stack traces, is additionally logged in the log files. 

Many commands can be "scheduled" by adding the flag `-s`, which means that the drone will finish any previous commands 
before proceeding to the scheduled command. Multiple commands can be scheduled at once. Entering a command that isn't 
scheduled clears the schedule, i.e. the drone will follow it immediately. 

A command is considered "complete" when some condition is met, depending on the command, or when it errors out, either 
because of an exception or because the flight controller denied the command, for example when trying to arm a drone 
without a GPS fix. Commands for multiple drones complete independently, i.e. if you schedule a takeoff and a move for 
two drones, but one of the drones doesn't reach the takeoff altitude, the other one will still start its move once its 
takeoff has completed.

The syntax below is as follows: 
- `<Parameters>` are mandatory positional parameters. 
- `<Parameters?>` are optional positional parameters.
- `-p` are boolean flags.
- `-p <parameter: defaultValue>` are optional parameters with a flag to indicate that they are being supplied. Usually, 
these have a default value.

You can also add `-h` or `--help` to print a help string, either for the whole interface or a specific command. The 
help string for plugin commands is sparser than core commands.

#### Commanding drones

- `connect <name> <connection-string?> -t <timeout: 30> -f <frequency: None>`: Connect to a drone. The parameter name is an arbitrary label that 
is used to refer to the drone with other commands. If a name matching an entry in the config file is used, the configuration from the file will
be loaded. The connection string, for example "udp://192.168.0.143:14550", defines how to connect to the drone. This parameter is optional,
by default "udp://:14540" is used. With `-t` a timeout in seconds can be specified, the default is 30s. The parameter `-f` specifies the
telemetry frequency from the drone. A number of messenges from the are drone are requested at this rate, such as position. A number of
components also use this frequency for their own purposes, such as time discretization of trajectories. If this parameter is omitted, the
default value from the config file is used.
- `disconnect <names> -f`: Close the connection to the specified drones. This command will refuse if the drones are 
armed or flying, add the `-f` flag to force disconnect.
- `arm <names> -s`: Arm one or more drones. Multiple drones can be armed at once by listing their name with a space 
between, i.e. `arm drone1 drone2 drone3`. This command is considered complete when the drone is armed and can be 
scheduled.
- `disarm <names> -s`: Disarm one or more drones. This command is complete when the drone has disarmed and 
can be scheduled.
- `takeoff <names> -a <altitude: 2> -s`: Perform a takeoff with the specified drones. The optional altitude 
parameter specifies the target altitude above the launch point. This command is complete when the drone has reached the 
target position and can be scheduled.
- `land <names> -s`: Land the specified drones at their current locations. This puts the drones into offboard mode! 
This command is complete when the drone has landed and can be scheduled.
- `flyto <name> <x> <y> <z> <yaw?> -t <tolerance: 0.25> -s`: Fly the specified drone to the position x, y, z in the 
local coordinate system. This puts the drones into offboard mode! The optional parameter `yaw` defines the facing of
the drone. The heading change and movement usually happen simultaneously. The optional parameter `-t` can be used to
specify a tolerance for how close the drone must be to the target position to have "reached" it. By default, this is
0.25m. This command is complete when the drone is within the tolerance of the target position and can be scheduled.
- `flytogps <name> <lat> <long> <amsl> <yaw?> -t <tolerance: 0.25> -s`: Fly the specified drone the provided GPS position.
Otherwise identical to `flyto`.

And many more!

### Configuration file

To simply working with a variety of drones, a number of parameters can be set per-drone in a config file. This allows you 
to save a name with connection string and a number of other parameters, such as acceleration limits, which will be loaded
and used automatically when `connect` with a corresponding name is called.

### Plugins

DroneManager comes with a plugin system for adding extra functionality! The core element are plugin modules, located in 
the "plugins" folder. Each plugin module contains one plugin class, which is a subclass of `plugin.Plugin`, and defines 
extra behaviour. The base class provides a framework for automatically generating CLI commands and booting up any 
background functions. See the plugin documentation (TODO) for more information. A number of plugins are shipped with 
DroneManager.

- `plugins`: Shows a list of available plugins.
- `load <name>`: Load a plugin by name. This must match one of the names shown by `plugins`.
- `unload <name>`: Unload a plugin by name.
- `loaded`: List currently loaded plugins.

By default, the `mission`, `controllers` and `external` plugins are loaded at startup. 
Controllers allows for manual control of drones using common gamepads. It uses pygame and provides functions for creating 
custom button and axis bindings.
Missions are essentially a special kind of plugin. They go into their own folder "missions". Do not try out missions with 
real drones without understanding what they do first!

- `mission-status`: Prints a list of all available missions, as well as an overview for each currently running mission.
- `mission-load <name> <label?>`: Load a mission by name. This must match one of the missions returned by 
`mission-status`. The optional parameter `label` can be used to assign the mission a specific name. Each mission must 
have a unique name, so this allows multiple missions of the same "type".

The plugin `external` creates a local UDP server that sends out information about connected drones and any running missions 
on request using json format. We provide a dummy client script that sends the request messages and prints the information
from DM to console.

#### Other Plugins

- `gimbal`: A plugin for controlling gimbals connected to the drone FC.
- `camera`: A plugin for controlling cameras connected to the drone FC. Quite specific for our camera and probably not generally applicable.
- `sensor`: A plugin for managing generic extra data sources. We also have example code for a specific sensor: An Ecowitt weather station.
- `scripts`: A plugin that allows executing arbitrary scripts from the terminal interface. They are executed in a separate process.
- `optitrack`: A plugin using the python code from the NatNetSDK. Can be used to connect to a running motive server and
  forward tracking information to any connected drones. This also requires that motive be configured to stream the information and that
  the flight controller on the drones themselves be configured to use this information.

## Examples

### Holodeck

TODO

### UAM Demo

This is a showcase demo where three drones look for a POI and start continuously observing it.
To run it, you will need three drones, preferably running PX4, a dummy object of interest and a 7 x 3 x 3 meter area 
where you can fly multiple drones with high precision.
In the event we are flying with three drones, "luke", "derek" and "corran". No object recognition takes place, the 
POI location is pre-determined.
We ran this demo in an indoor environment with an OptiTrack system for positioning.
The setup instructions below assume a similar setup.

If you don't have an indoor flying set up ready to go, we suggest going outside and using GPS instead. The setup will 
have to be modified to allow for positiong errors from GPS by increasing the flight area significantly, at least triple. 
Relevant parameters are in the `init` function of the mission and include `flight_area`, `search_space`, 
`start_position_x`, `flight_altitude`, `poi_position` and `holding_position`. You will also have to ensure the drones 
share a common local coordinate system. You can do this by powering them on one by one in the center of the flight 
area with the same heading and then moving them to their start positions.

#### Setup
1. Boot up dm and make sure you are in the correct Wi-Fi.
2. Connect to all the drones: `connect <name>` with names "luke", "derek" or "corran"
3. Load the mission scripts: `mission-load uam`
4. Add all the drones to the mission: `uam-add <name>` IMPORTANT! The order in which the drones are added matters. The 
first drone has start position (3, -1.25), the second one (3, 0) and the third (3, 1.25). If the drones are added in the 
wrong order, they might collide during flight as their paths can cross. With `uam-status`, you can see the order of the 
drones. If drones were added in the wrong order, you can either rearrange them on the field, or remove 
`uam-remove <name>` and re-add them.
5. Check that each drone reports the correct position. If they report 0,0, the tracking system isn't connected. 
6. Do `uam-set` to change the mission state to "ready-to-go". With `uam-unset` you can go back to Uninitialized.

#### Mission
With the drones connected and all the scripts loaded you can begin flying missions.
1. For the single search pattern: Start the mission with `uam-singlesearch`. The first drone will fly the rectangular 
search pattern and start circling the object indefinitely once it finds it. To return it, do `uam-rtb`.
2. For the group search: Start the mission with `uam-groupsearch`. All drones will launch and fly forwards to the other 
end. The drone that finds the POI will stay with the POI, the other two come back, land and disarm. The observing 
drone starts circling until its (faked) battery runs low, when one of the drones will arm, takeoff and fly to do the 
swap. The observing drone will stop circling and wait until the swapping drone has eyes on the object, at which point 
the observing drone will rtb on its own. This swapping happens indefinitely. To stop the mission, do `uam-rtb`.
3. To fly drones from any position to their start position one-by-one you can use `uam-reset`. This should only be used 
in Gazebo, the drones should already be at their start positions in a real demo.

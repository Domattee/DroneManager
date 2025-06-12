import asyncio
import datetime
import os
import shlex

from dronecontrol.dronemanager import DroneManager
from dronecontrol.drone import Drone, DroneMAVSDK
from dronecontrol.utils import common_formatter, check_cli_command_signatures

import textual.css.query
from textual import on, events
from textual.app import App, Screen, Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.widgets import Footer, Header, Log, Static, RadioSet, RadioButton, ProgressBar
from textual.widget import Widget

from dronecontrol.widgets import InputWithHistory, TextualLogHandler, DroneOverview, ArgParser, ArgumentParserError

import logging

# TODO: Need some kind of "awaiter" process, that awaits each of the CLI tasks. Can't do that in the CLI function, as it
# will block the UI. Create an extra tasks that awaits the CLI tasks? Who awaits the awaiter tasks?
# TODO: Fence, trajectory generator and trajectory follower managing somehow

pane_formatter = logging.Formatter('%(asctime)s %(levelname)s %(name)s - %(message)s', datefmt="%H:%M:%S")

DRONE_DICT = {
    "luke":   "udp://192.168.1.31:14561",
    "wedge":  "udp://192.168.1.32:14562",
    "derek":  "udp://192.168.1.33:14563",
    "tycho":  "udp://192.168.1.34:14564",
    "gavin":  "udp://192.168.1.35:14565",
    "corran": "udp://192.168.1.36:14566",
    "jaina":  "udp://192.168.1.37:14567",
    "kira":   "serial://COM5:460800",
    "riker":  "udp://192.168.0.3:14550",
}

UPDATE_RATE = 20  # How often the various screens update in Hz. TODO: Currently time delay after function, refactor to
# ensure actual 20hz refresh rate

DEFAULT_PLUGINS = ["mission"]


class StatusScreen(Screen):

    CSS = """
ProgressBar {
    width: 25;
    height: 1;
    layout: horizontal;
}

Bar {
    width: 20;
    height: 1;
}
"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dm: DroneManager = self.app.dm
        self.cur_drone: Drone | None = None
        self.logger = self.app.logger
        asyncio.create_task(self._update_values())
        self.dm.add_connect_func(self._add_drone)
        self.dm.add_remove_func(self._remove_drone)

    async def _update_values(self):
        while True:
            # Update fields
            try:
                if self.cur_drone is not None:
                    self.query_one("#name", expect_type=Static).update(f"{self.cur_drone.name}")
                    self.query_one("#address", expect_type=Static).update(f"{self.cur_drone.drone_addr}")
                    self.query_one("#attitude", expect_type=Static).update(f"{self.cur_drone.attitude}")
                    self.query_one("#battery", expect_type=ProgressBar).update(
                        progress=self.cur_drone.batteries[0].remaining)
                else:
                    self.query_one("#name", expect_type=Static).update("NAME: NO DRONE SELECTED")
                    self.query_one("#address", expect_type=Static).update("ADDRESS: NO DRONE SELECTED")
                    self.query_one("#attitude", expect_type=Static).update("ATTITUDE: NO DRONE SELECTED")
                    self.query_one("#battery", expect_type=ProgressBar).update(progress=0)
            except textual.app.NoMatches:
                pass
            except Exception as e:
                self.logger.error(f"Error updating values.")
                self.logger.debug({repr(e)}, exc_info=True)
            await asyncio.sleep(1/UPDATE_RATE)

    def compose(self):
        with Horizontal():
            with RadioSet(id="droneselector"):
                yield RadioButton("None", id="button_no_drone")
            with Vertical():
                yield Static(id="name", content="NAME: NO DRONE SELECTED", markup=False)
                yield Static(id="address", content="ADDRESS: NO DRONE SELECTED", markup=False)
                yield ProgressBar(id="battery", total=100, show_eta=False)
                yield Static(id="attitude", content="ATTITUDE: NO DRONE SELECTED", markup=False)
        yield Footer()

    def on_radio_set_changed(self, event: RadioSet.Changed) -> None:
        if event.pressed.label.plain == "None":
            self.cur_drone = None
        else:
            try:
                self.cur_drone = self.dm.drones[event.pressed.label.plain]
            except KeyError:
                pass

    async def _add_drone(self, name, drone):
        self.logger.debug(f"Adding radio button for {name}")
        radio_selector = RadioButton(f"{name}", id=f"button_{name}")
        radio_field = self.query_one("#droneselector", expect_type=RadioSet)
        await radio_field.mount(radio_selector)

    async def _remove_drone(self, name):
        if self.cur_drone is not None and self.cur_drone.name == name:
            # Have to change current drone to prevent stuff breaking
            self.cur_drone = None
        try:
            self.logger.debug(f"Removing radio button for {name}")
            await self.query_one(f"#button_{name}", expect_type=RadioButton).remove()
            # Move currently selected button after removal to prevent index errors
            selector = self.query_one(f"#droneselector", expect_type=RadioSet)
            selector.action_next_button()
        except textual.css.query.NoMatches:
            pass


class CommandScreen(Screen):
    # TODO: Make the CSS better, change widths and whatever dynamically
    # TODO: Print a pretty usage/command overview thing somewhere.
    # TODO: Currently you can enter commands after going "exit", need to prevent that

    # How often the drone overview screen is updated.
    STATUS_REFRESH_RATE = 20

    CSS = """
.text {
    text-style: bold;
}

#status {
    height: 5fr;
}

#usage {
    height: 1fr;
}

#sidebar {
    width: 97;
}
"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dm: DroneManager = self.app.dm
        self.drone_widgets: dict[str, Widget] = {}
        self.running_tasks: set[asyncio.Task] = set()
        # self.drones acts as the list/manager of connected drones, any function that writes or deletes items should
        # protect those writes/deletes with this lock. Read only functions can ignore it.
        self._kill_counter = 0  # Require kill all to be entered twice
        self.logger = self.app.logger
        self.log_pane_handlers = {}

        # Parser stuff
        self._exit_aliases = ["quit", "close", "q"]
        base_parser, command_parser = self._base_parser()
        self.parser = base_parser
        self.command_parser = command_parser

        self.dynamic_commands = {}

        self.dm.add_connect_func(self._add_drone_object)
        self.dm.add_remove_func(self._remove_drone_object)

        self.dm.add_plugin_load_func(self._load_plugin_commands)
        self.dm.add_plugin_unload_func(self._unload_plugin_commands)

        for plugin_name in DEFAULT_PLUGINS:
            asyncio.create_task(self.dm.load_plugin(plugin_name))

        self._awaiter_tasks = set()

    def _base_parser(self):
        parser = ArgParser(description="Interactive command line interface to connect and control multiple drones")
        command_parsers = parser.add_subparsers(title="command", description="Command to execute.", dest="command")

        connect_parser = command_parsers.add_parser("connect", help="Connect a drone")
        connect_parser.add_argument("drone", type=str, help="Name for the drone.")
        connect_parser.add_argument("drone_address", type=str, nargs='?',
                                    help="Connection string. Something like udp://:14540")
        connect_parser.add_argument("-sa", "--server_address", type=str, default=None,
                                    help="Address for the mavsdk server. If omitted, a server is started "
                                         "automatically. Use this only if you already have a server for this drone "
                                         "running (for example on another machine). Default None.")
        connect_parser.add_argument("-sp", "--server_port", type=int, default=50051,
                                    help="Port for the mavsdk server. Default 50051.")
        connect_parser.add_argument("-t", "--timeout", type=float, default=30, required=False,
                                    help="Timeout in seconds for connection attempts. Default 120s.")

        disconnect_parser = command_parsers.add_parser("disconnect", help="Disconnect one or more drones.")
        disconnect_parser.add_argument("drones", type=str, nargs="+", help="Which drones to disconnect.")
        disconnect_parser.add_argument("-f", "--force", action="store_true",
                                       help="If this flag is set, ignore any potential checks and force the disconnect.")

        arm_parser = command_parsers.add_parser("arm", help="Arm the named drone(s).")
        arm_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to arm")
        arm_parser.add_argument("-s", "--schedule", action="store_true",
                                help="Queue this action instead of executing immediately.")

        disarm_parser = command_parsers.add_parser("disarm", help="Disarm the named drone(s).")
        disarm_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to disarm")
        disarm_parser.add_argument("-s", "--schedule", action="store_true",
                                   help="Queue this action instead of executing immediately.")

        takeoff_parser = command_parsers.add_parser("takeoff", help="Puts the drone(s) into takeoff mode.")
        takeoff_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to take off with.")
        takeoff_parser.add_argument("-a", "--altitude", type=float, required=False, default=2.0,
                                    help="Takeoff altitude, default 2m, positive is up.")
        takeoff_parser.add_argument("-s", "--schedule", action="store_true", required=False,
                                    help="Queue this action instead of executing immediately.")

        flight_mode_parser = command_parsers.add_parser("mode", help="Change the drone(s) flight mode")
        flight_mode_parser.add_argument("mode", type=str,
                                        help=f"Target flight mode. Must be one of {self.dm.drone_class.VALID_FLIGHTMODES}.")
        flight_mode_parser.add_argument("drones", type=str, nargs="+",
                                        help="Drone(s) to change flight mode on.")
        flight_mode_parser.add_argument("-s", "--schedule", action="store_true",
                                        help="Queue this action instead of executing immediately.")

        fence_parser = command_parsers.add_parser("fence", help="Set a geofence-type thing. VERY WIP")
        fence_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to set the fence on.")
        fence_parser.add_argument("nl", type=float, help="Lower area limit along 'North' axis")
        fence_parser.add_argument("nu", type=float, help="Upper area limit along 'North' axis")
        fence_parser.add_argument("el", type=float, help="Lower area limit along 'East' axis")
        fence_parser.add_argument("eu", type=float, help="Upper area limit along 'East' axis")
        fence_parser.add_argument("h", type=float, help="Height limit, upper only. Positive for up.")

        fly_to_parser = command_parsers.add_parser("flyto", help="Send the drone to a local coordinate.")
        fly_to_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_to_parser.add_argument("x", type=float, help="Target x coordinate")
        fly_to_parser.add_argument("y", type=float, help="Target y coordinate")
        fly_to_parser.add_argument("z", type=float, help="Target z coordinate")
        fly_to_parser.add_argument("yaw", type=float, nargs="?", default=None,
                                   help="Target yaw in degrees. Default maintains current facing.")
        fly_to_parser.add_argument("-t", "--tolerance", type=float, required=False, default=0.25,
                                   help="Position tolerance")
        fly_to_parser.add_argument("-s", "--schedule", action="store_true",
                                   help="Queue this action instead of executing immediately.")

        fly_to_gps_parser = command_parsers.add_parser("flytogps", help="Send the drone to a GPS coordinate")
        fly_to_gps_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_to_gps_parser.add_argument("lat", type=float, help="Target latitude")
        fly_to_gps_parser.add_argument("long", type=float, help="Target longitude")
        fly_to_gps_parser.add_argument("alt", type=float, help="Target altitude (relative to takeoff)")
        fly_to_gps_parser.add_argument("yaw", type=float, nargs="?", default=None,
                                       help="Target yaw in degrees. Default maintains current facing.")
        fly_to_gps_parser.add_argument("-t", "--tolerance", type=float, required=False, default=0.25,
                                       help="Position tolerance")
        fly_to_gps_parser.add_argument("-s", "--schedule", action="store_true",
                                       help="Queue this action instead of executing immediately.")

        move_parser = command_parsers.add_parser("move", help="Send the drones x, y, z meters north, east or down.")
        move_parser.add_argument("drone", type=str, help="Name of the drone")
        move_parser.add_argument("x", type=float, help="How many meters to move north (negative for south).")
        move_parser.add_argument("y", type=float, help="How many meters to move east (negative for west).")
        move_parser.add_argument("z", type=float, help="How many meters to move down (negative for up).")
        move_parser.add_argument("yaw", type=float, nargs="?", default=0.0,
                                 help="How many degrees to move to the right (negative for left). Note that this does "
                                      "not wrap around, i.e. 350 degrees to right will move the drone 10 degrees to "
                                      "the left. Default 0 degrees.")
        move_parser.add_argument("-nogps", action="store_true",
                                 help="If this flag is set we move using the drones local coordinate system.")
        move_parser.add_argument("-t", "--tolerance", type=float, required=False, default=0.25,
                                 help="Position tolerance")
        move_parser.add_argument("-s", "--schedule", action="store_true",
                                 help="Queue this action instead of executing immediately.")

        land_parser = command_parsers.add_parser("land", help="Land the drone(s)")
        land_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to land")
        land_parser.add_argument("-s", "--schedule", action="store_true", help="Queue this action instead of "
                                                                               "executing immediately.")

        pause_parser = command_parsers.add_parser("pause", help="Pause the drone(s) task execution")
        pause_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to pause")

        resume_parser = command_parsers.add_parser("resume", help="Resume the drone(s) task execution")
        resume_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to resume")

        stop_parser = command_parsers.add_parser("stop", help="Stops (i.e. lands) drones. If no drones are listed,"
                                                 " stops all of them and then exits the application")
        stop_parser.add_argument("drones", type=str, nargs="*", help="Drone(s) to stop.")

        kill_parser = command_parsers.add_parser("kill", help="Kills (i.e. disarms and stops everything) drones. "
                                                 "If no drones are listed, kills all of them.")
        kill_parser.add_argument("drones", type=str, nargs="*", help="Drone(s) to kill.")

        plugin_load_parser = command_parsers.add_parser("load", help="Loads a given plugin.")
        plugin_load_parser.add_argument("plugin", type=str, help="Plugin name to load.")

        plugin_load_parser = command_parsers.add_parser("unload", help="Unloads a given plugin.")
        plugin_load_parser.add_argument("plugin", type=str, help="Plugin name to unload.")

        available_plugin_parser = command_parsers.add_parser("plugins", help="Prints a list of available plugins")

        loaded_plugin_parser = command_parsers.add_parser("loaded", help="Prints a list of loaded plugins")

        exit_parser = command_parsers.add_parser("exit", aliases=self._exit_aliases, help="Exits the application")

        cam_prepare_parser = command_parsers.add_parser("cam-prep", help="Prepare camera plugin")
        cam_prepare_parser.add_argument("drone", type=str, help="Which drones should take a picture")

        cam_settings_parser = command_parsers.add_parser("cam-settings", help="Start recording video")
        cam_settings_parser.add_argument("drone", type=str, help="Which drones should take a picture")

        cam_picture_parser = command_parsers.add_parser("cam-photo", help="Take a picture")
        cam_picture_parser.add_argument("drone", type=str, help="Which drones should take a picture")

        cam_video_start_parser = command_parsers.add_parser("cam-start", help="Start recording video")
        cam_video_start_parser.add_argument("drone", type=str, help="Which drones should take a picture")

        cam_video_stop_parser = command_parsers.add_parser("cam-stop", help="Start recording video")
        cam_video_stop_parser.add_argument("drone", type=str, help="Which drones should take a picture")

        cam_zoom_parser = command_parsers.add_parser("cam-zoom", help="Start recording video")
        cam_zoom_parser.add_argument("drone", type=str, help="Which drones should take a picture")
        cam_zoom_parser.add_argument("zoom", type=float, help="Target zoom level")

        ### RC - Parsers

        ql_parser = command_parsers.add_parser("qualify", help="Executes the 'qualify' function for the specified drones")
        ql_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to qualify.")
        ql_parser.add_argument("-a", "--altitude", type=float, required=False, default=2.0,
                               help="Altitude in meters above takeoff at which the course will be completed. "
                                    "Positive for up. Default 2m")

        rc_add_parser = command_parsers.add_parser("rc-add", help="Add drone(s) to the redcross demo")
        rc_add_parser.add_argument("drones", type=str, nargs="+", help="Drones to add.")

        rc_remove_parser = command_parsers.add_parser("rc-rm", help="Remove drone(s) from the redcross demo")
        rc_remove_parser.add_argument("drones", type=str, nargs="+", help="Drones to remove.")

        rc_stage_parser = command_parsers.add_parser("rc-stage", help="Perform a stage with the current drones")
        rc_stage_parser.add_argument("stage", type=int,
                                     help="Which stage to execute. Must be consecutive to the previous stage")
        

        return parser, command_parsers

    async def _load_plugin_commands(self, plugin_name, plugin):
        try:
            self.logger.debug(f"Loading CLI commands for plugin {plugin_name}")
            commands = plugin.cli_commands
            for command_name in commands:
                command = commands[command_name]
                cli_command = f"{plugin.PREFIX}-{command_name}".lower()
                self.logger.debug(f"Inspecting command {command_name}")
                tmp_parser = self.command_parser.add_parser(cli_command)
                for arg in check_cli_command_signatures(command):
                    is_invalid, name, is_list, is_required, accepts_none, base_type, is_kwonly = arg
                    arg_name = name if is_required else f"--{name}"
                    if is_invalid:
                        raise RuntimeError(f"CLI command {command_name} has invalid parameter types for parameter {name}!")
                    if is_list and is_required:
                        tmp_parser.add_argument(arg_name, type=base_type, nargs="+")
                    elif is_list and not is_required:
                        tmp_parser.add_argument(arg_name, type=base_type, nargs="*")
                    else:
                        tmp_parser.add_argument(arg_name, type=base_type)
                    self.logger.debug(f"Added Argument {arg_name}: {base_type, is_list, is_required}")
                self.dynamic_commands[cli_command] = command
        except Exception as e:
            self.logger.warning("Failed to load CLI commands for the plugin!")
            self.logger.debug(repr(e), exc_info=True)
            return e

    async def _unload_plugin_commands(self, plugin_name, plugin):
        # TODO: ALL OF IT
        # TODO: Apparently there isn't a great way to get rid of arguments in argparse, might have to delete the parser
        #  and generate a new one (which should be possible due to plugin list)
        pass

    async def _add_drone_object(self, name, drone):
        output = self.query_one("#output", expect_type=Log)
        status_field = self.query_one("#status", expect_type=VerticalScroll)
        self.logger.debug(f"Adding log pane handlers to {name}")
        drone_handler = TextualLogHandler(output)
        drone_handler.setLevel(logging.INFO)
        drone_handler.setFormatter(pane_formatter)
        drone.add_handler(drone_handler)
        self.logger.debug(f"Adding overview widget for {name}")
        drone_status_widget = DroneOverview(drone, UPDATE_RATE)
        self.drone_widgets[name] = drone_status_widget
        await status_field.mount(drone_status_widget)

    async def _remove_drone_object(self, name):
        try:
            await self.drone_widgets[name].remove()
        except KeyError:
            pass

    async def _cli_awaiter(self, task):
        try:
            if isinstance(task, asyncio.Task):
                await task
        except asyncio.CancelledError:
            pass
        except Exception as e:
            self.logger.error(f"Encountered an exception in a coroutine! See the log for more details")
            self.logger.debug(e, exc_info=True)

    @on(InputWithHistory.Submitted, "#cli")
    async def cli(self, message):
        value = message.value
        message.control.clear()
        tmp = None
        try:
            values = shlex.split(value)
            values[0] = values[0].lower()
            args = self.parser.parse_args(values)
        except ValueError as e:
            self.logger.warning(str(e))
            return
        except ArgumentParserError as e:
            self.logger.error(f"Exception parsing the argument: ")
            self.logger.debug(repr(e), exc_info=True)
            return
        except Exception as e:
            self.logger.warning("Exception during argument parsing!")
            self.logger.debug(repr(e), exc_info=True)
        try:

            command = args.command.lower()
            if command != "kill" or args.drones:
                self._kill_counter = 0

            if command == "connect":
                address = args.drone_address
                if args.drone in DRONE_DICT and not address:
                    address = DRONE_DICT[args.drone]
                elif not address:
                    address = "udp://:14540"
                tmp = asyncio.create_task(self.dm.connect_to_drone(args.drone, args.server_address,
                                                                   args.server_port, address, args.timeout))
            elif command == "disconnect":
                tmp = asyncio.create_task(self.dm.disconnect(args.drones, force=args.force))
            elif command == "arm":
                tmp = asyncio.create_task(self.dm.arm(args.drones, schedule=args.schedule))
            elif command == "disarm":
                tmp = asyncio.create_task(self.dm.disarm(args.drones, schedule=args.schedule))
            elif command == "takeoff":
                tmp = asyncio.create_task(self.dm.takeoff(args.drones, altitude=args.altitude, schedule=args.schedule))
            elif command == "mode":
                tmp = asyncio.create_task(self.dm.change_flightmode(args.drones, args.mode))
            elif command == "fence":
                self.dm.set_fence(args.drones, args.nl, args.nu, args.el, args.eu, args.h)
            elif command == "flyto":
                tmp = asyncio.create_task(self.dm.fly_to(args.drone, local=[args.x, args.y, args.z], yaw=args.yaw,
                                                         tol=args.tolerance, schedule=args.schedule))
            elif command == "flytogps":
                tmp = asyncio.create_task(self.dm.fly_to(args.drone, gps=[args.lat, args.long, args.alt], yaw=args.yaw,
                                                         tol=args.tolerance, schedule=args.schedule))
            elif command == "move":
                tmp = asyncio.create_task(self.dm.move(args.drone, [args.x, args.y, args.z], yaw=args.yaw,
                                                       use_gps=not args.nogps, tol=args.tolerance,
                                                       schedule=args.schedule))
            elif command == "land":
                tmp = asyncio.create_task(self.dm.land(args.drones, schedule=args.schedule))
            elif command == "pause":
                self.dm.pause(args.drones)
            elif command == "resume":
                self.dm.resume(args.drones)
            elif command == "stop":
                tmp = asyncio.create_task(self.dm.action_stop(args.drones))
            elif command == "kill":
                if not args.drones:
                    if self._kill_counter:
                        tmp = asyncio.create_task(self.dm.kill(args.drones))
                    else:
                        self.logger.warning("Are you sure? Enter kill again")
                        self._kill_counter += 1
                else:
                    tmp = asyncio.create_task(self.dm.kill(args.drones))
            elif command == "load":
                tmp = asyncio.create_task(self.dm.load_plugin(args.plugin))
            elif command == "unload":
                tmp = asyncio.create_task(self.dm.unload_plugin(args.plugin))
            elif command == "loaded":
                self.logger.info(f"Currently loaded plugins: {self.dm.currently_loaded_plugins()}")
            elif command == "plugins":
                available_but_not_loaded = [item for item in self.dm.plugin_options()
                                            if item not in self.dm.currently_loaded_plugins()]
                self.logger.info(f"Available plugins to load: {available_but_not_loaded}")
            elif command == "exit" or command in self._exit_aliases:
                exit_task = asyncio.create_task(self.exit())
            elif command in self.dynamic_commands:
                self.logger.debug(f"Performing plugin action {command}")
                func_arguments = vars(args).copy()
                func_arguments.pop("command")
                tmp = asyncio.create_task(self.dynamic_commands[command](**func_arguments))
            elif command == "cam-prep":
                tmp = asyncio.create_task(self.dm.prepare(args.drone))
            elif command == "cam-settings":
                tmp = asyncio.create_task(self.dm.get_settings(args.drone))
            elif command == "cam-photo":
                tmp = asyncio.create_task(self.dm.take_picture(args.drone))
            elif command == "cam-start":
                tmp = asyncio.create_task(self.dm.start_video(args.drone))
            elif command == "cam-stop":
                tmp = asyncio.create_task(self.dm.stop_video(args.drone))
            elif command == "cam-zoom":
                tmp = asyncio.create_task(self.dm.set_zoom(args.drone, args.zoom))
            self.running_tasks.add(tmp)
            self._awaiter_tasks.add(asyncio.create_task(self._cli_awaiter(tmp)))
        except Exception as e:
            self.logger.error("Encountered an exception executing the CLI!")
            self.logger.debug(repr(e), exc_info=True)

    async def action_stop(self, names):
        stop_app = False
        if not names:
            stop_app = True
        results = await self.dm.action_stop(names)
        for i, result in enumerate(results):
            # If one of the drones encounters an excepton
            if isinstance(result, Exception):
                stop_app = False
        if stop_app:
            self.logger.info("All drones stopped, exiting...")
            await asyncio.sleep(2)  # Beauty pause
            await self.exit()

    async def exit(self):
        """ Checks if any drones are armed and exits the app if not."""
        stop_app = True
        try:
            for name in self.dm.drones:
                if self.dm.drones[name].is_armed:
                    stop_app = False
            if stop_app:
                await asyncio.gather(*[self.dm.drones[name].disconnect(force=True) for name in self.dm.drones])
                for task in self.running_tasks:
                    if isinstance(task, asyncio.Task):
                        task.cancel()
                for task in self._awaiter_tasks:
                    if isinstance(task, asyncio.Task):
                        task.cancel()
                await asyncio.sleep(0.2)  # Beauty pause
                self.logger.info("Exiting...")
                await asyncio.sleep(1)  # Beauty pause
                await self.dm.close()
                self.app.exit()
            else:
                self.logger.warning("Can't exit the app with armed drones!")
        except Exception as e:
            self.logger.error("Exception during exit function!")
            self.logger.debug(repr(e), exc_info=True)

    def _schedule_initialization_tasks(self):
        asyncio.create_task(self._logging_setup())

    async def _logging_setup(self):
        output = None
        while output is None:
            try:
                output = self.query_one("#output", expect_type=Log)
            except textual.css.query.NoMatches:
                await asyncio.sleep(0.1)
        handler = TextualLogHandler(output)
        handler.setLevel(logging.INFO)
        handler.setFormatter(pane_formatter)
        self.logger.addHandler(handler)
        self.dm.logger.addHandler(handler)

    def _on_mount(self, event: events.Mount) -> None:
        super()._on_mount(event)
        self.query_one("#output", expect_type=Log).can_focus = False

    def compose(self):
        status_string = ""
        status_string += "Drone Status\n" + DroneOverview.header_string()

        yield Header()
        yield Vertical(
            Horizontal(
                Log(id="output", classes="text"),
                Vertical(
                    VerticalScroll(
                        Static(id="status_header", content=status_string),
                        id="status", classes="text evenvert"),
                    Static(id="usage", classes="text evenvert", content=self.parser.format_help(), markup=False),
                    id="sidebar",
                )
            ),
            InputWithHistory(placeholder="Command line", id="cli")
        )
        yield Footer()
        self._schedule_initialization_tasks()


class DroneApp(App):

    BINDINGS = {
        Binding("s", "cycle_control", "Swap Status/Control"),
    }
    TITLE = "DroneManager"
    MODES = {
        "control": CommandScreen,
        "status": StatusScreen,
    }

    def __init__(self, dm: DroneManager, logger=None):
        self.dm = dm
        if logger is None:
            self.logger = logging.getLogger("App")
            self.logger.setLevel(logging.DEBUG)
            filename = f"app_{datetime.datetime.now()}"
            filename = filename.replace(":", "_").replace(".", "_") + ".log"
            logdir = os.path.abspath("./logs")
            os.makedirs(logdir, exist_ok=True)
            file_handler = logging.FileHandler(os.path.join(logdir, filename))
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(common_formatter)
            self.logger.addHandler(file_handler)
        else:
            self.logger = logger
        self.command_screen: CommandScreen | None = None
        self.status_screen: StatusScreen | None = None
        super().__init__()

    def on_mount(self):
        self.switch_mode("control")

    def action_cycle_control(self):
        self.logger.debug("Switching between control and status screens")
        if self.current_mode == "control":
            self.logger.debug(f"Switching from control to status.")
            self.switch_mode("status")
        elif self.current_mode != "control":
            self.logger.debug("Switching to control")
            self.switch_mode("control")
        else:
            self.logger.debug("No valid target for switching")


def main():
    drone_type = DroneMAVSDK
    drone_manager = DroneManager(drone_type)
    app = DroneApp(drone_manager, logger=drone_manager.logger)
    app.run()

    logging.shutdown()


if __name__ == "__main__":
    main()

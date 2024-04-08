import asyncio
from asyncio.exceptions import TimeoutError
import sys
import argparse
import shlex
from typing import Dict
import numpy as np
import random

import textual.css.query
from textual import on
from textual.app import App
from textual.containers import Horizontal, Vertical
from textual.widgets import Footer, Header, Log, Static

from widgets import InputWithHistory, TextualLogHandler
from drones import Drone, DroneMAVSDK, DummyMAVDrone, parse_address
from betterparser import ArgParser

import logging


# Must start mavsdk_server first, with arguments: mavsdk_server_bin.exe -p <serverport> udp://:<droneport>
# Each mavsdk server can handle exactly one drone and there is no possibility of disconnecting or connecting to another
# drone (unless both drones use the same connection and whichever connected first dies)

DRONE_DICT = {
    "gavin": "udp://192.168.1.35:15565",
    "corran": "udp://192.168.1.36:15566",
    "jaina": "udp://192.168.1.37:14550",
}


class DroneManager(App):
    # TODO: Schedule take-off and land
    # TODO: Put a bunch of the code into their associated widgets and put those into their own files (i.e. cli into the
    #  tweaked input)
    # TODO: Status pane for each drone with much info: positions, velocity, attitude, gps info, battery, "health"
    #  checks, check what else
    # TODO: Handle MAVSDK crashes
    # TODO: Print a pretty usage/command overview thing somewhere.

    # TODO: Safety checks, such as vortex ring state avoidance.

    # TODO: Figure out what stop should do and if we even want it

    # How often the status screen is updated.
    STATUS_REFRESH_RATE = 20

#    BINDINGS = [
#        ("k", "stop", "STOP ALL")
#    ]

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
    width: 98;
}
"""

    def __init__(self, drone_class):
        super().__init__()
        self._drone_class = drone_class
        self.drones: Dict[str, Drone] = {}
        self.running_tasks = set()
        # self.drones acts as the list/manager of connected drones, any function that writes or deletes items should
        # protect those writes/deletes with this lock. Read only functions can ignore it.
        self.drone_lock = asyncio.Lock()
        self._kill_counter = 0  # Require kill all to be entered twice

        self.logger = logging.getLogger("manager")
        self.logger.setLevel(logging.DEBUG)

        self.parser = ArgParser(
            description="Interactive command line interface to connect and control multiple drones")
        subparsers = self.parser.add_subparsers(title="command",
                                                description="Command to execute.", dest="command")
        connect_parser = subparsers.add_parser("connect", help="Connect a drone")
        connect_parser.add_argument("drone", type=str, help="Name for the drone.")
        connect_parser.add_argument("drone_address", type=str, nargs='?',
                                    help="Connection string. Something like udp://:14540")
        connect_parser.add_argument("-sa", "--server_address", type=str, default=None,
                                    help="Address for the mavsdk server. If omitted, a server is started "
                                         "automatically. Use this only if you already have a server for this drone "
                                         "running (for example on another machine). Default None")
        connect_parser.add_argument("-sp", "--server_port", type=int, default=50051,
                                    help="Port for the mavsdk server. Default 50051.")
        connect_parser.add_argument("-t", "--timeout", type=float, default=5, required=False,
                                    help="Timeout in seconds for connection attempts.")
        arm_parser = subparsers.add_parser("arm", help="Arm the named drone(s).")
        arm_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to arm")

        disarm_parser = subparsers.add_parser("disarm", help="Disarm the named drone(s).")
        disarm_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to disarm")

        takeoff_parser = subparsers.add_parser("takeoff", help="Puts the drone(s) into takeoff mode.")
        takeoff_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to take off with.")

        offboard_parser = subparsers.add_parser("mode", help="Change the drone(s) flight mode")
        offboard_parser.add_argument("mode", type=str, help="Target flight mode. Must be one of {}.".format(
            self._drone_class.VALID_FLIGHTMODES))
        offboard_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to change flight mode on.")

        fly_to_parser = subparsers.add_parser("flyto", help="Send the drone to a local coordinate.")
        fly_to_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_to_parser.add_argument("x", type=float, help="Target x coordinate")
        fly_to_parser.add_argument("y", type=float, help="Target y coordinate")
        fly_to_parser.add_argument("z", type=float, help="Target z coordinate")
        fly_to_parser.add_argument("yaw", type=float, nargs="?", default=0.0, help="Target yaw in degrees. Default 0.")
        fly_to_parser.add_argument("-t", "--tolerance", type=float, required=False, default=0.5,
                                   help="Position tolerance")

        fly_to_gps_parser = subparsers.add_parser("flytogps", help="Send the drone to a GPS coordinate")
        fly_to_gps_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_to_gps_parser.add_argument("lat", type=float, help="Target latitude")
        fly_to_gps_parser.add_argument("long", type=float, help="Target longitude")
        fly_to_gps_parser.add_argument("alt", type=float, help="Target altitude (relative to takeoff)")
        fly_to_gps_parser.add_argument("yaw", type=float, nargs="?", default=0.0, help="Target yaw in degrees. "
                                                                                       "Default 0.")
        fly_to_gps_parser.add_argument("-t", "--tolerance", type=float, required=False, default=0.5,
                                       help="Position tolerance")

        fly_circle_parser = subparsers.add_parser("orbit", help="Fly in a circle, facing the center point")
        fly_circle_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_circle_parser.add_argument("radius", type=float, help="Radius of the circle")
        fly_circle_parser.add_argument("vel", type=float, help="Target velocity (negative for opposite direction)")
        fly_circle_parser.add_argument("center_lat", type=float, help="Latitude of the center of the circle")
        fly_circle_parser.add_argument("center_long", type=float, help="Longitude of the center of the circle")
        fly_circle_parser.add_argument("amsl", type=float, help="Altitude in terms of AMSL.")

        land_parser = subparsers.add_parser("land", help="Land the drone")
        land_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to land")

        stop_parser = subparsers.add_parser("stop", help="Stops (i.e. lands) drones. If no drones are listed, "
                                                         "stops all of them and then exits the application")
        stop_parser.add_argument("drones", type=str, nargs="*", help="Drone(s) to stop.")

        kill_parser = subparsers.add_parser("kill", help="Kills (i.e. disarms and stops everything) drones. If no "
                                                         "drones are listed, kills all of them.")
        kill_parser.add_argument("drones", type=str, nargs="*", help="Drone(s) to kill.")

    def run(self, *args, **kwargs):
        super().run(*args, **kwargs)

    @on(InputWithHistory.Submitted, "#cli")
    async def cli(self, message):
        value = message.value
        message.control.clear()
        tmp = None
        try:
            args = self.parser.parse_args(shlex.split(value))
        except ValueError as e:
            self.logger.error(repr(e), exc_info=True)
            return
        try:
            if args.command != "kill" or args.drones:
                self._kill_counter = 0

            if args.command == "connect":
                address = args.drone_address
                if args.drone in DRONE_DICT and not address:
                    address = DRONE_DICT[args.drone]
                elif not address:
                    address = "udp://:14540"
                tmp = asyncio.create_task(self.connect_to_drone(args.drone, args.server_address, args.server_port,
                                                                address, args.timeout), name=args.drone)
            elif args.command == "arm":
                tmp = asyncio.create_task(self.arm(args.drones))
            elif args.command == "disarm":
                tmp = asyncio.create_task(self.disarm(args.drones))
            elif args.command == "takeoff":
                tmp = asyncio.create_task(self.takeoff(args.drones))
            elif args.command == "mode":
                tmp = asyncio.create_task(self.change_flightmode(args.drones, args.mode))
            elif args.command == "flyto":
                tmp = asyncio.create_task(self.fly_to(args.drone, args.x, args.y, args.z, args.yaw, tol=args.tolerance))
            elif args.command == "flytogps":
                tmp = asyncio.create_task(self.fly_to_gps(args.drone, args.lat, args.long, args.alt, args.yaw,
                                                          tol=args.tolerance))
            elif args.command == "orbit":
                tmp = asyncio.create_task(self.orbit(args.drone, args.radius, args.vel, args.center_lat,
                                                     args.center_long, args.amsl))
            elif args.command == "land":
                tmp = asyncio.create_task(self.land(args.drones))
            elif args.command == "stop":
                await self.action_stop(args.drones)
            elif args.command == "kill":
                if not args.drones:
                    if self._kill_counter:
                        await self.kill(args.drones)
                    else:
                        self.logger.warning("Are you sure? Enter kill again")
                        self._kill_counter += 1
                else:
                    tmp = asyncio.create_task(self.kill(args.drones))
            self.running_tasks.add(tmp)
        except Exception as e:
            self.logger.error(repr(e))

    @property
    def used_drone_addrs(self):
        return [drone.drone_addr for drone in self.drones.values()]

    async def connect_to_drone(self,
                               name: str,
                               mavsdk_server_address: str | None,
                               mavsdk_server_port: int,
                               drone_address: str,
                               timeout: float, compid=160):
        _, parsed_addr, parsed_port = parse_address(string=drone_address)
        parsed_connection_string = parse_address(string=drone_address, return_string=True)
        self.logger.info(f"Trying to connect to drone {name} @{parsed_connection_string}")
        async with self.drone_lock:
            try:
                # Ensure that for each drone there is a one-to-one-to-one relation between name, mavsdk port and drone
                if name in self.drones:
                    self.logger.warning(f"A drone called {name} already exists. Each drone must have a unique name.")
                    return False
                if not mavsdk_server_address:
                    used_ports = [drone.server_port for drone in self.drones.values()]
                    used_compids = [drone.compid for drone in self.drones.values()]
                    while mavsdk_server_port in used_ports:
                        mavsdk_server_port = random.randint(10000, 60000)
                    while compid in used_compids:
                        compid += 1
                # Check that we don't already have this drone connected.
                for other_name in self.drones:
                    other_drone = self.drones[other_name]
                    _, other_addr, other_port = parse_address(string=other_drone.drone_addr)
                    if parsed_addr == other_addr and parsed_port == other_port:
                        self.logger.warning(f"{other_name} is already connected to drone with address {drone_address}.")
                        return False
                drone = self._drone_class(name, mavsdk_server_address, mavsdk_server_port, compid=compid)
                connected = await asyncio.wait_for(drone.connect(drone_address), timeout)
                if connected:
                    self.logger.info(f"Connected to {name}!")
                    self.drones[name] = drone
                    output = self.query_one("#output", expect_type=Log)
                    drone_handler = TextualLogHandler(output)
                    drone_handler.setLevel(logging.INFO)
                    drone_handler.setFormatter(logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)s %(name)s - %(message)s',
                                                                 datefmt="%H:%M:%S"))
                    drone.logger.addHandler(drone_handler)
                    return True
                else:
                    self.logger.warning(f"Failed to connect to drone {name}!")
                    self._remove_drone_object(name, drone)
                    return False
            except TimeoutError:
                self.logger.warning(f"Connection attempts to {name} timed out!")
                del drone
                return False

    async def _multiple_drone_action(self, action, names, start_string, *args, **kwargs):
        """

        :param action:
        :param names:
        :param start_string:
        :param success_string: Output string for the success of a single drone. Will have the name of the drone passed
                               to it.
        :param fail_string: Output string for the failure of a single drone. Will have the name of the drone passed
                            to it.
        :return:
        """
        self.logger.info(start_string.format(names))
        try:
            results = await asyncio.gather(*[action(self.drones[name], *args, **kwargs) for name in names],
                                           return_exceptions=True)
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    self.logger.error(f"Drone {names[i]} failed due to {repr(result)}")
        except KeyError:
            self.logger.warning("No drones named {}!".format([name for name in names if name not in self.drones]))
        except Exception as e:
            self.logger.error(repr(e))

    async def arm(self, names):
        await self._multiple_drone_action(self._drone_class.arm, names, "Arming drone(s) {}.")

    async def disarm(self, names):
        await self._multiple_drone_action(self._drone_class.disarm, names, "Disarming drone(s) {}.")

    async def takeoff(self, names):
        await self._multiple_drone_action(self._drone_class.takeoff, names, "Drone(s) {} taking off.")

    async def change_flightmode(self, names, flightmode):
        await self._multiple_drone_action(self._drone_class.change_flight_mode,
                                          names,
                                          "Changing flightmode for drone(s) {} to " + flightmode + ".",
                                          flightmode)

    async def land(self, names):
        await self._multiple_drone_action(self._drone_class.land, names, "Landing drone(s) {}.")

    async def fly_to(self, name, x, y, z, yaw, tol=0.5):
        point = np.array([x, y, z, yaw])
        self.logger.info(f"Queueing move to {point} for {name}.")
        try:
            coro = self.drones[name].fly_to_point(point, tolerance=tol)
            result = self.drones[name].schedule_task(coro)
            await result
        except KeyError:
            self.logger.warning(f"No drone named {name}!")
        except Exception as e:
            self.logger.error(repr(e))

    async def fly_to_gps(self, name, lat, long, alt, yaw, tol=0.5):
        self.logger.info(f"Queuing move to {(lat, long, alt)} for  {name}")
        try:
            coro = self.drones[name].fly_to_gps(lat, long, alt, yaw, tolerance=tol)
            result = self.drones[name].schedule_task(coro)
            await result
        except KeyError:
            self.logger.warning(f"No drone named {name}!")
        except Exception as e:
            self.logger.error(repr(e))

    async def orbit(self, name, radius, velocity, center_lat, center_long, amsl):
        try:
            await self.drones[name].orbit(radius, velocity, center_lat, center_long, amsl)
            self.logger.info(f"{name} flying in a circle.")
        except KeyError:
            self.logger.warning(f"No drone named {name}!")
        except Exception as e:
            self.logger.error(repr(e))

    async def _stop_drone(self, name):
        drone = self.drones[name]
        result = await drone.stop()
        self._remove_drone_object(name, drone)
        return result

    async def _kill_drone(self, name):
        drone = self.drones[name]
        result = await drone.kill()
        self._remove_drone_object(name, drone)
        return result

    def _remove_drone_object(self, name, drone: Drone):
        try:
            self.drones.pop(name)
        except KeyError:
            pass
        del drone.system
        drone.should_stop.set()
        del drone

    async def action_stop(self, names):
        stop_app = False
        if not names:
            stop_app = True
        async with self.drone_lock:
            if not names:
                self.logger.info("Stopping all drones!")
            else:
                self.logger.info(f"Stopping {names}")
            drones_to_stop = names if names else list(self.drones.keys())
            results = await asyncio.gather(*[self._stop_drone(name) for name in drones_to_stop], return_exceptions=True)
            for i, result in enumerate(results):
                # If one of the drones encounters an excepton
                if isinstance(result, Exception):
                    self.logger.critical(f"During stopping, drone {drones_to_stop[i]} encountered an exception "
                                         f"{repr(result)}!")
                    stop_app = False
            if stop_app:
                self.logger.info("All drones stopped, exiting...")
                await asyncio.sleep(2)  # Beauty pause
                self.exit()

    async def kill(self, names):
        async with self.drone_lock:
            if not names:
                self.logger.info("Killing all drones!")
            else:
                self.logger.info(f"Killing {names}")
            drones_to_stop = names if names else list(self.drones.keys())
            await asyncio.gather(*[self._kill_drone(name) for name in drones_to_stop], return_exceptions=True)

    async def update_status(self):
        output = None
        log = None
        while not output and not log:
            try:
                output = self.query_one("#status", expect_type=Static)
            except textual.css.query.NoMatches:
                await asyncio.sleep(0.1)
        try:
            while True:
                status_string = ""
                status_string += "Drone Status\n"
                format_string_drones = "{:<10}   {:>9}   {:>5}   {:>6}   {:>15}   {:>10.7f}   {:>6.3f}   {:>6.3f}   {:>6.3f}"
                format_string_header = "{:<10}   {:>9}   {:>5}   {:>6}   {:>15}   {:>10}   {:>6}   {:>6}   {:>6}"
                header_string = format_string_header.format("Name", "Connected", "Armed", "In-Air", "FlightMode",
                                                            "GPS", "NED", "Vel", "Alt")
                status_string += header_string + "\n"
                status_string += "="*len(header_string) + "\n"
                for name in list(self.drones.keys()):
                    drone = self.drones[name]
                    if len(name) > 10:
                        name = name[:7] + "..."
                    status_string += format_string_drones.format("", "", "", "", "",
                                                                 drone.position_global[0],
                                                                 drone.position_ned[0], drone.velocity[0], 0) + "\n"
                    status_string += format_string_drones.format(str(name), str(drone.is_connected),
                                                                 str(drone.is_armed), str(drone.in_air),
                                                                 str(drone.flightmode), drone.position_global[1],
                                                                 drone.position_ned[1], drone.velocity[1],
                                                                 drone.position_global[3]) + "\n"
                    status_string += format_string_drones.format("", "", "", "", "",
                                                                 drone.position_global[2],
                                                                 drone.position_ned[2], drone.velocity[2], 0) + "\n\n"

                output.update(status_string)
                await asyncio.sleep(1/self.STATUS_REFRESH_RATE)
        except Exception as e:
            self.logger.error(repr(e), exc_info=True)

    def _schedule_initialization_tasks(self):
        asyncio.create_task(self.update_status())
        asyncio.create_task(self._logging_setup())

    async def _logging_setup(self):
        output = None
        while output is None:
            try:
                output = self.query_one("#output", expect_type=Log)
            except textual.css.query.NoMatches:
                await asyncio.sleep(0.1)
        handler = TextualLogHandler(output)
        handler.setLevel(logging.DEBUG)
        handler.setFormatter(logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)s %(name)s - %(message)s',
                                               datefmt="%H:%M:%S"))
        self.logger.addHandler(handler)

    def compose(self):
        yield Header()
        yield Vertical(
            Horizontal(
                Log(id="output", classes="text"),
                Vertical(
                    Static(id="status", classes="text evenvert"),
                    Static(id="usage", classes="text evenvert", renderable=self.parser.format_help()),
                    id="sidebar",
                )
            ),
            InputWithHistory(placeholder="Command line", id="cli")
        )

        yield Footer()
        self._schedule_initialization_tasks()

        # Try to redirect stdout to our logger window to get prints from other modules
        # Redirect all the drone output to our text widget
        # TODO: Do this in a less stupid way. Either use logging or pass a reference to our log thing somehow.
        def decorator(func):
            def inner(inputstr):
                try:
                    self.logger.info(inputstr)
                    return func(inputstr)
                except:
                    return func(inputstr)
            return inner
        sys.stdout.write = decorator(sys.stdout.write)
        sys.stderr.write = decorator(sys.stderr.write)


if __name__ == "__main__":
    start_parser = argparse.ArgumentParser()
    start_parser.add_argument("-d", "--dummy", action="store_true", help="If set, use a dummy drone class")

    start_args = start_parser.parse_args()

    drone_type = DummyMAVDrone if start_args.dummy else DroneMAVSDK
    app = DroneManager(drone_type)
    app.run()

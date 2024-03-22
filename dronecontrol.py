import asyncio
from asyncio.exceptions import TimeoutError
import numpy as np
import sys
import argparse
import shlex
from typing import Dict

import textual.css.query
from textual import on
from textual.app import App
from textual.containers import Horizontal, Vertical
from textual.widgets import Footer, Header, Input, Log, Static

from mavsdk import System

# Must start mavsdk_server first, with arguments: mavsdk_server_bin.exe -p 50040 udp://:14540
# -p port : The port used by the computer to connect to the server
# udp://:port : The address of the drone.
# Each mavsdk server can handle exactly one drone and there is no possibility of disconnecting/reconnecting

MAV_SERVER_BINARY = "mavsdk_server_bin.exe"


# TURNS OUT ARGPARSE IS ARSE, DOESN'T THROW EXCEPTIONS AND JUST QUITS INSTEAD, LMAO
class ArgumentParserError(Exception):
    pass


class ArgParser(argparse.ArgumentParser):
    def error(self, message):
        if "invalid choice" in message:
            raise ValueError(message)
        elif "arguments are required" in message:
            raise ValueError(message)
        elif "unrecognized argument" in message:
            raise ValueError(message)
        else:
            raise ArgumentParserError(message)


class Drone:
    def __init__(self, name, mavsdk_server_address: str | None = None, mavsdk_server_port: int = 50051):
        self.name = name
        self.server_addr = mavsdk_server_address
        self.server_port = mavsdk_server_port
        self.connection_address = None
        self.system = System(mavsdk_server_address=self.server_addr, port=self.server_port)
        self.is_connected = False
        self.is_armed = False

    # TODO: Implement a mission manager/queue thing.
    async def connect(self, connection_address):
        self.connection_address = connection_address
        await self.system.connect(system_address=self.connection_address)
        async for state in self.system.core.connection_state():
            if state.is_connected:
                await self.schedule_update_tasks()
                return True
        return False

    async def schedule_update_tasks(self):
        asyncio.create_task(self._connect_check())
        asyncio.create_task(self._arm_check())

    async def _connect_check(self):
        async for state in self.system.core.connection_state():
            self.is_connected = state.is_connected

    async def _arm_check(self):
        async for arm in self.system.telemetry.armed():
            self.is_armed = arm

    @property
    async def is_offboard(self):
        try:
            offboard_status = await asyncio.wait_for(self.system.offboard.is_active(), .1)
            return offboard_status
        except TimeoutError:
            return False

    async def arm(self):
        if not self.is_armed:
            try:
                await self.system.action.arm()
                return True
            except Exception as e:  # add exception later
                print(e)
        else:
            return False

    async def disarm(self):
        if self.is_armed:
            try:
                await self.system.action.disarm()
                return True
            except Exception as e:  # add exception later
                print(e)
        else:
            return False

    async def offboard(self):
        await self.system.offboard.start()
        return True

    async def fly_to_point(self, point: np.ndarray, tolerance=0.5):
        raise NotImplementedError

    async def fly_circle(self, velocity, radius, angle, direction):
        raise NotImplementedError

    async def land(self):
        raise NotImplementedError

    async def stop(self):
        # land, disarm then stop?
        await self.disarm()
        return True

    async def kill(self):
        raise NotImplementedError


class DroneManager(App):
    # TODO: Convert all the 'action' functions to use _multiple_drones/_single_drone templates, except stop and connect
    # TODO: Figure out how stop should work

    # How often the status screen is updated.
    STATUS_REFRESH_RATE = 5
    # Number of seconds until connection attempts time out (to prevent mavsdk from locking up on wrong addressrd)
    CONNECT_TIMEOUT = 5

#    BINDINGS = [
#        ("k", "stop", "STOP ALL")
#    ]

    CSS = """
.text {
    text-style: bold;
}
    
#status {
    width: 50;
}
"""

    def __init__(self):
        super().__init__()
        self.drones: Dict[str, Drone] = {}
        self.running_tasks = set()
        # self.drones acts as the list/manager of connected drones, any function that writes or deletes items should
        # protect those writes/deletes with this lock. Read only functions can ignore it.
        self.drone_lock = asyncio.Lock()

        self.parser = ArgParser(
            description="Interactive command line interface to connect and control multiple drones")
        subparsers = self.parser.add_subparsers(title="command",
                                                description="Command to execute.", dest="command")
        connect_parser = subparsers.add_parser("connect")
        connect_parser.add_argument("drone", type=str, help="Name for the drone.")
        connect_parser.add_argument("drone_address", type=str, nargs='?', default="udp://:14540",
                                    help="Connection string. Something like udp://:14540")
        connect_parser.add_argument("-sa", "--server_address", type=str, default=None,
                                    help="Address for the mavsdk server. If omitted, a server is started "
                                         "automatically. Use this only if you already have a server for this drone "
                                         "running (for example on another machine). Default None")
        connect_parser.add_argument("-sp", "--server_port", type=int, default=50051,
                                    help="Port for the mavsdk server. Default 50051.")
        arm_parser = subparsers.add_parser("arm")
        arm_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to arm")

        disarm_parser = subparsers.add_parser("disarm")
        disarm_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to arm")

        offboard_parser = subparsers.add_parser("offboard")
        offboard_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to put into offboard mode")

        fly_to_parser = subparsers.add_parser("flyto")
        fly_to_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_to_parser.add_argument("x", type=float, help="Target x coordinate")
        fly_to_parser.add_argument("y", type=float, help="Target y coordinate")
        fly_to_parser.add_argument("z", type=float, help="Target z coordinate")
        fly_to_parser.add_argument("-t", "--tolerance", type=float, required=False, default=0.5,
                                   help="Position tolerance")

        fly_circle_parser = subparsers.add_parser("flycircle")
        fly_circle_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_circle_parser.add_argument("vel", type=float, help="Target velocity")
        fly_circle_parser.add_argument("radius", type=float, help="Radius of the circle")
        fly_circle_parser.add_argument("angle", type=float, help="Angle (?) of the circle")
        fly_circle_parser.add_argument("dir", type=str, choices=["cw", "ccw"],
                                       help="Direction of the circle")

        land_parser = subparsers.add_parser("land")
        land_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to land")

        stop_parser = subparsers.add_parser("stop", help="Stops (i.e. lands) drones. If no drones are listed, "
                                                         "stops all of them and then exits the application")
        stop_parser.add_argument("drones", type=str, nargs="*", help="If ")

    def run(self, *args, **kwargs):
        super().run(*args, **kwargs)

    @on(Input.Submitted, "#cli")
    async def cli(self, message):
        output = self.query_one("#output", expect_type=Log)
        value = message.value
        message.control.clear()
        try:
            args = self.parser.parse_args(shlex.split(value))
        except ValueError as e:
            output.write_line(repr(e))
            return
        try:
            if args.command == "connect":
                asyncio.create_task(self.connect_to_drone(args.drone, args.server_address, args.server_port,
                                                          args.drone_address), name=args.drone)
            if args.command == "arm":
                asyncio.create_task(self.arm(args.drones))
            if args.command == "disarm":
                asyncio.create_task(self.disarm(args.drones))
            if args.command == "offboard":
                asyncio.create_task(self.offboard(args.drones))
            if args.command == "flyto":
                asyncio.create_task(self.fly_to(args.drone, args.x, args.y, args.z, args.tolerance))
            if args.command == "flycircle":
                asyncio.create_task(self.fly_circle(args.drone, args.vel, args.radius, args.angle, args.dir))
            if args.command == "land":
                asyncio.create_task(self.land(args.drones))
            if args.command == "stop":
                await self.action_stop(args.drones)
            # TODO: Kill single drone
            # TODO: Kill all drones
        except Exception as e:
            output.write_line(repr(e))

    async def connect_to_drone(self, name, mavsdk_server_address, mavsdk_server_port, drone_address):
        output = self.query_one("#output", expect_type=Log)
        output.write_line(f"Trying to connect to drone {name}...")
        try:
            async with self.drone_lock:
                if name in self.drones:
                    output.write_line(f"A drone called {name} already exists. Each drone must have a unique name.")
                    return False
                drone = Drone(name, mavsdk_server_address, mavsdk_server_port)
                connected = await asyncio.wait_for(drone.connect(drone_address), self.CONNECT_TIMEOUT)
                if connected:
                    output.write_line(f"Connected to drone {name}!")
                    self.drones[name] = drone
                    return True
                else:
                    output.write_line(f"Failed to connect to drone {name}!")
                    del drone
                    return False
        except TimeoutError:
            output.write_line(f"Connection attempts to {name} timed out!")
            del drone
            return False

    async def _multiple_drone_action(self, action, names, start_string, success_string, fail_string,):
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
        output = self.query_one("#output", expect_type=Log)
        output.write_line(start_string.format(names))
        try:
            results = await asyncio.gather(*[action(self.drones[name]) for name in names], return_exceptions=True)
            for i, result in enumerate(results):
                if not result:
                    output.write_line(fail_string.format(names[i]))
                elif result and not isinstance(result, Exception):
                    output.write_line(success_string.format(names[i]))
                else:
                    output.write_line(f"Drone {names[i]} failed due to {repr(result)}")
        except KeyError:
            output.write_line("No drones named {}!".format([name for name in names if name not in self.drones]))
        except Exception as e:
            output.write_line(repr(e))

    async def arm(self, names):
        await self._multiple_drone_action(Drone.arm, names, "Arming drone(s) {}.", "{} armed!", "{} is already armed!")

    async def disarm(self, names):
        await self._multiple_drone_action(Drone.disarm, names, "Disarming drone(s) {}.", "{} disarmed!",
                                          "{} is already disarmed!")

    async def offboard(self, names):
        await self._multiple_drone_action(Drone.offboard, names, "Offboarding drone(s) {}.", "{} in offboard mode!",
                                          "Offboarding failed for drone {}!")

    async def land(self, names):
        await self._multiple_drone_action(Drone.land, names, "Landing drone(s) {}.", "{} landed.", "{} couldn't land!")

    async def fly_to(self, name, x, y, z, tol=0.5):
        point = np.array([x, y, z])
        output = self.query_one("#output", expect_type=Log)
        try:
            await self.drones[name].fly_to_point(point, tolerance=tol)
            output.write_line(f"Sending {name} to point {point}.")
        except KeyError:
            output.write_line(f"No drone named {name}!")
        except Exception as e:
            output.write_line(repr(e))

    async def fly_circle(self, name, velocity, radius, angle, direction):
        output = self.query_one("#output", expect_type=Log)
        try:
            await self.drones[name].fly_circle(velocity, radius, angle, direction)
            output.write_line(f"{name} flying in a circle.")
        except KeyError:
            output.write_line(f"No drone named {name}!")
        except Exception as e:
            output.write_line(repr(e))

    async def _stop_drone(self, name):
        drone = self.drones[name]
        await drone.stop()
        self.drones.pop(name)
        del drone

    async def action_stop(self, names):
        output = self.query_one("#output", expect_type=Log)
        async with self.drone_lock:
            if not names:
                output.write_line("Stopping all drones!")
            else:
                output.write_line(f"Stopping {names}")
            drones_to_stop = names if names else list(self.drones.keys())
            results = await asyncio.gather(*[self._stop_drone(name) for name in drones_to_stop], return_exceptions=True)
            for i, result in results:
                if isinstance(result, Exception):
                    output.write_line(f"During stopping, drone {drones_to_stop[i]} encountered an exception "
                                      f"{repr(result)}!")
            if not names:
                await asyncio.sleep(2)  # Beauty pause
                self.exit()

    async def update_status(self):
        output = None
        log = None
        while not output and not log:
            try:
                output = self.query_one("#status", expect_type=Static)
                log = self.query_one("#output", expect_type=Log)
            except textual.css.query.NoMatches:
                await asyncio.sleep(0.5)
        try:
            while True:
                status_string = ""
                status_string += "Drone Status\n"
                format_string = "{:<10}    {:>9}   {:>5}  {:>8}"
                header_string = format_string.format("Name", "Connected", "Armed", "Offboard")
                status_string += header_string + "\n"
                status_string += "="*len(header_string) + "\n"
                for name in list(self.drones.keys()):
                    drone = self.drones[name]
                    if len(name) > 10:
                        name = name[:7] + "..."
                    status_string += format_string.format(str(name),
                                                          str(drone.is_connected),
                                                          str(drone.is_armed),
                                                          str(await drone.is_offboard)) + "\n"

                output.update(status_string)
                await asyncio.sleep(1/self.STATUS_REFRESH_RATE)
        except Exception as e:
            log.write_line(repr(e))

    def schedule_background_tasks(self):
        asyncio.create_task(self.update_status())

    def compose(self):
        yield Header()
        yield Vertical(
            Horizontal(
                Log(id="output", classes="text"),
                Static(id="status", classes="text")
            ),
            Input(placeholder="Command line", id="cli")
        )
        # TODO: Add history to input

        yield Footer()
        self.schedule_background_tasks()

        # Try to redirect stdout to our logger window to get prints from other modules
        # Redirect all the drone output to our text widget
        def decorator(func):
            def inner(inputstr):
                try:
                    output = self.query_one("#output", expect_type=Log)
                    output.write_line(inputstr)
                    return func(inputstr)
                except:
                    return func(inputstr)
            return inner
        sys.stdout.write = decorator(sys.stdout.write)


if __name__ == "__main__":
    app = DroneManager()
    app.run()
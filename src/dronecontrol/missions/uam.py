import asyncio
import enum
import collections
from queue import PriorityQueue, Empty
import math
import time

import numpy as np

from dronecontrol.plugins.mission import Mission
from dronecontrol.utils import dist_ned
from dronecontrol.navigation.core import Waypoint, WayPointType

# TODO: Take circling function and put it into drone orbit function
# TODO: Better ready check, should consider position and status of each drone for the calling stage.


class UAMStages(enum.Enum):
    Uninitialized = enum.auto()
    Start = enum.auto()
    SearchSingle = enum.auto()
    SearchGroup = enum.auto()
    POIFound = enum.auto()
    Observation = enum.auto()
    Return = enum.auto()


class FakeBattery:

    TIME_SCALE = 180

    def __init__(self):
        self.level = 1.0  # float from 0 to 1.0
        self.critical_level = 0.33

    @property
    def battery_low(self):
        return self.level <= self.critical_level


class UAMMission(Mission):
    """ Multi-Stage mission for the UAM Demo 2025.

    The demo consists of two stages:T

    """

    def __init__(self, name, dm, logger):
        super().__init__(name, dm, logger)
        self.drones = collections.OrderedDict()

        mission_cli_commands = {
            "reset": self.reset,
            "set": self.set_start,
            "unset": self.set_uninit,
            "singlesearch": self.single_search,
            "rtb": self.rtb,
            "groupsearch": self.group_search,
        }
        self.cli_commands.update(mission_cli_commands)
        self.background_functions = [
            self._stage_managing_function(),
            self._battery_drainer(),
        ]

        # Static parameters for mission definition
        self.n_drones_max = 3
        self.current_stage = UAMStages.Uninitialized
        self.flight_area = [-3.75, 3.75, -1.75, 1.75, 4]
        self.search_space = [-3, 3, -1, 1]
        self.start_positions_y: dict[str, float] = {}
        self.start_position_x = 3.0
        self.start_yaw = 180
        self.yaw_rate = 30      
        self.yaw_tolerance = 3  # In degrees
        self.position_tolerance = 0.25  # In meters
        self.flight_altitude = 3  # in meters, positive for up
        self.poi_position = [-2, 0, -self.flight_altitude]  # in NED, altitude is just for convenient distance check.
        self.update_rate = 5  # Mission state is checked and progressed this often per second.

        # SingleSearch Parameters
        self.single_search_forward_leg = 1  # in meters
        self.pause_between_moves = 0.5  # Wait after every yaw or move command, in seconds

        # Observation Stage Parameters
        self.observation_diameter = 2  # in meters
        self.circling_speed = 0.2  # in m/s
        self._circling_speed_angular = math.pi * 2 / (math.pi*self.observation_diameter / self.circling_speed)
        self.swap_altitude = self.flight_altitude  # The height that the drones will do the swap at.
        self.POI_distance = 2  # Distance from drone to POI to be considered "outside" the observation area
        self.holding_position = [0.5, 0.5, -self.flight_altitude]
        self.departure_angle = 3*math.pi / 2
        self.departure_angle_tolerance = 5 * math.pi / 180
        self.arrival_angle = math.pi / 2

        # Dynamic attributes, each stage must appropriate set these
        self.drone_tasks = set()  # Keeps track of all the stage functions
        self._found_poi = None  # The drone that found the POI, if one found the POI
        self._observing_drone: str | None = None  # The drone that is currently observing the POI
        self.do_swap = False  # If we do the swap or not
        self._reached_departure = False  # If the observing drone has reached the departure position.
        self._stop_circling = False  # If the observing drone should stop circling once it reaches departure area
        self.flying_drones = set()  # List of names
        self.batteries: dict[str, FakeBattery] = {}  # Dict with batteries for each drone.

    async def _stage_managing_function(self):
        # Check the current stage every so often and cancel functions/start new functions when the stage changes
        # We manage the behaviour of the drones through these stages
        old_stage = self.current_stage
        while True:
            try:
                # If we have swapped stage, cancel old tasks and start new ones
                if self.current_stage is not old_stage:
                    # Cancel all current drone tasks
                    for task in self.drone_tasks:
                        if isinstance(task, asyncio.Task):
                            task.cancel()
                    # Tell the drones to maintain their current positions, unless we are in Start or Uninitialized
                    if self.current_stage is not UAMStages.Start or self.current_stage is not UAMStages.Uninitialized:
                        for drone in self.drones:
                            self.dm.drones[drone].fly_to(local=self.dm.drones[drone].position_ned)
                    # Start new drone tasks for the new stage
                    new_tasks = set()
                    if self.current_stage is UAMStages.SearchSingle:
                        new_tasks.add(asyncio.create_task(self._single_search()))
                        new_tasks.add(asyncio.create_task(self._check_found_poi()))
                    elif self.current_stage is UAMStages.POIFound:
                        new_tasks.add(asyncio.create_task(self.poi_found()))
                    elif self.current_stage is UAMStages.Observation:
                        new_tasks.add(asyncio.create_task(self.observation()))
                    elif self.current_stage is UAMStages.Return:
                        new_tasks.add(asyncio.create_task(self._rtb()))
                    elif self.current_stage is UAMStages.SearchGroup:
                        new_tasks.add(asyncio.create_task(self._group_search()))
                        new_tasks.add(asyncio.create_task(self._check_found_poi()))
                    elif self.current_stage is UAMStages.Start:
                        # Compute whatever attributes that depend on the number of drones available.
                        self._init_variables()
                    self.drone_tasks.update(new_tasks)
                    self._running_tasks.update(new_tasks)
                    old_stage = self.current_stage
                await asyncio.sleep(1/self.update_rate)
            except asyncio.CancelledError:
                self.logger.debug("Cancelling stage-managing function")
                return
            except Exception as e:
                self.logger.error("Stage managing function encountered an exception!")
                self.logger.debug(repr(e), exc_info=True)

    def _init_variables(self):
        start_positions = np.linspace(start=self.search_space[2], stop=self.search_space[3], num=len(self.drones))
        for i, name in enumerate(self.drones):
            self.start_positions_y[name] = start_positions[i]

    async def _battery_drainer(self):
        while True:
            try:
                await asyncio.sleep(1/self.update_rate)
                for drone_name in self.drones:
                    if drone_name in self.flying_drones:
                        battery = self.batteries[drone_name]
                        battery.level -= 1 / FakeBattery.TIME_SCALE / self.update_rate
                        if battery.level < battery.critical_level / 2:
                            battery.level = battery.critical_level / 2
                    else:
                        self.batteries[drone_name].level += 1 / FakeBattery.TIME_SCALE / self.update_rate / 2
            except asyncio.CancelledError:
                return
            except Exception as e:
                self.logger.error("Encountered an exception in the battery managing function!")
                self.logger.debug(repr(e), exc_info=True)

    async def _check_found_poi(self):
        # For each drone, check if we are within the search radius of the POI
        # TODO: Implement a better check, instead of distance to POI, compute vision cone and check POI in it
        while True:
            try:
                await asyncio.sleep(1 / self.update_rate)
                for drone in self.drones:
                    current_pos = self.dm.drones[drone].position_ned
                    if dist_ned(current_pos, self.poi_position) < 1:
                        self._found_poi = drone
                        self.current_stage = UAMStages.POIFound
                        self.logger.info(f"{drone} found POI!")
            except asyncio.CancelledError:
                self.logger.debug("Cancelling poi check function")
            except Exception as e:
                self.logger.error(f"The POI checker function encountered an exception!")
                self.logger.debug(repr(e), exc_info=True)
            await asyncio.sleep(1/self.update_rate)

    async def single_search(self):
        self.logger.info("Starting search with single drone!")
        self.current_stage = UAMStages.SearchSingle

    async def _single_search(self):
        # Do the search pattern ( Stage SingleSearch. If we find POI -> Stage POIFound, else RTB)
        flying_drone = list(self.drones.keys())[0]
        assert self.ready()
        self.do_swap = False
        try:
            # Do the thing
            # Arm, takeoff
            armed = await self.dm.arm([flying_drone])
            takeoff = False
            if armed:
                takeoff = await self.dm.takeoff([flying_drone], altitude=self.flight_altitude)
                self.flying_drones.add(flying_drone)
            go = True
            if not armed or not takeoff or isinstance(armed, Exception) or isinstance(takeoff, Exception):
                go = False
            if not go:
                self.logger.warning("Couldn't start the single search pattern due to denied arming or takeoff!")
                return False

            # Do the pattern
            n_steps = math.floor((self.search_space[1] - self.search_space[0]) / self.single_search_forward_leg)
            for repeat in range(1, n_steps):
                x_pos_new = self.search_space[1] - repeat*self.single_search_forward_leg
                if repeat % 2 == 0:
                    y_pos_current = self.search_space[3]
                    y_pos_new = self.search_space[2]
                    side_yaw = -90
                else:
                    y_pos_current = self.search_space[2]
                    y_pos_new = self.search_space[3]
                    side_yaw = 90
                # fly forward
                await self.dm.fly_to(flying_drone,
                                     local=[x_pos_new, y_pos_current, -self.flight_altitude],
                                     yaw=-180, tol=self.position_tolerance, schedule=False)
                await asyncio.sleep(self.pause_between_moves)
                # switch sides
                await self.dm.yaw_to(flying_drone, yaw=side_yaw, yaw_rate=self.yaw_rate, tol=self.yaw_tolerance,
                                     schedule=False, local=[x_pos_new, y_pos_current, -self.flight_altitude])
                await asyncio.sleep(self.pause_between_moves)
                await self.dm.fly_to(flying_drone,
                                     local=[x_pos_new, y_pos_new, -self.flight_altitude],
                                     yaw=side_yaw, tol=self.position_tolerance, schedule=False)
                await asyncio.sleep(self.pause_between_moves)
                await self.dm.yaw_to(flying_drone, yaw=-180, yaw_rate=self.yaw_rate, tol=self.yaw_tolerance,
                                     schedule=False, local=[x_pos_new, y_pos_new, -self.flight_altitude])
                await asyncio.sleep(self.pause_between_moves)
            await asyncio.sleep(3)
            self.logger.info("Reached end of search pattern without finding POI, RTB")
            self.current_stage = UAMStages.Return
        except asyncio.CancelledError:
            self.logger.debug("Cancelling single search function")
        except Exception as e:
            self.logger.error("Encountered an exception in the single search function!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    async def group_search(self):
        self.logger.info("Starting search with group of drones!")
        self.current_stage = UAMStages.SearchGroup

    async def _group_search(self):
        # Do the search pattern (Stage group stage during, then go to POIFound)
        assert self.ready()
        self.do_swap = True
        try:
            armed = await self.dm.arm(self.drones)
            takeoff = [False]
            if all(armed):
                self.flying_drones.update(self.drones.keys())
                takeoff = await self.dm.takeoff(self.drones, altitude=self.flight_altitude)

            go = True
            if (not all(armed) or not all(takeoff) or any([isinstance(s_armed, Exception) for s_armed in armed]) or
                    any([isinstance(s_takeoff, Exception) for s_takeoff in takeoff])):
                go = False
            if not go:
                self.logger.warning("Couldn't arm or takeoff with all drones! Stopping mission")
                tasks = []
                for drone in self.drones:
                    tasks.append(asyncio.create_task(self.dm.land([drone], schedule=True)))
                await asyncio.gather(*tasks)
                self.current_stage = UAMStages.Uninitialized
                self.flying_drones = set()
                return False

            # Do the pattern (i.e. just fly forward)
            end_positions = []
            for drone in self.drones:
                end_positions.append([self.search_space[0], self.start_positions_y[drone], -self.flight_altitude])

            coros = [self.dm.fly_to(drone, local=end_positions[i], tol=self.position_tolerance, yaw=180)
                     for i, drone in enumerate(self.drones)]
            await asyncio.gather(*coros)
            await asyncio.sleep(3)
            # Reached end of pattern without finding POI
            self.current_stage = UAMStages.Return
        except Exception as e:
            self.logger.error("Encountered an exception in the group search function!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    async def poi_found(self):
        # Send the drone that found the POI to a position on the circle, send everybody else back.
        # Start in POI, end in observation
        # Theta is the angle on the circle, with theta = 0 at y = 0, x = observation radius / 2
        # We approach the circle by flying directly to the closest point on it, while pointing at it.
        poi_tasks = []
        try:
            for drone in self.flying_drones:
                if drone == self._found_poi:
                    poi_tasks.append(asyncio.create_task(self._poi_task(drone)))
                else:
                    poi_tasks.append(asyncio.create_task(self._drone_rtb(drone, self.start_positions_y[drone],
                                                                         self.swap_altitude, wait=1)))
            await asyncio.gather(*poi_tasks)
            self._observing_drone = self._found_poi
            self._found_poi = None
            self.current_stage = UAMStages.Observation
        except asyncio.CancelledError:
            self.logger.debug("Cancelling poi_found function")
        except Exception as e:
            self.logger.error("An exception occurred in the POI function!")
            self.logger.debug(repr(e), exc_info=True)

    def _check_distance_to_poi(self, drones: str | list[str]):
        if isinstance(drones, str):
            drones = [drones]
        for drone in drones:
            if dist_ned(self.dm.drones[drone].position_ned, self.poi_position) < self.POI_distance:
                return False
        return True

    async def _poi_task(self, drone):
        # Fly to POI and start circling
        # First we slow down, by sending the current position + vel/2 as setpoint
        cur_pos = self.dm.drones[drone].position_ned
        cur_vel = self.dm.drones[drone].velocity
        target_pos = cur_pos + cur_vel / 2
        target_pos[2] = -self.flight_altitude
        if self.search_space[0] > target_pos[0]:
            target_pos[0] = self.search_space[0]
        elif target_pos[0] > self.search_space[1]:
            target_pos[0] = self.search_space[1]
        if self.search_space[2] > target_pos[1]:
            target_pos[1] = self.search_space[2]
        elif target_pos[1] > self.search_space[3]:
            target_pos[1] = self.search_space[3]
        await self.dm.fly_to(drone, local=target_pos, schedule=False, tol=self.position_tolerance)
        # Wait for other drones to be away from POI before circling
        other_drones = list(self.drones.keys())
        other_drones.remove(drone)
        while not self._check_distance_to_poi(other_drones):
            await asyncio.sleep(1 / self.update_rate)
        # Determine vector from POI center to drone, send drone to that point, wait short beauty pause,
        # then circle.
        theta = self._calculate_circle_angle(drone)
        x_pos, y_pos, target_yaw = self._calculate_xy_yaw(theta)
        await self.dm.yaw_to(drone, yaw=target_yaw, yaw_rate=self.yaw_rate, schedule=False, tol=self.yaw_tolerance)
        await asyncio.sleep(self.pause_between_moves)
        await self.dm.fly_to(drone, local=[x_pos, y_pos, -self.flight_altitude], yaw=target_yaw, schedule=False,
                             tol=self.position_tolerance)
        # Start circling: Have to add this to drone_tasks, so it gets cancelled and replaced with the proper obs task
        circle_task = self._observation_circling(drone)
        self.drone_tasks.add(asyncio.create_task(circle_task))

    async def observation(self):
        # Do the observation stage, with battery swap and everything (Stage observation throughout).
        # There should already be one drone observing the POI, saved in self.observing_drone
        assert self.ready()
        self.logger.info("Starting long-term observation phase!")
        try:
            # Start circling the observation drone
            observe_task = asyncio.create_task(self._observation_circling(self._observing_drone))
            self.drone_tasks.add(observe_task)
            while self.do_swap:  # Only do battery swap when we have multiple drones
                if self.batteries[self._observing_drone].battery_low:
                    self.logger.info("Observing drone battery going low!")
                    # Pick the drone back at base with the highest battery and launch it
                    swap_drone = await self._swap_launch_new_drone()

                    # Fly swap drone to holding area while observing drone keeps circling. Once
                    # observing drone reaches DEPARTURE_ANGLE, observing drone stops circling, swap_drone flies to
                    # ARRIVAL_ANGLE and yaws to POI and observing drone departs. Once observing is some distance away,
                    # start circling swap drone and change labels.
                    # Tell the observation drone to circle to departure angle and then stop
                    self._stop_circling = True
                    # Fly the swap drone to the holding area:
                    move_to_holding_yaw = self._yaw_to_point(swap_drone, self.holding_position)
                    await self.dm.yaw_to(swap_drone, yaw=move_to_holding_yaw, yaw_rate=self.yaw_rate, schedule=False,
                                         tol=self.yaw_tolerance)
                    await self.dm.fly_to(swap_drone, schedule=False, local=self.holding_position,
                                         tol=self.position_tolerance)
                    await asyncio.sleep(self.pause_between_moves)
                    await self.dm.yaw_to(swap_drone, yaw=self.start_yaw, local=self.holding_position,
                                         yaw_rate=self.yaw_rate, schedule=False, tol=self.yaw_tolerance)
                    self.logger.info(f"{swap_drone} reached holding position, waiting on observing drone!")

                    # Wait until both drones are in position
                    while not self._reached_departure:
                        await asyncio.sleep(1/self.update_rate)
                    self.logger.info("Ready to perform swap!")

                    # Fly new drone to arrival position
                    self.logger.info(f"Moving {swap_drone} to arrival position")
                    x, y, observe_yaw = self._calculate_xy_yaw(self.arrival_angle)
                    fly_to_arrival_yaw = self._yaw_to_point(swap_drone, [x, y])
                    await self.dm.yaw_to(swap_drone, yaw=fly_to_arrival_yaw, yaw_rate=self.yaw_rate,
                                         tol=self.yaw_tolerance, local=self.holding_position, schedule=False)
                    await self.dm.fly_to(swap_drone, local=[x, y, -self.flight_altitude], yaw=fly_to_arrival_yaw,
                                         tol=self.position_tolerance)
                    await self.dm.yaw_to(swap_drone, yaw=observe_yaw, yaw_rate=self.yaw_rate, tol=self.yaw_tolerance,
                                         local=[x, y, -self.flight_altitude], schedule=False)

                    # Both drones now watching POI on opposite ends of the POI
                    # Start flying old observing drone back
                    self.logger.info(f"{self._observing_drone} returning to base")
                    rtb_task = asyncio.create_task(self._drone_rtb(self._observing_drone,
                                                                   self.start_positions_y[self._observing_drone],
                                                                   self.swap_altitude))
                    self.drone_tasks.add(rtb_task)

                    # Wait until old observing drone is far enough away
                    while not self._check_distance_to_poi(self._observing_drone):
                        await asyncio.sleep(1/self.update_rate)

                    # Update relevant attributes and start circling new drone
                    self.logger.info(f"Starting observation with {swap_drone}")
                    self._observing_drone = swap_drone
                    self._stop_circling = False
                    self._reached_departure = False
                    observe_task = asyncio.create_task(self._observation_circling(self._observing_drone))
                    self.drone_tasks.add(observe_task)
                    await rtb_task
                await asyncio.sleep(1/self.update_rate)
        except asyncio.CancelledError:
            self.logger.debug("Cancelling observation function")
        except Exception as e:
            self.logger.error("Encountered an exception!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    def _get_drones_by_batterylevel(self):
        """ Note that the stored battery level is negative, as the lowest value is returned by the priority queue."""
        drones_by_batterylevel = PriorityQueue()
        for drone in self.drones:
            if drone == self._observing_drone:
                continue
            drones_by_batterylevel.put((-self.batteries[drone].level, drone))
        return drones_by_batterylevel

    async def _swap_launch_new_drone(self) -> str:
        """ Pick the drone with the highest battery and try to launch it. If that doesn't work, for example if the drone
        doesn't arm, try a different drone."""
        drones_by_batterylevel = self._get_drones_by_batterylevel()
        swap_drone = None
        while not swap_drone:
            try:
                _, candidate = drones_by_batterylevel.get()
            except Empty:
                drones_by_batterylevel = self._get_drones_by_batterylevel()
                _, candidate = drones_by_batterylevel.get()
            self.logger.info(f"Trying to do swap with {candidate}")
            # Launch the new drone
            armed = await self.dm.arm([candidate])
            if armed:
                takeoff = await self.dm.takeoff([candidate], altitude=self.swap_altitude)
                if takeoff:
                    swap_drone = candidate
                else:
                    await self.dm.disarm([candidate])
        self.flying_drones.add(swap_drone)
        return swap_drone

    async def _observation_circling(self, drone):
        # Slowly fly in a circle, pointing inwards, indefinitely until a stopping angle is set, in which case we fly to
        # that angle.
        # Theta is the angle on the circle, with theta = 0 at y = 0
        # We approach the circle by flying directly to the closest point on it, while pointing at it.
        try:
            start_time = time.time()
            start_theta = self._calculate_circle_angle(drone)
            await self.dm.drones[drone].trajectory_follower.deactivate()
            target_pos = np.zeros((3,))
            target_pos[2] = -self.flight_altitude
            while True:
                target_theta = (start_theta + self._circling_speed_angular * (time.time() - start_time)) % (math.pi * 2)
                x_pos, y_pos, target_yaw = self._calculate_xy_yaw(target_theta)
                target_pos[0] = x_pos
                target_pos[1] = y_pos
                waypoint = Waypoint(WayPointType.POS_NED, pos=target_pos, yaw=target_yaw)
                await self.dm.drones[drone].set_setpoint(waypoint)
                if self._stop_circling and abs(target_theta - self.departure_angle) < self.departure_angle_tolerance:
                    self._reached_departure = True
                    return
                await asyncio.sleep(1/self.update_rate)
        except asyncio.CancelledError:
            self.logger.debug("Cancelling observation function")
        except Exception as e:
            self.logger.error("Exception in circling function!")
            self.logger.debug(repr(e), exc_info=True)

    async def rtb(self):
        self.logger.info("Returning to base!")
        self.current_stage = UAMStages.Return

    async def _rtb(self):
        """ Return to base stage.

        Returns all drones still out in the field back to their start positions"""
        assert self.ready()
        try:
            # Do the thing, one after another
            for drone in list(self.flying_drones):
                await self._drone_rtb(drone, self.start_positions_y[drone], self.flight_altitude)
            self.current_stage = UAMStages.Start
        except Exception as e:
            self.logger.error("Encountered an exception during the RTB function!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    async def _drone_rtb(self, drone, start_position_y, altitude, wait=0):
        await asyncio.sleep(wait)
        return_yaw = self._yaw_to_point(drone, [self.start_position_x, start_position_y])
        swap_alt_pos = self.dm.drones[drone].position_ned
        swap_alt_pos[2] = -altitude
        start_hover_pos = [self.start_position_x, start_position_y, -altitude]
        await self.dm.fly_to(drone, local=swap_alt_pos, schedule=False, tol=self.position_tolerance)
        await self.dm.yaw_to(drone, yaw=return_yaw, yaw_rate=self.yaw_rate, local=swap_alt_pos, schedule=False,
                             tol=self.yaw_tolerance)
        await self.dm.fly_to(drone, local=start_hover_pos, yaw=return_yaw, tol=self.position_tolerance, schedule=False)
        await self.dm.yaw_to(drone, yaw=self.start_yaw, yaw_rate=self.yaw_rate, local=start_hover_pos, schedule=False,
                             tol=self.yaw_tolerance)
        await self.dm.land([drone])
        await self.dm.disarm([drone])
        try:
            self.flying_drones.remove(drone)
        except KeyError:
            pass

    def _yaw_to_point(self, drone, position):
        dx = position[0] - self.dm.drones[drone].position_ned[0]
        dy = position[1] - self.dm.drones[drone].position_ned[1]
        theta = (math.atan2(-dy, -dx)) % (math.pi * 2)
        yaw = ((theta + math.pi) % (math.pi * 2)) * 180 / math.pi  # Opposite angle in degrees
        # Have to convert target yaw from 0-360 to -180-180
        if yaw > 180:
            yaw = yaw - 360
        return yaw

    def _calculate_circle_angle(self, drone):
        dx = self.poi_position[0] - self.dm.drones[drone].position_ned[0]
        dy = self.poi_position[1] - self.dm.drones[drone].position_ned[1]
        theta = (math.atan2(-dy, -dx)) % (math.pi * 2)  # x-axis is forward, y-axis is right
        return theta

    def _calculate_xy_yaw(self, theta):
        target_yaw = ((theta + math.pi) % (math.pi * 2)) * 180 / math.pi  # Opposite angle in degrees
        # Have to convert target yaw from 0-360 to -180-180
        if target_yaw > 180.0:
            target_yaw = target_yaw - 360.0
        x_pos = math.cos(theta) * self.observation_diameter / 2 + self.poi_position[0]
        y_pos = math.sin(theta) * self.observation_diameter / 2 + self.poi_position[1]
        return x_pos, y_pos, target_yaw

    async def reset(self):
        self.logger.info("Resetting drones to start positions.")
        for task in self.drone_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()
        self.current_stage = UAMStages.Uninitialized
        #if len(self.drones) != self.n_drones_required:
        #    self.logger.info("Not enough drones to start mission!")
        #    return False
        if not len(self.drones) > 0:
            self.logger.warning("Can't fly a mission without any drones!")
            return False
        # Land all drones in case there are any in the air
        self.logger.info("Landing all drones")
        await asyncio.gather(*[self.dm.land([drone]) for drone in self.drones])
        self.logger.info("Flying all drones to start positions")
        for drone in self.drones:
            # Doesn't work in gazebo because of horrendous position noise and awful flight behaviour
            if not self.dm.drones[drone].is_at_pos([self.start_position_x, self.start_positions_y[drone], 0],
                                                   tolerance=self.position_tolerance):
                await self.dm.arm([drone])
                await self.dm.takeoff([drone], altitude=self.flight_altitude)
                await self._drone_rtb(drone, self.start_positions_y[drone], self.flight_altitude)
        self.current_stage = UAMStages.Start

    async def set_start(self):
        """ Set the current stage to the start stage.

        This should be called/used when the drones are all setup at their starting positions already."""
        self.logger.info("Set to Start stage!")
        self.current_stage = UAMStages.Start

    async def set_uninit(self):
        """ Set the current stage to Uninitialized.

        This function is only useful on a CLI to be able to reset without moving any drones. Some computations are
        performed when the "Start" stage is entered. Setting to Uninitialized and back allows reperforming these.
        """
        self.logger.info("Set to Uninitialized stage!")
        self.current_stage = UAMStages.Uninitialized

    async def status(self):
        self.logger.info(f"Mission {self.PREFIX} of type {self.__class__.__name__}: In stage {self.current_stage.name} "
                         f"with drones {list(self.drones.keys())}. Ready: {self.ready()}")

    def ready(self):
        drones_ready = all([self.mission_ready(drone) for drone in self.drones])
        return drones_ready and self.current_stage is not UAMStages.Uninitialized and len(self.drones) > 0

    async def add_drones(self, names: list[str]):
        self.logger.info(f"Adding drones {names} to mission!")
        self.current_stage = UAMStages.Uninitialized
        if len(self.drones) + len(names) > self.n_drones_max:
            self.logger.warning(f"Can't add this many drones to this kind of mission! These missions allow for at most "
                                f"{self.n_drones_max} drones")
            return False
        for name in names:
            try:
                self.drones[name] = self.dm.drones[name]
                self.dm.set_fence(name, *self.flight_area)
                self.batteries[name] = FakeBattery()
            except KeyError:
                self.logger.error(f"No drone named {name}")
        self._init_variables()

    async def remove_drones(self, names: list[str]):
        for name in names:
            try:
                self.drones.pop(name)
            except KeyError:
                self.logger.warning(f"No drone named {name}")

    def mission_ready(self, drone):
        # Check connection
        # Can't safely disarm in offboard due to landing issue, but do we want to stay armed whole demo?
        # Can't rearm
        return self.dm.drones[drone].is_connected

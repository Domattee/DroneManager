import asyncio
import numpy as np
import functools
from collections import OrderedDict

from abc import ABC, abstractmethod

from dronecontrol.drone import Waypoint, WayPointType, Drone, TrajectoryFollower
from dronecontrol.plugin import Plugin

# TODO: How to do extra CLI commands for each version?


class Formation(ABC):
    def __init__(self, dm, logger):
        self.dm = dm
        self.logger = logger
        self.drones: OrderedDict[str, Drone] = OrderedDict()

    @abstractmethod
    def add_drones(self, drone_names: list[str], *args, **kwargs):
        pass

    @abstractmethod
    def remove_drones(self, drone_names: list[str], *args, **kwargs):
        pass

    @abstractmethod
    def set_positions(self, *args, **kwargs):
        pass

    @abstractmethod
    async def fly_to(self, *args, **kwargs):
        pass

    @abstractmethod
    async def status(self):
        """ Should write some information about the current state of the formation to the logger"""
        pass


class Follower(Formation):
    """ Formation with one designated leader, whom the other drones follow.

    """

    def __init__(self, leader):
        super().__init__()
        self.drones = [leader]
        self.followers = []
        self.leader = leader

    class FollowerDroneTrajectoryFollower(TrajectoryFollower):
        """ Instead of following their own trajectory, drones with this follower in a LeaderFollower formation have
        setpoints based on the position of the leader and their position in the formation.

        """
        # TODO: All of it


class StaticLine(Formation):

    def __init__(self, dm, logger):
        super().__init__(dm, logger)
        self.offset = np.zeros((3,))
        self.offset.fill(np.nan)
        self.minimum_distance = 3
        self.positions: dict[str, np.ndarray] = {}

    @property
    def valid_formation(self):
        return np.sqrt(self.offset.dot(self.offset)) > self.minimum_distance and not np.isnan(self.offset).any()

    @property
    def leader(self):
        return next(iter(self.drones))

    async def start(self):
        self.logger.debug("Starting formations plugin...")

    async def set_positions(self, offset_north: float, offset_east: float, offset_up: float):
        self.offset[0] = offset_north
        self.offset[1] = offset_east
        self.offset[2] = offset_up
        self.logger.info(f"Formation offset set to {self.offset}")

    async def add_drones(self, drone_names: list[str]):
        for name in drone_names:
            assert name in self.dm.drones, f"No drone named {name}"
        for name in drone_names:
            self.drones[name] = self.dm.drones[name]
        await self.status()

    async def remove_drones(self, drone_names: list[str]):
        for name in drone_names:
            try:
                self.drones.pop(name)
            except KeyError:
                pass
        await self.status()

    async def status(self):
        self.logger.info(
            f"Drones {', '.join(self.drones)} currently flying in formation with offset {self.offset}.")
        if not self.valid_formation:
            self.logger.warning(f"Formation currently does not have valid offsets, flying is impossible.")

    async def fly_to(self, lat: float, long: float, amsl: float, yaw: float):
        formation_waypoint = Waypoint(WayPointType.POS_GLOBAL, gps=np.asarray([lat, long, amsl]))
        assert self.valid_formation
        self.logger.info(f"Formation flying to {lat, long, amsl}")
        tasks = []
        for drone in self.drones:
            assert drone in self.dm.drones
        current_leader_gps = Waypoint(WayPointType.POS_GLOBAL, gps=self.dm.drones[self.leader].position_global[:3])
        for i, name in enumerate(self.drones):
            leader_gps_waypoint = formation_waypoint.shift_gps(*self.offset * i)
            current_drone_gps = Waypoint(WayPointType.POS_GLOBAL, gps=self.dm.drones[name].position_global[:3])
            target_drone_gps = current_drone_gps.offset_gps(current_leader_gps, leader_gps_waypoint)
            self.logger.debug(f"Sending {name} to leader_gps {leader_gps_waypoint}, corrected to {target_drone_gps} for {name}")
            tasks.append(self.dm.fly_to_gps(name, lat=target_drone_gps.gps[0], long=target_drone_gps.gps[1],
                                            amsl=target_drone_gps.gps[2], yaw=yaw, schedule=False))
        await asyncio.gather(*tasks)
        self.logger.info("Formation has reached position!")

    async def close(self):
        """ Removes all drones from formation"""
        await self.remove_drones(list(self.drones))


class FormationsPlugin(Plugin):

    PREFIX = "form"
    active_formation: None | Formation = None

    def __init__(self, dm, logger):
        super().__init__(dm, logger)
        self.formations: dict[str, Formation] = {}
        self.available_types = {cls.__name__: cls for cls in Formation.__subclasses__()}
        self.cli_commands = {
            "new": self.new_formation,
            "active": self.change_active,
            "add": self.add_drones,
            "remove": self.remove_drones,
            "status": self.status,
            "flyto": self.fly_to,
            "moveby": self.move_by,
        }
        # TODO: Go through all available types and add their commands to this list as well
        # TODO: Figure out some way to show the user which commands are usable with with formation type

    async def new_formation(self, name: str, formation_type: str):
        assert formation_type in self.available_types
        new_formation_type = self.available_types[formation_type]
        new_formation = new_formation_type(self.dm, self.logger)
        self.active_formation = new_formation
        self.formations[name] = new_formation

    async def change_active(self, name: str):
        self.active_formation = self.formations[name]

    @functools.wraps(active_formation.add_drones)
    async def add_drones(self, drone_names: list[str], *args, **kwargs):
        self.active_formation.add_drones(drone_names, *args, **kwargs)

    @functools.wraps(active_formation.remove_drones)
    async def remove_drones(self, drone_names: list[str], *args, **kwargs):
        self.active_formation.remove_drones(drone_names, *args, **kwargs)

    async def status(self):
        for formation in self.formations:
            self.logger.info(f"Status of formation {formation}:")
            await self.formations[formation].status()

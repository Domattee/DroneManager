import asyncio
import numpy as np
import time
import pickle
from concurrent.futures import ProcessPoolExecutor

from dronecontrol.navigation.core import PathGenerator, Waypoint, WayPointType
from dronecontrol.navigation.GMP3 import GMP3, GMP3Config


class GMP3Generator(PathGenerator):

    # TODO: GMP3 currently doesn't run with less than 2 obstacles
    # TODO: Altitude and yaw. Both are currently just set immediately when we get a new target

    WAYPOINT_TYPES = {WayPointType.POS_VEL_NED}
    CAN_DO_GPS = False

    def __init__(self, drone, dt, logger):
        super().__init__(drone, logger, waypoint_type=WayPointType.POS_VEL_NED)
        self.GMP3_PARAMS = {
            "maxit": 100,
            "alpha": 0.8,
            "wdamp": 1,
            "delta": 0.01,
            "vx_max": 0.2,
            "vy_max": 0.2,
            "Q11": 1.0,
            "Q22": 1.0,
            "Q12": 0.01,
            "dt": dt,
            "x_max": 4.5,
            "y_max": 1.5,
            "obstacles": [
                (2.5, 1, 1),
                (0, -0.5, 1.25),
                (-2.5, 1, 1.111)
            ],
        }
        self.config = GMP3Config(**self.GMP3_PARAMS)
        self.gmp3 = GMP3(self.config)
        self.waypoints = None
        self.valid_path = False
        self.start_time = None
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized path generator {self.__class__.__name__}:\n   {attr_string}")

    async def create_path(self):
        try:
            self.logger.info("Calculating path...")
            cur_x, cur_y, _ = self.drone.position_ned
            target_x, target_y, _ = self.target_position.pos
            with ProcessPoolExecutor(max_workers=1) as executor:
                self.waypoints = await asyncio.get_running_loop().run_in_executor(executor, _calculate_path, cur_x,
                                                                                  cur_y, target_x, target_y, self.gmp3)
            valid = True
            for waypoint in self.waypoints:
                t, x, y, xdot, ydot = waypoint
                if not self.drone.check_waypoint(Waypoint(WayPointType.POS_VEL_NED,
                                                 pos=np.asarray([x, y, self.target_position.pos[2]]),
                                                 vel=np.asarray([xdot, ydot, 0]),
                                                 yaw=self.target_position.yaw)):
                    self.logger.debug(f"Generated waypoint {waypoint} is invalid")
                    valid = False
                    break
            if valid:
                self.logger.info("Found path!")
                self.logger.debug(f"Generated {len(self.waypoints)} waypoints: {self.waypoints}")
                self.start_time = time.time_ns()/1e9
                self.valid_path = True
                return True
            else:
                self.logger.warning(f"No valid path, generated trajectory violates waypoint constraints "
                                    f"(probably fence)!")
                self.valid_path = False
                return False
        except Exception as e:
            self.logger.error("Encountered an exception!")
            self.logger.debug(repr(e), exc_info=True)
            self.valid_path = False
            return False

    def next(self) -> Waypoint | None:
        if not self.valid_path:
            return None
        current_waypoint = None
        for wp in self.waypoints:
            if time.time_ns()/1e9 <= self.start_time + wp[0]:
                current_waypoint = wp
                break
        if current_waypoint is None:
            return None
        t, x, y, xdot, ydot = current_waypoint
        waypoint = Waypoint(WayPointType.POS_VEL_NED,
                            pos=np.asarray([x, y, self.target_position.pos[2]]),
                            vel=np.asarray([xdot, ydot, 0]),
                            yaw=self.target_position.yaw)
        return waypoint


def _calculate_path(cur_x, cur_y, target_x, target_y, gmp3):
    gmp3.calculate((cur_x, cur_y), (target_x, target_y))
    ts = gmp3.t
    xs = gmp3.x
    ys = gmp3.y
    xdots = gmp3.xdot
    ydots = gmp3.ydot
    waypoints = list(zip(ts, xs, ys, xdots, ydots))
    with open("waypoints.dump", "wb") as f:
        pickle.dump(waypoints, f)
    return waypoints

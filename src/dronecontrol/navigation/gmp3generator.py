import asyncio
import numpy as np
import time
import pickle
from concurrent.futures import ProcessPoolExecutor

from dronecontrol.navigation.core import PathGenerator, Waypoint, WayPointType
from dronecontrol.navigation.GMP3 import GMP3, GMP3Config


class GMP3Generator(PathGenerator):

    # TODO: GMP3 currently doesn't run with less than 2 obstacles
    # TODO: Yaw. Currently just set immediately when we get a new target

    WAYPOINT_TYPES = {WayPointType.POS_VEL_NED}
    CAN_DO_GPS = False

    def __init__(self, drone, dt, logger):
        super().__init__(drone, logger, waypoint_type=WayPointType.POS_VEL_NED)
        if self.drone.fence is not None:
            x_max = self.drone.fence.bounding_box[1]
            x_min = self.drone.fence.bounding_box[0]
            y_max = self.drone.fence.bounding_box[3]
            y_min = self.drone.fence.bounding_box[2]
            z_max = self.drone.fence.bounding_box[5]
            z_min = self.drone.fence.bounding_box[4]
        else:
            x_max = 20
            x_min = -20
            y_max = 20
            y_min = -20
            z_max = 0
            z_min = -5
        self.GMP3_PARAMS = {
            "maxit": 100,
            "alpha": 0.8,
            "wdamp": 1,
            "delta": 0.01,
            "vx_max": 1,
            "vy_max": 1,
            "vz_max": 1,
            "Q11": 1.0,
            "Q22": 0.8,
            "Q33": 0.001,
            "Q12": 0.0,
            "Q13": 0.0,
            "Q23": 0.0,
            "dt": dt,
            "x_max": x_max,
            "x_min": x_min,
            "y_max": y_max,
            "y_min": y_min,
            "z_max": z_max,
            "z_min": z_min,
            "obstacles": [
                (-10, 10, -5, 5 + drone.config.size/2),
                (-20, 25, -5, 5 + drone.config.size/2),
                (-35, 35, -5, 5 + drone.config.size/2)
            ],
        }
        self.config = GMP3Config(**self.GMP3_PARAMS)
        self.gmp3 = GMP3(self.config)
        self.waypoints = None
        self.valid_path = False
        self.start_time = None
        self.cur_wp = -1
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized path generator {self.__class__.__name__}:\n   {attr_string}")

    async def create_path(self):
        try:
            self.logger.info("Calculating path...")
            cur_x, cur_y, cur_z = self.drone.position_ned
            target_x, target_y, target_z = self.target_position.pos
            with ProcessPoolExecutor(max_workers=1) as executor:
                calc_task = asyncio.get_running_loop().run_in_executor(executor, _calculate_path, cur_x, cur_y,
                                                                       cur_z, target_x, target_y, target_z, self.gmp3)
                self.drone.current_action_tasks.add(calc_task)
                self.waypoints = await calc_task
            valid = True
            for waypoint in self.waypoints:
                t, x, y, z, xdot, ydot, zdot = waypoint
                if not self.drone.check_waypoint(Waypoint(WayPointType.POS_VEL_NED,
                                                 pos=np.asarray([x, y, z]),
                                                 vel=np.asarray([xdot, ydot, zdot]),
                                                 yaw=self.target_position.yaw)):
                    self.logger.debug(f"Generated waypoint {waypoint} is invalid")
                    valid = False
                    break
            self.logger.debug(f"Generated {len(self.waypoints)} waypoints: {self.waypoints}")
            if valid:
                self.logger.info("Found path!")
                self.start_time = time.time_ns()/1e9
                self.valid_path = True
                self.cur_wp = 0
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
        #while self.cur_wp < len(self.waypoints):
        #    _, x, y, z, xdot, ydot, zdot = self.waypoints[self.cur_wp]
        #    self.cur_wp += 1
        #    waypoint = Waypoint(WayPointType.POS_VEL_NED, pos=np.asarray([x, y, z]),
        #                        vel=np.asarray([xdot, ydot, zdot]), yaw=self.target_position.yaw)
        #    return waypoint
        #return None

        current_waypoint = None
        for wp in self.waypoints:
            if time.time_ns()/1e9 <= self.start_time + wp[0]:
                current_waypoint = wp
                break
        if current_waypoint is None:
            return None
        t, x, y, z, xdot, ydot, zdot = current_waypoint
        waypoint = Waypoint(WayPointType.POS_VEL_NED,
                            pos=np.asarray([x, y, z]),
                            vel=np.asarray([xdot, ydot, zdot]),
                            yaw=self.target_position.yaw)
        return waypoint


def _calculate_path(cur_x, cur_y, cur_z, target_x, target_y, target_z, gmp3):
    gmp3.calculate((cur_x, cur_y, cur_z), (target_x, target_y, target_z))
    ts = gmp3.t
    xs = gmp3.x
    ys = gmp3.y
    zs = gmp3.z
    xdots = gmp3.xdot
    ydots = gmp3.ydot
    zdots = gmp3.zdot
    waypoints = list(zip(ts, xs, ys, zs, xdots, ydots, zdots))
    with open("waypoints.dump", "wb") as f:
        pickle.dump(waypoints, f)
    return waypoints

import time

import ruckig

from dronecontrol.navigation.core import PathFollower, WayPointType, Waypoint
import dronecontrol


# TODO: Yaw. Ruckig doesn't like the -180 180 swap, can probably rely on drone parameters


class RuckigOfflineFollower(PathFollower):

    CAN_DO_GPS = False

    SETPOINT_TYPES = {WayPointType.POS_VEL_ACC_NED}

    WAYPOINT_TYPES = {WayPointType.POS_NED,
                      WayPointType.POS_VEL_NED,
                      WayPointType.POS_VEL_ACC_NED}

    def __init__(self, drone: "dronecontrol.drone.Drone", logger, dt: float, setpoint_type,
                 max_vel = 10.0, max_acc = 2.0, max_jerk = 1.0,
                 max_v_vel = 1.0, max_v_acc = 0.5, max_v_jerk = 1.0):
        super().__init__(drone, logger, dt, setpoint_type)
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized trajectory follower {self.__class__.__name__}:\n   {attr_string}")
        self.planner = None
        self.planner_input = None
        self.planner_output = None
        self.max_velocity = [max_vel, max_vel, max_v_vel]
        self.max_acceleration = [max_acc, max_acc, max_v_acc]
        self.max_jerk = [max_jerk, max_jerk, max_v_jerk]

        self.traj_start_time = None

    def activate(self):
        self.planner = ruckig.Ruckig(3)
        self.planner_input = ruckig.InputParameter(3)
        self.planner_output = ruckig.Trajectory(3)
        self.planner_input.max_velocity = self.max_velocity
        self.planner_input.max_acceleration = self.max_acceleration
        self.planner_input.max_jerk = self.max_jerk
        super().activate()

    async def deactivate(self):
        del self.planner_input
        self.planner_input = None
        del self.planner_output
        self.planner_output = None
        del self.planner
        self.planner = None
        await super().deactivate()

    def get_next_waypoint(self) -> bool:
        return (self.current_waypoint is None or
                self.drone.is_at_pos(self.current_waypoint.pos)
                and self.drone.is_at_heading(self.current_waypoint.yaw))

    async def set_setpoint(self, waypoint):
        # If we don't have a trajectory, create one
        if self._is_waypoint_new:
            # TODO: fence
            self.planner_input.current_position = self.drone.position_ned
            self.planner_input.current_velocity = self.drone.velocity
            self.planner_input.current_acceleration = [0, 0, 0]  # TODO: Drone doesn't report acceleration, maybe use target acc and just assume we hit it?
            self.planner_input.target_position = waypoint.pos
            if waypoint.type in [WayPointType.POS_VEL_NED, WayPointType.POS_VEL_ACC_NED]:
                self.planner_input.target_velocity = waypoint.vel
            if waypoint.type == WayPointType.POS_VEL_ACC_NED:
                self.planner_input.target_acceleration = waypoint.acc
            res = self.planner.calculate(self.planner_input, self.planner_output)
            if res == ruckig.Result.Working:
                self.logger.debug("Generated trajectory...")
                self.traj_start_time = time.time()
            else:
                self.logger.warning("Path follower is failing to produce setpoints, switching to 'HOLD' and deactivating")
                await self.drone.change_flight_mode("hold")
                await self.deactivate()
        # Set setpoint
        cur_time = time.time()-self.traj_start_time
        pos, vel, acc = self.planner_output.at_time(cur_time)
        setpoint = Waypoint(WayPointType.POS_VEL_ACC_NED, pos=pos, vel=vel, acc=acc, yaw=waypoint.yaw)
        await self.drone.set_setpoint(setpoint)


class RuckigOnlineFollower(PathFollower):
    # TODO: Doesn't work right atm, find out why

    CAN_DO_GPS = False

    SETPOINT_TYPES = {WayPointType.POS_VEL_ACC_NED}

    WAYPOINT_TYPES = {WayPointType.POS_NED,
                      WayPointType.POS_VEL_NED,
                      WayPointType.POS_VEL_ACC_NED}

    def __init__(self, drone: "dronecontrol.drone.Drone", logger, dt: float, setpoint_type,
                 max_vel = 10.0, max_acc = 2.0, max_jerk = 1.0,
                 max_v_vel = 1.0, max_v_acc = 0.5, max_v_jerk = 1.0):
        super().__init__(drone, logger, dt, setpoint_type)
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized trajectory follower {self.__class__.__name__}:\n   {attr_string}")
        self.planner = None
        self.planner_input = None
        self.planner_output = None
        self.max_velocity = [max_vel, max_vel, max_v_vel]
        self.max_acceleration = [max_acc, max_acc, max_v_acc]
        self.max_jerk = [max_jerk, max_jerk, max_v_jerk]

        self.error_count = 0
        self.max_error_count = 5

    def activate(self):
        self.planner = ruckig.Ruckig(3)
        self.planner_input = ruckig.InputParameter(3)
        self.planner_output = ruckig.OutputParameter(3)
        self.planner_input.max_velocity = self.max_velocity
        self.planner_input.max_acceleration = self.max_acceleration
        self.planner_input.max_jerk = self.max_jerk
        super().activate()

    async def deactivate(self):
        del self.planner_input
        self.planner_input = None
        del self.planner_output
        self.planner_output = None
        del self.planner
        self.planner = None
        await super().deactivate()

    def get_next_waypoint(self) -> bool:
        return (self.current_waypoint is None or
                self.drone.is_at_pos(self.current_waypoint.pos)
                and self.drone.is_at_heading(self.current_waypoint.yaw))

    async def set_setpoint(self, waypoint):
        # TODO: yaw + fence
        self.planner_input.current_position = self.drone.position_ned
        self.planner_input.current_velocity = self.drone.velocity
        self.planner_input.current_acceleration = self.planner_output.new_acceleration  # Don't get acceleration from drone so use last target acceleration as value
        self.planner_input.target_position = waypoint.pos
        if waypoint.type in [WayPointType.POS_VEL_NED, WayPointType.POS_VEL_ACC_NED]:
            self.planner_input.target_velocity = waypoint.vel
        if waypoint.type == WayPointType.POS_VEL_ACC_NED:
            self.planner_input.target_acceleration = waypoint.acc
        res = self.planner.update(self.planner_input, self.planner_output)
        if res is ruckig.Result.Working:
            pos = self.planner_output.new_position
            vel = self.planner_output.new_velocity
            acc = self.planner_output.new_acceleration
            yaw = waypoint.yaw
            setpoint = Waypoint(WayPointType.POS_VEL_ACC_NED, pos=pos, vel=vel, acc=acc, yaw=yaw)
            await self.drone.set_setpoint(setpoint)
        else:
            # TODO: Come up with something better to do here
            self.logger.warning("Path follower is failing to produce setpoints, switching to 'HOLD' and deactivating")
            await self.drone.change_flight_mode("hold")
            await self.deactivate()



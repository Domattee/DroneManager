from dronecontrol.navigation.core import PathFollower, WayPointType, Waypoint
import dronecontrol

import ruckig

# TODO: Yaw
# TODO: Live updates struggle with position latency

class RuckigFollower(PathFollower):

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
        self.planner_input = ruckig.InputParameter(3)
        self.planner_output = ruckig.OutputParameter(3)
        self.planner_input.max_velocity = [max_vel, max_vel, max_v_vel]
        self.planner_input.max_acceleration = [max_acc, max_acc, max_v_acc]
        self.planner_input.max_jerk = [max_jerk, max_jerk, max_v_jerk]

    def activate(self):
        super().activate()
        self.planner = ruckig.Ruckig(3, self.dt)

    async def deactivate(self):
        await super().deactivate()
        del self.planner_input
        del self.planner_output
        del self.planner

    def get_next_waypoint(self) -> bool:
        return (self.current_waypoint is None or
                self.drone.is_at_pos(self.current_waypoint.pos)
                and self.drone.is_at_heading(self.current_waypoint.yaw))

    async def set_setpoint(self, waypoint):
        # TODO: Yaw + fence
        self.planner_input.current_position = self.drone.position_ned
        self.planner_input.current_velocity = self.drone.velocity
        self.planner_input.current_acceleration = self.planner_output.new_acceleration  # Don't get acceleration from drone so use last target acceleration as value
        self.planner_input.target_position = waypoint.pos
        if waypoint.type in [WayPointType.POS_VEL_NED, WayPointType.POS_VEL_ACC_NED]:
            self.planner_input.target_velocity = waypoint.vel
        if waypoint.type == WayPointType.POS_VEL_ACC_NED:
            self.planner_input.target_acceleration = waypoint.acc
        res = self.planner.update(self.planner_input, self.planner_output)
        if res == ruckig.Result.Working:
            pos = self.planner_output.new_position
            vel = self.planner_output.new_velocity
            acc = self.planner_output.new_acceleration
            setpoint = Waypoint(WayPointType.POS_VEL_ACC_NED, pos=pos, vel=vel, acc=acc, yaw=waypoint.yaw)
            await self.drone.set_setpoint(setpoint)
        else:
            pass
            # TODO: Error handling?
            # TODO: If errors are mostly numerical instability, adding a small epsilon and going again might help


from dronecontrol.navigation.core import TrajectoryFollower, WayPointType, Waypoint
import dronecontrol


class TimFollower(TrajectoryFollower):

    CAN_DO_GPS = False

    SETPOINT_TYPES = {WayPointType.POS_VEL_ACC_NED}

    WAYPOINT_TYPES = {WayPointType.POS_NED,
                      WayPointType.POS_VEL_NED,
                      WayPointType.POS_VEL_ACC_NED}

    def __init__(self, drone: "dronecontrol.drone.Drone", logger, dt: float, setpoint_type):
        super().__init__(drone, logger, dt, setpoint_type)
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized trajectory follower {self.__class__.__name__}:\n   {attr_string}")

    def get_next_waypoint(self) -> bool:
        return (self.current_waypoint is None or
                self.drone.is_at_pos(self.current_waypoint.pos)
                and self.drone.is_at_heading(self.current_waypoint.yaw))

    async def set_setpoint(self, waypoint):
        # TODO: Tims script
        pass

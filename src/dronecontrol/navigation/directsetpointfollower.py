from dronecontrol.navigation.core import PathFollower, WayPointType
import dronecontrol


class DirectSetpointFollower(PathFollower):

    CAN_DO_GPS = True

    SETPOINT_TYPES = {WayPointType.POS_NED,
                      WayPointType.POS_GLOBAL,
                      WayPointType.VEL_NED,
                      WayPointType.POS_VEL_NED,
                      WayPointType.POS_VEL_ACC_NED}

    WAYPOINT_TYPES = {WayPointType.POS_NED,
                      WayPointType.POS_GLOBAL,
                      WayPointType.VEL_NED,
                      WayPointType.POS_VEL_NED,
                      WayPointType.POS_VEL_ACC_NED}

    def __init__(self, drone: "dronecontrol.drone.Drone", logger, dt, setpoint_type):
        super().__init__(drone, logger, dt, setpoint_type)
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized trajectory follower {self.__class__.__name__}:\n   {attr_string}")

    def get_next_waypoint(self) -> bool:
        return True

    async def set_setpoint(self, waypoint):
        await self.drone.set_setpoint(waypoint)

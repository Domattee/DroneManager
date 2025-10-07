import dronecontrol
from dronecontrol.navigation.core import PathGenerator, WayPointType


class DirectTargetGenerator(PathGenerator):
    """ Simply sends the target waypoint as static setpoints.
    """

    CAN_DO_GPS = True
    WAYPOINT_TYPES = {WayPointType.POS_NED, WayPointType.POS_GLOBAL}

    def __init__(self, drone: "dronecontrol.drone.Drone", logger, waypoint_type):
        super().__init__(drone, logger=logger, waypoint_type=waypoint_type)
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized trajectory generator {self.__class__.__name__}:\n   {attr_string}")

    async def create_path(self):
        return True

    def next(self):
        """ Should return None if the generator isn't ready to produce waypoints yet."""
        return self.target_position

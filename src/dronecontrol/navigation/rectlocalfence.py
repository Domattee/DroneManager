import numpy as np

from dronecontrol.navigation.core import Fence, Waypoint, WayPointType


class RectLocalFence(Fence):
    """ Class for rectangular fences in the local coordinate frame.

    Works by defining five limits: north upper and lower, east upper and lower, height. Waypoints will only be accepted
    if they use local (NED) coordinates and lie within the box between these five limits.
    The north lower limit should be lower than north upper, and the same for east.
    Note that height should be positive.
    """
    def __init__(self, north_lower, north_upper, east_lower, east_upper, down_lower, down_upper):
        super().__init__()
        assert north_lower < north_upper and east_lower < east_upper and down_lower < down_upper, \
            "Lower fence limits must be less than the upper ones!"
        self.north_lower = north_lower
        self.north_upper = north_upper
        self.east_lower = east_lower
        self.east_upper = east_upper
        self.down_lower = down_lower
        self.down_upper = down_upper

    def check_waypoint_compatible(self, point: Waypoint):
        if self.active and point.type in [WayPointType.POS_NED, WayPointType.POS_VEL_NED, WayPointType.POS_VEL_ACC_NED]:
            coord_north, coord_east, coord_down = point.pos
            if (self.north_lower < coord_north < self.north_upper
                    and self.east_lower < coord_east < self.east_upper
                    and self.down_lower < coord_down < self.down_upper):
                return True
        return False

    @property
    def bounding_box(self) -> np.ndarray:
        return np.asarray([self.north_lower, self.north_upper, self.east_lower, self.east_upper, self.down_lower, self.down_upper])

    def __str__(self):
        return (f"{self.__class__.__name__}, with limits N {self.north_lower, self.north_upper}, "
                f"E {self.east_lower, self.east_upper} and D {self.down_lower, self.down_upper}")

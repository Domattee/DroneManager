import numpy as np
import math

from dronecontrol.navigation.core import Fence, Waypoint, WayPointType


class RectLocalFence(Fence):
    """ Class for rectangular fences in the local coordinate frame.

    Works by defining yis limits: One upper and lower limit for each axis. Waypoints will only be accepted
    if they use local (NED) coordinates and lie within the box between these limits.
    Lower limits but be smaller than upper limits.
    Safety level is used only by the controller functionality.
    """
    def __init__(self, north_lower, north_upper, east_lower, east_upper, down_lower, down_upper, safety_level = 0):
        super().__init__()
        assert north_lower < north_upper and east_lower < east_upper and down_lower < down_upper, \
            "Lower fence limits must be less than the upper ones!"
        assert 0 <= safety_level <= 5 , "Safety Level must be between 0 and 5 and int"
        self.north_lower = north_lower
        self.north_upper = north_upper
        self.east_lower = east_lower
        self.east_upper = east_upper
        self.down_lower = down_lower
        self.down_upper = down_upper
        self.safety_level = int(safety_level)

    def check_waypoint_compatible(self, point: Waypoint):
        if self.active and point.type in [WayPointType.POS_NED, WayPointType.POS_VEL_NED, WayPointType.POS_VEL_ACC_NED]:
            coord_north, coord_east, coord_down = point.pos
            if (self.north_lower < coord_north < self.north_upper
                    and self.east_lower < coord_east < self.east_upper
                    and self.down_lower < coord_down < self.down_upper):
                return True
        return False

    def controller_safety(self, drone, forward_input, right_input, vertical_input, yaw_input, *args, **kwargs):
        # Common parameters for all axes
        margin = 1.0  # Safety margin in meters
        dt = 1.0 / drone.position_update_rate  # Time step for prediction

        # --- STEP 0: Scale Inputs to Physical Velocities (m/s) ---
        # We use the config limits to map the [-1, 1] stick inputs to real world speeds.

        # Horizontal (Symmetric)
        max_h_vel = drone.drone_params.max_h_vel
        if max_h_vel is None:
            max_h_vel = drone.config.max_h_vel
        V_Body_X = forward_input * max_h_vel
        V_Body_Y = right_input * max_h_vel

        # Vertical (Asymmetric: Up vs Down limits)
        # Note: In NED, Negative is Up.
        if vertical_input < 0:  # Ascending
            max_v_speed = drone.drone_params.max_up_vel
            if max_v_speed is None:
                max_v_speed = drone.config.max_up_vel
        else:  # Descending
            max_v_speed = drone.drone_params.max_down_vel
            if max_v_speed is None:
                max_v_speed = drone.config.max_down_vel

        V_Body_Z = vertical_input * max_v_speed

        # --- Horizontal Clamping (N-E Axes via Vector Rotation) ---

        yaw_deg = drone.attitude[2]
        yaw_rad = math.radians(yaw_deg)
        pos_ned = drone.position_ned[:2]  # [N, E]

        V_Body_desired_H = np.array([V_Body_X, V_Body_Y])

        # 1. Body Frame Velocity to NED Frame Velocity
        R_body_to_ned = np.array([
            [math.cos(yaw_rad), -math.sin(yaw_rad)],
            [math.sin(yaw_rad), math.cos(yaw_rad)]
        ])
        V_NED_desired_H = R_body_to_ned @ V_Body_desired_H
        V_NED_clamped_H = np.copy(V_NED_desired_H)

        # 2. Check and Clamp N-E Axes

        N_pos = pos_ned[0]
        V_N = V_NED_desired_H[0]
        N_lower = drone.fence.north_lower + margin
        N_upper = drone.fence.north_upper - margin

        E_pos = pos_ned[1]
        V_E = V_NED_desired_H[1]
        E_lower = drone.fence.east_lower + margin
        E_upper = drone.fence.east_upper - margin

        V_NED_clamped_H[0] = _clamp_axis(N_lower, N_upper, V_N, N_pos, dt)
        V_NED_clamped_H[1] = _clamp_axis(E_lower, E_upper, V_E, E_pos, dt)

        # 3. Clamped NED Velocity back to Body Frame Joystick Inputs
        R_ned_to_body = R_body_to_ned.T
        V_Body_clamped_H = R_ned_to_body @ V_NED_clamped_H

        # --- STEP 4a: Normalize Horizontal Back to [-1, 1] ---
        # We divide by the max speed to get back to the normalized joystick range
        if max_h_vel > 0:
            forward_input = V_Body_clamped_H[0] / max_h_vel
            right_input = V_Body_clamped_H[1] / max_h_vel
        else:
            forward_input = 0.0
            right_input = 0.0

        # --- Vertical Clamping (1D NED Check, Consistent Logic) ---

        D_pos = drone.position_ned[2]
        V_D = V_Body_Z  # Raw input [-1.0, 1.0] used as desired D velocity (negative = ascend)

        # D_lower is the ceiling (min D value), D_upper is the floor (max D value)
        D_lower = drone.fence.down_lower + margin
        D_upper = drone.fence.down_upper - margin

        V_D_clamped = V_D

        # Check Ceiling limit (Descend, V_D < 0)
        if V_D > 0 and D_pos + V_D * dt <= D_lower:
            # Clamp V_D to the speed that just reaches the ceiling limit
            V_D_clamped = min(0.0, (D_lower - D_pos) / dt)

        # Check Floor limit (Ascend, V_D > 0)
        elif V_D < 0 and D_pos + V_D * dt >= D_upper:
            # Clamp V_D to the speed that just reaches the floor limit
            V_D_clamped = max(0.0, (D_upper - D_pos) / dt)

        vertical_input = V_D_clamped / max_v_speed
        # --- FENCE LOGIC END ---

        if self.safety_level == 4:
            forward_input *= 0.5
            right_input *= 0.5
            vertical_input *= 0.5
            yaw_input *= 0.5
        elif self.safety_level == 5:
            forward_input *= 0.3
            right_input *= 0.3
            yaw_input *= 0.3
            vertical_input *= 0.4

        return forward_input, right_input, vertical_input, yaw_input

    @property
    def bounding_box(self) -> np.ndarray:
        return np.asarray([self.north_lower, self.north_upper, self.east_lower, self.east_upper, self.down_lower, self.down_upper])

    def __str__(self):
        return (f"{self.__class__.__name__}, with limits N {self.north_lower, self.north_upper}, "
                f"E {self.east_lower, self.east_upper} and D {self.down_lower, self.down_upper}, "
                f"Safety Level: {self.safety_level}")


def _clamp_axis(lower_limit, upper_limit, raw_input, drone_position, dt):
    if raw_input > 0 and drone_position + raw_input * dt >= upper_limit:
        clamped_input = max(0.0, (upper_limit - drone_position) / dt)
    elif raw_input < 0 and drone_position + raw_input * dt <= lower_limit:
        clamped_input = min(0.0, (lower_limit - drone_position) / dt)
    else:
        clamped_input = raw_input
    return clamped_input
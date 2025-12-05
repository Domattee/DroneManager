""" Plugin for using controllers and joysticks to control drones with DM

"""
import asyncio
import math
import numpy as np
from collections.abc import Callable

import pygame

from dronecontrol.drone import FlightMode
from dronecontrol.plugin import Plugin
from dronecontrol.utils import coroutine_awaiter


DEFAULT_FREQUENCY = 50


class InputMapping:
    """ Map actions to controller axis"""
    # TODO: UI for keybinds

    thrust_axis: int = None
    yaw_axis: int = None
    forward_axis: int = None
    right_axis: int = None
    arm_button: int = None
    disarm_button: int = None
    land_button: int = None
    takeoff_button: int = None
    control_button: int = None
    arm_hold_duration: float = 1.0  # How long the thrust/yaw stick should be held down left/right for disarm/arm. Not implemented at the moment.

    extra_button_inputs: dict[int, set[Callable]] = {}
    # Functions in this dict are called every time their button is pressed. They should not take any arguments. These
    # should be simple functions, not coroutines. They should also complete very quickly. Note that no check is
    # performed to see if the drone is controllable!

    extra_axis_inputs: dict[Callable, list[int]] = {}
    # Functions in this dict are called every timestep in the control loop. They should take one argument,
    # corresponding to the output of the axis. These should be simple functions, not coroutines. They should also
    # complete very quickly. Note that no check is performed to see if the drone is controllable!

    @classmethod
    def add_method_to_button(cls, button: int, method: Callable):
        if button in cls.extra_button_inputs:
            cls.extra_button_inputs[button].add(method)
        else:
            cls.extra_button_inputs[button] = {method}

    @classmethod
    def add_axis_method(cls, method: Callable, axes: list[int]):
        cls.extra_axis_inputs[method] = axes

    @classmethod
    def remove_method_from_button(cls, button: int, method: Callable):
        try:
            cls.extra_button_inputs[button].remove(method)
        except KeyError:
            pass

    @classmethod
    def remove_axis_method(cls, method: Callable):
        try:
            cls.extra_axis_inputs.pop(method)
        except KeyError:
            pass


class PS4Mapping(InputMapping):

    thrust_axis = -1        # Left stick down
    yaw_axis = 0            # Left stick right
    forward_axis = -3       # Right stick down
    right_axis = 2          # Right stick right
    arm_button = 0          # X
    disarm_button = 1       # Circle
    land_button = 12        # D-Pad Down
    takeoff_button = 11     # D-Pad up
    control_button = 5      # PS Button

    # All buttons:
    # X: 0
    # Circle: 1
    # Square: 2
    # Triangle: 3
    # Share button: 4
    # PS button: 5
    # Options button: 6
    # Pressing left stick: 7
    # Pressing right stick: 8
    # LB: 9
    # RB: 10
    # D-Pad up: 11
    # D-Pad down: 12
    # D-Pad left: 13
    # D-pad right: 14
    # Touch pad press: 15

    # All Axis. Positive direction, zero at neutral, except for triggers, which are at -1 when fully released.
    # Left Stick right: 0
    # Left Stick down: 1
    # Right Stick right: 2
    # Right Stick down: 3
    # Left Trigger: 4
    # Right Trigger: 5


class ControllerPlugin(Plugin):
    """
    """

    PREFIX = "control"

    def __init__(self, dm, logger, name, auto_set = False, auto_drone = False):
        """

        """
        super().__init__(dm, logger, name)
        self.background_functions = [
            self._event_processor(),
            self._control_loop(),
        ]
        pygame.init()
        pygame.joystick.init()
        self.controller: pygame.joystick.JoystickType | None = None
        self.cli_commands = {
            "check": self._check_controllers,
            "set": self.add_controller,
            "unset": self.remove_controller,
            "drone": self.set_drone,
            "status": self.status,
        }
        self._relevant_events = [pygame.JOYAXISMOTION,
                                 pygame.JOYBUTTONUP,
                                 pygame.JOYBUTTONDOWN,
                                 pygame.JOYHATMOTION,
                                 pygame.JOYBALLMOTION,
                                 pygame.JOYDEVICEADDED,
                                 pygame.JOYDEVICEREMOVED]
        self._frequency = DEFAULT_FREQUENCY
        self._control_frequency = DEFAULT_FREQUENCY
        self._drone_name: str | None = None
        self._in_control = False
        self._mapping: InputMapping | None = None

        self.print_button_axis_ids = False
        # Set this to True to log the IDs of any button presses or axis motions. Useful for development.
        # Axis motions can generate a lot of logs!

        self.dm.add_remove_func(self._drone_disconnected_callback)

        self.auto_set = auto_set  # If True and there is exactly one controller connected, connect to it automatically
        self.auto_drone = auto_drone  # If True and there is exactly one drone connected, control it automatically.

        self._disconnected = False  # True if we were connected to a controller and lost it unexpectedly


    async def add_controller(self, dev_id: int):
        """ Set which controller to use, matching the ID from `check`.

        """
        if dev_id >= pygame.joystick.get_count():
            self.logger.warning(f"No controller option {dev_id}, see control-check")
            return False
        await self.remove_controller()
        control_good = False
        self.controller = pygame.joystick.Joystick(dev_id)
        if self.controller.get_name() == "PS4 Controller":
            self._mapping = PS4Mapping
            control_good = True
        else:
            self.logger.warning(f"No mapping for {self.controller.get_name()}")
        if control_good:
            self.controller.init()
            self.logger.info(f"Connected to controller {self.controller.get_name()}, using {self._mapping.__name__}")
            await asyncio.sleep(0.1)
            self.controller.rumble(0.5, 0.5, 1)
        return True

    async def remove_controller(self):
        """ Remove the current controller. """
        if self.controller is not None:
            self.logger.debug("Disconnecting from controller")
            controller = self.controller
            self.controller = None
            controller.quit()

    async def status(self):
        """ Log current configuration of the controller plugin. """
        self.logger.info(f"Drone: {self._drone_name}. Control {self._in_control}. Controller: {self.controller}")

    async def set_drone(self, drone: str):
        """ Set which drone is controlled by the controller. """
        if drone not in self.dm.drones:
            self.logger.warning(f"No drone named {drone}")
            return False
        if self._in_control:
            self.logger.warning("Can't swap drones while in control of another drone!")
            return False
        self._drone_name = drone
        self._control_frequency = self.dm.drones[drone].position_update_rate
        self.logger.info(f"Now set for drone {self._drone_name}")
        return True

    async def _check_controllers(self):
        """ Check available controllers.
        """
        new_line = "\n"
        self.logger.info(f"Detected controllers: {[f'{i}: {pygame.joystick.Joystick(i).get_name()}{new_line}' for i in range(pygame.joystick.get_count())]}")

    async def _event_processor(self):
        self._disconnected = False
        while True:
            try:
                for event in pygame.event.get():
                    if event.type in self._relevant_events:
                        event_dict = event.dict
                        if "instance_id" in event_dict and event_dict["instance_id"] == self.controller.get_instance_id():
                            if event.type == pygame.JOYBUTTONDOWN:
                                self._process_button_press(event_dict["button"])
                            elif event.type == pygame.JOYBUTTONUP:
                                pass
                                # Could maybe do long-press type stuff
                                #self.logger.info(f"Released button {event_dict["button"]}")
                            elif event.type == pygame.JOYAXISMOTION:
                                pass  # Axis motion is handled in the control loop
                            elif event.type == pygame.JOYDEVICEREMOVED:
                                if event_dict["instance_id"] == self.controller.get_instance_id():
                                    self.logger.warning("Controller disconnected!")
                                    self._disconnected = True
                                    await self._release_control()
                                    await self.remove_controller()
                            if self.print_button_axis_ids:
                                self.logger.info(f"{event.type}, {event_dict}")
                        else:
                            if event.type == pygame.JOYDEVICEADDED and self._disconnected:
                                self._disconnected = False
                                await self.add_controller(event_dict["device_index"])
                                await self._take_control()
                            else:
                                self.logger.debug(f"{event.type}, {event_dict}")
            except Exception as e:
                self.logger.warning("Exception processing controller event!")
                self.logger.debug(repr(e), exc_info=True)
            await asyncio.sleep(1/self._frequency)

    def _process_button_press(self, button):
        action = None
        can_do_actions = self._in_control and self._drone_name is not None and self.dm.drones[self._drone_name].is_connected
        toggle_control = False
        if button == self._mapping.control_button:
            self.logger.info("Trying to toggling drone control...")
            if self._in_control:
                action = self._release_control()
            else:
                action = self._take_control()
            toggle_control = True
        elif button == self._mapping.arm_button:
            self.logger.debug("Arm button pressed")
            action = self.dm.arm(self._drone_name)
        elif button == self._mapping.disarm_button:
            self.logger.debug("Disarm button pressed")
            action = self.dm.disarm(self._drone_name)
        elif button == self._mapping.land_button:
            self.logger.debug("Land button pressed")
            action = self.dm.land(self._drone_name)
        elif button == self._mapping.takeoff_button:
            self.logger.debug("Takeoff button pressed")
            action = self.dm.takeoff(self._drone_name)
        else:
            self.logger.info(f"Pressed unbound button {button}")

        # Log information for user about current control state
        if action is not None and not can_do_actions and not toggle_control:
            if not self._in_control:
                self.logger.info("Received control inputs, but not in control of drone!")
                return False
            if self._drone_name is None:
                self.logger.info("Received control inputs, but not set to control any drone!")
            if self._drone_name not in self.dm.drones:
                self.logger.warning(f"Set to control drone with name {self._drone_name}, which is not connected!")
                return False
            if not self.dm.drones[self._drone_name].is_connected:
                self.logger.warning("No connection to drone!")
                return False

        # Do the action if we have an action and either can do it, or are toggling control (which is checked separately)
        if action is not None and self._drone_name is not None and (can_do_actions or toggle_control):
            # Cancel anything the drone might be doing
            self.dm.drones[self._drone_name].clear_queue()
            self.dm.drones[self._drone_name].cancel_action()
            action_task = asyncio.create_task(action)
            action_awaiter = coroutine_awaiter(action_task, self.logger)
            self._running_tasks.add(action_task)
            self._running_tasks.add(action_awaiter)

        # Also perform whatever other actions are bound to this key
        if button in self._mapping.extra_button_inputs:
            for func in self._mapping.extra_button_inputs[button]:
                try:
                    func()
                except Exception as e:
                    self.logger.warning(f"Encountered an exception processing function {func} for button {button}")
                    self.logger.debug(repr(e), exc_info=True)

        return True

    async def _take_control(self):
        # Put the drone into offboard if not already in it, and disable the path follower if it is active
        # Check that we have a drone and are connected to it
        if self._drone_name is None:
            self.logger.warning("Can't take control without knowing which drone to control!")
            return
        if not self.dm.drones[self._drone_name].is_connected:
            self.logger.warning("Can't take control of disconnected drones!")
            return

        await self.dm.drones[self._drone_name].manual_control_position()

        self.logger.info(f"Took control of {self._drone_name}")
        self._in_control = True

    async def _release_control(self):
        # Put the drone into HOLD mode
        self._in_control = False
        await self.dm.change_flightmode(self._drone_name, "hold")
        self.logger.info(f"Released control of {self._drone_name}")

    async def _drone_disconnected_callback(self, name):
        # If our drone got disconnected
        if name == self._drone_name:
            self.logger.info("The drone we were controlling was disconnected.")
            self._drone_name = None
            self._in_control = False

    def _clamp_axis(self, lower_limit, upper_limit, raw_input, drone_position, dt):
        # TODO: Account for actual drone velocity?
        if raw_input > 0 and drone_position + raw_input * dt >= upper_limit:
            clamped_input = max(0.0, (upper_limit - drone_position) / dt)
        elif raw_input < 0 and drone_position + raw_input * dt <= lower_limit:
            clamped_input = min(0.0, (lower_limit - drone_position) / dt)
        else:
            clamped_input = raw_input
        return clamped_input

    async def _control_loop(self):
        """ Take controller inputs to handle motion by setting velocity setpoints.

        Actions are performed as soon as the button press is detected, but continuous inputs, such as sticks, are
        updated here at self._control_frequency."""
        # TODO: Usual arm and disarm with moving stick bottom right and bottom left
        while True:
            try:
                await asyncio.sleep(1/self._control_frequency)
                # If auto_drone is True and there is one drone in drone manager, control it automatically
                if self.auto_drone and self._drone_name is None and len(self.dm.drones) == 1:
                    drone_name, = self.dm.drones
                    add_task = asyncio.create_task(self.set_drone(drone_name))
                    add_wait_task = coroutine_awaiter(add_task, self.logger)
                    self._running_tasks.add(add_task)
                    self._running_tasks.add(add_wait_task)

                # If auto_set is True and there is exactly one controller available, use it automatically
                if self.auto_set and self.controller is None and not self._disconnected and pygame.joystick.get_count() == 1:
                    set_task = asyncio.create_task(self.add_controller(0))
                    set_wait_task = coroutine_awaiter(set_task, self.logger)
                    self._running_tasks.add(set_task)
                    self._running_tasks.add(set_wait_task)

                # Process inputs
                if self._in_control and self._drone_name is not None:
                    # drone_config = self._drone_config
                    SAFETY_LEVEL = 0
                    vertical_input = self.stick_response(self._mapping.thrust_axis)
                    yaw_input = self.stick_response(self._mapping.yaw_axis)
                    right_input = self.stick_response(self._mapping.right_axis)
                    forward_input = self.stick_response(self._mapping.forward_axis)

                    # TODO: If we are landed and disarmed, left stick down and to the right will arm
                    # Should have to hold for 1 second, prevent any inputs until ... stick is centered / allow only vertical until we are in air?
                    #hold_counter_limit = self._mapping.arm_hold_duration * self._control_frequency

                    # TODO: If we are landed and armed, left stick down and to the left will disarm
                    # Should have to hold for 1 second, block any inputs except stick down or up while landed and armed?

                    # If we have non-zero inputs, and we aren't in the appropriate mode, put us into appropriate mode
                    if abs(vertical_input) > 0.01 or abs(yaw_input) > 0.01 or abs(right_input) > 0.01 or abs(forward_input) > 0.01:
                        if self.dm.drones[self._drone_name].flightmode != FlightMode.POSCTL:
                            await self.dm.drones[self._drone_name].manual_control_position()

                    # --- FENCE LOGIC START ---
                    # TODO: Should probably move this into fence, in a function that checks if motion would carry drone out of fence?
                    # Basically: Instead of just checking waypoints, also add a function that would predict if the drone would be carried out of the fence by a given speed.
                    # This moves the responsibility for this to the fence class, which is more future proof if we add irregular fences.
                    drone = self.dm.drones[self._drone_name]
                    if drone.fence is not None and drone.is_connected:
                        try:
                    # --- Initialization ---
                            fence = drone.fence
                            SAFETY_LEVEL = fence.safety_level
                            
                            # Common parameters for all axes
                            margin = 1.0  # Safety margin in meters
                            dt = 1.0 / self._control_frequency # Time step for prediction
                            
                            # Store raw inputs
                            V_Body_X = forward_input
                            V_Body_Y = right_input
                            V_Body_Z = vertical_input # Raw vertical input [-1.0, 1.0]

                            # --- Horizontal Clamping (N-E Axes via Vector Rotation) ---
                            
                            yaw_deg = drone.attitude[2]
                            yaw_rad = math.radians(yaw_deg)
                            pos_ned = drone.position_ned[:2] # [N, E]
                            
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

                            V_NED_clamped_H[0] = self._clamp_axis(N_lower, N_upper, V_N, N_pos, dt)
                            V_NED_clamped_H[1] = self._clamp_axis(E_lower, E_upper, V_E, E_pos, dt)

                            # 3. Clamped NED Velocity back to Body Frame Joystick Inputs
                            R_ned_to_body = R_body_to_ned.T
                            V_Body_clamped_H = R_ned_to_body @ V_NED_clamped_H
                            
                            forward_input = V_Body_clamped_H[0]
                            right_input = V_Body_clamped_H[1]
                            
                            # --- Vertical Clamping (1D NED Check, Consistent Logic) ---
                            
                            D_pos = drone.position_ned[2]
                            V_D = V_Body_Z # Raw input [-1.0, 1.0] used as desired D velocity (negative = ascend)
                            
                            # D_lower is the ceiling (min D value), D_upper is the floor (max D value)
                            D_lower = drone.fence.down_lower + margin 
                            D_upper = drone.fence.down_upper - margin 

                            V_D_clamped = V_D

                            # Check Ceiling limit (Ascend, V_D < 0)
                            #self.logger.info(f"V_D: {V_D}")
                            #self.logger.info(f"D_pos: {D_pos}")
                            #self.logger.info(f"dt: {dt}")
                            #self.logger.info(f"D_lower: {D_lower}")
                            if V_D > 0 and D_pos + V_D * dt <= D_lower:
                                # Clamp V_D to the speed that just reaches the ceiling limit
                                V_D_clamped = min(0.0, (D_lower - D_pos) / dt)
                                #if V_D_clamped == 0.0:
                                #    self.logger.warning("Fence: Upward motion clamped (Ceiling limit).")

                            # Check Floor limit (Descend, V_D > 0)
                            elif V_D < 0 and D_pos + V_D * dt >= D_upper:
                                # Clamp V_D to the speed that just reaches the floor limit
                                V_D_clamped = max(0.0, (D_upper - D_pos) / dt)
                                #if V_D_clamped == 0.0:
                                #    self.logger.warning("Fence: Downward motion clamped (Floor limit).")

                            # Update the final vertical input signal
                            vertical_input = V_D_clamped

                        except AttributeError:
                            # Catches errors if the fence object is missing expected attributes
                            self.logger.warning("Fence constraints could not be applied due to missing fence attributes.")
                        except Exception as e:
                            self.logger.error(f"Error applying vertical fence logic: {repr(e)}")
                    # --- FENCE LOGIC END ---

                    if SAFETY_LEVEL == 4:
                        forward_input *= 0.5
                        right_input *= 0.5
                        vertical_input = (vertical_input*0.5 + 1) / 2
                        yaw_input *= 0.5
                    elif SAFETY_LEVEL == 5:
                        forward_input *= 0.3
                        right_input *= 0.3
                        yaw_input *= 0.3
                        vertical_input = (vertical_input*0.4 + 1) / 2
                    else:
                        vertical_input = (vertical_input + 1) / 2  # Scale from -1/1 to 0/1


                    # TODO: Vertical input treatment is inconsistent. Should be postive for down, but instead is positive for up.
                    # Should invert this here and then unify the code above.
                    await self.dm.drones[self._drone_name].set_manual_control_input(forward_input, right_input, vertical_input, yaw_input)

                    # Also perform whatever other functions are bound to any other axis
                    for func, axes in self._mapping.extra_axis_inputs.items():
                        values = [self.controller.get_axis(axis) for axis in axes]
                        try:
                            func(values)
                        except Exception as e:
                            self.logger.warning(
                                f"Encountered an exception processing function {func} controllers")
                            self.logger.debug(repr(e), exc_info=True)
            except Exception as e:
                self.logger.warning("Error in control loop for joystick!")
                self.logger.debug(repr(e), exc_info=True)

    def stick_response(self, axis: int) -> float:
        """ Linear stick response with -5 to 5% dead zone.

        Axis should be the joystick axis. A negative number means that the response is inverted. """
        value = self.controller.get_axis(abs(axis))
        dz = 0.1
        if abs(value) < dz:
            return 0.0
        elif value > 0:
            value = (value - dz) / (1 - dz)
        else:
            value = (value + dz) / (1 - dz)
        return value * math.copysign(1, axis)

    async def close(self):
        if self.controller is not None:
            self.controller.quit()
        pygame.quit()
        await super().close()
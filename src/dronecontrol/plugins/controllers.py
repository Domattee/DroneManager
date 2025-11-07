""" Plugin for using controllers and joysticks to control drones with DM

"""
import asyncio
import math
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

    def __init__(self, dm, logger, name):
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
            "disconnect": self.remove_controller,
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

    async def add_controller(self, dev_id: int):
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
        if self.controller is not None:
            self.logger.debug("Disconnecting from controller")
            controller = self.controller
            self.controller = None
            controller.quit()

    async def status(self):
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
        new_line = "\n"
        self.logger.info(f"Detected controllers: {[f'{i}: {pygame.joystick.Joystick(i).get_name()}{new_line}' for i in range(pygame.joystick.get_count())]}")

    async def _event_processor(self):
        disconnected = False
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
                                    disconnected = True
                                    await self._release_control()
                                    await self.remove_controller()
                            if self.print_button_axis_ids:
                                self.logger.info(f"{event.type}, {event_dict}")
                        else:
                            if event.type == pygame.JOYDEVICEADDED and disconnected:
                                disconnected = False
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
            if not self.dm.drones[self._drone_name].is_connected:
                self.logger.warning("No connection to drone!")

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

    async def _control_loop(self):
        """ Take controller inputs to handle motion by setting velocity setpoints.

        Actions are performed as soon as the button press is detected, but continuous inputs, such as sticks, are
        updated here at self._control_frequency. """
        # TODO: Usual arm and disarm with moving stick bottom right and bottom left
        while True:
            try:
                await asyncio.sleep(1/self._control_frequency)
                if self._in_control and self._drone_name is not None:
                    # drone_config = self._drone_config
                    vertical_input = self.stick_response(self._mapping.thrust_axis)
                    yaw_input = self.stick_response(self._mapping.yaw_axis)
                    right_input = self.stick_response(self._mapping.right_axis)
                    forward_input = self.stick_response(self._mapping.forward_axis)

                    # If we have non-zero inputs, and we aren't in the appropriate mode, put us into appropriate mode
                    if abs(vertical_input) > 0.01 or abs(yaw_input) > 0.01 or abs(right_input) > 0.01 or abs(forward_input) > 0.01:
                        if self.dm.drones[self._drone_name].flightmode != FlightMode.POSCTL:
                            await self.dm.drones[self._drone_name].manual_control_position()

                    vertical_input = (vertical_input + 1) / 2  # Scale from -1/1 to 0/1

                    self.logger.debug(forward_input, right_input, vertical_input, yaw_input)
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
        if abs(value) < 0.05:
            value = 0.0
        return value * math.copysign(1, axis)

    async def close(self):
        if self.controller is not None:
            self.controller.quit()
        pygame.quit()
        await super().close()
"""Plugin for using controllers and joysticks to control drones with DM.

# TODO: Extensive documentation
# TODO: Description of actions and controller objects, how bindings and physical controllers work with them, etc...
"""
import asyncio
import copy
import enum
from collections import OrderedDict
import math
import pygame
import time
from typing import Callable

from dronemanager.drone import FlightMode
from dronemanager.plugin import Plugin
from dronemanager.utils import coroutine_awaiter


DEFAULT_FREQUENCY: float = 100
"""Default frequency for the control loop."""

DEFAULT_HOLD_DURATION: float = 2
"""Default duration for which a button must be pressed continuously to count as a "long press"."""


class ActionInputType(enum.Enum):
    """Possible input types for actions."""
    Button = enum.auto()
    Axis = enum.auto()


class Action:
    """Actions, i.e. functions, which can or should be bound to controls.

    There are two types of actions:

    * Button actions: Called when their bound button is activated. Button combinations are not allowed.
    * Axis actions: Called every step in the control loop. The functions registered to these actions should take a
      drone name and a list of axes as their arguments.

    An action can be inverted. For axis actions, this means that the axes inputs are inverted. For button actions, this
    means that the button must be pressed for a certain amount of time, and the action is executed when the button is
    released.

    Note that axis actions are not called if any of their input axes are unbound.
    """
    def __init__(self, label: str, input_type: ActionInputType, input_id: int | list[int | None] | None = None, func: Callable | None = None):
        self.label = label
        self.input_type = input_type
        self.input_id = input_id
        self.func = func

    @property
    def controller_id(self):
        """The actual ID used by the controller directly.

        DroneManager IDs are incremented by 1 to allow negatives to be used unambiguously."""
        return abs(self.input_id) - 1


# TODO: Functions for core actions somehow
CORE_ACTIONS = [
    Action("Thrust", ActionInputType.Axis, None, None),
    Action("Yaw", ActionInputType.Axis, None, None),
    Action("Forward", ActionInputType.Axis, None, None),
    Action("Right", ActionInputType.Axis, None, None),
    Action("Control", ActionInputType.Button, None, None),
    Action("Arm", ActionInputType.Button, None, None),
    Action("Disarm", ActionInputType.Button, None, None),
    Action("Takeoff", ActionInputType.Button, None, None),
    Action("Land", ActionInputType.Button, None, None),
]


class InputMapping:
    """Mapping that assigns controller axes and buttons to actions.

    This class tracks both the available axis and button inputs and a list of actions that can or should be bound to
    these inputs.

    Axes or buttons are assigned by integer IDs. For axes, negative IDs indicate a negative response, e.g. a "-2" for
    the forward axis means that moving the stick for axis "2" in the positive direction moves the drone backward.
    For buttons, a negative ID indicates that the button should be pressed for a long duration to trigger the action.

    # TODO: Describe key arguments, in particular labels, which list all possible inputs for a controller type
    # TODO: Describe common axis meaning.
    # TODO: Describe how bindings can be set in the CLI.
    # TODO: Describe axis / button constraints. Axes actions won't be executed unless all inputs are bound.
    # TODO: Button and axis IDs are not raw controller button or axis ids. Instead they are incremented by 1 to be 1-indexed.
    """

    UNBOUND_STRING: str = " - "
    """String representation for missing bindings."""

    def __init__(self, name: str, thrust_axis: int, yaw_axis: int, forward_axis: int, right_axis: int,
                 arm_button: int, disarm_button: int, takeoff_button: int, land_button: int, control_button: int,
                 axes_labels: dict[int, str], button_labels: dict[int, str],
                 hold_duration: float = DEFAULT_HOLD_DURATION):
        """

        Args:
            name:
            thrust_axis:
            yaw_axis:
            forward_axis:
            right_axis:
            control_button:
            arm_button:
            disarm_button:
            takeoff_button:
            land_button:
            axes_labels:
            button_labels:
            hold_duration:
        """
        self.name: str = name  #: Name for this mapping
        self._actions: OrderedDict[str, Action] = OrderedDict()  #: A dictionary listing actions by their label.
        self.axes_labels: dict[int, str] = axes_labels  #: A dictionary of all axes with human-readable labels.
        self.button_labels: dict[int, str] = button_labels  #: A dictionary of all axes with human-readable labels.
        self.hold_duration: float = hold_duration  #: How long a button must be pressed to count as a long press.

        # Check that axes and button labels are unique
        input_label_list = list(self.axes_labels.values()) + list(self.button_labels.values())
        input_label_set = set(input_label_list)
        assert len(input_label_list) == len(input_label_set), "Duplicate labels for some buttons or axes!"

        # Assign core actions and their inputs
        core_inputs = [thrust_axis,
                       yaw_axis,
                       forward_axis,
                       right_axis,
                       control_button,
                       arm_button,
                       disarm_button,
                       takeoff_button,
                       land_button]
        core_actions = copy.deepcopy(CORE_ACTIONS)
        for i, core_input in enumerate(core_inputs):
            action = core_actions[i]
            action.input_id = core_input
            self._actions[action.label] = action
            # Check that the input axes or buttons are defined for this controller.
            if action.input_type is ActionInputType.Button:
                assert abs(action.input_id) in self.button_labels, (f"Invalid controller mapping! Action {action.label}"
                                                                    f"is bound to button {abs(action.input_id)}, which"
                                                                    f"does not exist in mapping {self.name}!"
                                                                    f"Available buttons: {self.button_labels}")
            else:
                assert abs(action.input_id) in self.axes_labels, (f"Invalid controller mapping! Action {action.label}"
                                                                  f"is bound to axis {abs(action.input_id)}, which does"
                                                                  f"not exist in mapping {self.name}!"
                                                                  f"Available axes: {self.axes_labels}")

        # Check that base axes actions are unique. Inverted axes are not allowed as duplicates.
        base_axis_inputs: set[int] = set([action.controller_id for action in core_actions
                                          if action.input_type is ActionInputType.Axis])
        assert len(base_axis_inputs) == 4, ("Duplicate or missing controller bindings for base axes! Inverted and"
                                            "normal inputs referring to the same input axis are not allowed!")

        # Check that core button actions are unique. Unlike actions, holds are allowed.
        base_button_inputs: set[int] = set([action.input_id for action in CORE_ACTIONS
                                            if action.input_type is ActionInputType.Button])
        assert len(base_button_inputs) == 5, "Duplicate or missing controller bindings for base actions!"

        # Two extra dictionaries for faster access: button actions by their button id and axis actions by their function
        self._button_actions_by_button: dict[int | None, set[Action]] = {}
        self._axis_actions_by_function: dict[Callable, Action] = {}

        #: A set of action names with the core actions. These actions are protected.
        self._core_actions: set[str] = set()

        for action_label, action in self._actions:
            self._core_actions.add(action_label)
            if action.input_type is ActionInputType.Button:
                self._button_actions_by_button[abs(action.input_id)] = {action}
            else:
                if action.func is not None:
                    self._axis_actions_by_function[action.func] = action

    def action_input_mapping(self) -> OrderedDict[str, str | list[str]]:
        """Get a mapping from actions to button or axis labels.

        Returns:
            A dictionary listing the button, axis or axes labels for each registered action.
        """
        output = OrderedDict()
        # Add base axis and button actions
        for action_name, action in self._actions:
            if action.input_type is ActionInputType.Button:
                input_label = self._get_button_label(action.input_id)
            else:
                input_label = self._get_axes_label(action.input_id)
            output[action_name] = input_label
        return output

    def input_action_mapping(self) -> OrderedDict[str, list[str]]:
        """Get a mapping from button or axis labels to corresponding actions.

        Returns:
            A dictionary listing the registered action for each button or axis.
        """
        output = OrderedDict()
        for axis_id, axis_label in self.axes_labels:
            output[axis_label] = []
        for button_id, button_label in self.button_labels:
            output[button_label] = []

        # Go through each registered action and add them here
        for action_label, input_label in self.action_input_mapping():
            if isinstance(input_label, str):
                if input_label in output:
                    output[input_label].append(action_label)
            else:
                for single_label in input_label:
                    if single_label in output:
                        output[single_label].append(action_label)

        for label, action_labels in output:
            if len(action_labels) == 0:
                output[label] = self.UNBOUND_STRING
        return output

    def _get_axes_label(self, axis_ids: int | list[int | None] | None) -> str:
        # Three scenarios: int, None, or a list of either.
        if isinstance(axis_ids, list):
            axis_label = [self._get_axis_label(axis_id) for axis_id in axis_ids]
        else:
            axis_label = self._get_axis_label(axis_ids)
        return axis_label

    def _get_axis_label(self, axis_id: int | None):
        if axis_id is None:
            axis_label = self.UNBOUND_STRING
        elif abs(axis_id) not in self.axes_labels:
            axis_label = f"Invalid axes {abs(axis_id)}, does not exist in this mapping!"
        else:
            axis_label = self.axes_labels[abs(axis_id)]
            if axis_id < 0:
                axis_label += " - Inverted"
        return axis_label

    def _get_button_label(self, button_id: int | None) -> str:
        if button_id is None:
            button_label = self.UNBOUND_STRING
        elif abs(button_id) not in self.button_labels:
            button_label = f"Invalid button {abs(button_id)}, does not exist in this mapping!"
        else:
            button_label = self.button_labels[abs(button_id)]
            if button_id < 0:
                button_label = f"Hold {button_label}"
        return button_label

    def add_action(self, action_name: str, action_type: ActionInputType | None = None, func: Callable | None = None,
                   input_id: int | list[int | None ] | None = None):
        """Add an action to this binding.

        Can be used to perform two different actions: Either to add a completely new action, or to add a function to
        an already existing action.
        If no action with the provided name exists, a new one is created with the passed function and input ids.
        if an action with the name already exists, but doesn't have a function, insert the passed function into that
        action. In this case the arguments ``action_type`` and ``input_id`` are ignored.

        If the new action is a button action, ``input_id`` should be either an integer or ``None``.
        If the new action is an axis action, ``input_id`` must be a list containing integers or ``None``s.

        Args:
            action_name: The name for the action.
            action_type: The type of action.
            func: The function associated with the action. Can be ``None``, unless updating an existing action.
            input_id: The inputs bound for this action, or ``None``.

        Raises:
            RuntimeError: When trying to overwrite the function of an existing action.
            ValueError: If the combination of inputs is invalid as described above.
        """
        if action_name in self._actions:
            action = self._actions[action_name]
            if action.func is not None:
                raise RuntimeError(f"Can't overwrite function of already registered action {action_name}!")
            if func is None:
                raise ValueError(f"Must provide a function if updating existing action {action_name}!")
            action.func = func
            if action.input_type is ActionInputType.Axis:
                self._axis_actions_by_function[func] = action
        else:
            if action_type is None:
                raise ValueError("Must provide an action input type if creating new actions!")
            if action_type is ActionInputType.Axis and input_id is not None:
                if not isinstance(input_id, list):
                    raise ValueError("Must provide a list of axes inputs or None for axis actions!")
            elif action_type is ActionInputType.Button and input_id is not None:
                if not isinstance(input_id, int):
                    raise ValueError("Must provide a single integer or None for button actions!")
            action = Action(action_name, action_type, input_id=input_id, func=func)
            self._actions[action_name] = action
            if action_type is ActionInputType.Button:
                if input_id in self._button_actions_by_button:
                    self._button_actions_by_button[input_id].add(action)
                else:
                    self._button_actions_by_button[input_id] = {action}
            else:
                if func is not None:
                    self._axis_actions_by_function[func] = action

    def remove_action(self, action_name: str):
        """Remove an existing action.

        Does nothing if no action with this name exists. Core actions can't be removed.

        Args:
            action_name: The name of the action to be removed.

        Raises:
            RuntimeError: When trying to remove core actions.
        """
        if action_name in self._actions:
            if action_name in self._core_actions:
                raise RuntimeError(f"Can't remove core action {action_name}!")
            action = self._actions.pop(action_name)
            if action.input_type is ActionInputType.Button:
                self._button_actions_by_button[action.input_id].remove(action)
            else:
                if action.func is not None:
                    self._axis_actions_by_function.pop(action.func)

    def bind_action(self, action_name: str, input_id: int | list[int | None]):
        """Add or change the control bindings of an existing action.

        Core actions are protected from being changed. Button actions must receive a single integer. Axes actions must
        receive a list of input ids, which may only have a single entry and contain ``None``. Axes actions are not
        executed if they are partially bound.

        Args:
            action_name: The name of the action.
            input_id: The new bindings for the actions.

        Raises:
            RuntimeError: When trying to change the binding for core actions.
            KeyError: When trying to change the bindings for actions which do not exist.
            ValueError: When the input id does not match the action type as described above.
        """
        if action_name not in self._actions:
            raise KeyError(f"No action with name {action_name}")
        if action_name in self._core_actions:
            raise RuntimeError(f"Can't change the bindings for core action {action_name}!")
        action = self._actions[action_name]
        if action.input_type is ActionInputType.Button:
            if not isinstance(input_id, int):
                raise ValueError("Button actions must be bound to a single button.")
            # Update button_dict
            self._button_actions_by_button[action.input_id].remove(action)
            action.input_id = input_id
            self._button_actions_by_button[action.input_id].add(action)
        if action.input_type is ActionInputType.Axis:
            if not isinstance(input_id, list):
                raise ValueError(f"Axis action {action_name} must receive a list of input IDs or Nones!")
            else:
                for input_entry in input_id:
                    if input_entry is not None or not isinstance(input_id, int):
                        raise ValueError(f"Must provide a list of axes IDs or None for axis action {action_name}!")
                action.input_id = input_id

    def unbind_action(self, action_name: str):
        """Remove bindings from an action.

        Core actions are protected from being unbound.

        Args:
            action_name: The name of the action.

        Raises:
            RuntimeError: When trying to unbind core actions.
            KeyError: When trying to unbind actions which do not exist.
        """
        if action_name not in self._actions:
            raise KeyError(f"No action with name {action_name}")
        if action_name in self._core_actions:
            raise RuntimeError(f"Can't change the bindings for core action {action_name}!")
        self._actions[action_name].input_id = None

    def to_json(self):
        """Create a json string for saving the current mapping."""
        # TODO: All of this

    @classmethod
    def from_json(cls, json_string: str):
        """Create an InputMapping object from a json string.

        Args:
            json_string: An input mapping encoded as a json string.

        Returns:
            The new InputMapping object.
        """
        # TODO: All of this


class PS4Mapping(InputMapping):

    thrust_axis = 1         # Left stick down
    yaw_axis = 0            # Left stick right
    forward_axis = -3       # Right stick down
    right_axis = 2          # Right stick right
    arm_button = 0          # X
    disarm_button = 1       # Circle
    land_button = 12        # D-Pad down
    takeoff_button = 11     # D-Pad up
    control_button = 5      # PS Button
    flight_mode_button = 13 # D-Pad left

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


class Controller:
    # TODO: generic base class for controllers, should handle the control loop and have functions for setting/getting
    #  mappings

    def __init__(self, mapping: InputMapping):
        pass


class ControllerPlugin(Plugin):
    """
    """

    PREFIX = "control"

    # TODO: Load mappings from config

    # TODO: Functions to print action -> button/axis label
    # TODO: Functions to print button/axis label > action
    # TODO: Functions to print input id -> button/axis label

    # TODO: Hold disarm for kill

    def __init__(self, dm, logger, name, auto_set=False, auto_drone=False, control_frequency: float = DEFAULT_FREQUENCY):
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
        self._frequency = 100
        self._control_frequency = control_frequency
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
        self.control_mode = FlightMode.POSCTL  # The flight mode to use for the manual flight. Either POSCTL or ALTCTL.

    async def add_controller(self, dev_id: int):
        """Set which controller to use, matching the ID from `check`.

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
        """Remove the current controller."""
        if self.controller is not None:
            self.logger.debug("Disconnecting from controller")
            controller = self.controller
            self.controller = None
            controller.quit()

    async def status(self):
        """Log current configuration of the controller plugin."""
        self.logger.info(f"Drone: {self._drone_name}. Control {self._in_control}. Controller: {self.controller}")
        self.logger.info(f"Control scheme: {self._mapping}")

    async def set_drone(self, drone: str):
        """Set which drone is controlled by the controller."""
        if drone not in self.dm.drones:
            self.logger.warning(f"No drone named {drone}")
            return False
        if self._in_control:
            self.logger.warning("Can't swap drones while in control of another drone!")
            return False
        self._drone_name = drone
        #self._control_frequency = self.dm.drones[drone].position_update_rate
        self.logger.info(f"Now set for drone {self._drone_name}")
        return True

    async def _check_controllers(self):
        """Check available controllers.
        """
        new_line = "\n"
        controllers = [f'{i}: {pygame.joystick.Joystick(i).get_name()}{new_line}'
                       for i in range(pygame.joystick.get_count())]
        self.logger.info(f"Detected controllers: {controllers}")

    async def _event_processor(self):
        self._disconnected = False
        while True:
            try:
                for event in pygame.event.get():
                    if event.type in self._relevant_events:
                        event_dict = event.dict
                        if "instance_id" in event_dict \
                                and event_dict["instance_id"] == self.controller.get_instance_id():
                            if event.type == pygame.JOYBUTTONDOWN:
                                self._process_button_press(event_dict["button"])
                            elif event.type == pygame.JOYBUTTONUP:
                                pass
                                # Could maybe do long-press type stuff
                                # self.logger.info(f"Released button {event_dict["button"]}")
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
        can_do_actions = (self._in_control and self._drone_name is not None
                          and self.dm.drones[self._drone_name].is_connected)
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
            action = self.dm.takeoff(self._drone_name, altitude=1.5, allow_in_air=False)
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
            self.running_tasks.add(action_task)
            self.running_tasks.add(action_awaiter)

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

        if self.control_mode is FlightMode.POSCTL:
            await self.dm.drones[self._drone_name].manual_control_position()
        else:
            await self.dm.drones[self._drone_name].manual_control_altitude()

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

    async def _control_loop(self):
        """Take controller inputs to handle motion by setting velocity setpoints.

        Actions are performed as soon as the button press is detected, but continuous inputs, such as sticks, are
        updated here at self._control_frequency.
        """
        while True:
            next_loop_time = time.monotonic()
            try:
                prev_loop_time = next_loop_time
                next_loop_time = prev_loop_time + 1 / self._control_frequency
                loop_interval = max(0.0, next_loop_time - time.monotonic())
                # If the loop interval gets short, throw a warning.
                if loop_interval < 1 / (3 * self._control_frequency):
                    self.logger.warning("Controller loop saturating! Controls might become sluggish or unresponsive.")
                await asyncio.sleep(loop_interval)
                # If auto_drone is True and there is one drone in drone manager, control it automatically
                if self.auto_drone and self._drone_name is None and len(self.dm.drones) == 1:
                    drone_name, = self.dm.drones
                    add_task = asyncio.create_task(self.set_drone(drone_name))
                    add_wait_task = coroutine_awaiter(add_task, self.logger)
                    self.running_tasks.add(add_task)
                    self.running_tasks.add(add_wait_task)
                    continue

                # If auto_set is True and there is exactly one controller available, use it automatically
                if self.auto_set \
                        and self.controller is None \
                        and not self._disconnected \
                        and pygame.joystick.get_count() == 1:
                    set_task = asyncio.create_task(self.add_controller(0))
                    set_wait_task = coroutine_awaiter(set_task, self.logger)
                    self.running_tasks.add(set_task)
                    self.running_tasks.add(set_wait_task)
                    continue

                # Process inputs
                if self._in_control and self._drone_name is not None:
                    drone = self.dm.drones[self._drone_name]
                    # drone_config = self._drone_config
                    vertical_input = self.stick_response(self._mapping.thrust_axis)
                    yaw_input = self.stick_response(self._mapping.yaw_axis)
                    right_input = self.stick_response(self._mapping.right_axis)
                    forward_input = self.stick_response(self._mapping.forward_axis)

                    # If we have non-zero inputs, and we aren't in the appropriate mode, put us into appropriate mode
                    if abs(vertical_input) > 0.01 or abs(yaw_input) > 0.01 \
                            or abs(right_input) > 0.01 or abs(forward_input) > 0.01:
                        if drone.flightmode != self.control_mode:
                            if self.control_mode is FlightMode.POSCTL:
                                swap_to_manual_task = asyncio.create_task(drone.manual_control_position())
                            else:
                                swap_to_manual_task = asyncio.create_task(drone.manual_control_altitude())
                            self.running_tasks.add(swap_to_manual_task)
                            self.running_tasks.add(asyncio.create_task(coroutine_awaiter(swap_to_manual_task,
                                                                                         self.logger)))

                    # If we are connected and armed, send stick inputs to drone
                    if drone.is_connected:
                        if drone.fence is not None:
                            try:
                                forward_input, right_input, vertical_input, yaw_input = \
                                    drone.fence.controller_safety(drone, forward_input, right_input, vertical_input,
                                                                  yaw_input)
                            except AttributeError as e:
                                self.logger.warning("Fence constraints could not be applied "
                                                    "due to missing fence attributes.")
                                self.logger.debug(repr(e), exc_info=True)
                            except Exception as e:
                                self.logger.error("Error applying controller fence logic")
                                self.logger.debug(repr(e), exc_info=True)

                        # Scale from -1/1 to 0/1, also flip because MAVLink manual control has up as positive
                        vertical_input = (-vertical_input + 1) / 2
                        await self.dm.drones[self._drone_name].set_manual_control_input(forward_input,
                                                                                        right_input,
                                                                                        vertical_input,
                                                                                        yaw_input)

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
        """Linear stick response with -10 to 10% dead zone.

        Axis should be the joystick axis being scaled. A negative number means that the response is inverted.
        """
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

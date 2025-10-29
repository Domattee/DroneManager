""" Plugin for using controllers and joysticks to control drones with DM

"""
import asyncio

import pygame

from dronecontrol.plugin import Plugin
from dronecontrol.utils import coroutine_awaiter

# TODO: Actual control loop
# TODO: Controller state?
# TODO: Different controllers?
# TODO: Control mapping?
# TODO: Hotplugging?

class ControllerPlugin(Plugin):
    """
    """

    PREFIX = "control"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.background_functions = [
            self._event_processor(),
        ]
        pygame.init()
        pygame.joystick.init()
        self.controller: pygame.joystick.JoystickType | None = None
        self.cli_commands = {
            "check": self._check_controllers,
            "connect": self._add_controller,
            "disconnect": self._remove_controller,
        }
        self._frequency = 100

    async def _add_controller(self, dev_id: int):
        if dev_id >= pygame.joystick.get_count():
            self.logger.warning(f"No controller option {dev_id}, see control-check")
            return False
        await self._remove_controller()
        self.controller = pygame.joystick.Joystick(dev_id)
        self.controller.init()
        self.logger.info(f"Connected to controller {self.controller.get_name()}")
        self.controller.rumble(0.5, 0.5, 1)
        return True

    async def _remove_controller(self):
        if self.controller is not None:
            self.logger.debug("Disconnecting from controller")
            controller = self.controller
            self.controller = None
            controller.quit()

    async def _check_controllers(self):
        self.logger.info(f"Detected controllers: {[f"{i}: {pygame.joystick.Joystick(i).get_name()}\n" for i in range(pygame.joystick.get_count())]}")

    async def _event_processor(self):
        while True:
            try:
                for event in pygame.event.get():
                    self.logger.info(f"{event.type}, {event.dict}")
            except Exception as e:
                self.logger.warning("Exception processing controller event!")
                self.logger.debug(repr(e), exc_info=True)

    async def close(self):
        if self.controller is not None:
            self.controller.quit()
        pygame.quit()
        await super().close()
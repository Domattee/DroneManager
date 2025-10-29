""" Plugin for using controllers and joysticks to control drones with DM

"""
import asyncio

import hid

from dronecontrol.plugin import Plugin
from dronecontrol.utils import coroutine_awaiter

# TODO: Actual control loop
# TODO: Controller state?
# TODO: Different controllers?
# TODO: Control mapping?
# TODO: Hotplugging?
# TODO: Try pygame again

class ControllerPlugin(Plugin):
    """
    """

    PREFIX = "control"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.background_functions = [
            self._process_state_changes(),
        ]
        self.controller: hid.device | None = None
        self._possible_controllers = []
        self._reading_task = None
        self.cli_commands = {
            "check": self._check_controllers,
            "set": self._add_controller,
        }

    async def _add_controller(self, dev_id: int):
        if dev_id >= len(self._possible_controllers):
            self.logger.warning(f"No controller option {dev_id}, see control-check")
            return False
        if self.controller is not None:
            if self._reading_task is not None:
                self._reading_task.cancel()
            self.controller.close()
        self.controller = hid.device()
        vendor_id, product_id, product_string, manufacturer_string = self._possible_controllers[dev_id]
        self.controller.open(vendor_id, product_id)
        self.controller.set_nonblocking(True)
        self._reading_task = asyncio.create_task(self._read_controller())
        self._running_tasks.add(self._reading_task)
        self.logger.info(f"Connected to controller {product_string, manufacturer_string}")
        return True

    async def _check_controllers(self):
        controllers = [device for device in hid.enumerate() if device["usage"] in [4, 5] and device["product_string"] != "HIDI2C Device"]
        self._possible_controllers = [(controller["vendor_id"], controller["product_id"], controller["product_string"], controller["manufacturer_string"]) for controller in controllers]
        self.logger.info(f"HID Joysticks or contollers: {[f"{i}: {(item[2], item[3])}\n" for i, item in enumerate(self._possible_controllers)]}")

    async def _read_controller(self):
        while True:
            try:
                report = self.controller.read(64)
                await self._process_state(report)
            except Exception as e:
                self.logger.warning("Couldn't read controller state!")
                self.logger.debug(repr(e), exc_info=True)

    async def _process_state(self, report):
        self.logger.info(report)

    async def _process_state_changes(self):
        pass

    async def close(self):
        if self.controller is not None:
            self.controller.close()
        await super().close()
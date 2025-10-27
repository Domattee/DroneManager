""" Plugin for using controllers and joysticks to control drones with DM

"""
import asyncio

import hid

from dronecontrol.plugin import Plugin
from dronecontrol.utils import coroutine_awaiter

# TODO: Actual control loop

class ControllerPlugin(Plugin):
    """
    """

    PREFIX = "control"

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.background_functions = [
        #    self._check_devices(),
        ]
        self.controller: hid.device | None = None
        self._possible_controllers = []
        self.cli_commands = {
            "check": self._check_controllers,
            "set": self._add_controller,
        }

    async def _add_controller(self, dev_id: int):
        if dev_id >= len(self._possible_controllers):
            self.logger.warning(f"No controller option {dev_id}, see control-check")
            return False
        if self.controller is not None:
            self.controller.close()
        self.controller = hid.device()
        vendor_id, product_id, product_string, manufacturer_string = self._possible_controllers[dev_id]
        self.controller.open(vendor_id, product_id)
        self.controller.set_nonblocking(True)
        self.logger.info(f"Connected to controller {product_string, manufacturer_string}")
        return True

    async def close(self):
        if self.controller is not None:
            self.controller.close()
        await super().close()

    async def _check_controllers(self):
        controllers = [device for device in hid.enumerate() if device["usage"] in [4, 5] and device["product_string"] != "HIDI2C Device"]
        self._possible_controllers = [(controller["vendor_id"], controller["product_id"], controller["product_string"], controller["manufacturer_string"]) for controller in controllers]
        self.logger.info(f"HID Joysticks or contollers: {[f"{i}: {(item[2], item[3])}\n" for i, item in enumerate(self._possible_controllers)]}")

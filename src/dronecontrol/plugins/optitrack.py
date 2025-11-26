""" Plugin for using controllers and joysticks to control drones with DM

"""
import asyncio
import math
from collections.abc import Callable

from dronecontrol.plugin import Plugin
from dronecontrol.utils import coroutine_awaiter

from natnet import NatNetClient, DataFrame


class OptitrackPlugin(Plugin):
    """
    """

    PREFIX = "opti"

    def __init__(self, dm, logger, name, server_ip: str | None = None):
        """

        """
        super().__init__(dm, logger, name)
        self.background_functions = [
            #self._event_processor(),
            #self._control_loop(),
        ]
        self.cli_commands = {
            "connect": self.connect_server,
        }
        self.client: NatNetClient | None = None
        self.server_ip: str = server_ip if server_ip is not None else "127.0.0.1"
        self.local_ip: str = "127.0.0.1"

    async def connect_server(self, remote: str = None, local: str = None):
        if self.client is not None:
            self.logger.warning("Already connected to a NatNetserver, aborting.")
            return False

        if remote is not None:
            self.server_ip = remote
        if local is not None:
            self.local_ip = local
        self.logger.info(f"Connecting to NatNet Server @ {self.server_ip}")
        client = NatNetClient(server_ip_address=self.server_ip, local_ip_address=self.local_ip, use_multicast=False)
        try:
            client.connect(5)
            client.request_modeldef()
            client.run_async()
            client.on_data_frame_received_event.handlers.append()
        except ConnectionResetError as e:
            self.logger.warning("Couldn't connect to the server!")
            self.logger.debug(repr(e), exc_info = True)
            return False

        self.client = client
        return True

    def _data_frame_callback(self, data_frame: DataFrame):
        self.logger.info(f"Received dataframe {data_frame.} - {data_frame.rigid_bodies}")

    async def close(self):
        await super().close()
        if self.client is not None:
            self.client.shutdown()

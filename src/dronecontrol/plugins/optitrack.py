""" Plugin for using controllers and joysticks to control drones with DM

"""
import asyncio
import math
from collections.abc import Callable

from dronecontrol.plugin import Plugin
from dronecontrol.utils import coroutine_awaiter

from natnet import NatNetClient, DataFrame, RigidBody


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
            "add_drone": self.add_drone,
        }
        self.client: NatNetClient | None = None
        self.server_ip: str = server_ip if server_ip is not None else "127.0.0.1"
        self.local_ip: str = "127.0.0.1"
        self._drone_id_mapping: dict[int, str] = {}

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

    def add_drone(self, name: str, track_id: int):
        if name not in self.dm.drones:
            self.logger.warning(f"No drone named {name}")
        else:
            self._drone_id_mapping[track_id] = name

    def _data_frame_callback(self, data_frame: DataFrame):
        # TODO: Rework this to have one async function for each drone, instead of creating a new one every message
        # Probably save the latest information somewhere and then just send that with a given frequency.
        self.logger.info(f"Received dataframe {data_frame.prefix} - {data_frame.rigid_bodies}")
        for rigid_body in data_frame.rigid_bodies:
            if rigid_body.id_num in self._drone_id_mapping:
                send_task = asyncio.create_task(self._send_info_to_drone(rigid_body.id_num, rigid_body.pos, rigid_body.rot))
                send_awaiter = asyncio.create_task(coroutine_awaiter(send_task, self.logger))
                self._running_tasks.add(send_task)
                self._running_tasks.add(send_awaiter)

    async def _send_info_to_drone(self, track_id: int, pos, quat_rot):
        try:
            drone_name = self._drone_id_mapping[track_id]
            self.logger.info(f"Would send info to drone {drone_name, track_id}: {pos, quat_rot}")
            #self.dm.drones[drone_name].system.mocap.set_odometry()  # Odometry object
        except KeyError:
            pass


    async def close(self):
        await super().close()
        if self.client is not None:
            self.client.shutdown()

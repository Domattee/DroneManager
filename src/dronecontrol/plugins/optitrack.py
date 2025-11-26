""" Plugin for using controllers and joysticks to control drones with DM

"""
import asyncio

from dronecontrol.plugin import Plugin
from dronecontrol.utils import coroutine_awaiter
from dronecontrol.plugins.NatNet.NatNetClient import NatNetClient

class OptitrackPlugin(Plugin):
    """
    """

    PREFIX = "opti"

    def __init__(self, dm, logger, name, server_ip: str | None = None):
        """

        """
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "connect": self.connect_server,
            "add_drone": self.add_drone,
        }
        self.client: NatNetClient | None = None
        self.server_ip: str = server_ip if server_ip is not None else "127.0.0.1"
        self.local_ip: str = "127.0.0.1"
        self._drone_id_mapping: dict[int, str] = {}

        self._rigid_body_data: dict = {}
        self.frame_count: int = 0
        self.log_every = 100

    async def connect_server(self, remote: str = None, local: str = None):
        if self.client is not None:
            self.logger.warning("Already connected to a NatNetserver, aborting.")
            return

        if remote is not None:
            self.server_ip = remote
        if local is not None:
            self.local_ip = local
        self.logger.info(f"Connecting to NatNet Server @ {self.server_ip}")
        client = NatNetClient()
        client.set_client_address(self.local_ip)
        client.set_server_address(self.server_ip)
        client.rigid_body_listener = self._rigid_body_callback
        client.set_use_multicast(True)
        conn_good = False
        try:
            is_running = client.run("d")
            if not is_running:
                self.logger.error("Couldn't start the client!")

            else:
                await asyncio.sleep(1)
                if not client.connected():
                    self.logger.error("Couldn't connect to the server!")
                else:
                    conn_good = True
        except ConnectionResetError as e:
            self.logger.warning("Couldn't connect to the server!")
            self.logger.debug(repr(e), exc_info = True)
            return
        if conn_good:
            self.client = client
            self.logger.info("Connected to NatNet Server!")
            
        else:
            client.shutdown()

    def add_drone(self, name: str, track_id: int):
        if name not in self.dm.drones:
            self.logger.warning(f"No drone named {name}")
        else:
            self._drone_id_mapping[track_id] = name

    def remove_drone(self, name: str):
        to_remove = None
        for key, value in self._drone_id_mapping.items():
            if value == name:
                to_remove = key
        if to_remove:
            self._drone_id_mapping.pop(to_remove)

    def _rigid_body_callback(self, track_id, position, rotation):
        try:
            self.frame_count += 1
            self.frame_count = self.frame_count % 1000000
            if self.frame_count % self.log_every == 0:
                self.logger.info(f"Logging every {self.log_every}th rigid body frame {track_id} - {position, rotation}")
            if track_id in self._drone_id_mapping:
                drone_name = self._drone_id_mapping[track_id]
                self.logger.info(f"Would send info to drone {drone_name, track_id}: {position, rotation}")
                # TODO: Figoure out position and rotation transform
                #send_task = asyncio.create_task(self.dm.drones[drone_name].system.mocap.set_vision_position_estimate())
                #send_task_awaiter = asyncio.create_task(coroutine_awaiter(send_task, self.logger))
                #self._running_tasks.add(send_task)
                #self._running_tasks.add(send_task_awaiter)
        except Exception as e:
            self.logger.error("Error in rigid body callback:")
            self.logger.debug(repr(e), exc_info = True)

    async def close(self):
        if self.client is not None:
            self.client.shutdown()
        await super().close()
        

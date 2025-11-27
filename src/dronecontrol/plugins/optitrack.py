""" Plugin for using controllers and joysticks to control drones with DM

"""
import asyncio
import numpy as np
from scipy.spatial.transform import Rotation

from dronecontrol.plugin import Plugin
from dronecontrol.utils import coroutine_awaiter
from dronecontrol.plugins.NatNet.NatNetClient import NatNetClient



class CoordinateConversion:

    def __init__(self, n_axis: str, e_axis: str, d_axis: str):
        """

        """
        # The drone axes expressed as a tracking system axes. The axes must be aligned, only permutation and
        # direction can change.
        # For example:
        # n_axis = -z  (Drone north/forward axis is aligned with negative z-axis in the tracking coordinate system)
        # e_axis = -x (Drone east/right matches negative x-axis)
        # d_axis = y (Drone down matches tracking y-axis)
        self._choices = ["x", "-x", "y", "-y", "z", "-z"]
        assert n_axis in self._choices and e_axis in self._choices and d_axis in self._choices, f"Invalid axis for coordinate conversion, must be one of {self._choices}"
        self.axes = [n_axis, e_axis, d_axis]
        self.rotation: Rotation | None = None
        self._perm_matrix = np.zeros((3,3))
        self.make_rotation()

    def convert(self, tracking_pos, tracking_quat):
        converted_pos = self.rotation.apply(tracking_pos)
        converted_rot = self.rotation * Rotation.from_quat(tracking_quat) * self.rotation.inv().as_euler("xyz", degrees=False)
        return converted_pos, converted_rot

    def _make_perm_matrix(self):
        self._perm_matrix = np.zeros((3, 3))
        for i, axis in enumerate(self.axes):
            neg = axis.startswith("-")
            if axis.endswith("x"):
                pos = 0
            elif axis.endswith("y"):
                pos = 1
            else:
                pos = 2
            self._perm_matrix[i, pos] = -1 if neg else 1

    def make_rotation(self):
        self._make_perm_matrix()
        self.rotation = Rotation.from_matrix(self._perm_matrix)


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
        client.server_ip_address = self.server_ip
        client.local_ip_address = self.local_ip
        client.use_multicast = False
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
        # TODO: Rework this to have one async function for each drone, instead of creating a new one every message
        # Probably save the latest information somewhere and then just send that with a given frequency.
        self.logger.info(f"Received rigid body frame {track_id} - {position, rotation}")
        if track_id in self._drone_id_mapping:
            drone_name = self._drone_id_mapping[track_id]
            self.logger.info(f"Would send info to drone {drone_name, track_id}: {position, rotation}")
            send_task = asyncio.create_task(self.dm.drones[drone_name].system.mocap.set_vision_position_estimate())
            send_task_awaiter = asyncio.create_task(coroutine_awaiter(send_task, self.logger))
            self._running_tasks.add(send_task)
            self._running_tasks.add(send_task_awaiter)

    async def close(self):
        await super().close()
        if self.client is not None:
            self.client.shutdown()


if __name__ == "__main__":
    test_pos = [-1, -2, 3]
    test_attitudes = [[0, 10, 0], [10, 90, 10], [10, 180, 0]]
    test_perm_matrix = np.asarray([[0, 0, -1],
                                   [-1, 0, 0],
                                   [0, 1, 0]])
    opti_conv = CoordinateConversion("-z", "-x", "y")
    rot = opti_conv.rotation
    print("Permutation matrix: ", test_perm_matrix, opti_conv._perm_matrix)
    print("Test vector: ", test_pos, rot.apply(test_pos))
    print("Rotation matrix: ", rot.as_euler("xyz", degrees=True))
    for attitude in test_attitudes:
        att_rot = Rotation.from_euler("yxz", [*attitude], degrees=True)
        print("Test_attitude: ", attitude, (rot*att_rot*rot.inv()).as_euler("zxy", degrees=True))
""" Plugin for using controllers and joysticks to control drones with DM

"""
import asyncio
import math
import numpy as np
from scipy.spatial.transform import Rotation

from mavsdk.mocap import VisionPositionEstimate, PositionBody, AngleBody, MocapError, Covariance

from dronemanager.plugin import Plugin
from dronemanager.plugins.NatNet.NatNetClient import NatNetClient
from dronemanager.utils import coroutine_awaiter


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
        # xyz can also be written as capital letters to indicate extrinsic rotations, same convention as scipy
        self._choices = ["x", "-x", "y", "-y", "z", "-z"]
        self._choices.extend([choice.upper() for choice in self._choices])
        assert n_axis in self._choices and e_axis in self._choices and d_axis in self._choices, f"Invalid axis for coordinate conversion, must be one of {self._choices}"
        self.axes = [n_axis, e_axis, d_axis]
        self.rotation: Rotation | None = None
        self._inv_rotation: Rotation | None = None
        self._perm_matrix = np.zeros((3,3))
        self.rotation_sequence: str = ""
        self.make_rotation()

    def convert_euler(self, tracking_pos, tracking_euler, out_sequence = "XYZ", degrees = False, in_degrees = True):
        converted_pos = self.rotation.apply(tracking_pos)
        converted_rot = (self.rotation * Rotation.from_euler(self.rotation_sequence, tracking_euler, degrees=in_degrees)
                         * self._inv_rotation).as_euler(out_sequence, degrees=degrees)
        return converted_pos, converted_rot

    def convert_quat(self, tracking_pos, tracking_quat, out_sequence = "XYZ", degrees = False):
        converted_pos = self.rotation.apply(tracking_pos)
        converted_rot = (self.rotation * Rotation.from_quat(tracking_quat) *
                         self.rotation.inv()).as_euler(out_sequence,degrees=degrees)
        return converted_pos, converted_rot

    def _make_perm_matrix(self):
        self._perm_matrix = np.zeros((3, 3))
        seq = ""
        for i, axis in enumerate(self.axes):
            seq += axis[-1]
            axis = axis.lower()
            neg = axis.startswith("-")
            if axis.endswith("x") or axis.endswith(()):
                pos = 0
            elif axis.endswith("y"):
                pos = 1
            else:
                pos = 2
            self._perm_matrix[i, pos] = -1 if neg else 1
        self.rotation_sequence = seq

    def make_rotation(self):
        self._make_perm_matrix()
        self.rotation = Rotation.from_matrix(self._perm_matrix)
        self._inv_rotation = self.rotation.inv()


class OptitrackPlugin(Plugin):
    """
    """

    PREFIX = "opti"

    def __init__(self, dm, logger, name, server_ip: str | None = None, local_ip: str | None = None,
                 axes: list[str] | None = None, log_frames: bool = False):
        """

        """
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "connect": self.connect_server,
            "list": self.log_available_bodies,
            "add": self.add_drone,
            "remove": self.remove_drone,
            "status": self.status,
        }
        self.client: NatNetClient | None = None
        self.server_ip: str = server_ip if server_ip is not None else "127.0.0.1"
        self.local_ip: str = local_ip if local_ip is not None else "127.0.0.1"
        self._drone_id_mapping: dict[int, str] = {}
        self.available_bodies: dict[int, np.ndarray] = {}

        self.frame_count: int = 0
        self.log_rigid_frames: bool = log_frames
        self.log_every: int = 99

        if axes is None:
            axes = ["z", "-x", "-y"]
        self.coordinate_transform = CoordinateConversion(*axes)
        self._event_loop = asyncio.get_running_loop()
        self._stopping = False
        self._covariance_matrix = Covariance([math.nan])

    async def connect_server(self, remote: str = None, local: str = None):
        """ Connect to a NatNet server at the given IP remote and local IP addresses. Localhost by default. """
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
        client.new_frame_with_data_listener = self._new_frame_callback
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

    async def add_drone(self, name: str, track_id: int):
        """ Add a drone to the data forwarding system by name and track ID"""
        if name not in self.dm.drones:
            self.logger.warning(f"No drone named {name}")
        else:
            self._drone_id_mapping[track_id] = name

    async def remove_drone(self, name: str):
        """ Remove a drone from the data forwarding system by name"""
        to_remove = None
        for key, value in self._drone_id_mapping.items():
            if value == name:
                to_remove = key
        if to_remove:
            self._drone_id_mapping.pop(to_remove)

    async def log_available_bodies(self):
        """ Print available rigid bodies on the NatNet server"""
        if self.client is None:
            self.logger.warning("Not connected to a NatNet server!")
            return
        body_str = "\n".join([f"Track ID: {track_id}, Position {position}" for track_id, position in self.available_bodies.items()])
        self.logger.info("Available Rigid Bodies:\n" + body_str)

    async def status(self):
        if len(self._drone_id_mapping) > 0:
            out_str = "Streaming configured for following drones:\nNAME\tTRACK ID\n"
            for track_id, name in self._drone_id_mapping.items():
                out_str += f"{name}\t{track_id}\n"
            self.logger.info(out_str)
        await self.log_available_bodies()

    def _new_frame_callback(self, data_dict):
        try:
            if not self._stopping:
                body_dict = {}
                rigid_body_list = data_dict["mocap_data"].rigid_body_data.rigid_body_list
                for rb in rigid_body_list:
                    track_id = rb.id_num
                    position = rb.pos
                    body_dict[track_id] = position
                    rotation = rb.rot
                    if track_id in self._drone_id_mapping:
                        callback_task = asyncio.run_coroutine_threadsafe(self._process_rigid_body(track_id, position, rotation), self._event_loop)
                        callback_awaiter = asyncio.run_coroutine_threadsafe(coroutine_awaiter(callback_task, self.logger), self._event_loop)
                        self._running_tasks.add(callback_task)
                        self._running_tasks.add(callback_awaiter)
                self.available_bodies = body_dict
        except Exception as e:
            self.logger.error("Exception in new frame callback, see log for details.")
            self.logger.debug(repr(e), exc_info = True)

    async def _process_rigid_body(self, track_id, position, rotation):
        try:
            self.frame_count += 1
            self.frame_count = self.frame_count % 1000000
            if self.log_rigid_frames and self.frame_count % self.log_every == 0:
                self.logger.debug(f"Logging every {self.log_every}th rigid body frame RAW: {track_id} - {position, rotation}")
            if track_id in self._drone_id_mapping:
                drone_name = self._drone_id_mapping[track_id]
                conv_position, conv_rotation = self.coordinate_transform.convert_quat(position, rotation, out_sequence="xyz", degrees=False)
                try:
                    drone = self.dm.drones[drone_name]
                    if drone.is_connected:
                        vis_pos_estimate = VisionPositionEstimate(0,
                                                                PositionBody(*conv_position),
                                                                AngleBody(*conv_rotation),
                                                                self._covariance_matrix)
                        if self.log_rigid_frames and self.frame_count % self.log_every == 0:
                            self.logger.info(f"Logging every {self.log_every}th rigid body frame CONVERTED:{track_id} - {conv_position, conv_rotation}")
                        send_task = asyncio.create_task(self._error_wrapper(drone.system.mocap.set_vision_position_estimate, vis_pos_estimate))
                        send_task_awaiter = asyncio.create_task(coroutine_awaiter(send_task, self.logger))
                        self._running_tasks.add(send_task)
                        self._running_tasks.add(send_task_awaiter)
                except KeyError:
                    self.logger.warning(f"Received tracking data for drone '{drone_name}' which is no longer connected, removing...")
                    await self.remove_drone(drone_name)
        except UnboundLocalError:
            pass
        except Exception as e:
            self.logger.error("Exception in rigid body callback, see log for details.")
            self.logger.debug(repr(e), exc_info = True)

    async def close(self):
        self._stopping = True
        await super().close()
        if self.client is not None:
            self.client.shutdown()

    async def _error_wrapper(self, func, *args, **kwargs):
        try:
            res = await func(*args, **kwargs)
        except MocapError as e:
            self.logger.error(f"CameraError: {e._result.result_str}")
            return False
        return res

"""Plugin for using Motive optitrack systems with DM.

Receives rigid body information from Motive, assigns it to drones and forwards it as MAVLink messages. Generally, the
flight controllers of the drones also need to be configured to make use of these messages.
"""
import asyncio
import math
import numpy as np
from scipy.spatial.transform import Rotation

from mavsdk.mocap import VisionPositionEstimate, PositionBody, AngleBody, MocapError, Covariance

from dronemanager.plugin import Plugin
from .NatNet.NatNetClient import NatNetClient
from dronemanager.utils import coroutine_awaiter


class CoordinateConversion:

    def __init__(self, n_axis: str, e_axis: str, d_axis: str, body_forward: str, body_right: str, body_down: str):
        """

        """
        # The drone axes expressed as a tracking system axes. The axes must be aligned, only permutation and
        # direction can change. Optitrack assumes YPR for its axes.
        # For example:
        # n_axis = -z  (Drone north/forward axis is aligned with negative z-axis in the tracking coordinate system)
        # e_axis = -x (Drone east/right matches negative x-axis)
        # d_axis = y (Drone down matches tracking y-axis)
        # Also same for body. TODO: Write this out better
        # xyz can also be written as capital letters to indicate extrinsic rotations, same convention as scipy
        self._choices = ["x", "-x", "y", "-y", "z", "-z"]
        self._choices.extend([choice.upper() for choice in self._choices])
        assert n_axis in self._choices and e_axis in self._choices and d_axis in self._choices, \
            f"Invalid axis for coordinate conversion, must be one of {self._choices}"
        self.world_axes = [n_axis, e_axis, d_axis]
        self.body_axes = [body_forward, body_right, body_down]
        self.rotation_world: Rotation | None = None
        self.rotation_body: Rotation | None = None
        self.make_rotation()

    def convert_euler(self, tracking_pos, tracking_euler, in_sequence="XYZ", out_sequence="ZYX", out_degrees=False, in_degrees=True):
        tracking_rot = Rotation.from_euler(in_sequence, tracking_euler, degrees=in_degrees)
        return self._convert(tracking_pos, tracking_rot, out_sequence=out_sequence, degrees=out_degrees)

    def convert_quat(self, tracking_pos, tracking_quat, out_sequence="ZYX", degrees=False):
        tracking_rot = Rotation.from_quat(tracking_quat)
        return self._convert(tracking_pos, tracking_rot, out_sequence=out_sequence, degrees=degrees)

    def _convert(self, tracking_pos, tracking_rot, out_sequence, degrees=False):
        converted_pos = self.rotation_world.apply(tracking_pos)
        converted_rot = (self.rotation_world * tracking_rot * self.rotation_body).as_euler(out_sequence, degrees=degrees)
        return converted_pos, converted_rot

    def set_rotation_world(self, world_axes):
        self.world_axes = world_axes
        self.make_rotation()

    def set_rotation_body(self, body_axes):
        self.body_axes = body_axes
        self.make_rotation()

    def _make_perm_matrix(self, axes):
        mat = np.zeros((3, 3))
        for i, axis in enumerate(axes):
            axis = axis.lower()
            neg = axis.startswith("-")
            if axis.endswith("x"):
                pos = 0
            elif axis.endswith("y"):
                pos = 1
            else:
                pos = 2
            mat[i, pos] = -1 if neg else 1
        return mat

    def make_rotation(self):
        self.rotation_world = Rotation.from_matrix(self._make_perm_matrix(self.world_axes))
        self.rotation_body = Rotation.from_matrix(self._make_perm_matrix(self.body_axes).T)


class OptitrackPlugin(Plugin):
    """
    """

    PREFIX = "opti"

    def __init__(self, dm, logger, name, server_ip: str | None = None, local_ip: str | None = None,
                 world_axes: list[str] | None = None, body_axes: list[str] | None = None, log_frames: bool = False):
        """

        """
        super().__init__(dm, logger, name)
        self.cli_commands = {
            "connect": self.connect_server,
            "list": self.log_available_bodies,
            "add": self.add_drone,
            "remove": self.remove_drone,
            "status": self.status,
            "set-coords-world": self.set_coordinates_world,
            "set-coords-body": self.set_coordinates_body,
            "check-conv": self.check_conv,
        }
        self.client: NatNetClient | None = None
        self.server_ip: str = server_ip if server_ip is not None else "127.0.0.1"
        self.local_ip: str = local_ip if local_ip is not None else "127.0.0.1"
        self._drone_id_mapping: dict[int, str] = {}
        self.available_bodies: dict[int, tuple[np.ndarray, np.ndarray]] = {}

        self.frame_count: int = 0
        self.log_rigid_frames: bool = log_frames
        self.log_every: int = 99

        if world_axes is None:
            world_axes = ["x", "z", "-y"]
        if body_axes is None:
            body_axes = ["x", "z", "-y"]
        self.coordinate_transform = CoordinateConversion(*world_axes, *body_axes)
        self._event_loop = asyncio.get_running_loop()
        self._stopping = False
        self._covariance_matrix = Covariance([math.nan])

        self._err_count = [0]  # Stick in a list so we can pass by reference sort of work around

    async def connect_server(self, remote: str = None, local: str = None):
        """Connect to a NatNet server at the given IP remote and local IP addresses. Localhost by default."""
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
            self.logger.debug(repr(e), exc_info=True)
            return
        if conn_good:
            self.client = client
            self.logger.info("Connected to NatNet Server!")
        else:
            client.shutdown()

    async def add_drone(self, name: str, track_id: int):
        """Add a drone to the data forwarding system by name and track ID"""
        if name not in self.dm.drones:
            self.logger.warning(f"No drone named {name}")
        else:
            self._drone_id_mapping[track_id] = name

    async def remove_drone(self, name: str):
        """Remove a drone from the data forwarding system by name"""
        self._remove_drone(name)

    def _remove_drone(self, name):
        to_remove = None
        for key, value in self._drone_id_mapping.items():
            if value == name:
                to_remove = key
        if to_remove:
            self._drone_id_mapping.pop(to_remove)

    async def log_available_bodies(self):
        """Print available rigid bodies on the NatNet server"""
        if self.client is None:
            self.logger.warning("Not connected to a NatNet server!")
            return
        body_str = "\n".join([f"Track ID: {track_id}, Position {array[0]}"
                              for track_id, array in self.available_bodies.items()])
        self.logger.info("Available Rigid Bodies:\n" + body_str)

    def log_coordinate_system(self):
        self.logger.info(f"World axes: {self.coordinate_transform.world_axes}, body axes: {self.coordinate_transform.body_axes}")

    async def status(self):
        if len(self._drone_id_mapping) > 0:
            out_str = "Streaming configured for following drones:\nNAME\tTRACK ID\n"
            for track_id, name in self._drone_id_mapping.items():
                out_str += f"{name}\t{track_id}\n"
            self.logger.info(out_str)
        await self.log_available_bodies()
        self.log_coordinate_system()

    async def set_coordinates_world(self, x: str = "x", y: str = "y", z: str = "z",):
        # Change the coordinate system used by the plugin.
        try:
            self.coordinate_transform.set_rotation_world([x, y, z])
        except Exception as e:
            self.logger.error("Couldn't change world coordinate system, see log for details!")
            self.logger.debug(repr(e), exc_info=True)

    async def set_coordinates_body(self, x: str = "x", y: str = "y", z: str = "z",):
        # Change the coordinate system used by the plugin.
        try:
            self.coordinate_transform.set_rotation_body([x, y, z])
        except Exception as e:
            self.logger.error("Couldn't change body coordinate system, see log for details!")
            self.logger.debug(repr(e), exc_info=True)

    async def check_conv(self, track_id: int, out_sequence: str = "xyz", in_sequence: str = "XYZ"):
        """Log coordinate conversion information for the given track id.

        Useful to check that the conversion is happening properly. All axes sequences follow scipy rotation convention.

        Args:
            track_id: The Motive track ID to be used.
            out_sequence: The axis sequence for the output euler angles, default "xyz" for roll, pitch and yaw.
            in_sequence: The axis sequence to translate the optitrack quaternion to euler angles. Not used during normal
                operation, only for this diagnostic. Default, "XYZ".
        """
        if track_id not in self.available_bodies:
            self.logger.warning(f"No track ID {track_id}")
            await self.log_available_bodies()
        else:
            conv = self.coordinate_transform
            pos, quat = self.available_bodies[track_id]
            euler = Rotation.from_quat(quat).as_euler(in_sequence, degrees=True)
            conv_pos, _ = conv.convert_quat(pos, quat, out_sequence=out_sequence, degrees=False)
            _, conv_rot_deg = conv.convert_quat(pos, quat, out_sequence=out_sequence, degrees=True)
            self.logger.info(f"Rotation matrices from axes listing:"
                             f"\nWorld matrix {conv.rotation_world.as_matrix()}\tBody matrix {conv.rotation_body.as_matrix()}")
            self.logger.info(f"Initial and converted position: "
                             f"\nInitial {pos}\nConverted: {conv_pos}")
            self.logger.info(f"Initial and converted angles: "
                             f"\nQuat from Motive: {quat}\nEuler Motive: {euler}"
                             f"\nEuler (RPY) converted {conv_rot_deg}")

    def _new_frame_callback(self, data_dict):
        try:
            if not self._stopping:
                body_dict = {}
                rigid_body_list = data_dict["mocap_data"].rigid_body_data.rigid_body_list
                for rb in rigid_body_list:
                    track_id = rb.id_num
                    position = rb.pos
                    rotation = rb.rot
                    body_dict[track_id] = (position, rotation)
                    if track_id in self._drone_id_mapping:
                        self._process_rigid_body(track_id, position, rotation)
                self.available_bodies = body_dict
        except Exception as e:
            self.logger.error("Exception in new frame callback, see log for details.")
            self.logger.debug(repr(e), exc_info=True)

    def _process_rigid_body(self, track_id, position, rotation):
        try:
            self.frame_count += 1
            self.frame_count = self.frame_count % 1000000
            if self.log_rigid_frames and self.frame_count % self.log_every == 0:
                self.logger.debug(f"Logging every {self.log_every}th rigid body frame RAW: "
                                  f"{track_id} - {position, rotation}")
            if track_id in self._drone_id_mapping:
                drone_name = self._drone_id_mapping[track_id]
                conv_position, conv_rotation = self.coordinate_transform.convert_quat(position,
                                                                                      rotation,
                                                                                      out_sequence="xyz",
                                                                                      degrees=False)
                try:
                    drone = self.dm.drones[drone_name]
                    if drone.is_connected:
                        vis_pos_estimate = VisionPositionEstimate(0,
                                                                  PositionBody(*conv_position),
                                                                  AngleBody(*conv_rotation),
                                                                  self._covariance_matrix,
                                                                  0)
                        if self.log_rigid_frames and self.frame_count % self.log_every == 0:
                            self.logger.info(f"Logging every {self.log_every}th rigid body frame CONVERTED: "
                                             f"{track_id} - {conv_position, conv_rotation}")
                        send_task = asyncio.run_coroutine_threadsafe(
                            self._error_wrapper(drone.system.mocap.set_vision_position_estimate,
                                                self._err_count,
                                                vis_pos_estimate),
                            self._event_loop)
                        asyncio.run_coroutine_threadsafe(coroutine_awaiter(send_task, self.logger), self._event_loop)
                except KeyError:
                    self.logger.warning(f"Received tracking data for drone '{drone_name}' "
                                        f"which is no longer connected, removing...")
                    self._remove_drone(drone_name)
        except UnboundLocalError:
            pass
        except Exception as e:
            self.logger.error("Exception in rigid body callback, see log for details.")
            self.logger.debug(repr(e), exc_info=True)

    async def close(self):
        if self.client is not None:
            self.client.new_frame_with_data_listener = None
        self._stopping = True
        await super().close()
        if self.client is not None:
            self.client.shutdown()

    async def _error_wrapper(self, func, err_count, *args, **kwargs):
        try:
            res = await func(*args, **kwargs)
            err_count[0] = 0
        except MocapError as e:
            err_count[0] += 1
            if err_count[0] == 1:
                self.logger.error(f"MocapError: {e._result.result_str}")
            elif err_count[0] % 100 == 0:
                self.logger.error(f"{err_count[0]} MocapErrors: {e._result.result_str}")
            return False
        return res

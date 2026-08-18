"""Plugin for using Motive OptiTrack systems with DM.

Receives rigid body information from Motive, assigns it to drones and forwards it as MAVLink messages. Generally, the
flight controllers of the drones also need to be configured to make use of these messages.

Additionally, allows for arbitrary flipping or mirroring across axes to convert the Y-up or Z-up coordinate systems from
Motive to something that common FCs expect.
"""
import asyncio
import math
from collections.abc import Callable

import numpy as np
from scipy.spatial.transform import Rotation

from mavsdk.mocap import VisionPositionEstimate, PositionBody, AngleBody, MocapError, Covariance, MocapResult

from dronemanager.plugin import Plugin
from .NatNet.NatNetClient import NatNetClient
from dronemanager.utils import coroutine_awaiter


class CoordinateConversion:
    """Class for coordinate conversions.

    Allows for axes to arbitrarily permutated and flipped and provides conversion functions to convert input positions
    and rotations to be converted to the permuted and flipped coordinate system.

    There are two transformations happening, first between the Motive world coordinate system and the target system,
    and then between the drone body system and the Motive rigid body system. Transforms are defined by providing an
    axes in the origin system for each target axis.

    Consider a lab setup with Motive configured with Y-up and the X- and Z-axis in the horizontal plane. The rigid body
    for the drone was configured such that the forward direction of the drone and the X-axis of the rigid body align.
    For the drone-rigid-body conversion we would then have the drone forward parallel to the body X-axis, its right
    parallel to the Z-axis and its down direction antiparallel to the Y-Axis.
    A desired world coordinate system might have world-X parallel to Motive-Z, world-Y antiparallel to Motive-X and
    world-Z antiparallel to Motive-Y.

    Axes permutations are specified by stating the corresponding Motive axes for each world or drone axis, as a string.
    Antiparallel axis are indicated with a `-` sign.
    So the configuration for the example above would be ``CoordinateConversion("z", "-x", "-y", "x", "z", "-y")``.

    In theory, the axes can be freely chosen, but drone firmware usually expects specific coordinate conventions, such
    as vertical down positive.
    Note that the drone axes are not the current physical rotation of the drone, but rather the conversion between the
    Motive rigid body associated with that drone and the body coordinate system of the drone.
    """

    def __init__(self, world_x: str, world_y: str, world_z: str, drone_forward: str, drone_right: str, drone_down: str):
        """Create the CoordinateConversion.

        Args:
            world_x: Which Motive world axis corresponds to the desired world-X axis.
            world_y: Which Motive world axis corresponds to the desired world-Y axis.
            world_z: Which Motive world axis corresponds to the desired world-Z axis.
            drone_forward: Which Motive rigid body axis corresponds to the drones body X axis.
            drone_right: Which Motive rigid body axis corresponds to the drones body Y axis.
            drone_down: Which Motive rigid body axis corresponds to the drones body Z axis.
        """
        self._choices = ["x", "-x", "y", "-y", "z", "-z"]  #: The possible choices for the inputs.
        self._choices.extend([choice.upper() for choice in self._choices])
        assert world_x in self._choices and world_y in self._choices and world_z in self._choices, \
            f"Invalid axis for coordinate conversion, must be one of {self._choices}"
        self.world_axes = [world_x, world_y, world_z]  #: The axes of the target world coordinate system.
        self.body_axes = [drone_forward, drone_right, drone_down]  #: The axes of the drone coordinate system.
        self.rotation_world: Rotation | None = None  #: Scipy rotation for the motive-world conversion.
        self.rotation_body: Rotation | None = None  #: Scipy rotation for the rigid-body to drone conversion.
        self._make_rotation()

    def convert_euler(self, tracking_pos: np.ndarray, tracking_euler: np.ndarray, in_sequence: str = "XYZ",
                      out_sequence: str = "ZYX", out_degrees: bool = False, in_degrees: bool = True) \
            -> tuple[np.ndarray, list[float]]:
        """Perform the coordinate conversions for a position and euler angles.

        We use the scipy Rotation class for the conversions. For euler angles, the sequence strings define the order in
        which the axis are applied and whether the rotations are extrinsic or intrinsic. See the scipy documentation
        for more details.

        Args:
            tracking_pos: The input position.
            tracking_euler: The input orientation as euler angles.
            in_sequence: The sequence string for the input euler angles.
            out_sequence: The sequence string for the output euler angles.
            out_degrees: Whether the output angles should be in degrees.
            in_degrees: Whether the input angles are in degrees.

        Returns:
            A tuple with the converted position and orientation as euler angles.
        """
        tracking_rot = Rotation.from_euler(in_sequence, tracking_euler, degrees=in_degrees)
        return self._convert(tracking_pos, tracking_rot, out_sequence=out_sequence, degrees=out_degrees)

    def convert_quat(self, tracking_pos: np.ndarray, tracking_quat: np.ndarray,
                     out_sequence: str = "ZYX", degrees: bool = False) \
            -> tuple[np.ndarray, list[float]]:
        """Perform the coordinate conversions for a position and a quaternion.

        We use the scipy Rotation class for the conversions. For euler angles, the sequence strings define the order in
        which the axis are applied and whether the rotations are extrinsic or intrinsic. See the scipy documentation
        for more details.

        Args:
            tracking_pos: The input position.
            tracking_quat: The input orientation as a quaternion.
            out_sequence: The sequence string for the output euler angles.
            degrees: Whether the output angles should be in degrees.

        Returns:
            A tuple with the converted position and orientation as euler angles.
        """
        tracking_rot = Rotation.from_quat(tracking_quat)
        return self._convert(tracking_pos, tracking_rot, out_sequence=out_sequence, degrees=degrees)

    def _convert(self, tracking_pos: np.ndarray, tracking_rot: Rotation, out_sequence: str, degrees: bool = False) \
            -> tuple[np.ndarray, list[float]]:
        """Performs the actual conversion.

        Args:
            tracking_pos: The input position.
            tracking_rot: The input orientation as a Rotation object.
            out_sequence: The sequence string for the output euler angles.
            degrees: Whether the output angles should be in degrees.

        Returns:
            A tuple with the converted position and orientation as euler angles.
        """
        converted_pos = self.rotation_world.apply(tracking_pos)
        converted_rot = (self.rotation_world * tracking_rot * self.rotation_body).as_euler(out_sequence,
                                                                                           degrees=degrees)
        return converted_pos, converted_rot

    def set_rotation_world(self, world_axes: list[str]):
        """Set new axes for the world-Motive coordinate conversion.

        Args:
            world_axes: The new world axes. Should be a list of three strings, e.g. ``["z", "-x", "-y"]``.
        """
        self.world_axes = world_axes
        self._make_rotation()

    def set_rotation_body(self, body_axes: list[str]):
        """Set new axes for the rigid-body-drone coordinate conversion.

        Args:
            body_axes: The new body axes. Should be a list of three strings, e.g. ``["z", "-x", "-y"]``.
        """
        self.body_axes = body_axes
        self._make_rotation()

    def _make_perm_matrix(self, axes: list[str]) -> np.ndarray:
        """Creates a 3x3 permutation/rotation matrix from a list of axes strings.

        Args:
            axes: The axes from which the matrix will be generated.

        Returns:
            The rotation matrix.
        """
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

    def _make_rotation(self):
        """Create the rotation matrices."""
        self.rotation_world = Rotation.from_matrix(self._make_perm_matrix(self.world_axes))
        self.rotation_body = Rotation.from_matrix(self._make_perm_matrix(self.body_axes).T)


class OptitrackPlugin(Plugin):

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
        """Print available rigid bodies on the NatNet server."""
        if self.client is None:
            self.logger.warning("Not connected to a NatNet server!")
            return
        body_str = "\n".join([f"Track ID: {track_id}, Position {array[0]}"
                              for track_id, array in self.available_bodies.items()])
        self.logger.info("Available Rigid Bodies and their Motive positions:\n" + body_str)

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

    def _process_rigid_body(self, track_id: int, position: np.ndarray, rotation: np.ndarray):
        """Handle information for a single rigid body.

        Receives a track ID, position and rotation, performs the coordinate transformation from the optitrack coordinate
        system and then forwards the converted position and rotation to the drone linked with the track id.

        Args:
            track_id: The track id whose position and rotation we are receiving.
            position: The drone position received from Optitrack.
            rotation: The drone rotation received from Optitrack.
        """
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
        """Close the plugin, removing the NatNet callback and shutting down the NatNet client."""
        if self.client is not None:
            self.client.new_frame_with_data_listener = None
        self._stopping = True
        await super().close()
        if self.client is not None:
            self.client.shutdown()

    async def _error_wrapper(self, func: Callable, err_count: list[int], *args: any, **kwargs: any)\
            -> MocapResult | bool:
        """Wrapper for MAVSDK mocap functions.

        Intended for use with MAVSDK mocap coroutines. Catches exceptions and returns False instead.

        Args:
            func: The coroutine, as a callable.
            err_count: Single entry list with the error count. As a list to pass by reference.
            *args: Passed to the coroutine being executed.
            **kwargs: Passed to the coroutine being executed.

        Returns:
            The result of the mocap functions, or False if ``MocapError`` was raised.
        """
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

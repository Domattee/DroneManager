"""Module for pytest fixtures used by many tests."""
import asyncio
import logging
import numpy as np
import pytest
import pygame
import struct
from typing import AsyncGenerator, Any, Awaitable, Callable, Generator
from unittest.mock import Mock, AsyncMock

import cv2
from mavsdk import System

from dronemanager.core import DroneManager
from dronemanager.drone import DroneMAVSDK, DroneConfig, FlightMode, DroneParams
from dronemanager.navigation.core import PathGenerator, PathFollower, Waypoint, WayPointType
from dronemanager.navigation.rectlocalfence import RectLocalFence

from sitl import Px4Sitl, SitlSetupError, SITL_DRONE, px4_log_excerpt, wait_until


pygame.init()


@pytest.fixture
async def dm() -> AsyncGenerator[DroneManager, Any]:
    """Create DroneManager object for tests.

    Yields:
        A DroneManager instance.
    """
    drone_type = DroneMAVSDK
    dm = DroneManager(drone_type, log_to_console=False)
    yield dm
    await dm.close()


@pytest.fixture
def px4_sitl() -> Generator[Px4Sitl, Any, None]:
    """Start a fresh PX4 SITL instance for a test.

    Every test gets its own PX4 with default parameters, starting at the origin. Besides isolating the tests, this is
    necessary because PX4 only answers the first GCS address and port it hears from. With WSL's NAT networking,
    DroneManager connects from a new port each time, so a second connection to the same PX4 would get no answers.

    Fails the requesting tests with setup instructions if PX4 isn't available. On Windows, PX4 runs in WSL.

    Yields:
        The running SITL instance.
    """
    sitl = Px4Sitl(instance=0)
    try:
        sitl.start()
    except SitlSetupError as e:
        sitl.stop()
        pytest.fail(str(e), pytrace=False)
    yield sitl
    sitl.stop()


async def _connect_sitl_drone(dm: DroneManager, name: str, sitl: Px4Sitl):
    """Connect a SITL drone and wait until it has parameters and a GPS fix.

    Fails the test if that isn't possible.

    Args:
        dm: The DroneManager to connect with.
        name: The name for the drone.
        sitl: The running SITL instance.
    """
    connected = await dm.connect_to_drone(name, drone_address=sitl.address(), timeout=60, log_telemetry=False)
    if not connected:
        await dm.close()
        pytest.fail(f"Couldn't connect to PX4 SITL at {sitl.address()}.\n{px4_log_excerpt(sitl)}")
    drone = dm.drones[name]

    def drone_ready() -> bool:
        """Check that parameters are loaded and the drone has a GPS fix and position.

        Returns:
            Whether the drone is ready for the test.
        """
        return (drone.parameters_loaded and drone.fix_type is not None and drone.fix_type.value >= 3
                and abs(drone.position_global[0]) > 0)

    if not await wait_until(drone_ready, timeout=60):
        await dm.close()
        pytest.fail(f"SITL drone {name} didn't get parameters and a GPS fix.\n{px4_log_excerpt(sitl)}")


async def _land_and_close(dm: DroneManager):
    """Land and disarm all drones after a test, then close the DroneManager.

    Uses PX4's own landing, so the cleanup doesn't depend on the DroneManager code under test.

    Args:
        dm: The DroneManager.
    """
    flying = [name for name, drone in dm.drones.items() if drone.in_air]
    if flying:
        await dm.change_flightmode(flying, "land")
        if not await wait_until(lambda: not any(dm.drones[name].in_air for name in flying if name in dm.drones), 60):
            logging.warning("Landing the SITL drones after the test timed out.")
    armed = [name for name, drone in dm.drones.items() if drone.is_armed]
    if armed:
        await dm.disarm(armed)
    await dm.close()


@pytest.fixture
async def sitl_dm(px4_sitl: Px4Sitl) -> AsyncGenerator[DroneManager, Any]:
    """Create a DroneManager connected to the SITL drone, which is disarmed on the ground.

    The drone is called :py:data:`SITL_DRONE`. After the test, it is landed and disarmed if necessary.

    Args:
        px4_sitl: The running SITL instance.

    Yields:
        The DroneManager with the connected drone.
    """
    dm = DroneManager(DroneMAVSDK, log_to_console=False)
    await _connect_sitl_drone(dm, SITL_DRONE, px4_sitl)
    yield dm
    await _land_and_close(dm)


@pytest.fixture
async def sitl_fleet() -> AsyncGenerator[Callable[[int], Awaitable[DroneManager]], Any]:
    """Start several PX4 SITL instances and connect them to one DroneManager.

    Yields an async function that takes the number of drones and returns the DroneManager. The drones are called
    "sim0", "sim1" and so on. Each runs in its own PX4 instance with its own ports and MAVLink system ID.

    Yields:
        The function to start the fleet.
    """
    instances: list[Px4Sitl] = []
    managers: list[DroneManager] = []

    async def start_fleet(n_drones: int) -> DroneManager:
        """Start the PX4 instances and connect them.

        Args:
            n_drones: The number of drones.

        Returns:
            The DroneManager with the connected drones.
        """
        dm = DroneManager(DroneMAVSDK, log_to_console=False)
        managers.append(dm)
        for instance in range(n_drones):
            sitl = Px4Sitl(instance=instance)
            instances.append(sitl)
            try:
                # Starting blocks until PX4 is ready, so run it in a thread to keep the other drones' telemetry going
                await asyncio.to_thread(sitl.start)
            except SitlSetupError as e:
                pytest.fail(str(e), pytrace=False)
            await _connect_sitl_drone(dm, f"sim{instance}", sitl)
        return dm

    yield start_fleet
    for dm in managers:
        await _land_and_close(dm)
    for sitl in instances:
        sitl.stop()


class TCPStreamer:
    """A dummy TCP video streaming server.

    Listens for clients and sends random images of the set size to them at the set frequency.
    """
    def __init__(self):
        """Create the TCPStreamer instance."""
        self.ip: str = "127.0.0.1"  #: IP for the server.
        self.port: int = 5000  #: Port for the server
        self.frequency: float = 5  #: Frequency of the images.
        self.img_size = (360, 240, 3)  #: Size of the dummy image.

    async def start(self):
        """Start the TCP server."""
        logging.info("Starting TCPStreamer server.")
        server = await asyncio.start_server(self.send_images, self.ip, self.port)
        async with server:
            await server.serve_forever()

    async def send_images(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        """Send images at the set frequency.

        The reader is currently not used.

        Args:
            reader: Incoming stream.
            writer: Outgoing stream.
        """
        try:
            while True:
                img_data = np.random.randint(0, 256, self.img_size, dtype=np.uint8)
                res, encoded = cv2.imencode(".png", img_data)
                if not res:
                    logging.error("Couldn't encode dummy image.")
                img_bytes = encoded.tobytes()
                img_length = struct.pack("<I", len(img_bytes))

                writer.write(img_length)
                writer.write(img_bytes)
                await writer.drain()

                await asyncio.sleep(1/self.frequency)
        except (asyncio.CancelledError, ConnectionAbortedError, ConnectionResetError):
            pass
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:
                pass


@pytest.fixture
async def video_stream_source() -> AsyncGenerator[TCPStreamer, Any]:
    """Creates a dummy video stream source.

    Yields:
        The dummy tcp image streamer.
    """
    server = TCPStreamer()
    task = asyncio.create_task(server.start())
    yield server
    task.cancel()


@pytest.fixture
def mock_drone_getter() -> Callable[[int], Mock]:
    """Fixture that allows creating an arbitrary number of distinct mock drone object.

    Mocks a bunch of the individual components for each drone

    Returns:
        A Mock object specced to and wrapping DroneMAVSDK
    """
    def _mock_drone_creator(count: int) -> Mock:
        """Drone creation function.

        Args:
            count: A unique number for the mock drone. Used to ensure name uniqueness.

        Returns:
            A mock drone object with name "mock_<count>"
        """
        name = f"mock_{count}"
        # Core mocks
        mockdrone = Mock(spec=DroneMAVSDK)
        mockdrone.disconnect = AsyncMock()
        mockdrone.stop_execution = AsyncMock()
        mockdrone.system = Mock(spec=System)
        mockdrone.name = name

        # Optitrack mocks
        mockdrone.send_external_tracking_data = AsyncMock()
        mockdrone.system.mocap.set_vision_position_estimate = AsyncMock()

        # Drone property mocks
        mockdrone.path_generator = Mock(spec=PathGenerator)
        mockdrone.path_generator.target_position = Waypoint(WayPointType.POS_NED, pos=[2, 3, -6], yaw=90)
        mockdrone.path_follower = Mock(spec=PathFollower)
        mockdrone.config = DroneConfig(name, address="dummy_address")
        mockdrone.position_ned = np.asarray([1, 1, -2], dtype=np.float64)
        mockdrone.position_global = np.asarray([-1, -1, 300], dtype=np.float64)
        mockdrone.velocity = np.asarray([0.1, 0.2, 0.3], dtype=np.float64)
        mockdrone.attitude = np.asarray([5.0, 10.0, 20.0], dtype=np.float64)
        mockdrone.flightmode = FlightMode.HOLD
        mockdrone.is_connected = True
        mockdrone.is_armed = True
        mockdrone.in_air = True
        mockdrone.fence = RectLocalFence(0, 3, -1, 4, -10, -1)
        mockdrone.drone_params = DroneParams()
        mockdrone.drone_params.max_h_vel = 3
        mockdrone.drone_params.max_up_vel = 1
        mockdrone.drone_params.max_down_vel = 1
        mockdrone.drone_params.max_yaw_rate = 30

        # Function mocks
        mockdrone.arm = AsyncMock()

        def flight_mode_change_posctrl():
            """Mock function which changes the flight mode posctrl."""
            mockdrone.flightmode = FlightMode.POSCTL

        def change_flight_mode_side_effect(flightmode: FlightMode):
            """Mock flight mode chaning function.

            Args:
                flightmode: The new flight mode.
            """
            mockdrone.flightmode = flightmode

        mockdrone.manual_control_position.side_effect = flight_mode_change_posctrl
        mockdrone.set_manual_control_input = AsyncMock()
        mockdrone.execute_task = AsyncMock()
        mockdrone.change_flight_mode.side_effect = change_flight_mode_side_effect

        return mockdrone
    return _mock_drone_creator

"""Pipeline tests for the path generators, path followers and fences with a simulated PX4 drone.

DroneManager plans and follows paths with exchangeable generators and followers. The default combination is tested in
``test_flight.py``. These tests swap in the other implementations, as described in the developer guide, and fly with
them.
"""
import asyncio
from typing import Callable

import numpy as np
import pytest

from dronemanager.core import DroneManager
from dronemanager.drone import DroneMAVSDK
from dronemanager.navigation.core import PathFollower, PathGenerator, WayPointType
from dronemanager.navigation.directsetpointfollower import DirectSetpointFollower
from dronemanager.navigation.directtargetgenerator import DirectTargetGenerator
from dronemanager.navigation.gmp3generator import GMP3Generator
from dronemanager.navigation.rectlocalfence import RectLocalFence
from dronemanager.navigation.ruckigfollower import RuckigOfflineFollower, RuckigOnlineFollower
from dronemanager.navigation.velocityfollower import VelocityFollower

from sitl import SITL_DRONE, wait_until

pytestmark = pytest.mark.sitl

ALTITUDE = 3.0
"""Flight altitude for the tests, in meters."""
POSITION_TOLERANCE = 0.5
"""How close the drone must get to a target, in meters."""


def _direct_setpoints(drone: DroneMAVSDK) -> tuple[PathGenerator, PathFollower]:
    """Send the target directly as a position setpoint.

    Args:
        drone: The drone.

    Returns:
        The generator and follower.
    """
    return (DirectTargetGenerator(drone, drone.logger, WayPointType.POS_NED),
            DirectSetpointFollower(drone, drone.logger, 1 / drone.position_update_rate, WayPointType.POS_NED))


def _velocity(drone: DroneMAVSDK) -> tuple[PathGenerator, PathFollower]:
    """Fly to the target with velocity setpoints.

    Args:
        drone: The drone.

    Returns:
        The generator and follower.
    """
    return (DirectTargetGenerator(drone, drone.logger, WayPointType.POS_NED),
            VelocityFollower(drone, drone.logger, 1 / drone.position_update_rate))


def _ruckig_online(drone: DroneMAVSDK) -> tuple[PathGenerator, PathFollower]:
    """Fly to the target with a trajectory that Ruckig recomputes at every step.

    Args:
        drone: The drone.

    Returns:
        The generator and follower.
    """
    return (DirectTargetGenerator(drone, drone.logger, WayPointType.POS_NED),
            RuckigOnlineFollower(drone, drone.logger, 1 / drone.position_update_rate, WayPointType.POS_VEL_ACC_NED))


def _gmp3(drone: DroneMAVSDK) -> tuple[PathGenerator, PathFollower]:
    """Plan a path around obstacles with GMP3 and follow it with Ruckig.

    Args:
        drone: The drone.

    Returns:
        The generator and follower.
    """
    return (GMP3Generator(drone, 1 / drone.position_update_rate, drone.logger),
            RuckigOfflineFollower(drone, drone.logger, 1 / drone.position_update_rate, WayPointType.POS_VEL_ACC_NED))


async def _takeoff(dm: DroneManager):
    """Arm the drone and take off.

    Args:
        dm: The DroneManager.
    """
    assert await asyncio.wait_for(dm.arm(SITL_DRONE), 30) == [True]
    assert await asyncio.wait_for(dm.takeoff(SITL_DRONE, altitude=ALTITUDE), 60) == [True]


async def _use_navigation(drone: DroneMAVSDK, setup: Callable[[DroneMAVSDK], tuple[PathGenerator, PathFollower]]):
    """Replace the path generator and follower of a drone.

    Args:
        drone: The drone.
        setup: Function creating the new generator and follower.
    """
    if drone.path_follower.is_active:
        await drone.path_follower.deactivate()
    drone.path_generator, drone.path_follower = setup(drone)


@pytest.mark.parametrize("setup", [_direct_setpoints, _velocity, _ruckig_online, _gmp3],
                         ids=["direct_setpoints", "velocity", "ruckig_online", "gmp3"])
async def test_fly_with_navigation(sitl_dm: DroneManager, setup: Callable[[DroneMAVSDK],
                                                                          tuple[PathGenerator, PathFollower]]):
    """The drone reaches local targets with each combination of path generator and follower.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
        setup: Function creating the generator and follower.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    await _takeoff(sitl_dm)
    await _use_navigation(drone, setup)
    start = drone.position_ned.copy()
    for offset in ([4.0, 0.0, 0.0], [4.0, 3.0, 0.0]):
        target = np.array([start[0] + offset[0], start[1] + offset[1], -ALTITUDE])
        result = await asyncio.wait_for(sitl_dm.fly_to(SITL_DRONE, local=target, yaw=0, tol=POSITION_TOLERANCE,
                                                       schedule=False), 90)
        assert result == [True]
        assert drone.position_ned == pytest.approx(target, abs=POSITION_TOLERANCE)


async def test_fence_rejects_targets_outside(sitl_dm: DroneManager):
    """With a fence, targets outside it are rejected and the drone stays where it is.

    Args:
        sitl_dm: DroneManager connected to the SITL drone.
    """
    drone = sitl_dm.drones[SITL_DRONE]
    await _takeoff(sitl_dm)
    start = drone.position_ned.copy()
    fence = RectLocalFence(start[0] - 5, start[0] + 5, start[1] - 5, start[1] + 5, -6, 0)
    sitl_dm.set_fence(SITL_DRONE, fence)
    assert drone.fence is fence

    inside = np.array([start[0] + 3, start[1], -ALTITUDE])
    assert await asyncio.wait_for(sitl_dm.fly_to(SITL_DRONE, local=inside, yaw=0, schedule=False), 60) == [True]

    outside = np.array([start[0] + 20, start[1], -ALTITUDE])
    results = await asyncio.wait_for(sitl_dm.fly_to(SITL_DRONE, local=outside, yaw=0, schedule=False), 60)
    assert isinstance(results[0], AssertionError)
    await asyncio.sleep(2)
    assert drone.position_ned == pytest.approx(inside, abs=POSITION_TOLERANCE)

    sitl_dm.set_fence(SITL_DRONE, None)
    assert await wait_until(lambda: drone.fence is None, 1)

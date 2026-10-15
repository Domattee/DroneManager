"""Pipeline tests with several simulated PX4 drones: fleet commands and the UAM demo mission.

Each drone runs in its own PX4 instance. All instances start at the same place, but each has its own local coordinate
system with the origin at its start position, so local targets are relative to where each drone started.
"""
import asyncio
from typing import Awaitable, Callable

import numpy as np
import pytest

from dronemanager.core import DroneManager
from dronemanager.missions import uam
from dronemanager.missions.uam import UAMFlightArea, UAMStages
from dronemanager.navigation.rectlocalfence import RectLocalFence

from sitl import wait_until

pytestmark = pytest.mark.sitl

HOVER_TOLERANCE = 1.0
"""How close drones that already reached their target must still be to it, in meters. The fleet waits for the slowest
drone, while the others hover with SIH's position noise. The targets are 3 m apart, so this still identifies them."""
LANDED_TOLERANCE = 2.0
"""How close the drones must be to their start positions after landing, in meters. SIH simulates GPS and estimator
noise, and the estimated position of a drone on the ground keeps wandering, by more than a meter over a minute. The
drones land one after another, so this only separates "returned to base" from "stayed in the field", where the point
of interest is 5 m away from the start positions."""
PX4_LANDED_DELAY = 15
"""How long PX4's land detector may take to agree after DroneManager reported a landing, in seconds."""
MISSION_ALTITUDE = 2.0
"""Flight altitude of the UAM mission in the tests, in meters. The mission flies at 1 m above the local origin, which
is set up for the indoor demo. In SIH, the origin isn't exactly at ground level, so 1 m above it can be little more
than half a meter above the ground, low enough for PX4's land detector to trigger mid-flight."""


async def _load_uam(dm: DroneManager, drones: list[str]) -> uam.UAMMission:
    """Load the UAM mission and add the drones to it.

    Args:
        dm: The DroneManager.
        drones: The names of the drones for the mission.

    Returns:
        The mission.
    """
    await dm.load("mission")
    await getattr(dm, "mission").load("uam")
    mission: uam.UAMMission = getattr(dm, "uam")
    # Fly higher than in the indoor demo, see MISSION_ALTITUDE. Must happen before adding drones, which sets the fence.
    mission.flight_altitude = MISSION_ALTITUDE
    mission.swap_altitude = MISSION_ALTITUDE
    mission.poi_position[2] = -MISSION_ALTITUDE
    mission.holding_position[2] = -MISSION_ALTITUDE
    mission.flight_area = UAMFlightArea(-3.5, 3.5, -1.5, 1.5, MISSION_ALTITUDE + 1)
    mission.fence = RectLocalFence(*mission.flight_area.bounding_box, safety_level=3)
    assert await mission.add_drones(drones)
    assert await asyncio.wait_for(mission.reset(), 300)
    assert mission.current_stage is UAMStages.Start
    assert mission.ready()
    for drone in drones:
        position = dm.drones[drone].position_ned
        assert position[:2] == pytest.approx([mission.start_position_x, mission.start_positions_y[drone]],
                                             abs=LANDED_TOLERANCE)
        assert await wait_until(lambda: not dm.drones[drone].in_air, PX4_LANDED_DELAY)
    return mission


async def _return_to_base(dm: DroneManager, mission: uam.UAMMission):
    """Stop the mission and wait until all drones have landed, the returning ones at their start positions.

    Only the drones that were still flying are checked for their position. Drones that landed earlier in the mission
    have been on the ground for minutes, and the estimated position of a landed drone wanders in SIH.

    Args:
        dm: The DroneManager.
        mission: The mission.
    """
    returning = set(mission.flying_drones)
    assert returning, "No drone was flying anymore when returning to base."
    await mission.rtb()
    assert await wait_until(lambda: mission.current_stage is UAMStages.Start, 300), mission.current_stage
    for name in mission.drones:
        drone = dm.drones[name]
        assert await wait_until(lambda: not drone.in_air, PX4_LANDED_DELAY)
        if name in returning:
            start = [mission.start_position_x, mission.start_positions_y[name]]
            assert drone.position_ned[:2] == pytest.approx(start, abs=LANDED_TOLERANCE)


async def test_fleet_commands(sitl_fleet: Callable[[int], Awaitable[DroneManager]]):
    """Several drones are armed, take off, fly to their own targets and land together.

    Args:
        sitl_fleet: Function starting the simulated drones.
    """
    dm = await sitl_fleet(3)
    names = ["sim0", "sim1", "sim2"]
    assert await asyncio.wait_for(dm.arm(names), 30) == [True] * 3
    assert await asyncio.wait_for(dm.takeoff(names, altitude=3.0), 60) == [True] * 3
    assert all(dm.drones[name].in_air for name in names)

    targets = [dm.drones[name].position_ned + np.array([3.0 * (i + 1), -2.0 * i, 0.0]) for i, name in enumerate(names)]
    yaws = [0.0, 90.0, -90.0]
    results = await asyncio.wait_for(dm.fly_to(names, local=targets, yaw=yaws, tol=[0.25] * 3, schedule=False), 90)
    assert results == [True] * 3
    for name, target, yaw in zip(names, targets, yaws):
        assert dm.drones[name].position_ned == pytest.approx(target, abs=HOVER_TOLERANCE)
        assert dm.drones[name].attitude[2] == pytest.approx(yaw, abs=5)

    await asyncio.wait_for(dm.land(names), 120)
    # PX4 SIH sometimes doesn't detect the landing in offboard mode, see _finish_landing in test_flight.py
    await asyncio.wait_for(dm.change_flightmode(names, "land"), 30)
    assert await wait_until(lambda: not any(dm.drones[name].is_armed for name in names), 30)


async def test_uam_single_search(sitl_fleet: Callable[[int], Awaitable[DroneManager]]):
    """One drone searches the area, finds the point of interest, observes it and returns to base.

    Args:
        sitl_fleet: Function starting the simulated drones.
    """
    dm = await sitl_fleet(1)
    mission = await _load_uam(dm, ["sim0"])

    await mission.single_search()
    assert mission.current_stage is UAMStages.SearchSingle
    assert await wait_until(lambda: mission.current_stage is UAMStages.Observation, 300), mission.current_stage
    assert mission._observing_drone == "sim0"
    poi_distance = np.linalg.norm(dm.drones["sim0"].position_ned[:2] - np.array(mission.poi_position[:2]))
    assert poi_distance < mission.POI_distance

    await asyncio.sleep(10)  # Circle around the point of interest for a while
    assert dm.drones["sim0"].in_air
    await _return_to_base(dm, mission)


async def test_uam_group_search_with_battery_swap(sitl_fleet: Callable[[int], Awaitable[DroneManager]],
                                                  monkeypatch: pytest.MonkeyPatch):
    """Three drones search together, one observes the point of interest and is relieved when its battery runs low.

    The simulated battery drains much faster than in the demo, so the swap happens within the test.

    Args:
        sitl_fleet: Function starting the simulated drones.
        monkeypatch: Monkeypatch fixture.
    """
    monkeypatch.setattr(uam.FakeBattery, "TIME_SCALE", 45)
    names = ["sim0", "sim1", "sim2"]
    dm = await sitl_fleet(3)
    mission = await _load_uam(dm, names)

    await mission.group_search()
    assert mission.current_stage is UAMStages.SearchGroup
    assert await wait_until(lambda: mission.current_stage is UAMStages.Observation, 300), mission.current_stage
    first_observer = mission._observing_drone
    assert first_observer in names

    # The observing drone's battery runs low and another drone takes over
    assert await wait_until(lambda: mission._observing_drone not in (None, first_observer), 400)
    assert await wait_until(lambda: not dm.drones[first_observer].in_air, 120)
    assert dm.drones[mission._observing_drone].in_air

    # Returning to base is tested with the single search. Here, drones that landed earlier have been on the ground for
    # minutes, and their estimated positions wander too much in SIH to plan a return from them reliably.
    await mission.set_uninit()
    await asyncio.sleep(1)  # Let the mission cancel its drone tasks
    flying = [name for name in names if dm.drones[name].in_air]
    await asyncio.wait_for(dm.land(flying), 120)
    await asyncio.wait_for(dm.change_flightmode(flying, "land"), 30)
    assert await wait_until(lambda: not any(dm.drones[name].in_air for name in names), PX4_LANDED_DELAY)

"""Unit tests for the drone classes that don't need a drone."""
import asyncio
import logging

import numpy as np
import pytest

from dronemanager.core import DroneManager
from dronemanager.drone import Drone, LandingDetector


class QueueDrone(Drone):
    """A drone without a connection, to test the action queue of the base class."""


QueueDrone.__abstractmethods__ = frozenset()


RATE = 10
"""Sample rate of the synthetic telemetry in Hz."""
DESCENT_RATE = 0.6
"""Descent rate in the air in m/s. With the default landing descent rate of 0.3 m/s, PX4 descends at about 0.6 m/s, as
the position setpoint is below the drone."""


def _first_landed(trace: list[float]) -> float | None:
    """Feed a trace to a LandingDetector with the default settings and return when it first reports a landing.

    Args:
        trace: Samples of the NED down position at :py:data:`RATE`.

    Returns:
        The time of the first sample reported as landed, or None.
    """
    detector = LandingDetector()
    for i, z in enumerate(trace):
        if detector.update(i / RATE, z):
            return i / RATE
    return None


def _descent(start_z: float, duration: float, rate: float = DESCENT_RATE) -> list[float]:
    """A steady descent, or a drift if the rate is small.

    Args:
        start_z: Starting NED down position.
        duration: Duration in seconds.
        rate: Descent rate in m/s.

    Returns:
        The samples.
    """
    return [start_z + rate * i / RATE for i in range(int(duration * RATE))]


def test_landing_detected_on_the_ground():
    """A drone that descends and then sits on the ground is landed within the window after touching down."""
    trace = _descent(-6, 10) + _descent(0, 5, rate=0)
    landed = _first_landed(trace)
    assert landed is not None and 10 < landed <= 10 + LandingDetector().window


def test_noisy_descent_is_not_a_landing():
    """Position noise during a descent, with upward ticks between samples, isn't a landing."""
    rng = np.random.default_rng(0)
    trace = [z + rng.normal(0, 0.05) for z in _descent(-20, 30)]
    assert _first_landed(trace) is None


def test_short_stall_in_the_air_is_not_a_landing():
    """A stall of the descent for a second isn't a landing."""
    trace = _descent(-6, 3) + _descent(-4.2, 1, rate=0) + _descent(-4.2, 5)
    assert _first_landed(trace) is None


def test_drift_on_the_ground_is_a_landing():
    """The estimated altitude can drift after touching down, as recorded in SITL. This is still a landing."""
    for drift_rate in (-0.2, 0.13):
        trace = _descent(-6, 10) + _descent(0, 5, rate=drift_rate)
        landed = _first_landed(trace)
        assert landed is not None and 10 < landed <= 10 + LandingDetector().window


def test_noisy_ground_is_a_landing():
    """Position noise on the ground doesn't prevent detecting the landing."""
    rng = np.random.default_rng(1)
    trace = _descent(-6, 10) + [rng.normal(0, 0.05) for _ in range(5 * RATE)]
    landed = _first_landed(trace)
    assert landed is not None and landed <= 10 + 2 * LandingDetector().window


async def test_replaced_actions_cancel_the_callers_quietly(dm: DroneManager, caplog: pytest.LogCaptureFixture):
    """Callers waiting for an action that was replaced or removed from the queue are cancelled, without warnings.

    Args:
        dm: DroneManager instance.
        caplog: Captures the log messages.
    """
    caplog.set_level(logging.DEBUG)
    drone = QueueDrone("queue_drone", log_to_file=False)
    dm.drones[drone.name] = drone
    try:
        running = asyncio.create_task(dm.wait(drone.name, 10.0))
        queued = asyncio.create_task(dm.wait(drone.name, 10.0))
        await asyncio.sleep(0.5)
        assert len(drone.action_queue) == 1
        assert await asyncio.wait_for(dm.wait(drone.name, 0.1, schedule=False), 5) == [None]
        for replaced in (running, queued):
            with pytest.raises(asyncio.CancelledError):
                await asyncio.wait_for(replaced, 5)
        assert any("Cancelled actions" in record.message for record in caplog.records)
        assert not [record for record in caplog.records if record.levelno >= logging.WARNING]
    finally:
        dm.drones.pop(drone.name)
        for task in drone.running_tasks:
            task.cancel()

import logging
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dronemanager.plugins.rtcm3 import RTCM3Plugin


class DummyDrone:
    def __init__(self):
        self.is_connected = True
        self.mav_conn = None


class DummyDM:
    def __init__(self):
        self.drones = {}
        self.published = []

    def publish(self, topic, payload):
        self.published.append((topic, payload))


@pytest.fixture
def plugin():
    dm = DummyDM()
    logger = logging.getLogger("test_rtcm")
    plugin = RTCM3Plugin(dm, logger, "rtcm-test")
    return plugin, dm


def test_chunking_splits_large_packets(plugin):
    plugin_obj, _ = plugin
    payload = bytes(range(250))

    chunks = plugin_obj._chunk_packet(payload)

    assert chunks == [payload[:180], payload[180:]]


def test_forwarding_uses_connected_drones(plugin):
    plugin_obj, dm = plugin
    drone = DummyDrone()
    sent = []

    class DummyMavConn:
        def __init__(self):
            self.sent = []
            self.con_drone_in = type("Conn", (), {})()
            self.con_drone_in.mav = type("Mav", (), {"gps_rtcm_data_encode": lambda self, flags, length, data: {"flags": flags, "length": length, "data": data}})()

        def send_as_gcs(self, msg):
            self.sent.append(msg)

    drone.mav_conn = DummyMavConn()
    dm.drones["alpha"] = drone

    plugin_obj._forward_to_drones(b"abc")

    assert len(drone.mav_conn.sent) == 1

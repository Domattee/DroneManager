""" Plugins for communication to other software
"""
import asyncio
import socket
import json

from dronecontrol.plugin import Plugin

# TODO: Different process, maybe wait for client to ask for info, then start sending?


class UDPPlugin(Plugin):

    def __init__(self, dm, logger):
        super().__init__(dm, logger)
        self.port = 31659
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket = sock
        self.frequency = 5
        self.background_functions = [
            self._send_continuously()
        ]
        
    def _send_msg(self, msg: str):
        self.socket.sendto(msg.encode("utf-8"), ("localhost", self.port))

    async def _send_continuously(self):
        while True:
            try:
                await asyncio.sleep(1 / self.frequency)
                json_str = self._make_json()
                self.logger.debug(f"Sending json {json_str}")
                self._send_msg(json_str)
            except Exception as e:
                self.logger.debug("Exception sending data out over UDP!")
                self.logger.debug(repr(e), exc_info=True)

    def _make_json(self):
        drone_data = {}
        mission_data = {}
        for drone_name in self.dm.drones:
            drone = self.dm.drones[drone_name]
            drone_data[drone_name] = {
                "position": drone.position_ned.tolist(),
                "heading": drone.attitude[2],
                "conn": drone.is_connected,
                "armed": drone.is_armed,
                "in_air": drone.in_air,
            }
        if hasattr(self.dm, "mission"):  # Check that the mission plugin is actually loaded
            for mission_name in self.dm.mission.missions:
                mission = self.dm.mission.missions[mission_name]
                mission_data[mission.PREFIX] = {
                    "stage": mission.current_stage.name,
                    "drones": list(mission.drones.keys()),
                    "bat": {name: battery.level for name, battery in mission.batteries.items()},
                }
        data = {
            "drones": drone_data,
            "missions": mission_data,
        }
        return json.dumps(data)

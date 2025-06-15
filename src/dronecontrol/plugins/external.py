""" Plugins for communication to other software
"""
import asyncio
import socket
import json

from dronecontrol.plugin import Plugin

# TODO: Different process, maybe wait for client to ask for info, then start sending?


class UDPPlugin(Plugin):

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.port = 31659
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket = sock
        self.frequency = 5
        self.background_functions = [
            self._send_continuously()
        ]

    def _send_msg(self, msg: str):
        try:
            self.socket.sendto(msg.encode("utf-8"), ("localhost", self.port))
        except Exception as e:
            self.logger.warning("Exception sending out data! Check the log for details.")
            self.logger.debug(repr(e), exc_info=True)

    async def _send_continuously(self):
        while True:
            try:
                await asyncio.sleep(1 / self.frequency)
                json_str = self._make_json()
                self.logger.debug(f"Sending json {json_str}")
                self._send_msg(json_str)
            except Exception as e:
                self.logger.debug("Exception sending data out over UDP! Check the log for details.")
                self.logger.debug(repr(e), exc_info=True)

    def _make_json(self):
        drone_data = {}
        for drone_name in self.dm.drones:
            drone = self.dm.drones[drone_name]
            drone_data[drone_name] = {
                "position": drone.position_ned.tolist(),
                "speed": drone.speed,
                "heading": drone.attitude[2],
                "mode": drone.flightmode.name,
                "conn": drone.is_connected,
                "armed": drone.is_armed,
                "in_air": drone.in_air,
            }
        data = {"drones": drone_data}
        if hasattr(self.dm, "mission"):  # Check that the mission plugin is actually loaded
            mission_data = {}
            data["missions"] = mission_data
            for mission_name in self.dm.mission.missions:
                mission = self.dm.mission.missions[mission_name]
                mission_data[mission.PREFIX] = {
                    "flightarea": mission.flight_area.boundary_list(),
                    "stage": mission.current_stage.name,
                    "drones": list(mission.drones.keys()),
                }
                for info, func in mission.additional_info.items():
                    try:
                        mission_data[mission.PREFIX][info] = func()
                    except Exception as e:
                        self.logger.warning("Couldn't collect all mission information to send out due to an exception!")
                        self.logger.debug(repr(e), exc_info=True)
        return json.dumps(data)

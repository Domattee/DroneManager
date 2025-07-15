""" Plugins for communication to other software

Currently only features a basic UDP server which sends data on connected drones and running missions in a json format.
"""
import asyncio
import socket
import json
import time
import math

from dronecontrol.plugin import Plugin
from dronecontrol.utils import coroutine_awaiter


class UDPClient:

    def __init__(self, ip, port, frequency, duration):
        self.start_time = time.time()
        self.ip = ip
        self.port = port
        self.frequency = frequency
        self.duration = duration


class UDPPlugin(Plugin):
    """ Communication happens over port 31659. A client will send a json message with the desired frequency and duration
    (in seconds) to this port and the server starts answering. A duration of 0 means infinite. Frequency is capped between 1/60 and 20Hz.

    Example message from client:
    {
      "duration": 30,
      "frequency": 5
    }

    """

    def __init__(self, dm, logger, name):
        super().__init__(dm, logger, name)
        self.inport = 31659
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.bind(("", self.inport))
        self.socket = sock

        outsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.outsocket = outsock
        self.frequency = 5
        self.background_functions = [
            #self._send_continuously(),
            self._listen_for_clients(),
        ]

    async def _listen_for_clients(self):
        self.logger.debug("Listening for clients...")
        while True:
            try:
                rec_task = asyncio.get_running_loop().sock_recvfrom(self.socket, 1024)
                self._running_tasks.add(rec_task)
                msg, addr = await rec_task
                self.logger.debug(f"Received message from {addr}")
                json_dict = json.loads(msg)
                if "frequency" not in json_dict or "duration" not in json_dict:
                    self.logger.debug(f"Invalid JSON message received from {addr}")
                    continue
                ip, port = addr
                frequency = json_dict["frequency"]
                if frequency > 20:
                    frequency = 20
                elif frequency < 1/60:
                    frequency = 1/60
                duration = json_dict["duration"]
                if duration <= 0:
                    duration = 0
                client = UDPClient(ip, port, frequency, duration)
                send_task = asyncio.create_task(self._client_sender(client))
                awaiter_task = asyncio.create_task(coroutine_awaiter(send_task, self.logger))
                self._running_tasks.add(send_task)
                self._running_tasks.add(awaiter_task)
                self.logger.info(f"New client @{ip, port} with frequency {frequency} and duration {math.inf if duration == 0 else duration}.")
            except TimeoutError:
                pass
            except Exception as e:
                self.logger.warning("Exception listening for incoming UDP!")
                self.logger.debug(repr(e), exc_info=True)

    async def _client_sender(self, client: UDPClient):
        """ Send data to the client.
        """
        # TODO: If we ever have a larger number of clients we should cache the jsons somehow
        # TODO: The OSError for the send command is raised at the recvfrom for some reason, which breaks this client handling
        while client.duration == 0 or time.time() < (client.start_time + client.duration):
            try:
                data = self._make_json()
                self._send_msg(data, client.ip, client.port)
                await asyncio.sleep(1/client.frequency)
            except OSError as e:
                self.logger.info("Couldn't send information, closing connection...")
                self.logger.info(f"{e.errno}: {e.strerror}")
                break
            except Exception as e:
                self.logger.warning("Exception sending data out over UDP! Check the log for details.")
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

    def _send_msg(self, msg: str, ip="localhost", port=None):
        if port is None:
            port = self.inport
        try:
            self.outsocket.sendto(msg.encode("utf-8"), (ip, port))
        except Exception as e:
            self.logger.warning("Exception sending out data! Check the log for details.")
            self.logger.debug(repr(e), exc_info=True)

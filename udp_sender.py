""" Plugins for communication to other software
"""
import asyncio
import socket
import json
import time
import math


class UDPSender:

    def __init__(self):
        self.port = 31659
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket = sock
        self.frequency = 5
        self.start = time.time()

    async def send(self):
        while True:
            try:
                await asyncio.sleep(1 / self.frequency)
                json_str = self._make_json()
                self._send_msg(json_str)
            except Exception as e:
                pass

    def _make_json(self):
        t = time.time() - self.start
        drone_data = {}
        mission_data = {}
        drone_names = ["veryrealdrone", "absolutedrone", "bob"]
        for i, drone_name in enumerate(drone_names):
            drone_data[drone_name] = {
                "position": [math.sin(t) - 1.5 + i*1.5, math.cos(t), 2],
                "speed": math.sin(t)+math.cos(2*t),
                "attitude": [math.sin(t) * 30, math.sin(t) * 10, ((t+i*10)*10) % 360 - 180],
                "heading": ((t+i*10)*10) % 360 - 180,
                "mode": "offboard" if (t+2*i) % 10 < 5 else "averyandunreasonablylongmodename",
                "conn": (t+2*i) % 20 < 15,
                "armed": (t+2*i) % 20 < 13,
                "in_air": (t+2*i) % 20 < 11,
            }
        mission_data["dummy_mission"] = {
            "flightarea": [-4, 4, -2, 2, 2],
            "stage": "gleep" if t % 20 < 10 else "glorp",
            "drones": ["veryrealdrone", "bob"],
            "bat": {"veryrealdrone": abs(math.sin(t/10)),
                    "bob": abs(math.cos(t/10))},
        }
        data = {
            "drones": drone_data,
            "missions": mission_data,
        }
        return json.dumps(data)

    def _send_msg(self, msg: str):
        self.socket.sendto(msg.encode("utf-8"), ("localhost", self.port)) #("192.168.1.103", self.port))


async def main():
    sender = UDPSender()
    await sender.send()
    sender.socket.close()


if __name__ == "__main__":
    asyncio.run(main())

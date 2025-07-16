""" Plugins for communication to other software
"""
import asyncio
import socket
import json

LISTEN_TIME = 10


class UDPClient:

    def __init__(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", 0))
        self.socket = sock
        self.target = ("localhost", 31659)
        self.duration = 30
        self.frequency = 1

    async def send_init_message(self):
        msg = json.dumps({"duration": self.duration, "frequency": self.frequency})
        self.socket.sendto(msg.encode("utf-8"), self.target)

    async def receive(self):
        while True:
            try:
                msg = await asyncio.wait_for(asyncio.get_running_loop().sock_recv(self.socket, 1024), LISTEN_TIME)
                json_str = json.loads(msg)
                print(json_str)
            except TimeoutError:
                print("No messages...")
            except Exception as e:
                print("Exception receiving data out over UDP!", repr(e))


async def main():
    receiver = UDPClient()
    await receiver.send_init_message()
    await receiver.receive()
    receiver.socket.close()


if __name__ == "__main__":
    asyncio.run(main())

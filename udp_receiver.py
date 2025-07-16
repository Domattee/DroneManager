""" Plugins for communication to other software
"""
import asyncio
import socket
import json

LISTEN_TIME = 1


class UDPReceiver:

    def __init__(self):
        self.port = 31659
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", self.port))
        self.socket = sock

    async def _receive(self):
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
    receiver = UDPReceiver()
    await receiver._receive()
    receiver.socket.close()


if __name__ == "__main__":
    asyncio.run(main())

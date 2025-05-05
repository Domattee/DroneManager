""" Plugins for communication to other software
"""
import asyncio
import socket
import json
import time

LISTEN_TIME = 10


class UDPReceiver:

    def __init__(self):
        self.port = 31659
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", self.port))
        self.socket = sock
        self.rec_task = asyncio.create_task(self._receive())
        self.start = time.time()

    async def _receive(self):
        while True:
            try:
                msg = await asyncio.get_running_loop().run_in_executor(None, self.socket.recv, 1024)
                json_str = json.loads(msg)
                print(json_str)
            except Exception as e:
                print("Exception receiving data out over UDP!", repr(e))


async def main():
    receiver = UDPReceiver()
    try:
        await asyncio.wait_for(receiver.rec_task, LISTEN_TIME)
    except asyncio.TimeoutError:
        pass
    receiver.socket.close()


if __name__ == "__main__":
    asyncio.run(main())

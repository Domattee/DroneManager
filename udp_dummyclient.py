""" Plugins for communication to other software
"""
import asyncio
import socket
import json
import time
import argparse

LISTEN_TIME = 10
SERVER_PORT = 31659

class UDPClient:
    """ This client starts a connection and then keeps it alive until shut down.

    You can update the frequency of the messages by changing self.frequency, or the outgoing frequency by changing
    self.duration. It will be updated with the next hello message. You can also send one manually with send_update_message()
    """

    def __init__(self, server_ip: str, server_port: int, frequency = 1, duration = 30):
        self.frequency = frequency
        # How often should the server send messages.

        self.json_output = None
        # The latest message from the server is stored here.

        self.time_since_last = -1
        # Time (in time.time() format) of the last message from the server. You can use this
        # to check for connection loss.

        # Socket stuff
        self.socket = None
        self.target = (server_ip, server_port)
        self.duration = duration  # How long the server should send us info without a message from us. Making this very long is rude.

        self._running_tasks = set()
        self._receiving_messages = False

    def start(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", 0))
        self.socket = sock
        hello_task = asyncio.create_task(self._send_hello_messages())
        self._running_tasks.add(hello_task)
        listen_task = asyncio.create_task(self._receive())
        self._running_tasks.add(listen_task)

    def __enter__(self):
        return self

    def close(self):
        for task in self._running_tasks:
            if task is not None:
                task.cancel()
        self.socket.close()

    def __exit__(self, exit_type, value, traceback):
        self.close()

    def send_update_message(self):
        print(f"Sending update message. New frequency: {self.frequency} New keep-alive duration: {self.duration}")
        msg = json.dumps({"duration": self.duration, "frequency": self.frequency})
        self.socket.sendto(msg.encode("utf-8"), self.target)

    async def _send_hello_messages(self):
        while True:
            try:
                print(f"Sending {'initial' if not self._receiving_messages else 'recurring'} hello message...")
                msg = json.dumps({"duration": self.duration, "frequency": self.frequency})
                self.socket.sendto(msg.encode("utf-8"), self.target)
            except Exception as e:
                print("Exception sending initial hello packet! ", repr(e))
            # Send every 1 second if we're not getting information yet, otherwise more rarely
            if not self._receiving_messages:
                await asyncio.sleep(1)
            else:
                await asyncio.sleep(self.duration - self.duration / 5)

    async def _receive(self):
        while True:
            try:
                msg = await asyncio.wait_for(asyncio.get_running_loop().sock_recv(self.socket, 1024), LISTEN_TIME)
                json_str = json.loads(msg)
                self.json_output = json_str
                self._receiving_messages = True
                self.time_since_last = time.time()
                print(json_str)
            except (TimeoutError, asyncio.TimeoutError):
                self._receiving_messages = False
                print("No messages...")
                await asyncio.sleep(0)
            except Exception as e:
                print("Exception receiving data over UDP! ", repr(e))
                await asyncio.sleep(0)  # Prevent getting stuck in the loop when we get this exception


async def main_example(ip: str, port: int):
    receiver = UDPClient(ip, port)
    receiver.start()
    await asyncio.sleep(10)
    print("Requesting updates with 2 Hz instead, waiting for natural update message.")
    receiver.frequency = 2
    await asyncio.sleep(42)
    print("Requesting updates with 5 Hz, sending update message immediately.")
    receiver.frequency = 5
    receiver.send_update_message()
    await asyncio.sleep(10)
    receiver.close()
    print("Done")


async def main(ip: str, port: int):
    with UDPClient(ip, port) as receiver:
        receiver.frequency = 2
        receiver.duration = 60
        receiver.start()
        while True:
            await asyncio.sleep(60)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Dummy UDP client to show functioning of 'External' plugin of DM")
    parser.add_argument("address", type=str, default="127.0.0.1", nargs="?",
                        help="IP address running DM. Default localhost.")
    parser.add_argument("--port", type=int, default=SERVER_PORT, required=False,
                        help=f"Port on which the server is listening. Default {SERVER_PORT}. The server port is fixed "
                             f"in DM, so this argument is only necessary if you changed the external plugin.")
    parser.add_argument("-e", "--example", action="store_true",
                        help="If this flag is set, instead of trying to connect and request indefinitely, we run "
                             "through an example on the message frequencies and update messages.")
    args = parser.parse_args()

    if args.example:
        asyncio.run(main_example(args.address, args.port))
    else:
        asyncio.run(main(args.address, args.port))

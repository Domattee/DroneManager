""" Plugins for communication to other software
"""
import asyncio
import socket
import json
import time

LISTEN_TIME = 10
SERVER_PORT = 31659

class UDPClient:
    """ This client starts a connection and then keeps it alive until shut down.

    You can update the frequency of the messages by changing self.frequency, or the outgoing frequency by changing
    self.duration. It will be updated with the next hello message. You can also send one manually with send_update_message()
    """

    def __init__(self):
        self.frequency = 1
        # How often should the server send messages.

        self.json_output = None
        # The latest message from the server is stored here.

        self.time_since_last = -1
        # Time (in time.time() format) of the last message from the server. You can use this
        # to check for connection loss.

        # Socket stuff
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", 0))
        self.socket = sock
        self.target = ("localhost", SERVER_PORT)
        self.duration = 30  # How long the server should send us info without a message from us. Making this very long is rude.

        self._running_tasks = set()
        self._receiving_messages = False

    def start(self):
        listen_task = asyncio.create_task(self._receive())
        self._running_tasks.add(listen_task)
        hello_task = asyncio.create_task(self._send_hello_messages())
        self._running_tasks.add(hello_task)

    def close(self):
        for task in self._running_tasks:
            if task is not None:
                task.cancel()
        self.socket.close()

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
            except TimeoutError:
                self._receiving_messages = False
                print("No messages...")
            except Exception as e:
                print("Exception receiving data out over UDP! ", repr(e))


async def main():
    receiver = UDPClient()
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


if __name__ == "__main__":
    asyncio.run(main())

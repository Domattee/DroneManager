import asyncio
import socket
import json
import time
import websockets

WS_PORT = 8765

class UDPReceiver:
    def __init__(self):
        self.port = 31659
        self.ws_connections = set()  # Store WebSocket connections
        # UDP Socket setup
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", self.port))
        self.socket = sock
        self.rec_task = asyncio.create_task(self._receive())
        self.start = time.time()
        print(f"UDP Socket bound to port {self.port}")

    async def _receive(self):
        while True:
            try:
                msg = await asyncio.get_running_loop().run_in_executor(None, self.socket.recv, 1024)
                json_str = json.loads(msg)
                print(json_str)  # Keep the original print
                
                # Broadcast to WebSocket clients if any are connected
                if self.ws_connections:
                    await self.broadcast(json.dumps(json_str))
            except Exception as e:
                print("Exception receiving data:", repr(e))

    async def broadcast(self, message):
        if self.ws_connections:
            await asyncio.gather(
                *[ws.send(message) for ws in self.ws_connections],
                return_exceptions=True
            )

    async def handle_websocket(self, websocket):
        self.ws_connections.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.ws_connections.remove(websocket)

async def main():
    receiver = UDPReceiver()
    
    # Create and start WebSocket server
    async with websockets.serve(
        receiver.handle_websocket, 
        "0.0.0.0", 
        WS_PORT,
        ping_interval=None
    ):
        print(f"WebSocket server started on ws://0.0.0.0:{WS_PORT}")
        try:
            # Wait for UDP messages for the specified time
            await receiver.rec_task
        except asyncio.TimeoutError:
            print("Listen time expired")
        finally:
            print("-----------")
            receiver.socket.close()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped by user")
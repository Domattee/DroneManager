# udp_receiver.py
import asyncio
import socket
import json
import time
import websockets

LISTEN_TIME = 1000
WS_PORT = 8765

class UDPReceiver:
    def __init__(self):
        self.port = 31659
        self.ws_connections = set()
        # UDP Socket setup
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", self.port))
        print(f"UDP Socket bound to port {self.port}")
        self.socket = sock
        self.rec_task = asyncio.create_task(self._receive())
        self.start = time.time()

    async def _receive(self):
        print("Started UDP receiver...")
        while True:
            try:
                print("Waiting for UDP data...")
                msg = await asyncio.get_running_loop().run_in_executor(None, self.socket.recv, 1024)
                print(f"Received raw UDP data: {len(msg)} bytes")
                json_str = json.loads(msg)
                print(f"Parsed JSON: {json_str}")
                
                if self.ws_connections:
                    print(f"Broadcasting to {len(self.ws_connections)} WebSocket clients")
                    await self.broadcast(json.dumps(json_str))
                else:
                    print("No WebSocket clients connected")
            except json.JSONDecodeError as e:
                print(f"JSON decode error: {e}")
            except Exception as e:
                print(f"Error in _receive: {e}")

    async def broadcast(self, message):
        if self.ws_connections:
            try:
                await asyncio.gather(
                    *[ws.send(message) for ws in self.ws_connections],
                    return_exceptions=True
                )
                print("Broadcast complete")
            except Exception as e:
                print(f"Error in broadcast: {e}")

    async def register(self, websocket):
        self.ws_connections.add(websocket)
        print(f"New WebSocket client connected. Total clients: {len(self.ws_connections)}")

    async def unregister(self, websocket):
        self.ws_connections.remove(websocket)
        print(f"WebSocket client disconnected. Total clients: {len(self.ws_connections)}")

    async def handle_websocket(self, websocket):
        """Handle individual WebSocket connections"""
        print(f"New WebSocket connection from {websocket.remote_address}")
        await self.register(websocket)
        try:
            async for message in websocket:
                print(f"Received WebSocket message: {message}")
        except websockets.exceptions.ConnectionClosed:
            print("WebSocket connection closed")
        finally:
            await self.unregister(websocket)

async def main():
    receiver = UDPReceiver()
    
    print(f"Starting WebSocket server on port {WS_PORT}...")
    
    # Create WebSocket server
    async with websockets.serve(
        receiver.handle_websocket,
        "0.0.0.0",
        WS_PORT,
        ping_interval=None  # Disable ping/pong for testing
    ) as server:
        print(f"WebSocket server started on ws://0.0.0.0:{WS_PORT}")
        print(f"UDP server listening on port {receiver.port}")
        try:
            await asyncio.Future()  # run forever
        except Exception as e:
            print(f"Server error: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped by user")
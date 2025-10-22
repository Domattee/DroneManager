import asyncio
import socket
import json
import os
from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import uvicorn

class WebServer:
    def __init__(self):
        self.udp_port = 31660
        self.web_port = 8080
        self.clients = set()
        self.latest_data = None
        
        # Setup UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", self.udp_port))
        self.socket = sock
        
        # Setup FastAPI
        self.app = FastAPI()
        self.setup_routes()

    def setup_routes(self):
        @self.app.get("/")
        async def get_index():
            return FileResponse(os.path.join(os.path.dirname(__file__), 'static', 'index.html'))

        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.clients.add(websocket)
            try:
                if self.latest_data:
                    await websocket.send_json(self.latest_data)
                while True:
                    await websocket.receive_text()
            except:
                self.clients.remove(websocket)

        # Mount static files
        static_dir = os.path.join(os.path.dirname(__file__), 'static')
        self.app.mount("/static", StaticFiles(directory=static_dir), name="static")

    async def broadcast_data(self, data):
        self.latest_data = data
        for client in self.clients:
            try:
                await client.send_json(data)
            except:
                pass

    async def receive_udp(self):
        while True:
            try:
                msg = await asyncio.get_running_loop().run_in_executor(None, self.socket.recv, 1024)
                json_data = json.loads(msg)
                await self.broadcast_data(json_data)
            except Exception as e:
                print("Exception receiving data:", repr(e))

    async def start(self):
        config = uvicorn.Config(self.app, host="localhost", port=self.web_port)
        server = uvicorn.Server(config)
        
        print(f"Web server starting at http://localhost:{self.web_port}")
        print(f"UDP server listening on port {self.udp_port}")
        
        # Start UDP receiver in the background
        asyncio.create_task(self.receive_udp())
        
        # Start the web server
        await server.serve()

async def main():
    server = WebServer()
    await server.start()

if __name__ == "__main__":
    asyncio.run(main()) 
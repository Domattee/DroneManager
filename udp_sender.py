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
        self.frequency = 1  # Reduced frequency to 1 Hz for better debugging
        self.start = time.time()
        print(f"UDP Sender initialized, sending to port {self.port}")

    async def _send(self):
        print("Starting to send data...")
        while True:
            try:
                await asyncio.sleep(1 / self.frequency)
                json_str = self._make_json()
                self._send_msg(json_str)
                print(f"Sent data: {json_str[:100]}...")  # Print first 100 chars
            except Exception as e:
                print(f"Error sending data: {e}")

    def _make_json(self):
        t = time.time() - self.start
        drone_data = {}
        mission_data = {}
        drone_name = "veryrealdrone"
        drone_data[drone_name] = {
            "position": [math.sin(t), math.cos(t), 0.0],  # Changed NaN to 0.0
            "heading": (t*10) % 360 - 180,
            "conn": True,  # Set to True for testing
            "armed": True,  # Set to True for testing
            "in_air": True,  # Set to True for testing
        }
        mission_data["dummy_mission"] = {
            "stage": "test_stage",
            "drones": ["veryrealdrone"],
            "bat": {"veryrealdrone": 0.9},  # Fixed battery level for testing
        }
        data = {
            "drones": drone_data,
            "missions": mission_data,
            "timestamp": time.time()
        }
        return json.dumps(data)

    def _send_msg(self, msg: str):
        try:
            bytes_sent = self.socket.sendto(msg.encode("utf-8"), ("localhost", self.port))
            print(f"Sent {bytes_sent} bytes to localhost:{self.port}")
        except Exception as e:
            print(f"Error in _send_msg: {e}")

async def main():
    sender = UDPSender()
    try:
        await sender._send()
    except KeyboardInterrupt:
        print("\nSender stopped by user")
    finally:
        sender.socket.close()
        print("Socket closed")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nSender stopped by user")
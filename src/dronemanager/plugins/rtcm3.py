import asyncio

from dronemanager.plugin import Plugin


class RTCM3Plugin(Plugin):
    PREFIX = "rtcm3"
    DEPENDENCIES = []

    MAX_RTCM_PAYLOAD = 180

    def __init__(self, dm, logger, name, host="192.168.137.40", port=5016,
                 idle_timeout=30.0, reconnect_delay=2.0, **kwargs):
        super().__init__(dm, logger, name, **kwargs)

        self.host = host
        self.port = int(port)
        self.idle_timeout = float(idle_timeout)
        self.reconnect_delay = float(reconnect_delay)

        self.is_connected = False
        self.total_frames_parsed = 0
        self._should_run = True
        self._latest_packet: bytes | None = None
        self.forward_targets: set[str] = set()

        self.background_functions = []
        self.cli_commands = {
            "send": self.send_rtcm3_drone,
            "stop": self.stop_rtcm3_drone,
            "status": self.get_status,
        }

    async def start(self):
        await super().start()
        if not self._running_tasks:
            self._running_tasks.add(asyncio.create_task(self._port_lifecycle_manager()))

    async def shutdown(self):
        """Explicit shutdown hook called by DroneManager framework (if supported)."""
        self.logger.info("Stopping RTCM3 background processing...")
        self._should_run = False
        self.is_connected = False
        await asyncio.sleep(0.1)

    def _extract_rtcm_packets(self, payload: bytes | bytearray) -> tuple[list[bytes], bytes]:
        """Pull all complete RTCM3 frames from a byte stream, leaving any partial tail behind."""
        buffer = bytearray(payload)
        packets: list[bytes] = []

        while len(buffer) >= 3:
            if buffer[0] != 0xD3:
                del buffer[0]
                continue

            length = ((buffer[1] & 0x03) << 8) | buffer[2]
            total_packet_size = length + 6

            if len(buffer) < total_packet_size:
                break

            packets.append(bytes(buffer[:total_packet_size]))
            del buffer[:total_packet_size]

        return packets, bytes(buffer)

    async def _port_lifecycle_manager(self):
        """Async worker managing the TCP connection cycle."""
        self.logger.info(
            f"RTCM3 Plugin initialized. Listening for RTCM3 stream on {self.host}:{self.port} "
            f"(idle_timeout={self.idle_timeout}s, reconnect_delay={self.reconnect_delay}s)"
        )

        while self._should_run:
            reader = None
            writer = None
            try:
                reader, writer = await asyncio.open_connection(self.host, self.port)
                self.is_connected = True
                self.logger.info(f"TCP link locked onto {self.host}:{self.port}.")

                byte_buffer = bytearray()
                while self._should_run:
                    try:
                        data = await asyncio.wait_for(reader.read(4096), timeout=self.idle_timeout)
                    except asyncio.TimeoutError:
                        self.logger.warning(
                            f"RTCM3 TCP idle timeout after {self.idle_timeout}s; closing connection and reconnecting."
                        )
                        break

                    if not data:
                        self.logger.warning("RTCM3 TCP socket closed by peer; reconnecting.")
                        break

                    byte_buffer.extend(data)
                    packets, tail = self._extract_rtcm_packets(bytes(byte_buffer))
                    byte_buffer = bytearray(tail)

                    for rtcm_packet in packets:
                        msg_id = (rtcm_packet[3] << 4) | (rtcm_packet[4] >> 4)
                        self.total_frames_parsed += 1
                        self._distribute_packet(msg_id, rtcm_packet)

                    await asyncio.sleep(0)
            except (ConnectionResetError, ConnectionRefusedError, OSError) as exc:
                self.is_connected = False
                if self._should_run:
                    self.logger.warning(f"RTCM3 TCP connection lost ({exc}). Retrying in {self.reconnect_delay}s...")
                await asyncio.sleep(self.reconnect_delay)
            except Exception as exc:
                self.is_connected = False
                if self._should_run:
                    self.logger.error(f"RTCM3 parsing error occurred: {exc}")
                await asyncio.sleep(self.reconnect_delay)
            finally:
                self.is_connected = False
                if writer is not None:
                    writer.close()
                    try:
                        await writer.wait_closed()
                    except Exception:
                        pass

    def _distribute_packet(self, msg_id: int, packet: bytes):
        """Pushes the isolated frame out into the DroneManager ecosystem and the connected drones."""
        self._latest_packet = packet
        if hasattr(self.dm, "publish"):
            self.dm.publish("gnss:rtcm3", {"id": msg_id, "raw": packet})
        elif hasattr(self.dm, "state"):
            self.dm.state.last_rtcm_frame = {
                "id": msg_id,
                "bytes": packet,
                "time": asyncio.get_event_loop().time(),
            }

        self._forward_to_drones(packet)

    def _chunk_packet(self, payload: bytes, max_payload: int | None = None) -> list[bytes]:
        if max_payload is None:
            max_payload = self.MAX_RTCM_PAYLOAD
        return [payload[i:i + max_payload] for i in range(0, len(payload), max_payload)]

    def _forward_to_drones(self, packet: bytes):
        """Send RTCM3 bytes to the connected drones via MAVLink."""
        drones = getattr(self.dm, "drones", {})
        if self.forward_targets:
            selected = [name for name in self.forward_targets if name in drones]
        else:
            selected = list(drones.keys())

        for drone_name in selected:
            drone = drones[drone_name]
            if not getattr(drone, "is_connected", False):
                continue
            mav_conn = getattr(drone, "mav_conn", None)
            if mav_conn is None:
                continue
            self._forward_to_drone(mav_conn, packet)

    def _forward_to_drone(self, mav_conn, packet: bytes) -> bool:
        if mav_conn is None:
            return False

        mav = getattr(getattr(mav_conn, "con_drone_in", None), "mav", None)
        if mav is None or not hasattr(mav, "gps_rtcm_data_encode"):
            self.logger.debug("MAV connection does not support RTCM3 encoding")
            return False
        if not hasattr(mav_conn, "send_as_gcs"):
            self.logger.debug("MAV connection does not expose send_as_gcs")
            return False

        for chunk in self._chunk_packet(packet):
            msg = mav.gps_rtcm_data_encode(0, len(chunk), chunk)
            mav_conn.send_as_gcs(msg)
        return True

    async def send_rtcm3_drone(self, name: str, packet: bytes | None = None):
        """Forward RTCM3 data to a specific drone by name using its MAV connection."""
        if name not in self.dm.drones:
            self.logger.warning(f"No drone named '{name}'.")
            return False

        drone = self.dm.drones[name]
        mav_conn = getattr(drone, "mav_conn", None)
        if mav_conn is None:
            self.logger.warning(f"Drone '{name}' has no MAV connection configured.")
            return False

        if packet is None:
            packet = self._latest_packet
        if packet is None:
            self.logger.warning("No RTCM3 data is available to send.")
            return False

        self.forward_targets.add(name)
        self.logger.info(f"RTCM3 data forwarding locked explicitly to target drone: {name}.")
        return self._forward_to_drone(mav_conn, packet)

    async def stop_rtcm3_drone(self, name: str):
        """Remove a specific drone target from the forwarding filter registry."""
        if name in self.forward_targets:
            self.forward_targets.discard(name)
            self.logger.info(f"Stopped RTCM3 routing to drone '{name}'.")
            return True
        self.logger.warning(f"Drone '{name}' was not found.")
        return False

    async def get_status(self):
        """Inspect RTCM3 stream health."""
        connection_label = "CONNECTED" if self.is_connected else "DISCONNECTED/TCP UNAVAILABLE"
        self.logger.info(f"--- [RTCM3 Interface Diagnostic] ---")
        self.logger.info(f"Target TCP endpoint : {self.host}:{self.port}")
        self.logger.info(f"Hardware Status  : {connection_label}")
        self.logger.info(f"Frames Extracted : {self.total_frames_parsed}")

    
import logging
import datetime
import time
import os
import asyncio

from pymavlink import mavutil

# from pymavlink.dialects.v20 import cubepilot

formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)s %(name)s - %(message)s', datefmt="%H:%M:%S")

# TODO: Routing between multiple GCS so we can have my app and QGroundControl connected at the same time


class MAVPassthrough:
    def __init__(self, dialect="cubepilot", loggername="passthrough", log_messages=True):
        self.dialect = dialect
        self.source_system = 245
        self.source_component = 201
        self.con_drone_in: mavutil.mavudp | None = None
        self.con_gcs: mavutil.mavudp | None = None

        self.time_of_last_gcs = 0
        self.time_of_last_drone = 0
        self.disconnected_thresh = 2e9

        self.logger = logging.getLogger(loggername)
        self.logger.setLevel(logging.DEBUG)
        filename = f"{loggername}_{datetime.datetime.now()}"
        filename = filename.replace(":", "_").replace(".", "_") + ".log"
        logdir = os.path.abspath("./logs")
        os.makedirs(logdir, exist_ok=True)
        file_handler = logging.FileHandler(os.path.join(logdir, filename))
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)
        self.logging_handlers = []
        self.logging_handlers.append(file_handler)
        self.log_messages = log_messages

        self.running_tasks = set()
        self.should_stop = False

    def connect_gcs(self, address):
        self.running_tasks.add(asyncio.create_task(self._connect_gcs(address)))

    async def _connect_gcs(self, address):
        self.con_gcs = mavutil.mavlink_connection("udpout:" + address,
                                                  source_system=self.source_system,
                                                  source_component=self.source_component, dialect=self.dialect)
        self.running_tasks.add(asyncio.create_task(self._send_heartbeats_gsc()))
        await asyncio.sleep(0)
        self.logger.debug("Waiting for GCS heartbeat")
        while not self.con_gcs.wait_heartbeat(blocking=False):
            await asyncio.sleep(0.05)
        self.logger.debug("Got GCS heartbeat")
        self.time_of_last_gcs = time.time_ns()
        self.running_tasks.add(asyncio.create_task(self._send_pings_gcs()))
        self.running_tasks.add(asyncio.create_task(self._listen_gcs()))

    def connect_drone(self, loc, appendix, scheme="udp"):
        if scheme == "udp":
            self.running_tasks.add(asyncio.create_task(self._connect_drone_udp(loc, appendix)))
        elif scheme == "serial":
            self.running_tasks.add(asyncio.create_task(self._connect_drone_serial(loc, appendix)))

    async def _connect_drone_serial(self, path, baud):
        try:
            self.logger.debug(f"Connecting to drone @{path}:{baud}")

            tmp_con_drone_in = mavutil.mavlink_connection(f"{path}",
                                                           baud=baud,
                                                           source_system=self.source_system,
                                                           source_component=self.source_component,
                                                           dialect=self.dialect)

            received_hb = 0
            while received_hb < 3:
                self.logger.debug("Sending initial drone heartbeat")
                tmp_con_drone_in.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                     mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                                     0, 0, 0)
                await asyncio.sleep(0.1)
                m = tmp_con_drone_in.wait_heartbeat(blocking=False)
                if m is not None:
                    if m.get_srcSystem() != self.source_system and m.get_srcComponent() != self.source_component:
                        received_hb += 1
                        self.logger.debug(f"Got drone heartbeat! Received {received_hb} total")
                await asyncio.sleep(0.2)

            tmp_con_drone_in.close()

            self.con_drone_in = mavutil.mavlink_connection(f"{path}",
                                                           baud=baud,
                                                           source_system=self.source_system,
                                                           source_component=self.source_component,
                                                           dialect=self.dialect)

            self.running_tasks.add(asyncio.create_task(self._send_pings_drone()))
            self.running_tasks.add(asyncio.create_task(self._listen_drone()))
            self.running_tasks.add(asyncio.create_task(self._send_heartbeats_drone()))
        except Exception as e:
            self.logger.debug(f"Error during connection to drone: {repr(e)}", exc_info=True)

    async def _connect_drone_udp(self, ip, port):
        try:
            self.logger.debug(f"Connecting to drone @{ip}:{port}")

            self.logger.debug("Creating temp udp connections")
            tmp_con_drone_out = mavutil.mavlink_connection(f"udp:{ip}:{port}",
                                                           input=False,
                                                           source_system=self.source_system,
                                                           source_component=self.source_component, dialect=self.dialect)
            tmp_con_drone_in = mavutil.mavlink_connection(f"udp::{port}",
                                                          input=True,
                                                          source_system=self.source_system,
                                                          source_component=self.source_component, dialect=self.dialect)

            received_hb = 0
            while received_hb < 1:
                self.logger.debug("Sending initial drone heartbeat")
                tmp_con_drone_out.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                     mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                                     0, 0, 0)
                await asyncio.sleep(0.1)
                m = tmp_con_drone_in.wait_heartbeat(blocking=False)
                if m is not None:
                    if m.get_srcSystem() != self.source_system and m.get_srcComponent() != self.source_component:
                        received_hb += 1
                        self.logger.debug(f"Got drone heartbeat! Received {received_hb} total")
                await asyncio.sleep(0.2)

            tmp_con_drone_in.close()
            tmp_con_drone_out.close()

            self.con_drone_in = mavutil.mavlink_connection(f"udpin::{port}",
                                                           source_system=self.source_system,
                                                           source_component=self.source_component, dialect=self.dialect)

            self.running_tasks.add(asyncio.create_task(self._send_pings_drone()))
            self.running_tasks.add(asyncio.create_task(self._listen_drone()))
            self.running_tasks.add(asyncio.create_task(self._send_heartbeats_drone()))
        except Exception as e:
            self.logger.debug(f"Error during connection to drone: {repr(e)}", exc_info=True)

    def connected_to_drone(self):
        if time.time_ns() - self.time_of_last_drone < self.disconnected_thresh:
            return True
        else:
            return False

    def connected_to_gcs(self):
        if time.time_ns() - self.time_of_last_gcs < self.disconnected_thresh:
            return True
        else:
            return False

    def _process_message_for_return(self, msg):
        msg_id = msg._header.msgId  # msg.id is sometimes not set correctly.
        if msg_id == -1:
            self.logger.debug(f"Message with BAD_DATE id, can't resend: {msg_id}, {msg.to_dict()}")
            return False
        if msg_id == -2:
            self.logger.debug(f"Message with unkown MAVLink ID, can't resend: {msg_id}, {msg.get_type()} "
                              f"{msg.fieldnames}, {msg.fieldtypes}, {msg.orders}, {msg.lengths}, "
                              f"{msg.array_lengths}, {msg.crc_extra}, {msg.unpacker}")
            return False
        msg_class = mavutil.mavlink.mavlink_map[msg_id]
        msg.__class__ = msg_class
        return True
        # maybe like this?: message_class(**msg.to_dict())

    async def _listen_gcs(self):
        self.logger.debug("Starting to listen to GCS")
        while not self.should_stop:
            if self.con_gcs is not None:
                while not self.should_stop:
                    # Receive and log all messages from the GCS
                    msg = self.con_gcs.recv_match(blocking=False)
                    if msg is None:
                        await asyncio.sleep(0.0001)
                    else:
                        if self.log_messages:
                            self.logger.debug(f"Message from GCS, {msg.to_dict()}")
                        self.time_of_last_gcs = time.time_ns()
                        if self.con_drone_in is not None and self.connected_to_gcs():  # Send onward to the drone
                            self.con_drone_in.mav.srcSystem = msg.get_srcSystem()
                            self.con_drone_in.mav.srcComponent = msg.get_srcComponent()
                            if self._process_message_for_return(msg):
                                try:
                                    self.con_drone_in.mav.send(msg)
                                except Exception as e:
                                    self.logger.debug(f"Encountered an exception sending message to drone: "
                                                      f"{repr(e)}", exc_info=True)
                        self.con_gcs.mav.srcSystem = self.source_system
                        self.con_gcs.mav.srcComponent = self.source_component
            else:
                await asyncio.sleep(1)

    async def _listen_drone(self):
        self.logger.debug("Starting to listen to drone")
        while not self.should_stop:
            if self.con_drone_in is not None:
                while not self.should_stop:
                    # Receive and log all messages from the GCS
                    msg = self.con_drone_in.recv_match(blocking=False)
                    if msg is None:
                        await asyncio.sleep(0.0001)
                    else:
                        if self.log_messages:
                            self.logger.debug(f"Message from Drone, {msg.to_dict()}")
                        self.time_of_last_drone = time.time_ns()
                        if self.con_gcs is not None and self.connected_to_drone():
                            self.con_gcs.mav.srcSystem = msg.get_srcSystem()
                            self.con_gcs.mav.srcComponent = msg.get_srcComponent()
                            if self._process_message_for_return(msg):
                                try:
                                    self.con_gcs.mav.send(msg)
                                except Exception as e:
                                    self.logger.debug(f"Encountered an exception sending message to GCS: {repr(e)}",
                                                      exc_info=True)
                            self.con_gcs.mav.srcSystem = self.source_system
                            self.con_gcs.mav.srcComponent = self.source_component
            else:
                await asyncio.sleep(1)

    async def _send_pings_gcs(self):
        while not self.should_stop:
            self.logger.debug("Pinging GCS")
            self.con_gcs.mav.ping_send(int(time.time() * 1e6), 0, 0, 0)
            await asyncio.sleep(5)

    async def _send_pings_drone(self):
        while not self.should_stop:
            self.logger.debug("Pinging drone")
            self.con_drone_in.mav.ping_send(int(time.time() * 1e6), 0, 0, 0)
            await asyncio.sleep(5)

    async def _send_heartbeats_drone(self):
        while not self.should_stop:
            self.logger.debug("Sending heartbeat to drone")
            self.con_drone_in.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            await asyncio.sleep(0.5)

    async def _send_heartbeats_gsc(self):
        while not self.should_stop:
            self.logger.debug("Sending heartbeat to GCS")
            self.con_gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.logger.debug(f"GCS connection target system {self.con_gcs.target_system}")
            await asyncio.sleep(0.5)

    async def stop(self):
        self.logger.debug("Stopping")
        self.should_stop = True
        if self.con_drone_in:
            self.con_drone_in.close()
        if self.con_gcs:
            self.con_gcs.close()
        for task in self.running_tasks:
            task.cancel()
        for handler in self.logging_handlers:
            self.logger.removeHandler(handler)


async def main():
    snoop = MAVPassthrough()
    snoop.connect_drone(loc="192.168.1.37", appendix=14567)
    snoop.connect_gcs("127.0.0.1:15567")
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
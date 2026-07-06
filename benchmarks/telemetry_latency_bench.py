"""Telemetry latency/jitter benchmark for the paper's Evaluation section.

Measures the real end-to-end latency of DroneManager's UDP telemetry stream
(the same channel Unity's UDPReceiver subscribes to) using a standalone UDP
client that replicates Unity's wire protocol exactly (JSON subscribe handshake,
JSON telemetry packets). Requires a running PX4 SITL/Gazebo instance reachable
from this machine.

This is throwaway instrumentation for one benchmark run: `external.py` on this
branch embeds a `t_send` wall-clock timestamp in every telemetry packet so this
script can compute one-way latency without needing clock sync between processes
(both run on the same host). Not intended to be merged to main.

Usage:
    python benchmarks/telemetry_latency_bench.py --address udp://172.30.235.63:18570 --duration 45
"""
import argparse
import asyncio
import csv
import json
import logging
import socket
import statistics
import time
from pathlib import Path

from dronemanager.core import DroneManager
from dronemanager.drone import DroneMAVSDK

UDP_SERVER_PORT = 31659


class TelemetrySampler:
    """Replicates Unity's UDPReceiver: sends the subscribe handshake, then logs
    (recv_time - t_send) for every packet plus inter-arrival jitter."""

    def __init__(self, frequency: float, duration: float, server_port: int = UDP_SERVER_PORT):
        self.frequency = frequency
        self.duration = duration
        self.server_port = server_port
        self.samples: list[dict] = []

    async def run(self):
        loop = asyncio.get_running_loop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.0)
        sock.bind(("127.0.0.1", 0))

        subscribe_msg = json.dumps({"duration": self.duration, "frequency": self.frequency}).encode("utf-8")
        sock.sendto(subscribe_msg, ("127.0.0.1", self.server_port))
        last_recv_resub = time.time()

        end_time = time.time() + self.duration + 2.0
        last_recv = None
        while time.time() < end_time:
            try:
                data, _ = await loop.run_in_executor(None, sock.recvfrom, 65536)
            except (socket.timeout, OSError):
                continue
            recv_time = time.time()
            # Re-send the subscribe handshake periodically, mirroring Unity's requestInterval
            if recv_time - last_recv_resub > 15.0:
                sock.sendto(subscribe_msg, ("127.0.0.1", self.server_port))
                last_recv_resub = recv_time
            try:
                packet = json.loads(data)
            except json.JSONDecodeError:
                continue
            t_send = packet.get("t_send")
            if t_send is None:
                continue
            latency_ms = (recv_time - t_send) * 1000.0
            inter_arrival_ms = (recv_time - last_recv) * 1000.0 if last_recv is not None else None
            last_recv = recv_time
            n_drones = len(packet.get("drones", {}))
            self.samples.append({
                "recv_time": recv_time,
                "t_send": t_send,
                "latency_ms": latency_ms,
                "inter_arrival_ms": inter_arrival_ms,
                "n_drones": n_drones,
                "payload_bytes": len(data),
            })
        sock.close()

    def summary(self) -> dict:
        if not self.samples:
            return {"n_samples": 0}
        latencies = [s["latency_ms"] for s in self.samples]
        arrivals = [s["inter_arrival_ms"] for s in self.samples if s["inter_arrival_ms"] is not None]
        achieved_hz = 1000.0 / statistics.mean(arrivals) if arrivals else float("nan")
        return {
            "n_samples": len(self.samples),
            "requested_frequency_hz": self.frequency,
            "achieved_frequency_hz": achieved_hz,
            "latency_ms_mean": statistics.mean(latencies),
            "latency_ms_median": statistics.median(latencies),
            "latency_ms_stdev": statistics.stdev(latencies) if len(latencies) > 1 else 0.0,
            "latency_ms_p95": statistics.quantiles(latencies, n=100)[94] if len(latencies) > 20 else max(latencies),
            "latency_ms_p99": statistics.quantiles(latencies, n=100)[98] if len(latencies) > 20 else max(latencies),
            "latency_ms_max": max(latencies),
            "jitter_ms_stdev": statistics.stdev(arrivals) if len(arrivals) > 1 else 0.0,
            "payload_bytes_mean": statistics.mean(s["payload_bytes"] for s in self.samples),
        }

    def write_csv(self, path: Path):
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["recv_time", "t_send", "latency_ms", "inter_arrival_ms", "n_drones", "payload_bytes"])
            writer.writeheader()
            writer.writerows(self.samples)


async def drone_back_and_forth_timed(name: str, dm: DroneManager, duration: float):
    await dm.arm(name)
    await asyncio.sleep(1)
    await dm.takeoff(name)
    start_time = time.time()
    while time.time() - start_time < duration:
        await dm.fly_to(name, local=[10, 10, -3], tol=0.4)
        await dm.fly_to(name, local=[0, 0, -3], tol=0.4)
    disarmed = False
    tries = 0
    while not disarmed and tries < 10:
        await dm.land(name)
        await asyncio.sleep(2)
        await dm.disarm(name)
        await asyncio.sleep(1)
        disarmed = not dm.drones[name].is_armed
        tries += 1


async def main(args):
    logging.basicConfig(level=logging.INFO)
    dm = DroneManager(DroneMAVSDK, log_to_console=False, console_log_level=logging.INFO)
    for plugin in ["mission", "controllers", "external"]:
        await dm.load_plugin(plugin)

    sampler = TelemetrySampler(frequency=args.telemetry_hz, duration=args.duration)
    sampler_task = asyncio.create_task(sampler.run())

    try:
        await dm.connect_to_drone(args.name, None, None, args.address, log_telemetry=False,
                                   telemetry_frequency=args.mavlink_hz)
        await asyncio.sleep(2)
        await drone_back_and_forth_timed(args.name, dm, args.duration)
    finally:
        await sampler_task
        await dm.close()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"telemetry_latency_{args.tag}.csv"
    sampler.write_csv(csv_path)
    summary = sampler.summary()
    summary["tag"] = args.tag
    summary_path = out_dir / f"telemetry_latency_{args.tag}_summary.json"
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", default="udp://172.30.235.63:18570", help="MAVSDK connection string for the SITL drone")
    parser.add_argument("--name", default="kirk")
    parser.add_argument("--duration", type=float, default=45.0, help="Benchmark duration in seconds")
    parser.add_argument("--telemetry-hz", type=float, default=20.0, help="Requested UDP telemetry rate (Unity default)")
    parser.add_argument("--mavlink-hz", type=float, default=30.0, help="MAVSDK internal telemetry subscription rate")
    parser.add_argument("--out-dir", default="benchmarks/results")
    parser.add_argument("--tag", default="run1", help="Label for output files, e.g. '1drone_minimal'")
    asyncio.run(main(parser.parse_args()))

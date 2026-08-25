"""End-to-end teleoperation latency benchmark for the paper's Evaluation section.

Measures the full closed loop that an operator actually experiences:

    stick input --> MAVSDK/MAVLink --> PX4 --> vehicle accelerates (SITL physics)
                --> EKF estimates velocity --> telemetry back to DroneManager   [= T_motion]
                --> JSON/UDP --> Unity parses, interpolates, renders the frame   [= T_twin]

Both t0 (stick) and t1 (motion observed / twin rendered) are taken from the *same*
perf_counter in this process, so no cross-runtime clock synchronisation is needed: Unity
echoes back the sequence number of the telemetry packet whose frame it has just rendered
(see Assets/Scripts/HolodeckLatencyAck.cs), and we look that seq up in external.SENT_LOG.

IMPORTANT INTERPRETATION NOTE. "Stick -> motion" is *not* a software-latency number. It is
dominated by vehicle inertia and EKF filtering: the quadrotor physically cannot change
velocity instantly. It is therefore reported against a sweep of motion-detection thresholds
so the physics-dependent component is visible rather than hidden inside one chosen number.
The MAVSDK command-call duration is logged separately as a pure-software lower bound.

Throwaway instrumentation for one experiment; not intended for the main branch.

Usage (SITL must be running; Unity in Play mode only if --require-unity):
    python benchmarks/e2e_latency_bench.py --address udp://172.30.235.63:18570 \
        --trials 12 --require-unity --tag e2e_run1
"""
import argparse
import asyncio
import csv
import json
import logging
import math
import socket
import statistics
import time
from pathlib import Path

from dronemanager.core import DroneManager
from dronemanager.drone import DroneMAVSDK
from dronemanager.plugins import external

ACK_PORT = 31660
CONTROL_HZ = 50.0          # matches ControllerPlugin.DEFAULT_FREQUENCY
POLL_HZ = 1000.0           # how finely we watch for motion onset
NEUTRAL_VERTICAL = 0.5     # ControllerPlugin maps stick 0 -> (-0+1)/2 = 0.5

# Motion-detection thresholds on horizontal speed (m/s). Reported as a sweep because the
# answer genuinely depends on what you call "moving".
THRESHOLDS = [0.02, 0.05, 0.10, 0.20]


class AckListener:
    """Collects seq -> t_ack for telemetry packets Unity reports as rendered."""

    def __init__(self, port: int = ACK_PORT):
        self.port = port
        self.acks: dict[int, float] = {}
        self._stop = False
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.5)
        self._sock.bind(("127.0.0.1", port))

    async def run(self):
        loop = asyncio.get_running_loop()
        while not self._stop:
            try:
                data, _ = await loop.run_in_executor(None, self._sock.recvfrom, 64)
            except (socket.timeout, OSError):
                continue
            t_ack = time.perf_counter()
            try:
                seq = int(data.decode("utf-8").strip())
            except ValueError:
                continue
            # keep the first ack for a seq
            self.acks.setdefault(seq, t_ack)

    def stop(self):
        self._stop = True
        self._sock.close()


class StickStreamer:
    """Streams manual-control input at CONTROL_HZ, exactly as ControllerPlugin does.

    PX4 expects continuous manual-control input; a one-shot command would be treated as a
    dropout. We therefore stream neutral input continuously and simply change the commanded
    value at t0, which is what moving a real stick does.
    """

    def __init__(self, drone):
        self.drone = drone
        self.forward = 0.0
        self._stop = False
        self.call_durations: list[float] = []
        self.failures = 0
        self.last_error = None

    async def run(self):
        period = 1.0 / CONTROL_HZ
        while not self._stop:
            t_call = time.perf_counter()
            try:
                # (forward, right, vertical, yaw); vertical 0..1 with 0.5 = hold altitude
                await self.drone.set_manual_control_input(
                    float(self.forward), 0.0, NEUTRAL_VERTICAL, 0.0
                )
                self.call_durations.append((time.perf_counter() - t_call) * 1000.0)
            except Exception as e:  # do NOT swallow silently: a failing stick looks like a still drone
                self.failures += 1
                self.last_error = repr(e)
            await asyncio.sleep(period)

    def stop(self):
        self._stop = True


def h_speed(vel) -> float:
    return math.sqrt(float(vel[0]) ** 2 + float(vel[1]) ** 2)


async def wait_until_settled(drone, max_speed=0.05, stable_for=1.0, timeout=15.0):
    """Wait until the vehicle is hovering ~still, so each trial starts from rest."""
    start = time.perf_counter()
    settled_since = None
    while time.perf_counter() - start < timeout:
        if h_speed(drone.velocity) < max_speed:
            if settled_since is None:
                settled_since = time.perf_counter()
            elif time.perf_counter() - settled_since >= stable_for:
                return True
        else:
            settled_since = None
        await asyncio.sleep(0.02)
    return False


async def run_trial(drone, streamer, acks: AckListener, direction: float, hold_s: float,
                    drone_name: str):
    """One stick step. Returns per-threshold motion latencies and the twin-render latency."""
    await wait_until_settled(drone)
    baseline = h_speed(drone.velocity)

    seq_before = external._seq_counter  # only consider packets emitted after the step

    t0 = time.perf_counter()
    streamer.forward = direction

    trace = []
    crossings: dict[float, float] = {}
    deadline = t0 + hold_s
    poll_period = 1.0 / POLL_HZ
    while time.perf_counter() < deadline:
        now = time.perf_counter()
        v = h_speed(drone.velocity)
        trace.append((now - t0, v))
        for thr in THRESHOLDS:
            if thr not in crossings and (v - baseline) >= thr:
                crossings[thr] = (now - t0) * 1000.0
        if len(crossings) == len(THRESHOLDS):
            # keep flying a little so the twin has packets to render, but we have what we need
            if time.perf_counter() - t0 > 0.6:
                break
        await asyncio.sleep(poll_period)

    # Let the vehicle keep moving briefly so the motion is definitely telemetered + rendered
    await asyncio.sleep(0.5)
    streamer.forward = 0.0

    # --- correlate with the twin ---
    # Find the first telemetry packet emitted after t0 whose serialized velocity already shows
    # motion past the primary threshold, then look up when Unity said it had rendered it.
    primary = THRESHOLDS[1]  # 0.05 m/s
    twin_ms = None
    twin_seq = None
    for seq in sorted(k for k in external.SENT_LOG if k >= seq_before):
        rec = external.SENT_LOG[seq]
        vel = rec["vel"].get(drone_name)
        if vel is None:
            continue
        if (h_speed(vel) - baseline) >= primary:
            twin_seq = seq
            break
    if twin_seq is not None:
        # Unity may render a slightly later packet first if one is dropped between frames;
        # take the first ack at or after twin_seq.
        candidate = [s for s in acks.acks if s >= twin_seq]
        if candidate:
            chosen = min(candidate)
            twin_ms = (acks.acks[chosen] - t0) * 1000.0

    await wait_until_settled(drone, timeout=20.0)
    return {
        "baseline_speed": baseline,
        "crossings_ms": crossings,
        "twin_ms": twin_ms,
        "twin_seq": twin_seq,
        "n_trace": len(trace),
    }


async def main(args):
    logging.basicConfig(level=logging.WARNING)
    dm = DroneManager(DroneMAVSDK, log_to_console=False)
    for plugin in ["external"]:
        await dm.load_plugin(plugin)

    acks = AckListener()
    ack_task = asyncio.create_task(acks.run())

    results = []
    streamer = None
    stream_task = None
    try:
        ok = await dm.connect_to_drone(args.name, None, None, args.address,
                                       log_telemetry=False,
                                       telemetry_frequency=args.telemetry_hz)
        if not ok:
            raise RuntimeError("Could not connect to drone")
        drone = dm.drones[args.name]
        await asyncio.sleep(2)

        if args.require_unity:
            # A subscriber is present exactly when telemetry packets start being emitted, so
            # just watch the sequence counter rather than poking at plugin internals.
            print("Waiting for a telemetry subscriber (press Play in Unity)...")
            start_seq = external._seq_counter
            t_wait = time.perf_counter()
            while external._seq_counter == start_seq and time.perf_counter() - t_wait < 180:
                await asyncio.sleep(0.5)
            if external._seq_counter == start_seq:
                raise RuntimeError("No telemetry client subscribed - is Unity running and in Play mode?")
            print("Telemetry subscriber detected.")
            # Confirm it is actually acking rendered frames, not just receiving.
            t_wait = time.perf_counter()
            while not acks.acks and time.perf_counter() - t_wait < 30:
                await asyncio.sleep(0.5)
            if not acks.acks:
                print("WARNING: subscriber present but no render-acks received. "
                      "Is HolodeckLatencyAck.cs active? Twin latency will be unavailable.")
            else:
                print("Render-acks flowing from Unity.")

        # PX4 ignores MAVLink MANUAL_CONTROL unless it is told to accept a joystick as the
        # stick source. In SITL there is no RC transmitter at all, so with the default
        # COM_RC_IN_MODE (RC only) the commands are accepted by MAVSDK and then silently
        # dropped by the autopilot -- the drone just hovers.
        try:
            await drone.system.param.set_param_int("COM_RC_IN_MODE", 1)  # 1 = joystick only
            print("Set COM_RC_IN_MODE=1 (accept MAVLink manual control)")
        except Exception as e:
            print(f"WARNING: could not set COM_RC_IN_MODE: {e!r}")
        await asyncio.sleep(1.0)

        await dm.arm(args.name)
        await asyncio.sleep(1)
        await dm.takeoff(args.name)
        await asyncio.sleep(args.settle_s)

        # MAVSDK requires manual-control input to already be streaming at >5Hz *before*
        # start_position_control() will engage, so the streamer must come first.
        streamer = StickStreamer(drone)
        stream_task = asyncio.create_task(streamer.run())
        await asyncio.sleep(1.5)

        await drone.manual_control_position()
        await asyncio.sleep(1.0)

        mode = getattr(drone.flightmode, "name", str(drone.flightmode))
        print(f"Flight mode after handover: {mode}")
        if "POSCTL" not in str(mode).upper():
            raise RuntimeError(
                f"Manual control did not engage (flight mode is {mode}, expected POSCTL). "
                f"Stick input would be ignored. streamer failures={streamer.failures} "
                f"last_error={streamer.last_error}"
            )

        for i in range(args.trials):
            direction = 1.0 if i % 2 == 0 else -1.0  # alternate so we don't drift away
            r = await run_trial(drone, streamer, acks, direction, args.hold_s, args.name)
            r["trial"] = i
            r["direction"] = direction
            results.append(r)
            print(f"trial {i:2d} dir={direction:+.0f} "
                  f"motion={ {k: round(v,1) for k,v in r['crossings_ms'].items()} } "
                  f"twin={None if r['twin_ms'] is None else round(r['twin_ms'],1)}")
    finally:
        if streamer:
            streamer.forward = 0.0
            streamer.stop()
        if stream_task:
            await asyncio.sleep(0.2)
        try:
            await dm.land(args.name)
            await asyncio.sleep(3)
            await dm.disarm(args.name)
        except Exception:
            pass
        acks.stop()
        await dm.close()

    # ---- aggregate ----
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    summary = {"tag": args.tag, "trials": len(results),
               "telemetry_hz": args.telemetry_hz,
               "stick_command_failures": streamer.failures if streamer else None,
               "stick_last_error": streamer.last_error if streamer else None,
               "command_call_ms_mean": (statistics.mean(streamer.call_durations)
                                        if streamer and streamer.call_durations else None),
               "command_call_ms_p95": (statistics.quantiles(streamer.call_durations, n=100)[94]
                                       if streamer and len(streamer.call_durations) > 20 else None)}
    for thr in THRESHOLDS:
        vals = [r["crossings_ms"][thr] for r in results if thr in r["crossings_ms"]]
        if vals:
            summary[f"motion_ms_thr{thr}_mean"] = statistics.mean(vals)
            summary[f"motion_ms_thr{thr}_median"] = statistics.median(vals)
            summary[f"motion_ms_thr{thr}_stdev"] = statistics.stdev(vals) if len(vals) > 1 else 0.0
            summary[f"motion_ms_thr{thr}_n"] = len(vals)
    twins = [r["twin_ms"] for r in results if r["twin_ms"] is not None]
    if twins:
        summary["twin_ms_mean"] = statistics.mean(twins)
        summary["twin_ms_median"] = statistics.median(twins)
        summary["twin_ms_stdev"] = statistics.stdev(twins) if len(twins) > 1 else 0.0
        summary["twin_ms_n"] = len(twins)

    # True DroneManager -> Unity -> rendered latency, with the *real* Unity process in the loop
    # (separate Windows process, real UDP loopback, real JSON parse, real render). This is the
    # honest counterpart to the in-process transport proxy measured in telemetry_latency_bench.
    render_lat = []
    for seq, t_ack in acks.acks.items():
        rec = external.SENT_LOG.get(seq)
        if rec:
            render_lat.append((t_ack - rec["t_send"]) * 1000.0)
    if render_lat:
        rq = statistics.quantiles(render_lat, n=100) if len(render_lat) > 20 else None
        summary["packet_to_render_ms_mean"] = statistics.mean(render_lat)
        summary["packet_to_render_ms_median"] = statistics.median(render_lat)
        summary["packet_to_render_ms_stdev"] = (statistics.stdev(render_lat)
                                                if len(render_lat) > 1 else 0.0)
        summary["packet_to_render_ms_p95"] = rq[94] if rq else max(render_lat)
        summary["packet_to_render_ms_p99"] = rq[98] if rq else max(render_lat)
        summary["packet_to_render_ms_n"] = len(render_lat)

    with open(out_dir / f"e2e_{args.tag}_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    with open(out_dir / f"e2e_{args.tag}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trial", "direction", "baseline_speed"]
                   + [f"motion_ms_thr{t}" for t in THRESHOLDS] + ["twin_ms", "twin_seq"])
        for r in results:
            w.writerow([r["trial"], r["direction"], f"{r['baseline_speed']:.4f}"]
                       + [r["crossings_ms"].get(t, "") for t in THRESHOLDS]
                       + [r["twin_ms"] if r["twin_ms"] is not None else "", r["twin_seq"] or ""])
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--address", default="udp://172.30.235.63:18570")
    p.add_argument("--name", default="kirk")
    p.add_argument("--trials", type=int, default=12)
    p.add_argument("--hold-s", type=float, default=1.5, help="How long to hold the stick per trial")
    p.add_argument("--settle-s", type=float, default=6.0, help="Hover settle time after takeoff")
    p.add_argument("--telemetry-hz", type=float, default=50.0,
                   help="MAVSDK telemetry rate; bounds how finely motion onset can be resolved")
    p.add_argument("--require-unity", action="store_true",
                   help="Wait for a telemetry subscriber (Unity in Play mode) before starting")
    p.add_argument("--out-dir", default="benchmarks/results")
    p.add_argument("--tag", default="e2e_run1")
    asyncio.run(main(p.parse_args()))

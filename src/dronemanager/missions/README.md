# LiveGS capture missions

Missions that fly the LiveGS drone rig — a drone-mounted Ouster OS-DOME + Intel
D455 — to collect posed image/LiDAR pairs for Gaussian-splat reconstruction.

They ship here, alongside `uam.py` and `engel.py`, but they are **maintained in
the LiveGS repository** at `Scripts/dm_missions/`. Fix things there and copy
them back, or the next LiveGS change will silently overwrite the edit.

| file | what |
|---|---|
| `squarecapture.py` | the mission: square pattern, yaw sweep, rig capture at every stop |
| `squarecapture_plan.py` | the geometry — pure stdlib, no DroneManager import, unit-tested |
| `drone_capture_client.py` | stdlib-only client for the capture daemon on the drone |

`squarecapture_plan` and `drone_capture_client` appear in `mission-load`'s
completion list because it lists every `.py` in this directory. They are not
missions; loading them fails harmlessly.

Tests live in LiveGS: `tests/test_squarecapture_plan.py` (30) and
`tests/test_squarecapture_mission.py` (30), which stub DroneManager in
`sys.modules` and run the mission against a real mock capture daemon over a real
socket.

## What it does

A 10 m square at 10 m altitude, a stop at each of the 4 corners, 8 headings per
stop in 45° steps — **32 posed image/LiDAR pairs**:

```
corner 0  N  +0.00 E  +0.00 D -10.00  arrive yaw   -22.5  headings -22.5,+22.5,+67.5,+112.5,+157.5,-157.5,-112.5,-67.5
corner 1  N +10.00 E  +0.00 D -10.00  arrive yaw   -67.5  headings -67.5,-22.5,+22.5,+67.5,+112.5,+157.5,-157.5,-112.5
corner 2  N +10.00 E +10.00 D -10.00  arrive yaw  -112.5  headings -112.5,-67.5,-22.5,+22.5,+67.5,+112.5,+157.5,-157.5
corner 3  N  +0.00 E +10.00 D -10.00  arrive yaw  -157.5  headings -157.5,-112.5,-67.5,-22.5,+22.5,+67.5,+112.5,+157.5
4 stations, 32 captures
```

Each sweep runs 315° in one direction, and the next corner starts on the heading
the last one ended on — so the transit legs need no rotation at all.

The mission sequences **flight only**. The capture itself — sensor grab, per-beam
averaging, depth projection, FC-pose stamping, tlog — belongs to a daemon running
on the drone's Jetson, reached over `ssh -W` by `drone_capture_client.Link`.
Nothing here touches MAVLink directly.

## Run

```
connect Romulus udp://:14561
mission-load squarecapture --name sq
sq-add Romulus
sq-plan                  # print the schedule, fly nothing
sq-check                 # preflight: daemon, sensors, GPS, battery
sq-run flight01          # arm, takeoff, 32 captures, return, land
sq-status
sq-abort                 # stop and land where we are
```

Options:

`sq-run <session> [--side 10] [--altitude 10] [--scans 10] [--settle 2]
[--exposure 2] [--gate warn]`

`--gate` is the daemon's motion gate: `warn` records the gyro peak and captures
anyway, `block` refuses while moving, `off` ignores it. `warn` is the default
because hover vibration on this airframe is uncharted and a `block` gate could
refuse all 32 captures.

`sq-link <host> [--user u] [--port p] [--direct yes]` points the mission at a
different capture daemon; `--direct yes` is plain TCP instead of `ssh -W`.

Afterwards, pull the session and **check it**:

```bash
python Scripts/drone_capture_client.py --host 192.168.1.55 pull flight01 --dest recordings/flight01
python Scripts/capture_check.py recordings/flight01 --expect 32
```

The mission's own "32/32 captures" is a tally of daemon replies — a claim about
the wire, not the disk. It cannot see a truncated cloud, an image written at the
wrong resolution after a camera restart, a capture whose pose was absent, or a
hole in the heading sweep. `capture_check.py` opens the files and looks.

## Rehearse first — both halves work on the ground

**SITL**, no hardware at all. Run LiveGS's `mock_capture_daemon.py`, which speaks
the daemon protocol and writes placeholder files:

```bash
python Scripts/mock_capture_daemon.py --port 5757 --fail-every 5
```
```
connect Romulus udp://:14540          # PX4 SITL
mission-load squarecapture --name sq
sq-add Romulus
sq-link 127.0.0.1 --direct yes --port 5757
sq-run test01
```

`--fail-every 5` refuses every fifth shot so the retry-and-continue path runs
rather than being assumed.

★ **Verified 2026-08-29** against Gazebo + the mock: 32/32 in 197 s, all four
stations, every sweep complete, retries recovered.

**Bench dry run** — real daemon, real sensors, props off, no flight command
issued at all:

```
sq-dryrun bench02
```

## Prerequisites

1. **A passphrase-less SSH key on the ground station.** The mission drives the
   daemon over `ssh -W` as a non-interactive subprocess, so there is nothing to
   type a passphrase into. The failure is easy to misread: verbose ssh prints
   `Server accepts key` and then moves on to the next identity, because it can
   authorise the key but cannot *sign* with it. The mission just reports
   "Capture daemon unreachable".
   ```powershell
   ssh-keygen -t ed25519 -f $env:USERPROFILE\.ssh\id_drone -N '""' -C dronemanager-capture
   type $env:USERPROFILE\.ssh\id_drone.pub | ssh dronetrekkers@192.168.1.55 "cat >> .ssh/authorized_keys"
   ```
   Pin it in `%USERPROFILE%\.ssh\config`, or a passphrase-protected key earlier
   in the search order is offered instead:
   ```
   Host 192.168.1.55
     HostName 192.168.1.55
     User dronetrekkers
     IdentityFile ~/.ssh/id_drone
     IdentitiesOnly yes
   ```
   ⚠ Strip the CR when appending — a `.pub` read from a Windows file has CRLF.

2. **Connect on 14561, not 14550.** QGroundControl listens on 14550 by default,
   and DroneManager has no MAVLink router (`mavpassthrough.py:14`), so the two
   cannot share a port: whichever starts second fails to bind and reports the
   vehicle unreachable, with nothing pointing at a port conflict. mavproxy on
   the drone unicasts to both — 14550 for QGC, 14561 for DroneManager — so QGC
   can stay up as an independent safety monitor during an autonomous flight.
   ⚠ Never use `udp://<drone-ip>:14550`. That is mavproxy's master socket, the
   one holding the FC link, and whichever process talks to the FC on 14550
   becomes its peer.

3. **A GPS fix.** ⚠ `SYS_STATUS`'s GPS present bit is **not** a reliable test for
   whether a receiver exists — measured False on PX4 v1.17 while both receivers
   were connected and streaming UBX at over 1 kB/s. The authority is the flight
   controller itself:
   ```
   nsh> gps status
   ```
   `status: OK, port: /dev/ttyS0, baudrate: 115200` with a non-zero
   `rate reading` means the receiver is present and talking. `satellites_used: 0`
   then means sky view and nothing else.

4. **Offboard-loss failsafe.** Setpoints now cross Wi-Fi and mavproxy before
   reaching the FC. Check `COM_OF_LOSS_T` and set `COM_OBL_RC_ACT` to Hold or
   Position so a dropout produces a hover, not a flyaway. Keep the RC
   transmitter live: a mode switch overrides offboard and nothing in software
   can countermand it.

## Where the mission may be installed

Two locations, with **different import semantics** — `squarecapture.py` handles
both, and the difference is worth knowing before moving files around.

* **Here**, `dronemanager/missions/`: `MetaPlugin` imports it as a real package
  module, so the siblings arrive by relative import and `sys.path` is untouched.
* **`Documents/DroneManager/missions/`**: loaded by file path under a parent
  package that does not exist, so the relative import fails and the directory is
  appended to `sys.path` — the only way the siblings become importable.

All three files must travel together either way.

## Things that will bite

Found by reading DroneManager's source after symptoms that looked like something
else. None are documented upstream.

- **`is_at_heading` does not wrap** (`drone.py:399`). It compares
  `abs(cur - target)` after normalising the target into (-180, 180], so a target
  of exactly 180 becomes -180 and a drone at +179.9 computes 359.8 — `fly_to`
  and `yaw_to` both hang **forever**. `HEADINGS_8` is offset 22.5° to stay clear,
  and `capture_plan` refuses a heading set that is not.
- **`yaw_to` divides by a step count derived from the yaw delta**, so a delta of
  exactly zero is a `ZeroDivisionError`, not a no-op. The mission skips the call
  under 0.5°.
- **`yaw_to` deactivates the path follower** and streams its own setpoints. Pass
  `local=` explicitly or it re-samples a noisy `position_ned` as the hold point
  and the station drifts across the sweep.
- **`Drone.set_fence` is broken** (`drone.py:420`): it passes the logger as the
  first positional argument, which lands in `north_lower`. Use
  `dm.set_fence(name, RectLocalFence(...))`, which assigns a constructed
  instance.
- **Manager actions return exceptions, they do not raise them.**
  `_multiple_drone_action` uses `gather(return_exceptions=True)` and returns a
  LIST; a name-lookup miss returns `None` outright.
- **String vs list `names` is not cosmetic.** `fly_to`/`yaw_to` only unwrap raw
  per-drone arguments (`local=[n,e,d]` rather than `[[n,e,d]]`) when `names` is a
  bare string. `arm`/`takeoff`/`land`/`disarm` take the other path and want a
  list.
- **`land` neither disarms nor leaves offboard.** Follow it with `disarm` and
  `change_flightmode(name, "position")`, as `uam.py` does. In SITL with no RC
  that last call is denied with `Input Not Set`; harmless, and it succeeds on the
  real aircraft with the transmitter on.
- **Local NED is the EKF origin, not the takeoff point.** The square is anchored
  to `position_ned` read at run time; a fence built around (0, 0) would reject
  every waypoint of the flight.
- **Blocking calls stall the setpoint stream.** `PathFollower.follow` runs on the
  same event loop and PX4 drops out of offboard after ~0.5 s without setpoints,
  so the capture client is driven through `run_in_executor`, never inline.

## The data caveat that outlives the flight

FC pose is stamped into every capture as `trust: "prior_only"`, and that label is
the truth: splat-grade work needs attitude to 0.0092° and no flight controller is
close. These captures are a registration **seed**, not an answer.

The rig also hangs **25 cm below the flight controller**, and the lever arm
rotates with the airframe — at 10° of pitch it is no longer 25 cm of altitude.
`squarecapture_plan.PAYLOAD_OFFSET_FC_M` and `payload_position_ned()` carry it.
Because the offset is identical on every capture it will **not** show up as
registration noise; it biases the whole map the same way.

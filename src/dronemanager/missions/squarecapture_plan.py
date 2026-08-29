"""Flight-plan geometry for the square capture mission.

STDLIB ONLY, and it imports nothing from DroneManager or LiveGS on purpose:
everything here is arithmetic that decides where the drone stops and which way
it points, which is exactly the part that must be unit-tested on a workstation
rather than debugged at 10 m over a field. `squarecapture.py` is the thin async
shell that flies what this module computes.

The default plan is the campaign's dataset: a 10 m square at 10 m altitude,
a stop at each of the 4 corners, 8 headings per stop in 45 deg steps
=> 32 image/LiDAR pairs.

    >>> stations = capture_plan()
    >>> sum(len(s.captures) for s in stations)
    32
"""

import math
from collections import namedtuple

# --------------------------------------------------------------------------- #
#  Rig geometry
# --------------------------------------------------------------------------- #

#: Where the LiDAR/camera payload sits relative to the flight controller, in the
#: FC body frame (FRD: x forward, y right, z DOWN). The payload hangs 25 cm
#: below the FC, so z is POSITIVE.
#:
#: The capture daemon stamps FC pose and never launders it (`trust:
#: "prior_only"`). Whatever consumes those priors must push them through
#: `payload_position_ned` first — 25 cm is an order of magnitude larger than the
#: position accuracy the prior is there to seed, so skipping it is not a rounding
#: error, it is a systematic offset that biases every capture the same way and
#: therefore will NOT show up as registration noise.
PAYLOAD_OFFSET_FC_M = (0.0, 0.0, 0.25)

# --------------------------------------------------------------------------- #
#  Headings
# --------------------------------------------------------------------------- #

#: The 8 sweep headings, 45 deg apart, deliberately offset by 22.5 deg from the
#: cardinals.
#:
#: WHY THE OFFSET: DroneManager's `Drone.is_at_heading` (drone.py:399) compares
#: `abs(cur - target) < tolerance` with NO angular-difference wrap. It normalises
#: the target into (-180, 180], so a target of exactly 180 becomes -180, and a
#: drone sitting at +179.9 computes `abs(179.9 - -180) = 359.8` and never
#: converges: `fly_to` and `yaw_to` both spin on that check forever. Rotating the
#: whole set by 22.5 deg keeps every target 22.5 deg clear of the discontinuity
#: and costs the dataset nothing — the scene does not care which way is 0.
HEADINGS_8 = (-157.5, -112.5, -67.5, -22.5, 22.5, 67.5, 112.5, 157.5)

#: How close a heading may come to the +/-180 discontinuity before it is unsafe.
HEADING_WRAP_GUARD_DEG = 5.0


def normalize_heading(deg):
    """Fold a heading into (-180, 180], the range DroneManager commands in."""
    out = (float(deg) + 180.0) % 360.0 - 180.0
    # (-180 + 180) % 360 - 180 == -180; prefer the +180 end so the value stays
    # inside the half-open range the docstring promises.
    return 180.0 if out == -180.0 else out


def heading_delta(from_deg, to_deg):
    """Signed shortest rotation from one heading to another, in (-180, 180]."""
    return normalize_heading(to_deg - from_deg)


def is_wrap_safe(deg, guard=HEADING_WRAP_GUARD_DEG):
    """True if `is_at_heading` can actually converge on this target."""
    return abs(abs(normalize_heading(deg)) - 180.0) > guard


def order_headings(headings, from_yaw):
    """Rotate `headings` so the sweep starts at whichever one is closest to the
    drone's current heading, then proceeds monotonically around the circle.

    Every heading is visited exactly once either way, so the direction is free;
    the start is not. Beginning at the nearest heading saves up to 180 deg of
    rotation on arrival, and because the same 8 headings are used at every
    station, station N+1 starts on the heading station N ended on — the transit
    leg between corners needs no rotation at all.
    """
    ring = sorted(normalize_heading(h) for h in headings)
    if not ring:
        return []
    # Tie-break on the heading itself so a drone pointing exactly between two
    # candidates still produces one deterministic plan.
    start = min(range(len(ring)),
                key=lambda i: (abs(heading_delta(from_yaw, ring[i])), ring[i]))
    return ring[start:] + ring[:start]


# --------------------------------------------------------------------------- #
#  The plan
# --------------------------------------------------------------------------- #

Capture = namedtuple("Capture", "corner heading tag")
Station = namedtuple("Station", "corner ned arrival_yaw captures")


def square_corners(side_m):
    """The 4 corners of a square as (north, east) offsets from the drone's
    position at mission start. The operator therefore places the drone at the
    SOUTH-WEST corner and the square grows north and east from there.

    Returned in a traversal order that walks the perimeter (no diagonal).
    """
    s = float(side_m)
    return [(0.0, 0.0), (s, 0.0), (s, s), (0.0, s)]


def format_tag(corner, heading):
    """The `tag` string handed to the capture daemon, e.g. `c0_h-157.5`.

    It carries the SIGNED NED yaw that was actually commanded, not a compass
    bearing, so the tag can be compared against the recorded FC attitude without
    a convention conversion in between — that conversion is precisely where the
    tripod sessions kept losing an hour.
    """
    return "c%d_h%+06.1f" % (int(corner), normalize_heading(heading))


def capture_plan(side_m=10.0, altitude_m=10.0, origin_ned=(0.0, 0.0, 0.0),
                 headings=HEADINGS_8, initial_yaw=0.0):
    """Build the full station/heading schedule.

    :param side_m: square edge length, metres.
    :param altitude_m: height above the mission-start position, metres (positive
        up; it is converted to NED down internally).
    :param origin_ned: the drone's local NED position when the mission starts,
        which is the square's south-west corner. The local NED origin is the EKF
        origin, NOT the takeoff point, so this must be read from telemetry at run
        time rather than assumed to be zero.
    :param initial_yaw: the drone's heading at mission start, used only to pick
        the first station's sweep start.
    :returns: list of `Station`, each with an absolute NED target, the yaw to
        arrive on, and its ordered `Capture` list.
    """
    if side_m <= 0:
        raise ValueError("side_m must be positive")
    if altitude_m <= 0:
        raise ValueError("altitude_m must be positive")
    bad = [h for h in headings if not is_wrap_safe(h)]
    if bad:
        raise ValueError(
            "headings %s sit on the +/-180 discontinuity; is_at_heading cannot "
            "converge on them (see HEADINGS_8)" % (bad,))

    o_n, o_e, o_d = (float(v) for v in origin_ned)
    down = o_d - float(altitude_m)

    stations, yaw_cursor = [], float(initial_yaw)
    for idx, (dn, de) in enumerate(square_corners(side_m)):
        ordered = order_headings(headings, yaw_cursor)
        stations.append(Station(
            corner=idx,
            ned=(o_n + dn, o_e + de, down),
            arrival_yaw=ordered[0],
            captures=[Capture(idx, h, format_tag(idx, h)) for h in ordered],
        ))
        yaw_cursor = ordered[-1]
    return stations


def plan_summary(stations):
    """One human-readable line per station, for the mission log."""
    lines = []
    for s in stations:
        lines.append("corner %d  N%+7.2f E%+7.2f D%+7.2f  arrive yaw %+7.1f  "
                     "headings %s"
                     % (s.corner, s.ned[0], s.ned[1], s.ned[2], s.arrival_yaw,
                        ",".join("%+.1f" % c.heading for c in s.captures)))
    lines.append("%d stations, %d captures"
                 % (len(stations), sum(len(s.captures) for s in stations)))
    return lines


# --------------------------------------------------------------------------- #
#  Fence
# --------------------------------------------------------------------------- #

def fence_bounds(origin_ned=(0.0, 0.0, 0.0), side_m=10.0, altitude_m=10.0,
                 margin_m=2.0, ceiling_margin_m=3.0, ground_margin_m=1.0):
    """Arguments for `RectLocalFence`, in its declared order
    ``(north_lower, north_upper, east_lower, east_upper, down_lower, down_upper)``.

    Two traps this exists to avoid:

    * The fence is in ABSOLUTE local NED, but the square is defined relative to
      where the drone started, and the EKF origin is somewhere else entirely. A
      fence built around (0, 0) would reject every waypoint.
    * `down` is negative-up and the class asserts ``lower < upper``, so the
      *ceiling* is the lower bound. The ground must stay INSIDE the box or
      takeoff and landing setpoints fall outside it, hence `ground_margin_m`.
    """
    o_n, o_e, o_d = (float(v) for v in origin_ned)
    s, m = float(side_m), float(margin_m)
    return (o_n - m, o_n + s + m,
            o_e - m, o_e + s + m,
            o_d - float(altitude_m) - float(ceiling_margin_m),
            o_d + float(ground_margin_m))


# --------------------------------------------------------------------------- #
#  Pose priors
# --------------------------------------------------------------------------- #

def _rot_body_to_ned(roll_deg, pitch_deg, yaw_deg):
    """Aerospace ZYX (yaw-pitch-roll) rotation from FRD body to NED, row-major."""
    cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    cp, sp = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp,     cp * sr,                cp * cr),
    )


def payload_position_ned(fc_ned, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0,
                         offset_fc_m=PAYLOAD_OFFSET_FC_M):
    """Where the LiDAR/camera actually was, given where the FC says it was.

    The lever arm rotates with the airframe, so at 10 deg of pitch the 25 cm drop
    is no longer 25 cm of altitude — it is 4 cm of horizontal offset as well.
    Level flight is the special case, not the rule.
    """
    R = _rot_body_to_ned(roll_deg, pitch_deg, yaw_deg)
    return tuple(float(fc_ned[i]) + sum(R[i][j] * offset_fc_m[j] for j in range(3))
                 for i in range(3))

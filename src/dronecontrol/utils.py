import math
from urllib.parse import urlparse
import numpy as np
import logging
from pathlib import Path
import socket
import inspect
import typing
import types
import asyncio
from haversine import inverse_haversine, haversine, Direction, Unit

common_formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)s %(name)s - %(message)s', datefmt="%H:%M:%S")

CACHE_DIR = Path(__file__).parent.parent.parent.joinpath(".cache")

LOG_DIR = Path(__file__).parent.parent.parent.joinpath("logs")

EARTH_RADIUS = 6371000


def dist_ned(pos1, pos2):
    return np.sqrt(np.sum((pos1 - pos2) ** 2, axis=0))


def dist_gps(gps1, gps2):
    dist_horiz = haversine((gps1[0], gps1[1]), (gps2[0], gps2[1]), unit=Unit.METERS)
    dist_alt = gps1[2] - gps2[2]
    return math.sqrt(dist_horiz*dist_horiz + dist_alt*dist_alt)


def heading_ned(pos1, pos2):
    """ Heading from pos1 to pos2, going from -180 to +180 with 0 straight north.

    pos1 and pos2 must be NED arrays."""
    return math.atan2(pos2[1] - pos1[1], pos2[0] - pos1[0]) / math.pi * 180


def heading_gps(gps1, gps2):
    """ Heading between GPS coordinates, going from -180 to +180 with 0 straight north.

    Note that the GPS coordinates include an altitude, which is ignored in this function
    """
    diff_long_rad = (gps2[1] - gps1[1]) * math.pi / 180
    lat1_rad = gps1[0] * math.pi / 180
    lat2_rad = gps2[0] * math.pi / 180
    x = math.sin(diff_long_rad) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(diff_long_rad)
    return math.atan2(x, y) * 180 / math.pi


def relative_gps(north, east, up, lat, long, alt):
    """ Given a NEU offset and a GPS position, calculate the GPS coordinates of the offset position."""
    target_alt = alt + up
    coords = (lat, long)
    coords = inverse_haversine(coords, north, Direction.NORTH, unit=Unit.METERS)
    target_lat, target_long = inverse_haversine(coords, east, Direction.EAST, unit=Unit.METERS)
    return target_lat, target_long, target_alt


def offset_from_gps(origin, gps1, gps2):
    """ Given two GPS points, computes a heading and distance between the first two and then creates a new point
    separated fom the origin by the same heading and distance"""
    dist_horiz = haversine((gps1[0], gps1[1]), (gps2[0], gps2[1]), unit=Unit.METERS)
    dist_alt = gps2[2] - gps1[2]
    heading = heading_gps(gps1, gps2)
    lat, long = inverse_haversine(origin[:2], dist_horiz, heading, unit=Unit.METERS)
    return [lat, long, origin[2] + dist_alt]

def ned_from_gps(gps1, gps2):
    """Given two GPS points, compute the NED difference between the two positions, same as PX4 http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html"""
    lat1_rad = gps1[0]*math.pi / 180
    lat2_rad = gps2[0]*math.pi / 180
    long1_rad = gps1[1]*math.pi / 180
    long2_rad = gps2[1]*math.pi / 180
    cos_long_diff = math.cos(long2_rad-long1_rad)

    arg = math.sin(lat1_rad)*math.sin(lat2_rad) + math.cos(lat1_rad)*math.cos(lat2_rad)*cos_long_diff
    if arg > 1.0:
        arg = 1.0
    if arg < -1.0:
        arg = -1.0
    c = math.acos(arg)
    k = 1.0
    if abs(c) > 0:
        k = c / math.sin(c)

    north = k * (math.cos(lat1_rad)*math.sin(lat2_rad)-math.sin(lat1_rad)*math.cos(lat2_rad)*cos_long_diff) * EARTH_RADIUS
    east = k * (math.cos(lat2_rad) * math.sin(long2_rad-long1_rad)) * EARTH_RADIUS
    down = gps1[2] - gps2[2]
    return north, east, down

def get_free_port():
    """ Get a free network port.

    The port is not guaranteed to be free once the function returns.

    :return:
    """
    sock = socket.socket()
    sock.bind(("", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def parse_address(string):
    """ Used to ensure that udp://:14540, udp://localhost:14540 and udp://127.0.0.1:14540 are recognized as equivalent.

    Missing elements from the string or the other entries are replaced with defaults. These are udp, empty host and
    50051 for the scheme, host and port, respectively.
    """
    scheme, rest = string.split("://")
    if scheme == "serial":
        loc, append = rest.split(":")
    else:
        parse_drone_addr = urlparse(string)
        scheme = parse_drone_addr.scheme
        loc = parse_drone_addr.hostname
        append = parse_drone_addr.port
        if scheme is None:
            scheme = "udp"
        if loc is None:
            loc = ""
        if loc == "localhost":
            loc = ""
        elif loc == "127.0.0.1":
            loc = ""
        if append is None:
            append = 50051
    return scheme, loc, append


def check_cli_command_signatures(command):
    """

    If a signature is invalid, the other fields may not be populated correctly or at all.

    :param command:
    :return:
    """
    sig = inspect.signature(command)
    args_invalid = []
    args_name = []
    args_list = []
    args_required = []
    args_accepts_none = []
    args_types = []
    args_kwonly = []
    args_has_defaults = []
    args_defaults = []
    for param in sig.parameters.values():
        is_invalid = False
        is_list = False
        is_required = param.default is param.empty
        accepts_none = False
        param_type = str
        is_kwonly = param.kind is param.KEYWORD_ONLY
        has_default = param.default is not param.empty
        default = param.default if has_default else None

        # Cases we accept: Direct type, List of a type, Union of a type and None, Union of None and a list of a type
        if param.annotation is None or param.annotation is param.empty or len(typing.get_args(param.annotation)) > 2:
            is_invalid = True
        # Looking at the case of a union with a raw type or a union with a raw type and a list
        if typing.get_origin(param.annotation) in (typing.Union, types.UnionType):
            if type(None) not in typing.get_args(param.annotation):
                is_invalid = True
            accepts_none = True
            for type_arg in typing.get_args(param.annotation):
                if type_arg is type(None):  # Skip the None arg
                    continue
                elif typing.get_origin(type_arg) is not None:  # If its a container of some sort
                    if not typing.get_origin(type_arg) is list:  # Invalid if not a list
                        is_invalid = True
                    else:
                        is_list = True
                        if len(typing.get_args(type_arg)) > 1:  # Invalid if multiple arguments
                            is_invalid = True
                        else:
                            list_internal_type = typing.get_args(type_arg)[0]
                            # Check that we dont have nested lists:
                            if typing.get_origin(list_internal_type) is not None:
                                is_invalid = True
                            param_type = list_internal_type
                else:  # Parameter is not None or a container
                    is_list = False
                    param_type = type_arg
        else:
            if typing.get_origin(param.annotation) is not None:  # If it's a container of some sort
                if not typing.get_origin(param.annotation) is list:  # Invalid if not a list
                    is_invalid = True
                else:
                    is_list = True
                    if len(typing.get_args(param.annotation)) > 1:  # Invalid if multiple arguments
                        is_invalid = True
                    else:
                        list_internal_type = typing.get_args(param.annotation)[0]
                        # Check that we dont have nested lists:
                        if typing.get_origin(list_internal_type) is not None:
                            is_invalid = True
                        param_type = list_internal_type
            else:
                is_list = False
                param_type = param.annotation
        name = param.name
        args_invalid.append(is_invalid)
        args_name.append(name)
        args_list.append(is_list)
        args_required.append(is_required)
        args_accepts_none.append(accepts_none)
        args_types.append(param_type)
        args_kwonly.append(is_kwonly)
        args_has_defaults.append(has_default)
        args_defaults.append(default)

    return list(zip(args_invalid, args_name, args_list, args_required, args_accepts_none, args_types, args_kwonly, args_has_defaults, args_defaults))


async def coroutine_awaiter(task: asyncio.Task, logger):
    try:
        if isinstance(task, asyncio.Task):
            await task
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logger.error(f"Encountered an exception in a coroutine! See the log for more details")
        logger.debug(e, exc_info=True)

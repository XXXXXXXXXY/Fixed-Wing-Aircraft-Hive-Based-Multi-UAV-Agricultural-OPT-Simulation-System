#!/usr/bin/env python3
"""MAVLink bridge from scout_opt scenario JSON to ArduPilot SITL.

First bridge milestone:
- read QGC Fence-derived scenario JSON
- assign scout_boundary_routes to drones by SYSID
- arm, take off, and fly each Scout around its closed boundary route
- keep QGroundControl visualization alive through the normal 14550 path

Start SITL with scripts/start_ardupilot_sitl.ps1 first. That script opens
SERIAL1 udpclient ports 14600 + instance for this bridge.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pymavlink import mavutil


@dataclass
class LocalOrigin:
    lat: float
    lon: float


def local_to_latlon(point: dict[str, float], origin: LocalOrigin) -> tuple[float, float]:
    lat = origin.lat + point["y"] / 111_111.0
    lon = origin.lon + point["x"] / (111_111.0 * math.cos(math.radians(origin.lat)))
    return lat, lon


def wait_heartbeat(conn: mavutil.mavfile, sysid: int, timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg is not None and msg.get_srcSystem() == sysid:
            return
    raise TimeoutError(f"no heartbeat from sysid={sysid}")


def ack_command(conn: mavutil.mavfile, command: int, timeout_s: float = 8.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if msg is None:
            continue
        if getattr(msg, "command", None) == command:
            return msg.result in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            )
    return False


def set_mode(conn: mavutil.mavfile, mode: str) -> None:
    mode_id = conn.mode_mapping().get(mode)
    if mode_id is None:
        raise RuntimeError(f"mode {mode} not available; known={sorted(conn.mode_mapping())}")
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )


def arm_and_takeoff(conn: mavutil.mavfile, altitude_m: float) -> None:
    set_mode(conn, "GUIDED")
    time.sleep(0.5)
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    ack_command(conn, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout_s=8)
    time.sleep(0.5)
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude_m,
    )
    ack_command(conn, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout_s=8)


def goto_global(conn: mavutil.mavfile, lat: float, lon: float, alt_m: float) -> None:
    conn.mav.set_position_target_global_int_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def current_position(conn: mavutil.mavfile) -> tuple[float, float, float] | None:
    msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
    if msg is None:
        return None
    return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0


def distance_m(a_lat: float, a_lon: float, b_lat: float, b_lon: float) -> float:
    north = (b_lat - a_lat) * 111_111.0
    east = (b_lon - a_lon) * 111_111.0 * math.cos(math.radians(a_lat))
    return math.hypot(north, east)


def fly_route(conn: mavutil.mavfile, route: dict[str, Any], origin: LocalOrigin, dwell_s: float) -> None:
    altitude = float(route.get("altitude_m", 28.0))
    points = route.get("points") or []
    for idx, point in enumerate(points):
        lat, lon = local_to_latlon(point, origin)
        print(f"sysid={conn.target_system} scout point {idx + 1}/{len(points)} lat={lat:.7f} lon={lon:.7f}")
        goto_global(conn, lat, lon, altitude)
        deadline = time.time() + 45.0
        while time.time() < deadline:
            pos = current_position(conn)
            if pos is not None and distance_m(pos[0], pos[1], lat, lon) < 8.0:
                break
            goto_global(conn, lat, lon, altitude)
            time.sleep(1.0)
        time.sleep(dwell_s)


def connect_vehicle(port: int, sysid: int, timeout_s: float) -> mavutil.mavfile:
    conn = mavutil.mavlink_connection(
        f"udpin:127.0.0.1:{port}",
        source_system=240 + sysid,
        autoreconnect=True,
        robust_parsing=True,
    )
    wait_heartbeat(conn, sysid=sysid, timeout_s=timeout_s)
    conn.target_system = sysid
    conn.target_component = 1
    print(f"connected sysid={sysid} on udp:{port}")
    return conn


def run_bridge(args: argparse.Namespace) -> int:
    scenario = json.loads(Path(args.scenario).read_text(encoding="utf-8"))
    origin_data = scenario.get("origin")
    if not origin_data:
        raise SystemExit("scenario is missing origin; regenerate it with qgc_plan_to_scenario.py")
    origin = LocalOrigin(lat=float(origin_data["lat"]), lon=float(origin_data["lon"]))
    routes = scenario.get("scout_boundary_routes") or []
    if not routes:
        raise SystemExit("scenario has no scout_boundary_routes")

    vehicles: list[mavutil.mavfile] = []
    for index, route in enumerate(routes[: args.max_scouts]):
        sysid = index + 1
        port = args.base_port + index
        conn = connect_vehicle(port, sysid=sysid, timeout_s=args.timeout)
        vehicles.append(conn)
        print(f"arming scout sysid={sysid}")
        arm_and_takeoff(conn, altitude_m=float(route.get("altitude_m", args.altitude)))
        time.sleep(args.takeoff_settle_s)

    for conn, route in zip(vehicles, routes[: args.max_scouts]):
        fly_route(conn, route, origin, dwell_s=args.dwell_s)
        if args.rtl:
            set_mode(conn, "RTL")
            print(f"sysid={conn.target_system} RTL")

    print("scout bridge complete")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Fly Scout boundary routes from a scout_opt scenario in ArduPilot SITL.")
    parser.add_argument("scenario", help="scenario JSON generated by qgc_plan_to_scenario.py")
    parser.add_argument("--base-port", type=int, default=14600, help="SERIAL1 UDP input base port from start_ardupilot_sitl.ps1")
    parser.add_argument("--max-scouts", type=int, default=2, help="maximum scout drones/routes to run")
    parser.add_argument("--altitude", type=float, default=28.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--takeoff-settle-s", type=float, default=6.0)
    parser.add_argument("--dwell-s", type=float, default=1.0)
    parser.add_argument("--rtl", action="store_true", help="return scouts to launch after boundary scan")
    args = parser.parse_args()
    return run_bridge(args)


if __name__ == "__main__":
    raise SystemExit(main())

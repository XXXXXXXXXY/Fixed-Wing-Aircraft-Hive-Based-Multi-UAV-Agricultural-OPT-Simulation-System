#!/usr/bin/env python3
"""QGroundControl live demo bridge for scout_opt scenarios.

This bridge is intentionally independent from ArduPilot SITL control loops.
It streams MAVLink heartbeats and live positions to QGroundControl so the
project can demonstrate:
- scout drones following operator-drawn fence/field boundaries
- multiple field blocks from a QGC .plan import
- Hive/mothership moving through planned depot sites

Entity mapping:
- SYSID 1..N: Scout drones
- SYSID 200: Hive / mobile mothership, shown as a ground rover
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


DRONE_SYSID_BASE = 1
HIVE_SYSID = 200
FIXED_WING_SYSID_BASE = 100
MPS_PER_KMH = 1000.0 / 3600.0

MINIMAL_PARAMS: list[tuple[str, float, int]] = [
    ("SYSID_THISMAV", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("FRAME_CLASS", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("ARMING_CHECK", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    ("WPNAV_SPEED", 1200.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ("RTL_ALT", 1500.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
]


@dataclass
class Origin:
    lat: float
    lon: float


@dataclass
class Entity:
    sysid: int
    mav_type: int
    route: list[dict[str, float]]
    speed_mps: float
    altitude_m: float
    loop: bool
    route_index: int = 0
    position: dict[str, float] | None = None
    reached_route_end: bool = False


@dataclass
class WorkStrip:
    block_id: int
    start: dict[str, float]
    end: dict[str, float]
    center: dict[str, float]
    depot_index: int
    length_m: float = 0.0
    area_ha: float = 0.0


def local_to_latlon(point: dict[str, float], origin: Origin) -> tuple[float, float]:
    lat = origin.lat + point["y"] / 111_111.0
    lon = origin.lon + point["x"] / (111_111.0 * math.cos(math.radians(origin.lat)))
    return lat, lon


def distance_m(a: dict[str, float], b: dict[str, float]) -> float:
    return math.hypot(b["x"] - a["x"], b["y"] - a["y"])


def heading_deg(a: dict[str, float], b: dict[str, float]) -> float:
    return (math.degrees(math.atan2(b["x"] - a["x"], b["y"] - a["y"])) + 360.0) % 360.0


def advance_entity(entity: Entity, dt_s: float) -> float:
    if not entity.route:
        return 0.0

    if entity.position is None:
        entity.position = dict(entity.route[0])
        entity.route_index = 1 if len(entity.route) > 1 else 0
        return 0.0

    if entity.reached_route_end:
        return 0.0

    remaining_step = entity.speed_mps * dt_s
    last_heading = 0.0

    while remaining_step > 0.0 and entity.route:
        if entity.route_index >= len(entity.route):
            if entity.loop:
                entity.route_index = 0
            else:
                entity.reached_route_end = True
                break

        target = entity.route[entity.route_index]
        dist = distance_m(entity.position, target)
        if dist < 0.05:
            entity.route_index += 1
            continue

        last_heading = heading_deg(entity.position, target)
        step = min(remaining_step, dist)
        ratio = step / dist
        entity.position["x"] += (target["x"] - entity.position["x"]) * ratio
        entity.position["y"] += (target["y"] - entity.position["y"]) * ratio
        remaining_step -= step

        if step >= dist - 0.05:
            entity.route_index += 1

    return last_heading


def send_heartbeat(conn: mavutil.mavfile, entity: Entity) -> None:
    conn.mav.srcSystem = entity.sysid
    conn.mav.srcComponent = 1
    conn.mav.heartbeat_send(
        entity.mav_type,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
        4,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def send_position(
    conn: mavutil.mavfile,
    entity: Entity,
    origin: Origin,
    heading: float,
    boot_ms: int,
) -> None:
    if entity.position is None:
        return

    lat, lon = local_to_latlon(entity.position, origin)
    rel_alt_mm = int(entity.altitude_m * 1000.0)
    conn.mav.srcSystem = entity.sysid
    conn.mav.srcComponent = 1
    conn.mav.global_position_int_send(
        boot_ms,
        int(lat * 1e7),
        int(lon * 1e7),
        rel_alt_mm,
        rel_alt_mm,
        0,
        0,
        0,
        int(heading * 100.0),
    )
    conn.mav.gps_raw_int_send(
        int(time.time() * 1_000_000),
        3,
        int(lat * 1e7),
        int(lon * 1e7),
        rel_alt_mm,
        80,
        80,
        int(entity.speed_mps * 100.0),
        int(heading * 100.0),
        12,
    )
    conn.mav.sys_status_send(
        0,
        0,
        0,
        500,
        12000,
        -1,
        85,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def send_param_value(
    conn: mavutil.mavfile,
    entity: Entity,
    param_index: int,
) -> None:
    name, value, param_type = MINIMAL_PARAMS[param_index]
    if name == "SYSID_THISMAV":
        value = float(entity.sysid)
    conn.mav.srcSystem = entity.sysid
    conn.mav.srcComponent = 1
    conn.mav.param_value_send(
        name.encode("ascii"),
        float(value),
        param_type,
        len(MINIMAL_PARAMS),
        param_index,
    )


def send_all_params(conn: mavutil.mavfile, entity: Entity) -> None:
    for index in range(len(MINIMAL_PARAMS)):
        send_param_value(conn, entity, index)
        time.sleep(0.01)


def send_autopilot_version(conn: mavutil.mavfile, entity: Entity) -> None:
    conn.mav.srcSystem = entity.sysid
    conn.mav.srcComponent = 1
    conn.mav.autopilot_version_send(
        0,
        0,
        0,
        0,
        0,
        b"\x00" * 8,
        b"\x00" * 8,
        b"\x00" * 8,
        0,
        0,
        0,
        0,
        0,
    )


def handle_qgc_requests(conn: mavutil.mavfile, entities: list[Entity]) -> None:
    by_sysid = {entity.sysid: entity for entity in entities}
    while True:
        try:
            msg = conn.recv_match(blocking=False)
        except ConnectionResetError:
            return
        if msg is None:
            return

        msg_type = msg.get_type()
        target_system = int(getattr(msg, "target_system", 0) or 0)
        targets = entities if target_system == 0 else [by_sysid[target_system]] if target_system in by_sysid else []

        if msg_type == "PARAM_REQUEST_LIST":
            for entity in targets:
                send_all_params(conn, entity)
        elif msg_type == "PARAM_REQUEST_READ":
            for entity in targets:
                index = int(getattr(msg, "param_index", -1))
                if 0 <= index < len(MINIMAL_PARAMS):
                    send_param_value(conn, entity, index)
                    continue
                raw_id = getattr(msg, "param_id", b"")
                if isinstance(raw_id, bytes):
                    param_id = raw_id.decode("ascii", errors="ignore").strip("\x00")
                else:
                    param_id = str(raw_id).strip("\x00")
                for param_index, (name, _, _) in enumerate(MINIMAL_PARAMS):
                    if name == param_id:
                        send_param_value(conn, entity, param_index)
                        break
        elif msg_type == "COMMAND_LONG":
            command = int(getattr(msg, "command", 0) or 0)
            if command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
                requested = int(getattr(msg, "param1", 0) or 0)
                if requested == mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                    for entity in targets:
                        send_autopilot_version(conn, entity)
                        conn.mav.srcSystem = entity.sysid
                        conn.mav.srcComponent = 1
                        conn.mav.command_ack_send(
                            command,
                            mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        )


def build_drone_entities(scenario: dict[str, Any], max_scouts: int, scout_speed_mps: float) -> list[Entity]:
    entities: list[Entity] = []
    routes = scenario.get("scout_boundary_routes") or []

    for index, route in enumerate(routes[:max_scouts]):
        points = [dict(p) for p in route.get("points", [])]
        if len(points) < 2:
            continue
        entities.append(
            Entity(
                sysid=DRONE_SYSID_BASE + index,
                mav_type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
                route=points,
                speed_mps=scout_speed_mps,
                altitude_m=float(route.get("altitude_m", 28.0)),
                loop=True,
            )
        )

    return entities


def point_in_polygon(point: dict[str, float], polygon: list[dict[str, float]]) -> bool:
    inside = False
    x = point["x"]
    y = point["y"]
    count = len(polygon)
    j = count - 1
    for i in range(count):
        pi = polygon[i]
        pj = polygon[j]
        intersects = ((pi["y"] > y) != (pj["y"] > y)) and (
            x < (pj["x"] - pi["x"]) * (y - pi["y"]) / ((pj["y"] - pi["y"]) or 1e-9) + pi["x"]
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def horizontal_intersections(y: float, polygon: list[dict[str, float]]) -> list[float]:
    xs: list[float] = []
    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]
        if abs(a["y"] - b["y"]) < 1e-9:
            continue
        low = min(a["y"], b["y"])
        high = max(a["y"], b["y"])
        if y < low or y >= high:
            continue
        ratio = (y - a["y"]) / (b["y"] - a["y"])
        xs.append(a["x"] + (b["x"] - a["x"]) * ratio)
    xs.sort()
    return xs


def nearest_depot_index(point: dict[str, float], depots: list[dict[str, float]]) -> int:
    if not depots:
        return 0
    return min(range(len(depots)), key=lambda i: distance_m(point, depots[i]))


def build_work_strips(
    scenario: dict[str, Any],
    swath_m: float,
    depots: list[dict[str, float]],
) -> list[WorkStrip]:
    strips: list[WorkStrip] = []
    for block in scenario.get("field_blocks", []):
        polygon = [dict(p) for p in block.get("boundary_points", [])]
        if len(polygon) < 3:
            continue
        min_y = min(p["y"] for p in polygon)
        max_y = max(p["y"] for p in polygon)
        y = min_y + swath_m * 0.5
        reverse = False
        while y <= max_y - swath_m * 0.25:
            xs = horizontal_intersections(y, polygon)
            for pair in range(0, len(xs) - 1, 2):
                start_x = xs[pair]
                end_x = xs[pair + 1]
                if end_x - start_x < swath_m * 0.8:
                    continue
                start = {"x": start_x, "y": y}
                end = {"x": end_x, "y": y}
                center = {"x": (start_x + end_x) * 0.5, "y": y}
                if not point_in_polygon(center, polygon):
                    continue
                if reverse:
                    start, end = end, start
                strips.append(
                    WorkStrip(
                        block_id=int(block.get("id", 0)),
                        start=start,
                        end=end,
                        center=center,
                        depot_index=nearest_depot_index(center, depots),
                        length_m=distance_m(start, end),
                        area_ha=distance_m(start, end) * swath_m / 10_000.0,
                    )
                )
                reverse = not reverse
            y += swath_m

    strips.sort(key=lambda s: (s.depot_index, distance_m(s.center, depots[s.depot_index]) if depots else 0.0))
    return strips


def extract_depot_points(scenario: dict[str, Any]) -> list[dict[str, float]]:
    depots: list[dict[str, float]] = []
    for depot in scenario.get("depot_sites") or []:
        point = depot.get("point") or depot
        if "x" in point and "y" in point:
            depots.append({"x": float(point["x"]), "y": float(point["y"])})
    if depots:
        return depots
    for block in scenario.get("field_blocks") or []:
        center = block.get("center")
        if center:
            depots.append({"x": float(center["x"]), "y": float(center["y"])})
    return depots


def build_work_entities(
    scenario: dict[str, Any],
    drone_count: int,
    work_speed_mps: float,
    transit_speed_mps: float,
    swath_m: float,
    altitude_m: float,
    fixed_wing_enabled: bool = False,
    fixed_wing_count: int = 1,
    fixed_wing_speed_mps: float = 42.0,
    fixed_wing_swath_m: float = 36.0,
    fixed_wing_min_strip_m: float = 260.0,
    fixed_wing_min_area_ha: float = 0.7,
    airport: dict[str, float] | None = None,
) -> tuple[list[Entity], list[dict[str, float]]]:
    depots = extract_depot_points(scenario)
    if not depots:
        raise SystemExit("scenario has no depot_sites or block centers for work replay")

    strips = build_work_strips(scenario, swath_m=swath_m, depots=depots)
    if not strips:
        raise SystemExit("no work strips generated from field polygons")

    fixed_wing_strips: list[WorkStrip] = []
    drone_strips: list[WorkStrip] = []
    if fixed_wing_enabled:
        for strip in strips:
            suitable = (
                strip.length_m >= fixed_wing_min_strip_m
                and strip.area_ha >= fixed_wing_min_area_ha
            )
            if suitable:
                fixed_wing_strips.append(strip)
            else:
                drone_strips.append(strip)
    else:
        drone_strips = strips

    drone_count = max(1, min(8, drone_count))
    routes: list[list[dict[str, float]]] = [[dict(depots[0])] for _ in range(drone_count)]
    route_costs = [0.0 for _ in range(drone_count)]
    current_positions = [dict(depots[0]) for _ in range(drone_count)]

    for strip in drone_strips:
        depot = depots[strip.depot_index]
        best_drone = 0
        best_score = float("inf")
        best_start = strip.start
        best_end = strip.end

        for drone_id in range(drone_count):
            candidates = [(strip.start, strip.end), (strip.end, strip.start)]
            for start, end in candidates:
                transit = distance_m(current_positions[drone_id], depot) + distance_m(depot, start)
                work = distance_m(start, end)
                score = route_costs[drone_id] + transit / max(0.1, transit_speed_mps) + work / max(0.1, work_speed_mps)
                if score < best_score:
                    best_score = score
                    best_drone = drone_id
                    best_start = start
                    best_end = end

        route = routes[best_drone]
        if distance_m(route[-1], depot) > 1.0:
            route.append(dict(depot))
        route.append(dict(best_start))
        route.append(dict(best_end))
        current_positions[best_drone] = dict(best_end)
        route_costs[best_drone] = best_score

    for drone_id, route in enumerate(routes):
        if distance_m(route[-1], depots[-1]) > 1.0:
            route.append(dict(depots[-1]))

    entities = [
        Entity(
            sysid=DRONE_SYSID_BASE + i,
            mav_type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
            route=routes[i],
            speed_mps=transit_speed_mps,
            altitude_m=altitude_m,
            loop=False,
        )
        for i in range(drone_count)
        if len(routes[i]) > 1
    ]

    if fixed_wing_enabled and fixed_wing_strips:
        fw_count = max(1, min(4, fixed_wing_count))
        if airport is None:
            min_x = min(min(s.start["x"], s.end["x"]) for s in strips)
            min_y = min(min(s.start["y"], s.end["y"]) for s in strips)
            airport = {"x": min_x - 1200.0, "y": min_y - 900.0}

        fw_routes: list[list[dict[str, float]]] = [[dict(airport)] for _ in range(fw_count)]
        fw_costs = [0.0 for _ in range(fw_count)]
        fw_positions = [dict(airport) for _ in range(fw_count)]

        fixed_wing_strips.sort(key=lambda s: (-s.length_m, s.block_id, s.center["y"]))
        for strip in fixed_wing_strips:
            best_plane = 0
            best_score = float("inf")
            best_start = strip.start
            best_end = strip.end
            for plane_id in range(fw_count):
                for start, end in ((strip.start, strip.end), (strip.end, strip.start)):
                    ferry = distance_m(fw_positions[plane_id], start)
                    work = distance_m(start, end)
                    score = fw_costs[plane_id] + ferry / fixed_wing_speed_mps + work / fixed_wing_speed_mps
                    if score < best_score:
                        best_score = score
                        best_plane = plane_id
                        best_start = start
                        best_end = end
            fw_routes[best_plane].append(dict(best_start))
            fw_routes[best_plane].append(dict(best_end))
            fw_positions[best_plane] = dict(best_end)
            fw_costs[best_plane] = best_score

        for plane_id, route in enumerate(fw_routes):
            if len(route) <= 1:
                continue
            route.append(dict(airport))
            entities.append(
                Entity(
                    sysid=FIXED_WING_SYSID_BASE + plane_id,
                    mav_type=mavutil.mavlink.MAV_TYPE_FIXED_WING,
                    route=route,
                    speed_mps=fixed_wing_speed_mps,
                    altitude_m=55.0,
                    loop=False,
                )
            )

        print(
            f"hybrid allocation: fixed_wing_strips={len(fixed_wing_strips)} "
            f"drone_strips={len(drone_strips)} fixed_wing_count={fw_count}"
        )

    return entities, depots


def build_entities_from_visual_plan(plan: dict[str, Any]) -> list[Entity]:
    hive = plan.get("hive") or {}
    drone_cfg = plan.get("drones") or {}
    fixed_cfg = plan.get("fixed_wing") or {}
    stops = [
        {"x": float(p["x"]), "y": float(p["y"])}
        for p in hive.get("stops", [])
        if "x" in p and "y" in p
    ]
    if not stops and "start" in hive:
        start = hive["start"]
        stops = [{"x": float(start["x"]), "y": float(start["y"])}]
    if not stops:
        stops = [{"x": 0.0, "y": 0.0}]

    drone_count = int(drone_cfg.get("count", 8))
    drone_count = max(1, min(8, drone_count))
    cruise_speed = float(drone_cfg.get("cruise_speed_mps", 12.0))
    spray_speed = float(drone_cfg.get("spray_speed_mps", 5.0))
    altitude = float(drone_cfg.get("altitude_m", 18.0))

    drone_routes: list[list[dict[str, float]]] = [[dict(stops[0])] for _ in range(drone_count)]
    drone_costs = [0.0 for _ in range(drone_count)]
    drone_positions = [dict(stops[0]) for _ in range(drone_count)]

    fixed_tasks: list[dict[str, Any]] = []
    drone_tasks: list[dict[str, Any]] = []
    for task in plan.get("tasks", []):
        if task.get("handling") == "fixed_wing":
            fixed_tasks.append(task)
        else:
            drone_tasks.append(task)

    for task in drone_tasks:
        route = task.get("route") or []
        if len(route) < 2:
            continue
        start = {"x": float(route[0]["x"]), "y": float(route[0]["y"])}
        end = {"x": float(route[1]["x"]), "y": float(route[1]["y"])}
        center = task.get("center") or start
        nearest_stop = min(stops, key=lambda p: distance_m(p, {"x": float(center["x"]), "y": float(center["y"])}))
        assigned = int(task.get("assigned_drone_id", -1)) - 1
        if assigned < 0 or assigned >= drone_count:
            assigned = min(range(drone_count), key=lambda i: drone_costs[i] + distance_m(drone_positions[i], nearest_stop))

        candidates = [(start, end), (end, start)]
        best_start, best_end = min(candidates, key=lambda pair: distance_m(drone_positions[assigned], pair[0]))
        if distance_m(drone_routes[assigned][-1], nearest_stop) > 1.0:
            drone_routes[assigned].append(dict(nearest_stop))
        drone_routes[assigned].append(dict(best_start))
        drone_routes[assigned].append(dict(best_end))
        drone_positions[assigned] = dict(best_end)
        drone_costs[assigned] += (
            distance_m(nearest_stop, best_start) / max(0.1, cruise_speed)
            + distance_m(best_start, best_end) / max(0.1, spray_speed)
        )

    for i in range(drone_count):
        if len(drone_routes[i]) > 1 and distance_m(drone_routes[i][-1], stops[-1]) > 1.0:
            drone_routes[i].append(dict(stops[-1]))

    entities: list[Entity] = [
        Entity(
            sysid=DRONE_SYSID_BASE + i,
            mav_type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
            route=drone_routes[i],
            speed_mps=cruise_speed,
            altitude_m=altitude,
            loop=False,
        )
        for i in range(drone_count)
        if len(drone_routes[i]) > 1
    ]

    if fixed_cfg.get("enabled") and int(fixed_cfg.get("count", 0)) > 0 and fixed_tasks:
        fw_count = max(1, min(4, int(fixed_cfg.get("count", 1))))
        fw_speed = float(fixed_cfg.get("speed_mps", 42.0))
        fw_alt = float(fixed_cfg.get("altitude_m", 55.0))
        airport_data = fixed_cfg.get("airport") or {"x": -1200.0, "y": -900.0}
        airport = {"x": float(airport_data["x"]), "y": float(airport_data["y"])}
        fw_routes: list[list[dict[str, float]]] = [[dict(airport)] for _ in range(fw_count)]
        fw_costs = [0.0 for _ in range(fw_count)]
        fw_positions = [dict(airport) for _ in range(fw_count)]

        for task in fixed_tasks:
            route = task.get("route") or []
            if len(route) < 2:
                continue
            start = {"x": float(route[0]["x"]), "y": float(route[0]["y"])}
            end = {"x": float(route[1]["x"]), "y": float(route[1]["y"])}
            best_plane = min(range(fw_count), key=lambda i: fw_costs[i] + distance_m(fw_positions[i], start) / fw_speed)
            if distance_m(fw_positions[best_plane], end) < distance_m(fw_positions[best_plane], start):
                start, end = end, start
            fw_routes[best_plane].append(dict(start))
            fw_routes[best_plane].append(dict(end))
            fw_positions[best_plane] = dict(end)
            fw_costs[best_plane] += distance_m(start, end) / fw_speed

        for i in range(fw_count):
            if len(fw_routes[i]) <= 1:
                continue
            fw_routes[i].append(dict(airport))
            entities.append(
                Entity(
                    sysid=FIXED_WING_SYSID_BASE + i,
                    mav_type=mavutil.mavlink.MAV_TYPE_FIXED_WING,
                    route=fw_routes[i],
                    speed_mps=fw_speed,
                    altitude_m=fw_alt,
                    loop=False,
                )
            )

    hive_speed = float(hive.get("speed_kmh", 30.0)) * MPS_PER_KMH
    entities.append(
        Entity(
            sysid=HIVE_SYSID,
            mav_type=mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
            route=stops,
            speed_mps=hive_speed,
            altitude_m=0.0,
            loop=False,
        )
    )

    print(
        f"OPT visual plan: drone_tasks={len(drone_tasks)} fixed_wing_tasks={len(fixed_tasks)} "
        f"hive_stops={len(stops)} hive_speed={hive_speed * 3.6:.1f}km/h"
    )
    return entities


def build_hive_entity(scenario: dict[str, Any], hive_speed_kmh: float) -> Entity | None:
    route = extract_depot_points(scenario)

    if not route:
        return None

    return Entity(
        sysid=HIVE_SYSID,
        mav_type=mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
        route=route,
        speed_mps=hive_speed_kmh * MPS_PER_KMH,
        altitude_m=0.0,
        loop=False,
    )


def run(args: argparse.Namespace) -> int:
    scenario = json.loads(Path(args.scenario).read_text(encoding="utf-8"))
    origin_data = scenario.get("origin")
    if not origin_data:
        raise SystemExit("scenario is missing origin; regenerate with qgc_plan_to_scenario.py")

    origin = Origin(lat=float(origin_data["lat"]), lon=float(origin_data["lon"]))
    display_mode = args.mode
    if scenario.get("visual_plan_version"):
        display_mode = "opt_visual_plan"
        entities = build_entities_from_visual_plan(scenario)
    elif args.mode in ("work", "hybrid"):
        entities, depots = build_work_entities(
            scenario,
            drone_count=args.drone_count,
            work_speed_mps=args.work_speed_mps,
            transit_speed_mps=args.transit_speed_mps,
            swath_m=args.swath_m,
            altitude_m=args.work_altitude_m,
            fixed_wing_enabled=args.mode == "hybrid",
            fixed_wing_count=args.fixed_wing_count,
            fixed_wing_speed_mps=args.fixed_wing_speed_mps,
            fixed_wing_swath_m=args.fixed_wing_swath_m,
            fixed_wing_min_strip_m=args.fixed_wing_min_strip_m,
            fixed_wing_min_area_ha=args.fixed_wing_min_area_ha,
        )
        if args.show_hive:
            entities.append(
                Entity(
                    sysid=HIVE_SYSID,
                    mav_type=mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                    route=depots,
                    speed_mps=args.hive_speed_kmh * MPS_PER_KMH,
                    altitude_m=0.0,
                    loop=False,
                )
            )
    else:
        entities = build_drone_entities(scenario, args.max_scouts, args.scout_speed_mps)
        hive = build_hive_entity(scenario, args.hive_speed_kmh)
        if hive is not None and args.show_hive:
            entities.append(hive)

    if not entities:
        raise SystemExit("no drawable entities found in scenario")

    conn = mavutil.mavlink_connection(args.out, source_system=255)
    tick_s = 1.0 / max(1.0, args.rate_hz)
    start = time.time()
    last = start
    heartbeat_timer = 0.0

    print(
        f"QGC demo bridge mode={display_mode} streaming {len(entities)} entities to {args.out}; "
        f"hive_speed={args.hive_speed_kmh:.1f} km/h"
    )

    while True:
        now = time.time()
        dt_s = max(0.001, now - last)
        last = now
        boot_ms = int((now - start) * 1000.0)
        heartbeat_timer += dt_s

        for entity in entities:
            heading = advance_entity(entity, dt_s)
            if heartbeat_timer >= 1.0:
                send_heartbeat(conn, entity)
            send_position(conn, entity, origin, heading, boot_ms)

        handle_qgc_requests(conn, entities)

        if heartbeat_timer >= 1.0:
            heartbeat_timer = 0.0

        if args.duration_s > 0 and now - start >= args.duration_s:
            break

        time.sleep(tick_s)

    print("QGC demo bridge stopped")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Stream scout_opt scenario motion into QGroundControl.")
    parser.add_argument("scenario", help="scenario JSON generated by qgc_plan_to_scenario.py")
    parser.add_argument("--mode", choices=("scout", "work", "hybrid"), default="scout")
    parser.add_argument("--out", default="udpout:127.0.0.1:14550", help="MAVLink output, default QGC UDP 14550")
    parser.add_argument("--max-scouts", type=int, default=8)
    parser.add_argument("--drone-count", type=int, default=8)
    parser.add_argument("--scout-speed-mps", type=float, default=6.0)
    parser.add_argument("--work-speed-mps", type=float, default=5.0)
    parser.add_argument("--transit-speed-mps", type=float, default=12.0)
    parser.add_argument("--work-altitude-m", type=float, default=18.0)
    parser.add_argument("--swath-m", type=float, default=28.0)
    parser.add_argument("--fixed-wing-count", type=int, default=1)
    parser.add_argument("--fixed-wing-speed-mps", type=float, default=42.0)
    parser.add_argument("--fixed-wing-swath-m", type=float, default=36.0)
    parser.add_argument("--fixed-wing-min-strip-m", type=float, default=260.0)
    parser.add_argument("--fixed-wing-min-area-ha", type=float, default=0.7)
    parser.add_argument("--hive-speed-kmh", type=float, default=30.0)
    parser.add_argument("--hide-hive", dest="show_hive", action="store_false")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 means run until Ctrl+C")
    parser.set_defaults(show_hive=True)
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Route real ArduPilot SITL vehicles to QGroundControl.

This script does not impersonate a flight controller. It connects to real
ArduPilot SITL TCP masters and forwards MAVLink packets between those vehicles
and QGroundControl UDP.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import time
from dataclasses import dataclass
from pathlib import Path

from pymavlink import mavutil


@dataclass
class VehicleLink:
    sysid: int
    port: int
    role: str
    conn: mavutil.mavfile
    last_seen: float = 0.0
    position: tuple[float, float, float] | None = None
    route: list[tuple[float, float, float]] | None = None
    route_index: int = 0
    last_logged_route_index: int = -1
    arm_accepted: bool = False
    takeoff_accepted: bool = False
    last_arm_attempt: float = 0.0
    last_takeoff_attempt: float = 0.0
    last_target_sent: float = 0.0
    last_stream_request: float = 0.0
    last_diag_time: float = 0.0
    last_sample_time: float = 0.0
    last_sample_position: tuple[float, float, float] | None = None
    distance_travelled_m: float = 0.0
    speed_mps: float = 0.0
    target: tuple[float, float, float] | None = None
    mode_name: str = "unknown"
    armed: bool = False


def local_to_latlon(point: dict[str, float], origin: dict[str, float]) -> tuple[float, float]:
    lat0 = float(origin["lat"])
    lon0 = float(origin["lon"])
    lat = lat0 + float(point["y"]) / 111_111.0
    lon = lon0 + float(point["x"]) / (111_111.0 * math.cos(math.radians(lat0)))
    return lat, lon


def distance_m(a_lat: float, a_lon: float, b_lat: float, b_lon: float) -> float:
    north = (b_lat - a_lat) * 111_111.0
    east = (b_lon - a_lon) * 111_111.0 * math.cos(math.radians(a_lat))
    return math.hypot(north, east)


def local_distance(a: dict[str, float], b: dict[str, float]) -> float:
    return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))


def local_orient(a: dict[str, float], b: dict[str, float], c: dict[str, float]) -> float:
    return (float(b["x"]) - float(a["x"])) * (float(c["y"]) - float(a["y"])) - (
        float(b["y"]) - float(a["y"])
    ) * (float(c["x"]) - float(a["x"]))


def local_segments_intersect(
    a: dict[str, float], b: dict[str, float], c: dict[str, float], d: dict[str, float]
) -> bool:
    o1, o2, o3, o4 = local_orient(a, b, c), local_orient(a, b, d), local_orient(c, d, a), local_orient(c, d, b)
    return ((o1 > 0 > o2) or (o1 < 0 < o2)) and ((o3 > 0 > o4) or (o3 < 0 < o4))


def local_same_point(a: dict[str, float], b: dict[str, float], eps: float = 1e-6) -> bool:
    return abs(float(a["x"]) - float(b["x"])) <= eps and abs(float(a["y"]) - float(b["y"])) <= eps


def local_point_in_polygon(point: dict[str, float], polygon: list[dict[str, float]]) -> bool:
    inside = False
    j = len(polygon) - 1
    for i, a in enumerate(polygon):
        b = polygon[j]
        dy = float(b["y"]) - float(a["y"])
        if (
            abs(dy) > 1e-9
            and ((float(a["y"]) > float(point["y"])) != (float(b["y"]) > float(point["y"])))
            and float(point["x"])
            < (float(b["x"]) - float(a["x"])) * (float(point["y"]) - float(a["y"])) / dy + float(a["x"])
        ):
            inside = not inside
        j = i
    return inside


def local_boundary_path(poly: list[dict[str, float]], i: int, j: int) -> tuple[float, list[dict[str, float]]]:
    n = len(poly)
    cw = [i]
    while cw[-1] != j:
        cw.append((cw[-1] + 1) % n)
    ccw = [i]
    while ccw[-1] != j:
        ccw.append((ccw[-1] - 1 + n) % n)

    def length(path: list[int]) -> float:
        return sum(local_distance(poly[path[k]], poly[path[k + 1]]) for k in range(len(path) - 1))

    cw_len = length(cw)
    ccw_len = length(ccw)
    return (cw_len, [poly[k] for k in cw]) if cw_len <= ccw_len else (ccw_len, [poly[k] for k in ccw])


def local_hive_leg_route(
    a: dict[str, float], b: dict[str, float], blocks: list[dict[str, object]]
) -> list[dict[str, float]]:
    for block in blocks:
        poly = [dict(p) for p in (block.get("boundary") or [])]  # type: ignore[union-attr]
        if len(poly) >= 2 and poly[0] == poly[-1]:
            poly = poly[:-1]
        if len(poly) < 3:
            continue
        mid = {"x": (float(a["x"]) + float(b["x"])) / 2.0, "y": (float(a["y"]) + float(b["y"])) / 2.0}
        def crosses_poly(x: dict[str, float], y: dict[str, float]) -> bool:
            m = {"x": (float(x["x"]) + float(y["x"])) / 2.0, "y": (float(x["y"]) + float(y["y"])) / 2.0}
            return local_point_in_polygon(m, poly) or sum(
                local_segments_intersect(x, y, poly[k], poly[(k + 1) % len(poly)]) for k in range(len(poly))
            ) >= 2

        crosses = local_point_in_polygon(mid, poly) or sum(
            local_segments_intersect(a, b, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))
        ) >= 2
        if not crosses:
            continue
        best_len = float("inf")
        best_route: list[dict[str, float]] = [a, b]
        for i in range(len(poly)):
            for j in range(len(poly)):
                if crosses_poly(a, poly[i]) or crosses_poly(poly[j], b):
                    continue
                boundary_len, boundary_route = local_boundary_path(poly, i, j)
                candidate_len = local_distance(a, poly[i]) + boundary_len + local_distance(poly[j], b)
                if candidate_len < best_len:
                    best_len = candidate_len
                    best_route = [a] + boundary_route + [b]
        return best_route
    return [a, b]


def local_hive_route(stops: list[dict[str, float]], blocks: list[dict[str, object]]) -> list[dict[str, float]]:
    if len(stops) < 2:
        return stops
    route = [stops[0]]
    for i in range(len(stops) - 1):
        route.extend(local_hive_visibility_route(stops[i], stops[i + 1], blocks)[1:])
    return route


def local_clean_polygons(blocks: list[dict[str, object]]) -> list[list[dict[str, float]]]:
    polygons: list[list[dict[str, float]]] = []
    for block in blocks:
        poly = [dict(p) for p in (block.get("boundary") or [])]  # type: ignore[union-attr]
        if len(poly) >= 2 and local_same_point(poly[0], poly[-1]):
            poly = poly[:-1]
        if len(poly) >= 3:
            polygons.append(poly)
    return polygons


def local_segment_blocked(a: dict[str, float], b: dict[str, float], polygons: list[list[dict[str, float]]]) -> bool:
    mid = {"x": (float(a["x"]) + float(b["x"])) * 0.5, "y": (float(a["y"]) + float(b["y"])) * 0.5}
    for poly in polygons:
        if local_point_in_polygon(mid, poly):
            return True
        for i in range(len(poly)):
            c, d = poly[i], poly[(i + 1) % len(poly)]
            if local_same_point(a, c) or local_same_point(a, d) or local_same_point(b, c) or local_same_point(b, d):
                continue
            if local_segments_intersect(a, b, c, d):
                return True
    return False


def local_hive_visibility_route(
    a: dict[str, float], b: dict[str, float], blocks: list[dict[str, object]]
) -> list[dict[str, float]]:
    polygons = local_clean_polygons(blocks)
    if not local_segment_blocked(a, b, polygons):
        return [a, b]

    nodes: list[dict[str, float]] = [a, b]
    for poly in polygons:
        nodes.extend(poly)
    n = len(nodes)
    graph: list[list[tuple[int, float]]] = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if local_segment_blocked(nodes[i], nodes[j], polygons):
                continue
            w = local_distance(nodes[i], nodes[j])
            graph[i].append((j, w))
            graph[j].append((i, w))

    costs = [float("inf")] * n
    prev = [-1] * n
    used = [False] * n
    costs[0] = 0.0
    for _ in range(n):
        u = min((idx for idx in range(n) if not used[idx]), key=lambda idx: costs[idx], default=-1)
        if u < 0 or costs[u] == float("inf"):
            break
        if u == 1:
            break
        used[u] = True
        for v, w in graph[u]:
            if costs[u] + w < costs[v]:
                costs[v] = costs[u] + w
                prev[v] = u
    if prev[1] < 0:
        return [a, b]
    path_idx: list[int] = []
    cur = 1
    while cur >= 0:
        path_idx.append(cur)
        cur = prev[cur]
    path_idx.reverse()
    return [nodes[idx] for idx in path_idx]


def local_line_polygon_intervals(
    poly: list[dict[str, float]], angle: float, cross: float
) -> list[tuple[float, float]]:
    ux, uy = math.cos(angle), math.sin(angle)
    vx, vy = -uy, ux
    hits: list[float] = []
    for i, a in enumerate(poly):
        b = poly[(i + 1) % len(poly)]
        ax, ay = float(a["x"]), float(a["y"])
        bx, by = float(b["x"]), float(b["y"])
        ca = ax * vx + ay * vy
        cb = bx * vx + by * vy
        denom = cb - ca
        if abs(denom) < 1e-9:
            continue
        s = (cross - ca) / denom
        if -1e-6 <= s <= 1.0 + 1e-6:
            x = ax + (bx - ax) * s
            y = ay + (by - ay) * s
            t = x * ux + y * uy
            if all(abs(t - old) > 0.05 for old in hits):
                hits.append(t)
    hits.sort()
    return [(hits[i], hits[i + 1]) for i in range(0, len(hits) - 1, 2) if hits[i + 1] - hits[i] > 6.0]


def local_polygon_cross_range(polygons: list[list[dict[str, float]]], angle: float) -> tuple[float, float]:
    vx, vy = -math.sin(angle), math.cos(angle)
    values = [float(p["x"]) * vx + float(p["y"]) * vy for poly in polygons for p in poly]
    return min(values), max(values)


def merged_fixed_wing_corridors(plan: dict) -> list[dict[str, object]]:
    mission = plan.get("fixed_wing_trajectory") or []
    if len(mission) >= 2:
        return [{
            "area_ha": sum(float(item.get("area_ha", 0.0) or 0.0) for item in plan.get("fixed_wing_routes", [])),
            "length_m": sum(local_distance(mission[i], mission[i + 1]) for i in range(len(mission) - 1)),
            "route": mission,
        }]

    fixed = plan.get("fixed_wing") or {}
    swath = float(fixed.get("swath_m", 22.0) or 22.0)
    target_area = float(fixed.get("assigned_area_ha", 0.0) or 0.0)
    if target_area <= 0.0:
        target_area = sum(float(item.get("area_ha", 0.0) or 0.0) for item in plan.get("fixed_wing_routes", []))

    polygons = local_clean_polygons(plan.get("work_area", {}).get("blocks", []))
    if not polygons:
        return []

    spacing = max(swath, 18.0)
    best_score = -1.0
    best_angle = 0.0
    best_rows: list[tuple[float, float, float]] = []
    best_areas: list[float] = []
    for deg in range(0, 180, 3):
        angle = math.radians(float(deg))
        min_cross, max_cross = local_polygon_cross_range(polygons, angle)
        rows: list[tuple[float, float, float, float, float]] = []
        cross = min_cross + spacing * 0.5
        while cross <= max_cross - spacing * 0.25:
            intervals: list[tuple[float, float]] = []
            for poly in polygons:
                intervals.extend(local_line_polygon_intervals(poly, angle, cross))
            if intervals:
                min_t = min(a for a, _b in intervals)
                max_t = max(b for _a, b in intervals)
                work_len = sum(b - a for a, b in intervals)
                corridor_len = max_t - min_t
                empty_len = max(0.0, corridor_len - work_len)
                area = work_len * swath / 10000.0
                net_score = work_len - empty_len * 0.72
                if area >= 0.18 and net_score > 40.0:
                    rows.append((net_score, corridor_len, min_t, max_t, cross, area))
            cross += spacing
        if not rows:
            continue
        rows.sort(key=lambda row: row[0], reverse=True)
        selected: list[tuple[float, float, float]] = []
        areas: list[float] = []
        covered = 0.0
        for _net_score, _corridor_len, min_t, max_t, cross, area in rows:
            selected.append((min_t, max_t, cross))
            areas.append(area)
            covered += area
            if target_area > 0.0 and covered >= target_area:
                break
        score = (
            sum(row[0] ** 1.12 for row in rows[: min(35, len(rows))])
            + rows[0][1] * 0.35
            - len(selected) * swath * 1.8
        )
        if score > best_score:
            best_score = score
            best_angle = angle
            best_rows = selected
            best_areas = areas

    ux, uy = math.cos(best_angle), math.sin(best_angle)
    vx, vy = -uy, ux
    corridors: list[dict[str, object]] = []
    for index, (min_t, max_t, cross) in enumerate(best_rows):
        corridors.append({
            "area_ha": best_areas[index] if index < len(best_areas) else 0.0,
            "length_m": max_t - min_t,
            "route": [
                {"x": ux * min_t + vx * cross, "y": uy * min_t + vy * cross},
                {"x": ux * max_t + vx * cross, "y": uy * max_t + vy * cross},
            ],
        })
    corridors.sort(key=lambda item: float(item.get("length_m", 0.0)), reverse=True)
    return corridors


def drop_leading_home_points(
    route: list[tuple[float, float, float]],
    home: tuple[float, float],
    min_distance_m: float = 20.0,
) -> list[tuple[float, float, float]]:
    """Avoid sending the first drone target back to the Hive launch point."""
    first_real = 0
    for i, point in enumerate(route):
        if distance_m(home[0], home[1], point[0], point[1]) >= min_distance_m:
            first_real = i
            break
    else:
        return route
    return route[first_real:]


def build_vehicle_routes(
    visual_plan_path: str,
    copter_count: int,
    plane_count: int,
    rover_count: int,
    altitude_m: float,
) -> dict[int, list[tuple[float, float, float]]]:
    plan = json.loads(Path(visual_plan_path).read_text(encoding="utf-8"))
    origin = plan.get("origin")
    if not origin:
        raise SystemExit("visual plan is missing origin")

    hive = plan.get("hive") or {}
    work_blocks = (plan.get("work_area") or {}).get("blocks") or []
    stops = [
        {"x": float(p["x"]), "y": float(p["y"])}
        for p in hive.get("stops", [])
        if "x" in p and "y" in p
    ]
    start = stops[0] if stops else {"x": 0.0, "y": 0.0}
    final_hive = stops[-1] if stops else start
    start_latlon = local_to_latlon(start, origin)
    final_hive_latlon = local_to_latlon(final_hive, origin)
    routes: dict[int, list[tuple[float, float, float]]] = {i + 1: [] for i in range(copter_count)}

    for scout in plan.get("scout_routes", []):
        if copter_count <= 0:
            break
        assigned = int(scout.get("drone_id", 1))
        if assigned < 1 or assigned > copter_count:
            assigned = (assigned - 1) % copter_count + 1
        scout_alt = float(scout.get("altitude_m", altitude_m + 10.0))
        for point in scout.get("route") or []:
            lat, lon = local_to_latlon(point, origin)
            routes[assigned].append((lat, lon, scout_alt))

    for task in plan.get("tasks", []):
        if task.get("handling") != "drone":
            continue
        if copter_count <= 0:
            continue
        raw_route = task.get("route") or []
        if len(raw_route) < 2:
            continue
        assigned = int(task.get("assigned_drone_id", -1))
        if assigned < 1 or assigned > copter_count:
            assigned = (int(task.get("id", 1)) - 1) % copter_count + 1
        for point in raw_route:
            lat, lon = local_to_latlon(point, origin)
            routes[assigned].append((lat, lon, altitude_m))

    fixed_wing = plan.get("fixed_wing") or {}
    if plane_count > 0 and fixed_wing.get("enabled"):
        plane_sysid = 100
        plane_alt = float(fixed_wing.get("altitude_m", 55.0))
        tank_area_ha = float(fixed_wing.get("tank_area_ha", 189.3))
        airport = fixed_wing.get("airport") or {"x": -1200.0, "y": -900.0}
        plane_route: list[tuple[float, float, float]] = []
        lat, lon = local_to_latlon(airport, origin)
        sortie_area = 0.0
        plane_route.append((lat, lon, plane_alt))
        for corridor in merged_fixed_wing_corridors(plan):
            area = float(corridor.get("area_ha", 0.0))
            if sortie_area > 0.0 and sortie_area + area > tank_area_ha:
                plane_route.append((lat, lon, plane_alt))
                sortie_area = 0.0
            for point in corridor.get("route") or []:
                plat, plon = local_to_latlon(point, origin)
                plane_route.append((plat, plon, plane_alt))
            sortie_area += area
        plane_route.append((lat, lon, plane_alt))
        routes[plane_sysid] = plane_route

    if rover_count > 0:
        rover_sysid = 200
        rover_route: list[tuple[float, float, float]] = []
        for stop in local_hive_route(stops, work_blocks):
            lat, lon = local_to_latlon(stop, origin)
            rover_route.append((lat, lon, 0.0))
        routes[rover_sysid] = rover_route

    for sysid, route_points in routes.items():
        if route_points:
            if 1 <= sysid <= copter_count:
                cleaned = drop_leading_home_points(route_points, start_latlon)
                route_points[:] = cleaned
                route_points.append((final_hive_latlon[0], final_hive_latlon[1], altitude_m))
            print(f"loaded OPT route sysid={sysid} waypoints={len(route_points)}", flush=True)
    return routes


def export_work_area_kml(visual_plan_path: str, output_path: str) -> None:
    plan = json.loads(Path(visual_plan_path).read_text(encoding="utf-8"))
    origin = plan.get("origin")
    if not origin:
        return
    work_area = plan.get("work_area") or {}
    blocks = work_area.get("blocks") or []
    if not blocks:
        return

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<kml xmlns="http://www.opengis.net/kml/2.2">',
        "  <Document>",
        "    <name>Scout OPT Work Area</name>",
        "    <Style id=\"fieldBoundary\"><LineStyle><color>ff00ffff</color><width>4</width></LineStyle><PolyStyle><color>3300ffff</color></PolyStyle></Style>",
    ]
    for block in blocks:
        points = block.get("boundary") or []
        if len(points) < 3:
            continue
        coords: list[str] = []
        for point in points:
            lat, lon = local_to_latlon(point, origin)
            coords.append(f"{lon:.8f},{lat:.8f},0")
        if coords[0] != coords[-1]:
            coords.append(coords[0])
        name = block.get("name") or f"field block {block.get('block_id', '')}"
        lines.extend([
            "    <Placemark>",
            f"      <name>{name}</name>",
            "      <styleUrl>#fieldBoundary</styleUrl>",
            "      <Polygon><outerBoundaryIs><LinearRing><coordinates>",
            "        " + " ".join(coords),
            "      </coordinates></LinearRing></outerBoundaryIs></Polygon>",
            "    </Placemark>",
        ])
    lines.extend(["  </Document>", "</kml>", ""])
    out.write_text("\n".join(lines), encoding="utf-8")
    print(f"exported work area KML: {out}", flush=True)


def connect_vehicle(sysid: int, port: int, role: str, timeout_s: float) -> VehicleLink:
    conn = mavutil.mavlink_connection(
        f"tcp:127.0.0.1:{port}",
        source_system=250,
        autoreconnect=True,
        robust_parsing=True,
    )
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg is not None:
            real_sysid = msg.get_srcSystem()
            print(f"connected ArduPilot role={role} sysid={real_sysid} tcp:{port}", flush=True)
            return VehicleLink(sysid=real_sysid, port=port, role=role, conn=conn, last_seen=time.time())
    raise TimeoutError(f"no ArduPilot heartbeat on tcp:{port}")


def set_mode_named(vehicle: VehicleLink, mode_name: str) -> bool:
    modes = vehicle.conn.mode_mapping()
    mode_id = modes.get(mode_name)
    if mode_id is None:
        return False
    vehicle.conn.target_system = vehicle.sysid
    vehicle.conn.target_component = 1
    vehicle.conn.mav.command_long_send(
        vehicle.sysid,
        1,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.conn.mav.set_mode_send(
        vehicle.sysid,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    return True


def set_guided(vehicle: VehicleLink) -> None:
    set_mode_named(vehicle, "GUIDED")


def request_position_stream(vehicle: VehicleLink) -> None:
    now = time.time()
    if now - vehicle.last_stream_request < 10.0:
        return
    vehicle.last_stream_request = now
    for message_id, hz in (
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 2.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 2.0),
    ):
        vehicle.conn.mav.command_long_send(
            vehicle.sysid,
            1,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            1_000_000.0 / hz,
            0,
            0,
            0,
            0,
            0,
        )


def arm_and_takeoff(vehicle: VehicleLink, altitude_m: float) -> None:
    if vehicle.role == "rover":
        now = time.time()
        if not vehicle.arm_accepted and now - vehicle.last_arm_attempt >= 4.0:
            set_guided(vehicle)
            vehicle.conn.mav.command_long_send(
                vehicle.sysid,
                1,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                2989,
                0,
                0,
                0,
                0,
                0,
            )
            vehicle.last_arm_attempt = now
            print(f"sysid={vehicle.sysid} rover arm requested", flush=True)
        vehicle.takeoff_accepted = True
        return
    now = time.time()
    if not vehicle.arm_accepted and now - vehicle.last_arm_attempt >= 4.0:
        set_guided(vehicle)
        vehicle.conn.mav.command_long_send(
            vehicle.sysid,
            1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            2989,
            0,
            0,
            0,
            0,
            0,
        )
        vehicle.last_arm_attempt = now
        print(f"sysid={vehicle.sysid} arm requested", flush=True)
        return
    if vehicle.role == "plane":
        if vehicle.position is not None and vehicle.position[2] > min(15.0, altitude_m * 0.5):
            vehicle.takeoff_accepted = True
            set_guided(vehicle)
            return
        if now - vehicle.last_takeoff_attempt >= 4.0:
            if set_mode_named(vehicle, "TAKEOFF"):
                print(f"sysid={vehicle.sysid} plane TAKEOFF mode requested", flush=True)
            else:
                print(f"sysid={vehicle.sysid} plane TAKEOFF mode unavailable", flush=True)
            vehicle.conn.mav.command_long_send(
                vehicle.sysid,
                1,
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
            vehicle.last_takeoff_attempt = now
        return
    needs_takeoff = not vehicle.takeoff_accepted
    if vehicle.role != "rover" and vehicle.position is not None and vehicle.position[2] < min(3.0, altitude_m * 0.5):
        needs_takeoff = True
    if vehicle.arm_accepted and needs_takeoff and now - vehicle.last_takeoff_attempt >= 4.0:
        set_guided(vehicle)
        vehicle.conn.mav.command_long_send(
            vehicle.sysid,
            1,
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
        vehicle.last_takeoff_attempt = now
        print(f"sysid={vehicle.sysid} takeoff requested", flush=True)


def send_goto(vehicle: VehicleLink, lat: float, lon: float, alt_m: float) -> None:
    if vehicle.role == "plane":
        reposition_cmd = getattr(mavutil.mavlink, "MAV_CMD_DO_REPOSITION", 192)
        vehicle.conn.mav.command_int_send(
            vehicle.sysid,
            1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            reposition_cmd,
            0,
            0,
            -1,
            0,
            0,
            0,
            int(lat * 1e7),
            int(lon * 1e7),
            alt_m,
        )
        return
    vehicle.conn.mav.set_position_target_global_int_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        vehicle.sysid,
        1,
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


def update_opt_route(vehicle: VehicleLink, altitude_m: float) -> None:
    if not vehicle.route or vehicle.route_index >= len(vehicle.route):
        return
    vehicle_altitude_m = vehicle.route[0][2] if vehicle.role == "plane" and vehicle.route else altitude_m
    arm_and_takeoff(vehicle, vehicle_altitude_m)
    if not vehicle.takeoff_accepted:
        return
    if vehicle.role != "rover" and vehicle.position is not None:
        if vehicle.position[2] < min(3.0, altitude_m * 0.5):
            return
    now = time.time()
    target = vehicle.route[vehicle.route_index]
    vehicle.target = target
    if vehicle.position is not None:
        dist = distance_m(vehicle.position[0], vehicle.position[1], target[0], target[1])
        accept_radius_m = 90.0 if vehicle.role == "plane" else 10.0
        if dist < accept_radius_m:
            vehicle.route_index += 1
            if vehicle.route_index >= len(vehicle.route):
                print(f"sysid={vehicle.sysid} OPT route complete", flush=True)
                return
            target = vehicle.route[vehicle.route_index]
            vehicle.target = target
    target_period_s = 3.0 if vehicle.role == "plane" else 1.0
    if now - vehicle.last_target_sent >= target_period_s:
        set_guided(vehicle)
        send_goto(vehicle, target[0], target[1], target[2])
        vehicle.last_target_sent = now
        if vehicle.route_index != vehicle.last_logged_route_index:
            vehicle.last_logged_route_index = vehicle.route_index
            print(
                f"sysid={vehicle.sysid} goto wp={vehicle.route_index + 1}/{len(vehicle.route)} "
                f"lat={target[0]:.7f} lon={target[1]:.7f} alt={target[2]:.1f}",
                flush=True,
            )


def update_motion_stats(vehicle: VehicleLink, now: float) -> None:
    if vehicle.position is None:
        return
    if vehicle.last_sample_position is not None and vehicle.last_sample_time > 0.0:
        dt = max(0.001, now - vehicle.last_sample_time)
        step_m = distance_m(
            vehicle.last_sample_position[0],
            vehicle.last_sample_position[1],
            vehicle.position[0],
            vehicle.position[1],
        )
        if step_m < 1000.0:
            vehicle.distance_travelled_m += step_m
            vehicle.speed_mps = step_m / dt
    vehicle.last_sample_position = vehicle.position
    vehicle.last_sample_time = now


def target_distance(vehicle: VehicleLink) -> float | None:
    if vehicle.position is None or vehicle.target is None:
        return None
    return distance_m(vehicle.position[0], vehicle.position[1], vehicle.target[0], vehicle.target[1])


def write_path_sample(writer: csv.writer | None, vehicle: VehicleLink, now: float) -> None:
    if writer is None or vehicle.position is None:
        return
    target = vehicle.target
    dist_to_target = target_distance(vehicle)
    writer.writerow([
        f"{now:.3f}",
        vehicle.sysid,
        vehicle.role,
        vehicle.route_index + 1 if vehicle.route else 0,
        len(vehicle.route) if vehicle.route else 0,
        f"{vehicle.position[0]:.8f}",
        f"{vehicle.position[1]:.8f}",
        f"{vehicle.position[2]:.2f}",
        f"{vehicle.speed_mps:.2f}",
        f"{vehicle.distance_travelled_m:.2f}",
        f"{dist_to_target:.2f}" if dist_to_target is not None else "",
        f"{target[0]:.8f}" if target else "",
        f"{target[1]:.8f}" if target else "",
        f"{target[2]:.2f}" if target else "",
    ])


def print_diagnostics(vehicle: VehicleLink, now: float, interval_s: float) -> None:
    if interval_s <= 0.0 or now - vehicle.last_diag_time < interval_s:
        return
    vehicle.last_diag_time = now
    if vehicle.position is None:
        print(f"REALPATH sysid={vehicle.sysid} role={vehicle.role} no_position_yet", flush=True)
        return
    dist_to_target = target_distance(vehicle)
    dist_text = f"{dist_to_target:.1f}m" if dist_to_target is not None else "n/a"
    stuck = vehicle.route and vehicle.takeoff_accepted and vehicle.speed_mps < 0.3
    if not vehicle.arm_accepted:
        status = "WAIT_ARM"
    elif not vehicle.takeoff_accepted and vehicle.role != "rover":
        status = "WAIT_TAKEOFF"
    else:
        status = "STUCK?" if stuck else "moving"
    print(
        f"REALPATH sysid={vehicle.sysid} role={vehicle.role} wp="
        f"{vehicle.route_index + 1 if vehicle.route else 0}/{len(vehicle.route) if vehicle.route else 0} "
        f"mode={vehicle.mode_name} armed={int(vehicle.armed)} "
        f"lat={vehicle.position[0]:.7f} lon={vehicle.position[1]:.7f} alt={vehicle.position[2]:.1f} "
        f"speed={vehicle.speed_mps:.2f}mps travelled={vehicle.distance_travelled_m:.1f}m "
        f"target_dist={dist_text} status={status}",
        flush=True,
    )


def active_copters_away_from_hive(vehicles: list[VehicleLink]) -> int:
    home: tuple[float, float] | None = None
    for vehicle in vehicles:
        if vehicle.role == "rover" and vehicle.route:
            home = (vehicle.route[0][0], vehicle.route[0][1])
            break
    if home is None:
        for vehicle in vehicles:
            if vehicle.role == "copter" and vehicle.route:
                home = (vehicle.route[-1][0], vehicle.route[-1][1])
                break
    if home is None:
        return 0

    count = 0
    for vehicle in vehicles:
        if vehicle.role != "copter" or vehicle.position is None:
            continue
        if not vehicle.armed:
            continue
        if distance_m(home[0], home[1], vehicle.position[0], vehicle.position[1]) > 30.0:
            count += 1
    return count


def route(args: argparse.Namespace) -> int:
    vehicles: list[VehicleLink] = []
    if args.count is not None:
        args.copters = args.count
    for i in range(args.copters):
        vehicles.append(connect_vehicle(i + 1, args.base_port + i * 10, "copter", args.timeout))
    for i in range(args.planes):
        vehicles.append(connect_vehicle(100 + i, args.plane_base_port + i * 10, "plane", args.timeout))
    for i in range(args.rovers):
        vehicles.append(connect_vehicle(200 + i, args.rover_base_port + i * 10, "rover", args.timeout))

    if args.visual_plan:
        routes = build_vehicle_routes(args.visual_plan, args.copters, args.planes, args.rovers, args.takeoff_alt)
        if args.work_area_kml:
            export_work_area_kml(args.visual_plan, args.work_area_kml)
        for vehicle in vehicles:
            vehicle.route = routes.get(vehicle.sysid)

    qgc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    qgc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    qgc_sock.bind(("0.0.0.0", args.listen_port))
    qgc_sock.setblocking(False)
    qgc_addr = (args.qgc_host, args.qgc_port)
    target_by_sysid = {v.sysid: v for v in vehicles}
    path_file = None
    path_writer = None
    if args.path_log:
        path_path = Path(args.path_log)
        path_path.parent.mkdir(parents=True, exist_ok=True)
        path_file = path_path.open("w", newline="", encoding="utf-8")
        path_writer = csv.writer(path_file)
        path_writer.writerow([
            "time_s",
            "sysid",
            "role",
            "wp_index",
            "wp_count",
            "lat",
            "lon",
            "rel_alt_m",
            "speed_mps",
            "travelled_m",
            "target_dist_m",
            "target_lat",
            "target_lon",
            "target_alt_m",
        ])

    print(
        f"router active: {len(vehicles)} real ArduPilot vehicles -> "
        f"QGC {args.qgc_host}:{args.qgc_port}, listening for QGC on UDP {args.listen_port}",
        flush=True,
    )

    hive_work_started = args.copters <= 0
    while True:
        copters_away = active_copters_away_from_hive(vehicles)
        if copters_away > 2:
            hive_work_started = True
        for vehicle in vehicles:
            request_position_stream(vehicle)
            while True:
                msg = vehicle.conn.recv_msg()
                if msg is None:
                    break
                vehicle.last_seen = time.time()
                if msg.get_type() == "HEARTBEAT":
                    vehicle.mode_name = mavutil.mode_string_v10(msg)
                    vehicle.armed = bool(
                        int(getattr(msg, "base_mode", 0) or 0)
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                elif msg.get_type() == "GLOBAL_POSITION_INT":
                    vehicle.position = (
                        float(msg.lat) / 1e7,
                        float(msg.lon) / 1e7,
                        float(msg.relative_alt) / 1000.0,
                    )
                    now = time.time()
                    update_motion_stats(vehicle, now)
                    write_path_sample(path_writer, vehicle, now)
                    if path_file is not None:
                        path_file.flush()
                elif msg.get_type() == "COMMAND_ACK":
                    command_raw = getattr(msg, "command", 0)
                    result_raw = getattr(msg, "result", -1)
                    command = int(0 if command_raw is None else command_raw)
                    result = int(-1 if result_raw is None else result_raw)
                    if command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        vehicle.arm_accepted = result in (
                            mavutil.mavlink.MAV_RESULT_ACCEPTED,
                            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                        )
                    elif command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                        if result in (
                            mavutil.mavlink.MAV_RESULT_ACCEPTED,
                            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                        ):
                            vehicle.takeoff_accepted = True
                    print(
                        f"sysid={vehicle.sysid} ACK command={command} result={result}",
                        flush=True,
                    )
                elif msg.get_type() == "STATUSTEXT":
                    text = getattr(msg, "text", "")
                    if isinstance(text, bytes):
                        text = text.decode("utf-8", errors="ignore")
                    print(f"sysid={vehicle.sysid} STATUSTEXT {text}", flush=True)
                packet = msg.get_msgbuf()
                if packet:
                    qgc_sock.sendto(packet, qgc_addr)

            if args.visual_plan:
                if vehicle.role == "rover" and args.copters > 0:
                    if not hive_work_started or copters_away > 2:
                        print_diagnostics(vehicle, time.time(), args.diagnostics_interval)
                        continue
                update_opt_route(vehicle, args.takeoff_alt)
            print_diagnostics(vehicle, time.time(), args.diagnostics_interval)

        while True:
            try:
                data, _ = qgc_sock.recvfrom(4096)
            except BlockingIOError:
                break
            except ConnectionResetError:
                break
            if not data:
                break

            parser = mavutil.mavlink.MAVLink(None)
            target_system = 0
            for byte in data:
                msg = parser.parse_char(bytes([byte]))
                if msg is None:
                    continue
                target_system = int(getattr(msg, "target_system", 0) or 0)
                break

            if target_system in target_by_sysid:
                target_by_sysid[target_system].conn.write(data)
            else:
                for vehicle in vehicles:
                    vehicle.conn.write(data)

        time.sleep(0.005)


def main() -> int:
    parser = argparse.ArgumentParser(description="Real ArduPilot SITL TCP <-> QGroundControl UDP router.")
    parser.add_argument("--count", type=int, default=None, help="legacy alias for --copters")
    parser.add_argument("--copters", type=int, default=1)
    parser.add_argument("--planes", type=int, default=0)
    parser.add_argument("--rovers", type=int, default=0)
    parser.add_argument("--base-port", type=int, default=5760)
    parser.add_argument("--plane-base-port", type=int, default=5860)
    parser.add_argument("--rover-base-port", type=int, default=5960)
    parser.add_argument("--listen-port", type=int, default=14560)
    parser.add_argument("--qgc-host", default="127.0.0.1")
    parser.add_argument("--qgc-port", type=int, default=14550)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--visual-plan")
    parser.add_argument("--work-area-kml", default=".tmp/real_sitl/work_area.kml")
    parser.add_argument("--takeoff-alt", type=float, default=18.0)
    parser.add_argument("--path-log", default=".tmp/real_sitl/actual_paths.csv")
    parser.add_argument("--diagnostics-interval", type=float, default=5.0)
    return route(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())

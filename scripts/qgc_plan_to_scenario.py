#!/usr/bin/env python3
"""Convert QGroundControl fence/polygon plans into scout_opt scenario JSON.

The ideal workflow is:
  QGC Fence polygon -> Scout boundary route -> field_blocks -> boundary-road
  hive candidate points. The converter treats each polygon as an operator-
  selected task region. The polygon outside/exterior is considered accessible
  to the hive in this idealized model.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def is_coord_pair(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) >= 2
        and isinstance(value[0], (int, float))
        and isinstance(value[1], (int, float))
    )


def is_polygon(value: Any) -> bool:
    return isinstance(value, list) and len(value) >= 3 and all(is_coord_pair(p) for p in value)


def collect_polygons(value: Any, out: list[list[list[float]]]) -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            lowered = key.lower()
            if ("polygon" in lowered or "poly" == lowered or "fence" in lowered) and is_polygon(child):
                out.append([[float(p[0]), float(p[1])] for p in child])
            else:
                collect_polygons(child, out)
    elif isinstance(value, list):
        if is_polygon(value):
            return
        for child in value:
            collect_polygons(child, out)


def centroid_latlon(polygons: list[list[list[float]]]) -> tuple[float, float]:
    points = [p for poly in polygons for p in poly]
    if not points:
        raise ValueError("no polygon points found")
    return (
        sum(p[0] for p in points) / len(points),
        sum(p[1] for p in points) / len(points),
    )


def polygon_signature(polygon: list[list[float]]) -> tuple[tuple[float, float], ...]:
    rounded = [(round(p[0], 7), round(p[1], 7)) for p in polygon]
    rotations = [tuple(rounded[i:] + rounded[:i]) for i in range(len(rounded))]
    reversed_points = list(reversed(rounded))
    rotations.extend(tuple(reversed_points[i:] + reversed_points[:i]) for i in range(len(reversed_points)))
    return min(rotations)


def dedupe_polygons(polygons: list[list[list[float]]]) -> list[list[list[float]]]:
    seen: set[tuple[tuple[float, float], ...]] = set()
    unique: list[list[list[float]]] = []
    for polygon in polygons:
        signature = polygon_signature(polygon)
        if signature in seen:
            continue
        seen.add(signature)
        unique.append(polygon)
    return unique


def project(point: list[float], origin_lat: float, origin_lon: float) -> dict[str, float]:
    lat, lon = point[0], point[1]
    meters_per_deg_lat = 111_111.0
    meters_per_deg_lon = 111_111.0 * math.cos(math.radians(origin_lat))
    return {
        "x": (lon - origin_lon) * meters_per_deg_lon,
        "y": (lat - origin_lat) * meters_per_deg_lat,
    }


def polygon_area_and_center(points: list[dict[str, float]]) -> tuple[float, dict[str, float]]:
    twice_area = 0.0
    cx = 0.0
    cy = 0.0
    for idx, a in enumerate(points):
        b = points[(idx + 1) % len(points)]
        cross = a["x"] * b["y"] - b["x"] * a["y"]
        twice_area += cross
        cx += (a["x"] + b["x"]) * cross
        cy += (a["y"] + b["y"]) * cross
    if abs(twice_area) < 1e-9:
        raise ValueError("polygon area is zero")
    signed_area = twice_area / 2.0
    return abs(signed_area), {"x": cx / (6.0 * signed_area), "y": cy / (6.0 * signed_area)}


def outward_offset_points(points: list[dict[str, float]], offset_m: float) -> list[dict[str, float]]:
    signed_twice_area = 0.0
    for idx, a in enumerate(points):
        b = points[(idx + 1) % len(points)]
        signed_twice_area += a["x"] * b["y"] - b["x"] * a["y"]
    ccw = signed_twice_area > 0.0

    shifted: list[dict[str, float]] = []
    for idx, p in enumerate(points):
        prev_point = points[idx - 1]
        next_point = points[(idx + 1) % len(points)]
        edge_x = next_point["x"] - prev_point["x"]
        edge_y = next_point["y"] - prev_point["y"]
        length = math.hypot(edge_x, edge_y)
        if length < 1e-6:
            shifted.append(dict(p))
            continue
        if ccw:
            normal_x = edge_y / length
            normal_y = -edge_x / length
        else:
            normal_x = -edge_y / length
            normal_y = edge_x / length
        shifted.append({"x": p["x"] + normal_x * offset_m, "y": p["y"] + normal_y * offset_m})
    return shifted


def sample_polyline(points: list[dict[str, float]], spacing_m: float) -> list[dict[str, float]]:
    samples: list[dict[str, float]] = []
    for idx, a in enumerate(points):
        b = points[(idx + 1) % len(points)]
        dx = b["x"] - a["x"]
        dy = b["y"] - a["y"]
        length = math.hypot(dx, dy)
        steps = max(1, int(math.ceil(length / spacing_m)))
        for step in range(steps):
            t = step / steps
            samples.append({"x": a["x"] + dx * t, "y": a["y"] + dy * t})
    return samples


def dedupe_points(points: list[dict[str, float]], tolerance_m: float) -> list[dict[str, float]]:
    unique: list[dict[str, float]] = []
    for point in points:
        if all(math.hypot(point["x"] - other["x"], point["y"] - other["y"]) > tolerance_m for other in unique):
            unique.append(point)
    return unique


def point_in_polygon(point: dict[str, float], polygon: list[dict[str, float]]) -> bool:
    inside = False
    j = len(polygon) - 1
    for i, a in enumerate(polygon):
        b = polygon[j]
        dy = b["y"] - a["y"]
        if (
            abs(dy) > 1e-9
            and ((a["y"] > point["y"]) != (b["y"] > point["y"]))
            and point["x"] < (b["x"] - a["x"]) * (point["y"] - a["y"]) / dy + a["x"]
        ):
            inside = not inside
        j = i
    return inside


def build_depots(blocks: list[dict[str, Any]], spacing_m: float, outside_offset_m: float) -> list[dict[str, Any]]:
    candidates: list[dict[str, float]] = []
    for block in blocks:
        boundary = block.get("boundary_points") or []
        if len(boundary) >= 3:
            outside_boundary = outward_offset_points(boundary, outside_offset_m)
            candidates.extend(sample_polyline(outside_boundary, spacing_m))
        else:
            center = block["center"]
            candidates.extend(
                [
                    {"x": center["x"] - 260.0, "y": center["y"] - 280.0},
                    {"x": center["x"] + 260.0, "y": center["y"] + 280.0},
                ]
            )

    candidates = dedupe_points(candidates, spacing_m * 0.45)
    field_polygons = [block.get("boundary_points") or [] for block in blocks]
    candidates = [
        point for point in candidates
        if all(len(poly) < 3 or not point_in_polygon(point, poly) for poly in field_polygons)
    ]
    depots: list[dict[str, Any]] = []
    for point in candidates:
        depots.append(
            {
                "id": len(depots) + 1,
                "point": point,
                "usable_area_m2": 520.0,
                "road_accessible": True,
                "slope_risk": 0.08,
                "notes": ["ideal model: generated along task boundary road"],
            }
        )
    return depots


def build_scout_routes(blocks: list[dict[str, Any]], altitude_m: float) -> list[dict[str, Any]]:
    routes: list[dict[str, Any]] = []
    for block in blocks:
        boundary = block.get("boundary_points") or []
        if len(boundary) < 3:
            continue
        closed = boundary + [boundary[0]]
        routes.append(
            {
                "block_id": block["id"],
                "mode": "boundary_follow",
                "altitude_m": altitude_m,
                "speed_profile": "slow_safe_scout",
                "points": closed,
                "notes": [
                    "Scout should fly this closed boundary before OPT creates final small work tasks.",
                    "The confirmed fence boundary becomes the field block for automatic zone generation.",
                ],
            }
        )
    return routes


def convert(input_path: Path, output_path: Path, depot_spacing_m: float, outside_offset_m: float) -> None:
    plan = json.loads(input_path.read_text(encoding="utf-8"))
    raw_polygons: list[list[list[float]]] = []
    collect_polygons(plan, raw_polygons)
    raw_polygons = dedupe_polygons(raw_polygons)
    if not raw_polygons:
        raise SystemExit("No QGroundControl polygon found. Add a Survey/Pattern polygon and save the .plan file.")

    origin_lat, origin_lon = centroid_latlon(raw_polygons)
    blocks: list[dict[str, Any]] = []
    all_points: list[dict[str, float]] = []
    for index, polygon in enumerate(raw_polygons, start=1):
        projected = [project(p, origin_lat, origin_lon) for p in polygon]
        area_m2, center = polygon_area_and_center(projected)
        all_points.extend(projected)
        blocks.append(
            {
                "id": index,
                "name": f"qgc field {index}",
                "selected": True,
                "boundary_points": projected,
                "center": center,
                "area_hectares": area_m2 / 10000.0,
                "risk": 0.32,
                "notes": ["imported from QGroundControl .plan polygon"],
            }
        )

    min_x = min(p["x"] for p in all_points)
    max_x = max(p["x"] for p in all_points)
    min_y = min(p["y"] for p in all_points)
    max_y = max(p["y"] for p in all_points)
    scenario = {
        "source": str(input_path),
        "source_mode": "qgroundcontrol_fence_polygon",
        "road_model": "task_boundary_exterior_accessible",
        "origin": {"lat": origin_lat, "lon": origin_lon},
        "boundary_points": [
            {"x": min_x, "y": min_y},
            {"x": max_x, "y": min_y},
            {"x": max_x, "y": max_y},
            {"x": min_x, "y": max_y},
        ],
        "field_blocks": blocks,
        "scout_boundary_routes": build_scout_routes(blocks, altitude_m=28.0),
        "depot_sites": build_depots(blocks, depot_spacing_m, outside_offset_m),
        "terrain_complexity": 0.35,
        "obstacle_density": 0.22,
        "notes": [
            "Generated from QGroundControl Fence/Polygon selection.",
            "Scout flies the closed fence boundary route to confirm the work region.",
            "Ideal road model: task boundaries and all exterior space are hive-accessible.",
            "Hive candidates are sampled along the outside of the task boundary.",
        ],
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(scenario, indent=2), encoding="utf-8")
    total_area = sum(block["area_hectares"] for block in blocks)
    print(f"wrote {output_path}")
    print(
        f"blocks={len(blocks)} area_ha={total_area:.2f} "
        f"scout_routes={len(scenario['scout_boundary_routes'])} depots={len(scenario['depot_sites'])}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert QGroundControl Fence/Polygon .plan to scout_opt scenario JSON.")
    parser.add_argument("input", type=Path, help="QGroundControl .plan file with one or more Fence/Polygon regions")
    parser.add_argument("-o", "--output", type=Path, default=Path("configs/qgc_scenario.json"))
    parser.add_argument("--depot-spacing-m", type=float, default=180.0, help="Hive candidate spacing along field boundary roads.")
    parser.add_argument("--outside-offset-m", type=float, default=35.0, help="Offset hive candidates outside the task boundary.")
    args = parser.parse_args()
    convert(args.input, args.output, args.depot_spacing_m, args.outside_offset_m)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

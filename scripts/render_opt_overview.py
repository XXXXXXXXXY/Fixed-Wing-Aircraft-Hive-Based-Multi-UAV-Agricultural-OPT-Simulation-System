#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon


def point_xy(point: dict[str, Any]) -> tuple[float, float]:
    return float(point["x"]), float(point["y"])


def draw_polyline(ax: Any, points: list[dict[str, Any]], color: str, width: float, alpha: float, z: int) -> None:
    if len(points) < 2:
        return
    xs = [float(p["x"]) for p in points]
    ys = [float(p["y"]) for p in points]
    ax.plot(xs, ys, color=color, linewidth=width, alpha=alpha, zorder=z)


def draw_dashed_polyline(ax: Any,
                         points: list[dict[str, Any]],
                         color: str,
                         width: float,
                         alpha: float,
                         z: int) -> None:
    if len(points) < 2:
        return
    xs = [float(p["x"]) for p in points]
    ys = [float(p["y"]) for p in points]
    ax.plot(xs, ys, color=color, linewidth=width, alpha=alpha, linestyle=(0, (5, 5)), zorder=z)


def strip_patch(points: list[dict[str, Any]], swath_m: float, color: str, alpha: float, z: int) -> Polygon | None:
    if len(points) < 2:
        return None
    a = points[0]
    b = points[-1]
    ax = float(a["x"])
    ay = float(a["y"])
    bx = float(b["x"])
    by = float(b["y"])
    length = math.hypot(bx - ax, by - ay)
    if length <= 0.001:
        return None
    half = max(0.2, swath_m * 0.5)
    nx = -(by - ay) / length * half
    ny = (bx - ax) / length * half
    return Polygon(
        [(ax + nx, ay + ny), (bx + nx, by + ny), (bx - nx, by - ny), (ax - nx, ay - ny)],
        closed=True,
        facecolor=color,
        edgecolor="none",
        alpha=alpha,
        zorder=z,
    )


def orient(a: dict[str, Any], b: dict[str, Any], c: dict[str, Any]) -> float:
    return (float(b["x"]) - float(a["x"])) * (float(c["y"]) - float(a["y"])) - (
        float(b["y"]) - float(a["y"])
    ) * (float(c["x"]) - float(a["x"]))


def segments_intersect(a: dict[str, Any], b: dict[str, Any], c: dict[str, Any], d: dict[str, Any]) -> bool:
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return ((o1 > 0 > o2) or (o1 < 0 < o2)) and ((o3 > 0 > o4) or (o3 < 0 < o4))


def same_point(a: dict[str, Any], b: dict[str, Any], eps: float = 1e-6) -> bool:
    return abs(float(a["x"]) - float(b["x"])) <= eps and abs(float(a["y"]) - float(b["y"])) <= eps


def point_in_polygon(point: dict[str, Any], polygon: list[dict[str, Any]]) -> bool:
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


def dist(a: dict[str, Any], b: dict[str, Any]) -> float:
    return ((float(a["x"]) - float(b["x"])) ** 2 + (float(a["y"]) - float(b["y"])) ** 2) ** 0.5


def heading(a: dict[str, Any], b: dict[str, Any]) -> float:
    return math.atan2(float(b["y"]) - float(a["y"]), float(b["x"]) - float(a["x"]))


def heading_diff(a: float, b: float) -> float:
    diff = abs(a - b) % (2.0 * math.pi)
    return min(diff, 2.0 * math.pi - diff)


def close_point(a: dict[str, Any], b: dict[str, Any], eps: float = 1.25) -> bool:
    return dist(a, b) <= eps


def line_polygon_intervals(poly: list[dict[str, Any]], angle: float, cross: float) -> list[tuple[float, float]]:
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


def polygon_cross_range(polygons: list[list[dict[str, Any]]], angle: float) -> tuple[float, float]:
    vx, vy = -math.sin(angle), math.cos(angle)
    values = [float(p["x"]) * vx + float(p["y"]) * vy for poly in polygons for p in poly]
    return min(values), max(values)


def global_fixed_wing_corridors(plan: dict[str, Any]) -> list[list[dict[str, float]]]:
    fixed = plan.get("fixed_wing") or {}
    swath = float(fixed.get("swath_m", 22.0) or 22.0)
    target_area = float(fixed.get("assigned_area_ha", 0.0) or 0.0)
    if target_area <= 0.0:
        target_area = sum(float(item.get("area_ha", 0.0) or 0.0) for item in plan.get("fixed_wing_routes", []))

    polygons = clean_polygons(plan.get("work_area", {}).get("blocks", []))
    if not polygons:
        return []

    spacing = max(swath, 18.0)
    best_angle = 0.0
    best_score = -1.0
    best_rows: list[tuple[float, float, float]] = []

    for deg in range(0, 180, 3):
        angle = math.radians(float(deg))
        min_cross, max_cross = polygon_cross_range(polygons, angle)
        rows: list[tuple[float, float, float]] = []
        cross = min_cross + spacing * 0.5
        while cross <= max_cross - spacing * 0.25:
            intervals: list[tuple[float, float]] = []
            for poly in polygons:
                intervals.extend(line_polygon_intervals(poly, angle, cross))
            if intervals:
                min_t = min(a for a, _b in intervals)
                max_t = max(b for _a, b in intervals)
                work_len = sum(b - a for a, b in intervals)
                corridor_len = max_t - min_t
                empty_len = max(0.0, corridor_len - work_len)
                if work_len * swath / 10000.0 >= 0.18:
                    net_score = work_len - empty_len * 0.72
                    if net_score > 40.0:
                        rows.append((net_score, corridor_len, min_t, max_t, cross, work_len, empty_len))  # type: ignore[arg-type]
            cross += spacing
        if not rows:
            continue
        rows.sort(key=lambda row: row[0], reverse=True)
        covered = 0.0
        selected: list[tuple[float, float, float]] = []
        for _net_score, _corridor_len, min_t, max_t, cross, work_len, _empty_len in rows:  # type: ignore[misc]
            selected.append((min_t, max_t, cross))
            covered += work_len * swath / 10000.0
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

    ux, uy = math.cos(best_angle), math.sin(best_angle)
    vx, vy = -uy, ux
    corridors: list[tuple[float, list[dict[str, float]]]] = []
    for min_t, max_t, cross in best_rows:
        corridor = [
            {"x": ux * min_t + vx * cross, "y": uy * min_t + vy * cross},
            {"x": ux * max_t + vx * cross, "y": uy * max_t + vy * cross},
        ]
        corridors.append((max_t - min_t, corridor))
    corridors.sort(key=lambda item: item[0], reverse=True)
    return [item[1] for item in corridors]


def merged_fixed_wing_corridors(plan: dict[str, Any]) -> list[list[dict[str, float]]]:
    routes: list[list[dict[str, float]]] = []
    for item in plan.get("fixed_wing_routes") or []:
        route = item.get("route") or []
        if len(route) >= 2:
            routes.append(route)
    if not routes:
        return global_fixed_wing_corridors(plan)

    corridors = list(routes)
    mission = plan.get("fixed_wing_trajectory") or []
    if len(mission) < 2:
        return corridors

    ordered: list[list[dict[str, float]]] = []
    used: set[int] = set()
    for i in range(len(mission) - 1):
        a = mission[i]
        b = mission[i + 1]
        for idx, route in enumerate(routes):
            if idx in used or len(route) < 2:
                continue
            r0, r1 = route[0], route[1]
            if close_point(a, r0) and close_point(b, r1):
                ordered.append([r0, r1])
                used.add(idx)
                break
            if close_point(a, r1) and close_point(b, r0):
                ordered.append([r1, r0])
                used.add(idx)
                break

    swath = float((plan.get("fixed_wing") or {}).get("swath_m", 22.0) or 22.0)
    max_continuous_gap = swath * 18.0
    max_heading_change = math.radians(7.0)
    for prev, nxt in zip(ordered, ordered[1:]):
        prev_heading = heading(prev[0], prev[1])
        next_heading = heading(nxt[0], nxt[1])
        gap = dist(prev[1], nxt[0])
        if gap <= max_continuous_gap and heading_diff(prev_heading, next_heading) <= max_heading_change:
            corridors.append([prev[1], nxt[0]])
    return corridors


def cleanup_sitl_bin_logs(actual_paths: Path | None, keep_logs: bool) -> int:
    if keep_logs or actual_paths is None:
        return 0
    root = actual_paths.parent
    if not root.exists() or root.name != "real_sitl":
        return 0
    deleted = 0
    for path in root.glob("**/*.BIN"):
        try:
            path.unlink()
            deleted += 1
        except OSError:
            pass
    return deleted


def boundary_distance(poly: list[dict[str, Any]], i: int, j: int) -> tuple[float, list[dict[str, Any]]]:
    n = len(poly)
    cw = [i]
    while cw[-1] != j:
        cw.append((cw[-1] + 1) % n)
    ccw = [i]
    while ccw[-1] != j:
        ccw.append((ccw[-1] - 1 + n) % n)

    def length(path: list[int]) -> float:
        return sum(dist(poly[path[k]], poly[path[k + 1]]) for k in range(len(path) - 1))

    return (length(cw), [poly[k] for k in cw]) if length(cw) <= length(ccw) else (length(ccw), [poly[k] for k in ccw])


def hive_leg_route(a: dict[str, Any], b: dict[str, Any], blocks: list[dict[str, Any]]) -> list[dict[str, Any]]:
    for block in blocks:
        poly = list(block.get("boundary") or [])
        if len(poly) >= 2 and poly[0] == poly[-1]:
            poly = poly[:-1]
        if len(poly) < 3:
            continue
        mid = {"x": (float(a["x"]) + float(b["x"])) / 2.0, "y": (float(a["y"]) + float(b["y"])) / 2.0}
        crosses = point_in_polygon(mid, poly) or sum(
            segments_intersect(a, b, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))
        ) >= 2
        if not crosses:
            continue
        def crosses_poly(x: dict[str, Any], y: dict[str, Any]) -> bool:
            m = {"x": (float(x["x"]) + float(y["x"])) / 2.0, "y": (float(x["y"]) + float(y["y"])) / 2.0}
            return point_in_polygon(m, poly) or sum(
                segments_intersect(x, y, poly[k], poly[(k + 1) % len(poly)]) for k in range(len(poly))
            ) >= 2

        best: list[dict[str, Any]] | None = None
        best_len = float("inf")
        for i in range(len(poly)):
            for j in range(len(poly)):
                if crosses_poly(a, poly[i]) or crosses_poly(poly[j], b):
                    continue
                boundary_len, boundary_path = boundary_distance(poly, i, j)
                candidate_len = dist(a, poly[i]) + boundary_len + dist(poly[j], b)
                if candidate_len < best_len:
                    best_len = candidate_len
                    best = [a] + boundary_path + [b]
        return best or [a, b]
    return [a, b]


def hive_route(stops: list[dict[str, Any]], blocks: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if len(stops) < 2:
        return stops
    route = [stops[0]]
    for i in range(len(stops) - 1):
        route.extend(hive_visibility_route(stops[i], stops[i + 1], blocks)[1:])
    return route


def clean_polygons(blocks: list[dict[str, Any]]) -> list[list[dict[str, Any]]]:
    polygons: list[list[dict[str, Any]]] = []
    for block in blocks:
        poly = [dict(p) for p in (block.get("boundary") or [])]
        if len(poly) >= 2 and same_point(poly[0], poly[-1]):
            poly = poly[:-1]
        if len(poly) >= 3:
            polygons.append(poly)
    return polygons


def segment_blocked(a: dict[str, Any], b: dict[str, Any], polygons: list[list[dict[str, Any]]]) -> bool:
    mid = {"x": (float(a["x"]) + float(b["x"])) * 0.5, "y": (float(a["y"]) + float(b["y"])) * 0.5}
    for poly in polygons:
        if point_in_polygon(mid, poly):
            return True
        for i in range(len(poly)):
            c, d = poly[i], poly[(i + 1) % len(poly)]
            if same_point(a, c) or same_point(a, d) or same_point(b, c) or same_point(b, d):
                continue
            if segments_intersect(a, b, c, d):
                return True
    return False


def hive_visibility_route(a: dict[str, Any], b: dict[str, Any], blocks: list[dict[str, Any]]) -> list[dict[str, Any]]:
    polygons = clean_polygons(blocks)
    if not segment_blocked(a, b, polygons):
        return [a, b]

    nodes: list[dict[str, Any]] = [a, b]
    for poly in polygons:
        nodes.extend(poly)

    n = len(nodes)
    graph: list[list[tuple[int, float]]] = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if segment_blocked(nodes[i], nodes[j], polygons):
                continue
            w = dist(nodes[i], nodes[j])
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


def draw_actual_paths(ax: Any, csv_path: Path) -> bool:
    if not csv_path.exists() or csv_path.stat().st_size <= 0:
        return False
    by_sysid: dict[str, list[tuple[float, float]]] = {}
    with csv_path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for row in reader:
            sysid = row.get("sysid", "")
            lat = row.get("lat", "")
            lon = row.get("lon", "")
            if not sysid or not lat or not lon:
                continue
            by_sysid.setdefault(sysid, []).append((float(lon), float(lat)))
    for sysid, points in by_sysid.items():
        if len(points) < 2:
            continue
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        ax.plot(xs, ys, linewidth=1.4, alpha=0.6, label=f"actual {sysid}")
    return bool(by_sysid)


def render(plan_path: Path, out_path: Path, actual_paths: Path | None, keep_sitl_logs: bool = False) -> int:
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    fig, ax = plt.subplots(figsize=(15, 10), dpi=180)
    ax.set_facecolor("#f8fafc")

    work_blocks = plan.get("work_area", {}).get("blocks", [])
    for block in work_blocks:
        boundary = block.get("boundary") or []
        if len(boundary) < 3:
            continue
        poly = Polygon(
            [point_xy(p) for p in boundary],
            closed=True,
            facecolor="#dbeafe",
            edgecolor="#0369a1",
            linewidth=2.2,
            alpha=0.34,
            zorder=1,
        )
        ax.add_patch(poly)
        center = block.get("center") or {}
        if "x" in center and "y" in center:
            ax.text(
                float(center["x"]),
                float(center["y"]),
                f"Field {block.get('block_id', '')}\n{float(block.get('area_ha', 0.0)):.1f} ha",
                ha="center",
                va="center",
                fontsize=9,
                color="#075985",
                zorder=7,
            )

    for scout in plan.get("scout_routes", []):
        draw_polyline(ax, scout.get("route") or [], "#0f766e", 1.5, 0.8, 5)

    fixed_wing_trajectory = plan.get("fixed_wing_trajectory") or []
    if fixed_wing_trajectory:
        draw_dashed_polyline(ax, fixed_wing_trajectory, "#ef4444", 0.75, 0.24, 3)

    fixed_swath = float((plan.get("fixed_wing") or {}).get("swath_m", 19.8) or 19.8)
    for item in plan.get("fixed_wing_routes") or []:
        route = item.get("route") or []
        patch = strip_patch(route, fixed_swath, "#fecaca", 0.46, 3)
        if patch is not None:
            ax.add_patch(patch)
    for corridor in merged_fixed_wing_corridors(plan):
        draw_polyline(ax, corridor, "#dc2626", 0.58, 0.52, 4)

    small_block_ids = {
        block.get("block_id")
        for block in work_blocks
        if float(block.get("area_ha", 0.0) or 0.0) < 20.0
    }
    for task in plan.get("tasks", []):
        coverage = task.get("coverage_route") or []
        is_small_block = task.get("block_id") in small_block_ids
        color = "#7dd3fc"
        swath_m = 10.0 if is_small_block else 7.0
        alpha = 0.72 if is_small_block else 0.62
        zorder = 8 if is_small_block else 6
        if coverage:
            for segment in coverage:
                patch = strip_patch(segment, swath_m, color, alpha, zorder)
                if patch is not None:
                    ax.add_patch(patch)
        else:
            route = task.get("route") or []
            patch = strip_patch(route, swath_m, color, alpha, zorder)
            if patch is not None:
                ax.add_patch(patch)

    hive = plan.get("hive") or {}
    stops = hive.get("stops") or []
    if stops:
        draw_polyline(ax, hive_route(stops, work_blocks), "#111827", 3.2, 0.9, 8)
        xs = [float(p["x"]) for p in stops]
        ys = [float(p["y"]) for p in stops]
        ax.scatter(xs, ys, s=145, marker="s", color="#111827", edgecolor="white", linewidth=1.6, zorder=9)
        for i, p in enumerate(stops, start=1):
            ax.text(float(p["x"]) + 24, float(p["y"]) + 24, f"Hive {i}", fontsize=10, weight="bold", zorder=10)

    fixed = plan.get("fixed_wing") or {}
    if fixed.get("enabled") and fixed.get("airport"):
        x, y = point_xy(fixed["airport"])
        ax.scatter([x], [y], s=170, marker="^", color="#7c2d12", edgecolor="white", linewidth=1.4, zorder=10)
        ax.text(x + 35, y + 35, "Fixed-wing airport", fontsize=10, weight="bold", color="#7c2d12", zorder=10)

    actual_drawn = draw_actual_paths(ax, actual_paths) if actual_paths else False

    legend_items = [
        Line2D([0], [0], color="#0369a1", lw=2.2, label="Work area boundary"),
        Line2D([0], [0], color="#0f766e", lw=1.8, label="Scout boundary scan"),
        Line2D([0], [0], color="#dc2626", lw=2.6, label="Fixed-wing spray strips (solid)"),
        Line2D([0], [0], color="#ef4444", lw=1.4, linestyle=(0, (5, 5)), label="Fixed-wing turns / ferry (dashed)"),
        Line2D([0], [0], color="#7dd3fc", lw=5.0, label="UAV completed repair / edge area"),
        Line2D([0], [0], color="#111827", lw=3.0, marker="s", label="Hive stops and movement"),
        Line2D([0], [0], color="#7c2d12", lw=0, marker="^", markersize=10, label="Fixed-wing airport"),
    ]
    if actual_drawn:
        legend_items.append(Line2D([0], [0], color="#6b7280", lw=1.4, label="Actual SITL paths"))
    ax.legend(handles=legend_items, loc="upper left", frameon=True, framealpha=0.92)

    ax.set_title("Scout OPT Agricultural Mission Overview", fontsize=16, weight="bold")
    ax.set_xlabel("Local X / east-west meters")
    ax.set_ylabel("Local Y / north-south meters")
    ax.grid(True, color="#cbd5e1", linewidth=0.6, alpha=0.65)
    ax.set_aspect("equal", adjustable="datalim")
    ax.margins(0.08)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    return cleanup_sitl_bin_logs(actual_paths, keep_sitl_logs)


def main() -> int:
    parser = argparse.ArgumentParser(description="Render OPT work area, routes, and Hive stops.")
    parser.add_argument("--plan", default="configs/opt_visual_plan.json")
    parser.add_argument("--out", default=".tmp/real_sitl/opt_overview.png")
    parser.add_argument("--actual-paths", default=".tmp/real_sitl/actual_paths.csv")
    parser.add_argument("--keep-sitl-logs", action="store_true", help="Keep large ArduPilot .BIN logs after rendering.")
    args = parser.parse_args()
    deleted = render(Path(args.plan), Path(args.out), Path(args.actual_paths), keep_sitl_logs=args.keep_sitl_logs)
    print(args.out)
    if deleted:
        print(f"deleted_sitl_bin_logs={deleted}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
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


def orient(a: dict[str, Any], b: dict[str, Any], c: dict[str, Any]) -> float:
    return (float(b["x"]) - float(a["x"])) * (float(c["y"]) - float(a["y"])) - (
        float(b["y"]) - float(a["y"])
    ) * (float(c["x"]) - float(a["x"]))


def segments_intersect(a: dict[str, Any], b: dict[str, Any], c: dict[str, Any], d: dict[str, Any]) -> bool:
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return ((o1 > 0 > o2) or (o1 < 0 < o2)) and ((o3 > 0 > o4) or (o3 < 0 < o4))


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
        leg = hive_leg_route(stops[i], stops[i + 1], blocks)
        route.extend(leg[1:])
    return route


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


def render(plan_path: Path, out_path: Path, actual_paths: Path | None) -> None:
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

    for task in plan.get("tasks", []):
        route = task.get("route") or []
        if task.get("handling") == "fixed_wing":
            draw_polyline(ax, route, "#dc2626", 1.0, 0.56, 4)
        else:
            draw_polyline(ax, route, "#2563eb", 0.95, 0.50, 3)

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
        Line2D([0], [0], color="#dc2626", lw=1.5, label="Fixed-wing spray strips"),
        Line2D([0], [0], color="#2563eb", lw=1.5, label="Drone work / repair strips"),
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


def main() -> int:
    parser = argparse.ArgumentParser(description="Render OPT work area, routes, and Hive stops.")
    parser.add_argument("--plan", default="configs/opt_visual_plan.json")
    parser.add_argument("--out", default=".tmp/real_sitl/opt_overview.png")
    parser.add_argument("--actual-paths", default=".tmp/real_sitl/actual_paths.csv")
    args = parser.parse_args()
    render(Path(args.plan), Path(args.out), Path(args.actual_paths))
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

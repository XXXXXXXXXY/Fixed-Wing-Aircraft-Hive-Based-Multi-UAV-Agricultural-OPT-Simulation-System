#!/usr/bin/env python3
"""Print real SITL path movement sampled by ardupilot_real_router.py."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


def distance_m(a_lat: float, a_lon: float, b_lat: float, b_lon: float) -> float:
    north = (b_lat - a_lat) * 111_111.0
    east = (b_lon - a_lon) * 111_111.0 * math.cos(math.radians(a_lat))
    return math.hypot(north, east)


def local_to_latlon(point: dict[str, float], origin: dict[str, float]) -> tuple[float, float]:
    lat0 = float(origin["lat"])
    lon0 = float(origin["lon"])
    return (
        lat0 + float(point["y"]) / 111_111.0,
        lon0 + float(point["x"]) / (111_111.0 * math.cos(math.radians(lat0))),
    )


def planned_summary(path: str) -> dict[int, dict[str, object]]:
    if not path or not Path(path).exists():
        return {}
    plan = json.loads(Path(path).read_text(encoding="utf-8"))
    origin = plan["origin"]
    out: dict[int, dict[str, object]] = {}
    for task in plan.get("tasks", []):
        handling = task.get("handling")
        if handling == "drone":
            sysid = int(task.get("assigned_drone_id", -1))
            if sysid < 1:
                continue
        elif handling == "fixed_wing":
            sysid = 100
        else:
            continue
        points = task.get("route") or []
        if len(points) < 2:
            continue
        first = local_to_latlon(points[0], origin)
        last = local_to_latlon(points[-1], origin)
        entry = out.setdefault(sysid, {"segments": 0, "first": first, "last": last})
        entry["segments"] = int(entry["segments"]) + 1
        entry["last"] = last

    hive = plan.get("hive") or {}
    stops = hive.get("stops") or []
    if stops:
        first = local_to_latlon(stops[0], origin)
        last = local_to_latlon(stops[-1], origin)
        out[200] = {"segments": len(stops), "first": first, "last": last}
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize actual SITL paths recorded by the real router.")
    parser.add_argument("--csv", default=".tmp/real_sitl/actual_paths.csv")
    parser.add_argument("--visual-plan", default="configs/opt_visual_plan.json")
    parser.add_argument("--min-move-m", type=float, default=5.0)
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise SystemExit(f"path log not found: {csv_path}")

    rows_by_sysid: dict[int, list[dict[str, str]]] = defaultdict(list)
    with csv_path.open("r", newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            lat = float(row["lat"])
            lon = float(row["lon"])
            if abs(lat) < 1.0 and abs(lon) < 1.0:
                continue
            rows_by_sysid[int(row["sysid"])].append(row)

    planned = planned_summary(args.visual_plan)
    print("REAL PATH TEST")
    print(f"log={csv_path}")
    print(f"vehicles={len(rows_by_sysid)}")

    for sysid in sorted(rows_by_sysid):
        rows = rows_by_sysid[sysid]
        first = rows[0]
        last = rows[-1]
        first_lat = float(first["lat"])
        first_lon = float(first["lon"])
        last_lat = float(last["lat"])
        last_lon = float(last["lon"])
        displacement = distance_m(first_lat, first_lon, last_lat, last_lon)
        travelled = float(last["travelled_m"])
        speed = float(last["speed_mps"])
        target_dist = last["target_dist_m"] or "n/a"
        moved = "YES" if travelled >= args.min_move_m else "NO"
        plan = planned.get(sysid)
        plan_text = ""
        if plan:
            pfirst = plan["first"]
            plast = plan["last"]
            plan_text = (
                f" planned_segments={plan['segments']} "
                f"planned_first=({pfirst[0]:.7f},{pfirst[1]:.7f}) "
                f"planned_last=({plast[0]:.7f},{plast[1]:.7f})"
            )
        print(
            f"sysid={sysid:>3} role={last['role']:<6} samples={len(rows):>5} "
            f"moved={moved:<3} travelled={travelled:>8.1f}m displacement={displacement:>7.1f}m "
            f"last_speed={speed:>5.2f}mps wp={last['wp_index']}/{last['wp_count']} "
            f"target_dist={target_dist} last=({last_lat:.7f},{last_lon:.7f},alt={float(last['rel_alt_m']):.1f})"
            f"{plan_text}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

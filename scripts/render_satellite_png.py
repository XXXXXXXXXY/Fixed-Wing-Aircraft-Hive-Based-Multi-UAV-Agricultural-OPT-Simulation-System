#!/usr/bin/env python3
from __future__ import annotations

import argparse
import io
import json
import math
from pathlib import Path
from typing import Any

import requests
from PIL import Image, ImageDraw, ImageFont

from render_opt_overview import hive_route, merged_fixed_wing_corridors
from render_satellite_overlay import local_to_latlon


TILE_SIZE = 256


def latlon_to_pixel(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    sin_lat = math.sin(math.radians(lat))
    world = TILE_SIZE * (2 ** zoom)
    x = (lon + 180.0) / 360.0 * world
    y = (0.5 - math.log((1.0 + sin_lat) / (1.0 - sin_lat)) / (4.0 * math.pi)) * world
    return x, y


def pixel_to_tile(x: float, y: float) -> tuple[int, int]:
    return int(math.floor(x / TILE_SIZE)), int(math.floor(y / TILE_SIZE))


def fetch_tile(x: int, y: int, z: int) -> Image.Image:
    url = f"https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
    resp = requests.get(url, timeout=20)
    resp.raise_for_status()
    return Image.open(io.BytesIO(resp.content)).convert("RGB")


def draw_polyline(
    draw: ImageDraw.ImageDraw,
    points: list[tuple[float, float]],
    color: tuple[int, int, int, int],
    width: int,
) -> None:
    if len(points) < 2:
        return
    draw.line(points, fill=color, width=width, joint="curve")


def draw_dashed_polyline(
    draw: ImageDraw.ImageDraw,
    points: list[tuple[float, float]],
    color: tuple[int, int, int, int],
    width: int,
    dash: float = 18.0,
    gap: float = 14.0,
) -> None:
    if len(points) < 2:
        return
    for a, b in zip(points, points[1:]):
        ax, ay = a
        bx, by = b
        length = math.hypot(bx - ax, by - ay)
        if length <= 0.001:
            continue
        ux = (bx - ax) / length
        uy = (by - ay) / length
        pos = 0.0
        while pos < length:
            end = min(length, pos + dash)
            p0 = (ax + ux * pos, ay + uy * pos)
            p1 = (ax + ux * end, ay + uy * end)
            draw.line([p0, p1], fill=color, width=width)
            pos += dash + gap


def plan_points(plan: dict[str, Any]) -> list[tuple[float, float]]:
    origin = plan["origin"]
    out: list[tuple[float, float]] = []
    for block in plan.get("work_area", {}).get("blocks", []):
        for p in block.get("boundary") or []:
            out.append(local_to_latlon(origin, p))
    fixed = plan.get("fixed_wing") or {}
    if fixed.get("airport"):
        out.append(local_to_latlon(origin, fixed["airport"]))
    return out


def render(plan_path: Path, out_path: Path, zoom: int) -> None:
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    origin = plan["origin"]
    latlons = plan_points(plan)
    if not latlons:
        raise SystemExit("plan has no drawable geographic points")

    px = [latlon_to_pixel(lat, lon, zoom)[0] for lat, lon in latlons]
    py = [latlon_to_pixel(lat, lon, zoom)[1] for lat, lon in latlons]
    pad = 260
    min_px, max_px = min(px) - pad, max(px) + pad
    min_py, max_py = min(py) - pad, max(py) + pad
    min_tx, min_ty = pixel_to_tile(min_px, min_py)
    max_tx, max_ty = pixel_to_tile(max_px, max_py)

    width = (max_tx - min_tx + 1) * TILE_SIZE
    height = (max_ty - min_ty + 1) * TILE_SIZE
    base = Image.new("RGB", (width, height), "#0f172a")
    for tx in range(min_tx, max_tx + 1):
        for ty in range(min_ty, max_ty + 1):
            tile = fetch_tile(tx, ty, zoom)
            base.paste(tile, ((tx - min_tx) * TILE_SIZE, (ty - min_ty) * TILE_SIZE))

    overlay = Image.new("RGBA", base.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)

    def xy(point: dict[str, Any]) -> tuple[float, float]:
        lat, lon = local_to_latlon(origin, point)
        x, y = latlon_to_pixel(lat, lon, zoom)
        return x - min_tx * TILE_SIZE, y - min_ty * TILE_SIZE

    for block in plan.get("work_area", {}).get("blocks", []):
        boundary = block.get("boundary") or []
        if len(boundary) < 3:
            continue
        pts = [xy(p) for p in boundary]
        draw.polygon(pts, fill=(14, 165, 233, 42), outline=(2, 132, 199, 235))
        draw.line(pts + [pts[0]], fill=(2, 132, 199, 255), width=5)

    for scout in plan.get("scout_routes", []):
        draw_polyline(draw, [xy(p) for p in scout.get("route") or []], (15, 118, 110, 230), 4)

    fixed_wing_trajectory = plan.get("fixed_wing_trajectory") or []
    if fixed_wing_trajectory:
        draw_dashed_polyline(draw, [xy(p) for p in fixed_wing_trajectory], (239, 68, 68, 120), 2)

    for corridor in merged_fixed_wing_corridors(plan):
        draw_polyline(draw, [xy(p) for p in corridor], (220, 38, 38, 230), 4)

    for task in plan.get("tasks", []):
        coverage = task.get("coverage_route") or []
        if coverage:
            for segment in coverage:
                draw_polyline(draw, [xy(p) for p in segment], (37, 99, 235, 170), 2)
        else:
            draw_polyline(draw, [xy(p) for p in task.get("route") or []], (37, 99, 235, 185), 3)

    hive = plan.get("hive") or {}
    stops = hive.get("stops") or []
    safe_hive = hive_route(stops, plan.get("work_area", {}).get("blocks", []))
    draw_polyline(draw, [xy(p) for p in safe_hive], (17, 24, 39, 255), 8)
    for i, stop in enumerate(stops, start=1):
        sx, sy = xy(stop)
        draw.rectangle((sx - 9, sy - 9, sx + 9, sy + 9), fill=(17, 24, 39, 255), outline=(255, 255, 255, 255), width=2)
        draw.text((sx + 12, sy - 8), f"Hive {i}", fill=(255, 255, 255, 255))

    fixed = plan.get("fixed_wing") or {}
    if fixed.get("airport"):
        ax, ay = xy(fixed["airport"])
        draw.polygon([(ax, ay - 16), (ax - 15, ay + 14), (ax + 15, ay + 14)], fill=(124, 45, 18, 255))
        draw.text((ax + 18, ay - 10), "Fixed-wing airport", fill=(255, 255, 255, 255))

    legend_x, legend_y = 24, 24
    legend_w, legend_h = 430, 202
    draw.rounded_rectangle(
        (legend_x, legend_y, legend_x + legend_w, legend_y + legend_h),
        radius=10,
        fill=(255, 255, 255, 218),
    )
    try:
        font = ImageFont.truetype("arial.ttf", 18)
        title_font = ImageFont.truetype("arialbd.ttf", 22)
    except OSError:
        font = ImageFont.load_default()
        title_font = font
    draw.text((legend_x + 16, legend_y + 12), "Xiaolizhuang Satellite OPT Overlay", fill=(15, 23, 42, 255), font=title_font)
    rows = [
        ((2, 132, 199, 255), "Work area boundary"),
        ((220, 38, 38, 255), "Fixed-wing spray strips"),
        ((239, 68, 68, 180), "Fixed-wing turns / ferry"),
        ((37, 99, 235, 255), "Drone work / repair strips"),
        ((17, 24, 39, 255), "Hive route outside fields"),
        ((15, 118, 110, 255), "Scout boundary scan"),
    ]
    for idx, (color, label) in enumerate(rows):
        y = legend_y + 52 + idx * 23
        draw.line((legend_x + 18, y + 9, legend_x + 74, y + 9), fill=color, width=5)
        draw.text((legend_x + 88, y), label, fill=(15, 23, 42, 255), font=font)

    result = Image.alpha_composite(base.convert("RGBA"), overlay)
    result.thumbnail((2200, 1400), Image.Resampling.LANCZOS)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    result.convert("RGB").save(out_path, quality=92)


def cleanup_sitl_bin_logs(keep_logs: bool) -> int:
    if keep_logs:
        return 0
    root = Path(".tmp/real_sitl")
    if not root.exists():
        return 0
    deleted = 0
    for path in root.glob("**/*.BIN"):
        try:
            path.unlink()
            deleted += 1
        except OSError:
            pass
    return deleted


def main() -> int:
    parser = argparse.ArgumentParser(description="Render a static satellite PNG with OPT overlays.")
    parser.add_argument("--plan", default="configs/xiaolizhuang_opt_visual_plan.json")
    parser.add_argument("--out", default="docs/xiaolizhuang_satellite_overlay.png")
    parser.add_argument("--zoom", type=int, default=14)
    parser.add_argument("--keep-sitl-logs", action="store_true", help="Keep large ArduPilot .BIN logs after rendering.")
    args = parser.parse_args()
    render(Path(args.plan), Path(args.out), args.zoom)
    print(args.out)
    deleted = cleanup_sitl_bin_logs(args.keep_sitl_logs)
    if deleted:
        print(f"deleted_sitl_bin_logs={deleted}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

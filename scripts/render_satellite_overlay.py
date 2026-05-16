#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from html import escape
from pathlib import Path
from typing import Any

from render_opt_overview import hive_route


EARTH_RADIUS_M = 6_378_137.0


def local_to_latlon(origin: dict[str, Any], point: dict[str, Any]) -> tuple[float, float]:
    lat0 = math.radians(float(origin["lat"]))
    lon0 = math.radians(float(origin["lon"]))
    north = float(point["y"])
    east = float(point["x"])
    lat = lat0 + north / EARTH_RADIUS_M
    lon = lon0 + east / (EARTH_RADIUS_M * max(0.01, math.cos(lat0)))
    return math.degrees(lat), math.degrees(lon)


def latlon_list(origin: dict[str, Any], points: list[dict[str, Any]]) -> list[tuple[float, float]]:
    return [local_to_latlon(origin, p) for p in points if "x" in p and "y" in p]


def style_for(kind: str) -> tuple[str, int, float]:
    if kind == "field":
        return "#0284c7", 3, 0.75
    if kind == "scout":
        return "#0f766e", 2, 0.85
    if kind == "fixed":
        return "#dc2626", 2, 0.75
    if kind == "drone":
        return "#2563eb", 2, 0.65
    if kind == "hive":
        return "#111827", 5, 0.95
    return "#64748b", 2, 0.8


def js_points(points: list[tuple[float, float]]) -> str:
    return "[" + ",".join(f"[{lat:.8f},{lon:.8f}]" for lat, lon in points) + "]"


def kml_coords(points: list[tuple[float, float]]) -> str:
    return " ".join(f"{lon:.8f},{lat:.8f},0" for lat, lon in points)


def html_polyline(name: str, points: list[tuple[float, float]], kind: str) -> str:
    color, width, opacity = style_for(kind)
    return (
        f"L.polyline({js_points(points)}, "
        f"{{color:'{color}',weight:{width},opacity:{opacity}}})"
        f".bindPopup('{escape(name)}').addTo({kind}Layer);\n"
    )


def html_polygon(name: str, points: list[tuple[float, float]]) -> str:
    color, width, opacity = style_for("field")
    return (
        f"L.polygon({js_points(points)}, "
        f"{{color:'{color}',weight:{width},opacity:{opacity},fillColor:'#38bdf8',fillOpacity:0.18}})"
        f".bindPopup('{escape(name)}').addTo(fieldLayer);\n"
    )


def kml_line(name: str, points: list[tuple[float, float]], color: str) -> str:
    return f"""
    <Placemark>
      <name>{escape(name)}</name>
      <Style><LineStyle><color>{color}</color><width>3</width></LineStyle></Style>
      <LineString><tessellate>1</tessellate><coordinates>{kml_coords(points)}</coordinates></LineString>
    </Placemark>"""


def kml_poly(name: str, points: list[tuple[float, float]]) -> str:
    if points and points[0] != points[-1]:
        points = points + [points[0]]
    return f"""
    <Placemark>
      <name>{escape(name)}</name>
      <Style>
        <LineStyle><color>ccff8402</color><width>3</width></LineStyle>
        <PolyStyle><color>3338bdf8</color></PolyStyle>
      </Style>
      <Polygon><outerBoundaryIs><LinearRing><coordinates>{kml_coords(points)}</coordinates></LinearRing></outerBoundaryIs></Polygon>
    </Placemark>"""


def render(plan_path: Path, html_out: Path, kml_out: Path) -> None:
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    origin = plan["origin"]
    html_shapes: list[str] = []
    kml_shapes: list[str] = []
    all_points: list[tuple[float, float]] = []

    for block in plan.get("work_area", {}).get("blocks", []):
        pts = latlon_list(origin, block.get("boundary") or [])
        if len(pts) >= 3:
            name = f"Field {block.get('block_id')} {float(block.get('area_ha', 0.0)):.1f} ha"
            html_shapes.append(html_polygon(name, pts))
            kml_shapes.append(kml_poly(name, pts))
            all_points.extend(pts)

    for scout in plan.get("scout_routes", []):
        pts = latlon_list(origin, scout.get("route") or [])
        if len(pts) >= 2:
            html_shapes.append(html_polyline(f"Scout drone {scout.get('drone_id')}", pts, "scout"))
            kml_shapes.append(kml_line(f"Scout drone {scout.get('drone_id')}", pts, "cc0f766e"))
            all_points.extend(pts)

    for item in plan.get("fixed_wing_routes", []):
        pts = latlon_list(origin, item.get("route") or [])
        if len(pts) >= 2:
            html_shapes.append(html_polyline(f"Fixed-wing task {item.get('task_id')}", pts, "fixed"))
            kml_shapes.append(kml_line(f"Fixed-wing task {item.get('task_id')}", pts, "cc2626dc"))
            all_points.extend(pts)

    for task in plan.get("tasks", []):
        pts = latlon_list(origin, task.get("route") or [])
        if len(pts) >= 2:
            html_shapes.append(html_polyline(f"Drone task {task.get('id')}", pts, "drone"))
            kml_shapes.append(kml_line(f"Drone task {task.get('id')}", pts, "cc2563eb"))
            all_points.extend(pts)

    hive_stops = plan.get("hive", {}).get("stops") or []
    hive_safe = hive_route(hive_stops, plan.get("work_area", {}).get("blocks", []))
    hive_pts = latlon_list(origin, hive_safe)
    if len(hive_pts) >= 2:
        html_shapes.append(html_polyline("Hive movement", hive_pts, "hive"))
        kml_shapes.append(kml_line("Hive movement", hive_pts, "cc111827"))
        all_points.extend(hive_pts)

    fixed = plan.get("fixed_wing") or {}
    airport_js = ""
    if fixed.get("airport"):
        lat, lon = local_to_latlon(origin, fixed["airport"])
        airport_js = (
            f"L.marker([{lat:.8f},{lon:.8f}], {{title:'Fixed-wing airport'}})"
            ".bindPopup('Fixed-wing airport').addTo(markerLayer);\n"
        )
        kml_shapes.append(f"""
    <Placemark>
      <name>Fixed-wing airport</name>
      <Point><coordinates>{lon:.8f},{lat:.8f},0</coordinates></Point>
    </Placemark>""")
        all_points.append((lat, lon))

    center = all_points[0] if all_points else (float(origin["lat"]), float(origin["lon"]))
    html_out.parent.mkdir(parents=True, exist_ok=True)
    kml_out.parent.mkdir(parents=True, exist_ok=True)
    html_out.write_text(f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>{escape(plan_path.stem)} satellite overlay</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    html, body, #map {{ height: 100%; margin: 0; }}
    .legend {{ background: white; padding: 10px 12px; border-radius: 6px; box-shadow: 0 1px 8px #0003; font: 13px Arial; }}
    .legend div {{ margin: 4px 0; }}
    .swatch {{ display:inline-block; width:18px; height:3px; margin-right:7px; vertical-align:middle; }}
  </style>
</head>
<body>
<div id="map"></div>
<script>
const map = L.map('map').setView([{center[0]:.8f},{center[1]:.8f}], 15);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}', {{
  maxZoom: 20,
  attribution: 'Tiles © Esri'
}}).addTo(map);
const fieldLayer = L.layerGroup().addTo(map);
const scoutLayer = L.layerGroup().addTo(map);
const fixedLayer = L.layerGroup().addTo(map);
const droneLayer = L.layerGroup().addTo(map);
const hiveLayer = L.layerGroup().addTo(map);
const markerLayer = L.layerGroup().addTo(map);
{''.join(html_shapes)}
{airport_js}
const bounds = L.latLngBounds({js_points(all_points)});
if (bounds.isValid()) map.fitBounds(bounds.pad(0.08));
L.control.layers(null, {{
  'Field boundary': fieldLayer,
  'Scout boundary scan': scoutLayer,
  'Fixed-wing spray': fixedLayer,
  'Drone work/repair': droneLayer,
  'Hive movement': hiveLayer,
  'Markers': markerLayer
}}, {{collapsed:false}}).addTo(map);
const legend = L.control({{position:'bottomleft'}});
legend.onAdd = function() {{
  const div = L.DomUtil.create('div', 'legend');
  div.innerHTML = '<b>OPT Satellite Overlay</b>' +
    '<div><span class="swatch" style="background:#0284c7"></span>Field boundary</div>' +
    '<div><span class="swatch" style="background:#0f766e"></span>Scout</div>' +
    '<div><span class="swatch" style="background:#dc2626"></span>Fixed-wing</div>' +
    '<div><span class="swatch" style="background:#2563eb"></span>Drone</div>' +
    '<div><span class="swatch" style="background:#111827;height:5px"></span>Hive</div>';
  return div;
}};
legend.addTo(map);
</script>
</body>
</html>
""", encoding="utf-8")

    kml_out.write_text(f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>{escape(plan_path.stem)} satellite overlay</name>
    {''.join(kml_shapes)}
  </Document>
</kml>
""", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Render OPT routes onto satellite imagery as HTML and KML overlays.")
    parser.add_argument("--plan", default="configs/xiaolizhuang_opt_visual_plan.json")
    parser.add_argument("--html-out", default="docs/xiaolizhuang_satellite_overlay.html")
    parser.add_argument("--kml-out", default="docs/xiaolizhuang_satellite_overlay.kml")
    args = parser.parse_args()
    render(Path(args.plan), Path(args.html_out), Path(args.kml_out))
    print(args.html_out)
    print(args.kml_out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from scout_opt.core.models import DepotSite, FieldBlock, FieldMap, OperationZone, Point
from scout_opt.core.tasks import build_operation_zones_from_blocks


@dataclass(frozen=True)
class ManualScoutObservation:
    boundary_points: list[Point] = field(default_factory=list)
    field_blocks: list[FieldBlock] = field(default_factory=list)
    obstacles: list[Point] = field(default_factory=list)
    road_access_points: list[Point] = field(default_factory=list)
    candidate_depot_points: list[Point] = field(default_factory=list)
    depot_sites: list[DepotSite] = field(default_factory=list)
    terrain_complexity: float | None = None
    obstacle_density: float | None = None
    zone_risks: dict[int, float] = field(default_factory=dict)
    zone_notes: dict[int, list[str]] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)


def apply_manual_scout_observation(field: FieldMap, observation: ManualScoutObservation) -> None:
    if observation.boundary_points:
        field.boundary_center = _centroid(observation.boundary_points)
    if observation.obstacles:
        field.obstacles = list(observation.obstacles)
    if observation.road_access_points:
        field.road_access_points = list(observation.road_access_points)
    if observation.candidate_depot_points:
        field.candidate_depot_points = list(observation.candidate_depot_points)
    if observation.depot_sites:
        field.depot_sites = list(observation.depot_sites)
        field.candidate_depot_points = [site.point for site in field.depot_sites]
    if observation.terrain_complexity is not None:
        field.terrain_complexity = observation.terrain_complexity
    if observation.obstacle_density is not None:
        field.obstacle_density = observation.obstacle_density
    field.manual_notes.extend(observation.notes)

    if observation.field_blocks:
        field.field_blocks = [block for block in observation.field_blocks if block.selected]
        field.area_hectares = sum(block.area_hectares for block in field.field_blocks)
        field.boundary_center = _centroid([block.center for block in field.field_blocks])
        block_depots = [
            point
            for block in field.field_blocks
            for point in block.candidate_depot_points
        ]
        if block_depots:
            field.candidate_depot_points = block_depots
        field.operation_zones = build_operation_zones_from_blocks(field.field_blocks)
        field.tasks = []

    for zone in field.operation_zones:
        if zone.id in observation.zone_risks:
            zone.risk = observation.zone_risks[zone.id]
        if zone.id in observation.zone_notes:
            zone.notes.extend(observation.zone_notes[zone.id])
        zone.scanned = True

    if not field.operation_zones:
        field.operation_zones = _fallback_zones_from_boundary(field)

    field.scanned = True


def load_manual_scout_observation(path: str | Path) -> ManualScoutObservation:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return ManualScoutObservation(
        boundary_points=_points(data.get("boundary_points", [])),
        field_blocks=_field_blocks(data.get("field_blocks", [])),
        obstacles=_points(data.get("obstacles", [])),
        road_access_points=_points(data.get("road_access_points", [])),
        candidate_depot_points=_points(data.get("candidate_depot_points", [])),
        depot_sites=_depot_sites(data.get("depot_sites", [])),
        terrain_complexity=_optional_float(data.get("terrain_complexity")),
        obstacle_density=_optional_float(data.get("obstacle_density")),
        zone_risks={int(k): float(v) for k, v in data.get("zone_risks", {}).items()},
        zone_notes={
            int(k): [str(note) for note in notes]
            for k, notes in data.get("zone_notes", {}).items()
        },
        notes=[str(note) for note in data.get("notes", [])],
    )


def _points(items: list[dict[str, Any]]) -> list[Point]:
    return [Point(float(item["x"]), float(item["y"])) for item in items]


def _field_blocks(items: list[dict[str, Any]]) -> list[FieldBlock]:
    blocks: list[FieldBlock] = []
    for item in items:
        boundary = _points(item.get("boundary_points", []))
        center = _point_or_centroid(item.get("center"), boundary)
        blocks.append(
            FieldBlock(
                id=int(item["id"]),
                name=str(item.get("name", f"block-{item['id']}")),
                boundary_points=boundary,
                area_hectares=float(item.get("area_hectares", _polygon_area_hectares(boundary))),
                center=center,
                risk=float(item.get("risk", 0.0)),
                selected=bool(item.get("selected", True)),
                notes=[str(note) for note in item.get("notes", [])],
                candidate_depot_points=_points(item.get("candidate_depot_points", [])),
            )
        )
    return blocks


def _depot_sites(items: list[dict[str, Any]]) -> list[DepotSite]:
    sites: list[DepotSite] = []
    for idx, item in enumerate(items):
        point_data = item["point"]
        sites.append(
            DepotSite(
                id=int(item.get("id", idx + 1)),
                point=Point(float(point_data["x"]), float(point_data["y"])),
                usable_area_m2=float(item.get("usable_area_m2", 400.0)),
                road_accessible=bool(item.get("road_accessible", True)),
                slope_risk=float(item.get("slope_risk", 0.0)),
                notes=[str(note) for note in item.get("notes", [])],
            )
        )
    return sites


def _point_or_centroid(center_data: dict[str, Any] | None, boundary: list[Point]) -> Point:
    if center_data:
        return Point(float(center_data["x"]), float(center_data["y"]))
    if boundary:
        return _centroid(boundary)
    return Point(0.0, 0.0)


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    return float(value)


def _centroid(points: list[Point]) -> Point:
    return Point(
        x=sum(point.x for point in points) / len(points),
        y=sum(point.y for point in points) / len(points),
    )


def _polygon_area_hectares(points: list[Point]) -> float:
    if len(points) < 3:
        return 0.0
    area_m2 = 0.0
    for idx, point in enumerate(points):
        nxt = points[(idx + 1) % len(points)]
        area_m2 += point.x * nxt.y - nxt.x * point.y
    return abs(area_m2) / 2.0 / 10_000.0


def _fallback_zones_from_boundary(field: FieldMap) -> list[OperationZone]:
    center = field.boundary_center
    area = field.area_hectares / 4.0
    return [
        OperationZone(id=1, name="manual-nw", center=Point(center.x - 120, center.y + 120), area_hectares=area, scanned=True),
        OperationZone(id=2, name="manual-ne", center=Point(center.x + 120, center.y + 120), area_hectares=area, scanned=True),
        OperationZone(id=3, name="manual-sw", center=Point(center.x - 120, center.y - 120), area_hectares=area, scanned=True),
        OperationZone(id=4, name="manual-se", center=Point(center.x + 120, center.y - 120), area_hectares=area, scanned=True),
    ]

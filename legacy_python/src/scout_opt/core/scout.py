from __future__ import annotations

from scout_opt.core.models import DepotSite, FieldBlock, FieldMap, OperationZone, Point
from scout_opt.core.tasks import build_operation_zones_from_blocks


def decide_scout_count(
    area_size: float,
    terrain_complexity: float,
    obstacle_density: float,
    drone_inventory: int,
) -> int:
    if area_size < 20 and terrain_complexity < 0.3 and obstacle_density < 0.25:
        return 1
    if area_size < 80:
        return min(2, drone_inventory)
    return min(3, max(1, drone_inventory // 3))


def synthesize_scout_outputs(field: FieldMap) -> None:
    radius = max(150.0, (field.area_hectares * 10_000.0) ** 0.5 / 2.0)
    field.road_access_points = [
        Point(-radius - 120.0, 0.0),
        Point(0.0, -radius - 90.0),
        Point(radius + 150.0, radius * 0.25),
    ]
    field.candidate_depot_points = [
        Point(-radius * 0.75, 0.0),
        Point(0.0, -radius * 0.65),
        Point(radius * 0.55, radius * 0.2),
        Point(0.0, 0.0),
    ]
    field.depot_sites = [
        DepotSite(id=idx + 1, point=point, usable_area_m2=400.0, road_accessible=True)
        for idx, point in enumerate(field.candidate_depot_points)
    ]
    zone_area = field.area_hectares / 4.0
    if not field.field_blocks:
        field.operation_zones = [
            OperationZone(id=1, center=Point(-radius * 0.35, -radius * 0.35), area_hectares=zone_area),
            OperationZone(id=2, center=Point(radius * 0.35, -radius * 0.35), area_hectares=zone_area),
            OperationZone(id=3, center=Point(-radius * 0.35, radius * 0.35), area_hectares=zone_area),
            OperationZone(id=4, center=Point(radius * 0.35, radius * 0.35), area_hectares=zone_area),
        ]
    else:
        field.operation_zones = build_operation_zones_from_blocks(field.field_blocks)
    field.scanned = True

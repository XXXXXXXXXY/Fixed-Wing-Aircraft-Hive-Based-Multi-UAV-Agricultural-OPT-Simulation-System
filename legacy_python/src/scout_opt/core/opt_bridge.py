from __future__ import annotations

from scout_opt.core.models import FieldMap, Mothership, OperationZone, Point
from scout_opt.core.opt_engine import DepotCandidate, FieldZone, MothershipConfig
from scout_opt.core.scout import synthesize_scout_outputs


def mothership_to_opt_config(mothership: Mothership) -> MothershipConfig:
    return MothershipConfig(
        drone_slots=mothership.drone_slots,
        fast_chargers=mothership.fast_chargers,
        refill_ports=mothership.refill_ports,
        move_speed_mps=mothership.move_speed_mps,
        current_point=mothership.position,
    )


def field_to_opt_zones(field: FieldMap) -> list[FieldZone]:
    if not field.operation_zones:
        synthesize_scout_outputs(field)
    return [
        FieldZone(
            id=zone.id,
            center=zone.center,
            area_ha=zone.area_hectares,
            remaining_area_ha=zone.remaining_area_hectares,
            risk=zone.risk or min(1.0, field.terrain_complexity * 0.55 + field.obstacle_density * 0.45),
            terrain_complexity=field.terrain_complexity,
            obstacle_density=field.obstacle_density,
            completed=zone.remaining_area_hectares <= 0.001,
        )
        for zone in field.operation_zones
    ]


def field_to_depot_candidates(field: FieldMap) -> list[DepotCandidate]:
    if not field.candidate_depot_points:
        synthesize_scout_outputs(field)
    return [
        DepotCandidate(
            id=idx + 1,
            point=point,
            road_cost=_road_cost(point, field),
            terrain_risk=field.terrain_complexity * 0.5 + field.obstacle_density * 0.5,
            communication_score=_quality_score(point, center=field.boundary_center, soft_radius_m=900.0),
            rtk_score=_quality_score(point, center=field.boundary_center, soft_radius_m=1000.0),
        )
        for idx, point in enumerate(field.candidate_depot_points)
    ]


def opt_zone_to_operation_zone(zone: FieldZone) -> OperationZone:
    return OperationZone(
        id=zone.id,
        center=zone.center,
        area_hectares=zone.area_ha,
        treated_area_hectares=max(0.0, zone.area_ha - zone.remaining_area_ha),
        scanned=True,
    )


def _road_cost(point: Point, field: FieldMap) -> float:
    if not field.road_access_points:
        return 1000.0
    road_distance = min(point.distance_to(road) for road in field.road_access_points)
    block_roundtrip_cost = 0.0
    for block in field.field_blocks:
        if not block.selected:
            continue
        # Larger blocks have more repeated sorties, so depot placement should
        # account for weighted out-and-back flight cost.
        block_roundtrip_cost += point.distance_to(block.center) * 2.0 * max(1.0, block.area_hectares / 10.0)
    return road_distance + block_roundtrip_cost / 20.0


def _quality_score(point: Point, center: Point, soft_radius_m: float) -> float:
    distance = point.distance_to(center)
    return max(0.25, min(1.0, 1.0 - distance / (soft_radius_m * 2.5)))

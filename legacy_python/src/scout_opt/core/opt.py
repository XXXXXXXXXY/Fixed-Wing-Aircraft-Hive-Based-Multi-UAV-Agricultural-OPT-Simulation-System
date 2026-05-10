from __future__ import annotations

from scout_opt.core.models import Drone, FieldMap, Mothership, Point


def communication_cost(point: Point) -> float:
    return max(0.0, abs(point.x) + abs(point.y) - 800.0) / 100.0


def rtk_cost(point: Point) -> float:
    return 0.4 if abs(point.x) > 900.0 or abs(point.y) > 900.0 else 0.0


def distance_cost(point: Point, field: FieldMap) -> float:
    return point.distance_to(field.boundary_center) / 100.0


def road_access_cost(point: Point, road_access_points: list[Point]) -> float:
    if not road_access_points:
        return 10.0
    return min(point.distance_to(road) for road in road_access_points) / 120.0


def terrain_risk_cost(point: Point, field: FieldMap) -> float:
    return field.terrain_complexity * 4.0 + field.obstacle_density * 3.0


def drone_empty_flight_cost(point: Point, drones: list[Drone]) -> float:
    if not drones:
        return 0.0
    return sum(drone.position.distance_to(point) for drone in drones) / len(drones) / 120.0


def expected_recharge_queue_cost(active_drone_count: int, chargers: int) -> float:
    if chargers <= 0:
        return 999.0
    pressure = max(0, active_drone_count - chargers)
    return pressure * 1.5


def score_depot_point(point: Point, field: FieldMap, drones: list[Drone], chargers: int) -> float:
    return (
        distance_cost(point, field)
        + road_access_cost(point, field.road_access_points)
        + terrain_risk_cost(point, field)
        + communication_cost(point)
        + rtk_cost(point)
        + drone_empty_flight_cost(point, drones)
        + expected_recharge_queue_cost(len(drones), chargers)
    )


def choose_best_depot_point(field: FieldMap, drones: list[Drone], chargers: int) -> Point:
    candidates = field.candidate_depot_points or [field.boundary_center]
    return min(candidates, key=lambda point: score_depot_point(point, field, drones, chargers))


def rank_depot_points(field: FieldMap, drones: list[Drone], chargers: int) -> list[Point]:
    candidates = field.candidate_depot_points or [field.boundary_center]
    return sorted(candidates, key=lambda point: score_depot_point(point, field, drones, chargers))


def estimate_required_depot_count(field: FieldMap, effective_radius_m: float = 550.0) -> int:
    field_side_m = (field.area_hectares * 10_000.0) ** 0.5
    if field_side_m <= effective_radius_m * 1.4:
        return 1
    if field_side_m <= effective_radius_m * 2.4:
        return 2
    return 3


def plan_depot_sequence(field: FieldMap, drones: list[Drone], chargers: int) -> list[Point]:
    count = estimate_required_depot_count(field)
    ranked = rank_depot_points(field, drones, chargers)
    return ranked[:count]


def should_relocate_for_rolling_transition(
    mothership: Mothership,
    next_point: Point,
    remaining_cleanup_seconds: float,
) -> bool:
    move_seconds = mothership.position.distance_to(next_point) / mothership.move_speed_mps
    return move_seconds <= remaining_cleanup_seconds and mothership.position.distance_to(next_point) > 80.0


def total_assignment_cost(
    drone: Drone,
    depot: Point,
    field: FieldMap,
    work_time_seconds: float,
    charging_wait_seconds: float,
    refill_wait_seconds: float,
) -> float:
    empty_flight_time = drone.position.distance_to(field.boundary_center) / 12.0
    return_time = field.boundary_center.distance_to(depot) / 12.0
    risk_penalty = (field.terrain_complexity + field.obstacle_density) * 120.0
    return (
        work_time_seconds
        + empty_flight_time
        + return_time
        + charging_wait_seconds
        + refill_wait_seconds
        + risk_penalty
        + communication_cost(depot) * 60.0
        + rtk_cost(depot) * 60.0
        + terrain_risk_cost(depot, field) * 20.0
    )

from __future__ import annotations

from dataclasses import dataclass

from scout_opt.core.models import Drone, DroneSpec, FieldMap, FieldTask, Mothership, Point, TaskStatus
from scout_opt.core.resources import dynamic_remaining_capacity_area, estimate_return_energy


@dataclass(frozen=True)
class RadiusPolicy:
    base_radius_m: float = 520.0
    max_radius_m: float = 900.0
    min_task_area_ha: float = 0.05
    assist_min_capacity_ha: float = 0.35
    assist_gain_threshold: float = 0.15
    relocation_min_new_tasks: int = 2
    relocation_radius_gain: int = 2


@dataclass(frozen=True)
class TaskScore:
    task: FieldTask
    score: float
    radius_m: float
    capacity_ha: float


def working_radius_for_depot(
    mothership: Mothership,
    drones: list[Drone],
    spec: DroneSpec,
    policy: RadiusPolicy = RadiusPolicy(),
) -> float:
    capable = [
        dynamic_remaining_capacity_area(drone, mothership.position, spec)
        for drone in drones
        if drone.battery > 0.25 and drone.chemical > 0.1
    ]
    if not capable:
        return policy.base_radius_m
    avg_capacity = sum(capable) / len(capable)
    queue_pressure = max(0, len(drones) - mothership.fast_chargers) * 8.0
    radius = policy.base_radius_m + avg_capacity * 28.0 - queue_pressure
    return max(policy.base_radius_m * 0.65, min(policy.max_radius_m, radius))


def tasks_inside_radius(
    field: FieldMap,
    depot: Point,
    radius_m: float,
    include_assigned: bool = False,
) -> list[FieldTask]:
    allowed_status = {TaskStatus.PENDING}
    if include_assigned:
        allowed_status |= {TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}
    return [
        task
        for task in field.tasks
        if task.status in allowed_status
        and task.remaining_area_hectares > 0
        and depot.distance_to(task.center) <= radius_m
    ]


def score_task_for_drone(
    task: FieldTask,
    drone: Drone,
    mothership: Mothership,
    spec: DroneSpec,
    queue_pressure: float,
    policy: RadiusPolicy = RadiusPolicy(),
) -> TaskScore | None:
    radius_m = working_radius_for_depot(mothership, [drone], spec, policy)
    distance_to_depot = mothership.position.distance_to(task.center)
    if distance_to_depot > min(policy.max_radius_m, radius_m * 1.25):
        return None

    capacity = dynamic_remaining_capacity_area(drone, mothership.position, spec)
    if capacity < policy.min_task_area_ha:
        return None

    empty_distance = drone.position.distance_to(task.center)
    return_distance = task.center.distance_to(mothership.position)
    return_energy = estimate_return_energy(drone, mothership.position, spec)
    over_capacity_penalty = max(0.0, task.remaining_area_hectares - capacity) * 60.0
    underuse_penalty = max(0.0, capacity - task.remaining_area_hectares) * 0.8
    risk_penalty = task.risk * 45.0
    radius_penalty = max(0.0, distance_to_depot - radius_m) / 8.0
    score = (
        empty_distance / 12.0
        + return_distance / 14.0
        + return_energy * 250.0
        + over_capacity_penalty
        + underuse_penalty
        + risk_penalty
        + queue_pressure
        + radius_penalty
        - task.priority * 8.0
    )
    return TaskScore(task=task, score=score, radius_m=radius_m, capacity_ha=capacity)


def choose_radius_task_for_drone(
    field: FieldMap,
    drone: Drone,
    mothership: Mothership,
    spec: DroneSpec,
    queue_pressure: float,
    policy: RadiusPolicy = RadiusPolicy(),
) -> TaskScore | None:
    radius_m = working_radius_for_depot(mothership, [drone], spec, policy)
    candidates = tasks_inside_radius(field, mothership.position, radius_m)
    if not candidates:
        candidates = tasks_inside_radius(field, mothership.position, policy.max_radius_m)
    scored = [
        score
        for task in candidates
        if (score := score_task_for_drone(task, drone, mothership, spec, queue_pressure, policy)) is not None
    ]
    if not scored:
        return None
    return min(scored, key=lambda item: item.score)


def choose_assist_task_for_drone(
    field: FieldMap,
    drone: Drone,
    mothership: Mothership,
    spec: DroneSpec,
    active_drones: list[Drone],
    policy: RadiusPolicy = RadiusPolicy(),
) -> TaskScore | None:
    capacity = dynamic_remaining_capacity_area(drone, mothership.position, spec)
    if capacity < policy.assist_min_capacity_ha:
        return None
    radius = working_radius_for_depot(mothership, active_drones + [drone], spec, policy)
    candidates = tasks_inside_radius(field, mothership.position, radius, include_assigned=True)
    scored: list[TaskScore] = []
    for task in candidates:
        if task.assigned_drone_id == drone.id:
            continue
        score = score_task_for_drone(task, drone, mothership, spec, queue_pressure=0.0, policy=policy)
        if score is None:
            continue
        remaining_ratio = task.remaining_area_hectares / max(0.001, task.area_hectares)
        assist_gain = min(capacity, task.remaining_area_hectares) / max(0.001, task.remaining_area_hectares)
        if remaining_ratio >= 0.2 and assist_gain >= policy.assist_gain_threshold:
            scored.append(score)
    if not scored:
        return None
    return min(scored, key=lambda item: item.score)


def should_switch_mothership_point(
    field: FieldMap,
    mothership: Mothership,
    drones: list[Drone],
    spec: DroneSpec,
    next_point: Point,
    cleanup_eta_seconds: float,
    policy: RadiusPolicy = RadiusPolicy(),
) -> bool:
    current_radius = working_radius_for_depot(mothership, drones, spec, policy)
    current_tasks = tasks_inside_radius(field, mothership.position, current_radius, include_assigned=True)
    next_tasks = tasks_inside_radius(field, next_point, policy.base_radius_m, include_assigned=True)
    move_seconds = mothership.position.distance_to(next_point) / max(0.001, mothership.move_speed_mps)
    if len(next_tasks) < policy.relocation_min_new_tasks:
        return False
    if len(next_tasks) - len(current_tasks) < policy.relocation_radius_gain and current_tasks:
        return False
    return move_seconds <= cleanup_eta_seconds

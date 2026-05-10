from __future__ import annotations

from scout_opt.core.models import Drone, FieldBlock, FieldMap, FieldTask, OperationZone, Point, TaskStatus


def build_coverage_tasks(field: FieldMap, max_task_area_hectares: float = 4.0) -> None:
    if field.tasks:
        return
    if not field.operation_zones and field.field_blocks:
        field.operation_zones = build_operation_zones_from_blocks(field.field_blocks)
    if not field.operation_zones:
        field.operation_zones = [
            OperationZone(id=1, center=field.boundary_center, area_hectares=field.area_hectares, scanned=True)
        ]

    tasks: list[FieldTask] = []
    task_id = 1
    for zone in field.operation_zones:
        pieces = max(1, round(zone.area_hectares / max_task_area_hectares))
        piece_area = zone.area_hectares / pieces
        for idx in range(pieces):
            offset = (idx - (pieces - 1) / 2.0) * 35.0
            center = Point(zone.center.x + offset, zone.center.y + ((idx % 2) * 35.0))
            risk = min(1.0, field.terrain_complexity * 0.55 + field.obstacle_density * 0.45)
            tasks.append(
                FieldTask(
                    id=task_id,
                    zone_id=zone.id,
                    block_id=zone.block_id,
                    center=center,
                    area_hectares=piece_area,
                    remaining_area_hectares=piece_area,
                    priority=1.0 + risk,
                    risk=risk,
                )
            )
            task_id += 1
    field.tasks = tasks


def build_operation_zones_from_blocks(
    blocks: list[FieldBlock],
    target_zone_area_hectares: float = 8.0,
) -> list[OperationZone]:
    zones: list[OperationZone] = []
    zone_id = 1
    for block in blocks:
        if not block.selected:
            continue
        zone_count = max(1, round(block.area_hectares / target_zone_area_hectares))
        zone_area = block.area_hectares / zone_count
        cols = max(1, round(zone_count ** 0.5))
        spacing = max(90.0, (block.area_hectares * 10_000.0 / zone_count) ** 0.5)
        for idx in range(zone_count):
            row = idx // cols
            col = idx % cols
            x_offset = (col - (cols - 1) / 2.0) * spacing
            y_offset = (row - ((zone_count - 1) // cols) / 2.0) * spacing
            zones.append(
                OperationZone(
                    id=zone_id,
                    name=f"{block.name}-zone-{idx + 1}",
                    center=Point(block.center.x + x_offset, block.center.y + y_offset),
                    area_hectares=zone_area,
                    scanned=True,
                    risk=block.risk,
                    block_id=block.id,
                    notes=list(block.notes),
                )
            )
            zone_id += 1
    return zones


def open_tasks(field: FieldMap) -> list[FieldTask]:
    return [
        task
        for task in field.tasks
        if task.status in {TaskStatus.PENDING, TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}
        and task.remaining_area_hectares > 0
    ]


def pending_tasks(field: FieldMap) -> list[FieldTask]:
    return [
        task
        for task in field.tasks
        if task.status == TaskStatus.PENDING and task.remaining_area_hectares > 0
    ]


def task_by_id(field: FieldMap, task_id: int | None) -> FieldTask | None:
    if task_id is None:
        return None
    for task in field.tasks:
        if task.id == task_id:
            return task
    return None


def choose_best_task(field: FieldMap, drone: Drone, depot: Point, queue_pressure: float) -> FieldTask | None:
    candidates = pending_tasks(field)
    if not candidates:
        return None

    def score(task: FieldTask) -> float:
        drone_distance = drone.position.distance_to(task.center) / 100.0
        depot_distance = depot.distance_to(task.center) / 120.0
        risk_penalty = task.risk * 4.0
        urgency_bonus = task.priority * 1.6
        return drone_distance + depot_distance + risk_penalty + queue_pressure - urgency_bonus

    return min(candidates, key=score)


def mark_task_assigned(task: FieldTask, drone: Drone, now_seconds: float) -> None:
    task.status = TaskStatus.IN_PROGRESS
    task.assigned_drone_id = drone.id
    task.started_at_seconds = task.started_at_seconds if task.started_at_seconds is not None else now_seconds
    drone.assigned_task_id = task.id
    drone.assigned_area_hectares = task.remaining_area_hectares
    drone.target = task.center


def release_task(field: FieldMap, drone: Drone) -> None:
    task = task_by_id(field, drone.assigned_task_id)
    if task and task.status != TaskStatus.DONE:
        task.status = TaskStatus.PENDING
        task.assigned_drone_id = None
    drone.assigned_task_id = None
    drone.assigned_area_hectares = 0.0


def apply_task_progress(field: FieldMap, task: FieldTask, area_done: float, now_seconds: float) -> None:
    actual = min(task.remaining_area_hectares, area_done)
    task.remaining_area_hectares = max(0.0, task.remaining_area_hectares - actual)
    field.treated_area_hectares = min(field.area_hectares, field.treated_area_hectares + actual)
    for zone in field.operation_zones:
        if zone.id == task.zone_id:
            zone.treated_area_hectares = min(zone.area_hectares, zone.treated_area_hectares + actual)
            break
    if task.remaining_area_hectares <= 0.001:
        task.status = TaskStatus.DONE
        task.assigned_drone_id = None
        task.completed_at_seconds = now_seconds

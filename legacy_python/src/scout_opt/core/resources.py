from __future__ import annotations

from scout_opt.core.charging import required_charge_for_next_task
from scout_opt.core.models import Drone, DroneSpec, DroneState, FieldMap, Mothership, Point


def estimate_return_energy(drone: Drone, depot: Point, spec: DroneSpec) -> float:
    distance_km = drone.position.distance_to(depot) / 1000.0
    return distance_km * spec.battery_drain_per_km_empty


def work_energy_for_area(area_hectares: float, spec: DroneSpec) -> float:
    work_hours = area_hectares / spec.spray_rate_hectares_per_hour
    return work_hours * spec.battery_drain_per_hour_work


def dynamic_remaining_capacity_area(
    drone: Drone,
    depot: Point,
    spec: DroneSpec,
) -> float:
    return_energy = estimate_return_energy(drone, depot, spec)
    available_battery = max(0.0, drone.battery - return_energy - spec.safety_battery_margin)
    battery_area = (
        available_battery / spec.battery_drain_per_hour_work * spec.spray_rate_hectares_per_hour
        if spec.battery_drain_per_hour_work > 0
        else 0.0
    )
    chemical_area = drone.chemical / spec.chemical_per_hectare if spec.chemical_per_hectare > 0 else 0.0
    capacity = max(0.0, min(battery_area, chemical_area))
    drone.return_energy_required = return_energy
    drone.remaining_capacity_area = capacity
    return capacity


def target_charge_for_area(
    area_hectares: float,
    drone: Drone,
    depot: Point,
    spec: DroneSpec,
) -> float:
    task_energy = work_energy_for_area(area_hectares, spec)
    return_energy = estimate_return_energy(drone, depot, spec)
    return required_charge_for_next_task(
        task_energy=task_energy,
        return_energy=return_energy,
        safety_margin=spec.safety_battery_margin,
    )


def update_dynamic_resource_model(drones: list[Drone], mothership: Mothership, spec: DroneSpec) -> None:
    for drone in drones:
        dynamic_remaining_capacity_area(drone, mothership.position, spec)


def needs_recall(drone: Drone, mothership: Mothership, spec: DroneSpec) -> bool:
    return_energy = estimate_return_energy(drone, mothership.position, spec)
    if drone.state in {DroneState.SCOUTING, DroneState.WORKING, DroneState.ASSISTING, DroneState.CLEANUP}:
        return drone.battery <= return_energy + spec.safety_battery_margin or drone.chemical < 0.05
    return False


def estimate_remaining_work_time_seconds(field: FieldMap, drones: list[Drone], spec: DroneSpec) -> float:
    active = [
        drone
        for drone in drones
        if drone.state in {DroneState.WORKING, DroneState.ASSISTING, DroneState.CLEANUP}
    ]
    if not active:
        return float("inf") if field.remaining_area_hectares > 0 else 0.0
    total_rate = len(active) * spec.spray_rate_hectares_per_hour
    return field.remaining_area_hectares / total_rate * 3600.0

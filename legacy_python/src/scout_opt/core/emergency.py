from __future__ import annotations

from dataclasses import dataclass, field

from scout_opt.core.models import Drone, DroneState, FieldMap, Mothership, Point, WeatherSeverity
from scout_opt.core.weather import design_emergency_landing_spots


ACTIVE_FLIGHT_STATES = {
    DroneState.SCOUTING,
    DroneState.WORKING,
    DroneState.ASSISTING,
    DroneState.CLEANUP,
    DroneState.PREDEPLOY,
}


@dataclass(frozen=True)
class RecoveryDecision:
    returning_drone_ids: list[int] = field(default_factory=list)
    emergency_landing_assignments: dict[int, Point] = field(default_factory=dict)
    reason: str = ""


def build_weather_recovery_decision(
    field: FieldMap,
    mothership: Mothership,
    drones: list[Drone],
) -> RecoveryDecision:
    severity = mothership.weather.severity
    active = [drone for drone in drones if drone.state in ACTIVE_FLIGHT_STATES]
    if severity not in {WeatherSeverity.SEVERE, WeatherSeverity.EMERGENCY} or not active:
        return RecoveryDecision()

    active.sort(key=lambda drone: drone.position.distance_to(mothership.position))
    if severity == WeatherSeverity.SEVERE:
        return RecoveryDecision(
            returning_drone_ids=[drone.id for drone in active],
            reason="severe weather normal recovery by nearest-first OPT",
        )

    spots = ensure_emergency_landing_spots(field, mothership.position, len(drones))
    returning: list[int] = []
    landing: dict[int, Point] = {}
    for idx, drone in enumerate(active):
        if can_reach_mothership_before_weather(drone, mothership):
            returning.append(drone.id)
        else:
            landing[drone.id] = spots[idx % len(spots)]

    return RecoveryDecision(
        returning_drone_ids=returning,
        emergency_landing_assignments=landing,
        reason="emergency weather recovery with separated landing spots",
    )


def can_reach_mothership_before_weather(drone: Drone, mothership: Mothership) -> bool:
    # A practical local heuristic: in emergency weather, only drones already near
    # the mothership should attempt return; far drones are safer in separated spots.
    distance_m = drone.position.distance_to(mothership.position)
    return distance_m <= 260.0 and drone.battery > drone.return_energy_required + 0.12


def ensure_emergency_landing_spots(field: FieldMap, center: Point, drone_count: int) -> list[Point]:
    if len(field.emergency_landing_spots) >= drone_count:
        return field.emergency_landing_spots
    field.emergency_landing_spots = design_emergency_landing_spots(center, drone_count)
    return field.emergency_landing_spots

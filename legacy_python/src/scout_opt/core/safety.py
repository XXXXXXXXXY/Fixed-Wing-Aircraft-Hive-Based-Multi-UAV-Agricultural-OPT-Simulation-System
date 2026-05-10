from __future__ import annotations

from dataclasses import dataclass

from scout_opt.core.models import Drone, DroneState, FieldMap, Mothership


@dataclass(frozen=True)
class SafetyReport:
    ok: bool
    reasons: list[str]


def check_safety_constraints(field: FieldMap, mothership: Mothership, drones: list[Drone]) -> SafetyReport:
    reasons: list[str] = []
    if not mothership.systems.weather_check:
        reasons.append("weather check unavailable")
    if not mothership.systems.communication_check:
        reasons.append("communication check unavailable")
    if not mothership.systems.rtk_check:
        reasons.append("rtk check unavailable")
    if field.terrain_complexity > 0.85:
        reasons.append("terrain complexity is high")
    if field.obstacle_density > 0.75:
        reasons.append("obstacle density is high")
    if not mothership.weather.flight_allowed:
        reasons.append("weather flight constraints exceeded")
    elif not mothership.weather.spray_allowed:
        reasons.append("weather spray constraints exceeded")
    for drone in drones:
        if drone.state in {DroneState.SCOUTING, DroneState.WORKING, DroneState.ASSISTING} and drone.battery < 0.12:
            reasons.append(f"drone {drone.id} battery critically low")
    return SafetyReport(ok=not reasons, reasons=reasons)

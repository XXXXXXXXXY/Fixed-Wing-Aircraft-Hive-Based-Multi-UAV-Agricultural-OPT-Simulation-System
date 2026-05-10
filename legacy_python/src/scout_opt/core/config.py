from __future__ import annotations

from dataclasses import dataclass

from scout_opt.core.models import Drone, DroneSpec, FieldMap, Mothership, Point


@dataclass
class SimulationConfig:
    field: FieldMap
    mothership: Mothership
    drones: list[Drone]
    drone_spec: DroneSpec


def default_simulation_config(
    drone_count: int = 8,
    field_area_hectares: float = 72.0,
    terrain_complexity: float = 0.45,
    obstacle_density: float = 0.25,
) -> SimulationConfig:
    mothership = Mothership()
    drones = [
        Drone(id=i + 1, position=Point(mothership.position.x, mothership.position.y))
        for i in range(drone_count)
    ]
    field = FieldMap(
        area_hectares=field_area_hectares,
        terrain_complexity=terrain_complexity,
        obstacle_density=obstacle_density,
    )
    return SimulationConfig(
        field=field,
        mothership=mothership,
        drones=drones,
        drone_spec=DroneSpec(),
    )

from __future__ import annotations

from scout_opt.adapters.base import FleetAdapter
from scout_opt.core.engine import SimulationEngine
from scout_opt.core.models import DroneState, Point, TelemetrySnapshot


class InMemorySimulationAdapter(FleetAdapter):
    """Adapter for the local Python simulation engine."""

    def __init__(self, engine: SimulationEngine) -> None:
        self.engine = engine

    async def connect(self) -> None:
        return None

    async def telemetry_all(self) -> list[TelemetrySnapshot]:
        return self.engine.telemetry()

    async def set_state_hint(self, drone_id: int, state: DroneState) -> None:
        for drone in self.engine.config.drones:
            if drone.id == drone_id:
                drone.state = state
                return

    async def goto(self, drone_id: int, point: Point, altitude_m: float) -> None:
        for drone in self.engine.config.drones:
            if drone.id == drone_id:
                drone.target = point
                return

    async def return_to_depot(self, drone_id: int, depot: Point) -> None:
        for drone in self.engine.config.drones:
            if drone.id == drone_id:
                drone.target = depot
                drone.state = DroneState.RETURNING
                return

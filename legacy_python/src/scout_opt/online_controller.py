from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from scout_opt.adapters.base import FleetAdapter
from scout_opt.adapters.mavsdk_adapter import MavsdkFleetAdapter, MavsdkVehicleEndpoint
from scout_opt.core.config import SimulationConfig, default_simulation_config
from scout_opt.core.coverage import generate_lawnmower_path
from scout_opt.core.geo import GeoOrigin, local_to_latlon
from scout_opt.core.models import DroneState, FieldMap, Point, TelemetrySnapshot
from scout_opt.core.opt_bridge import (
    field_to_depot_candidates,
    field_to_opt_zones,
    mothership_to_opt_config,
)
from scout_opt.core.opt_engine import AgriculturalOptEngine, MissionPhase, OptPlan
from scout_opt.core.scout import synthesize_scout_outputs
from scout_opt.core.tasks import build_coverage_tasks


@dataclass(frozen=True)
class SitlConfig:
    endpoints: list[MavsdkVehicleEndpoint]
    origin: GeoOrigin | None = None


def load_sitl_config(path: str | Path) -> SitlConfig:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    endpoints = [
        MavsdkVehicleEndpoint(
            drone_id=int(item["drone_id"]),
            system_address=str(item["system_address"]),
        )
        for item in data["vehicles"]
    ]
    origin_data = data.get("origin")
    origin = None
    if origin_data:
        origin = GeoOrigin(
            latitude_deg=float(origin_data["latitude_deg"]),
            longitude_deg=float(origin_data["longitude_deg"]),
        )
    return SitlConfig(endpoints=endpoints, origin=origin)


class AgriculturalOnlineController:
    def __init__(
        self,
        adapter: FleetAdapter,
        config: SimulationConfig,
        origin: GeoOrigin | None = None,
    ) -> None:
        self.adapter = adapter
        self.config = config
        self.origin = origin
        self.opt = AgriculturalOptEngine(mothership_to_opt_config(config.mothership))

    async def build_plan(self, scout_finished: bool) -> OptPlan:
        ensure_field_ready(self.config.field, scout_finished=scout_finished)
        telemetry = await self.adapter.telemetry_all()
        zones = field_to_opt_zones(self.config.field)
        depot_candidates = field_to_depot_candidates(self.config.field)
        return self.opt.build_opt_plan(
            telemetry=telemetry,
            zones=zones,
            depot_candidates=depot_candidates,
            scout_finished=scout_finished,
        )

    async def execute_plan(self, plan: OptPlan, altitude_m: float = 20.0) -> None:
        if plan.phase == MissionPhase.SCOUTING:
            for drone_id in plan.scout_drone_ids:
                await self.adapter.set_state_hint(drone_id, DroneState.SCOUTING)
                if plan.depot is not None:
                    await self.adapter.goto(drone_id, self._to_adapter_point(plan.depot.point), altitude_m)
            return

        if plan.depot is not None:
            await self._execute_assignments(plan, altitude_m=altitude_m)

    async def _execute_assignments(self, plan: OptPlan, altitude_m: float) -> None:
        task_by_zone = {
            task.zone_id: task
            for task in self.config.field.tasks
            if task.remaining_area_hectares > 0
        }
        for drone_id, zone_id in plan.assignments.items():
            task = task_by_zone.get(zone_id)
            if task is None:
                continue
            await self.adapter.set_state_hint(drone_id, DroneState.WORKING)
            path = generate_lawnmower_path(task)
            if path:
                await self.adapter.goto(drone_id, self._to_adapter_point(path[0]), altitude_m)

    def _to_adapter_point(self, point: Point) -> Point:
        if self.origin is None:
            return point
        latitude, longitude = local_to_latlon(point, self.origin)
        return Point(x=longitude, y=latitude)


def ensure_field_ready(field: FieldMap, scout_finished: bool) -> None:
    if scout_finished or not field.scanned:
        synthesize_scout_outputs(field)
        build_coverage_tasks(field)


def build_mavsdk_controller(
    sitl_config_path: str | Path,
    simulation_config: SimulationConfig | None = None,
) -> AgriculturalOnlineController:
    sitl_config = load_sitl_config(sitl_config_path)
    adapter = MavsdkFleetAdapter(sitl_config.endpoints)
    return AgriculturalOnlineController(
        adapter=adapter,
        config=simulation_config or default_simulation_config(),
        origin=sitl_config.origin,
    )


def synthetic_telemetry(config: SimulationConfig) -> list[TelemetrySnapshot]:
    return [
        TelemetrySnapshot(
            drone_id=drone.id,
            state=drone.state,
            battery=drone.battery,
            chemical=drone.chemical,
            position=drone.position,
            flight_mode="DRY_RUN",
        )
        for drone in config.drones
    ]

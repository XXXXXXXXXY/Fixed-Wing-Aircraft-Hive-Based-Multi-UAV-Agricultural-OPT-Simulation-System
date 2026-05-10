from __future__ import annotations

from dataclasses import dataclass

from scout_opt.adapters.base import FleetAdapter
from scout_opt.core.models import DroneState, Point, TelemetrySnapshot


@dataclass(frozen=True)
class MavsdkVehicleEndpoint:
    system_address: str
    drone_id: int


class MavsdkFleetAdapter(FleetAdapter):
    """MAVSDK-Python adapter skeleton for ArduPilot/PX4 SITL.

    This class intentionally avoids importing mavsdk at module import time, so the
    pure Python simulation works without optional SITL dependencies installed.
    """

    def __init__(self, endpoints: list[MavsdkVehicleEndpoint]) -> None:
        self.endpoints = endpoints
        self.systems: dict[int, object] = {}
        self.state_hints: dict[int, DroneState] = {}

    async def connect(self) -> None:
        try:
            from mavsdk import System
        except ImportError as exc:
            raise RuntimeError(
                "MAVSDK is not installed. Install with `pip install -e .[sitl]` before using SITL."
            ) from exc

        for endpoint in self.endpoints:
            system = System()
            await system.connect(system_address=endpoint.system_address)
            self.systems[endpoint.drone_id] = system

    async def telemetry_all(self) -> list[TelemetrySnapshot]:
        snapshots: list[TelemetrySnapshot] = []
        for drone_id, system in self.systems.items():
            battery = await self._first(system.telemetry.battery())
            position = await self._first(system.telemetry.position())
            state = self.state_hints.get(drone_id, DroneState.IDLE)
            snapshots.append(
                TelemetrySnapshot(
                    drone_id=drone_id,
                    state=state,
                    battery=max(0.0, min(1.0, battery.remaining_percent)),
                    chemical=1.0,
                    position=Point(position.longitude_deg, position.latitude_deg),
                    flight_mode="MAVSDK",
                    gps_ok=True,
                )
            )
        return snapshots

    async def set_state_hint(self, drone_id: int, state: DroneState) -> None:
        self.state_hints[drone_id] = state

    async def goto(self, drone_id: int, point: Point, altitude_m: float) -> None:
        system = self.systems[drone_id]
        await system.action.goto_location(
            latitude_deg=point.y,
            longitude_deg=point.x,
            absolute_altitude_m=altitude_m,
            yaw_deg=0.0,
        )

    async def return_to_depot(self, drone_id: int, depot: Point) -> None:
        await self.goto(drone_id, depot, altitude_m=20.0)
        self.state_hints[drone_id] = DroneState.RETURNING

    async def _first(self, stream: object) -> object:
        async for item in stream:
            return item
        raise RuntimeError("Telemetry stream ended without a value.")

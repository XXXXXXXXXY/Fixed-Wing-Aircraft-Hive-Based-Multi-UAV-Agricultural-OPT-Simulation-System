from __future__ import annotations

from abc import ABC, abstractmethod

from scout_opt.core.models import DroneState, Point, TelemetrySnapshot


class FleetAdapter(ABC):
    """Boundary between scheduler logic and a simulator or real flight stack."""

    @abstractmethod
    async def connect(self) -> None:
        """Open connections to every vehicle."""

    @abstractmethod
    async def telemetry_all(self) -> list[TelemetrySnapshot]:
        """Read current fleet telemetry."""

    @abstractmethod
    async def set_state_hint(self, drone_id: int, state: DroneState) -> None:
        """Expose scheduler state to the adapter for logging or command mapping."""

    @abstractmethod
    async def goto(self, drone_id: int, point: Point, altitude_m: float) -> None:
        """Command a vehicle to fly to a point."""

    @abstractmethod
    async def return_to_depot(self, drone_id: int, depot: Point) -> None:
        """Recall a vehicle to the current mothership/depot point."""

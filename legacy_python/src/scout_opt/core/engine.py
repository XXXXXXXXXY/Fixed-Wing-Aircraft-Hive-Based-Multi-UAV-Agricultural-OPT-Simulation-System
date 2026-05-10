from __future__ import annotations

from scout_opt.core.charging import ChargingQueue
from scout_opt.core.config import SimulationConfig
from scout_opt.core.models import SimulationEvent, SimulationResult, TelemetrySnapshot
from scout_opt.core.scheduler import MissionScheduler
from scout_opt.core.weather import MothershipWeatherStation


class SimulationEngine:
    def __init__(self, config: SimulationConfig, dt_seconds: float = 60.0) -> None:
        self.config = config
        self.dt_seconds = dt_seconds
        self.now_seconds = 0.0
        self.events: list[SimulationEvent] = []
        self.charging_queue = ChargingQueue(
            fast_chargers=config.mothership.fast_chargers,
            refill_ports=config.mothership.refill_ports,
            storage_slots=config.mothership.drone_slots,
            charging_slots=[None] * config.mothership.fast_chargers,
            refill_slots=[None] * config.mothership.refill_ports,
        )
        self.scheduler = MissionScheduler(self.charging_queue)
        self.weather_station = MothershipWeatherStation()

    def telemetry(self) -> list[TelemetrySnapshot]:
        return [
            TelemetrySnapshot(
                drone_id=drone.id,
                state=drone.state,
                battery=drone.battery,
                chemical=drone.chemical,
                position=drone.position,
            )
            for drone in self.config.drones
        ]

    def step(self) -> None:
        if self.weather_station.should_update(self.now_seconds):
            self.config.mothership.weather = self.weather_station.update(self.now_seconds)
            self.events.append(
                SimulationEvent(
                    self.now_seconds,
                    "weather",
                    (
                        f"weather updated wind={self.config.mothership.weather.wind_speed_mps:.1f}m/s "
                        f"gust={self.config.mothership.weather.wind_gust_mps:.1f}m/s "
                        f"humidity={self.config.mothership.weather.humidity:.2f}"
                    ),
                )
            )
        self.events.extend(
            self.scheduler.step(
                now_seconds=self.now_seconds,
                dt_seconds=self.dt_seconds,
                field=self.config.field,
                mothership=self.config.mothership,
                drones=self.config.drones,
                spec=self.config.drone_spec,
            )
        )
        self.now_seconds += self.dt_seconds

    def run(self, max_steps: int = 240) -> SimulationResult:
        for _ in range(max_steps):
            if self.config.field.remaining_area_hectares <= 0.001:
                break
            self.step()
        completed = self.config.field.remaining_area_hectares <= 0.001
        if completed:
            self.config.field.treated_area_hectares = self.config.field.area_hectares
        return SimulationResult(
            completed=completed,
            elapsed_seconds=self.now_seconds,
            treated_area_hectares=self.config.field.treated_area_hectares,
            mothership_position=self.config.mothership.position,
            events=self.events,
        )

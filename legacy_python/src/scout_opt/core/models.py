from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import hypot
from typing import Optional


class DroneState(str, Enum):
    IDLE = "idle"
    SCOUTING = "scouting"
    WORKING = "working"
    ASSISTING = "assisting"
    RETURNING = "returning"
    CHARGING = "charging"
    REFILLING = "refilling"
    STANDBY = "standby"
    ENTERING_NEW_ZONE = "entering_new_zone"
    CLEANUP = "cleanup"
    PREDEPLOY = "predeploy"
    EMERGENCY_LANDING = "emergency_landing"
    LANDED = "landed"


class TaskStatus(str, Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    DONE = "done"
    BLOCKED = "blocked"


class WeatherSeverity(str, Enum):
    NORMAL = "normal"
    WATCH = "watch"
    WARNING = "warning"
    SEVERE = "severe"
    EMERGENCY = "emergency"


@dataclass(frozen=True)
class Point:
    x: float
    y: float

    def distance_to(self, other: "Point") -> float:
        return hypot(self.x - other.x, self.y - other.y)


@dataclass
class FieldMap:
    area_hectares: float
    terrain_complexity: float
    obstacle_density: float
    boundary_center: Point = field(default_factory=lambda: Point(0.0, 0.0))
    scanned: bool = False
    treated_area_hectares: float = 0.0
    road_access_points: list[Point] = field(default_factory=list)
    candidate_depot_points: list[Point] = field(default_factory=list)
    depot_sites: list["DepotSite"] = field(default_factory=list)
    operation_zones: list["OperationZone"] = field(default_factory=list)
    field_blocks: list["FieldBlock"] = field(default_factory=list)
    tasks: list["FieldTask"] = field(default_factory=list)
    manual_notes: list[str] = field(default_factory=list)
    obstacles: list[Point] = field(default_factory=list)
    emergency_landing_spots: list[Point] = field(default_factory=list)

    @property
    def remaining_area_hectares(self) -> float:
        return max(0.0, self.area_hectares - self.treated_area_hectares)


@dataclass
class MothershipSystems:
    weather_check: bool = True
    terrain_check: bool = True
    road_check: bool = True
    rtk_check: bool = True
    communication_check: bool = True
    obstacle_check: bool = True
    battery_health_check: bool = True
    chemical_level_check: bool = True


@dataclass
class Mothership:
    drone_slots: int = 8
    fast_chargers: int = 2
    refill_ports: int = 2
    position: Point = field(default_factory=lambda: Point(-600.0, 0.0))
    moving: bool = False
    destination: Optional[Point] = None
    move_remaining_seconds: float = 0.0
    move_speed_mps: float = 8.0
    operation_plan: list[Point] = field(default_factory=list)
    operation_plan_index: int = 0
    weather: "WeatherSnapshot" = field(default_factory=lambda: WeatherSnapshot())
    systems: MothershipSystems = field(default_factory=MothershipSystems)


@dataclass
class Drone:
    id: int
    state: DroneState = DroneState.IDLE
    battery: float = 1.0
    chemical: float = 1.0
    position: Point = field(default_factory=lambda: Point(-600.0, 0.0))
    assigned_area_hectares: float = 0.0
    remaining_capacity_area: float = 0.0
    return_energy_required: float = 0.0
    eta_seconds: Optional[float] = None
    task_elapsed_seconds: float = 0.0
    target: Optional[Point] = None
    target_charge: float = 0.8
    role_detail: str = ""
    assigned_task_id: Optional[int] = None

    def is_available_for_field_work(self) -> bool:
        return self.state in {DroneState.IDLE, DroneState.STANDBY} and self.battery > 0.35 and self.chemical > 0.2


@dataclass
class OperationZone:
    id: int
    center: Point
    area_hectares: float
    treated_area_hectares: float = 0.0
    scanned: bool = False
    name: str = ""
    risk: float = 0.0
    block_id: int | None = None
    notes: list[str] = field(default_factory=list)

    @property
    def remaining_area_hectares(self) -> float:
        return max(0.0, self.area_hectares - self.treated_area_hectares)


@dataclass
class FieldBlock:
    id: int
    name: str
    boundary_points: list[Point]
    area_hectares: float
    center: Point
    risk: float = 0.0
    selected: bool = True
    notes: list[str] = field(default_factory=list)
    candidate_depot_points: list[Point] = field(default_factory=list)

    @property
    def completed_area_hectares(self) -> float:
        return 0.0


@dataclass
class DepotSite:
    id: int
    point: Point
    usable_area_m2: float = 400.0
    road_accessible: bool = True
    slope_risk: float = 0.0
    notes: list[str] = field(default_factory=list)

    def can_deploy(self, required_area_m2: float = 180.0, max_slope_risk: float = 0.65) -> bool:
        return self.road_accessible and self.usable_area_m2 >= required_area_m2 and self.slope_risk <= max_slope_risk


@dataclass
class FieldTask:
    id: int
    zone_id: int
    block_id: int | None
    center: Point
    area_hectares: float
    remaining_area_hectares: float
    priority: float
    risk: float
    status: TaskStatus = TaskStatus.PENDING
    assigned_drone_id: Optional[int] = None
    started_at_seconds: Optional[float] = None
    completed_at_seconds: Optional[float] = None

    @property
    def progress(self) -> float:
        if self.area_hectares <= 0:
            return 1.0
        return 1.0 - max(0.0, self.remaining_area_hectares) / self.area_hectares


@dataclass(frozen=True)
class DroneSpec:
    cruise_speed_mps: float = 12.0
    scout_speed_mps: float = 6.0
    spray_rate_hectares_per_hour: float = 8.0
    scout_rate_hectares_per_hour: float = 25.0
    battery_drain_per_hour_work: float = 0.38
    battery_drain_per_hour_scout: float = 0.24
    battery_drain_per_km_empty: float = 0.025
    chemical_per_hectare: float = 0.11
    safety_battery_margin: float = 0.15


@dataclass(frozen=True)
class WeatherSnapshot:
    wind_speed_mps: float = 2.5
    wind_gust_mps: float = 4.0
    temperature_c: float = 26.0
    humidity: float = 0.55
    precipitation_mmph: float = 0.0
    visibility_m: float = 5000.0
    updated_at_seconds: float = 0.0

    @property
    def severity(self) -> WeatherSeverity:
        if self.wind_gust_mps >= 16.0 or self.precipitation_mmph >= 6.0 or self.visibility_m < 300.0:
            return WeatherSeverity.EMERGENCY
        if self.wind_gust_mps >= 13.0 or self.precipitation_mmph >= 2.0 or self.visibility_m < 800.0:
            return WeatherSeverity.SEVERE
        if self.wind_speed_mps >= 7.0 or self.wind_gust_mps >= 10.0 or self.precipitation_mmph >= 0.8:
            return WeatherSeverity.WARNING
        if self.wind_speed_mps >= 5.5 or self.wind_gust_mps >= 8.0 or self.precipitation_mmph >= 0.2 or self.humidity >= 0.9:
            return WeatherSeverity.WATCH
        return WeatherSeverity.NORMAL

    @property
    def spray_allowed(self) -> bool:
        return self.wind_speed_mps <= 7.0 and self.wind_gust_mps <= 10.0 and self.precipitation_mmph <= 0.2

    @property
    def flight_allowed(self) -> bool:
        return self.wind_gust_mps <= 13.0 and self.visibility_m >= 800.0 and self.precipitation_mmph <= 2.0


@dataclass(frozen=True)
class TelemetrySnapshot:
    drone_id: int
    state: DroneState
    battery: float
    chemical: float
    position: Point
    flight_mode: str = "SIM"
    gps_ok: bool = True


@dataclass
class SimulationEvent:
    time_seconds: float
    kind: str
    message: str


@dataclass
class SimulationResult:
    completed: bool
    elapsed_seconds: float
    treated_area_hectares: float
    mothership_position: Point
    events: list[SimulationEvent]

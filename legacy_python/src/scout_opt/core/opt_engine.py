from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from scout_opt.core.models import DroneState, Point, TelemetrySnapshot


class MissionPhase(str, Enum):
    PRE_SCOUT = "pre_scout"
    SCOUTING = "scouting"
    OPT_PLANNING = "opt_planning"
    WORKING = "working"
    CLEANUP = "cleanup"
    DEPOT_RELOCATING = "depot_relocating"
    DONE = "done"


@dataclass
class FieldZone:
    id: int
    center: Point
    area_ha: float
    remaining_area_ha: float
    risk: float = 0.0
    terrain_complexity: float = 0.0
    obstacle_density: float = 0.0
    assigned_drone_ids: list[int] = field(default_factory=list)
    completed: bool = False


@dataclass
class DepotCandidate:
    id: int
    point: Point
    road_cost: float = 0.0
    terrain_risk: float = 0.0
    communication_score: float = 1.0
    rtk_score: float = 1.0
    score: float = 0.0


@dataclass
class MothershipConfig:
    drone_slots: int = 8
    fast_chargers: int = 2
    refill_ports: int = 2
    move_speed_mps: float = 8.0
    current_point: Point | None = None


@dataclass
class DroneCapacityModel:
    spray_speed_ha_per_min: float = 0.35
    battery_minutes_full: float = 22.0
    chemical_full_ha: float = 2.5
    safe_return_margin: float = 0.15
    min_assist_resource: float = 0.20
    charge_20_to_80_minutes: float = 5.0


@dataclass
class OptConfig:
    scout_speed_factor: float = 0.45
    max_scout_count: int = 3
    cleanup_working_threshold: int = 2
    predeploy_count: int = 2
    relocation_enabled: bool = True


@dataclass
class OptPlan:
    phase: MissionPhase
    depot: DepotCandidate | None
    scout_drone_ids: list[int] = field(default_factory=list)
    worker_drone_ids: list[int] = field(default_factory=list)
    standby_drone_ids: list[int] = field(default_factory=list)
    predeploy_drone_ids: list[int] = field(default_factory=list)
    cleanup_drone_id: int | None = None
    assignments: dict[int, int] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)


class AgriculturalOptEngine:
    """Scout-driven multi-UAV agricultural OPT scheduling core."""

    def __init__(
        self,
        mothership: MothershipConfig,
        capacity: DroneCapacityModel | None = None,
        config: OptConfig | None = None,
    ) -> None:
        self.mothership = mothership
        self.capacity = capacity or DroneCapacityModel()
        self.config = config or OptConfig()
        self.phase = MissionPhase.PRE_SCOUT
        self.active_depot: DepotCandidate | None = None
        self.next_depot: DepotCandidate | None = None
        self.charger_slots: dict[int, int | None] = {
            i: None for i in range(self.mothership.fast_chargers)
        }

    def decide_scout_count(self, zones: list[FieldZone], drone_count: int) -> int:
        total_area = sum(self._get(z, "area_ha", 0.0) for z in zones)
        avg_complexity = self._avg([self._get(z, "terrain_complexity", 0.0) for z in zones])
        avg_obstacles = self._avg([self._get(z, "obstacle_density", 0.0) for z in zones])
        avg_risk = self._avg([self._get(z, "risk", 0.0) for z in zones])
        complexity_score = avg_complexity + avg_obstacles + avg_risk

        if total_area < 20 and complexity_score < 0.6:
            count = 1
        elif total_area < 80 and complexity_score < 1.2:
            count = 2
        else:
            count = 3
        return max(1, min(count, self.config.max_scout_count, max(1, drone_count // 2)))

    def select_scout_drones(self, telemetry: list[TelemetrySnapshot], scout_count: int) -> list[int]:
        candidates = [
            t
            for t in telemetry
            if t.state in (DroneState.IDLE, DroneState.STANDBY) and t.battery >= 0.45
        ]
        candidates.sort(key=lambda t: t.battery, reverse=True)
        return [t.drone_id for t in candidates[:scout_count]]

    def score_depot_candidates(
        self,
        candidates: list[DepotCandidate],
        zones: list[FieldZone],
    ) -> list[DepotCandidate]:
        scored: list[DepotCandidate] = []
        for c in candidates:
            distance_cost = self._average_distance_to_zones(c.point, zones)
            road_cost = self._get(c, "road_cost", 0.0)
            terrain_cost = self._get(c, "terrain_risk", 0.0) * 1000.0
            comm_penalty = (1.0 - self._get(c, "communication_score", 1.0)) * 1500.0
            rtk_penalty = (1.0 - self._get(c, "rtk_score", 1.0)) * 1500.0
            c.score = distance_cost + road_cost + terrain_cost + comm_penalty + rtk_penalty
            scored.append(c)
        scored.sort(key=lambda x: x.score)
        return scored

    def choose_best_depot(
        self,
        candidates: list[DepotCandidate],
        zones: list[FieldZone],
    ) -> DepotCandidate:
        if not candidates:
            raise ValueError("No depot candidates provided.")
        scored = self.score_depot_candidates(candidates, zones)
        self.active_depot = scored[0]
        self.mothership.current_point = scored[0].point
        return scored[0]

    def estimate_return_battery_required(self, drone_pos: Point, depot: Point) -> float:
        dist = self.distance(drone_pos, depot)
        return min(0.8, dist / 1000.0 * 0.08)

    def effective_work_capacity_ha(self, telemetry: TelemetrySnapshot, depot: Point) -> float:
        return_batt = self.estimate_return_battery_required(telemetry.position, depot)
        usable_battery = telemetry.battery - return_batt - self.capacity.safe_return_margin
        if usable_battery <= 0:
            return 0.0
        battery_minutes = usable_battery * self.capacity.battery_minutes_full
        battery_area = battery_minutes * self.capacity.spray_speed_ha_per_min
        chemical_area = telemetry.chemical * self.capacity.chemical_full_ha
        return max(0.0, min(battery_area, chemical_area))

    def can_assist(self, telemetry: TelemetrySnapshot, depot: Point) -> bool:
        return_batt = self.estimate_return_battery_required(telemetry.position, depot)
        battery_after_return = telemetry.battery - return_batt
        return (
            battery_after_return > self.capacity.min_assist_resource
            and telemetry.chemical > self.capacity.min_assist_resource
            and self.effective_work_capacity_ha(telemetry, depot) > 0.1
        )

    def build_initial_work_plan(
        self,
        telemetry: list[TelemetrySnapshot],
        zones: list[FieldZone],
        depot: DepotCandidate,
        scout_ids: list[int],
    ) -> OptPlan:
        available = [
            t
            for t in telemetry
            if t.drone_id not in scout_ids
            and t.state in (DroneState.IDLE, DroneState.STANDBY)
            and t.battery >= 0.35
        ]
        available.sort(key=lambda t: self.effective_work_capacity_ha(t, depot.point), reverse=True)
        worker_count = min(7, len(available))
        workers = available[:worker_count]
        standby = available[worker_count:]
        open_zones = [z for z in zones if not z.completed and z.remaining_area_ha > 0]
        open_zones.sort(key=lambda z: self.distance(depot.point, z.center))

        assignments: dict[int, int] = {}
        for drone in workers:
            cap = self.effective_work_capacity_ha(drone, depot.point)
            zone = self._pick_best_zone_for_drone(drone, open_zones, depot.point, cap)
            if zone is None:
                continue
            assignments[drone.drone_id] = zone.id
            if drone.drone_id not in zone.assigned_drone_ids:
                zone.assigned_drone_ids.append(drone.drone_id)

        return OptPlan(
            phase=MissionPhase.WORKING,
            depot=depot,
            scout_drone_ids=scout_ids,
            worker_drone_ids=[t.drone_id for t in workers],
            standby_drone_ids=[t.drone_id for t in standby],
            assignments=assignments,
            notes=["Initial work plan built by OPT."],
        )

    def assign_assist_tasks(
        self,
        telemetry: list[TelemetrySnapshot],
        zones: list[FieldZone],
        depot: DepotCandidate,
    ) -> dict[int, int]:
        assist_assignments: dict[int, int] = {}
        unfinished = [z for z in zones if not z.completed and z.remaining_area_ha > 0]
        if not unfinished:
            return assist_assignments
        unfinished.sort(key=lambda z: (z.remaining_area_ha, z.risk), reverse=True)

        for t in telemetry:
            if t.state not in (DroneState.IDLE, DroneState.STANDBY, DroneState.ASSISTING):
                continue
            if not self.can_assist(t, depot.point):
                continue
            zone = self._pick_best_zone_for_drone(
                telemetry=t,
                zones=unfinished,
                depot=depot.point,
                capacity_ha=self.effective_work_capacity_ha(t, depot.point),
            )
            if zone is not None:
                assist_assignments[t.drone_id] = zone.id
                if t.drone_id not in zone.assigned_drone_ids:
                    zone.assigned_drone_ids.append(t.drone_id)
        return assist_assignments

    def drones_should_return(
        self,
        telemetry: list[TelemetrySnapshot],
        depot: DepotCandidate,
    ) -> list[int]:
        returning: list[int] = []
        for t in telemetry:
            if t.state in (DroneState.RETURNING, DroneState.CHARGING, DroneState.REFILLING):
                continue
            return_batt = self.estimate_return_battery_required(t.position, depot.point)
            if t.battery <= return_batt + self.capacity.safe_return_margin:
                returning.append(t.drone_id)
                continue
            if t.chemical <= 0.05:
                returning.append(t.drone_id)
                continue
            if self.effective_work_capacity_ha(t, depot.point) <= 0.05:
                returning.append(t.drone_id)
        return returning

    def required_charge_for_task(self, task_area_ha: float, return_battery: float) -> float:
        task_minutes = task_area_ha / max(0.001, self.capacity.spray_speed_ha_per_min)
        task_battery = task_minutes / max(0.001, self.capacity.battery_minutes_full)
        required = task_battery + return_battery + self.capacity.safe_return_margin
        if required <= 0.8:
            return max(0.25, required)
        return min(1.0, required)

    def estimate_charge_minutes(self, from_battery: float, to_battery: float) -> float:
        from_battery = max(0.0, min(1.0, from_battery))
        to_battery = max(0.0, min(1.0, to_battery))
        if to_battery <= from_battery:
            return 0.0
        fast_rate = 0.60 / self.capacity.charge_20_to_80_minutes
        return (to_battery - from_battery) / fast_rate

    def schedule_charging_queue(
        self,
        returning_drone_ids: list[int],
        telemetry: list[TelemetrySnapshot],
    ) -> dict[int, str]:
        status: dict[int, str] = {}
        tel_by_id = {t.drone_id: t for t in telemetry}
        returning_drone_ids.sort(
            key=lambda drone_id: tel_by_id[drone_id].battery if drone_id in tel_by_id else 1.0
        )
        free_chargers = [
            slot for slot, drone_id in self.charger_slots.items() if drone_id is None
        ]
        for drone_id in returning_drone_ids:
            if free_chargers:
                slot = free_chargers.pop(0)
                self.charger_slots[slot] = drone_id
                status[drone_id] = "fast_charging"
            else:
                status[drone_id] = "waiting_slot"
        return status

    def should_relocate_depot(
        self,
        telemetry: list[TelemetrySnapshot],
        zones: list[FieldZone],
        current_depot: DepotCandidate,
        next_depot: DepotCandidate | None,
    ) -> bool:
        if not self.config.relocation_enabled or next_depot is None:
            return False
        unfinished = [z for z in zones if not z.completed and z.remaining_area_ha > 0]
        if not unfinished:
            return False
        old_zone_remaining = sum(z.remaining_area_ha for z in unfinished)
        working = [t for t in telemetry if t.state in (DroneState.WORKING, DroneState.ASSISTING)]
        if len(working) > self.config.cleanup_working_threshold:
            return False
        cleanup_eta = self.estimate_cleanup_eta_minutes(working, old_zone_remaining)
        move_eta = self.estimate_depot_move_minutes(current_depot.point, next_depot.point)
        return move_eta <= cleanup_eta

    def estimate_cleanup_eta_minutes(
        self,
        working: list[TelemetrySnapshot],
        remaining_area_ha: float,
    ) -> float:
        if not working:
            return float("inf")
        total_rate = len(working) * self.capacity.spray_speed_ha_per_min
        return remaining_area_ha / max(0.001, total_rate)

    def estimate_depot_move_minutes(self, from_point: Point, to_point: Point) -> float:
        dist_m = self.distance(from_point, to_point)
        return dist_m / max(0.001, self.mothership.move_speed_mps) / 60.0

    def build_relocation_plan(
        self,
        telemetry: list[TelemetrySnapshot],
        old_zones: list[FieldZone],
        new_zones: list[FieldZone],
        current_depot: DepotCandidate,
        next_depot: DepotCandidate,
    ) -> OptPlan:
        candidates = [
            t
            for t in telemetry
            if t.state in (DroneState.IDLE, DroneState.STANDBY, DroneState.ASSISTING)
            and self.can_assist(t, current_depot.point)
        ]
        candidates.sort(
            key=lambda t: (
                self.effective_work_capacity_ha(t, current_depot.point),
                t.battery,
                t.chemical,
            ),
            reverse=True,
        )
        cleanup_id = candidates[0].drone_id if candidates else None
        predeploy = [t.drone_id for t in candidates[1 : 1 + self.config.predeploy_count]]
        self.next_depot = next_depot
        return OptPlan(
            phase=MissionPhase.DEPOT_RELOCATING,
            depot=next_depot,
            cleanup_drone_id=cleanup_id,
            predeploy_drone_ids=predeploy,
            notes=[
                "Relocation plan generated.",
                "Cleanup drone stays for old zone.",
                "Predeploy drones enter next depot control zone.",
            ],
        )

    def old_zone_can_be_left_to_cleanup_drone(
        self,
        cleanup: TelemetrySnapshot,
        old_zones: list[FieldZone],
        current_depot: DepotCandidate,
    ) -> bool:
        remaining = sum(z.remaining_area_ha for z in old_zones if not z.completed)
        cap = self.effective_work_capacity_ha(cleanup, current_depot.point)
        return remaining <= cap

    def build_opt_plan(
        self,
        telemetry: list[TelemetrySnapshot],
        zones: list[FieldZone],
        depot_candidates: list[DepotCandidate],
        scout_finished: bool,
    ) -> OptPlan:
        if not telemetry:
            raise ValueError("No telemetry available.")
        if self.active_depot is None:
            depot = self.choose_best_depot(depot_candidates, zones)
        else:
            depot = self.active_depot
        if not scout_finished:
            scout_count = self.decide_scout_count(zones, len(telemetry))
            scout_ids = self.select_scout_drones(telemetry, scout_count)
            return OptPlan(
                phase=MissionPhase.SCOUTING,
                depot=depot,
                scout_drone_ids=scout_ids,
                notes=[
                    f"Scout count decided: {scout_count}",
                    "Scout drones should scan full field with slow avoidance posture.",
                ],
            )
        return self.build_initial_work_plan(
            telemetry=telemetry,
            zones=zones,
            depot=depot,
            scout_ids=[],
        )

    def _pick_best_zone_for_drone(
        self,
        telemetry: TelemetrySnapshot,
        zones: list[FieldZone],
        depot: Point,
        capacity_ha: float,
    ) -> FieldZone | None:
        if capacity_ha <= 0:
            return None
        candidates = [z for z in zones if not z.completed and z.remaining_area_ha > 0]
        if not candidates:
            return None

        def score(z: FieldZone) -> float:
            fly_cost = self.distance(telemetry.position, z.center)
            depot_cost = self.distance(depot, z.center)
            risk_cost = z.risk * 1000.0
            over_size_penalty = max(0.0, z.remaining_area_ha - capacity_ha) * 500.0
            assigned_penalty = len(z.assigned_drone_ids) * 2000.0
            return fly_cost + depot_cost * 0.25 + risk_cost + over_size_penalty + assigned_penalty

        candidates.sort(key=score)
        return candidates[0]

    def _average_distance_to_zones(self, point: Point, zones: list[FieldZone]) -> float:
        if not zones:
            return 0.0
        return sum(self.distance(point, z.center) for z in zones) / len(zones)

    @staticmethod
    def distance(a: Point, b: Point) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    @staticmethod
    def _avg(values: list[float]) -> float:
        if not values:
            return 0.0
        return sum(values) / len(values)

    @staticmethod
    def _get(obj: Any, name: str, default: Any) -> Any:
        return getattr(obj, name, default)

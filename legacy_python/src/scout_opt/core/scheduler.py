from __future__ import annotations

from scout_opt.core.charging import ChargingQueue
from scout_opt.core.depot_planner import plan_minimal_depot_operations
from scout_opt.core.emergency import build_weather_recovery_decision
from scout_opt.core.models import Drone, DroneSpec, DroneState, FieldMap, Mothership, SimulationEvent, TelemetrySnapshot
from scout_opt.core.opt import (
    plan_depot_sequence,
    should_relocate_for_rolling_transition,
    total_assignment_cost,
)
from scout_opt.core.opt_bridge import (
    field_to_depot_candidates,
    field_to_opt_zones,
    mothership_to_opt_config,
)
from scout_opt.core.opt_engine import AgriculturalOptEngine
from scout_opt.core.resources import (
    dynamic_remaining_capacity_area,
    estimate_remaining_work_time_seconds,
    needs_recall,
    target_charge_for_area,
    update_dynamic_resource_model,
)
from scout_opt.core.radius_scheduler import (
    choose_assist_task_for_drone,
    choose_radius_task_for_drone,
    should_switch_mothership_point,
)
from scout_opt.core.safety import check_safety_constraints
from scout_opt.core.scout import decide_scout_count, synthesize_scout_outputs
from scout_opt.core.tasks import (
    apply_task_progress,
    build_coverage_tasks,
    choose_best_task,
    mark_task_assigned,
    open_tasks,
    release_task,
    task_by_id,
)
from scout_opt.core.weather import WeatherAdjustedSpec, adjust_spec_for_weather


class MissionScheduler:
    def __init__(self, charging_queue: ChargingQueue) -> None:
        self.charging_queue = charging_queue
        self.scout_started = False
        self.scout_completed = False
        self.scout_ids: set[int] = set()
        self.operation_plan_ready = False
        self.opt_engine: AgriculturalOptEngine | None = None

    def _ensure_opt_engine(self, mothership: Mothership) -> None:
        if self.opt_engine is None:
            self.opt_engine = AgriculturalOptEngine(mothership_to_opt_config(mothership))

    def step(
        self,
        now_seconds: float,
        dt_seconds: float,
        field: FieldMap,
        mothership: Mothership,
        drones: list[Drone],
        spec: DroneSpec,
    ) -> list[SimulationEvent]:
        events: list[SimulationEvent] = []
        self._ensure_opt_engine(mothership)
        weather_spec = adjust_spec_for_weather(spec, mothership.weather)
        update_dynamic_resource_model(drones, mothership, spec)
        safety_report = check_safety_constraints(field, mothership, drones)
        if not safety_report.ok:
            events.append(SimulationEvent(now_seconds, "safety", "; ".join(safety_report.reasons[:3])))
        self._complete_emergency_landings(now_seconds, drones, events)
        if self._handle_weather_recovery(now_seconds, field, mothership, drones, events):
            self.charging_queue.step(drones, dt_seconds)
            return events
        self._dock_returning_drones_for_weather(now_seconds, field, drones, spec, events)
        if not weather_spec.flight_allowed:
            events.append(SimulationEvent(now_seconds, "weather", "flight paused by weather constraints"))
            self.charging_queue.step(drones, dt_seconds)
            return events
        if not weather_spec.spray_allowed and any(
            drone.state in {DroneState.WORKING, DroneState.ASSISTING, DroneState.CLEANUP} for drone in drones
        ):
            events.append(SimulationEvent(now_seconds, "weather", "spraying paused by weather constraints"))
        self._update_mothership_motion(now_seconds, dt_seconds, mothership, drones, events)
        self._assign_scout_if_needed(now_seconds, field, drones, spec, events)
        self._update_active_drones(dt_seconds, field, drones, spec, weather_spec, events, now_seconds)
        if field.scanned:
            self._initialize_operation_plan(now_seconds, field, mothership, drones, events)
            self._relocate_mothership_if_better(now_seconds, field, mothership, drones, spec, events)
            self._predeploy_to_next_zone(now_seconds, field, mothership, drones, spec, events)
            self._assign_working_drones(now_seconds, field, mothership, drones, spec, events)
            self._assign_assisting_drones(now_seconds, field, mothership, drones, spec, events)
        self._recall_low_resource_drones(now_seconds, field, mothership, drones, spec, events)
        self.charging_queue.step(drones, dt_seconds)
        return events

    def _assign_scout_if_needed(
        self,
        now_seconds: float,
        field: FieldMap,
        drones: list[Drone],
        spec: DroneSpec,
        events: list[SimulationEvent],
    ) -> None:
        if self.scout_started:
            return
        telemetry = [
            drone_snapshot(drone)
            for drone in drones
        ]
        opt_zones = field_to_opt_zones(field)
        scout_count = self.opt_engine.decide_scout_count(opt_zones, len(drones)) if self.opt_engine else decide_scout_count(
            field.area_hectares,
            field.terrain_complexity,
            field.obstacle_density,
            len(drones),
        )
        scout_ids = (
            self.opt_engine.select_scout_drones(telemetry, scout_count)
            if self.opt_engine
            else [drone.id for drone in drones[:scout_count]]
        )
        by_id = {drone.id: drone for drone in drones}
        for drone_id in scout_ids:
            drone = by_id[drone_id]
            drone.state = DroneState.SCOUTING
            drone.assigned_area_hectares = field.area_hectares / max(1, len(scout_ids))
            drone.task_elapsed_seconds = 0.0
            drone.role_detail = "initial_scout"
            self.scout_ids.add(drone.id)
        self.scout_started = True
        events.append(SimulationEvent(now_seconds, "scout", f"assigned {scout_count} scout drone(s)"))

    def _update_active_drones(
        self,
        dt_seconds: float,
        field: FieldMap,
        drones: list[Drone],
        spec: DroneSpec,
        weather_spec: WeatherAdjustedSpec,
        events: list[SimulationEvent],
        now_seconds: float,
    ) -> None:
        dt_hours = dt_seconds / 3600.0
        for drone in drones:
            if drone.state == DroneState.SCOUTING:
                covered = weather_spec.scout_rate_hectares_per_hour * dt_hours
                drone.assigned_area_hectares = max(0.0, drone.assigned_area_hectares - covered)
                drone.battery = max(
                    0.0,
                    drone.battery
                    - spec.battery_drain_per_hour_scout * weather_spec.battery_scout_multiplier * dt_hours,
                )
                if drone.assigned_area_hectares <= 0.0:
                    drone.state = DroneState.RETURNING
                    events.append(SimulationEvent(now_seconds, "scout", f"drone {drone.id} completed scout segment"))

            elif drone.state in {DroneState.WORKING, DroneState.ASSISTING, DroneState.CLEANUP, DroneState.PREDEPLOY}:
                if drone.state == DroneState.PREDEPLOY:
                    drone.battery = max(
                        0.0,
                        drone.battery
                        - spec.battery_drain_per_hour_scout * weather_spec.battery_scout_multiplier * 0.5 * dt_hours,
                    )
                    drone.state = DroneState.STANDBY
                    events.append(SimulationEvent(now_seconds, "predeploy", f"drone {drone.id} staged near next zone"))
                    continue
                if not weather_spec.spray_allowed:
                    drone.battery = max(
                        0.0,
                        drone.battery
                        - spec.battery_drain_per_hour_work * 0.25 * weather_spec.battery_work_multiplier * dt_hours,
                    )
                    continue

                task = task_by_id(field, drone.assigned_task_id)
                if task is None:
                    drone.state = DroneState.RETURNING
                    events.append(SimulationEvent(now_seconds, "task", f"drone {drone.id} lost task assignment"))
                    continue

                area_done = min(task.remaining_area_hectares, weather_spec.spray_rate_hectares_per_hour * dt_hours)
                area_done *= weather_spec.spray_effectiveness
                apply_task_progress(field, task, area_done, now_seconds)
                drone.assigned_area_hectares = max(0.0, drone.assigned_area_hectares - area_done)
                drone.battery = max(
                    0.0,
                    drone.battery
                    - spec.battery_drain_per_hour_work * weather_spec.battery_work_multiplier * dt_hours,
                )
                drone.chemical = max(0.0, drone.chemical - spec.chemical_per_hectare * area_done)
                if drone.assigned_area_hectares <= 0.0 and task.remaining_area_hectares > 0.0:
                    release_task(field, drone)
                    drone.state = DroneState.RETURNING
                    events.append(
                        SimulationEvent(
                            now_seconds,
                            "resource",
                            f"drone {drone.id} exhausted assigned capacity on task {task.id}",
                        )
                    )
                    continue
                if task.remaining_area_hectares <= 0.0 or field.remaining_area_hectares <= 0.0:
                    drone.assigned_task_id = None
                    if open_tasks(field) and dynamic_remaining_capacity_area(drone, field.boundary_center, spec) > 0.8:
                        next_task = choose_best_task(field, drone, field.boundary_center, queue_pressure=0.0)
                        if next_task is not None:
                            drone.state = DroneState.ASSISTING
                            mark_task_assigned(next_task, drone, now_seconds)
                            drone.assigned_area_hectares = min(next_task.remaining_area_hectares, drone.remaining_capacity_area)
                            events.append(
                                SimulationEvent(
                                    now_seconds,
                                    "assist",
                                    f"drone {drone.id} reassigned task {next_task.id} ({drone.assigned_area_hectares:.2f} ha)",
                                )
                            )
                        else:
                            drone.state = DroneState.RETURNING
                    else:
                        drone.state = DroneState.RETURNING
                        events.append(SimulationEvent(now_seconds, "work", f"drone {drone.id} completed task {task.id}"))

            elif drone.state == DroneState.RETURNING:
                drone.state = DroneState.CHARGING
                target_charge = target_charge_for_area(4.0, drone, field.boundary_center, spec)
                self.charging_queue.enqueue(drone, target_charge=target_charge)
                if self.scout_started and not self.scout_completed and all(
                    drone_by_id.state in {DroneState.CHARGING, DroneState.REFILLING, DroneState.STANDBY}
                    for drone_by_id in drones
                    if drone_by_id.id in self.scout_ids
                ):
                    synthesize_scout_outputs(field)
                    build_coverage_tasks(field)
                    self.scout_completed = True
                    events.append(
                        SimulationEvent(
                            now_seconds,
                            "opt",
                            f"scout outputs synthesized; {len(field.tasks)} coverage tasks generated",
                        )
                    )

            elif drone.state == DroneState.EMERGENCY_LANDING:
                drone.state = DroneState.LANDED
                events.append(SimulationEvent(now_seconds, "emergency", f"drone {drone.id} landed at emergency spot"))

    def _update_mothership_motion(
        self,
        now_seconds: float,
        dt_seconds: float,
        mothership: Mothership,
        drones: list[Drone],
        events: list[SimulationEvent],
    ) -> None:
        if not mothership.moving or mothership.destination is None:
            return
        mothership.move_remaining_seconds = max(0.0, mothership.move_remaining_seconds - dt_seconds)
        if mothership.move_remaining_seconds > 0.0:
            return
        mothership.position = mothership.destination
        mothership.destination = None
        mothership.moving = False
        for drone in drones:
            if drone.state in {DroneState.IDLE, DroneState.STANDBY, DroneState.CHARGING}:
                drone.position = mothership.position
        events.append(SimulationEvent(now_seconds, "depot", f"mothership arrived at {mothership.position}"))

    def _initialize_operation_plan(
        self,
        now_seconds: float,
        field: FieldMap,
        mothership: Mothership,
        drones: list[Drone],
        events: list[SimulationEvent],
    ) -> None:
        if self.operation_plan_ready:
            return
        build_coverage_tasks(field)
        depot_plan = plan_minimal_depot_operations(field, mothership)
        if depot_plan.stops:
            mothership.operation_plan = depot_plan.points
            if self.opt_engine:
                depot_candidates = field_to_depot_candidates(field)
                opt_zones = field_to_opt_zones(field)
                ranked = self.opt_engine.score_depot_candidates(depot_candidates, opt_zones) if depot_candidates else []
                if ranked:
                    self.opt_engine.active_depot = ranked[0]
                    self.opt_engine.mothership.current_point = ranked[0].point
            if depot_plan.uncovered_task_ids:
                events.append(
                    SimulationEvent(
                        now_seconds,
                        "depot",
                        f"depot plan has uncovered tasks {sorted(depot_plan.uncovered_task_ids)}",
                    )
                )
        else:
            mothership.operation_plan = plan_depot_sequence(field, drones, mothership.fast_chargers)
        mothership.operation_plan_index = 0
        self.operation_plan_ready = True
        events.append(
            SimulationEvent(
                now_seconds,
                "opt",
                f"planned {len(mothership.operation_plan)} mothership operation point(s)",
            )
        )

    def _relocate_mothership_if_better(
        self,
        now_seconds: float,
        field: FieldMap,
        mothership: Mothership,
        drones: list[Drone],
        spec: DroneSpec,
        events: list[SimulationEvent],
    ) -> None:
        if mothership.moving:
            return
        if not mothership.operation_plan:
            depot_candidates = field_to_depot_candidates(field)
            opt_zones = field_to_opt_zones(field)
            if self.opt_engine and depot_candidates:
                best = self.opt_engine.choose_best_depot(depot_candidates, opt_zones).point
            else:
                best = field.boundary_center
            mothership.operation_plan = [best]

        current_index = mothership.operation_plan_index
        current_target = mothership.operation_plan[current_index]
        if mothership.position.distance_to(current_target) > 100.0 and current_index == 0:
            self._start_mothership_move(now_seconds, mothership, current_target, events, "initial operation point")
            return

        next_index = current_index + 1
        if next_index >= len(mothership.operation_plan):
            return
        remaining_cleanup_seconds = estimate_remaining_work_time_seconds(field, drones, spec)
        next_point = mothership.operation_plan[next_index]
        if should_switch_mothership_point(
            field=field,
            mothership=mothership,
            drones=drones,
            spec=spec,
            next_point=next_point,
            cleanup_eta_seconds=remaining_cleanup_seconds,
        ) or (
            field.remaining_area_hectares <= 8.0
            and should_relocate_for_rolling_transition(mothership, next_point, remaining_cleanup_seconds)
        ):
            mothership.operation_plan_index = next_index
            self._start_mothership_move(now_seconds, mothership, next_point, events, "rolling transition")

    def _start_mothership_move(
        self,
        now_seconds: float,
        mothership: Mothership,
        destination: object,
        events: list[SimulationEvent],
        reason: str,
    ) -> None:
        distance = mothership.position.distance_to(destination)
        mothership.destination = destination
        mothership.move_remaining_seconds = distance / mothership.move_speed_mps
        mothership.moving = True
        events.append(
            SimulationEvent(
                now_seconds,
                "depot",
                f"mothership moving to {destination} for {reason} ({mothership.move_remaining_seconds:.0f}s)",
            )
        )

    def _assign_working_drones(
        self,
        now_seconds: float,
        field: FieldMap,
        mothership: Mothership,
        drones: list[Drone],
        spec: DroneSpec,
        events: list[SimulationEvent],
    ) -> None:
        if field.remaining_area_hectares <= 0.0:
            return
        if mothership.moving:
            return
        available = [drone for drone in drones if drone.is_available_for_field_work()]
        available.sort(
            key=lambda drone: total_assignment_cost(
                drone,
                mothership.position,
                field,
                work_time_seconds=1800.0,
                charging_wait_seconds=len(self.charging_queue.waiting_queue) * 600.0,
                refill_wait_seconds=0.0,
            )
        )
        for drone in available:
            if field.remaining_area_hectares <= 0.0:
                return
            scored = choose_radius_task_for_drone(
                field,
                drone,
                mothership,
                spec,
                queue_pressure=len(self.charging_queue.waiting_queue) * 0.2,
            )
            task = scored.task if scored is not None else choose_best_task(
                field,
                drone,
                mothership.position,
                queue_pressure=len(self.charging_queue.waiting_queue) * 0.2,
            )
            if task is None:
                return
            capacity = scored.capacity_ha if scored is not None else dynamic_remaining_capacity_area(drone, mothership.position, spec)
            if task.remaining_area_hectares <= capacity <= 2.5 or field.remaining_area_hectares <= 2.5:
                role = DroneState.CLEANUP
                segment = min(task.remaining_area_hectares, capacity)
            else:
                role = DroneState.WORKING
                segment = min(task.remaining_area_hectares, capacity)
            if segment <= 0.5 and role != DroneState.CLEANUP:
                target_charge = target_charge_for_area(4.0, drone, mothership.position, spec)
                self.charging_queue.enqueue(drone, target_charge=target_charge)
                continue
            drone.state = role
            mark_task_assigned(task, drone, now_seconds)
            drone.assigned_area_hectares = segment
            events.append(
                SimulationEvent(
                    now_seconds,
                    role.value,
                    f"drone {drone.id} assigned task {task.id} zone {task.zone_id} ({segment:.2f} ha)",
                )
            )

    def _assign_assisting_drones(
        self,
        now_seconds: float,
        field: FieldMap,
        mothership: Mothership,
        drones: list[Drone],
        spec: DroneSpec,
        events: list[SimulationEvent],
    ) -> None:
        if field.remaining_area_hectares <= 0.0 or mothership.moving:
            return
        active_count = sum(
            1 for drone in drones if drone.state in {DroneState.WORKING, DroneState.ASSISTING, DroneState.CLEANUP}
        )
        if active_count >= max(2, len(drones) - 1):
            return
        helpers = [
            drone
            for drone in drones
            if drone.state == DroneState.STANDBY and dynamic_remaining_capacity_area(drone, mothership.position, spec) > 1.0
        ]
        helpers.sort(key=lambda drone: drone.remaining_capacity_area, reverse=True)
        for drone in helpers[: max(0, len(drones) - 1 - active_count)]:
            if field.remaining_area_hectares <= 3.0:
                return
            active_drones = [
                active
                for active in drones
                if active.state in {DroneState.WORKING, DroneState.ASSISTING, DroneState.CLEANUP}
            ]
            scored = choose_assist_task_for_drone(
                field=field,
                drone=drone,
                mothership=mothership,
                spec=spec,
                active_drones=active_drones,
            )
            task = scored.task if scored is not None else choose_best_task(field, drone, mothership.position, queue_pressure=0.0)
            if task is None:
                return
            drone.state = DroneState.ASSISTING
            mark_task_assigned(task, drone, now_seconds)
            drone.assigned_area_hectares = min(task.remaining_area_hectares, drone.remaining_capacity_area)
            events.append(
                SimulationEvent(
                    now_seconds,
                    "assist",
                    f"drone {drone.id} assisting task {task.id} ({drone.assigned_area_hectares:.2f} ha)",
                )
            )

    def _predeploy_to_next_zone(
        self,
        now_seconds: float,
        field: FieldMap,
        mothership: Mothership,
        drones: list[Drone],
        spec: DroneSpec,
        events: list[SimulationEvent],
    ) -> None:
        if not mothership.operation_plan or mothership.operation_plan_index + 1 >= len(mothership.operation_plan):
            return
        if field.remaining_area_hectares > 12.0:
            return
        next_point = mothership.operation_plan[mothership.operation_plan_index + 1]
        candidates = [
            drone
            for drone in drones
            if drone.state == DroneState.STANDBY and dynamic_remaining_capacity_area(drone, next_point, spec) > 2.0
        ]
        candidates.sort(key=lambda drone: drone.remaining_capacity_area, reverse=True)
        for drone in candidates[:1]:
            drone.state = DroneState.PREDEPLOY
            drone.target = next_point
            drone.role_detail = "next_zone_staging"
            events.append(SimulationEvent(now_seconds, "predeploy", f"drone {drone.id} entering next zone"))

    def _recall_low_resource_drones(
        self,
        now_seconds: float,
        field: FieldMap,
        mothership: Mothership,
        drones: list[Drone],
        spec: DroneSpec,
        events: list[SimulationEvent],
    ) -> None:
        for drone in drones:
            if needs_recall(drone, mothership, spec):
                release_task(field, drone)
                drone.state = DroneState.RETURNING
                events.append(SimulationEvent(now_seconds, "safety", f"drone {drone.id} recalled for low resource"))

    def _handle_weather_recovery(
        self,
        now_seconds: float,
        field: FieldMap,
        mothership: Mothership,
        drones: list[Drone],
        events: list[SimulationEvent],
    ) -> bool:
        decision = build_weather_recovery_decision(field, mothership, drones)
        if not decision.returning_drone_ids and not decision.emergency_landing_assignments:
            return False
        by_id = {drone.id: drone for drone in drones}
        for drone_id in decision.returning_drone_ids:
            drone = by_id[drone_id]
            release_task(field, drone)
            drone.state = DroneState.RETURNING
            drone.target = mothership.position
        for drone_id, spot in decision.emergency_landing_assignments.items():
            drone = by_id[drone_id]
            release_task(field, drone)
            drone.state = DroneState.EMERGENCY_LANDING
            drone.target = spot
        events.append(
            SimulationEvent(
                now_seconds,
                "emergency",
                (
                    f"{decision.reason}; returning={decision.returning_drone_ids}; "
                    f"landing={list(decision.emergency_landing_assignments)}"
                ),
            )
        )
        return True

    def _complete_emergency_landings(
        self,
        now_seconds: float,
        drones: list[Drone],
        events: list[SimulationEvent],
    ) -> None:
        for drone in drones:
            if drone.state == DroneState.EMERGENCY_LANDING:
                drone.state = DroneState.LANDED
                events.append(
                    SimulationEvent(
                        now_seconds,
                        "emergency",
                        f"drone {drone.id} landed at emergency spot {drone.target}",
                    )
                )

    def _dock_returning_drones_for_weather(
        self,
        now_seconds: float,
        field: FieldMap,
        drones: list[Drone],
        spec: DroneSpec,
        events: list[SimulationEvent],
    ) -> None:
        for drone in drones:
            if drone.state == DroneState.RETURNING:
                drone.state = DroneState.CHARGING
                target_charge = target_charge_for_area(4.0, drone, field.boundary_center, spec)
                self.charging_queue.enqueue(drone, target_charge=target_charge)
                events.append(SimulationEvent(now_seconds, "recovery", f"drone {drone.id} recovered to mothership"))


def drone_snapshot(drone: Drone) -> TelemetrySnapshot:
    return TelemetrySnapshot(
        drone_id=drone.id,
        state=drone.state,
        battery=drone.battery,
        chemical=drone.chemical,
        position=drone.position,
    )

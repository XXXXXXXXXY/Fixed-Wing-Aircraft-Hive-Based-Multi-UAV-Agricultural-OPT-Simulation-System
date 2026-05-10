from __future__ import annotations

from dataclasses import dataclass, field

from scout_opt.core.models import Drone, DroneState


def required_charge_for_next_task(
    task_energy: float,
    return_energy: float,
    safety_margin: float = 0.15,
) -> float:
    return max(0.3, min(0.9, task_energy + return_energy + safety_margin))


@dataclass
class ChargingQueue:
    fast_chargers: int = 2
    refill_ports: int = 2
    storage_slots: int = 8
    charging_slots: list[int | None] = field(default_factory=lambda: [None, None])
    refill_slots: list[int | None] = field(default_factory=lambda: [None, None])
    waiting_queue: list[int] = field(default_factory=list)
    refill_queue: list[int] = field(default_factory=list)
    charge_rate_per_hour: float = 0.85
    refill_rate_per_hour: float = 1.8

    def enqueue(self, drone: Drone, target_charge: float | None = None) -> None:
        if target_charge is not None:
            drone.target_charge = max(0.3, min(0.95, target_charge))
        if drone.battery < drone.target_charge:
            if drone.id not in self.waiting_queue and drone.id not in self.charging_slots:
                self.waiting_queue.append(drone.id)
            drone.state = DroneState.CHARGING
        elif drone.chemical < 0.85:
            self.enqueue_refill(drone)
        else:
            drone.state = DroneState.STANDBY

    def enqueue_refill(self, drone: Drone) -> None:
        if drone.id not in self.refill_queue and drone.id not in self.refill_slots:
            self.refill_queue.append(drone.id)
        drone.state = DroneState.REFILLING

    def step(self, drones: list[Drone], dt_seconds: float) -> None:
        by_id = {drone.id: drone for drone in drones}
        for idx, assigned_id in enumerate(self.charging_slots):
            if assigned_id is None and self.waiting_queue:
                self.charging_slots[idx] = self.waiting_queue.pop(0)
        for idx, assigned_id in enumerate(self.refill_slots):
            if assigned_id is None and self.refill_queue:
                self.refill_slots[idx] = self.refill_queue.pop(0)

        dt_hours = dt_seconds / 3600.0
        for idx, assigned_id in enumerate(list(self.charging_slots)):
            if assigned_id is None:
                continue
            drone = by_id[assigned_id]
            drone.battery = min(1.0, drone.battery + self.charge_rate_per_hour * dt_hours)
            if drone.battery >= drone.target_charge:
                drone.task_elapsed_seconds = 0.0
                self.charging_slots[idx] = None
                if drone.chemical < 0.85:
                    self.enqueue_refill(drone)
                else:
                    drone.state = DroneState.STANDBY

        for idx, assigned_id in enumerate(list(self.refill_slots)):
            if assigned_id is None:
                continue
            drone = by_id[assigned_id]
            drone.chemical = min(1.0, drone.chemical + self.refill_rate_per_hour * dt_hours)
            if drone.chemical >= 0.85:
                drone.state = DroneState.STANDBY
                self.refill_slots[idx] = None

from __future__ import annotations

import unittest

from scout_opt.core.config import default_simulation_config
from scout_opt.core.manual_scout import apply_manual_scout_observation, load_manual_scout_observation
from scout_opt.core.models import DroneState, Point
from scout_opt.core.models import TaskStatus
from scout_opt.core.radius_scheduler import (
    choose_assist_task_for_drone,
    choose_radius_task_for_drone,
    should_switch_mothership_point,
    tasks_inside_radius,
    working_radius_for_depot,
)
from scout_opt.core.tasks import build_coverage_tasks


class RadiusSchedulerTests(unittest.TestCase):
    def test_radius_task_choice_prefers_depot_work_area(self) -> None:
        config = default_simulation_config()
        apply_manual_scout_observation(
            config.field,
            load_manual_scout_observation("configs/manual_scout_two_blocks.example.json"),
        )
        build_coverage_tasks(config.field)
        config.mothership.position = Point(-430.0, -280.0)
        drone = config.drones[0]
        drone.position = config.mothership.position

        scored = choose_radius_task_for_drone(
            config.field,
            drone,
            config.mothership,
            config.drone_spec,
            queue_pressure=0.0,
        )

        self.assertIsNotNone(scored)
        self.assertEqual(scored.task.block_id, 1)

    def test_assist_task_requires_useful_capacity(self) -> None:
        config = default_simulation_config()
        apply_manual_scout_observation(
            config.field,
            load_manual_scout_observation("configs/manual_scout_two_blocks.example.json"),
        )
        build_coverage_tasks(config.field)
        active = config.drones[0]
        active.state = DroneState.WORKING
        helper = config.drones[1]
        helper.state = DroneState.STANDBY
        helper.battery = 0.9
        helper.chemical = 0.9

        scored = choose_assist_task_for_drone(
            config.field,
            helper,
            config.mothership,
            config.drone_spec,
            active_drones=[active],
        )

        self.assertIsNotNone(scored)
        self.assertGreater(scored.capacity_ha, 0.35)

    def test_mothership_switch_uses_next_radius_gain(self) -> None:
        config = default_simulation_config()
        apply_manual_scout_observation(
            config.field,
            load_manual_scout_observation("configs/manual_scout_two_blocks.example.json"),
        )
        build_coverage_tasks(config.field)
        config.mothership.position = Point(-430.0, -280.0)
        next_point = Point(460.0, 280.0)
        for task in config.field.tasks:
            if task.block_id == 1:
                task.status = TaskStatus.DONE
                task.remaining_area_hectares = 0.0

        self.assertTrue(
            should_switch_mothership_point(
                config.field,
                config.mothership,
                config.drones,
                config.drone_spec,
                next_point=next_point,
                cleanup_eta_seconds=10_000.0,
            )
        )

    def test_tasks_inside_radius_filters_far_tasks(self) -> None:
        config = default_simulation_config()
        apply_manual_scout_observation(
            config.field,
            load_manual_scout_observation("configs/manual_scout_two_blocks.example.json"),
        )
        build_coverage_tasks(config.field)
        near_tasks = tasks_inside_radius(config.field, Point(-430.0, -280.0), radius_m=500.0)

        self.assertTrue(near_tasks)
        self.assertTrue(all(task.center.distance_to(Point(-430.0, -280.0)) <= 500.0 for task in near_tasks))
        self.assertGreater(working_radius_for_depot(config.mothership, config.drones, config.drone_spec), 0.0)


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import unittest

from scout_opt.core.config import default_simulation_config
from scout_opt.core.field_plan import apply_manual_field_plan, load_manual_field_plan
from scout_opt.core.manual_scout import apply_manual_scout_observation, load_manual_scout_observation
from scout_opt.core.opt_bridge import field_to_depot_candidates, field_to_opt_zones, mothership_to_opt_config
from scout_opt.core.opt_engine import AgriculturalOptEngine, MissionPhase
from scout_opt.core.tasks import build_coverage_tasks
from scout_opt.core.engine import SimulationEngine


class ManualWorkflowTests(unittest.TestCase):
    def test_manual_plan_and_manual_scout_build_work_plan(self) -> None:
        config = default_simulation_config()
        apply_manual_field_plan(
            config.field,
            load_manual_field_plan("configs/manual_field_plan.example.json"),
        )
        apply_manual_scout_observation(
            config.field,
            load_manual_scout_observation("configs/manual_scout.example.json"),
        )
        build_coverage_tasks(config.field)

        opt = AgriculturalOptEngine(mothership_to_opt_config(config.mothership))
        plan = opt.build_opt_plan(
            telemetry=SimulationEngine(config).telemetry(),
            zones=field_to_opt_zones(config.field),
            depot_candidates=field_to_depot_candidates(config.field),
            scout_finished=True,
        )

        self.assertEqual(plan.phase, MissionPhase.WORKING)
        self.assertTrue(plan.assignments)
        self.assertEqual(len(config.field.operation_zones), 4)
        self.assertTrue(config.field.scanned)
        self.assertTrue(config.field.manual_notes)

    def test_manual_scout_two_blocks_auto_generates_zones(self) -> None:
        config = default_simulation_config()
        apply_manual_scout_observation(
            config.field,
            load_manual_scout_observation("configs/manual_scout_two_blocks.example.json"),
        )
        build_coverage_tasks(config.field)

        opt = AgriculturalOptEngine(mothership_to_opt_config(config.mothership))
        depots = opt.score_depot_candidates(
            field_to_depot_candidates(config.field),
            field_to_opt_zones(config.field),
        )

        self.assertEqual(len(config.field.field_blocks), 2)
        self.assertGreater(len(config.field.operation_zones), len(config.field.field_blocks))
        self.assertTrue(all(zone.block_id in {1, 2} for zone in config.field.operation_zones))
        self.assertTrue(all(task.block_id in {1, 2} for task in config.field.tasks))
        self.assertTrue(depots)

    def test_block_area_can_be_calculated_from_boundary(self) -> None:
        observation = load_manual_scout_observation("configs/manual_scout_area_from_boundary.example.json")
        self.assertAlmostEqual(observation.field_blocks[0].area_hectares, 2.0)


if __name__ == "__main__":
    unittest.main()

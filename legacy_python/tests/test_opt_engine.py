from __future__ import annotations

import unittest

from scout_opt.core.config import default_simulation_config
from scout_opt.core.engine import SimulationEngine
from scout_opt.core.opt_bridge import (
    field_to_depot_candidates,
    field_to_opt_zones,
    mothership_to_opt_config,
)
from scout_opt.core.opt_engine import AgriculturalOptEngine, MissionPhase
from scout_opt.core.scout import synthesize_scout_outputs
from scout_opt.core.tasks import build_coverage_tasks


class OptEngineTests(unittest.TestCase):
    def test_builds_scout_plan(self) -> None:
        config = default_simulation_config()
        synthesize_scout_outputs(config.field)
        opt = AgriculturalOptEngine(mothership_to_opt_config(config.mothership))

        plan = opt.build_opt_plan(
            telemetry=SimulationEngine(config).telemetry(),
            zones=field_to_opt_zones(config.field),
            depot_candidates=field_to_depot_candidates(config.field),
            scout_finished=False,
        )

        self.assertEqual(plan.phase, MissionPhase.SCOUTING)
        self.assertGreaterEqual(len(plan.scout_drone_ids), 1)
        self.assertIsNotNone(plan.depot)

    def test_builds_work_plan(self) -> None:
        config = default_simulation_config()
        synthesize_scout_outputs(config.field)
        build_coverage_tasks(config.field)
        opt = AgriculturalOptEngine(mothership_to_opt_config(config.mothership))
        telemetry = SimulationEngine(config).telemetry()

        plan = opt.build_opt_plan(
            telemetry=telemetry,
            zones=field_to_opt_zones(config.field),
            depot_candidates=field_to_depot_candidates(config.field),
            scout_finished=True,
        )

        self.assertEqual(plan.phase, MissionPhase.WORKING)
        self.assertTrue(plan.assignments)
        self.assertLessEqual(len(plan.worker_drone_ids), 7)


if __name__ == "__main__":
    unittest.main()

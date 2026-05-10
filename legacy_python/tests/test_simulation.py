from __future__ import annotations

import unittest

from scout_opt.core.config import default_simulation_config
from scout_opt.core.coverage import estimate_path_distance_m, generate_lawnmower_path
from scout_opt.core.engine import SimulationEngine
from scout_opt.core.scout import synthesize_scout_outputs
from scout_opt.core.tasks import build_coverage_tasks


class SimulationTests(unittest.TestCase):
    def test_default_field_completes(self) -> None:
        config = default_simulation_config()
        engine = SimulationEngine(config)
        result = engine.run(max_steps=900)

        self.assertTrue(result.completed)
        self.assertAlmostEqual(config.field.treated_area_hectares, config.field.area_hectares)
        self.assertTrue(config.field.tasks)
        self.assertTrue(all(task.remaining_area_hectares <= 0.001 for task in config.field.tasks))

    def test_large_field_completes(self) -> None:
        config = default_simulation_config(
            field_area_hectares=160.0,
            terrain_complexity=0.6,
            obstacle_density=0.35,
        )
        engine = SimulationEngine(config)
        result = engine.run(max_steps=900)

        self.assertTrue(result.completed)
        self.assertGreaterEqual(len(config.mothership.operation_plan), 1)

    def test_task_path_generation(self) -> None:
        config = default_simulation_config()
        synthesize_scout_outputs(config.field)
        build_coverage_tasks(config.field)
        path = generate_lawnmower_path(config.field.tasks[0])

        self.assertGreater(len(path), 2)
        self.assertGreater(estimate_path_distance_m(path), 0.0)


if __name__ == "__main__":
    unittest.main()

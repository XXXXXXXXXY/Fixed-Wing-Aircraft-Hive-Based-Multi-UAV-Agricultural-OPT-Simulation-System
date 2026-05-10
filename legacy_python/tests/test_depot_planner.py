from __future__ import annotations

import unittest

from scout_opt.core.config import default_simulation_config
from scout_opt.core.depot_planner import DepotPlanningPolicy, is_site_deployable, plan_minimal_depot_operations
from scout_opt.core.manual_scout import apply_manual_scout_observation, load_manual_scout_observation
from scout_opt.core.tasks import build_coverage_tasks


class DepotPlannerTests(unittest.TestCase):
    def test_depot_plan_filters_sites_that_cannot_deploy(self) -> None:
        config = default_simulation_config()
        apply_manual_scout_observation(
            config.field,
            load_manual_scout_observation("configs/manual_scout_two_blocks.example.json"),
        )

        deployable = [
            site for site in config.field.depot_sites
            if is_site_deployable(site, config.field, DepotPlanningPolicy())
        ]

        self.assertEqual({site.id for site in deployable}, {1, 3})

    def test_depot_plan_uses_minimal_sites_for_split_fields(self) -> None:
        config = default_simulation_config()
        apply_manual_scout_observation(
            config.field,
            load_manual_scout_observation("configs/manual_scout_two_blocks.example.json"),
        )
        build_coverage_tasks(config.field)

        plan = plan_minimal_depot_operations(config.field, config.mothership)

        self.assertEqual([stop.site.id for stop in plan.stops], [1, 3])
        self.assertFalse(plan.uncovered_task_ids)
        self.assertGreater(plan.total_move_distance_m, 0.0)


if __name__ == "__main__":
    unittest.main()

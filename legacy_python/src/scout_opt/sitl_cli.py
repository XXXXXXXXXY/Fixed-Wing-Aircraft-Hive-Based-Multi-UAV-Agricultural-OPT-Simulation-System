from __future__ import annotations

import argparse
import asyncio

from scout_opt.adapters.sim_adapter import InMemorySimulationAdapter
from scout_opt.core.config import default_simulation_config
from scout_opt.core.field_plan import apply_manual_field_plan, load_manual_field_plan
from scout_opt.core.manual_scout import apply_manual_scout_observation, load_manual_scout_observation
from scout_opt.online_controller import (
    AgriculturalOnlineController,
    build_mavsdk_controller,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run OPT plan generation against SITL or dry-run telemetry.")
    parser.add_argument("--config", default="configs/sitl_8.json", help="SITL endpoint config JSON.")
    parser.add_argument("--connect", action="store_true", help="Connect to MAVSDK endpoints and execute the plan.")
    parser.add_argument("--scout-finished", action="store_true", help="Build a work plan instead of scout plan.")
    parser.add_argument("--field-area", type=float, default=72.0)
    parser.add_argument("--terrain-complexity", type=float, default=0.45)
    parser.add_argument("--obstacle-density", type=float, default=0.25)
    parser.add_argument("--field-plan", help="JSON file with manually planned operation zones.")
    parser.add_argument("--manual-scout", help="JSON file with manually confirmed scout observations.")
    return parser


async def async_main() -> None:
    args = build_parser().parse_args()
    simulation_config = default_simulation_config(
        field_area_hectares=args.field_area,
        terrain_complexity=args.terrain_complexity,
        obstacle_density=args.obstacle_density,
    )
    if args.field_plan:
        apply_manual_field_plan(
            simulation_config.field,
            load_manual_field_plan(args.field_plan),
        )
    if args.manual_scout:
        apply_manual_scout_observation(
            simulation_config.field,
            load_manual_scout_observation(args.manual_scout),
        )
        args.scout_finished = True

    if args.connect:
        controller = build_mavsdk_controller(args.config, simulation_config=simulation_config)
        await controller.adapter.connect()
    else:
        controller = AgriculturalOnlineController(
            adapter=InMemorySimulationAdapterPlaceholder(simulation_config),
            config=simulation_config,
        )

    plan = await controller.build_plan(scout_finished=args.scout_finished)
    print(f"phase: {plan.phase.value}")
    print(f"depot: {plan.depot.point if plan.depot else None}")
    print(f"scouts: {plan.scout_drone_ids}")
    print(f"workers: {plan.worker_drone_ids}")
    print(f"standby: {plan.standby_drone_ids}")
    print(f"assignments: {plan.assignments}")
    for note in plan.notes:
        print(f"- {note}")

    if args.connect:
        await controller.execute_plan(plan)


class InMemorySimulationAdapterPlaceholder(InMemorySimulationAdapter):
    def __init__(self, config) -> None:
        from scout_opt.core.engine import SimulationEngine

        super().__init__(SimulationEngine(config))


def main() -> None:
    asyncio.run(async_main())


if __name__ == "__main__":
    main()

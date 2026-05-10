from __future__ import annotations

import argparse

from scout_opt.core.config import default_simulation_config
from scout_opt.core.engine import SimulationEngine


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the scout-driven agricultural multi-UAV OPT simulator."
    )
    parser.add_argument("--steps", type=int, default=240, help="Number of simulation steps.")
    parser.add_argument("--dt", type=float, default=60.0, help="Step duration in seconds.")
    parser.add_argument("--drones", type=int, default=8, help="Drone inventory.")
    parser.add_argument("--field-area", type=float, default=72.0, help="Field area in hectares.")
    parser.add_argument("--terrain-complexity", type=float, default=0.45)
    parser.add_argument("--obstacle-density", type=float, default=0.25)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    config = default_simulation_config(
        drone_count=args.drones,
        field_area_hectares=args.field_area,
        terrain_complexity=args.terrain_complexity,
        obstacle_density=args.obstacle_density,
    )
    engine = SimulationEngine(config=config, dt_seconds=args.dt)
    result = engine.run(max_steps=args.steps)

    print("Scout-Driven Multi-UAV Agricultural OPT Simulation")
    print(f"completed: {result.completed}")
    print(f"time_hours: {result.elapsed_seconds / 3600:.2f}")
    print(f"treated_area_ha: {result.treated_area_hectares:.2f}/{config.field.area_hectares:.2f}")
    print(f"mothership_position: {result.mothership_position}")
    print(f"events: {len(result.events)}")
    for event in result.events[-12:]:
        print(f"- t={event.time_seconds:7.0f}s {event.kind}: {event.message}")


if __name__ == "__main__":
    main()

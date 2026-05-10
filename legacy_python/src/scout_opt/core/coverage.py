from __future__ import annotations

from math import ceil, sqrt

from scout_opt.core.models import FieldTask, Point


def generate_lawnmower_path(
    task: FieldTask,
    swath_width_m: float = 8.0,
    lane_length_m: float | None = None,
) -> list[Point]:
    """Generate a simple local coverage path for a task.

    This is intentionally deterministic and simulator-friendly. In SITL, these
    points become local mission waypoints after coordinate conversion.
    """

    square_side_m = sqrt(task.area_hectares * 10_000.0)
    length = lane_length_m or square_side_m
    lanes = max(2, ceil(square_side_m / swath_width_m))
    start_x = task.center.x - length / 2.0
    start_y = task.center.y - (lanes - 1) * swath_width_m / 2.0

    points: list[Point] = []
    for lane in range(lanes):
        y = start_y + lane * swath_width_m
        if lane % 2 == 0:
            points.append(Point(start_x, y))
            points.append(Point(start_x + length, y))
        else:
            points.append(Point(start_x + length, y))
            points.append(Point(start_x, y))
    return points


def estimate_path_distance_m(points: list[Point]) -> float:
    if len(points) < 2:
        return 0.0
    return sum(points[idx].distance_to(points[idx - 1]) for idx in range(1, len(points)))

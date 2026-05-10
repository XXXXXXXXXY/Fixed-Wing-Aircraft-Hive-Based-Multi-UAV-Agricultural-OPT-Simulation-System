from __future__ import annotations

from dataclasses import dataclass, field

from scout_opt.core.models import DepotSite, FieldMap, FieldTask, Mothership, Point, TaskStatus


@dataclass(frozen=True)
class DepotPlanningPolicy:
    coverage_radius_m: float = 620.0
    max_coverage_radius_m: float = 900.0
    required_deploy_area_m2: float = 180.0
    min_obstacle_clearance_m: float = 35.0
    max_slope_risk: float = 0.65


@dataclass
class DepotStop:
    site: DepotSite
    covered_task_ids: set[int] = field(default_factory=set)
    sequence_index: int = 0
    travel_from_previous_m: float = 0.0


@dataclass
class DepotOperationPlan:
    stops: list[DepotStop]
    uncovered_task_ids: set[int]
    total_move_distance_m: float

    @property
    def points(self) -> list[Point]:
        return [stop.site.point for stop in self.stops]


def plan_minimal_depot_operations(
    field: FieldMap,
    mothership: Mothership,
    policy: DepotPlanningPolicy = DepotPlanningPolicy(),
) -> DepotOperationPlan:
    tasks = [
        task
        for task in field.tasks
        if task.status != TaskStatus.DONE and task.remaining_area_hectares > 0
    ]
    if not tasks:
        return DepotOperationPlan(stops=[], uncovered_task_ids=set(), total_move_distance_m=0.0)

    sites = _candidate_sites(field)
    suitable = [
        site
        for site in sites
        if is_site_deployable(site, field, policy)
    ]
    if not suitable:
        suitable = [DepotSite(id=1, point=field.boundary_center, usable_area_m2=policy.required_deploy_area_m2)]

    coverage = {
        site.id: _covered_tasks(site.point, tasks, policy.coverage_radius_m)
        for site in suitable
    }
    selected: list[DepotSite] = []
    uncovered = {task.id for task in tasks}

    while uncovered:
        best = _best_next_site(suitable, coverage, uncovered, selected, mothership.position)
        if best is None:
            break
        selected.append(best)
        uncovered -= coverage[best.id]

        # Remove already selected sites from future choices.
        suitable = [site for site in suitable if site.id != best.id]

    # If strict radius missed some tasks, retry selected candidates with larger radius
    # before declaring uncovered. This handles separated fields while keeping the
    # preferred plan compact.
    if uncovered:
        for site in selected:
            uncovered -= _covered_tasks(site.point, tasks, policy.max_coverage_radius_m)

    ordered = _order_sites_by_nearest_route(selected, mothership.position)
    stops: list[DepotStop] = []
    previous = mothership.position
    total_distance = 0.0
    for idx, site in enumerate(ordered):
        travel = previous.distance_to(site.point)
        total_distance += travel
        stops.append(
            DepotStop(
                site=site,
                covered_task_ids=_covered_tasks(site.point, tasks, policy.max_coverage_radius_m),
                sequence_index=idx,
                travel_from_previous_m=travel,
            )
        )
        previous = site.point

    return DepotOperationPlan(
        stops=stops,
        uncovered_task_ids=uncovered,
        total_move_distance_m=total_distance,
    )


def is_site_deployable(site: DepotSite, field: FieldMap, policy: DepotPlanningPolicy) -> bool:
    if not site.can_deploy(policy.required_deploy_area_m2, policy.max_slope_risk):
        return False
    if field.obstacles and min(site.point.distance_to(obstacle) for obstacle in field.obstacles) < policy.min_obstacle_clearance_m:
        return False
    return True


def _candidate_sites(field: FieldMap) -> list[DepotSite]:
    if field.depot_sites:
        return field.depot_sites
    if field.candidate_depot_points:
        return [
            DepotSite(id=idx + 1, point=point)
            for idx, point in enumerate(field.candidate_depot_points)
        ]
    return [DepotSite(id=1, point=field.boundary_center)]


def _covered_tasks(point: Point, tasks: list[FieldTask], radius_m: float) -> set[int]:
    return {
        task.id
        for task in tasks
        if point.distance_to(task.center) <= radius_m
    }


def _best_next_site(
    sites: list[DepotSite],
    coverage: dict[int, set[int]],
    uncovered: set[int],
    selected: list[DepotSite],
    current_position: Point,
) -> DepotSite | None:
    if not sites:
        return None
    previous = selected[-1].point if selected else current_position

    def score(site: DepotSite) -> tuple[int, float, float]:
        newly_covered = len(coverage[site.id] & uncovered)
        travel = previous.distance_to(site.point)
        deploy_quality = site.usable_area_m2 - site.slope_risk * 200.0
        return newly_covered, -travel, deploy_quality

    best = max(sites, key=score)
    return best if coverage[best.id] & uncovered else None


def _order_sites_by_nearest_route(sites: list[DepotSite], start: Point) -> list[DepotSite]:
    remaining = list(sites)
    ordered: list[DepotSite] = []
    current = start
    while remaining:
        site = min(remaining, key=lambda item: current.distance_to(item.point))
        ordered.append(site)
        remaining.remove(site)
        current = site.point
    return ordered

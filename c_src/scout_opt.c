#include "scout_opt.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SO_ANGLE_CANDIDATE_COUNT SO_STRIP_ANGLE_CANDIDATE_POOL
#define SO_UAV_MAX_TRACK_PASSES 240
#define SO_INTER_FIELD_EMPTY_PENALTY_SCALE 0.855

static double so_distance(SoPoint a, SoPoint b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

static double so_random_effective_chemical_l_per_ha(void) {
    return 18.0;
}

static SoPlannerWeights so_default_planner_weights(void) {
    SoPlannerWeights w;
    memset(&w, 0, sizeof(w));
    w.strip_empty_w = 1.0;
    w.strip_row_w = 1.8;
    w.strip_crosswind_w = 80.0;
    w.strip_avg_row_bonus_w = 0.08;
    w.global_empty_w = 0.82;
    w.global_row_w = 1.25;
    w.global_crosswind_w = 120.0;
    w.global_avg_row_bonus_w = 0.10;
    w.angle_transition_empty_w = 0.18;
    w.angle_transition_turn_w = 0.42;
    w.angle_transition_change_w = 0.55;
    w.angle_endpoint_empty_w = 0.16;
    w.angle_endpoint_turn_w = 0.48;
    w.uav_empty_w = SO_INTER_FIELD_EMPTY_PENALTY_SCALE / 12.0;
    w.uav_return_w = SO_INTER_FIELD_EMPTY_PENALTY_SCALE / 14.0;
    w.uav_risk_w = 45.0;
    w.uav_route_eff_bonus_w = 16.0;
    w.uav_phase_boundary_w = 36.0;
    w.uav_phase_repair_w = 54.0;
    w.bundle_empty_w = SO_INTER_FIELD_EMPTY_PENALTY_SCALE / 8.0;
    w.bundle_risk_w = 20.0;
    w.bundle_route_eff_bonus_w = 18.0;
    w.bundle_boundary_delay_w = 42.0;
    w.assist_entry_w = 0.10;
    w.assist_overfit_w = 46.0;
    w.assist_underfit_w = 1.2;
    w.assist_collaborator_w = 32.0;
    w.assist_risk_w = 24.0;
    w.assist_uncommitted_bonus_w = 20.0;
    w.assist_route_eff_bonus_w = 8.0;
    return w;
}

static double so_planner_rand01(unsigned int *state) {
    *state = *state * 1664525u + 1013904223u;
    return (double)((*state >> 8) & 0x00FFFFFFu) / 16777216.0;
}

static double so_jitter_weight(double base, double ratio, unsigned int *state) {
    const double u = so_planner_rand01(state);
    const double factor = 1.0 + ratio * (2.0 * u - 1.0);
    return base * fmax(0.05, factor);
}

static SoPlannerWeights so_sample_planner_weights(SoPlannerWeights base,
                                                  unsigned int seed,
                                                  int trial) {
    if (trial <= 0) {
        return base;
    }

    unsigned int state =
        seed ^ (0x9E3779B9u * (unsigned int)(trial + 1)) ^ 0xA5A5A5A5u;
    SoPlannerWeights w = base;
    const double normal = 0.18;
    const double broad = 0.25;

    w.strip_empty_w = so_jitter_weight(base.strip_empty_w, broad, &state);
    w.strip_row_w = so_jitter_weight(base.strip_row_w, normal, &state);
    w.strip_crosswind_w = so_jitter_weight(base.strip_crosswind_w, normal, &state);
    w.strip_avg_row_bonus_w = so_jitter_weight(base.strip_avg_row_bonus_w, normal, &state);
    w.global_empty_w = so_jitter_weight(base.global_empty_w, broad, &state);
    w.global_row_w = so_jitter_weight(base.global_row_w, normal, &state);
    w.global_crosswind_w = so_jitter_weight(base.global_crosswind_w, normal, &state);
    w.global_avg_row_bonus_w = so_jitter_weight(base.global_avg_row_bonus_w, normal, &state);
    w.angle_transition_empty_w = so_jitter_weight(base.angle_transition_empty_w, normal, &state);
    w.angle_transition_turn_w = so_jitter_weight(base.angle_transition_turn_w, normal, &state);
    w.angle_transition_change_w = so_jitter_weight(base.angle_transition_change_w, normal, &state);
    w.angle_endpoint_empty_w = so_jitter_weight(base.angle_endpoint_empty_w, normal, &state);
    w.angle_endpoint_turn_w = so_jitter_weight(base.angle_endpoint_turn_w, normal, &state);
    w.uav_empty_w = so_jitter_weight(base.uav_empty_w, broad, &state);
    w.uav_return_w = so_jitter_weight(base.uav_return_w, normal, &state);
    w.uav_risk_w = so_jitter_weight(base.uav_risk_w, normal, &state);
    w.uav_route_eff_bonus_w = so_jitter_weight(base.uav_route_eff_bonus_w, normal, &state);
    w.uav_phase_boundary_w = so_jitter_weight(base.uav_phase_boundary_w, normal, &state);
    w.uav_phase_repair_w = so_jitter_weight(base.uav_phase_repair_w, normal, &state);
    w.bundle_empty_w = so_jitter_weight(base.bundle_empty_w, broad, &state);
    w.bundle_risk_w = so_jitter_weight(base.bundle_risk_w, normal, &state);
    w.bundle_route_eff_bonus_w = so_jitter_weight(base.bundle_route_eff_bonus_w, normal, &state);
    w.bundle_boundary_delay_w = so_jitter_weight(base.bundle_boundary_delay_w, normal, &state);
    w.assist_entry_w = so_jitter_weight(base.assist_entry_w, broad, &state);
    w.assist_overfit_w = so_jitter_weight(base.assist_overfit_w, normal, &state);
    w.assist_underfit_w = so_jitter_weight(base.assist_underfit_w, normal, &state);
    w.assist_collaborator_w = so_jitter_weight(base.assist_collaborator_w, normal, &state);
    w.assist_risk_w = so_jitter_weight(base.assist_risk_w, normal, &state);
    w.assist_uncommitted_bonus_w = so_jitter_weight(base.assist_uncommitted_bonus_w, normal, &state);
    w.assist_route_eff_bonus_w = so_jitter_weight(base.assist_route_eff_bonus_w, normal, &state);
    return w;
}

typedef struct {
    double angle_deg;
    double score;
    double work_m;
    double empty_m;
    int row_count;
} SoStripAngleCandidate;

typedef enum {
    SO_PATH_STRATEGY_MULTI_ISLAND = 0,
    SO_PATH_STRATEGY_PARTITION_DP = 1,
    SO_PATH_STRATEGY_LONGEST_CORRIDOR = 2
} SoPathStrategy;

typedef struct {
    SoPathStrategy strategy;
    const char *name;
    bool selected[SO_MAX_TASKS];
    double fixed_area_ha;
    double uav_fallback_area_ha;
    double fixed_route_cost_usd;
    double uav_fallback_cost_usd;
    double total_cost_usd;
    double estimated_time_h;
    double work_m;
    double empty_m;
    double turn_m;
    int row_count;
    int turn_count;
    int order[SO_MAX_TASKS];
    int order_count;
    double score;
} SoFixedWingPlanCandidate;

static bool so_mothership_service_busy(const SoSimulation *sim);
static int so_cleanup_open_near(const SoSimulation *sim, SoPoint point, double radius);
static int so_active_field_drone_count(const SoSimulation *sim);
static int so_active_charger_slot_count(const SoSimulation *sim);
static int so_active_drone_count_for_block(const SoSimulation *sim, int block_id);
static double so_active_assigned_area_for_task(const SoSimulation *sim, int task_id);
static double so_block_perimeter_m(const SoFieldBlock *block);
static double so_hive_route_distance(const SoSimulation *sim, SoPoint a, SoPoint b);
static bool so_consume_launch_landing_service(SoSimulation *sim, double seconds);
static bool so_consume_charger_handling_service(SoSimulation *sim, double seconds);
static void so_reset_service_budgets(SoSimulation *sim);
static double so_random_uav_service_s(SoSimulation *sim);
static void so_event(SoSimulation *sim, const char *message);
static const SoFieldBlock *so_find_block_const(const SoSimulation *sim, int block_id);
static SoPoint so_point(double x, double y);
static void so_choose_global_strip_angles(SoSimulation *sim, double out_angles[SO_MAX_BLOCKS]);
static void so_block_projection_range(const SoFieldBlock *block,
                                      double angle_deg,
                                      double *out_min_cross,
                                      double *out_max_cross);
static int so_line_block_intervals(const SoFieldBlock *block,
                                   double angle_rad,
                                   double cross,
                                   double *mins,
                                   double *maxs,
                                   int max_intervals);
static bool so_uav_marked_pass_segment(const SoFieldBlock *block,
                                       double angle_rad,
                                       double swath,
                                       double base_cross,
                                       double route_min_t,
                                       double route_max_t,
                                       int pass_index,
                                       int pass_count,
                                       SoPoint *out_start,
                                       SoPoint *out_end);
static bool so_segments_intersect(SoPoint a, SoPoint b, SoPoint c, SoPoint d);
static bool so_fixed_wing_candidate_task_route(const SoFieldTask *task,
                                               double area_ha,
                                               double swath_m,
                                               SoPoint *start,
                                               SoPoint *end);
static void so_add_routed_task(SoSimulation *sim,
                               int zone_id,
                               int block_id,
                               SoPoint start,
                               SoPoint end,
                               double area_ha,
                               double priority,
                               double risk,
                               double angle,
                               double route_efficiency,
                               SoTaskKind kind);

static double so_angle_diff_rad(double a, double b) {
    double diff = fmod(fabs(a - b), M_PI);
    if (diff > M_PI / 2.0) {
        diff = M_PI - diff;
    }
    return diff;
}

static double so_heading_diff_rad(double a, double b) {
    double diff = fmod(fabs(a - b), 2.0 * M_PI);
    if (diff > M_PI) {
        diff = 2.0 * M_PI - diff;
    }
    return diff;
}

static double so_heading_between(SoPoint a, SoPoint b) {
    return atan2(b.y - a.y, b.x - a.x);
}

static double so_route_curve_limit_deg(bool fixed_wing) {
    return fixed_wing ? 3.0 : 5.0;
}

static double so_curve_length_factor(double curve_deg) {
    const double theta = fabs(curve_deg) * M_PI / 180.0;
    if (theta < 1e-6) {
        return 1.0;
    }
    return theta / fmax(1e-6, 2.0 * sin(theta * 0.5));
}

static SoPoint so_curved_midpoint(SoPoint start, SoPoint end, double curve_deg) {
    SoPoint mid = so_point((start.x + end.x) * 0.5, (start.y + end.y) * 0.5);
    const double theta = curve_deg * M_PI / 180.0;
    if (fabs(theta) < 1e-6) {
        return mid;
    }
    const double chord = so_distance(start, end);
    const double sagitta = chord * tan(theta * 0.25) * 0.5;
    const double heading = so_heading_between(start, end);
    mid.x += -sin(heading) * sagitta;
    mid.y += cos(heading) * sagitta;
    return mid;
}

static void so_store_task_route(SoFieldTask *task,
                                SoPoint start,
                                SoPoint end,
                                double curve_deg) {
    task->has_planned_route = true;
    task->route_start = start;
    task->route_end = end;
    task->route_curve_deg = curve_deg;
    task->strip_angle_deg = so_heading_between(start, end) * 180.0 / M_PI;
    if (fabs(curve_deg) > 0.01) {
        task->has_route_mid = true;
        task->route_mid = so_curved_midpoint(start, end, curve_deg);
    } else {
        task->has_route_mid = false;
        task->route_mid = so_point((start.x + end.x) * 0.5,
                                   (start.y + end.y) * 0.5);
    }
}

static double so_mod2pi(double value) {
    double out = fmod(value, 2.0 * M_PI);
    if (out < 0.0) {
        out += 2.0 * M_PI;
    }
    return out;
}

static double so_shortest_dubins_length(SoPoint from,
                                        double from_heading,
                                        SoPoint to,
                                        double to_heading,
                                        double radius_m) {
    if (radius_m <= 1.0) {
        return so_distance(from, to);
    }
    const double dx = (to.x - from.x) / radius_m;
    const double dy = (to.y - from.y) / radius_m;
    const double d = hypot(dx, dy);
    if (d < 1e-6) {
        return 0.0;
    }
    const double theta = atan2(dy, dx);
    const double alpha = so_mod2pi(from_heading - theta);
    const double beta = so_mod2pi(to_heading - theta);
    const double sa = sin(alpha);
    const double sb = sin(beta);
    const double ca = cos(alpha);
    const double cb = cos(beta);
    const double cab = cos(alpha - beta);
    double best = 1e100;
    double p2;
    double tmp0;
    double tmp1;

    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sa - sb);
    if (p2 >= 0.0) {
        tmp0 = d + sa - sb;
        tmp1 = atan2(cb - ca, tmp0);
        const double len = so_mod2pi(-alpha + tmp1) + sqrt(p2) + so_mod2pi(beta - tmp1);
        best = fmin(best, len);
    }
    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (-sa + sb);
    if (p2 >= 0.0) {
        tmp0 = d - sa + sb;
        tmp1 = atan2(ca - cb, tmp0);
        const double len = so_mod2pi(alpha - tmp1) + sqrt(p2) + so_mod2pi(-beta + tmp1);
        best = fmin(best, len);
    }
    p2 = -2.0 + d * d + 2.0 * cab + 2.0 * d * (sa + sb);
    if (p2 >= 0.0) {
        const double p = sqrt(p2);
        tmp0 = atan2(-ca - cb, d + sa + sb) - atan2(-2.0, p);
        const double len = so_mod2pi(-alpha + tmp0) + p + so_mod2pi(-so_mod2pi(beta) + tmp0);
        best = fmin(best, len);
    }
    p2 = -2.0 + d * d + 2.0 * cab - 2.0 * d * (sa + sb);
    if (p2 >= 0.0) {
        const double p = sqrt(p2);
        tmp0 = atan2(ca + cb, d - sa - sb) - atan2(2.0, p);
        const double len = so_mod2pi(alpha - tmp0) + p + so_mod2pi(beta - tmp0);
        best = fmin(best, len);
    }
    tmp0 = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sa - sb)) / 8.0;
    if (fabs(tmp0) <= 1.0) {
        const double p = so_mod2pi(2.0 * M_PI - acos(tmp0));
        const double t = so_mod2pi(alpha - atan2(ca - cb, d - sa + sb) + p * 0.5);
        const double q = so_mod2pi(alpha - beta - t + p);
        best = fmin(best, t + p + q);
    }
    tmp0 = (6.0 - d * d + 2.0 * cab + 2.0 * d * (-sa + sb)) / 8.0;
    if (fabs(tmp0) <= 1.0) {
        const double p = so_mod2pi(2.0 * M_PI - acos(tmp0));
        const double t = so_mod2pi(-alpha - atan2(ca - cb, d + sa - sb) + p * 0.5);
        const double q = so_mod2pi(beta - alpha - t + p);
        best = fmin(best, t + p + q);
    }
    if (best >= 1e90) {
        return so_distance(from, to) +
               so_heading_diff_rad(from_heading, to_heading) * radius_m;
    }
    return best * radius_m;
}

static SoPoint so_point(double x, double y) {
    SoPoint p;
    p.x = x;
    p.y = y;
    return p;
}

typedef struct {
    double spray_usd;
    double empty_usd;
    double turn_usd;
    double energy_usd;
    double risk_usd;
    double unfinished_usd;
    double total_usd;
    double spray_distance_m;
    double empty_distance_m;
    double turn_distance_m;
    double unfinished_area_ha;
} SoOperationalCost;

typedef struct {
    int pass_count;
    int track_change_count;
    double shift_distance_m;
    double shift_time_s;
    double shift_spray_area_ha;
    double shift_energy_units;
} SoUavTrackPlan;

static double so_task_spray_distance_m(const SoSimulation *sim, double area_ha) {
    return area_ha * 10000.0 / fmax(0.001, sim->spec.spray_swath_m);
}

static int so_uav_pass_count_for_width(double work_width_m, double swath_m) {
    const double swath = fmax(0.001, swath_m);
    if (work_width_m <= swath) {
        return 1;
    }
    const int full_passes = (int)floor(work_width_m / swath);
    const double residual_width = work_width_m - (double)full_passes * swath;
    return fmax(1, full_passes + (residual_width > swath * 0.05 ? 1 : 0));
}

static double so_uav_effective_pass_route_length_m(const SoFieldTask *task,
                                                   double area_ha,
                                                   double route_length_m) {
    const double length = fmax(1.0, route_length_m);
    if (task == NULL || task->kind == SO_TASK_INTERIOR_STRIP || area_ha <= 0.001) {
        return length;
    }
    const double residual_length_floor_m =
        sqrt(fmax(1.0, area_ha * 10000.0)) * 1.6;
    return fmax(length, residual_length_floor_m);
}

static int so_uav_residual_scanline_pass_count(const SoSimulation *sim,
                                               const SoFieldTask *task,
                                               double area_ha,
                                               SoPoint start,
                                               SoPoint end,
                                               double swath,
                                               int max_passes) {
    const SoFieldBlock *block = so_find_block_const(sim, task != NULL ? task->block_id : -1);
    if (task == NULL || task->kind == SO_TASK_INTERIOR_STRIP ||
        block == NULL || block->boundary_count < 3 || area_ha <= 0.001) {
        return 0;
    }
    const double length = so_distance(start, end);
    if (length <= 1.0) {
        return 0;
    }
    const double ux = (end.x - start.x) / length;
    const double uy = (end.y - start.y) / length;
    const double nx = -uy;
    const double ny = ux;
    const double angle_rad = atan2(uy, ux);
    const double base_cross = ((start.x + end.x) * 0.5) * nx +
                              ((start.y + end.y) * 0.5) * ny;
    double min_cross = 0.0;
    double max_cross = 0.0;
    so_block_projection_range(block, angle_rad * 180.0 / M_PI, &min_cross, &max_cross);
    const double step = fabs(base_cross - max_cross) <= fabs(base_cross - min_cross)
                            ? -fabs(swath)
                            : fabs(swath);
    const double target_area_m2 = area_ha * 10000.0;
    double covered_m2 = 0.0;
    int count = 0;
    for (int i = 0; i < max_passes; i++) {
        const double cross = base_cross + step * (double)i;
        if (cross < min_cross - swath || cross > max_cross + swath) {
            break;
        }
        double mins[SO_MAX_BOUNDARY_POINTS / 2];
        double maxs[SO_MAX_BOUNDARY_POINTS / 2];
        const int intervals =
            so_line_block_intervals(block, angle_rad, cross, mins, maxs,
                                    SO_MAX_BOUNDARY_POINTS / 2);
        double longest = 0.0;
        for (int k = 0; k < intervals; k++) {
            longest = fmax(longest, maxs[k] - mins[k]);
        }
        if (longest <= 8.0) {
            continue;
        }
        count++;
        covered_m2 += longest * swath;
        if (covered_m2 >= target_area_m2 * 0.98) {
            break;
        }
    }
    return count;
}

static double so_uav_track_diagonal_factor(const SoSimulation *sim,
                                           const SoFieldTask *task,
                                           double strip_angle_deg) {
    const SoFieldBlock *block = so_find_block_const(sim, task != NULL ? task->block_id : -1);
    if (block == NULL || block->boundary_count < 3) {
        return 1.0;
    }

    const double strip_rad = strip_angle_deg * M_PI / 180.0;
    double best = 1.0;
    for (int i = 0; i < block->boundary_count; i++) {
        const SoPoint a = block->boundary[i];
        const SoPoint b = block->boundary[(i + 1) % block->boundary_count];
        const double edge_len = so_distance(a, b);
        if (edge_len < 8.0) {
            continue;
        }
        const double edge_heading = so_heading_between(a, b);
        const double diff = so_angle_diff_rad(edge_heading, strip_rad);
        const double sin_diff = fabs(sin(diff));
        if (sin_diff < 0.18) {
            continue;
        }
        const double factor = fmin(1.65, fmax(1.0, 1.0 / sin_diff));
        best = fmax(best, factor);
    }
    return best;
}

static bool so_uav_marked_pass_segment(const SoFieldBlock *block,
                                       double angle_rad,
                                       double swath,
                                       double base_cross,
                                       double route_min_t,
                                       double route_max_t,
                                       int pass_index,
                                       int pass_count,
                                       SoPoint *out_start,
                                       SoPoint *out_end) {
    if (block == NULL || block->boundary_count < 3 || out_start == NULL ||
        out_end == NULL || pass_count <= 0 || pass_index < 0 ||
        pass_index >= pass_count || route_max_t - route_min_t <= 1.0) {
        return false;
    }
    const double effective_swath = fmax(0.001, swath);
    const double ux = cos(angle_rad);
    const double uy = sin(angle_rad);
    const double nx = -uy;
    const double ny = ux;
    double min_cross = 0.0;
    double max_cross = 0.0;
    so_block_projection_range(block, angle_rad * 180.0 / M_PI,
                              &min_cross, &max_cross);
    if (max_cross - min_cross <= effective_swath * 0.25) {
        return false;
    }

    const double first_mark = min_cross + effective_swath * 0.5;
    const double center_mark = (base_cross - first_mark) / effective_swath;
    const int first_index =
        (int)floor(center_mark - ((double)pass_count - 1.0) * 0.5 + 0.5);
    const int mark_index = first_index + pass_index;
    const double cross = first_mark + (double)mark_index * effective_swath;
    if (cross < min_cross - effective_swath * 0.25 ||
        cross > max_cross + effective_swath * 0.25) {
        return false;
    }

    double mins[SO_MAX_BOUNDARY_POINTS / 2];
    double maxs[SO_MAX_BOUNDARY_POINTS / 2];
    const int intervals =
        so_line_block_intervals(block, angle_rad, cross, mins, maxs,
                                SO_MAX_BOUNDARY_POINTS / 2);
    int best = -1;
    double best_overlap = 0.0;
    const double clip_pad = fmin(8.0, effective_swath * 0.45);
    for (int k = 0; k < intervals; k++) {
        const double lo = fmax(mins[k] + clip_pad, route_min_t);
        const double hi = fmin(maxs[k] - clip_pad, route_max_t);
        const double overlap = hi - lo;
        if (overlap > best_overlap) {
            best_overlap = overlap;
            best = k;
        }
    }
    if (best < 0 || best_overlap <= 8.0) {
        return false;
    }

    const double a_t = fmax(mins[best] + clip_pad, route_min_t);
    const double b_t = fmin(maxs[best] - clip_pad, route_max_t);
    out_start->x = ux * a_t + nx * cross;
    out_start->y = uy * a_t + ny * cross;
    out_end->x = ux * b_t + nx * cross;
    out_end->y = uy * b_t + ny * cross;
    return true;
}

static SoUavTrackPlan so_uav_track_plan_for_route(const SoSimulation *sim,
                                                  const SoFieldTask *task,
                                                  double area_ha,
                                                  SoPoint start,
                                                  SoPoint end) {
    SoUavTrackPlan plan;
    memset(&plan, 0, sizeof(plan));
    const double swath = fmax(0.001, sim->spec.spray_swath_m);
    const double route_length_m = fmax(1.0, so_distance(start, end));
    plan.pass_count =
        so_uav_residual_scanline_pass_count(sim, task, area_ha, start, end, swath,
                                            SO_UAV_MAX_TRACK_PASSES);
    if (plan.pass_count <= 0) {
        const double pass_route_length_m =
            so_uav_effective_pass_route_length_m(task, area_ha, route_length_m);
        const double work_width_m = area_ha * 10000.0 / fmax(1.0, pass_route_length_m);
        plan.pass_count = so_uav_pass_count_for_width(work_width_m, swath);
    }
    plan.track_change_count = fmax(0, plan.pass_count - 1);
    const double diagonal_factor =
        so_uav_track_diagonal_factor(sim, task,
                                     so_heading_between(start, end) * 180.0 / M_PI);
    plan.shift_distance_m =
        (double)plan.track_change_count * swath * diagonal_factor;
    plan.shift_spray_area_ha = plan.shift_distance_m * swath / 10000.0;
    const double spray_speed_mps =
        sim->spec.spray_rate_ha_h * 10000.0 /
        fmax(1.0, sim->spec.spray_swath_m * 3600.0);
    const double lateral_speed_mps = fmax(1.0, spray_speed_mps * 0.45);
    plan.shift_time_s =
        (double)plan.track_change_count * sim->spec.turn_time_s +
        plan.shift_distance_m / lateral_speed_mps;
    plan.shift_energy_units =
        (double)plan.track_change_count * sim->spec.turn_battery_cost * 0.55 +
        plan.shift_distance_m / 1000.0 * sim->spec.battery_drain_km_empty * 1.15 +
        plan.shift_time_s / 3600.0 * sim->spec.battery_drain_h_work * 0.38;
    return plan;
}

static SoUavTrackPlan so_uav_track_plan_for_task(const SoSimulation *sim,
                                                 const SoFieldTask *task,
                                                 double area_ha) {
    SoPoint start = task != NULL ? task->route_start : so_point(0.0, 0.0);
    SoPoint end = task != NULL ? task->route_end : so_point(1.0, 0.0);
    if (task == NULL || !task->has_planned_route || so_distance(start, end) <= 1.0) {
        const double angle = task != NULL ? task->strip_angle_deg * M_PI / 180.0 : 0.0;
        const double length = fmax(1.0, sqrt(fmax(1.0, area_ha * 10000.0)) * 1.35);
        const SoPoint center = task != NULL ? task->center : so_point(0.0, 0.0);
        const double dx = cos(angle) * length * 0.5;
        const double dy = sin(angle) * length * 0.5;
        start = so_point(center.x - dx, center.y - dy);
        end = so_point(center.x + dx, center.y + dy);
    }
    return so_uav_track_plan_for_route(sim, task, area_ha, start, end);
}

static void so_apply_uav_track_change_metrics(SoSimulation *sim,
                                              SoFieldTask *task,
                                              double area_ha,
                                              SoPoint start,
                                              SoPoint end) {
    SoUavTrackPlan plan = so_uav_track_plan_for_route(sim, task, area_ha, start, end);
    task->turn_count = plan.track_change_count;
    task->turn_time_s = plan.shift_time_s;
    task->turn_energy_cost = plan.shift_energy_units;
}

static double so_uav_track_change_spray_area_ha(const SoSimulation *sim,
                                                const SoFieldTask *task,
                                                double area_ha) {
    SoUavTrackPlan plan = so_uav_track_plan_for_task(sim, task, area_ha);
    return fmin(fmax(0.0, area_ha), fmax(0.0, plan.shift_spray_area_ha));
}

static void so_log_drone_route_point(SoDrone *drone, SoPoint point) {
    if (drone == NULL || drone->route_point_count >= SO_MAX_DRONE_ROUTE_POINTS ||
        drone->route_segment_count <= 0) {
        return;
    }
    if (drone->route_point_count > 0 &&
        so_distance(drone->route_points[drone->route_point_count - 1], point) <= 0.5) {
        return;
    }
    drone->route_points[drone->route_point_count++] = point;
    drone->route_segment_point_count[drone->route_segment_count - 1]++;
}

static double so_point_segment_distance_m(SoPoint p, SoPoint a, SoPoint b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 <= 1e-9) {
        return so_distance(p, a);
    }
    const double t = fmax(0.0, fmin(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
    const SoPoint q = so_point(a.x + dx * t, a.y + dy * t);
    return so_distance(p, q);
}

static bool so_uav_spray_segment_conflicts_fixed_wing(const SoSimulation *sim,
                                                      const SoFieldTask *task,
                                                      SoPoint a,
                                                      SoPoint b) {
    if (sim == NULL || task == NULL || !sim->fixed_wing.enabled ||
        so_distance(a, b) <= 0.5) {
        return false;
    }
    const double clearance_m =
        fmax(2.0, (sim->fixed_wing.swath_width_m + sim->spec.spray_swath_m) * 0.35);
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *fixed = &sim->field.tasks[i];
        if (fixed->block_id != task->block_id || fixed->fixed_wing_area_ha <= 0.001) {
            continue;
        }
        SoPoint fs;
        SoPoint fe;
        if (!so_fixed_wing_candidate_task_route(fixed,
                                                fixed->fixed_wing_area_ha,
                                                sim->fixed_wing.swath_width_m,
                                                &fs, &fe)) {
            continue;
        }
        if (so_segments_intersect(a, b, fs, fe)) {
            return true;
        }
        const double d0 = so_point_segment_distance_m(a, fs, fe);
        const double d1 = so_point_segment_distance_m(b, fs, fe);
        const double d2 = so_point_segment_distance_m(fs, a, b);
        const double d3 = so_point_segment_distance_m(fe, a, b);
        if (fmin(fmin(d0, d1), fmin(d2, d3)) <= clearance_m) {
            return true;
        }
    }
    return false;
}

static void so_begin_drone_route_segment(SoDrone *drone,
                                         SoDroneRouteKind kind,
                                         int block_id,
                                         int task_id,
                                         SoPoint start) {
    if (drone == NULL ||
        drone->route_segment_count >= SO_MAX_DRONE_ROUTE_SEGMENTS ||
        drone->route_point_count >= SO_MAX_DRONE_ROUTE_POINTS) {
        return;
    }
    const int seg = drone->route_segment_count++;
    drone->route_segment_start[seg] = drone->route_point_count;
    drone->route_segment_point_count[seg] = 0;
    drone->route_segment_block_id[seg] = block_id;
    drone->route_segment_task_id[seg] = task_id;
    drone->route_segment_kind[seg] = kind;
    so_log_drone_route_point(drone, start);
}

static void so_log_drone_route_segment(SoDrone *drone,
                                       SoDroneRouteKind kind,
                                       int block_id,
                                       int task_id,
                                       SoPoint start,
                                       SoPoint end) {
    if (drone == NULL || so_distance(start, end) <= 0.5) {
        return;
    }
    if (drone->route_segment_count > 0 && drone->route_point_count > 0) {
        const int seg = drone->route_segment_count - 1;
        const SoPoint last = drone->route_points[drone->route_point_count - 1];
        if (drone->route_segment_kind[seg] == kind &&
            drone->route_segment_block_id[seg] == block_id &&
            drone->route_segment_task_id[seg] == task_id &&
            so_distance(last, start) <= 0.5) {
            so_log_drone_route_point(drone, end);
            return;
        }
    }
    so_begin_drone_route_segment(drone, kind, block_id, task_id, start);
    so_log_drone_route_point(drone, end);
}

static void so_log_drone_transfer_segment(SoDrone *drone, SoPoint start, SoPoint end) {
    so_log_drone_route_segment(drone, SO_DRONE_ROUTE_TRANSFER, -1, -1, start, end);
}

static void so_reset_drone_route_logs(SoSimulation *sim) {
    for (int i = 0; i < sim->drone_count; i++) {
        sim->drones[i].route_point_count = 0;
        sim->drones[i].route_segment_count = 0;
    }
}

static void so_log_uav_task_route(SoSimulation *sim,
                                  SoDrone *drone,
                                  const SoFieldTask *task,
                                  double area_ha,
                                  double reserved_area_before_ha,
                                  SoPoint start,
                                  SoPoint end,
                                  int collaborative_slot) {
    if (drone == NULL || task == NULL) {
        return;
    }
    (void)collaborative_slot;
    const double length = so_distance(start, end);
    if (length <= 1.0) {
        const SoDroneRouteKind kind =
            so_uav_spray_segment_conflicts_fixed_wing(sim, task, start, end)
                ? SO_DRONE_ROUTE_TRANSFER
                : SO_DRONE_ROUTE_SPRAY;
        so_log_drone_route_segment(drone, kind, task->block_id, task->id, start, end);
        return;
    }
    const double swath = fmax(0.001, sim->spec.spray_swath_m);
    const double ux = (end.x - start.x) / length;
    const double uy = (end.y - start.y) / length;
    const double nx = -uy;
    const double ny = ux;
    const double angle_rad = atan2(uy, ux);
    const SoFieldBlock *block = so_find_block_const(sim, task->block_id);
    const double completed_task_area_ha =
        fmax(0.0, fmin(task->area_ha, task->area_ha - task->remaining_ha));
    const double committed_task_area_ha =
        fmax(0.0, fmin(task->area_ha,
                       completed_task_area_ha + fmax(0.0, reserved_area_before_ha)));
    if (task->kind != SO_TASK_INTERIOR_STRIP && block != NULL && block->boundary_count >= 3) {
        double base_cross = ((start.x + end.x) * 0.5) * nx +
                            ((start.y + end.y) * 0.5) * ny;
        double min_cross = 0.0;
        double max_cross = 0.0;
        so_block_projection_range(block, angle_rad * 180.0 / M_PI, &min_cross, &max_cross);
        const double step = fabs(base_cross - max_cross) <= fabs(base_cross - min_cross)
                                ? -swath
                                : swath;
        const int completed_passes =
            (int)floor(committed_task_area_ha * 10000.0 /
                       fmax(1.0, length * swath));
        base_cross += step * (double)completed_passes;
        const double target_area_m2 = area_ha * 10000.0;
        double covered_m2 = 0.0;
        SoPoint prev_end = so_point(0.0, 0.0);
        bool has_prev_end = false;
        for (int i = 0; i < SO_UAV_MAX_TRACK_PASSES; i++) {
            const double cross = base_cross + step * (double)i;
            if (cross < min_cross - swath || cross > max_cross + swath) {
                break;
            }
            double mins[SO_MAX_BOUNDARY_POINTS / 2];
            double maxs[SO_MAX_BOUNDARY_POINTS / 2];
            const int intervals =
                so_line_block_intervals(block, angle_rad, cross, mins, maxs,
                                        SO_MAX_BOUNDARY_POINTS / 2);
            int best = -1;
            double best_len = 0.0;
            for (int k = 0; k < intervals; k++) {
                const double len = maxs[k] - mins[k];
                if (len > best_len) {
                    best_len = len;
                    best = k;
                }
            }
            if (best < 0 || best_len <= 8.0) {
                continue;
            }
            const double margin = fmin(12.0, fmax(0.0, best_len * 0.01));
            SoPoint a = so_point(ux * (mins[best] + margin) + nx * cross,
                                 uy * (mins[best] + margin) + ny * cross);
            SoPoint b = so_point(ux * (maxs[best] - margin) + nx * cross,
                                 uy * (maxs[best] - margin) + ny * cross);
            if (i % 2 == 1) {
                const SoPoint tmp = a;
                a = b;
                b = tmp;
            }
            if (has_prev_end) {
                const SoDroneRouteKind connector_kind =
                    so_uav_spray_segment_conflicts_fixed_wing(sim, task, prev_end, a)
                        ? SO_DRONE_ROUTE_TRANSFER
                        : SO_DRONE_ROUTE_SPRAY;
                so_log_drone_route_segment(drone, connector_kind, task->block_id, task->id,
                                           prev_end, a);
            }
            const SoDroneRouteKind pass_kind =
                so_uav_spray_segment_conflicts_fixed_wing(sim, task, a, b)
                    ? SO_DRONE_ROUTE_TRANSFER
                    : SO_DRONE_ROUTE_SPRAY;
            so_log_drone_route_segment(drone, pass_kind, task->block_id, task->id, a, b);
            covered_m2 += best_len * swath;
            prev_end = b;
            has_prev_end = true;
            if (covered_m2 >= target_area_m2 * 0.98) {
                return;
            }
        }
        if (has_prev_end) {
            return;
        }
    }
    const double pass_route_length_m =
        so_uav_effective_pass_route_length_m(task, area_ha, length);
    const double work_width_m = area_ha * 10000.0 / fmax(1.0, pass_route_length_m);
    int pass_count = so_uav_pass_count_for_width(work_width_m, swath);
    if (pass_count > SO_UAV_MAX_TRACK_PASSES) {
        pass_count = SO_UAV_MAX_TRACK_PASSES;
    }
    const double route_cross = ((start.x + end.x) * 0.5) * nx +
                               ((start.y + end.y) * 0.5) * ny;
    double base_cross = route_cross;
    const double completed_width_m =
        committed_task_area_ha * 10000.0 / fmax(1.0, pass_route_length_m);
    base_cross += floor(completed_width_m / swath + 0.01) * swath;
    const double committed_offset_m = base_cross - route_cross;
    const double route_min_t = fmin(start.x * ux + start.y * uy,
                                    end.x * ux + end.y * uy);
    const double route_max_t = fmax(start.x * ux + start.y * uy,
                                    end.x * ux + end.y * uy);
    SoPoint prev_end = so_point(0.0, 0.0);
    bool has_prev_end = false;
    for (int i = 0; i < pass_count; i++) {
        const double offset =
            committed_offset_m +
            ((double)i - ((double)pass_count - 1.0) * 0.5) * swath;
        SoPoint a = so_point(start.x + nx * offset, start.y + ny * offset);
        SoPoint b = so_point(end.x + nx * offset, end.y + ny * offset);
        bool marked_segment = false;
        if (block != NULL && block->boundary_count >= 3) {
            marked_segment =
                so_uav_marked_pass_segment(block, angle_rad, swath, base_cross,
                                           route_min_t, route_max_t, i, pass_count,
                                           &a, &b);
        }
        if (i % 2 == 1) {
            const SoPoint tmp = a;
            a = b;
            b = tmp;
        }
        if (!marked_segment && block != NULL && block->boundary_count >= 3) {
            double mins[SO_MAX_BOUNDARY_POINTS / 2];
            double maxs[SO_MAX_BOUNDARY_POINTS / 2];
            const double cross = a.x * nx + a.y * ny;
            const double mid_t = ((a.x + b.x) * 0.5) * ux + ((a.y + b.y) * 0.5) * uy;
            const int intervals =
                so_line_block_intervals(block, angle_rad, cross, mins, maxs,
                                        SO_MAX_BOUNDARY_POINTS / 2);
            int best = -1;
            double best_gap = 1e100;
            for (int k = 0; k < intervals; k++) {
                const double center_t = (mins[k] + maxs[k]) * 0.5;
                const double gap = fabs(center_t - mid_t);
                if (maxs[k] - mins[k] > 8.0 && gap < best_gap) {
                    best_gap = gap;
                    best = k;
                }
            }
            if (best < 0) {
                continue;
            }
            const double margin = fmin(12.0, fmax(0.0, (maxs[best] - mins[best]) * 0.01));
            a = so_point(ux * (mins[best] + margin) + nx * cross,
                         uy * (mins[best] + margin) + ny * cross);
            b = so_point(ux * (maxs[best] - margin) + nx * cross,
                         uy * (maxs[best] - margin) + ny * cross);
            if (i % 2 == 1) {
                const SoPoint tmp = a;
                a = b;
                b = tmp;
            }
        }
        if (has_prev_end) {
            const SoDroneRouteKind connector_kind =
                so_uav_spray_segment_conflicts_fixed_wing(sim, task, prev_end, a)
                    ? SO_DRONE_ROUTE_TRANSFER
                    : SO_DRONE_ROUTE_SPRAY;
            so_log_drone_route_segment(drone, connector_kind, task->block_id, task->id,
                                       prev_end, a);
        }
        const SoDroneRouteKind pass_kind =
            so_uav_spray_segment_conflicts_fixed_wing(sim, task, a, b)
                ? SO_DRONE_ROUTE_TRANSFER
                : SO_DRONE_ROUTE_SPRAY;
        so_log_drone_route_segment(drone, pass_kind, task->block_id, task->id, a, b);
        prev_end = b;
        has_prev_end = true;
    }
}

static double so_weather_risk_factor(SoWeather weather, double terrain_factor) {
    const double wind_factor = fmin(1.0, fmax(0.0, weather.wind_speed_mps - 3.0) / 7.0);
    const double gust_factor = fmin(1.0, fmax(0.0, weather.wind_gust_mps - 5.0) / 8.0);
    const double humidity_factor = weather.humidity < 0.35
                                       ? fmin(1.0, (0.35 - weather.humidity) / 0.35)
                                       : fmin(1.0, fmax(0.0, weather.humidity - 0.80) / 0.20);
    const double rain_factor = fmin(1.0, weather.precipitation_mmph / 2.0);
    return fmax(0.0, 0.22 * wind_factor + 0.22 * gust_factor +
                         0.16 * humidity_factor + 0.28 * rain_factor +
                         0.12 * fmax(0.0, terrain_factor));
}

static SoOperationalCost so_make_operational_cost(double spray_usd,
                                                  double empty_usd,
                                                  double turn_usd,
                                                  double energy_usd,
                                                  double unfinished_usd,
                                                  double base_without_risk,
                                                  double risk_factor,
                                                  double spray_m,
                                                  double empty_m,
                                                  double turn_m,
                                                  double unfinished_ha) {
    SoOperationalCost cost;
    cost.spray_usd = spray_usd;
    cost.empty_usd = empty_usd;
    cost.turn_usd = turn_usd;
    cost.energy_usd = energy_usd;
    cost.unfinished_usd = unfinished_usd;
    cost.risk_usd = base_without_risk * risk_factor;
    cost.total_usd = base_without_risk + cost.risk_usd + unfinished_usd;
    cost.spray_distance_m = spray_m;
    cost.empty_distance_m = empty_m;
    cost.turn_distance_m = turn_m;
    cost.unfinished_area_ha = unfinished_ha;
    return cost;
}

static SoOperationalCost so_uav_operational_task_cost(const SoSimulation *sim,
                                                      const SoDrone *drone,
                                                      const SoFieldTask *task,
                                                      SoPoint recovery_point,
                                                      double task_area_ha,
                                                      double capacity_ha) {
    const double area = fmax(0.0, task_area_ha);
    const double spray_m = area * 10000.0 / fmax(0.001, sim->spec.spray_swath_m);
    const double empty_m = (drone != NULL ? so_distance(drone->position, task->center) : 0.0) +
                           so_distance(task->center, recovery_point);
    const SoUavTrackPlan track_plan =
        so_uav_track_plan_for_task(sim, task, area);
    const double turn_m = track_plan.shift_distance_m;
    const double chemical_usd =
        area * sim->spec.chemical_l_per_ha * sim->spec.chemical_cost_usd_per_l;
    const double spray_operation_usd =
        spray_m / 1000.0 * sim->spec.flight_cost_usd_per_km * 0.62;
    const double spray_usd = chemical_usd + spray_operation_usd;
    const double empty_penalty_m = empty_m * SO_INTER_FIELD_EMPTY_PENALTY_SCALE;
    const double empty_usd = empty_penalty_m / 1000.0 * sim->spec.flight_cost_usd_per_km;
    const double turn_energy_units = track_plan.shift_energy_units;
    const double turn_usd =
        turn_m / 1000.0 * sim->spec.flight_cost_usd_per_km * 0.18;
    const double work_energy_units =
        area / fmax(0.001, sim->spec.spray_rate_ha_h) * sim->spec.battery_drain_h_work;
    const double empty_energy_units =
        empty_penalty_m / 1000.0 * sim->spec.battery_drain_km_empty;
    const double energy_usd =
        (work_energy_units + empty_energy_units + turn_energy_units) *
        sim->spec.battery_capacity_kwh *
        sim->spec.electricity_price_usd_per_kwh;
    const double unfinished_ha = fmax(0.0, area - capacity_ha);
    const double unfinished_usd = unfinished_ha * sim->spec.unfinished_penalty_usd_per_ha;
    const double base = spray_usd + empty_usd + turn_usd + energy_usd;
    const double risk_factor = so_weather_risk_factor(sim->mothership.weather,
                                                      task->risk + sim->field.terrain_complexity * 0.35);
    return so_make_operational_cost(spray_usd, empty_usd, turn_usd, energy_usd,
                                    unfinished_usd, base, risk_factor,
                                    spray_m, empty_m, turn_m, unfinished_ha);
}

static SoOperationalCost so_fixed_wing_operational_task_cost(const SoSimulation *sim,
                                                             const SoFieldTask *task,
                                                             double task_area_ha,
                                                             double ferry_m) {
    const double area = fmax(0.0, task_area_ha);
    const double spray_m = area * 10000.0 / fmax(0.001, sim->fixed_wing.swath_width_m);
    const double turn_angle_rad = M_PI;
    const double turn_m = (double)task->turn_count * turn_angle_rad *
                          fmax(1.0, sim->fixed_wing.turn_radius_m);
    const double chemical_usd =
        area * sim->fixed_wing.chemical_l_per_ha * sim->fixed_wing.chemical_cost_usd_per_l;
    const double spray_operation_usd =
        spray_m / 1000.0 * sim->fixed_wing.flight_cost_usd_per_km * 0.58;
    const double spray_usd = chemical_usd + spray_operation_usd;
    const double empty_usd = ferry_m / 1000.0 * sim->fixed_wing.flight_cost_usd_per_km;
    const double turn_fuel_h = (double)task->turn_count * sim->fixed_wing.turn_fuel_h +
                               turn_m / fmax(0.001, sim->fixed_wing.work_speed_mps) / 3600.0 * 1.18;
    const double turn_usd =
        turn_m / 1000.0 * sim->fixed_wing.flight_cost_usd_per_km * 0.35;
    const double spray_energy_h = spray_m / fmax(0.001, sim->fixed_wing.work_speed_mps) / 3600.0;
    const double empty_energy_h = ferry_m / fmax(0.001, sim->fixed_wing.cruise_speed_mps) / 3600.0;
    const double energy_usd =
        (spray_energy_h + empty_energy_h + turn_fuel_h) * sim->fixed_wing.fuel_cost_usd_per_h;
    const double unfinished_usd = 0.0;
    const double base = spray_usd + empty_usd + turn_usd + energy_usd;
    const double risk_factor = so_weather_risk_factor(sim->mothership.weather,
                                                      task->risk + sim->field.terrain_complexity * 0.45);
    return so_make_operational_cost(spray_usd, empty_usd, turn_usd, energy_usd,
                                    unfinished_usd, base, risk_factor,
                                    spray_m, ferry_m, turn_m, 0.0);
}

static void so_add_uav_flight_cost(SoSimulation *sim, double distance_m) {
    if (distance_m <= 0.0) {
        return;
    }
    sim->uav_flight_distance_m += distance_m;
    sim->uav_flight_cost_usd += distance_m / 1000.0 * sim->spec.flight_cost_usd_per_km;
}

static void so_add_uav_takeoff_cost(SoSimulation *sim) {
    sim->uav_takeoffs++;
    sim->uav_launch_cost_usd += sim->spec.launch_cost_usd;
}

static void so_add_fixed_wing_flight_cost(SoSimulation *sim, double distance_m) {
    if (distance_m <= 0.0) {
        return;
    }
    sim->fixed_wing.flight_distance_m += distance_m;
    sim->fixed_wing.flight_cost_usd +=
        distance_m / 1000.0 * sim->fixed_wing.flight_cost_usd_per_km;
    sim->fixed_wing.total_cost_usd =
        sim->fixed_wing.flight_cost_usd + sim->fixed_wing.airport_cost_usd;
}

static void so_add_fixed_wing_sortie_cost(SoSimulation *sim) {
    if (sim->fixed_wing.aircraft_count <= 0) {
        return;
    }
    const double aircraft = (double)sim->fixed_wing.aircraft_count;
    sim->fixed_wing.airport_cost_usd +=
        aircraft * (sim->fixed_wing.takeoff_cost_usd + sim->fixed_wing.airport_service_cost_usd);
    so_add_fixed_wing_flight_cost(sim, sim->fixed_wing.average_ferry_round_trip_m * aircraft);
}

static double so_uav_chemical_area_ha(const SoSimulation *sim) {
    const double fixed_done =
        sim->fixed_wing.enabled ? fmax(0.0, sim->fixed_wing.completed_area_ha) : 0.0;
    return fmax(0.0, fmin(sim->field.area_ha, sim->field.treated_ha) - fixed_done);
}

static double so_fixed_wing_chemical_area_ha(const SoSimulation *sim) {
    if (!sim->fixed_wing.enabled) {
        return 0.0;
    }
    return fmax(0.0, fmin(sim->fixed_wing.assigned_area_ha,
                          sim->fixed_wing.completed_area_ha));
}

static double so_direct_mission_cost_usd(const SoSimulation *sim) {
    const double uav_chemical_cost =
        so_uav_chemical_area_ha(sim) *
        sim->spec.chemical_l_per_ha *
        sim->spec.chemical_cost_usd_per_l;
    const double fixed_chemical_cost =
        so_fixed_wing_chemical_area_ha(sim) *
        sim->fixed_wing.chemical_l_per_ha *
        sim->fixed_wing.chemical_cost_usd_per_l;
    const double fixed_fuel_cost =
        sim->fixed_wing.completed_area_ha /
        fmax(0.001, sim->fixed_wing.spray_rate_ha_h) *
        sim->fixed_wing.fuel_cost_usd_per_h;
    return sim->uav_flight_cost_usd +
           sim->uav_launch_cost_usd +
           sim->uav_electricity_cost_usd +
           uav_chemical_cost +
           sim->fixed_wing.flight_cost_usd +
           sim->fixed_wing.airport_cost_usd +
           fixed_chemical_cost +
           fixed_fuel_cost +
           sim->mothership.move_cost_usd +
           sim->mothership.stop_cost_usd;
}

static double so_estimate_uav_final_repair_cost_usd(const SoSimulation *sim,
                                                    double repair_area_ha) {
    if (repair_area_ha <= 0.001) {
        return 0.0;
    }
    const double spray_m =
        repair_area_ha * 10000.0 / fmax(0.001, sim->spec.spray_swath_m);
    const double chemical =
        repair_area_ha *
        sim->spec.chemical_l_per_ha *
        sim->spec.chemical_cost_usd_per_l;
    const double flight =
        spray_m / 1000.0 * sim->spec.flight_cost_usd_per_km;
    const double work_h =
        repair_area_ha / fmax(0.001, sim->spec.spray_rate_ha_h);
    const double energy_units =
        work_h * sim->spec.battery_drain_h_work +
        spray_m / 1000.0 * sim->spec.battery_drain_km_empty * 0.35;
    const double electricity =
        energy_units * sim->spec.battery_capacity_kwh *
        sim->spec.electricity_price_usd_per_kwh;
    return chemical + flight + electricity + sim->spec.launch_cost_usd;
}

static double so_hive_move_cost_usd(const SoSimulation *sim, double distance_m) {
    return distance_m / 1000.0 * sim->mothership.truck_cost_usd_per_km;
}

static void so_event(SoSimulation *sim, const char *message) {
    if (sim->event_count >= SO_MAX_EVENTS) {
        return;
    }
    snprintf(sim->events[sim->event_count], sizeof(sim->events[sim->event_count]),
             "t=%7.0fs %.120s", sim->now_s, message);
    sim->event_count++;
}

static SoWeatherSeverity so_weather_severity(SoWeather weather) {
    if (weather.wind_gust_mps >= 16.0 || weather.precipitation_mmph >= 6.0 || weather.visibility_m < 300.0) {
        return SO_WEATHER_EMERGENCY;
    }
    if (weather.wind_gust_mps >= 13.0 || weather.precipitation_mmph >= 2.0 || weather.visibility_m < 800.0) {
        return SO_WEATHER_SEVERE;
    }
    if (weather.wind_speed_mps >= 7.0 || weather.wind_gust_mps >= 10.0 || weather.precipitation_mmph >= 0.8) {
        return SO_WEATHER_WARNING;
    }
    if (weather.wind_speed_mps >= 5.5 || weather.wind_gust_mps >= 8.0 || weather.precipitation_mmph >= 0.2 ||
        weather.humidity >= 0.9) {
        return SO_WEATHER_WATCH;
    }
    return SO_WEATHER_NORMAL;
}

static double so_weather_interval(SoWeatherSeverity severity) {
    switch (severity) {
        case SO_WEATHER_EMERGENCY:
            return 15.0;
        case SO_WEATHER_SEVERE:
            return 30.0;
        case SO_WEATHER_WARNING:
            return 60.0;
        case SO_WEATHER_WATCH:
            return 90.0;
        case SO_WEATHER_NORMAL:
        default:
            return 120.0;
    }
}

static bool so_spray_allowed(SoWeather weather) {
    return weather.wind_speed_mps <= 7.0 && weather.wind_gust_mps <= 10.0 && weather.precipitation_mmph <= 0.2;
}

static bool so_flight_allowed(SoWeather weather) {
    return weather.wind_gust_mps <= 13.0 && weather.visibility_m >= 800.0 && weather.precipitation_mmph <= 2.0;
}

static SoWeatherAdjustedSpec so_adjust_for_weather(SoDroneSpec spec, SoWeather weather) {
    const double wind_penalty = fmin(0.45, fmax(0.0, weather.wind_speed_mps - 2.0) * 0.055);
    const double gust_penalty = fmin(0.25, fmax(0.0, weather.wind_gust_mps - 5.0) * 0.04);
    const double humidity_bonus = (weather.humidity >= 0.45 && weather.humidity <= 0.75) ? 0.04 : -0.04;
    double spray_factor = fmax(0.35, fmin(1.05, 1.0 - wind_penalty - gust_penalty + humidity_bonus));
    double cruise_factor = fmax(0.55, fmin(1.0, 1.0 - wind_penalty * 0.55 - gust_penalty * 0.35));
    const double rain_penalty = fmin(0.7, weather.precipitation_mmph * 0.18);
    const double humidity_penalty = fmax(0.0, weather.humidity - 0.82) * 0.8;
    double spray_effectiveness = fmax(0.0, fmin(1.0, spray_factor - rain_penalty - humidity_penalty));

    if (!so_spray_allowed(weather)) {
        spray_factor = 0.0;
        spray_effectiveness = 0.0;
    }
    if (!so_flight_allowed(weather)) {
        cruise_factor = 0.0;
    }

    SoWeatherAdjustedSpec adjusted;
    adjusted.scout_rate_ha_h = spec.scout_rate_ha_h * fmax(0.3, cruise_factor);
    adjusted.spray_rate_ha_h = spec.spray_rate_ha_h * spray_factor;
    adjusted.cruise_speed_mps = spec.cruise_speed_mps * cruise_factor;
    adjusted.battery_work_multiplier = 1.0 + wind_penalty * 0.9 + gust_penalty * 0.5;
    adjusted.battery_scout_multiplier = 1.0 + wind_penalty * 0.6;
    adjusted.spray_effectiveness = spray_effectiveness;
    adjusted.spray_allowed = so_spray_allowed(weather);
    adjusted.flight_allowed = so_flight_allowed(weather);
    return adjusted;
}

static void so_update_weather(SoSimulation *sim) {
    if (sim->now_s + 1e-9 < sim->next_weather_update_s) {
        return;
    }

    const double wind = 2.8 + 1.4 * sin(sim->now_s / 900.0);
    const double gust = wind + 1.8 + 0.8 * sin(sim->now_s / 420.0);
    SoWeather weather;
    weather.wind_speed_mps = fmax(0.0, wind);
    weather.wind_gust_mps = fmax(wind, gust);
    weather.temperature_c = 26.0 + 2.0 * sin(sim->now_s / 2400.0);
    weather.humidity = fmax(0.2, fmin(0.95, 0.55 + 0.12 * sin(sim->now_s / 1800.0)));
    weather.precipitation_mmph = 0.0;
    weather.visibility_m = 5000.0;
    weather.wind_direction_deg = fmod(70.0 + 35.0 * sin(sim->now_s / 2100.0), 360.0);
    weather.updated_at_s = sim->now_s;
    sim->mothership.weather = weather;

    char msg[160];
    snprintf(msg, sizeof(msg), "weather updated severity=%s wind=%.1fm/s gust=%.1fm/s dir=%.0fdeg humidity=%.2f",
             so_weather_severity_name(so_weather_severity(weather)), weather.wind_speed_mps,
             weather.wind_gust_mps, weather.wind_direction_deg, weather.humidity);
    so_event(sim, msg);
    sim->next_weather_update_s = sim->now_s + so_weather_interval(so_weather_severity(weather));
}

static double so_estimate_return_energy(const SoDrone *drone, SoPoint depot, SoDroneSpec spec) {
    const double drain_km = drone != NULL && drone->sortie_battery_drain_km_empty > 0.0
                                ? drone->sortie_battery_drain_km_empty
                                : spec.battery_drain_km_empty;
    return so_distance(drone->position, depot) / 1000.0 * drain_km;
}

static double so_drone_work_drain_h(const SoDrone *drone, const SoDroneSpec *spec) {
    return drone != NULL && drone->sortie_battery_drain_h_work > 0.0
               ? drone->sortie_battery_drain_h_work
               : spec->battery_drain_h_work;
}

static double so_drone_scout_drain_h(const SoDrone *drone, const SoDroneSpec *spec) {
    return drone != NULL && drone->sortie_battery_drain_h_scout > 0.0
               ? drone->sortie_battery_drain_h_scout
               : spec->battery_drain_h_scout;
}

static double so_drone_empty_drain_km(const SoDrone *drone, const SoDroneSpec *spec) {
    return drone != NULL && drone->sortie_battery_drain_km_empty > 0.0
               ? drone->sortie_battery_drain_km_empty
               : spec->battery_drain_km_empty;
}

static double so_drone_chemical_per_ha(const SoDrone *drone, const SoDroneSpec *spec) {
    return drone != NULL && drone->sortie_chemical_per_ha > 0.0
               ? drone->sortie_chemical_per_ha
               : spec->chemical_per_ha;
}

static double so_drone_battery_capacity_kwh(const SoDrone *drone, const SoDroneSpec *spec) {
    return drone != NULL && drone->sortie_battery_capacity_kwh > 0.0
               ? drone->sortie_battery_capacity_kwh
               : spec->battery_capacity_kwh;
}

static SoDroneSpec so_t200_spec_for_modules(SoDroneSpec base, int modules) {
    if (modules < base.min_battery_modules) {
        modules = base.min_battery_modules;
    }
    if (modules > base.max_battery_modules) {
        modules = base.max_battery_modules;
    }
    base.battery_modules = modules;
    base.battery_capacity_kwh =
        (double)modules * base.battery_module_capacity_kwh;
    const int extra_modules = modules > 1 ? modules - 1 : 0;
    const double available_kg =
        base.modeled_payload_capacity_kg -
        base.spray_system_weight_kg -
        (double)extra_modules * base.battery_module_weight_kg;
    base.chemical_tank_l =
        fmax(40.0, fmin(base.chemical_tank_max_l,
                        available_kg / fmax(0.001, base.chemical_density_kg_per_l)));
    base.selected_payload_kg =
        base.chemical_tank_l * base.chemical_density_kg_per_l +
        base.spray_system_weight_kg +
        (double)extra_modules * base.battery_module_weight_kg;
    base.work_power_kw =
        48.0 + (double)modules * 1.8 + base.chemical_tank_l * 0.045;
    base.scout_power_kw =
        20.0 + (double)modules * 1.1;
    base.battery_drain_h_work =
        base.work_power_kw / fmax(0.001, base.battery_capacity_kwh);
    base.battery_drain_h_scout =
        base.scout_power_kw / fmax(0.001, base.battery_capacity_kwh);
    base.battery_drain_km_empty =
        (18.0 + (double)modules * 0.9) /
        fmax(0.001, base.battery_capacity_kwh) /
        fmax(0.001, base.cruise_speed_mps * 3.6);
    base.chemical_per_ha =
        base.chemical_l_per_ha / fmax(1.0, base.chemical_tank_l);
    base.chemical_tank_area_ha =
        base.chemical_tank_l / fmax(0.001, base.chemical_l_per_ha);
    return base;
}

static double so_sortie_capacity_for_spec(const SoDrone *drone,
                                          SoPoint depot,
                                          const SoDroneSpec *spec,
                                          double *out_return_energy) {
    const double return_energy =
        so_distance(drone->position, depot) / 1000.0 * spec->battery_drain_km_empty;
    const double available_battery =
        fmax(0.0, drone->battery - return_energy - spec->safety_battery_margin);
    const double battery_area =
        available_battery / fmax(0.001, spec->battery_drain_h_work) *
        spec->spray_rate_ha_h;
    const double chemical_area =
        drone->chemical / fmax(0.001, spec->chemical_per_ha);
    if (out_return_energy != NULL) {
        *out_return_energy = return_energy;
    }
    return fmax(0.0, fmin(battery_area, chemical_area));
}

static void so_set_drone_sortie_spec(SoDrone *drone, const SoDroneSpec *spec) {
    drone->sortie_battery_modules = spec->battery_modules;
    drone->sortie_battery_capacity_kwh = spec->battery_capacity_kwh;
    drone->sortie_chemical_tank_l = spec->chemical_tank_l;
    drone->sortie_chemical_per_ha = spec->chemical_per_ha;
    drone->sortie_chemical_tank_area_ha = spec->chemical_tank_area_ha;
    drone->sortie_battery_drain_h_work = spec->battery_drain_h_work;
    drone->sortie_battery_drain_h_scout = spec->battery_drain_h_scout;
    drone->sortie_battery_drain_km_empty = spec->battery_drain_km_empty;
}

static double so_choose_drone_sortie_configuration(SoSimulation *sim,
                                                   SoDrone *drone,
                                                   const SoFieldTask *task,
                                                   SoPoint depot) {
    const int options[3] = {1, 2, 4};
    double best_score = -1e100;
    double best_capacity = 0.0;
    SoDroneSpec best_spec = sim->spec;
    const double target_area =
        task != NULL ? fmax(0.25, task->remaining_ha) : sim->spec.chemical_tank_area_ha;
    const double outbound_m =
        task != NULL ? so_distance(drone->position, task->center) : 0.0;

    for (int i = 0; i < 3; i++) {
        SoDroneSpec candidate = so_t200_spec_for_modules(sim->spec, options[i]);
        double return_energy = 0.0;
        const double capacity =
            so_sortie_capacity_for_spec(drone, depot, &candidate, &return_energy);
        const double assigned = fmin(target_area, capacity);
        const double outbound_energy =
            outbound_m / 1000.0 * candidate.battery_drain_km_empty;
        const double spare =
            drone->battery - return_energy - outbound_energy -
            assigned / fmax(0.001, candidate.spray_rate_ha_h) *
                candidate.battery_drain_h_work -
            candidate.safety_battery_margin;
        const double unmet_penalty = fmax(0.0, target_area - capacity) * 45.0;
        const double payload_penalty =
            fmax(0.0, candidate.selected_payload_kg - candidate.modeled_payload_capacity_kg) * 100.0;
        const double oversize_penalty =
            candidate.battery_modules * 0.20 +
            fmax(0.0, capacity - target_area) * 0.35;
        const double score =
            assigned * 18.0 + spare * 4.0 -
            unmet_penalty - payload_penalty - oversize_penalty;
        if (score > best_score) {
            best_score = score;
            best_capacity = capacity;
            best_spec = candidate;
        }
    }

    so_set_drone_sortie_spec(drone, &best_spec);
    drone->return_energy_required = so_distance(drone->position, depot) / 1000.0 *
                                    best_spec.battery_drain_km_empty;
    drone->remaining_capacity_ha = best_capacity;
    return best_capacity;
}

static double so_estimate_dynamic_capacity(const SoDrone *drone,
                                           SoPoint depot,
                                           SoDroneSpec spec,
                                           double *out_return_energy) {
    const double return_energy = so_estimate_return_energy(drone, depot, spec);
    const double available_battery = fmax(0.0, drone->battery - return_energy - spec.safety_battery_margin);
    const double battery_area = available_battery / so_drone_work_drain_h(drone, &spec) * spec.spray_rate_ha_h;
    const double chemical_area = drone->chemical / so_drone_chemical_per_ha(drone, &spec);
    const double capacity = fmax(0.0, fmin(battery_area, chemical_area));
    if (out_return_energy != NULL) {
        *out_return_energy = return_energy;
    }
    return capacity;
}

static double so_dynamic_capacity(SoDrone *drone, SoPoint depot, SoDroneSpec spec) {
    double return_energy = 0.0;
    const double capacity =
        so_estimate_dynamic_capacity(drone, depot, spec, &return_energy);
    drone->return_energy_required = return_energy;
    drone->remaining_capacity_ha = capacity;
    return capacity;
}

static bool so_task_open(const SoFieldTask *task) {
    return task->status == SO_TASK_PENDING && task->remaining_ha > 0.001;
}

static double so_repair_threshold_ha(const SoSimulation *sim) {
    const double battery_area =
        (0.8 - sim->spec.safety_battery_margin) / sim->spec.battery_drain_h_work * sim->spec.spray_rate_ha_h;
    const double chemical_area = 0.8 / sim->spec.chemical_per_ha;
    return fmax(0.8, fmin(2.5, fmin(battery_area, chemical_area) * 0.22));
}

static bool so_repair_sized_task(const SoSimulation *sim, const SoFieldTask *task) {
    return task->kind == SO_TASK_REPAIR || task->remaining_ha <= so_repair_threshold_ha(sim);
}

static bool so_depot_scarce(const SoSimulation *sim) {
    int deployable = 0;
    for (int i = 0; i < sim->field.depot_count; i++) {
        const SoDepotSite *site = &sim->field.depots[i];
        if (site->road_accessible && site->usable_area_m2 >= 180.0 && site->slope_risk <= 0.65) {
            deployable++;
        }
    }
    return deployable * 2 <= sim->field.block_count;
}

static bool so_regular_corridor_layout(const SoSimulation *sim) {
    int deployable = 0;
    for (int i = 0; i < sim->field.depot_count; i++) {
        const SoDepotSite *site = &sim->field.depots[i];
        if (site->road_accessible && site->usable_area_m2 >= 180.0 && site->slope_risk <= 0.65) {
            deployable++;
        }
    }
    return sim->field.terrain_complexity <= 0.25 &&
           sim->field.obstacle_density <= 0.15 &&
           deployable * 2 >= sim->field.block_count;
}

static double so_task_service_radius(const SoSimulation *sim, const SoFieldTask *task) {
    if (so_repair_sized_task(sim, task)) {
        return so_depot_scarce(sim) ? 5200.0 : 2200.0;
    }
    if (so_regular_corridor_layout(sim)) {
        return 1450.0;
    }
    return so_depot_scarce(sim) ? 5200.0 : 900.0;
}

static double so_working_radius(const SoSimulation *sim) {
    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < sim->drone_count; i++) {
        const SoDrone *drone = &sim->drones[i];
        if (drone->battery > 0.25 && drone->chemical > 0.1) {
            sum += so_estimate_dynamic_capacity(drone, sim->mothership.position,
                                                sim->spec, NULL);
            count++;
        }
    }
    if (count == 0) {
        return 520.0;
    }
    const double avg = sum / count;
    const double queue_pressure = fmax(0.0, (double)(sim->drone_count - sim->mothership.fast_chargers)) * 8.0;
    return fmax(338.0, fmin(900.0, 520.0 + avg * 28.0 - queue_pressure));
}

static int so_open_task_count_within(const SoSimulation *sim, SoPoint point, double radius) {
    int count = 0;
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *task = &sim->field.tasks[i];
        if (task->status != SO_TASK_DONE && task->remaining_ha > 0.001 &&
            so_distance(point, task->center) <= radius) {
            count++;
        }
    }
    return count;
}

static double so_radial_push_radius(const SoSimulation *sim) {
    const int near_open = so_open_task_count_within(sim, sim->mothership.position, 420.0);
    const int mid_open = so_open_task_count_within(sim, sim->mothership.position, 680.0);
    if (near_open > 2) {
        return 460.0;
    }
    if (mid_open > 3) {
        return 720.0;
    }
    return 900.0;
}

static const SoFieldBlock *so_find_block_const(const SoSimulation *sim, int block_id) {
    for (int i = 0; i < sim->field.block_count; i++) {
        if (sim->field.blocks[i].id == block_id) {
            return &sim->field.blocks[i];
        }
    }
    return NULL;
}

static bool so_block_allows_curved_spray_route(const SoFieldBlock *block) {
    if (block == NULL) {
        return false;
    }
    if (block->boundary_count >= 3) {
        const double area_m2 = fmax(1.0, block->area_ha * 10000.0);
        const double compactness = so_block_perimeter_m(block) / sqrt(area_m2);
        return block->boundary_count > 5 && compactness > 4.18;
    }
    return false;
}

static int so_task_curve_limit_steps(const SoSimulation *sim,
                                     const SoFieldTask *task,
                                     bool fixed_wing) {
    const SoFieldBlock *block = so_find_block_const(sim, task != NULL ? task->block_id : -1);
    if (!so_block_allows_curved_spray_route(block)) {
        return 0;
    }
    return (int)round(so_route_curve_limit_deg(fixed_wing));
}

static double so_fixed_wing_suitability(const SoSimulation *sim, const SoFieldBlock *block) {
    if (block == NULL || block->area_ha < 10.0 || block->risk > 0.78 ||
        sim->field.obstacle_density > 0.72 || sim->field.terrain_complexity > 0.76) {
        return 0.0;
    }

    double score = 0.96 - block->risk * 0.08 -
                   sim->field.obstacle_density * 0.06 -
                   sim->field.terrain_complexity * 0.05;
    if (block->area_ha >= 80.0) {
        score += 0.035;
    } else if (block->area_ha >= 22.0) {
        score += 0.02;
    } else if (block->area_ha < 16.0) {
        score -= 0.14;
    }
    return fmax(0.0, fmin(0.985, score));
}

static int so_line_block_intervals(const SoFieldBlock *block,
                                   double angle_rad,
                                   double cross,
                                   double *mins,
                                   double *maxs,
                                   int max_intervals) {
    if (block == NULL || block->boundary_count < 3 || max_intervals <= 0) {
        return 0;
    }
    const double ux = cos(angle_rad);
    const double uy = sin(angle_rad);
    const double vx = -uy;
    const double vy = ux;
    double hits[SO_MAX_BOUNDARY_POINTS];
    int hit_count = 0;
    for (int i = 0; i < block->boundary_count; i++) {
        const SoPoint a = block->boundary[i];
        const SoPoint b = block->boundary[(i + 1) % block->boundary_count];
        const double ca = a.x * vx + a.y * vy;
        const double cb = b.x * vx + b.y * vy;
        const double denom = cb - ca;
        if (fabs(denom) < 1e-9) {
            continue;
        }
        const double s = (cross - ca) / denom;
        if (s < -1e-6 || s > 1.0 + 1e-6) {
            continue;
        }
        const double x = a.x + (b.x - a.x) * s;
        const double y = a.y + (b.y - a.y) * s;
        const double t = x * ux + y * uy;
        bool duplicate = false;
        for (int h = 0; h < hit_count; h++) {
            if (fabs(hits[h] - t) < 0.05) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate && hit_count < SO_MAX_BOUNDARY_POINTS) {
            hits[hit_count++] = t;
        }
    }
    for (int i = 0; i < hit_count - 1; i++) {
        for (int j = i + 1; j < hit_count; j++) {
            if (hits[j] < hits[i]) {
                const double tmp = hits[i];
                hits[i] = hits[j];
                hits[j] = tmp;
            }
        }
    }
    int count = 0;
    for (int i = 0; i + 1 < hit_count && count < max_intervals; i += 2) {
        if (hits[i + 1] - hits[i] > 6.0) {
            mins[count] = hits[i];
            maxs[count] = hits[i + 1];
            count++;
        }
    }
    return count;
}

static void so_add_uav_drone_electricity_cost(SoSimulation *sim,
                                              const SoDrone *drone,
                                              double battery_units) {
    if (battery_units <= 0.0) {
        return;
    }
    sim->uav_energy_used_battery_units += battery_units;
    sim->uav_energy_used_kwh +=
        battery_units * so_drone_battery_capacity_kwh(drone, &sim->spec);
    sim->uav_electricity_cost_usd +=
        battery_units *
        so_drone_battery_capacity_kwh(drone, &sim->spec) *
        sim->spec.electricity_price_usd_per_kwh;
}

static bool so_fixed_wing_candidate_task_route(const SoFieldTask *task,
                                               double area_ha,
                                               double swath_m,
                                               SoPoint *start,
                                               SoPoint *end) {
    if (task == NULL || area_ha <= 0.001) {
        return false;
    }
    if (task->has_planned_route && so_distance(task->route_start, task->route_end) > 1.0) {
        *start = task->route_start;
        *end = task->route_end;
        return true;
    }
    const double angle = task->strip_angle_deg * M_PI / 180.0;
    const double length =
        area_ha * 10000.0 / fmax(1.0, swath_m);
    const double dx = cos(angle) * length * 0.5;
    const double dy = sin(angle) * length * 0.5;
    *start = so_point(task->center.x - dx, task->center.y - dy);
    *end = so_point(task->center.x + dx, task->center.y + dy);
    return length > 1.0;
}

static const char *so_path_strategy_name(SoPathStrategy strategy) {
    switch (strategy) {
        case SO_PATH_STRATEGY_MULTI_ISLAND:
            return "multi_island";
        case SO_PATH_STRATEGY_PARTITION_DP:
            return "partition_dp";
        case SO_PATH_STRATEGY_LONGEST_CORRIDOR:
            return "longest_corridor";
        default:
            return "unknown";
    }
}

const char *so_optimization_profile_name(SoOptimizationProfile profile) {
    switch (profile) {
        case SO_OPT_PROFILE_TIME:
            return "time_optimal";
        case SO_OPT_PROFILE_COST:
            return "cost_optimal";
        case SO_OPT_PROFILE_BALANCED:
        default:
            return "balanced";
    }
}

static double so_fixed_wing_route_choice_score(const SoSimulation *sim,
                                               SoPathStrategy strategy,
                                               const SoFieldTask *task,
                                               int current_block_id,
                                               double transition_m,
                                               double turn_m,
                                               double strip_m) {
    const double empty_scale =
        sim->planner_weights.global_empty_w / fmax(0.001, 0.82);
    const double turn_scale =
        sim->planner_weights.global_row_w / fmax(0.001, 1.25);
    const double strip_scale =
        sim->planner_weights.global_avg_row_bonus_w / fmax(0.001, 0.10);
    switch (strategy) {
        case SO_PATH_STRATEGY_MULTI_ISLAND:
            return transition_m * empty_scale + turn_m * 0.85 * turn_scale;
        case SO_PATH_STRATEGY_PARTITION_DP: {
            const double block_switch =
                current_block_id >= 0 && current_block_id != task->block_id ? 260.0 : -80.0;
            return transition_m * empty_scale + turn_m * 0.95 * turn_scale + block_switch;
        }
        case SO_PATH_STRATEGY_LONGEST_CORRIDOR:
            return transition_m * 0.72 * empty_scale +
                   turn_m * 0.90 * turn_scale -
                   strip_m * 0.42 * strip_scale;
        default:
            return transition_m * empty_scale + turn_m * turn_scale;
    }
}

static double so_effective_uav_parallel_rate_ha_h(const SoSimulation *sim) {
    const int charger_slot_count = so_active_charger_slot_count(sim);
    const int refill_count = sim->mothership.refill_ports < SO_MAX_REFILL_PORTS
                                 ? sim->mothership.refill_ports
                                 : SO_MAX_REFILL_PORTS;
    const double sortie_area_ha =
        fmax(0.1, fmin(sim->spec.chemical_tank_area_ha,
                       (0.82 - sim->spec.safety_battery_margin) /
                           fmax(0.001, sim->spec.battery_drain_h_work) *
                           sim->spec.spray_rate_ha_h));
    const double work_h = sortie_area_ha / fmax(0.001, sim->spec.spray_rate_ha_h);
    const double charge_h = 0.72 * (12.5 / 60.0);
    const double refill_h = 0.85 / 8.0;
    const double service_h =
        charge_h * (double)sim->drone_count / fmax(1.0, (double)charger_slot_count) +
        refill_h * (double)sim->drone_count / fmax(1.0, (double)refill_count);
    const double cycle_h = work_h + service_h * 0.38 + 0.035;
    const double fleet_rate =
        (double)sim->drone_count * sortie_area_ha / fmax(0.001, cycle_h);
    return fmax(sim->spec.spray_rate_ha_h * 0.85,
                fmin((double)sim->drone_count * sim->spec.spray_rate_ha_h * 0.72,
                     fleet_rate));
}

static void so_select_fixed_wing_fleet(SoSimulation *sim, double eligible_area_ha, double weighted_round_trip_m) {
    if (!sim->fixed_wing.enabled || eligible_area_ha < 20.0) {
        sim->fixed_wing.aircraft_count = 0;
        sim->fixed_wing.model_name[0] = '\0';
        return;
    }

    typedef struct {
        const char *name;
        double swath_m;
        double speed_mps;
        double work_speed_mps;
        double efficiency;
        double setup_s;
        double tank_ha;
        double tank_l;
        double fuel_l;
        double payload_kg;
        double endurance_h;
        double turnaround_s;
        double ferry_s;
        double cost_h;
        double turn_s;
        double turn_fuel_h;
        double flight_usd_km;
        double takeoff_usd;
        double airport_service_usd;
    } AircraftOption;

    const AircraftOption options[] = {
        {"light_fixed_wing", 13.5, 38.0, 38.0, 0.50, 18.0 * 60.0, 36.0, 720.0, 300.0, 1200.0, 2.2, 10.0 * 60.0, 8.0 * 60.0, 0.45, 32.0, 32.0 / 3600.0 * 1.18, 4.20, 75.0, 90.0},
        {"air_tractor_at_502b", 19.8, 68.9, 59.0, 280.0 * 10000.0 / (19.8 * 59.0 * 3600.0), 18.0 * 60.0, 189.3, 1893.0, 644.0, 2450.0, 644.0 / 205.0, 13.0 * 60.0, 10.0 * 60.0, 0.70, 42.0, 42.0 / 3600.0 * 1.22, 6.50, 120.0, 180.0},
        {"large_fixed_wing", 24.3, 50.0, 50.0, 0.56, 28.0 * 60.0, 86.0, 1700.0, 700.0, 2800.0, 3.0, 17.0 * 60.0, 12.0 * 60.0, 1.05, 48.0, 48.0 / 3600.0 * 1.25, 7.80, 150.0, 230.0},
    };

    double best_score = 1e100;
    int best_option = 0;
    int best_count = 1;
    for (int o = 0; o < 3; o++) {
        const double rate = options[o].swath_m * options[o].work_speed_mps * options[o].efficiency * 3600.0 / 10000.0;
        const int max_count = eligible_area_ha > 220.0 ? 2 : 1;
        for (int count = 1; count <= max_count; count++) {
            const double fixed_wing_l_per_ha =
                sim->fixed_wing.chemical_l_per_ha > 0.001
                    ? sim->fixed_wing.chemical_l_per_ha
                    : options[o].tank_l / fmax(0.001, options[o].tank_ha);
            const double tank_area_ha =
                options[o].tank_l / fmax(0.001, fixed_wing_l_per_ha);
            const double sortie_area_by_fuel = options[o].endurance_h * 0.82 * rate;
            const double sortie_area = fmin(tank_area_ha, sortie_area_by_fuel);
            const double sorties = ceil(eligible_area_ha / fmax(0.001, sortie_area * count));
            const double spray_h = eligible_area_ha / fmax(0.001, rate * count);
            const double service_h = fmax(0.0, sorties - 1.0) * options[o].turnaround_s / 3600.0;
            const double setup_h = options[o].setup_s / 3600.0;
            const double route_h = weighted_round_trip_m / fmax(0.001, options[o].speed_mps) / 3600.0;
            const double economic_penalty = options[o].cost_h * count;
            const double route_cost_usd =
                (weighted_round_trip_m / 1000.0) * options[o].flight_usd_km * count;
            const double airport_cost_usd =
                (options[o].takeoff_usd + options[o].airport_service_usd) * count * sorties;
            const double idle_penalty = count > 1 && eligible_area_ha < 360.0 ? 0.35 : 0.0;
            const double score = spray_h + service_h + setup_h + route_h + economic_penalty +
                                 (route_cost_usd + airport_cost_usd) * 0.0008 + idle_penalty;
            if (score < best_score) {
                best_score = score;
                best_option = o;
                best_count = count;
            }
        }
    }

    const AircraftOption *selected = &options[best_option];
    sim->fixed_wing.aircraft_count = best_count;
    snprintf(sim->fixed_wing.model_name, sizeof(sim->fixed_wing.model_name), "%s", selected->name);
    sim->fixed_wing.swath_width_m = selected->swath_m;
    sim->fixed_wing.cruise_speed_mps = selected->speed_mps;
    sim->fixed_wing.work_speed_mps = selected->work_speed_mps;
    sim->fixed_wing.spray_efficiency = selected->efficiency;
    sim->fixed_wing.setup_time_s = selected->setup_s;
    sim->fixed_wing.turnaround_time_s = selected->turnaround_s;
    sim->fixed_wing.turn_time_s = selected->turn_s;
    sim->fixed_wing.turn_fuel_h = selected->turn_fuel_h;
    sim->fixed_wing.planned_turns = 0;
    sim->fixed_wing.tank_l = selected->tank_l;
    sim->fixed_wing.fuel_l = selected->fuel_l;
    sim->fixed_wing.payload_kg = selected->payload_kg;
    sim->fixed_wing.fuel_endurance_h = selected->endurance_h;
    sim->fixed_wing.ferry_time_s = selected->ferry_s;
    sim->fixed_wing.planned_turn_non_spray_time_s = 0.0;
    sim->fixed_wing.turn_non_spray_time_s = 0.0;
    sim->fixed_wing.sortie_remaining_ha = 0.0;
    sim->fixed_wing.fuel_remaining_h = 0.0;
    sim->fixed_wing.service_remaining_s = 0.0;
    sim->fixed_wing.sorties_completed = 0;
    sim->fixed_wing.economic_cost_h = selected->cost_h * best_count;
    sim->fixed_wing.average_ferry_round_trip_m = weighted_round_trip_m;
    sim->fixed_wing.flight_cost_usd_per_km = selected->flight_usd_km;
    sim->fixed_wing.takeoff_cost_usd = selected->takeoff_usd;
    sim->fixed_wing.airport_service_cost_usd = selected->airport_service_usd;
    sim->fixed_wing.effective_chemical_l_per_ha =
        sim->fixed_wing.effective_chemical_l_per_ha > 0.0
            ? sim->fixed_wing.effective_chemical_l_per_ha
            : sim->spec.effective_chemical_l_per_ha;
    sim->fixed_wing.deposition_efficiency =
        sim->fixed_wing.deposition_efficiency > 0.0
            ? sim->fixed_wing.deposition_efficiency
            : 0.60;
    sim->fixed_wing.chemical_l_per_ha =
        sim->fixed_wing.effective_chemical_l_per_ha /
        fmax(0.001, sim->fixed_wing.deposition_efficiency);
    sim->fixed_wing.tank_area_ha =
        sim->fixed_wing.tank_l / fmax(0.001, sim->fixed_wing.chemical_l_per_ha);
    sim->fixed_wing.chemical_cost_usd_per_l = sim->fixed_wing.chemical_cost_usd_per_l > 0.0
                                                  ? sim->fixed_wing.chemical_cost_usd_per_l
                                                  : 1.15;
    sim->fixed_wing.fuel_burn_l_per_h =
        sim->fixed_wing.fuel_burn_l_per_h > 0.0
            ? sim->fixed_wing.fuel_burn_l_per_h
            : 205.0;
    sim->fixed_wing.fuel_price_usd_per_l =
        sim->fixed_wing.fuel_price_usd_per_l > 0.0
            ? sim->fixed_wing.fuel_price_usd_per_l
            : 1.03;
    sim->fixed_wing.fuel_cost_usd_per_h =
        sim->fixed_wing.fuel_burn_l_per_h *
        sim->fixed_wing.fuel_price_usd_per_l;
    sim->fixed_wing.turn_radius_m = sim->fixed_wing.turn_radius_m > 1.0
                                        ? sim->fixed_wing.turn_radius_m
                                        : 185.0;
    sim->fixed_wing.unfinished_penalty_usd_per_ha =
        sim->fixed_wing.unfinished_penalty_usd_per_ha > 0.0
            ? sim->fixed_wing.unfinished_penalty_usd_per_ha
            : 520.0;
    sim->fixed_wing.flight_distance_m = 0.0;
    sim->fixed_wing.flight_cost_usd = 0.0;
    sim->fixed_wing.airport_cost_usd = 0.0;
    sim->fixed_wing.total_cost_usd = 0.0;
    sim->fixed_wing.spray_rate_ha_h =
        selected->swath_m * selected->work_speed_mps * selected->efficiency * 3600.0 / 10000.0 * best_count;
}

static double so_uav_planning_task_cost_usd(const SoSimulation *sim, const SoFieldTask *task) {
    SoDrone virtual_drone;
    memset(&virtual_drone, 0, sizeof(virtual_drone));
    virtual_drone.state = SO_DRONE_IDLE;
    virtual_drone.battery = 1.0;
    virtual_drone.chemical = 1.0;
    virtual_drone.position = sim->mothership.position;
    const double single_sortie_capacity =
        fmax(0.25, fmin(sim->spec.chemical_tank_area_ha,
                        (0.80 - sim->spec.safety_battery_margin) /
                            fmax(0.001, sim->spec.battery_drain_h_work) *
                            sim->spec.spray_rate_ha_h));
    const double sortie_count = ceil(task->remaining_ha / single_sortie_capacity);
    const SoOperationalCost cost =
        so_uav_operational_task_cost(sim, &virtual_drone, task, sim->mothership.position,
                                     task->remaining_ha, task->remaining_ha);
    const double repeated_empty_m =
        fmax(0.0, sortie_count - 1.0) * so_distance(sim->mothership.position, task->center) * 2.0;
    const double repeated_empty_usd =
        repeated_empty_m / 1000.0 * sim->spec.flight_cost_usd_per_km;
    const double service_cycle_usd =
        sortie_count * sim->spec.launch_cost_usd;
    return cost.total_usd + repeated_empty_usd + service_cycle_usd;
}

static bool so_task_matches_fixed_wing_pass_through(const SoSimulation *sim,
                                                    const SoFieldTask *task,
                                                    SoPoint *out_start,
                                                    SoPoint *out_end,
                                                    double *out_angle_deg,
                                                    double *out_connection_m) {
    if (task == NULL || task->kind != SO_TASK_INTERIOR_STRIP ||
        task->remaining_ha <= 0.001 || task->status == SO_TASK_DONE) {
        return false;
    }
    const SoFieldBlock *block = so_find_block_const(sim, task->block_id);
    if (block == NULL || block->area_ha > 20.0 || block->risk > 0.72) {
        return false;
    }

    const double swath = fmax(1.0, sim->fixed_wing.swath_width_m);
    const double max_cross_m = swath * 0.72;
    const double max_extension_m = fmax(650.0, swath * 36.0);
    double best_score = 1e100;
    SoPoint best_start = task->center;
    SoPoint best_end = task->center;
    double best_angle = task->strip_angle_deg;
    double best_connection_m = 0.0;

    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *fixed = &sim->field.tasks[i];
        if (fixed->fixed_wing_area_ha <= 0.001 || !fixed->has_planned_route ||
            fixed->block_id == task->block_id) {
            continue;
        }
        const double dx = fixed->route_end.x - fixed->route_start.x;
        const double dy = fixed->route_end.y - fixed->route_start.y;
        const double length = hypot(dx, dy);
        if (length <= 10.0) {
            continue;
        }
        const double ux = dx / length;
        const double uy = dy / length;
        const double px = task->center.x - fixed->route_start.x;
        const double py = task->center.y - fixed->route_start.y;
        const double along = px * ux + py * uy;
        const double cross = fabs(px * uy - py * ux);
        const double extension =
            along < 0.0 ? -along : (along > length ? along - length : 0.0);
        if (cross > max_cross_m || extension > max_extension_m) {
            continue;
        }

        const double route_len = task->remaining_ha * 10000.0 / swath;
        const SoPoint start = so_point(task->center.x - ux * route_len * 0.5,
                                       task->center.y - uy * route_len * 0.5);
        const SoPoint end = so_point(task->center.x + ux * route_len * 0.5,
                                     task->center.y + uy * route_len * 0.5);
        const double score = cross * 4.0 + extension * 0.35 +
                             block->area_ha * 0.5 + block->risk * 60.0;
        if (score < best_score) {
            best_score = score;
            best_start = start;
            best_end = end;
            best_angle = atan2(uy, ux) * 180.0 / M_PI;
            best_connection_m = extension + cross;
        }
    }

    if (best_score >= 1e90) {
        return false;
    }
    *out_start = best_start;
    *out_end = best_end;
    *out_angle_deg = best_angle;
    *out_connection_m = best_connection_m;
    return true;
}

static bool so_fixed_wing_candidate_eligible(const SoSimulation *sim,
                                             const SoFieldTask *task) {
    if (task == NULL || task->kind != SO_TASK_INTERIOR_STRIP ||
        task->status == SO_TASK_DONE || task->remaining_ha < 0.25) {
        return false;
    }
    const SoFieldBlock *block = so_find_block_const(sim, task->block_id);
    if (so_fixed_wing_suitability(sim, block) < 0.55) {
        return false;
    }
    const double strip_len_m =
        task->remaining_ha * 10000.0 /
        fmax(0.001, sim->fixed_wing.swath_width_m);
    return strip_len_m >= 240.0;
}

static double so_fixed_wing_route_increment_cost_usd(const SoSimulation *sim,
                                                     const SoFieldTask *task,
                                                     double transition_m,
                                                     double turn_m,
                                                     double strip_m,
                                                     bool starts_new_sortie) {
    const double area_ha = task->remaining_ha;
    const double spray_cost =
        area_ha * sim->fixed_wing.chemical_l_per_ha *
        fmax(0.0, sim->fixed_wing.chemical_cost_usd_per_l);
    const double flight_cost =
        (transition_m + strip_m) / 1000.0 *
        fmax(0.0, sim->fixed_wing.flight_cost_usd_per_km);
    const double transition_h =
        transition_m / fmax(0.001, sim->fixed_wing.cruise_speed_mps) / 3600.0;
    const double work_h =
        strip_m / fmax(0.001, sim->fixed_wing.work_speed_mps) / 3600.0;
    const double turn_h =
        turn_m / fmax(0.001, sim->fixed_wing.work_speed_mps) / 3600.0;
    const double fuel_cost =
        (transition_h + work_h + turn_h * 0.18) *
        fmax(0.0, sim->fixed_wing.fuel_cost_usd_per_h);
    const double sortie_cost =
        starts_new_sortie
            ? sim->fixed_wing.takeoff_cost_usd +
                  sim->fixed_wing.airport_service_cost_usd
            : 0.0;
    return spray_cost + flight_cost + fuel_cost + sortie_cost;
}

static double so_fixed_wing_best_transition_for_task(const SoSimulation *sim,
                                                     const SoFieldTask *task,
                                                     SoPoint current,
                                                     double current_heading,
                                                     SoPoint *out_start,
                                                     SoPoint *out_end,
                                                     double *out_heading,
                                                     double *out_transition_m,
                                                     double *out_turn_m,
                                                     double *out_strip_m,
                                                     double *out_curve_deg) {
    SoPoint a;
    SoPoint b;
    if (!so_fixed_wing_candidate_task_route(task, task->remaining_ha,
                                            sim->fixed_wing.swath_width_m,
                                            &a, &b)) {
        return 1e100;
    }

    double best_score = 1e100;
    for (int dir = 0; dir < 2; dir++) {
        const SoPoint start = dir == 0 ? a : b;
        const SoPoint end = dir == 0 ? b : a;
        const double chord_heading = so_heading_between(start, end);
        const int curve_limit = so_task_curve_limit_steps(sim, task, true);
        for (int curve_step = -curve_limit; curve_step <= curve_limit; curve_step++) {
            const double curve_deg = (double)curve_step;
            const double curve_rad = curve_deg * M_PI / 180.0;
            const double start_heading = chord_heading - curve_rad * 0.5;
            const double end_heading = chord_heading + curve_rad * 0.5;
            const double turn_m =
                so_heading_diff_rad(current_heading, start_heading) *
                fmax(1.0, sim->fixed_wing.turn_radius_m);
            const double transition_m =
                so_shortest_dubins_length(current, current_heading,
                                          start, start_heading,
                                          fmax(1.0, sim->fixed_wing.turn_radius_m));
            const double strip_m =
                so_distance(start, end) * so_curve_length_factor(curve_deg);
            const double score = transition_m + turn_m * 0.35 +
                                 strip_m * 0.015;
            if (score < best_score) {
                best_score = score;
                *out_start = start;
                *out_end = end;
                *out_heading = end_heading;
                *out_transition_m = transition_m;
                *out_turn_m = turn_m;
                *out_strip_m = strip_m;
                *out_curve_deg = curve_deg;
            }
        }
    }
    return best_score;
}

static void so_apply_fixed_wing_route_directions(SoSimulation *sim,
                                                 const SoFixedWingPlanCandidate *plan) {
    SoPoint current = sim->fixed_wing.airport;
    double current_heading = 0.0;
    double sortie_area = 0.0;
    int current_block_id = -1;
    const double tank_area = fmax(1.0, sim->fixed_wing.tank_area_ha);
    bool consumed[SO_MAX_TASKS] = {false};

    if (plan->order_count > 0) {
        for (int o = 0; o < plan->order_count; o++) {
            const int idx = plan->order[o];
            if (idx < 0 || idx >= sim->field.task_count || !plan->selected[idx]) {
                continue;
            }
            SoFieldTask *task = &sim->field.tasks[idx];
            if (sortie_area > 0.001 &&
                sortie_area + task->remaining_ha > tank_area) {
                current = sim->fixed_wing.airport;
                current_heading = 0.0;
                sortie_area = 0.0;
                current_block_id = -1;
            }
            SoPoint start;
            SoPoint end;
            double heading = 0.0;
            double transition_m = 0.0;
            double turn_m = 0.0;
            double strip_m = 0.0;
            double curve_deg = 0.0;
            if (so_fixed_wing_best_transition_for_task(
                    sim, task, current, current_heading,
                    &start, &end, &heading,
                    &transition_m, &turn_m, &strip_m, &curve_deg) >= 1e90) {
                continue;
            }
            so_store_task_route(task, start, end, curve_deg);
            current = end;
            current_heading = heading;
            current_block_id = task->block_id;
            sortie_area += task->remaining_ha;
            (void)transition_m;
            (void)turn_m;
            (void)strip_m;
        }
        return;
    }

    for (;;) {
        int best = -1;
        SoPoint best_start = so_point(0.0, 0.0);
        SoPoint best_end = so_point(0.0, 0.0);
        double best_heading = 0.0;
        double best_curve_deg = 0.0;
        double best_score = 1e100;

        for (int i = 0; i < sim->field.task_count; i++) {
            SoFieldTask *task = &sim->field.tasks[i];
            if (!plan->selected[i] || consumed[i]) {
                continue;
            }
            if (sortie_area > 0.001 &&
                sortie_area + task->remaining_ha > tank_area) {
                continue;
            }
            SoPoint start;
            SoPoint end;
            double heading = 0.0;
            double transition_m = 0.0;
            double turn_m = 0.0;
            double strip_m = 0.0;
            double curve_deg = 0.0;
            if (so_fixed_wing_best_transition_for_task(
                    sim, task, current, current_heading,
                    &start, &end, &heading,
                    &transition_m, &turn_m, &strip_m, &curve_deg) >= 1e90) {
                continue;
            }
            const double score =
                so_fixed_wing_route_choice_score(sim, plan->strategy, task,
                                                 current_block_id,
                                                 transition_m, turn_m, strip_m);
            if (score < best_score) {
                best = i;
                best_start = start;
                best_end = end;
                best_heading = heading;
                best_curve_deg = curve_deg;
                best_score = score;
            }
        }

        if (best < 0) {
            bool any_left = false;
            for (int i = 0; i < sim->field.task_count; i++) {
                if (plan->selected[i] && !consumed[i]) {
                    any_left = true;
                    break;
                }
            }
            if (!any_left || sortie_area <= 0.001) {
                break;
            }
            current = sim->fixed_wing.airport;
            current_heading = 0.0;
            sortie_area = 0.0;
            current_block_id = -1;
            continue;
        }

        SoFieldTask *task = &sim->field.tasks[best];
        so_store_task_route(task, best_start, best_end, best_curve_deg);
        current = best_end;
        current_heading = best_heading;
        current_block_id = task->block_id;
        sortie_area += task->remaining_ha;
        consumed[best] = true;
    }
}

static void so_finalize_fixed_wing_plan_metrics(const SoSimulation *sim,
                                                SoFixedWingPlanCandidate *plan) {
    SoPoint current = sim->fixed_wing.airport;
    double current_heading = 0.0;
    double sortie_area = 0.0;
    int current_block_id = -1;
    const double tank_area = fmax(1.0, sim->fixed_wing.tank_area_ha);
    bool consumed[SO_MAX_TASKS] = {false};

    plan->fixed_route_cost_usd = 0.0;
    plan->fixed_area_ha = 0.0;
    plan->work_m = 0.0;
    plan->empty_m = 0.0;
    plan->turn_m = 0.0;
    plan->row_count = 0;
    plan->turn_count = 0;

    if (plan->order_count > 0) {
        for (int o = 0; o < plan->order_count; o++) {
            const int idx = plan->order[o];
            if (idx < 0 || idx >= sim->field.task_count || !plan->selected[idx]) {
                continue;
            }
            const SoFieldTask *task = &sim->field.tasks[idx];
            if (sortie_area > 0.001 &&
                sortie_area + task->remaining_ha > tank_area) {
                const double return_m =
                    so_shortest_dubins_length(current, current_heading,
                                              sim->fixed_wing.airport, 0.0,
                                              fmax(1.0, sim->fixed_wing.turn_radius_m));
                plan->empty_m += return_m;
                plan->fixed_route_cost_usd +=
                    return_m / 1000.0 * sim->fixed_wing.flight_cost_usd_per_km +
                    return_m / fmax(0.001, sim->fixed_wing.cruise_speed_mps) /
                        3600.0 * sim->fixed_wing.fuel_cost_usd_per_h;
                current = sim->fixed_wing.airport;
                current_heading = 0.0;
                sortie_area = 0.0;
                current_block_id = -1;
            }

            SoPoint start;
            SoPoint end;
            double heading = 0.0;
            double transition_m = 0.0;
            double turn_m = 0.0;
            double strip_m = 0.0;
            double curve_deg = 0.0;
            if (so_fixed_wing_best_transition_for_task(
                    sim, task, current, current_heading,
                    &start, &end, &heading,
                    &transition_m, &turn_m, &strip_m, &curve_deg) >= 1e90) {
                continue;
            }

            const bool starts_new_sortie = sortie_area <= 0.001;
            plan->fixed_route_cost_usd +=
                so_fixed_wing_route_increment_cost_usd(sim, task,
                                                       transition_m, turn_m,
                                                       strip_m,
                                                       starts_new_sortie);
            plan->empty_m += fmax(0.0, transition_m - turn_m);
            plan->turn_m += turn_m;
            plan->work_m += strip_m;
            plan->fixed_area_ha += task->remaining_ha;
            plan->row_count++;
            if (turn_m > 1.0) {
                plan->turn_count++;
            }
            current = end;
            current_heading = heading;
            current_block_id = task->block_id;
            sortie_area += task->remaining_ha;
        }
    } else {
    for (;;) {
        int best = -1;
        SoPoint best_end = so_point(0.0, 0.0);
        double best_heading = 0.0;
        double best_transition = 0.0;
        double best_turn = 0.0;
        double best_strip = 0.0;
        double best_score = 1e100;

        for (int i = 0; i < sim->field.task_count; i++) {
            const SoFieldTask *task = &sim->field.tasks[i];
            if (!plan->selected[i] || consumed[i]) {
                continue;
            }
            if (sortie_area > 0.001 &&
                sortie_area + task->remaining_ha > tank_area) {
                continue;
            }
            SoPoint start;
            SoPoint end;
            double heading = 0.0;
            double transition_m = 0.0;
            double turn_m = 0.0;
            double strip_m = 0.0;
            double curve_deg = 0.0;
            if (so_fixed_wing_best_transition_for_task(
                    sim, task, current, current_heading,
                    &start, &end, &heading,
                    &transition_m, &turn_m, &strip_m, &curve_deg) >= 1e90) {
                continue;
            }
            const double score =
                so_fixed_wing_route_choice_score(sim, plan->strategy, task,
                                                 current_block_id,
                                                 transition_m, turn_m, strip_m);
            if (score < best_score) {
                best = i;
                best_end = end;
                best_heading = heading;
                best_transition = transition_m;
                best_turn = turn_m;
                best_strip = strip_m;
                best_score = score;
            }
        }

        if (best < 0) {
            bool any_left = false;
            for (int i = 0; i < sim->field.task_count; i++) {
                if (plan->selected[i] && !consumed[i]) {
                    any_left = true;
                    break;
                }
            }
            if (!any_left) {
                break;
            }
            if (sortie_area > 0.001) {
                const double return_m =
                    so_shortest_dubins_length(current, current_heading,
                                              sim->fixed_wing.airport, 0.0,
                                              fmax(1.0, sim->fixed_wing.turn_radius_m));
                plan->empty_m += return_m;
                plan->fixed_route_cost_usd +=
                    return_m / 1000.0 * sim->fixed_wing.flight_cost_usd_per_km +
                    return_m / fmax(0.001, sim->fixed_wing.cruise_speed_mps) /
                        3600.0 * sim->fixed_wing.fuel_cost_usd_per_h;
            } else {
                break;
            }
            current = sim->fixed_wing.airport;
            current_heading = 0.0;
            sortie_area = 0.0;
            current_block_id = -1;
            continue;
        }

        const SoFieldTask *task = &sim->field.tasks[best];
        const bool starts_new_sortie = sortie_area <= 0.001;
        plan->fixed_route_cost_usd +=
            so_fixed_wing_route_increment_cost_usd(sim, task,
                                                   best_transition, best_turn,
                                                   best_strip,
                                                   starts_new_sortie);
        plan->empty_m += fmax(0.0, best_transition - best_turn);
        plan->turn_m += best_turn;
        plan->work_m += best_strip;
        plan->fixed_area_ha += task->remaining_ha;
        plan->row_count++;
        if (best_turn > 1.0) {
            plan->turn_count++;
        }
        current = best_end;
        current_heading = best_heading;
        current_block_id = task->block_id;
        sortie_area += task->remaining_ha;
        consumed[best] = true;
    }
    }

    if (plan->row_count > 0) {
        const double return_m =
            so_shortest_dubins_length(current, current_heading,
                                      sim->fixed_wing.airport, 0.0,
                                      fmax(1.0, sim->fixed_wing.turn_radius_m));
        plan->empty_m += return_m;
        plan->fixed_route_cost_usd +=
            return_m / 1000.0 * sim->fixed_wing.flight_cost_usd_per_km +
            return_m / fmax(0.001, sim->fixed_wing.cruise_speed_mps) /
                3600.0 * sim->fixed_wing.fuel_cost_usd_per_h;
    }

    const double work_h =
        plan->work_m / fmax(0.001, sim->fixed_wing.work_speed_mps) / 3600.0;
    const double empty_h =
        plan->empty_m / fmax(0.001, sim->fixed_wing.cruise_speed_mps) / 3600.0;
    const double turn_h =
        plan->turn_m / fmax(0.001, sim->fixed_wing.work_speed_mps) / 3600.0;
    const double fixed_time_h =
        (work_h + empty_h + turn_h) /
        fmax(1.0, (double)sim->fixed_wing.aircraft_count);
    const double uav_parallel_rate_ha_h =
        fmax(0.001, so_effective_uav_parallel_rate_ha_h(sim));
    const double uav_fallback_time_h =
        plan->uav_fallback_area_ha / uav_parallel_rate_ha_h;
    plan->estimated_time_h = fmax(fixed_time_h, uav_fallback_time_h);
    plan->total_cost_usd =
        plan->fixed_route_cost_usd + plan->uav_fallback_cost_usd;
    plan->score =
        plan->estimated_time_h * 360.0 +
        plan->total_cost_usd * 0.08 +
        plan->empty_m / 1000.0 * 12.0 +
        (double)plan->turn_count * 1.5;
}

static double so_profile_plan_choice_value(const SoSimulation *sim,
                                           const SoFixedWingPlanCandidate *candidate,
                                           double min_time_h,
                                           double min_cost_usd) {
    if (sim->optimization_profile == SO_OPT_PROFILE_TIME) {
        const double free_cost = min_cost_usd * 1.18 + 1500.0;
        const double cost_overrun = fmax(0.0, candidate->total_cost_usd - free_cost);
        return candidate->estimated_time_h + cost_overrun / 5200.0;
    }
    if (sim->optimization_profile == SO_OPT_PROFILE_COST) {
        const double time_guard_h = min_time_h * 2.35 + 0.75;
        const double time_overrun_h = fmax(0.0, candidate->estimated_time_h - time_guard_h);
        const double total_candidate_area =
            candidate->fixed_area_ha + candidate->uav_fallback_area_ha;
        const double fixed_area_cap =
            fmax(0.0, total_candidate_area * 0.74);
        const double fixed_area_overrun =
            fmax(0.0, candidate->fixed_area_ha - fixed_area_cap);
        return candidate->total_cost_usd +
               time_overrun_h * 420.0 +
               fixed_area_overrun * 1200.0;
    }
    return candidate->score;
}

static double so_profile_plan_internal_value(const SoSimulation *sim,
                                             const SoFixedWingPlanCandidate *candidate) {
    if (sim->optimization_profile == SO_OPT_PROFILE_TIME) {
        return candidate->estimated_time_h * 5200.0 +
               candidate->total_cost_usd * 0.06 +
               candidate->empty_m / 1000.0 * 3.0 *
                   SO_INTER_FIELD_EMPTY_PENALTY_SCALE *
                   sim->planner_weights.global_empty_w / fmax(0.001, 0.82);
    }
    if (sim->optimization_profile == SO_OPT_PROFILE_COST) {
        const double total_area =
            candidate->fixed_area_ha + candidate->uav_fallback_area_ha;
        const double fixed_area_cap = total_area * 0.74;
        const double fixed_area_overrun =
            fmax(0.0, candidate->fixed_area_ha - fixed_area_cap);
        return candidate->total_cost_usd +
               candidate->estimated_time_h * 220.0 +
               fixed_area_overrun * 1200.0;
    }
    return candidate->score;
}

static void so_eval_fixed_wing_greedy_plan(const SoSimulation *sim,
                                           SoPathStrategy strategy,
                                           const int *eligible,
                                           const double *uav_cost,
                                           int eligible_count,
                                           SoFixedWingPlanCandidate *plan) {
    memset(plan, 0, sizeof(*plan));
    plan->strategy = strategy;
    plan->name = so_path_strategy_name(strategy);

    SoPoint current = sim->fixed_wing.airport;
    double current_heading = 0.0;
    double sortie_area = 0.0;
    int current_block_id = -1;
    const double tank_area = fmax(1.0, sim->fixed_wing.tank_area_ha);
    double cost_profile_fixed_area_cap = 1e100;
    if (sim->optimization_profile == SO_OPT_PROFILE_COST) {
        double eligible_area = 0.0;
        for (int e = 0; e < eligible_count; e++) {
            eligible_area += sim->field.tasks[eligible[e]].remaining_ha;
        }
        cost_profile_fixed_area_cap = eligible_area * 0.74;
    }

    for (;;) {
        int best_slot = -1;
        double best_rank = 1e100;
        SoPoint best_start = so_point(0.0, 0.0);
        SoPoint best_end = so_point(0.0, 0.0);
        double best_heading = 0.0;
        double best_transition = 0.0;
        double best_turn = 0.0;
        double best_strip = 0.0;

        for (int e = 0; e < eligible_count; e++) {
            const int idx = eligible[e];
            const SoFieldTask *task = &sim->field.tasks[idx];
            if (plan->selected[idx]) {
                continue;
            }
            if (plan->fixed_area_ha + task->remaining_ha > cost_profile_fixed_area_cap) {
                continue;
            }
            if (sortie_area > 0.001 &&
                sortie_area + task->remaining_ha > tank_area) {
                continue;
            }
            SoPoint start;
            SoPoint end;
            double heading = 0.0;
            double transition_m = 0.0;
            double turn_m = 0.0;
            double strip_m = 0.0;
            double curve_deg = 0.0;
            if (so_fixed_wing_best_transition_for_task(
                    sim, task, current, current_heading,
                    &start, &end, &heading,
                    &transition_m, &turn_m, &strip_m, &curve_deg) >= 1e90) {
                continue;
            }
            const bool starts_new_sortie = sortie_area <= 0.001;
            const double fixed_inc =
                so_fixed_wing_route_increment_cost_usd(sim, task,
                                                       transition_m, turn_m,
                                                       strip_m,
                                                       starts_new_sortie);
            const double fixed_time_h =
                (transition_m / fmax(0.001, sim->fixed_wing.cruise_speed_mps) +
                 strip_m / fmax(0.001, sim->fixed_wing.work_speed_mps) +
                 turn_m / fmax(0.001, sim->fixed_wing.work_speed_mps)) /
                3600.0 / fmax(1.0, (double)sim->fixed_wing.aircraft_count);
            const double uav_time_h =
                task->remaining_ha /
                fmax(0.001, so_effective_uav_parallel_rate_ha_h(sim));
            const double direct_delta = fixed_inc - uav_cost[e];
            const double time_delta_h = fixed_time_h - uav_time_h;
            const double time_saving_h = -time_delta_h;
            const double cost_ratio =
                fixed_inc / fmax(1.0, uav_cost[e]);
            const double cost_premium_per_ha =
                fmax(0.0, direct_delta) /
                fmax(0.05, task->remaining_ha);
            double objective_delta = direct_delta;
            if (sim->optimization_profile == SO_OPT_PROFILE_TIME) {
                if (time_saving_h <= 0.01 ||
                    (direct_delta > 0.0 &&
                     cost_ratio > 3.6 &&
                     cost_premium_per_ha > 220.0)) {
                    continue;
                }
                objective_delta = time_delta_h * 2400.0 + direct_delta * 0.01;
            } else if (sim->optimization_profile == SO_OPT_PROFILE_COST) {
                const bool directly_cheaper = direct_delta < 0.0;
                const bool bounded_time_rescue =
                    time_saving_h >= 0.08 &&
                    cost_ratio <= 4.6 &&
                    cost_premium_per_ha <= 520.0;
                if (!directly_cheaper && !bounded_time_rescue) {
                    continue;
                }
                objective_delta = directly_cheaper
                                      ? direct_delta
                                      : direct_delta * 0.24 - time_saving_h * 980.0;
            } else if (sim->optimization_profile == SO_OPT_PROFILE_BALANCED) {
                objective_delta = direct_delta * 0.18 + time_delta_h * 900.0;
            }
            if (objective_delta >= 0.0) {
                continue;
            }
            double rank = objective_delta;
            if (strategy == SO_PATH_STRATEGY_MULTI_ISLAND) {
                rank += transition_m / 140.0 + turn_m / 220.0;
            } else if (strategy == SO_PATH_STRATEGY_LONGEST_CORRIDOR) {
                const double effective =
                    task->remaining_ha /
                    fmax(0.001, (transition_m + turn_m + strip_m) / 1000.0);
                rank -= effective * 42.0 + strip_m / 55.0;
                rank += task->risk * 300.0 +
                        sim->field.obstacle_density * 250.0;
            } else {
                const double block_switch =
                    current_block_id >= 0 && current_block_id != task->block_id
                        ? 80.0
                        : -20.0;
                rank += block_switch + turn_m / 180.0;
            }
            if (rank < best_rank) {
                best_slot = e;
                best_rank = rank;
                best_start = start;
                best_end = end;
                best_heading = heading;
                best_transition = transition_m;
                best_turn = turn_m;
                best_strip = strip_m;
            }
        }

        if (best_slot < 0) {
            bool any_unselected = false;
            for (int e = 0; e < eligible_count; e++) {
                if (!plan->selected[eligible[e]]) {
                    any_unselected = true;
                    break;
                }
            }
            if (sortie_area > 0.001 && any_unselected) {
                current = sim->fixed_wing.airport;
                current_heading = 0.0;
                sortie_area = 0.0;
                current_block_id = -1;
                continue;
            }
            break;
        }

        const int task_idx = eligible[best_slot];
        plan->selected[task_idx] = true;
        if (plan->order_count < SO_MAX_TASKS) {
            plan->order[plan->order_count++] = task_idx;
        }
        current = best_end;
        current_heading = best_heading;
        current_block_id = sim->field.tasks[task_idx].block_id;
        sortie_area += sim->field.tasks[task_idx].remaining_ha;
        plan->fixed_area_ha += sim->field.tasks[task_idx].remaining_ha;
        (void)best_start;
        (void)best_transition;
        (void)best_turn;
        (void)best_strip;
    }

    for (int e = 0; e < eligible_count; e++) {
        if (!plan->selected[eligible[e]]) {
            plan->uav_fallback_cost_usd += uav_cost[e];
            plan->uav_fallback_area_ha += sim->field.tasks[eligible[e]].remaining_ha;
        }
    }
    so_finalize_fixed_wing_plan_metrics(sim, plan);
}

static void so_eval_fixed_wing_dp_plan(const SoSimulation *sim,
                                       const int *eligible,
                                       const double *uav_cost,
                                       int eligible_count,
                                       SoFixedWingPlanCandidate *plan) {
    if (eligible_count <= 0 || eligible_count > 14) {
        so_eval_fixed_wing_greedy_plan(sim, SO_PATH_STRATEGY_PARTITION_DP,
                                       eligible, uav_cost, eligible_count, plan);
        return;
    }

    const int n = eligible_count;
    const int state_count = 1 << n;
    const int stride = n * 2;
    double *dp = (double *)malloc((size_t)state_count * (size_t)stride * sizeof(double));
    if (dp == NULL) {
        so_eval_fixed_wing_greedy_plan(sim, SO_PATH_STRATEGY_PARTITION_DP,
                                       eligible, uav_cost, eligible_count, plan);
        return;
    }
    for (int i = 0; i < state_count * stride; i++) {
        dp[i] = 1e100;
    }

    SoPoint starts[SO_MAX_TASKS][2];
    SoPoint ends[SO_MAX_TASKS][2];
    double headings[SO_MAX_TASKS][2];
    double strips[SO_MAX_TASKS];
    for (int i = 0; i < n; i++) {
        const SoFieldTask *task = &sim->field.tasks[eligible[i]];
        SoPoint a;
        SoPoint b;
        so_fixed_wing_candidate_task_route(task, task->remaining_ha,
                                           sim->fixed_wing.swath_width_m,
                                           &a, &b);
        starts[i][0] = a;
        ends[i][0] = b;
        starts[i][1] = b;
        ends[i][1] = a;
        strips[i] = so_distance(a, b);
        for (int d = 0; d < 2; d++) {
            headings[i][d] = so_heading_between(starts[i][d], ends[i][d]);
            const double transition_m =
                so_shortest_dubins_length(sim->fixed_wing.airport, 0.0,
                                          starts[i][d], headings[i][d],
                                          fmax(1.0, sim->fixed_wing.turn_radius_m));
            const double turn_m =
                so_heading_diff_rad(0.0, headings[i][d]) *
                fmax(1.0, sim->fixed_wing.turn_radius_m);
            dp[((1 << i) * stride) + i * 2 + d] =
                so_fixed_wing_route_increment_cost_usd(sim, task,
                                                       transition_m, turn_m,
                                                       strips[i], true);
        }
    }

    for (int mask = 1; mask < state_count; mask++) {
        for (int last = 0; last < n; last++) {
            if ((mask & (1 << last)) == 0) {
                continue;
            }
            for (int last_dir = 0; last_dir < 2; last_dir++) {
                const int base = mask * stride + last * 2 + last_dir;
                const double base_cost = dp[base];
                if (base_cost >= 1e90) {
                    continue;
                }
                for (int next = 0; next < n; next++) {
                    if ((mask & (1 << next)) != 0) {
                        continue;
                    }
                    const int next_mask = mask | (1 << next);
                    const SoFieldTask *task = &sim->field.tasks[eligible[next]];
                    for (int next_dir = 0; next_dir < 2; next_dir++) {
                        const double transition_m =
                            so_shortest_dubins_length(ends[last][last_dir],
                                                      headings[last][last_dir],
                                                      starts[next][next_dir],
                                                      headings[next][next_dir],
                                                      fmax(1.0, sim->fixed_wing.turn_radius_m));
                        const double turn_m =
                            so_heading_diff_rad(headings[last][last_dir],
                                                headings[next][next_dir]) *
                            fmax(1.0, sim->fixed_wing.turn_radius_m);
                        const double inc =
                            so_fixed_wing_route_increment_cost_usd(
                                sim, task, transition_m, turn_m,
                                strips[next], false);
                        const int to = next_mask * stride + next * 2 + next_dir;
                        if (base_cost + inc < dp[to]) {
                            dp[to] = base_cost + inc;
                        }
                    }
                }
            }
        }
    }

    double total_uav = 0.0;
    double total_area = 0.0;
    for (int i = 0; i < n; i++) {
        total_uav += uav_cost[i];
        total_area += sim->field.tasks[eligible[i]].remaining_ha;
    }
    const double fixed_area_cap =
        sim->optimization_profile == SO_OPT_PROFILE_COST ? total_area * 0.74 : 1e100;
    int best_mask = 0;
    SoFixedWingPlanCandidate baseline;
    memset(&baseline, 0, sizeof(baseline));
    baseline.strategy = SO_PATH_STRATEGY_PARTITION_DP;
    baseline.name = so_path_strategy_name(SO_PATH_STRATEGY_PARTITION_DP);
    baseline.uav_fallback_cost_usd = total_uav;
    baseline.uav_fallback_area_ha = total_area;
    so_finalize_fixed_wing_plan_metrics(sim, &baseline);
    double best_value = so_profile_plan_internal_value(sim, &baseline);
    for (int mask = 1; mask < state_count; mask++) {
        double fixed_area = 0.0;
        for (int i = 0; i < n; i++) {
            if ((mask & (1 << i)) != 0) {
                fixed_area += sim->field.tasks[eligible[i]].remaining_ha;
            }
        }
        if (fixed_area > fixed_area_cap) {
            continue;
        }
        double best_route = 1e100;
        for (int last = 0; last < n; last++) {
            if ((mask & (1 << last)) == 0) {
                continue;
            }
            for (int dir = 0; dir < 2; dir++) {
                const double return_m =
                    so_shortest_dubins_length(ends[last][dir], headings[last][dir],
                                              sim->fixed_wing.airport, 0.0,
                                              fmax(1.0, sim->fixed_wing.turn_radius_m));
                const double return_cost =
                    return_m / 1000.0 * sim->fixed_wing.flight_cost_usd_per_km +
                    return_m / fmax(0.001, sim->fixed_wing.cruise_speed_mps) /
                        3600.0 * sim->fixed_wing.fuel_cost_usd_per_h;
                const double route = dp[mask * stride + last * 2 + dir] + return_cost;
                if (route < best_route) {
                    best_route = route;
                }
            }
        }
        double fallback = 0.0;
        double fallback_area = 0.0;
        for (int i = 0; i < n; i++) {
            if ((mask & (1 << i)) == 0) {
                fallback += uav_cost[i];
                fallback_area += sim->field.tasks[eligible[i]].remaining_ha;
            }
        }
        SoFixedWingPlanCandidate candidate;
        memset(&candidate, 0, sizeof(candidate));
        candidate.strategy = SO_PATH_STRATEGY_PARTITION_DP;
        candidate.name = so_path_strategy_name(SO_PATH_STRATEGY_PARTITION_DP);
        candidate.uav_fallback_cost_usd = fallback;
        candidate.uav_fallback_area_ha = fallback_area;
        for (int i = 0; i < n; i++) {
            if ((mask & (1 << i)) != 0) {
                candidate.selected[eligible[i]] = true;
            }
        }
        so_finalize_fixed_wing_plan_metrics(sim, &candidate);
        candidate.fixed_route_cost_usd = best_route;
        candidate.total_cost_usd = best_route + fallback;
        const double value = so_profile_plan_internal_value(sim, &candidate);
        if (value < best_value) {
            best_value = value;
            best_mask = mask;
        }
    }

    memset(plan, 0, sizeof(*plan));
    plan->strategy = SO_PATH_STRATEGY_PARTITION_DP;
    plan->name = so_path_strategy_name(SO_PATH_STRATEGY_PARTITION_DP);
    for (int i = 0; i < n; i++) {
        if ((best_mask & (1 << i)) != 0) {
            plan->selected[eligible[i]] = true;
        } else {
            plan->uav_fallback_cost_usd += uav_cost[i];
            plan->uav_fallback_area_ha += sim->field.tasks[eligible[i]].remaining_ha;
        }
    }
    free(dp);
    so_finalize_fixed_wing_plan_metrics(sim, plan);
}

static void so_plan_fixed_wing_coverage(SoSimulation *sim) {
    if (!sim->fixed_wing.enabled || sim->fixed_wing.planned) {
        return;
    }

    double eligible_area = 0.0;
    double weighted_round_trip_m = 0.0;
    int eligible[SO_MAX_TASKS];
    double eligible_uav_cost[SO_MAX_TASKS];
    int eligible_count = 0;

    for (int i = 0; i < sim->field.task_count; i++) {
        SoFieldTask *task = &sim->field.tasks[i];
        if (!so_fixed_wing_candidate_eligible(sim, task)) {
            continue;
        }
        eligible[eligible_count] = i;
        eligible_uav_cost[eligible_count] = so_uav_planning_task_cost_usd(sim, task);
        eligible_count++;
        eligible_area += task->remaining_ha;
        weighted_round_trip_m +=
            task->remaining_ha *
            so_distance(sim->fixed_wing.airport, task->center) * 2.0;
    }

    sim->fixed_wing.effective_chemical_l_per_ha =
        sim->fixed_wing.effective_chemical_l_per_ha > 0.0
            ? sim->fixed_wing.effective_chemical_l_per_ha
            : sim->spec.effective_chemical_l_per_ha;
    sim->fixed_wing.deposition_efficiency =
        sim->fixed_wing.deposition_efficiency > 0.0
            ? sim->fixed_wing.deposition_efficiency
            : 0.60;
    sim->fixed_wing.chemical_l_per_ha =
        sim->fixed_wing.effective_chemical_l_per_ha /
        fmax(0.001, sim->fixed_wing.deposition_efficiency);

    so_select_fixed_wing_fleet(sim, eligible_area,
                               eligible_area > 0.001 ? weighted_round_trip_m / eligible_area : 0.0);
    if (sim->fixed_wing.aircraft_count <= 0) {
        sim->fixed_wing.planned = true;
        return;
    }

    SoFixedWingPlanCandidate candidates[3];
    so_eval_fixed_wing_greedy_plan(sim, SO_PATH_STRATEGY_MULTI_ISLAND,
                                   eligible, eligible_uav_cost,
                                   eligible_count, &candidates[0]);
    so_eval_fixed_wing_dp_plan(sim, eligible, eligible_uav_cost,
                               eligible_count, &candidates[1]);
    so_eval_fixed_wing_greedy_plan(sim, SO_PATH_STRATEGY_LONGEST_CORRIDOR,
                                   eligible, eligible_uav_cost,
                                   eligible_count, &candidates[2]);

    double min_time_h = candidates[0].estimated_time_h;
    double min_cost_usd = candidates[0].total_cost_usd;
    for (int i = 1; i < 3; i++) {
        if (candidates[i].estimated_time_h < min_time_h) {
            min_time_h = candidates[i].estimated_time_h;
        }
        if (candidates[i].total_cost_usd < min_cost_usd) {
            min_cost_usd = candidates[i].total_cost_usd;
        }
    }

    int best = 0;
    double best_choice_value =
        so_profile_plan_choice_value(sim, &candidates[0], min_time_h, min_cost_usd);
    for (int i = 1; i < 3; i++) {
        const double choice_value =
            so_profile_plan_choice_value(sim, &candidates[i], min_time_h, min_cost_usd);
        if (choice_value < best_choice_value) {
            best = i;
            best_choice_value = choice_value;
        }
    }

    SoFixedWingPlanCandidate *chosen = &candidates[best];
    double covered_area = 0.0;
    int planned_turns = 0;
    const double cost_profile_fixed_area_cap =
        sim->optimization_profile == SO_OPT_PROFILE_COST ? eligible_area * 0.74 : 1e100;
    so_apply_fixed_wing_route_directions(sim, chosen);
    for (int e = 0; e < eligible_count; e++) {
        const int idx = eligible[e];
        SoFieldTask *task = &sim->field.tasks[idx];
        if (!chosen->selected[idx]) {
            continue;
        }
        const double fixed_area = task->remaining_ha;
        covered_area += fixed_area;
        planned_turns += task->turn_count;

        task->fixed_wing_area_ha = fixed_area;
        task->remaining_ha = 0.0;
        task->status = SO_TASK_DONE;
    }

    for (int i = 0; i < sim->field.task_count; i++) {
        SoFieldTask *task = &sim->field.tasks[i];
        SoPoint pass_start;
        SoPoint pass_end;
        double pass_angle = 0.0;
        double pass_connection_m = 0.0;
        if (!so_task_matches_fixed_wing_pass_through(sim, task, &pass_start, &pass_end,
                                                     &pass_angle, &pass_connection_m)) {
            continue;
        }
        SoFieldTask pass_task = *task;
        pass_task.turn_count = 0;
        const double fixed_incremental_cost =
            so_fixed_wing_operational_task_cost(
                sim, &pass_task, task->remaining_ha, pass_connection_m).total_usd;
        const double uav_incremental_cost =
            so_uav_planning_task_cost_usd(sim, task);
        if (fixed_incremental_cost >= uav_incremental_cost) {
            continue;
        }

        const double fixed_area = task->remaining_ha;
        if (covered_area + fixed_area > cost_profile_fixed_area_cap) {
            continue;
        }
        covered_area += fixed_area;
        task->fixed_wing_area_ha = fixed_area;
        task->remaining_ha = 0.0;
        task->status = SO_TASK_DONE;
        so_store_task_route(task, pass_start, pass_end, 0.0);
        task->strip_angle_deg = pass_angle;
        task->turn_count = 0;
        task->turn_time_s = 0.0;
        task->turn_energy_cost = 0.0;
    }

    sim->fixed_wing.assigned_area_ha = covered_area;
    snprintf(sim->fixed_wing.path_strategy,
             sizeof(sim->fixed_wing.path_strategy),
             "%s",
             chosen->name);
    sim->fixed_wing.path_strategy_score = chosen->score;
    for (int i = 0; i < 3; i++) {
        sim->fixed_wing.path_strategy_time_h[i] = candidates[i].estimated_time_h;
        sim->fixed_wing.path_strategy_cost_usd[i] = candidates[i].total_cost_usd;
        sim->fixed_wing.path_strategy_scoreboard[i] = candidates[i].score;
    }
    sim->fixed_wing.corridor_count = chosen->row_count;
    sim->fixed_wing.corridor_work_m = chosen->work_m;
    sim->fixed_wing.corridor_empty_m = chosen->empty_m;
    sim->fixed_wing.corridor_total_m =
        chosen->work_m + chosen->empty_m + chosen->turn_m;
    sim->fixed_wing.planned_turns = chosen->turn_count;
    sim->fixed_wing.planned_turn_non_spray_time_s =
        chosen->turn_m / fmax(0.001, sim->fixed_wing.work_speed_mps) /
        fmax(1.0, (double)sim->fixed_wing.aircraft_count);
    sim->fixed_wing.flight_distance_m = 0.0;
    sim->fixed_wing.flight_cost_usd = 0.0;
    sim->fixed_wing.airport_cost_usd = 0.0;
    sim->fixed_wing.total_cost_usd = 0.0;
    so_add_fixed_wing_flight_cost(sim, sim->fixed_wing.corridor_total_m);
    if (sim->fixed_wing.corridor_count <= 0) {
        sim->fixed_wing.planned_turns = planned_turns;
    }
    if (covered_area > 0.001) {
        sim->fixed_wing.ferry_time_s =
            fmax(sim->fixed_wing.ferry_time_s,
                 (weighted_round_trip_m / fmax(0.001, eligible_area)) /
                     fmax(0.001, sim->fixed_wing.cruise_speed_mps));
        sim->fixed_wing.ferry_time_s +=
            sim->fixed_wing.corridor_empty_m /
            fmax(0.001, sim->fixed_wing.cruise_speed_mps) /
            fmax(1.0, (double)sim->fixed_wing.aircraft_count);
        sim->fixed_wing.turn_non_spray_time_s =
            sim->fixed_wing.planned_turn_non_spray_time_s;
    }
    sim->fixed_wing.completed_area_ha = 0.0;
    sim->fixed_wing.planned = true;
    if (covered_area > 0.001) {
        so_event(sim, "fixed-wing main strips assigned");
    }
}

static void so_update_fixed_wing(SoSimulation *sim, SoWeatherAdjustedSpec weather) {
    if (!sim->fixed_wing.enabled || !sim->fixed_wing.planned ||
        sim->fixed_wing.assigned_area_ha <= sim->fixed_wing.completed_area_ha ||
        sim->fixed_wing.aircraft_count <= 0) {
        return;
    }
    if (!weather.flight_allowed || !weather.spray_allowed) {
        return;
    }

    if (sim->fixed_wing.setup_time_s > 0.001) {
        sim->fixed_wing.setup_time_s = fmax(0.0, sim->fixed_wing.setup_time_s - sim->dt_s);
        return;
    }
    if (sim->fixed_wing.service_remaining_s > 0.001) {
        sim->fixed_wing.service_remaining_s = fmax(0.0, sim->fixed_wing.service_remaining_s - sim->dt_s);
        return;
    }

    const double dt_h = sim->dt_s / 3600.0;
    if (sim->fixed_wing.turn_non_spray_time_s > 0.001) {
        const double consumed_s = fmin(sim->dt_s, sim->fixed_wing.turn_non_spray_time_s);
        sim->fixed_wing.turn_non_spray_time_s =
            fmax(0.0, sim->fixed_wing.turn_non_spray_time_s - consumed_s);
        sim->fixed_wing.fuel_remaining_h =
            fmax(0.0, sim->fixed_wing.fuel_remaining_h -
                          consumed_s / 3600.0 * 1.18);
        return;
    }

    const double wind_penalty = fmin(0.22, fmax(0.0, sim->mothership.weather.wind_speed_mps - 4.0) * 0.035);
    if (sim->fixed_wing.sortie_remaining_ha <= 0.001 || sim->fixed_wing.fuel_remaining_h <= 0.001) {
        sim->fixed_wing.sortie_remaining_ha = sim->fixed_wing.tank_area_ha * sim->fixed_wing.aircraft_count;
        const double ferry_fuel_h = sim->fixed_wing.ferry_time_s / 3600.0;
        sim->fixed_wing.fuel_remaining_h =
            fmax(0.0, sim->fixed_wing.fuel_endurance_h - ferry_fuel_h);
        sim->fixed_wing.service_remaining_s =
            sim->fixed_wing.ferry_time_s +
            (sim->fixed_wing.sorties_completed == 0
                 ? 0.0
                 : sim->fixed_wing.turnaround_time_s);
        sim->fixed_wing.sorties_completed++;
        so_add_fixed_wing_sortie_cost(sim);
        return;
    }

    const double possible_by_rate = sim->fixed_wing.spray_rate_ha_h * weather.spray_effectiveness *
                                    (1.0 - wind_penalty) * dt_h;
    const double sortie_turn_reserve = fmin(0.18, sim->fixed_wing.turn_fuel_h * 2.0);
    const double possible_by_fuel = fmax(0.0, sim->fixed_wing.fuel_remaining_h - sortie_turn_reserve) *
                                    sim->fixed_wing.spray_rate_ha_h;
    const double done = fmin(sim->fixed_wing.assigned_area_ha - sim->fixed_wing.completed_area_ha,
                             fmin(sim->fixed_wing.sortie_remaining_ha,
                                  fmin(possible_by_rate, possible_by_fuel)));
    sim->fixed_wing.completed_area_ha += done;
    sim->fixed_wing.sortie_remaining_ha = fmax(0.0, sim->fixed_wing.sortie_remaining_ha - done);
    sim->fixed_wing.fuel_remaining_h = fmax(0.0, sim->fixed_wing.fuel_remaining_h - dt_h);
    sim->field.treated_ha = fmin(sim->field.area_ha, sim->field.treated_ha + done);
}

static double so_next_depot_pull(const SoSimulation *sim, const SoFieldTask *task) {
    if (sim->mothership.operation_plan_index + 1 >= sim->mothership.operation_plan_count) {
        return 0.0;
    }
    const SoPoint next = sim->mothership.operation_plan[sim->mothership.operation_plan_index + 1];
    const double current_dist = so_distance(sim->mothership.position, task->center);
    const double next_dist = so_distance(next, task->center);
    if (current_dist < 650.0 || next_dist > current_dist) {
        return 0.0;
    }
    return fmin(35.0, (current_dist - next_dist) / 12.0);
}

static bool so_zone_has_open_interior(const SoSimulation *sim, int zone_id) {
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *task = &sim->field.tasks[i];
        if (task->zone_id == zone_id && task->kind == SO_TASK_INTERIOR_STRIP &&
            task->status != SO_TASK_DONE && task->remaining_ha > 0.001) {
            return true;
        }
    }
    return false;
}

static bool so_task_allowed_in_moving_window(const SoSimulation *sim, const SoFieldTask *task) {
    if (so_repair_sized_task(sim, task)) {
        return true;
    }
    if (!sim->mothership.moving) {
        return true;
    }
    return so_distance(sim->mothership.destination, task->center) <= 900.0;
}

static int so_choose_task_for_drone(SoSimulation *sim, SoDrone *drone, double queue_pressure, bool moving_window) {
    const double radius = so_working_radius(sim);
    const double push_radius = so_radial_push_radius(sim);
    double best_score = 1e100;
    int best = -1;

    for (int i = 0; i < sim->field.task_count; i++) {
        SoFieldTask *task = &sim->field.tasks[i];
        if (!so_task_open(task)) {
            continue;
        }
        if (moving_window && !so_task_allowed_in_moving_window(sim, task)) {
            continue;
        }

        const double depot_dist = so_distance(sim->mothership.position, task->center);
        if (depot_dist > so_task_service_radius(sim, task)) {
            continue;
        }

        const double capacity =
            so_estimate_dynamic_capacity(drone, sim->mothership.position,
                                         sim->spec, NULL);
        if (capacity < 0.05) {
            continue;
        }

        const double empty_dist = so_distance(drone->position, task->center);
        const double return_dist = so_distance(task->center, sim->mothership.position);
        const double over_capacity = fmax(0.0, task->remaining_ha - capacity) * 60.0;
        const double underuse = fmax(0.0, capacity - task->remaining_ha) * 0.8;
        const double radius_penalty = fmax(0.0, depot_dist - radius) / 8.0;
        const double push_penalty = fmax(0.0, depot_dist - push_radius) / 3.5;
        const double repair_bonus = so_repair_sized_task(sim, task) ? 28.0 : 0.0;
        const double next_depot_bonus = so_next_depot_pull(sim, task);
        const int active_in_block = so_active_drone_count_for_block(sim, task->block_id);
        const double block_load_penalty =
            (double)active_in_block *
            (task->remaining_ha <= 1.25 ? 26.0 :
             (task->kind == SO_TASK_BOUNDARY ? 18.0 : 8.0));
        const double resource_pressure =
            (drone->battery < 0.55 ? (0.55 - drone->battery) * 55.0 : 0.0) +
            (drone->chemical < 0.35 ? (0.35 - drone->chemical) * 75.0 : 0.0);
        const double low_resource_task_penalty =
            resource_pressure *
            (empty_dist / 600.0 + fmax(0.0, task->remaining_ha - capacity * 0.85));
        const double capacity_ratio = task->remaining_ha / fmax(0.05, capacity);
        const double fit_penalty =
            task->kind == SO_TASK_INTERIOR_STRIP && capacity_ratio < 0.18
                ? (0.18 - capacity_ratio) * 70.0
                : 0.0;
        const double cluster_bonus =
            fmin(18.0, (double)so_open_task_count_within(sim, task->center, 260.0) * 2.6);
        double phase_penalty = 0.0;
        if (so_zone_has_open_interior(sim, task->zone_id)) {
            if (task->kind == SO_TASK_BOUNDARY) {
                phase_penalty += sim->planner_weights.uav_phase_boundary_w;
            } else if (task->kind == SO_TASK_REPAIR) {
                phase_penalty += sim->planner_weights.uav_phase_repair_w;
            }
        } else if (task->kind == SO_TASK_REPAIR) {
            phase_penalty += sim->planner_weights.uav_phase_repair_w * (12.0 / 54.0);
        }
        const SoOperationalCost operational_cost =
            so_uav_operational_task_cost(sim, drone, task, sim->mothership.position,
                                         task->remaining_ha, capacity);
        const double drone_economic_penalty = operational_cost.total_usd * 0.18;
        const double strip_bonus =
            task->kind == SO_TASK_INTERIOR_STRIP
                ? sim->planner_weights.uav_route_eff_bonus_w * task->route_efficiency
                : 0.0;
        const double score =
            empty_dist * sim->planner_weights.uav_empty_w +
            return_dist * sim->planner_weights.uav_return_w +
            over_capacity + underuse +
                             task->risk * sim->planner_weights.uav_risk_w +
                             queue_pressure + radius_penalty + push_penalty + phase_penalty +
                             drone_economic_penalty + block_load_penalty + low_resource_task_penalty +
                             fit_penalty -
                             task->priority * 8.0 - repair_bonus - next_depot_bonus -
                             strip_bonus - cluster_bonus;

        if (score < best_score) {
            best_score = score;
            best = i;
        }
    }
    return best;
}

static bool so_site_deployable(const SoField *field, const SoDepotSite *site) {
    if (!site->road_accessible || site->usable_area_m2 < 180.0 || site->slope_risk > 0.65) {
        return false;
    }
    for (int b = 0; b < field->block_count; b++) {
        const SoFieldBlock *block = &field->blocks[b];
        if (!block->selected || block->boundary_count < 3) {
            continue;
        }
        bool inside = false;
        int j = block->boundary_count - 1;
        for (int i = 0; i < block->boundary_count; i++) {
            const SoPoint a = block->boundary[i];
            const SoPoint c = block->boundary[j];
            const double dy = c.y - a.y;
            const bool crosses = fabs(dy) > 1e-9 &&
                                 ((a.y > site->point.y) != (c.y > site->point.y)) &&
                                 (site->point.x < (c.x - a.x) * (site->point.y - a.y) / dy + a.x);
            if (crosses) {
                inside = !inside;
            }
            j = i;
        }
        if (inside) {
            return false;
        }
    }
    return true;
}

static bool so_point_in_block_polygon(SoPoint point, const SoFieldBlock *block) {
    if (block == NULL || !block->selected || block->boundary_count < 3) {
        return false;
    }
    bool inside = false;
    int j = block->boundary_count - 1;
    for (int i = 0; i < block->boundary_count; i++) {
        const SoPoint a = block->boundary[i];
        const SoPoint b = block->boundary[j];
        const double dy = b.y - a.y;
        const bool crosses = fabs(dy) > 1e-9 &&
                             ((a.y > point.y) != (b.y > point.y)) &&
                             (point.x < (b.x - a.x) * (point.y - a.y) / dy + a.x);
        if (crosses) {
            inside = !inside;
        }
        j = i;
    }
    return inside;
}

static double so_orient(SoPoint a, SoPoint b, SoPoint c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

static bool so_on_segment(SoPoint a, SoPoint b, SoPoint p) {
    return fabs(so_orient(a, b, p)) <= 1e-6 &&
           p.x >= fmin(a.x, b.x) - 1e-6 && p.x <= fmax(a.x, b.x) + 1e-6 &&
           p.y >= fmin(a.y, b.y) - 1e-6 && p.y <= fmax(a.y, b.y) + 1e-6;
}

static bool so_segments_intersect(SoPoint a, SoPoint b, SoPoint c, SoPoint d) {
    const double o1 = so_orient(a, b, c);
    const double o2 = so_orient(a, b, d);
    const double o3 = so_orient(c, d, a);
    const double o4 = so_orient(c, d, b);
    if (((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) &&
        ((o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0))) {
        return true;
    }
    return so_on_segment(a, b, c) || so_on_segment(a, b, d) ||
           so_on_segment(c, d, a) || so_on_segment(c, d, b);
}

static bool so_hive_segment_crosses_field(const SoField *field, SoPoint a, SoPoint b) {
    const SoPoint mid = so_point((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
    for (int block_idx = 0; block_idx < field->block_count; block_idx++) {
        const SoFieldBlock *block = &field->blocks[block_idx];
        if (!block->selected || block->boundary_count < 3) {
            continue;
        }
        if (so_point_in_block_polygon(a, block) || so_point_in_block_polygon(b, block) ||
            so_point_in_block_polygon(mid, block)) {
            return true;
        }
        int strict_intersections = 0;
        for (int i = 0; i < block->boundary_count; i++) {
            const SoPoint c = block->boundary[i];
            const SoPoint d = block->boundary[(i + 1) % block->boundary_count];
            if (so_segments_intersect(a, b, c, d)) {
                strict_intersections++;
            }
        }
        if (strict_intersections >= 2) {
            return true;
        }
    }
    return false;
}

static double so_boundary_path_distance(const SoFieldBlock *block, int from_idx, int to_idx) {
    if (block == NULL || block->boundary_count < 3) {
        return 1e12;
    }
    double clockwise = 0.0;
    int i = from_idx;
    while (i != to_idx) {
        const int next = (i + 1) % block->boundary_count;
        clockwise += so_distance(block->boundary[i], block->boundary[next]);
        i = next;
    }

    double counter = 0.0;
    i = from_idx;
    while (i != to_idx) {
        const int prev = (i - 1 + block->boundary_count) % block->boundary_count;
        counter += so_distance(block->boundary[i], block->boundary[prev]);
        i = prev;
    }
    return fmin(clockwise, counter);
}

static double so_hive_detour_around_block(SoPoint a, SoPoint b, const SoFieldBlock *block) {
    double best = 1e12;
    for (int i = 0; i < block->boundary_count; i++) {
        for (int j = 0; j < block->boundary_count; j++) {
            const double candidate =
                so_distance(a, block->boundary[i]) +
                so_boundary_path_distance(block, i, j) +
                so_distance(block->boundary[j], b);
            if (candidate < best) {
                best = candidate;
            }
        }
    }
    return best;
}

static double so_hive_route_distance(const SoSimulation *sim, SoPoint a, SoPoint b) {
    double route = so_distance(a, b);
    for (int block_idx = 0; block_idx < sim->field.block_count; block_idx++) {
        const SoFieldBlock *block = &sim->field.blocks[block_idx];
        if (!block->selected || block->boundary_count < 3) {
            continue;
        }
        SoField single;
        memset(&single, 0, sizeof(single));
        single.block_count = 1;
        single.blocks[0] = *block;
        if (!so_hive_segment_crosses_field(&single, a, b)) {
            continue;
        }
        route = fmax(route, so_hive_detour_around_block(a, b, block));
    }
    return route;
}

static double so_hive_travel_minutes(const SoSimulation *sim, SoPoint a, SoPoint b) {
    return so_hive_route_distance(sim, a, b) / fmax(0.001, sim->mothership.move_speed_mps) / 60.0;
}

static int so_count_interior_tasks_covered(const SoSimulation *sim, SoPoint point, double radius) {
    int count = 0;
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *task = &sim->field.tasks[i];
        if (task->status != SO_TASK_DONE && task->remaining_ha > 0.001 &&
            so_distance(point, task->center) <= radius) {
            count++;
        }
    }
    return count;
}

static int so_pending_major_task_count(const SoSimulation *sim) {
    int count = 0;
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *task = &sim->field.tasks[i];
        if (task->status != SO_TASK_DONE && task->remaining_ha > 0.001) {
            count++;
        }
    }
    return count;
}

static bool so_depot_plan_contains(const SoSimulation *sim, SoPoint point, double tolerance_m) {
    for (int i = 0; i < sim->mothership.operation_plan_count; i++) {
        if (so_distance(sim->mothership.operation_plan[i], point) <= tolerance_m) {
            return true;
        }
    }
    return false;
}

static void so_optimize_depot_order(SoSimulation *sim) {
    SoPoint unique[SO_MAX_DEPOTS];
    int unique_count = 0;
    for (int i = 0; i < sim->mothership.operation_plan_count; i++) {
        bool duplicate = false;
        for (int j = 0; j < unique_count; j++) {
            if (so_distance(unique[j], sim->mothership.operation_plan[i]) <= 120.0) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate && unique_count < SO_MAX_DEPOTS) {
            unique[unique_count++] = sim->mothership.operation_plan[i];
        }
    }

    bool used[SO_MAX_DEPOTS];
    memset(used, 0, sizeof(used));
    SoPoint ordered[SO_MAX_DEPOTS];
    int ordered_count = 0;
    SoPoint current = sim->mothership.position;

    while (ordered_count < unique_count) {
        int best = -1;
        double best_score = 1e100;
        for (int i = 0; i < unique_count; i++) {
            if (used[i]) {
                continue;
            }
            const int cover = so_count_interior_tasks_covered(sim, unique[i], 900.0);
            const double travel = so_hive_route_distance(sim, current, unique[i]);
            if (travel >= 1e11) {
                continue;
            }
            const double score = travel - cover * 85.0;
            if (score < best_score) {
                best_score = score;
                best = i;
            }
        }
        if (best < 0) {
            break;
        }
        used[best] = true;
        ordered[ordered_count++] = unique[best];
        current = unique[best];
    }

    for (int i = 0; i < ordered_count; i++) {
        sim->mothership.operation_plan[i] = ordered[i];
    }
    sim->mothership.operation_plan_count = ordered_count;
}

static void so_reverse_plan_segment(SoSimulation *sim, int begin, int end) {
    while (begin < end) {
        const SoPoint tmp = sim->mothership.operation_plan[begin];
        sim->mothership.operation_plan[begin] = sim->mothership.operation_plan[end];
        sim->mothership.operation_plan[end] = tmp;
        begin++;
        end--;
    }
}

static void so_two_opt_depot_order(SoSimulation *sim) {
    bool improved = true;
    int guard = 0;
    while (improved && guard++ < 24) {
        improved = false;
        for (int i = 0; i < sim->mothership.operation_plan_count - 2; i++) {
            for (int k = i + 1; k < sim->mothership.operation_plan_count - 1; k++) {
                const SoPoint a = i == 0 ? sim->mothership.position : sim->mothership.operation_plan[i - 1];
                const SoPoint b = sim->mothership.operation_plan[i];
                const SoPoint c = sim->mothership.operation_plan[k];
                const SoPoint d = sim->mothership.operation_plan[k + 1];
                const double before = so_hive_route_distance(sim, a, b) + so_hive_route_distance(sim, c, d);
                const double after = so_hive_route_distance(sim, a, c) + so_hive_route_distance(sim, b, d);
                if (after + 1.0 < before) {
                    so_reverse_plan_segment(sim, i, k);
                    improved = true;
                }
            }
        }
    }
}

static int so_best_depot_for_task(const SoSimulation *sim, const SoFieldTask *task) {
    int best = -1;
    double best_score = 1e100;
    for (int s = 0; s < sim->field.depot_count; s++) {
        const SoDepotSite *site = &sim->field.depots[s];
        if (!so_site_deployable(&sim->field, site)) {
            continue;
        }
        const double dist = so_distance(site->point, task->center);
        const double hive_route_m = so_hive_route_distance(sim, sim->mothership.position, site->point);
        if (hive_route_m >= 1e11) {
            continue;
        }
        const double score = dist + hive_route_m * 0.18 + site->slope_risk * 120.0;
        if (score < best_score) {
            best_score = score;
            best = s;
        }
    }
    return best;
}

static void so_insert_cleanup_stop_if_needed(SoSimulation *sim) {
    if (sim->mothership.operation_plan_count >= SO_MAX_DEPOTS) {
        return;
    }

    int far_task = -1;
    double far_dist = 0.0;
    for (int t = 0; t < sim->field.task_count; t++) {
        const SoFieldTask *task = &sim->field.tasks[t];
        if (task->kind != SO_TASK_INTERIOR_STRIP || task->remaining_ha <= 0.001) {
            continue;
        }
        double best_plan_dist = 1e100;
        for (int p = 0; p < sim->mothership.operation_plan_count; p++) {
            best_plan_dist = fmin(best_plan_dist, so_distance(sim->mothership.operation_plan[p], task->center));
        }
        if (best_plan_dist > far_dist) {
            far_dist = best_plan_dist;
            far_task = t;
        }
    }

    if (far_task < 0 || far_dist <= 1030.0) {
        return;
    }
    const int depot_idx = so_best_depot_for_task(sim, &sim->field.tasks[far_task]);
    if (depot_idx < 0) {
        return;
    }
    const SoPoint point = sim->field.depots[depot_idx].point;
    if (so_depot_plan_contains(sim, point, 160.0)) {
        return;
    }

    int insert_at = sim->mothership.operation_plan_count;
    double best_extra = 1e100;
    for (int i = 0; i <= sim->mothership.operation_plan_count; i++) {
        const SoPoint prev = i == 0 ? sim->mothership.position : sim->mothership.operation_plan[i - 1];
        if (i == sim->mothership.operation_plan_count) {
            const double extra = so_hive_route_distance(sim, prev, point);
            if (extra < best_extra) {
                best_extra = extra;
                insert_at = i;
            }
        } else {
            const SoPoint next = sim->mothership.operation_plan[i];
            const double extra = so_hive_route_distance(sim, prev, point) +
                                 so_hive_route_distance(sim, point, next) -
                                 so_hive_route_distance(sim, prev, next);
            if (extra < best_extra) {
                best_extra = extra;
                insert_at = i;
            }
        }
    }

    for (int i = sim->mothership.operation_plan_count; i > insert_at; i--) {
        sim->mothership.operation_plan[i] = sim->mothership.operation_plan[i - 1];
    }
    sim->mothership.operation_plan[insert_at] = point;
    sim->mothership.operation_plan_count++;
}

static void so_plan_depots(SoSimulation *sim) {
    bool covered[SO_MAX_TASKS];
    memset(covered, 0, sizeof(covered));
    int uncovered = 0;
    for (int t = 0; t < sim->field.task_count; t++) {
        covered[t] = sim->field.tasks[t].status == SO_TASK_DONE ||
                     sim->field.tasks[t].remaining_ha <= 0.001;
        if (!covered[t]) {
            uncovered++;
        }
    }
    sim->mothership.operation_plan_count = 0;
    const double primary_cover_radius = so_depot_scarce(sim) ? 2200.0 : (so_regular_corridor_layout(sim) ? 1450.0 : 940.0);

    while (uncovered > 0 && sim->mothership.operation_plan_count < SO_MAX_DEPOTS) {
        int best_site = -1;
        double best_score = -1e100;
        SoPoint previous = sim->mothership.operation_plan_count == 0
                               ? sim->mothership.position
                               : sim->mothership.operation_plan[sim->mothership.operation_plan_count - 1];

        for (int s = 0; s < sim->field.depot_count; s++) {
            const SoDepotSite *site = &sim->field.depots[s];
            if (!so_site_deployable(&sim->field, site)) {
                continue;
            }
            if (so_depot_plan_contains(sim, site->point, 140.0)) {
                continue;
            }
            int cover = 0;
            double covered_area = 0.0;
            double covered_priority = 0.0;
            for (int t = 0; t < sim->field.task_count; t++) {
                const SoFieldTask *task = &sim->field.tasks[t];
                if (!covered[t] && so_distance(site->point, task->center) <= primary_cover_radius) {
                    const double kind_weight = task->kind == SO_TASK_INTERIOR_STRIP ? 1.0 :
                                               (task->kind == SO_TASK_BOUNDARY ? 0.62 : 0.42);
                    cover++;
                    covered_area += task->remaining_ha * kind_weight;
                    covered_priority += task->priority * kind_weight;
                }
            }
            const double move_minutes = so_hive_travel_minutes(sim, previous, site->point);
            if (move_minutes >= 1e8) {
                continue;
            }
            const double move_distance_m = so_hive_route_distance(sim, previous, site->point);
            const double hive_cost_penalty =
                (so_hive_move_cost_usd(sim, move_distance_m) + sim->mothership.deployment_stop_cost_usd) * 0.12;
            double backtrack_penalty = 0.0;
            if (sim->mothership.operation_plan_count >= 2) {
                const SoPoint before = sim->mothership.operation_plan[sim->mothership.operation_plan_count - 2];
                const double prev_angle = atan2(previous.y - before.y, previous.x - before.x);
                const double next_angle = atan2(site->point.y - previous.y, site->point.x - previous.x);
                backtrack_penalty = so_angle_diff_rad(prev_angle, next_angle) * 18.0;
            }
            const double stop_cost = 30.0 + sim->mothership.operation_plan_count * 4.0;
            const double score = covered_area * 42.0 + covered_priority * 9.0 + cover * 6.0 -
                                 move_minutes * 3.5 - stop_cost - hive_cost_penalty -
                                 site->slope_risk * 18.0 - backtrack_penalty;
            if (cover > 0 && score > best_score) {
                best_score = score;
                best_site = s;
            }
        }

        if (best_site < 0 || best_score <= 0.0) {
            break;
        }

        SoPoint depot = sim->field.depots[best_site].point;
        sim->mothership.operation_plan[sim->mothership.operation_plan_count++] = depot;
        for (int t = 0; t < sim->field.task_count; t++) {
            if (!covered[t] && so_distance(depot, sim->field.tasks[t].center) <= primary_cover_radius) {
                covered[t] = true;
                uncovered--;
            }
        }
    }

    if (sim->mothership.operation_plan_count == 0) {
        for (int s = 0; s < sim->field.depot_count; s++) {
            if (so_site_deployable(&sim->field, &sim->field.depots[s])) {
                sim->mothership.operation_plan[0] = sim->field.depots[s].point;
                sim->mothership.operation_plan_count = 1;
                break;
            }
        }
        if (sim->mothership.operation_plan_count == 0) {
            sim->mothership.operation_plan[0] = sim->mothership.position;
            sim->mothership.operation_plan_count = 1;
        }
    }
    so_optimize_depot_order(sim);
    so_insert_cleanup_stop_if_needed(sim);
    so_two_opt_depot_order(sim);
    sim->mothership.operation_plan_index = 0;
    sim->mothership.stop_cost_usd =
        (double)sim->mothership.operation_plan_count * sim->mothership.deployment_stop_cost_usd;
    so_event(sim, "depot plan fixed after scout");
}

static double so_residual_uav_chunk_target_ha(const SoSimulation *sim) {
    const int options[3] = {1, 2, 4};
    double best_cycle_area = 0.0;
    for (int i = 0; i < 3; i++) {
        SoDroneSpec candidate = so_t200_spec_for_modules(sim->spec, options[i]);
        const double battery_area =
            fmax(0.0, 0.92 - candidate.safety_battery_margin) /
            fmax(0.001, candidate.battery_drain_h_work) *
            candidate.spray_rate_ha_h;
        const double chemical_area =
            candidate.chemical_tank_l /
            fmax(0.001, candidate.chemical_l_per_ha);
        best_cycle_area = fmax(best_cycle_area, fmin(battery_area, chemical_area));
    }
    const double profile_factor =
        sim->optimization_profile == SO_OPT_PROFILE_TIME ? 1.10 :
        sim->optimization_profile == SO_OPT_PROFILE_COST ? 0.90 : 1.0;
    return fmax(0.65, fmin(4.25, best_cycle_area * 0.82 * profile_factor));
}

static bool so_residual_chunk_route(const SoSimulation *sim,
                                    const SoFieldTask *task,
                                    int chunk_index,
                                    int chunk_count,
                                    double total_area_ha,
                                    SoPoint *out_start,
                                    SoPoint *out_end,
                                    SoPoint *out_center) {
    if (task == NULL || out_start == NULL || out_end == NULL ||
        out_center == NULL || chunk_count <= 0) {
        return false;
    }

    SoPoint base_start = task->route_start;
    SoPoint base_end = task->route_end;
    if (!task->has_planned_route || so_distance(base_start, base_end) <= 1.0) {
        const double angle_rad = task->strip_angle_deg * M_PI / 180.0;
        const double length =
            fmax(20.0, sqrt(fmax(1.0, total_area_ha * 10000.0)) * 1.35);
        const double dx = cos(angle_rad) * length * 0.5;
        const double dy = sin(angle_rad) * length * 0.5;
        base_start = so_point(task->center.x - dx, task->center.y - dy);
        base_end = so_point(task->center.x + dx, task->center.y + dy);
    }

    const double length = so_distance(base_start, base_end);
    if (length <= 1.0) {
        *out_start = task->center;
        *out_end = task->center;
        *out_center = task->center;
        return false;
    }

    const double ux = (base_end.x - base_start.x) / length;
    const double uy = (base_end.y - base_start.y) / length;
    const double nx = -uy;
    const double ny = ux;
    const double angle_rad = atan2(uy, ux);
    const double width_total_m =
        total_area_ha * 10000.0 / fmax(1.0, length);
    const double chunk_width_m =
        fmax(sim->spec.spray_swath_m,
             width_total_m / fmax(1.0, (double)chunk_count));
    const double offset_m =
        ((double)chunk_index + 0.5) * chunk_width_m -
        width_total_m * 0.5;

    SoPoint start = so_point(base_start.x + nx * offset_m,
                             base_start.y + ny * offset_m);
    SoPoint end = so_point(base_end.x + nx * offset_m,
                           base_end.y + ny * offset_m);

    const SoFieldBlock *block = so_find_block_const(sim, task->block_id);
    if (block != NULL && block->boundary_count >= 3) {
        const double cross =
            ((start.x + end.x) * 0.5) * nx +
            ((start.y + end.y) * 0.5) * ny;
        const double route_mid_t =
            ((start.x + end.x) * 0.5) * ux +
            ((start.y + end.y) * 0.5) * uy;
        double mins[SO_MAX_BOUNDARY_POINTS / 2];
        double maxs[SO_MAX_BOUNDARY_POINTS / 2];
        const int intervals =
            so_line_block_intervals(block, angle_rad, cross, mins, maxs,
                                    SO_MAX_BOUNDARY_POINTS / 2);
        int best = -1;
        double best_gap = 1e100;
        for (int k = 0; k < intervals; k++) {
            const double len = maxs[k] - mins[k];
            if (len <= 8.0) {
                continue;
            }
            const double center_t = (mins[k] + maxs[k]) * 0.5;
            const double gap = fabs(center_t - route_mid_t);
            if (gap < best_gap) {
                best = k;
                best_gap = gap;
            }
        }
        if (best >= 0) {
            const double len = maxs[best] - mins[best];
            const double margin = fmin(12.0, fmax(0.0, len * 0.01));
            start = so_point(ux * (mins[best] + margin) + nx * cross,
                             uy * (mins[best] + margin) + ny * cross);
            end = so_point(ux * (maxs[best] - margin) + nx * cross,
                           uy * (maxs[best] - margin) + ny * cross);
        }
    }

    *out_start = start;
    *out_end = end;
    *out_center = so_point((start.x + end.x) * 0.5,
                           (start.y + end.y) * 0.5);
    return so_distance(start, end) > 1.0;
}

typedef struct {
    double lo;
    double hi;
} SoInterval1D;

typedef struct {
    double area_ha;
    double weighted_x;
    double weighted_y;
    SoPoint route_start;
    SoPoint route_end;
    double route_len_m;
    int segment_count;
} SoSpatialResidualAccumulator;

static bool so_clip_param_range(double coeff,
                                double constant,
                                double min_value,
                                double max_value,
                                double *lo,
                                double *hi) {
    if (fabs(coeff) <= 1e-9) {
        return constant >= min_value - 1e-6 &&
               constant <= max_value + 1e-6;
    }
    double a = (min_value - constant) / coeff;
    double b = (max_value - constant) / coeff;
    if (a > b) {
        const double tmp = a;
        a = b;
        b = tmp;
    }
    *lo = fmax(*lo, a);
    *hi = fmin(*hi, b);
    return *hi > *lo + 1.0;
}

static bool so_fixed_wing_swath_interval_on_line(const SoSimulation *sim,
                                                 const SoFieldTask *fixed,
                                                 double line_angle_rad,
                                                 double line_cross,
                                                 double *out_lo,
                                                 double *out_hi) {
    if (sim == NULL || fixed == NULL || fixed->fixed_wing_area_ha <= 0.001 ||
        out_lo == NULL || out_hi == NULL) {
        return false;
    }

    SoPoint fs;
    SoPoint fe;
    if (!so_fixed_wing_candidate_task_route(fixed,
                                            fixed->fixed_wing_area_ha,
                                            sim->fixed_wing.swath_width_m,
                                            &fs, &fe)) {
        if (!fixed->has_planned_route ||
            so_distance(fixed->route_start, fixed->route_end) <= 1.0) {
            return false;
        }
        fs = fixed->route_start;
        fe = fixed->route_end;
    }

    const double fixed_len = so_distance(fs, fe);
    if (fixed_len <= 1.0) {
        return false;
    }
    const double lux = cos(line_angle_rad);
    const double luy = sin(line_angle_rad);
    const double lnx = -luy;
    const double lny = lux;
    const double fux = (fe.x - fs.x) / fixed_len;
    const double fuy = (fe.y - fs.y) / fixed_len;
    const double fnx = -fuy;
    const double fny = fux;
    const double fixed_start_t = fs.x * fux + fs.y * fuy;
    const double fixed_end_t = fe.x * fux + fe.y * fuy;
    const double along_min =
        fmin(fixed_start_t, fixed_end_t) -
        fmax(2.0, sim->fixed_wing.swath_width_m * 0.12);
    const double along_max =
        fmax(fixed_start_t, fixed_end_t) +
        fmax(2.0, sim->fixed_wing.swath_width_m * 0.12);
    const double fixed_cross = fs.x * fnx + fs.y * fny;
    const double half_width =
        sim->fixed_wing.swath_width_m * 0.5 +
        fmax(0.0, sim->spec.spray_swath_m) * 0.25;

    double lo = -1e100;
    double hi = 1e100;
    double coeff = lux * fux + luy * fuy;
    double constant = line_cross * (lnx * fux + lny * fuy);
    if (!so_clip_param_range(coeff, constant, along_min, along_max, &lo, &hi)) {
        return false;
    }
    coeff = lux * fnx + luy * fny;
    constant = line_cross * (lnx * fnx + lny * fny);
    if (!so_clip_param_range(coeff, constant,
                             fixed_cross - half_width,
                             fixed_cross + half_width,
                             &lo, &hi)) {
        return false;
    }
    *out_lo = lo;
    *out_hi = hi;
    return hi > lo + 1.0;
}

static int so_subtract_interval(SoInterval1D *segments,
                                int count,
                                int max_count,
                                double cut_lo,
                                double cut_hi) {
    if (segments == NULL || count <= 0 || max_count <= 0 ||
        cut_hi <= cut_lo + 1e-6) {
        return count;
    }
    int out_count = 0;
    SoInterval1D out[SO_MAX_BOUNDARY_POINTS];
    for (int i = 0; i < count && out_count < max_count; i++) {
        const double lo = segments[i].lo;
        const double hi = segments[i].hi;
        if (cut_hi <= lo || cut_lo >= hi) {
            out[out_count++] = segments[i];
            continue;
        }
        if (cut_lo > lo + 1.0 && out_count < max_count) {
            out[out_count++] = (SoInterval1D){lo, fmin(cut_lo, hi)};
        }
        if (cut_hi < hi - 1.0 && out_count < max_count) {
            out[out_count++] = (SoInterval1D){fmax(cut_hi, lo), hi};
        }
    }
    for (int i = 0; i < out_count; i++) {
        segments[i] = out[i];
    }
    return out_count;
}

static void so_reset_spatial_residual_accumulator(SoSpatialResidualAccumulator *acc) {
    memset(acc, 0, sizeof(*acc));
}

static int so_flush_spatial_residual_task(SoSimulation *sim,
                                          const SoFieldBlock *block,
                                          double angle_deg,
                                          int zone_id,
                                          int bundle_hint,
                                          SoSpatialResidualAccumulator *acc) {
    if (sim == NULL || block == NULL || acc == NULL ||
        acc->area_ha <= 0.001 || sim->field.task_count >= SO_MAX_TASKS) {
        if (acc != NULL) {
            so_reset_spatial_residual_accumulator(acc);
        }
        return 0;
    }
    const SoPoint center =
        so_point(acc->weighted_x / fmax(0.001, acc->area_ha),
                 acc->weighted_y / fmax(0.001, acc->area_ha));
    SoPoint start = acc->route_start;
    SoPoint end = acc->route_end;
    if (so_distance(start, end) <= 1.0) {
        const double rad = angle_deg * M_PI / 180.0;
        const double len =
            fmax(10.0, acc->area_ha * 10000.0 /
                           fmax(0.001, sim->spec.spray_swath_m));
        start = so_point(center.x - cos(rad) * len * 0.5,
                         center.y - sin(rad) * len * 0.5);
        end = so_point(center.x + cos(rad) * len * 0.5,
                       center.y + sin(rad) * len * 0.5);
    }

    const int before = sim->field.task_count;
    const double route_eff =
        fmax(0.62, fmin(1.22, acc->route_len_m /
                                  fmax(1.0, acc->route_len_m +
                                                (double)fmax(0, acc->segment_count - 1) *
                                                    sim->spec.spray_swath_m)));
    so_add_routed_task(sim, zone_id, block->id, start, end, acc->area_ha,
                       1.35 + block->risk,
                       fmin(1.0, block->risk + 0.10),
                       angle_deg,
                       route_eff,
                       SO_TASK_INTERIOR_STRIP);
    if (sim->field.task_count > before) {
        SoFieldTask *task = &sim->field.tasks[before];
        task->center = center;
        task->bundle_hint = bundle_hint;
        task->turn_count = fmax(0, acc->segment_count - 1);
        task->turn_time_s = (double)task->turn_count * sim->spec.turn_time_s;
        task->turn_energy_cost =
            (double)task->turn_count * sim->spec.turn_battery_cost * 0.55;
        so_reset_spatial_residual_accumulator(acc);
        return 1;
    }
    so_reset_spatial_residual_accumulator(acc);
    return 0;
}

static void so_add_spatial_residual_segment(SoSimulation *sim,
                                            const SoFieldBlock *block,
                                            double angle_deg,
                                            int zone_id,
                                            int bundle_hint,
                                            double target_chunk_ha,
                                            SoPoint start,
                                            SoPoint end,
                                            double area_ha,
                                            SoSpatialResidualAccumulator *acc,
                                            int *created) {
    if (area_ha <= 0.001 || acc == NULL || created == NULL) {
        return;
    }
    if (acc->area_ha > 0.001 &&
        acc->area_ha + area_ha > target_chunk_ha) {
        *created += so_flush_spatial_residual_task(sim, block, angle_deg,
                                                   zone_id, bundle_hint, acc);
    }
    const double len = so_distance(start, end);
    const SoPoint center = so_point((start.x + end.x) * 0.5,
                                    (start.y + end.y) * 0.5);
    acc->weighted_x += center.x * area_ha;
    acc->weighted_y += center.y * area_ha;
    acc->area_ha += area_ha;
    acc->segment_count++;
    if (len > acc->route_len_m) {
        acc->route_start = start;
        acc->route_end = end;
        acc->route_len_m = len;
    }
}

static double so_block_residual_angle_deg(const SoSimulation *sim,
                                          const SoFieldBlock *block) {
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    for (int t = 0; t < sim->field.task_count; t++) {
        const SoFieldTask *task = &sim->field.tasks[t];
        if (task->block_id != block->id) {
            continue;
        }
        const double weight =
            task->fixed_wing_area_ha > 0.001
                ? task->fixed_wing_area_ha * 0.35
                : fmax(0.0, task->remaining_ha);
        if (weight <= 0.001) {
            continue;
        }
        const double rad = task->strip_angle_deg * M_PI / 180.0;
        sum_sin += sin(2.0 * rad) * weight;
        sum_cos += cos(2.0 * rad) * weight;
    }
    if (fabs(sum_sin) + fabs(sum_cos) <= 1e-9) {
        return 0.0;
    }
    double angle = atan2(sum_sin, sum_cos) * 0.5 * 180.0 / M_PI;
    while (angle < 0.0) {
        angle += 180.0;
    }
    while (angle >= 180.0) {
        angle -= 180.0;
    }
    return angle;
}

static int so_disable_block_uav_source_tasks(SoSimulation *sim,
                                             int block_id,
                                             int task_limit) {
    int disabled = 0;
    for (int t = 0; t < task_limit; t++) {
        SoFieldTask *task = &sim->field.tasks[t];
        if (task->block_id != block_id || task->fixed_wing_area_ha > 0.001 ||
            task->remaining_ha <= 0.001 || task->status == SO_TASK_DONE) {
            continue;
        }
        task->area_ha = 0.0;
        task->remaining_ha = 0.0;
        task->status = SO_TASK_DONE;
        task->assigned_drone_id = -1;
        disabled++;
    }
    return disabled;
}

static int so_rebuild_block_spatial_uav_residual(SoSimulation *sim,
                                                 const SoFieldBlock *block,
                                                 int task_limit,
                                                 double *out_area_ha) {
    if (out_area_ha != NULL) {
        *out_area_ha = 0.0;
    }
    if (sim == NULL || block == NULL || block->boundary_count < 3 ||
        sim->field.task_count >= SO_MAX_TASKS) {
        return 0;
    }

    double fixed_area_ha = 0.0;
    int fixed_count = 0;
    for (int t = 0; t < task_limit; t++) {
        const SoFieldTask *task = &sim->field.tasks[t];
        if (task->block_id == block->id && task->fixed_wing_area_ha > 0.001) {
            fixed_area_ha += task->fixed_wing_area_ha;
            fixed_count++;
        }
    }
    const double residual_cap_ha =
        fmax(0.0, block->area_ha - fixed_area_ha);
    if (residual_cap_ha <= 0.02) {
        so_disable_block_uav_source_tasks(sim, block->id, task_limit);
        return 0;
    }

    const int start_task_count = sim->field.task_count;
    const double angle_deg = so_block_residual_angle_deg(sim, block);
    const double angle_rad = angle_deg * M_PI / 180.0;
    const double ux = cos(angle_rad);
    const double uy = sin(angle_rad);
    const double nx = -uy;
    const double ny = ux;
    const double swath = fmax(0.001, sim->spec.spray_swath_m);
    const double spacing = swath;
    const double target_chunk_ha =
        fmin(6.5, so_residual_uav_chunk_target_ha(sim) * 1.45);
    const int zone_id = sim->field.zone_count > 0 ? 1 : 1;
    const int bundle_hint = 2000 + block->id;

    double min_cross = 0.0;
    double max_cross = 0.0;
    so_block_projection_range(block, angle_deg, &min_cross, &max_cross);
    if (max_cross <= min_cross) {
        return 0;
    }

    SoSpatialResidualAccumulator acc;
    so_reset_spatial_residual_accumulator(&acc);
    int created = 0;
    double generated_area_ha = 0.0;
    bool cap_reached = false;

    for (double cross = min_cross + spacing * 0.5;
         cross <= max_cross - spacing * 0.25 &&
         sim->field.task_count < SO_MAX_TASKS && !cap_reached;
         cross += spacing) {
        double mins[SO_MAX_BOUNDARY_POINTS / 2];
        double maxs[SO_MAX_BOUNDARY_POINTS / 2];
        const int intervals =
            so_line_block_intervals(block, angle_rad, cross, mins, maxs,
                                    SO_MAX_BOUNDARY_POINTS / 2);
        for (int i = 0; i < intervals &&
             sim->field.task_count < SO_MAX_TASKS && !cap_reached; i++) {
            SoInterval1D segments[SO_MAX_BOUNDARY_POINTS];
            int segment_count = 1;
            segments[0] = (SoInterval1D){mins[i], maxs[i]};
            for (int t = 0; t < task_limit && segment_count > 0; t++) {
                const SoFieldTask *fixed = &sim->field.tasks[t];
                if (fixed->block_id != block->id ||
                    fixed->fixed_wing_area_ha <= 0.001) {
                    continue;
                }
                double cover_lo = 0.0;
                double cover_hi = 0.0;
                if (!so_fixed_wing_swath_interval_on_line(sim, fixed,
                                                          angle_rad, cross,
                                                          &cover_lo, &cover_hi)) {
                    continue;
                }
                segment_count =
                    so_subtract_interval(segments, segment_count,
                                         SO_MAX_BOUNDARY_POINTS,
                                         cover_lo, cover_hi);
            }

            for (int s = 0; s < segment_count &&
                 sim->field.task_count < SO_MAX_TASKS; s++) {
                double lo = segments[s].lo;
                double hi = segments[s].hi;
                const double len = hi - lo;
                if (len <= fmax(8.0, swath * 0.75)) {
                    continue;
                }
                double area_ha = len * swath / 10000.0;
                if (generated_area_ha + acc.area_ha + area_ha > residual_cap_ha) {
                    const double remaining_ha =
                        residual_cap_ha - generated_area_ha - acc.area_ha;
                    if (remaining_ha <= 0.001) {
                        cap_reached = true;
                        break;
                    }
                    area_ha = remaining_ha;
                    hi = lo + area_ha * 10000.0 / swath;
                    cap_reached = true;
                }
                const double margin = fmin(3.0, fmax(0.0, (hi - lo) * 0.01));
                const SoPoint start =
                    so_point(ux * (lo + margin) + nx * cross,
                             uy * (lo + margin) + ny * cross);
                const SoPoint end =
                    so_point(ux * (hi - margin) + nx * cross,
                             uy * (hi - margin) + ny * cross);
                so_add_spatial_residual_segment(sim, block, angle_deg, zone_id,
                                                bundle_hint, target_chunk_ha,
                                                start, end, area_ha, &acc,
                                                &created);
            }
        }
    }
    created += so_flush_spatial_residual_task(sim, block, angle_deg, zone_id,
                                              bundle_hint, &acc);
    for (int t = start_task_count; t < sim->field.task_count; t++) {
        generated_area_ha += sim->field.tasks[t].remaining_ha;
    }

    const double minimum_useful_area = fmin(0.01, residual_cap_ha * 0.02);
    if (created <= 0 || generated_area_ha < minimum_useful_area) {
        sim->field.task_count = start_task_count;
        return 0;
    }

    so_disable_block_uav_source_tasks(sim, block->id, task_limit);
    sim->field.residual_spatial_task_count += created;
    sim->field.residual_spatial_area_ha += generated_area_ha;
    if (out_area_ha != NULL) {
        *out_area_ha = generated_area_ha;
    }
    return created;
}

static void so_compact_superseded_uav_source_tasks(SoSimulation *sim) {
    int write = 0;
    for (int read = 0; read < sim->field.task_count; read++) {
        const SoFieldTask *task = &sim->field.tasks[read];
        const bool superseded =
            task->area_ha <= 0.001 &&
            task->remaining_ha <= 0.001 &&
            task->fixed_wing_area_ha <= 0.001;
        if (superseded) {
            continue;
        }
        if (write != read) {
            sim->field.tasks[write] = sim->field.tasks[read];
        }
        write++;
    }
    sim->field.task_count = write;
}

static int so_split_residual_task_for_uav(SoSimulation *sim, SoFieldTask *task) {
    if (task == NULL || task->remaining_ha <= 0.001 ||
        task->status == SO_TASK_DONE || sim->field.task_count >= SO_MAX_TASKS) {
        return 1;
    }

    const double target_chunk_ha = so_residual_uav_chunk_target_ha(sim);
    int desired_chunks =
        (int)ceil(task->remaining_ha / fmax(0.25, target_chunk_ha));
    if (desired_chunks <= 1) {
        return 1;
    }
    desired_chunks = (int)fmin(12.0, (double)desired_chunks);
    const int available_extra = SO_MAX_TASKS - sim->field.task_count;
    int chunk_count = desired_chunks;
    if (chunk_count > available_extra + 1) {
        chunk_count = available_extra + 1;
    }
    if (chunk_count < 1) {
        chunk_count = 1;
    }
    if (chunk_count <= 1) {
        return 1;
    }

    const double original_area_ha = task->remaining_ha;
    const SoFieldTask original_task = *task;
    const double chunk_area_ha = original_area_ha / (double)chunk_count;
    const int zone_id = task->zone_id;
    const int block_id = task->block_id;
    const double priority = task->priority;
    const double risk = task->risk;
    const double angle = task->strip_angle_deg;
    const double route_efficiency = task->route_efficiency;
    const SoTaskKind kind = task->kind;
    const int bundle_hint = task->bundle_hint;

    for (int c = 0; c < chunk_count; c++) {
        SoPoint start = task->route_start;
        SoPoint end = task->route_end;
        SoPoint center = task->center;
        so_residual_chunk_route(sim, &original_task, c, chunk_count, original_area_ha,
                                &start, &end, &center);
        if (c == 0) {
            task->center = center;
            task->area_ha = chunk_area_ha;
            task->remaining_ha = chunk_area_ha;
            task->priority = priority;
            task->risk = risk;
            task->route_efficiency = route_efficiency;
            task->kind = kind;
            task->bundle_hint = bundle_hint;
            task->status = SO_TASK_PENDING;
            task->assigned_drone_id = -1;
            so_store_task_route(task, start, end, 0.0);
            continue;
        }

        const int before = sim->field.task_count;
        so_add_routed_task(sim, zone_id, block_id, start, end, chunk_area_ha,
                           priority, risk, angle, route_efficiency, kind);
        if (sim->field.task_count > before) {
            SoFieldTask *added = &sim->field.tasks[before];
            added->bundle_hint = bundle_hint;
            added->status = SO_TASK_PENDING;
            added->assigned_drone_id = -1;
            added->fixed_wing_area_ha = 0.0;
        }
    }

    sim->field.residual_rebuild_split_count += chunk_count - 1;
    sim->field.residual_rebuild_area_ha += original_area_ha;
    return chunk_count;
}

static void so_rebuild_uav_residual_tasks_after_fixed_wing(SoSimulation *sim) {
    if (!sim->fixed_wing.planned) {
        return;
    }

    bool block_has_fixed_wing[SO_MAX_BLOCKS];
    memset(block_has_fixed_wing, 0, sizeof(block_has_fixed_wing));
    for (int t = 0; t < sim->field.task_count; t++) {
        const SoFieldTask *task = &sim->field.tasks[t];
        if (task->fixed_wing_area_ha <= 0.001) {
            continue;
        }
        for (int b = 0; b < sim->field.block_count; b++) {
            if (sim->field.blocks[b].id == task->block_id) {
                block_has_fixed_wing[b] = true;
                break;
            }
        }
    }

    sim->field.residual_rebuild_split_count = 0;
    sim->field.residual_rebuild_area_ha = 0.0;
    sim->field.residual_spatial_task_count = 0;
    sim->field.residual_spatial_area_ha = 0.0;

    const int pre_spatial_task_count = sim->field.task_count;
    for (int b = 0; b < sim->field.block_count; b++) {
        const SoFieldBlock *block = &sim->field.blocks[b];
        if (!block->selected || block->boundary_count < 3) {
            continue;
        }
        double spatial_area_ha = 0.0;
        const int made =
            so_rebuild_block_spatial_uav_residual(sim, block,
                                                  pre_spatial_task_count,
                                                  &spatial_area_ha);
        if (made > 0) {
            char msg[160];
            snprintf(msg, sizeof(msg),
                     "spatial UAV residual rebuilt: block=%d tasks=%d area=%.2fha fixed_subtract=%s",
                     block->id, made, spatial_area_ha,
                     block_has_fixed_wing[b] ? "true" : "false");
            so_event(sim, msg);
        }
    }
    so_compact_superseded_uav_source_tasks(sim);

    int residual_count = 0;
    int route_limited_count = 0;
    double residual_area_ha = 0.0;
    const double radius = so_working_radius(sim);
    const int original_task_count = sim->field.task_count;
    for (int t = 0; t < original_task_count; t++) {
        SoFieldTask *task = &sim->field.tasks[t];
        if (task->remaining_ha <= 0.001 || task->status == SO_TASK_DONE) {
            continue;
        }

        task->status = SO_TASK_PENDING;
        task->assigned_drone_id = -1;
        task->fixed_wing_area_ha = 0.0;
        residual_count++;
        residual_area_ha += task->remaining_ha;

        bool same_block_had_fixed_wing = false;
        for (int b = 0; b < sim->field.block_count; b++) {
            if (sim->field.blocks[b].id == task->block_id) {
                same_block_had_fixed_wing = block_has_fixed_wing[b];
                break;
            }
        }

        int reachable_depots = 0;
        double best_depot_dist = 1e100;
        double best_hive_route = 1e100;
        for (int s = 0; s < sim->field.depot_count; s++) {
            const SoDepotSite *site = &sim->field.depots[s];
            if (!so_site_deployable(&sim->field, site)) {
                continue;
            }
            const double task_dist = so_distance(site->point, task->center);
            if (task_dist > radius * 1.35) {
                continue;
            }
            const double hive_route = so_hive_route_distance(sim, sim->mothership.position, site->point);
            if (hive_route >= 1e11) {
                continue;
            }
            reachable_depots++;
            best_depot_dist = fmin(best_depot_dist, task_dist);
            best_hive_route = fmin(best_hive_route, hive_route);
        }

        if (same_block_had_fixed_wing && task->bundle_hint < 2000) {
            task->bundle_hint = 1000 + task->block_id;
            task->priority += task->kind == SO_TASK_INTERIOR_STRIP ? 0.18 : 0.42;
        }
        if (task->kind == SO_TASK_BOUNDARY || task->kind == SO_TASK_REPAIR) {
            task->priority += 0.25;
            task->route_efficiency = fmax(0.62, task->route_efficiency);
        }
        if (reachable_depots == 0) {
            route_limited_count++;
            task->priority -= 0.35;
            task->risk = fmin(1.0, task->risk + 0.08);
        } else {
            const double access_bonus =
                fmax(0.0, radius - best_depot_dist) / fmax(1.0, radius) * 0.35 +
                fmax(0.0, 2200.0 - best_hive_route) / 2200.0 * 0.20;
            task->priority += access_bonus;
            task->route_efficiency = fmin(1.08, task->route_efficiency + access_bonus * 0.08);
        }

        const int chunks = task->bundle_hint >= 2000
                               ? 1
                               : so_split_residual_task_for_uav(sim, task);
        if (chunks > 1) {
            residual_count += chunks - 1;
        }
    }

    if (residual_count > 0) {
        char msg[160];
        snprintf(msg, sizeof(msg),
                 "uav residual work rebuilt after fixed-wing: tasks=%d area=%.2fha route_limited=%d split_chunks=%d",
                 residual_count, residual_area_ha, route_limited_count,
                 sim->field.residual_rebuild_split_count);
        so_event(sim, msg);
    }
}

static bool so_task_directed_route(const SoSimulation *sim,
                                   const SoFieldTask *task,
                                   double area_ha,
                                   SoPoint current,
                                   SoPoint recovery,
                                   SoPoint *out_start,
                                   SoPoint *out_end,
                                   double *out_curve_deg) {
    SoPoint a;
    SoPoint b;
    if (task->has_planned_route && so_distance(task->route_start, task->route_end) > 1.0) {
        a = task->route_start;
        b = task->route_end;
    } else {
        const double angle = task->strip_angle_deg * M_PI / 180.0;
        const double length =
            fmax(1.0, sqrt(fmax(1.0, area_ha * 10000.0)) * 1.35);
        const double dx = cos(angle) * length * 0.5;
        const double dy = sin(angle) * length * 0.5;
        a = so_point(task->center.x - dx, task->center.y - dy);
        b = so_point(task->center.x + dx, task->center.y + dy);
    }

    double best_score = 1e100;
    SoPoint best_start = a;
    SoPoint best_end = b;
    for (int dir = 0; dir < 2; dir++) {
        const SoPoint start = dir == 0 ? a : b;
        const SoPoint end = dir == 0 ? b : a;
        const double chord_heading = so_heading_between(start, end);
        const double entry_heading = so_heading_between(current, start);
        const double exit_heading = so_heading_between(end, recovery);
        const double score =
            so_distance(current, start) +
            so_distance(end, recovery) * 0.12 +
            so_heading_diff_rad(entry_heading, chord_heading) * sim->spec.turn_radius_m * 0.08 +
            so_heading_diff_rad(chord_heading, exit_heading) * sim->spec.turn_radius_m * 0.03;
        if (score < best_score) {
            best_score = score;
            best_start = start;
            best_end = end;
        }
    }
    *out_start = best_start;
    *out_end = best_end;
    *out_curve_deg = 0.0;
    return so_distance(*out_start, *out_end) > 1.0;
}

static void so_assign_drone_to_task(SoSimulation *sim,
                                    SoDrone *drone,
                                    SoFieldTask *task,
                                    double capacity,
                                    double takeoff_service_s,
                                    int collaborative_slot,
                                    SoPoint recovery_point) {
    const bool launching = drone->state == SO_DRONE_IDLE || drone->state == SO_DRONE_STANDBY;
    const bool has_primary_route =
        collaborative_slot > 0 && task->has_planned_route &&
        so_distance(task->route_start, task->route_end) > 1.0;
    const SoPoint primary_end = task->route_end;
    const double reserved_area_before_ha =
        fmax(0.0, so_active_assigned_area_for_task(sim, task->id));
    double sortie_capacity = capacity;
    if (launching) {
        sortie_capacity =
            so_choose_drone_sortie_configuration(sim, drone, task, recovery_point);
    }
    const double assigned_area = task->kind == SO_TASK_INTERIOR_STRIP
                                     ? fmin(sortie_capacity * 0.92, task->remaining_ha + 5.5)
                                     : fmin(task->remaining_ha, sortie_capacity);
    const double route_area = fmin(task->remaining_ha, assigned_area);
    SoPoint route_start = task->center;
    SoPoint route_end = task->center;
    double route_curve_deg = 0.0;
    so_task_directed_route(sim, task, route_area,
                           drone->position, recovery_point,
                           &route_start, &route_end, &route_curve_deg);
    if (has_primary_route &&
        so_distance(route_start, primary_end) > so_distance(route_end, primary_end)) {
        const SoPoint tmp = route_start;
        route_start = route_end;
        route_end = tmp;
    }
    so_apply_uav_track_change_metrics(sim, task, route_area, route_start, route_end);
    const double outbound_m = so_distance(drone->position, route_start);
    const double outbound_energy = outbound_m / 1000.0 * so_drone_empty_drain_km(drone, &sim->spec);
    if (launching) {
        so_add_uav_takeoff_cost(sim);
        const int modules = drone->sortie_battery_modules >= 1 && drone->sortie_battery_modules <= 4
                                ? drone->sortie_battery_modules
                                : sim->spec.battery_modules;
        sim->uav_sorties_by_battery_modules[modules]++;
    }
    so_log_drone_transfer_segment(drone, drone->position, route_start);
    so_log_uav_task_route(sim, drone, task, route_area, reserved_area_before_ha,
                          route_start, route_end,
                          collaborative_slot);
    so_add_uav_flight_cost(sim, outbound_m);
    so_add_uav_drone_electricity_cost(sim, drone, outbound_energy + task->turn_energy_cost);
    drone->battery = fmax(0.0, drone->battery - outbound_energy - task->turn_energy_cost);
    drone->position = route_end;
    drone->state = task->remaining_ha <= sortie_capacity && task->remaining_ha <= 2.5 ? SO_DRONE_CLEANUP : SO_DRONE_WORKING;
    if (so_repair_sized_task(sim, task)) {
        drone->state = SO_DRONE_ASSISTING;
    }
    drone->assigned_task_id = task->id;
    drone->assigned_area_ha = assigned_area;
    drone->assigned_task_area_ha = route_area;
    if (collaborative_slot <= 0) {
        so_store_task_route(task, route_start, route_end, route_curve_deg);
    }
    so_add_uav_flight_cost(sim, so_task_spray_distance_m(sim, route_area) *
                                    so_curve_length_factor(task->route_curve_deg));
    drone->target = route_end;
    drone->has_target = true;
    drone->travel_remaining_s = task->turn_time_s;
    if (launching) {
        drone->travel_remaining_s += takeoff_service_s;
    }
    const double task_battery = (drone->assigned_area_ha / fmax(0.001, sim->spec.spray_rate_ha_h)) *
                                so_drone_work_drain_h(drone, &sim->spec) + task->turn_energy_cost;
    const double return_energy = so_estimate_return_energy(drone, recovery_point, sim->spec);
    drone->target_charge = fmax(0.35, fmin(0.8, task_battery + return_energy + sim->spec.safety_battery_margin));
    task->status = SO_TASK_IN_PROGRESS;
    task->assigned_drone_id = drone->id;
}

static int so_choose_bundle_continuation(SoSimulation *sim, const SoDrone *drone) {
    double best_score = 1e100;
    int best = -1;
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *task = &sim->field.tasks[i];
        if (!so_task_open(task) || task->kind == SO_TASK_REPAIR) {
            continue;
        }
        if (so_distance(sim->mothership.position, task->center) > so_task_service_radius(sim, task)) {
            continue;
        }
        SoPoint route_start = task->center;
        SoPoint route_end = task->center;
        double route_curve_deg = 0.0;
        so_task_directed_route(sim, task,
                               fmin(task->remaining_ha, drone->assigned_area_ha),
                               drone->position, sim->mothership.position,
                               &route_start, &route_end, &route_curve_deg);
        const double leg = so_distance(drone->position, route_start);
        if (leg > 260.0 && task->kind != SO_TASK_BOUNDARY) {
            continue;
        }
        double score =
            leg * sim->planner_weights.bundle_empty_w +
            task->risk * sim->planner_weights.bundle_risk_w;
        if (task->kind == SO_TASK_INTERIOR_STRIP) {
            score -= sim->planner_weights.bundle_route_eff_bonus_w * task->route_efficiency;
        }
        if (task->kind == SO_TASK_BOUNDARY && so_zone_has_open_interior(sim, task->zone_id)) {
            score += sim->planner_weights.bundle_boundary_delay_w;
        }
        if (score < best_score) {
            best_score = score;
            best = i;
        }
    }
    return best;
}

static void so_continue_bundle_or_return(SoSimulation *sim, SoDrone *drone) {
    if (drone->assigned_area_ha <= 0.35 || drone->battery <= 0.28 || drone->chemical <= 0.12) {
        drone->assigned_task_id = -1;
        drone->assigned_task_area_ha = 0.0;
        drone->state = SO_DRONE_RETURNING;
        return;
    }
    const int next_idx = so_choose_bundle_continuation(sim, drone);
    if (next_idx < 0) {
        drone->assigned_task_id = -1;
        drone->assigned_task_area_ha = 0.0;
        drone->state = SO_DRONE_RETURNING;
        return;
    }
    SoFieldTask *next = &sim->field.tasks[next_idx];
    const double assigned_area = fmin(next->remaining_ha, drone->assigned_area_ha);
    SoPoint route_start = next->center;
    SoPoint route_end = next->center;
    double route_curve_deg = 0.0;
    so_task_directed_route(sim, next, assigned_area,
                           drone->position, sim->mothership.position,
                           &route_start, &route_end, &route_curve_deg);
    so_apply_uav_track_change_metrics(sim, next, assigned_area, route_start, route_end);
    const double outbound_m = so_distance(drone->position, route_start);
    const double outbound_energy = outbound_m / 1000.0 * so_drone_empty_drain_km(drone, &sim->spec);
    so_log_drone_transfer_segment(drone, drone->position, route_start);
    so_log_uav_task_route(sim, drone, next, assigned_area, 0.0,
                          route_start, route_end, 0);
    so_add_uav_flight_cost(sim, outbound_m);
    so_add_uav_drone_electricity_cost(sim, drone, outbound_energy + next->turn_energy_cost);
    drone->battery = fmax(0.0, drone->battery - outbound_energy - next->turn_energy_cost);
    drone->position = route_end;
    drone->assigned_task_id = next->id;
    drone->target = route_end;
    drone->travel_remaining_s = next->turn_time_s;
    drone->state = next->kind == SO_TASK_BOUNDARY ? SO_DRONE_CLEANUP : SO_DRONE_WORKING;
    drone->assigned_area_ha = assigned_area;
    drone->assigned_task_area_ha = assigned_area;
    so_store_task_route(next, route_start, route_end, route_curve_deg);
    so_add_uav_flight_cost(sim, so_task_spray_distance_m(sim, drone->assigned_area_ha) *
                                    so_curve_length_factor(next->route_curve_deg));
    next->status = SO_TASK_IN_PROGRESS;
    next->assigned_drone_id = drone->id;
}

static bool so_can_finish_relocation_cleanup(const SoSimulation *sim,
                                             const SoDrone *drone,
                                             const SoFieldTask *task,
                                             SoWeatherAdjustedSpec weather) {
    if (!sim->mothership.moving || !so_repair_sized_task(sim, task)) {
        return false;
    }
    if (drone->battery <= 0.50) {
        return false;
    }

    const double empty_s = so_distance(drone->position, task->center) / fmax(0.001, weather.cruise_speed_mps);
    const double work_s = task->remaining_ha /
                          fmax(0.001, weather.spray_rate_ha_h * weather.spray_effectiveness *
                                          fmax(0.5, task->route_efficiency)) *
                          3600.0;
    const double recover_s = so_distance(task->center, sim->mothership.destination) /
                             fmax(0.001, weather.cruise_speed_mps);
    const double total_s = empty_s + work_s + recover_s + 90.0;
    if (total_s > sim->mothership.move_remaining_s + 180.0) {
        return false;
    }

    const double return_energy = so_estimate_return_energy(drone, sim->mothership.destination, sim->spec);
    const double available_battery = fmax(0.0, drone->battery - return_energy - sim->spec.safety_battery_margin);
    const double battery_area =
        available_battery / so_drone_work_drain_h(drone, &sim->spec) * sim->spec.spray_rate_ha_h;
    const double chemical_area = drone->chemical / so_drone_chemical_per_ha(drone, &sim->spec);
    const double capacity = fmax(0.0, fmin(battery_area, chemical_area));
    return capacity >= task->remaining_ha + 0.05;
}

static int so_choose_relocation_cleanup_task(SoSimulation *sim,
                                             const SoDrone *drone,
                                             SoWeatherAdjustedSpec weather) {
    double best_score = 1e100;
    int best = -1;

    for (int i = 0; i < sim->field.task_count; i++) {
        SoFieldTask *task = &sim->field.tasks[i];
        if (!so_task_open(task) || !so_can_finish_relocation_cleanup(sim, drone, task, weather)) {
            continue;
        }

        const double empty_dist = so_distance(drone->position, task->center);
        const double destination_dist = so_distance(task->center, sim->mothership.destination);
        const double old_stop_dist = so_distance(task->center, sim->mothership.position);
        const double moving_value = fmax(0.0, old_stop_dist - destination_dist) / 20.0;
        const double repair_value = so_repair_sized_task(sim, task) ? 32.0 : 0.0;
        const double score =
            empty_dist * sim->planner_weights.uav_empty_w +
            destination_dist * sim->planner_weights.uav_return_w * (14.0 / 18.0) +
            task->risk * sim->planner_weights.uav_risk_w * (25.0 / 45.0) -
                             task->priority * 8.0 - moving_value - repair_value;

        if (score < best_score) {
            best_score = score;
            best = i;
        }
    }

    return best;
}

static void so_assign_relocation_cleanup(SoSimulation *sim, SoWeatherAdjustedSpec weather) {
    if (!sim->mothership.moving || !weather.spray_allowed) {
        return;
    }

    int assigned = 0;
    for (int pass = 0; pass < 2; pass++) {
        int best = -1;
        double best_battery = -1.0;
        for (int i = 0; i < sim->drone_count; i++) {
            SoDrone *drone = &sim->drones[i];
            if (drone->state != SO_DRONE_STANDBY || drone->battery <= 0.50 || drone->chemical < 0.18) {
                continue;
            }
            if (drone->battery > best_battery) {
                best_battery = drone->battery;
                best = i;
            }
        }
        if (best < 0) {
            break;
        }
        SoDrone *drone = &sim->drones[best];
        const int task_idx = so_choose_relocation_cleanup_task(sim, drone, weather);
        if (task_idx < 0) {
            break;
        }

        SoFieldTask *task = &sim->field.tasks[task_idx];
        const double capacity = so_dynamic_capacity(drone, sim->mothership.destination, sim->spec);
        const double takeoff_service_s = so_random_uav_service_s(sim);
        if (!so_consume_launch_landing_service(sim, takeoff_service_s)) {
            break;
        }
        so_assign_drone_to_task(sim, drone, task, capacity, takeoff_service_s, 0,
                                sim->mothership.destination);
        drone->state = SO_DRONE_CLEANUP;
        assigned++;
        so_event(sim, "relocation cleanup assigned");
        if (assigned >= 2) {
            break;
        }
    }
}

static int so_active_field_drone_count(const SoSimulation *sim) {
    int count = 0;
    for (int i = 0; i < sim->drone_count; i++) {
        const SoDrone *drone = &sim->drones[i];
        if (drone->state == SO_DRONE_SCOUTING ||
            drone->state == SO_DRONE_WORKING ||
            drone->state == SO_DRONE_ASSISTING ||
            drone->state == SO_DRONE_CLEANUP ||
            drone->state == SO_DRONE_RETURNING) {
            count++;
        }
    }
    return count;
}

static int so_cleanup_open_near(const SoSimulation *sim, SoPoint point, double radius) {
    int count = 0;
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *task = &sim->field.tasks[i];
        if (task->status != SO_TASK_DONE && task->remaining_ha > 0.001 &&
            so_distance(point, task->center) <= radius) {
            count++;
        }
    }
    return count;
}

static bool so_score_strip_angle_deg(const SoSimulation *sim,
                                     const SoFieldBlock *block,
                                     double angle_deg,
                                     SoStripAngleCandidate *out_candidate) {
    if (block == NULL || block->boundary_count < 3) {
        return false;
    }

    const double swath = sim->fixed_wing.enabled && sim->fixed_wing.swath_width_m > 1.0
                             ? sim->fixed_wing.swath_width_m
                             : fmax(3.2, sim->spec.spray_swath_m);
    const double spacing = fmax(swath, sim->fixed_wing.enabled ? 18.0 : sim->spec.spray_swath_m);
    const double empty_connection_weight =
        (sim->fixed_wing.enabled ? 1.15 : 0.72) *
        SO_INTER_FIELD_EMPTY_PENALTY_SCALE *
        sim->planner_weights.strip_empty_w;
    const double turn_radius = sim->fixed_wing.enabled && sim->fixed_wing.turn_radius_m > 1.0
                                   ? sim->fixed_wing.turn_radius_m
                                   : fmax(1.0, sim->spec.turn_radius_m);
    const SoPoint entry_ref = sim->fixed_wing.enabled ? sim->fixed_wing.airport : sim->mothership.position;
    const double approach_heading = atan2(block->center.y - entry_ref.y, block->center.x - entry_ref.x);
    const double wind_rad = sim->mothership.weather.wind_direction_deg * M_PI / 180.0;
    const double wind_strength = fmin(1.0, sim->mothership.weather.wind_speed_mps / 8.0);
    const double rad = angle_deg * M_PI / 180.0;
    double min_cross = 0.0;
    double max_cross = 0.0;
    so_block_projection_range(block, angle_deg, &min_cross, &max_cross);
    if (max_cross <= min_cross) {
        return false;
    }

    double work = 0.0;
    double empty = 0.0;
    int rows = 0;
    for (double cross = min_cross + spacing * 0.5;
         cross <= max_cross - spacing * 0.25;
         cross += spacing) {
        double mins[SO_MAX_BOUNDARY_POINTS / 2];
        double maxs[SO_MAX_BOUNDARY_POINTS / 2];
        const int intervals = so_line_block_intervals(block, rad, cross, mins, maxs, SO_MAX_BOUNDARY_POINTS / 2);
        if (intervals <= 0) {
            continue;
        }
        double row_min = 1e100;
        double row_max = -1e100;
        double row_work = 0.0;
        for (int i = 0; i < intervals; i++) {
            row_min = fmin(row_min, mins[i]);
            row_max = fmax(row_max, maxs[i]);
            row_work += maxs[i] - mins[i];
        }
        if (row_work <= 8.0 || row_max <= row_min) {
            continue;
        }
        work += row_work;
        empty += fmax(0.0, row_max - row_min - row_work);
        rows++;
    }
    if (rows <= 0 || work <= 0.0) {
        return false;
    }

    const double entry_turn_m =
        fmin(so_angle_diff_rad(approach_heading, rad),
             so_angle_diff_rad(approach_heading, rad + M_PI)) * turn_radius;
    const double avg_row_m = work / (double)rows;
    const double crosswind = fabs(sin(rad - wind_rad));
    const double wind_penalty =
        crosswind * wind_strength *
        sim->planner_weights.strip_crosswind_w * (1.0 + block->risk);
    const double score =
        work
        - empty * empty_connection_weight
        - (double)rows * swath * sim->planner_weights.strip_row_w
        - entry_turn_m * 0.45
        - wind_penalty
        + avg_row_m * sim->planner_weights.strip_avg_row_bonus_w;

    if (out_candidate != NULL) {
        out_candidate->angle_deg = angle_deg;
        out_candidate->score = score;
        out_candidate->work_m = work;
        out_candidate->empty_m = empty;
        out_candidate->row_count = rows;
    }
    return true;
}

static int so_strip_angle_candidates(const SoSimulation *sim,
                                     const SoFieldBlock *block,
                                     SoStripAngleCandidate candidates[SO_ANGLE_CANDIDATE_COUNT]) {
    int count = 0;
    for (int a = 0; a < 180; a += 3) {
        SoStripAngleCandidate candidate;
        if (!so_score_strip_angle_deg(sim, block, (double)a, &candidate)) {
            continue;
        }
        int insert_at = count;
        while (insert_at > 0 && candidates[insert_at - 1].score < candidate.score) {
            if (insert_at < SO_ANGLE_CANDIDATE_COUNT) {
                candidates[insert_at] = candidates[insert_at - 1];
            }
            insert_at--;
        }
        if (insert_at < SO_ANGLE_CANDIDATE_COUNT) {
            candidates[insert_at] = candidate;
            if (count < SO_ANGLE_CANDIDATE_COUNT) {
                count++;
            }
        }
    }
    if (count == 0) {
        candidates[0].angle_deg = 0.0;
        candidates[0].score = 0.0;
        candidates[0].work_m = 0.0;
        candidates[0].empty_m = 0.0;
        candidates[0].row_count = 0;
        return 1;
    }
    return count;
}

static double so_angle_transition_cost(const SoSimulation *sim,
                                       const SoFieldBlock *from_block,
                                       double from_angle_deg,
                                       const SoFieldBlock *to_block,
                                       double to_angle_deg) {
    const double radius = sim->fixed_wing.enabled && sim->fixed_wing.turn_radius_m > 1.0
                              ? sim->fixed_wing.turn_radius_m
                              : fmax(1.0, sim->spec.turn_radius_m);
    const double from_rad = from_angle_deg * M_PI / 180.0;
    const double to_rad = to_angle_deg * M_PI / 180.0;
    const double center_dist = so_distance(from_block->center, to_block->center);
    const double direct_heading = so_heading_between(from_block->center, to_block->center);
    const double exit_turn_m =
        fmin(so_angle_diff_rad(from_rad, direct_heading),
             so_angle_diff_rad(from_rad + M_PI, direct_heading)) * radius;
    const double entry_turn_m =
        fmin(so_angle_diff_rad(direct_heading, to_rad),
             so_angle_diff_rad(direct_heading, to_rad + M_PI)) * radius;
    const double angle_change_m = so_angle_diff_rad(from_rad, to_rad) * radius;
    const double platform_scale = sim->fixed_wing.enabled ? 1.0 : 0.42;
    return center_dist * sim->planner_weights.angle_transition_empty_w *
               SO_INTER_FIELD_EMPTY_PENALTY_SCALE +
           (exit_turn_m + entry_turn_m) *
               sim->planner_weights.angle_transition_turn_w * platform_scale +
           angle_change_m *
               sim->planner_weights.angle_transition_change_w * platform_scale;
}

static double so_angle_endpoint_cost(const SoSimulation *sim,
                                     SoPoint endpoint,
                                     const SoFieldBlock *block,
                                     double angle_deg) {
    const double radius = sim->fixed_wing.enabled && sim->fixed_wing.turn_radius_m > 1.0
                              ? sim->fixed_wing.turn_radius_m
                              : fmax(1.0, sim->spec.turn_radius_m);
    const double angle_rad = angle_deg * M_PI / 180.0;
    const double heading = so_heading_between(endpoint, block->center);
    const double distance_m = so_distance(endpoint, block->center);
    const double entry_turn_m =
        fmin(so_angle_diff_rad(heading, angle_rad),
             so_angle_diff_rad(heading, angle_rad + M_PI)) * radius;
    const double platform_scale = sim->fixed_wing.enabled ? 1.0 : 0.42;
    return distance_m * sim->planner_weights.angle_endpoint_empty_w *
               SO_INTER_FIELD_EMPTY_PENALTY_SCALE +
           entry_turn_m *
               sim->planner_weights.angle_endpoint_turn_w * platform_scale;
}

static void so_choose_partition_dp_angles(const SoSimulation *sim,
                                          double out_angles[SO_MAX_BLOCKS]) {
    int block_indices[SO_MAX_BLOCKS];
    int block_count = 0;
    SoStripAngleCandidate candidates[SO_MAX_BLOCKS][SO_ANGLE_CANDIDATE_COUNT];
    int candidate_counts[SO_MAX_BLOCKS] = {0};
    double dp[SO_MAX_BLOCKS][SO_ANGLE_CANDIDATE_COUNT];
    int previous[SO_MAX_BLOCKS][SO_ANGLE_CANDIDATE_COUNT];

    for (int i = 0; i < SO_MAX_BLOCKS; i++) {
        out_angles[i] = 0.0;
    }

    for (int b = 0; b < sim->field.block_count && block_count < SO_MAX_BLOCKS; b++) {
        const SoFieldBlock *block = &sim->field.blocks[b];
        if (!block->selected) {
            continue;
        }
        block_indices[block_count] = b;
        candidate_counts[block_count] =
            so_strip_angle_candidates(sim, block, candidates[block_count]);
        out_angles[b] = candidates[block_count][0].angle_deg;
        block_count++;
    }

    if (block_count <= 0) {
        return;
    }

    const SoPoint start = sim->fixed_wing.enabled ? sim->fixed_wing.airport : sim->mothership.position;
    for (int i = 0; i < block_count; i++) {
        for (int j = 0; j < SO_ANGLE_CANDIDATE_COUNT; j++) {
            dp[i][j] = 1e100;
            previous[i][j] = -1;
        }
    }

    const SoFieldBlock *first_block = &sim->field.blocks[block_indices[0]];
    for (int c = 0; c < candidate_counts[0]; c++) {
        dp[0][c] = -candidates[0][c].score +
                   so_angle_endpoint_cost(sim, start, first_block, candidates[0][c].angle_deg);
    }

    for (int i = 1; i < block_count; i++) {
        const SoFieldBlock *prev_block = &sim->field.blocks[block_indices[i - 1]];
        const SoFieldBlock *block = &sim->field.blocks[block_indices[i]];
        for (int c = 0; c < candidate_counts[i]; c++) {
            const double local_cost = -candidates[i][c].score;
            for (int p = 0; p < candidate_counts[i - 1]; p++) {
                const double transition_cost =
                    so_angle_transition_cost(sim,
                                             prev_block,
                                             candidates[i - 1][p].angle_deg,
                                             block,
                                             candidates[i][c].angle_deg);
                const double cost = dp[i - 1][p] + local_cost + transition_cost;
                if (cost < dp[i][c]) {
                    dp[i][c] = cost;
                    previous[i][c] = p;
                }
            }
        }
    }

    int best_candidate = 0;
    double best_cost = 1e100;
    const SoPoint end = sim->fixed_wing.enabled ? sim->fixed_wing.airport : sim->mothership.position;
    const int last = block_count - 1;
    const SoFieldBlock *last_block = &sim->field.blocks[block_indices[last]];
    for (int c = 0; c < candidate_counts[last]; c++) {
        const double total_cost =
            dp[last][c] +
            so_angle_endpoint_cost(sim, end, last_block, candidates[last][c].angle_deg);
        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_candidate = c;
        }
    }

    for (int i = last; i >= 0; i--) {
        const int block_index = block_indices[i];
        out_angles[block_index] = candidates[i][best_candidate].angle_deg;
        best_candidate = previous[i][best_candidate];
        if (best_candidate < 0 && i > 0) {
            best_candidate = 0;
        }
    }
}

static void so_choose_global_strip_angles(SoSimulation *sim, double out_angles[SO_MAX_BLOCKS]) {
    so_choose_partition_dp_angles(sim, out_angles);
}

static SoPoint so_offset_by_angle(SoPoint center, double angle_deg, double along, double cross) {
    const double rad = angle_deg * M_PI / 180.0;
    const double ux = cos(rad);
    const double uy = sin(rad);
    const double vx = -uy;
    const double vy = ux;
    return so_point(center.x + ux * along + vx * cross, center.y + uy * along + vy * cross);
}

static void so_block_projection_range(const SoFieldBlock *block,
                                      double angle_deg,
                                      double *out_min_cross,
                                      double *out_max_cross) {
    const double rad = angle_deg * M_PI / 180.0;
    const double vx = -sin(rad);
    const double vy = cos(rad);
    double min_cross = 1e100;
    double max_cross = -1e100;

    if (block != NULL && block->boundary_count >= 3) {
        for (int i = 0; i < block->boundary_count; i++) {
            const double c = block->boundary[i].x * vx + block->boundary[i].y * vy;
            min_cross = fmin(min_cross, c);
            max_cross = fmax(max_cross, c);
        }
    } else if (block != NULL) {
        const double side = sqrt(fmax(1.0, block->area_ha) * 10000.0);
        const double c = block->center.x * vx + block->center.y * vy;
        min_cross = c - side * 0.5;
        max_cross = c + side * 0.5;
    }

    if (min_cross > max_cross) {
        min_cross = -100.0;
        max_cross = 100.0;
    }
    *out_min_cross = min_cross;
    *out_max_cross = max_cross;
}

static void so_add_task(SoSimulation *sim,
                        int zone_id,
                        int block_id,
                        SoPoint center,
                        double area_ha,
                        double priority,
                        double risk,
                        double angle,
                        double route_efficiency,
                        SoTaskKind kind) {
    if (area_ha <= 0.001) {
        return;
    }
    if (sim->field.task_count >= SO_MAX_TASKS) {
        sim->field.dropped_task_count++;
        sim->field.dropped_task_area_ha += area_ha;
        if (sim->field.dropped_task_count == 1 ||
            sim->field.dropped_task_count % 16 == 0) {
            char msg[160];
            snprintf(msg, sizeof(msg),
                     "task dropped: SO_MAX_TASKS reached count=%d area=%.3fha total_dropped=%.3fha",
                     sim->field.dropped_task_count,
                     area_ha,
                     sim->field.dropped_task_area_ha);
            so_event(sim, msg);
        }
        return;
    }
    SoFieldTask *task = &sim->field.tasks[sim->field.task_count];
    int max_id = 0;
    for (int i = 0; i < sim->field.task_count; i++) {
        if (sim->field.tasks[i].id > max_id) {
            max_id = sim->field.tasks[i].id;
        }
    }
    task->id = max_id + 1;
    task->zone_id = zone_id;
    task->block_id = block_id;
    task->center = center;
    task->area_ha = area_ha;
    task->remaining_ha = area_ha;
    task->priority = priority;
    task->risk = risk;
    task->strip_angle_deg = angle;
    task->route_efficiency = route_efficiency;
    task->has_planned_route = false;
    task->route_start = center;
    task->route_mid = center;
    task->route_end = center;
    task->has_route_mid = false;
    task->route_curve_deg = 0.0;
    task->fixed_wing_area_ha = 0.0;
    task->turn_count = kind == SO_TASK_INTERIOR_STRIP ? 2 : 4;
    task->turn_time_s = (double)task->turn_count * sim->spec.turn_time_s;
    task->turn_energy_cost = (double)task->turn_count * sim->spec.turn_battery_cost;
    task->bundle_hint = zone_id;
    task->kind = kind;
    task->status = SO_TASK_PENDING;
    task->assigned_drone_id = -1;
    sim->field.task_count++;
}

static void so_add_routed_task(SoSimulation *sim,
                               int zone_id,
                               int block_id,
                               SoPoint start,
                               SoPoint end,
                               double area_ha,
                               double priority,
                               double risk,
                               double angle,
                               double route_efficiency,
                               SoTaskKind kind) {
    const int before = sim->field.task_count;
    const SoPoint center = so_point((start.x + end.x) * 0.5, (start.y + end.y) * 0.5);
    so_add_task(sim, zone_id, block_id, center, area_ha, priority, risk, angle, route_efficiency, kind);
    if (sim->field.task_count > before) {
        SoFieldTask *task = &sim->field.tasks[before];
        so_store_task_route(task, start, end, 0.0);
    }
}

static int so_add_scanline_interior_tasks(SoSimulation *sim,
                                          const SoFieldBlock *block,
                                          double angle,
                                          double interior_area_ha,
                                          double target_task_area_ha) {
    if (block == NULL || block->boundary_count < 3 || interior_area_ha <= 0.001) {
        return 0;
    }

    double min_cross = 0.0;
    double max_cross = 0.0;
    so_block_projection_range(block, angle, &min_cross, &max_cross);
    if (max_cross <= min_cross) {
        return 0;
    }

    const double angle_rad = angle * M_PI / 180.0;
    const double ux = cos(angle_rad);
    const double uy = sin(angle_rad);
    const double vx = -uy;
    const double vy = ux;
    const double strip_width_m = sim->fixed_wing.enabled ? 19.8 : fmax(3.2, sim->spec.spray_swath_m);
    const double spacing = fmax(strip_width_m, sim->fixed_wing.enabled ? 18.0 : sim->spec.spray_swath_m);
    const double target_area = fmax(0.35, target_task_area_ha);
    double remaining_area = interior_area_ha;

    if (sim->fixed_wing.enabled) {
        int made = 0;
        for (double cross = min_cross + spacing * 0.5;
             cross <= max_cross - spacing * 0.25 && remaining_area > 0.001 &&
             sim->field.task_count < SO_MAX_TASKS;
             cross += spacing) {
            double mins[SO_MAX_BOUNDARY_POINTS / 2];
            double maxs[SO_MAX_BOUNDARY_POINTS / 2];
            const int intervals = so_line_block_intervals(block, angle_rad, cross, mins, maxs, SO_MAX_BOUNDARY_POINTS / 2);
            for (int i = 0; i < intervals && remaining_area > 0.001 && sim->field.task_count < SO_MAX_TASKS; i++) {
                const double len = maxs[i] - mins[i];
                if (len <= 8.0) {
                    continue;
                }
                double seg_area = len * spacing / 10000.0;
                if (seg_area > remaining_area) {
                    seg_area = remaining_area;
                }
                if (seg_area <= 0.001) {
                    break;
                }
                const double start_t = mins[i];
                const double end_t = maxs[i];
                const SoPoint start = so_point(ux * start_t + vx * cross, uy * start_t + vy * cross);
                const SoPoint end = so_point(ux * end_t + vx * cross, uy * end_t + vy * cross);
                const int zone_id = sim->field.zone_count > 0 ? 1 + (made % sim->field.zone_count) : 1;
                so_add_routed_task(sim, zone_id, block->id, start, end, seg_area,
                                   1.2 + block->risk, block->risk, angle,
                                   1.12 + fmax(0.0, 0.35 - block->risk) * 0.18,
                                   SO_TASK_INTERIOR_STRIP);
                made++;
                remaining_area = fmax(0.0, remaining_area - seg_area);
            }
        }
        return made;
    }

    double acc_area = 0.0;
    double acc_x = 0.0;
    double acc_y = 0.0;
    double acc_work_m = 0.0;
    double acc_total_m = 0.0;
    int acc_segments = 0;
    int made = 0;

    for (double cross = min_cross + spacing * 0.5;
         cross <= max_cross - spacing * 0.25 && remaining_area > 0.001 &&
         sim->field.task_count < SO_MAX_TASKS;
         cross += spacing) {
        double mins[SO_MAX_BOUNDARY_POINTS / 2];
        double maxs[SO_MAX_BOUNDARY_POINTS / 2];
        const int intervals = so_line_block_intervals(block, angle_rad, cross, mins, maxs, SO_MAX_BOUNDARY_POINTS / 2);
        for (int i = 0; i < intervals && remaining_area > 0.001 && sim->field.task_count < SO_MAX_TASKS; i++) {
            const double len = maxs[i] - mins[i];
            if (len <= 8.0) {
                continue;
            }
            double seg_area = len * spacing / 10000.0;
            if (seg_area > remaining_area) {
                seg_area = remaining_area;
            }
            if (acc_area > 0.001 && acc_area + seg_area > target_area) {
                const int zone_id = sim->field.zone_count > 0 ? 1 + (made % sim->field.zone_count) : 1;
                const double route_eff = fmax(0.55, fmin(1.25, acc_work_m / fmax(1.0, acc_total_m)));
                so_add_task(sim, zone_id, block->id,
                            so_point(acc_x / acc_area, acc_y / acc_area),
                            acc_area,
                            1.2 + block->risk,
                            block->risk,
                            angle,
                            route_eff,
                            SO_TASK_INTERIOR_STRIP);
                made++;
                acc_area = 0.0;
                acc_x = 0.0;
                acc_y = 0.0;
                acc_work_m = 0.0;
                acc_total_m = 0.0;
                acc_segments = 0;
            }

            const double mid_t = (mins[i] + maxs[i]) * 0.5;
            const SoPoint midpoint = so_point(ux * mid_t + vx * cross, uy * mid_t + vy * cross);
            acc_area += seg_area;
            acc_x += midpoint.x * seg_area;
            acc_y += midpoint.y * seg_area;
            acc_work_m += len;
            acc_total_m += len;
            acc_segments++;
            remaining_area = fmax(0.0, remaining_area - seg_area);
        }
    }

    if (acc_area > 0.001 && sim->field.task_count < SO_MAX_TASKS) {
        const int zone_id = sim->field.zone_count > 0 ? 1 + (made % sim->field.zone_count) : 1;
        const double fragment_penalty = acc_segments > 1 ? 0.04 * (double)(acc_segments - 1) : 0.0;
        const double route_eff = fmax(0.55, fmin(1.25, acc_work_m / fmax(1.0, acc_total_m) - fragment_penalty));
        so_add_task(sim, zone_id, block->id,
                    so_point(acc_x / acc_area, acc_y / acc_area),
                    acc_area,
                    1.2 + block->risk,
                    block->risk,
                    angle,
                    route_eff,
                    SO_TASK_INTERIOR_STRIP);
        made++;
    }

    return made;
}

static bool so_score_global_corridor_angle(const SoSimulation *sim,
                                           double angle_deg,
                                           const double interior_budget[SO_MAX_BLOCKS],
                                           SoStripAngleCandidate *out_candidate) {
    const double swath =
        sim->fixed_wing.enabled && sim->fixed_wing.swath_width_m > 1.0
            ? sim->fixed_wing.swath_width_m
            : fmax(3.2, sim->spec.spray_swath_m);
    const double spacing = fmax(swath, 18.0);
    const double angle_rad = angle_deg * M_PI / 180.0;
    const double vx = -sin(angle_rad);
    const double vy = cos(angle_rad);
    double min_cross = 1e100;
    double max_cross = -1e100;

    for (int b = 0; b < sim->field.block_count; b++) {
        const SoFieldBlock *block = &sim->field.blocks[b];
        if (!block->selected || block->boundary_count < 3 || interior_budget[b] <= 0.001) {
            continue;
        }
        for (int p = 0; p < block->boundary_count; p++) {
            const double c = block->boundary[p].x * vx + block->boundary[p].y * vy;
            min_cross = fmin(min_cross, c);
            max_cross = fmax(max_cross, c);
        }
    }
    if (min_cross > max_cross) {
        return false;
    }

    double work = 0.0;
    double empty = 0.0;
    int rows = 0;
    for (double cross = min_cross + spacing * 0.5;
         cross <= max_cross - spacing * 0.25;
         cross += spacing) {
        double row_min = 1e100;
        double row_max = -1e100;
        double row_work = 0.0;
        int row_fragments = 0;
        for (int b = 0; b < sim->field.block_count; b++) {
            const SoFieldBlock *block = &sim->field.blocks[b];
            if (!block->selected || block->boundary_count < 3 || interior_budget[b] <= 0.001) {
                continue;
            }
            double mins[SO_MAX_BOUNDARY_POINTS / 2];
            double maxs[SO_MAX_BOUNDARY_POINTS / 2];
            const int intervals =
                so_line_block_intervals(block, angle_rad, cross, mins, maxs,
                                        SO_MAX_BOUNDARY_POINTS / 2);
            for (int i = 0; i < intervals; i++) {
                const double len = maxs[i] - mins[i];
                if (len <= 8.0) {
                    continue;
                }
                row_min = fmin(row_min, mins[i]);
                row_max = fmax(row_max, maxs[i]);
                row_work += len;
                row_fragments++;
            }
        }
        if (row_work <= 8.0 || row_max <= row_min) {
            continue;
        }
        work += row_work;
        empty += fmax(0.0, row_max - row_min - row_work);
        rows += fmax(1, row_fragments);
    }
    if (work <= 0.0 || rows <= 0) {
        return false;
    }

    const double wind_rad = sim->mothership.weather.wind_direction_deg * M_PI / 180.0;
    const double crosswind = fabs(sin(angle_rad - wind_rad));
    const double wind_strength = fmin(1.0, sim->mothership.weather.wind_speed_mps / 8.0);
    const double avg_row_m = work / (double)rows;
    const double score =
        work
        - empty * sim->planner_weights.global_empty_w
        - (double)rows * swath * sim->planner_weights.global_row_w
        - crosswind * wind_strength * sim->planner_weights.global_crosswind_w
        + avg_row_m * sim->planner_weights.global_avg_row_bonus_w;

    if (out_candidate != NULL) {
        out_candidate->angle_deg = angle_deg;
        out_candidate->score = score;
        out_candidate->work_m = work;
        out_candidate->empty_m = empty;
        out_candidate->row_count = rows;
    }
    return true;
}

static int so_global_corridor_angle_candidates(const SoSimulation *sim,
                                               const double interior_budget[SO_MAX_BLOCKS],
                                               SoStripAngleCandidate candidates[SO_ANGLE_CANDIDATE_COUNT]) {
    int count = 0;
    for (int a = 0; a < 180; a += 3) {
        SoStripAngleCandidate candidate;
        if (!so_score_global_corridor_angle(sim, (double)a, interior_budget, &candidate)) {
            continue;
        }
        int insert_at = count;
        while (insert_at > 0 && candidates[insert_at - 1].score < candidate.score) {
            if (insert_at < SO_ANGLE_CANDIDATE_COUNT) {
                candidates[insert_at] = candidates[insert_at - 1];
            }
            insert_at--;
        }
        if (insert_at < SO_ANGLE_CANDIDATE_COUNT) {
            candidates[insert_at] = candidate;
            if (count < SO_ANGLE_CANDIDATE_COUNT) {
                count++;
            }
        }
    }
    return count;
}

static int so_add_path_first_fixed_wing_tasks(SoSimulation *sim,
                                              double interior_budget[SO_MAX_BLOCKS]) {
    if (!sim->fixed_wing.enabled) {
        return 0;
    }

    SoStripAngleCandidate candidates[SO_ANGLE_CANDIDATE_COUNT];
    const int candidate_count =
        so_global_corridor_angle_candidates(sim, interior_budget, candidates);
    if (candidate_count <= 0) {
        return 0;
    }

    const double swath =
        sim->fixed_wing.swath_width_m > 1.0 ? sim->fixed_wing.swath_width_m : 19.8;
    const double spacing = fmax(swath, 18.0);
    int made = 0;
    const int angle_limit = fmin(candidate_count, 4);

    for (int c = 0; c < angle_limit; c++) {
        const double angle = candidates[c].angle_deg;
        const double angle_rad = angle * M_PI / 180.0;
        const double ux = cos(angle_rad);
        const double uy = sin(angle_rad);
        const double vx = -uy;
        const double vy = ux;
        double min_cross = 1e100;
        double max_cross = -1e100;
        for (int b = 0; b < sim->field.block_count; b++) {
            const SoFieldBlock *block = &sim->field.blocks[b];
            if (!block->selected || block->boundary_count < 3 || interior_budget[b] <= 0.001) {
                continue;
            }
            for (int p = 0; p < block->boundary_count; p++) {
                const double cross = block->boundary[p].x * vx + block->boundary[p].y * vy;
                min_cross = fmin(min_cross, cross);
                max_cross = fmax(max_cross, cross);
            }
        }
        if (min_cross > max_cross) {
            continue;
        }

        for (double cross = min_cross + spacing * 0.5;
             cross <= max_cross - spacing * 0.25 && sim->field.task_count < SO_MAX_TASKS;
             cross += spacing) {
            double row_min = 1e100;
            double row_max = -1e100;
            double row_work = 0.0;
            typedef struct {
                int block_index;
                double min_t;
                double max_t;
                double length_m;
            } CorridorSegment;
            CorridorSegment segments[SO_MAX_BLOCKS * (SO_MAX_BOUNDARY_POINTS / 2)];
            int segment_count = 0;

            for (int b = 0; b < sim->field.block_count; b++) {
                const SoFieldBlock *block = &sim->field.blocks[b];
                if (!block->selected || block->boundary_count < 3 || interior_budget[b] <= 0.001) {
                    continue;
                }
                double mins[SO_MAX_BOUNDARY_POINTS / 2];
                double maxs[SO_MAX_BOUNDARY_POINTS / 2];
                const int intervals =
                    so_line_block_intervals(block, angle_rad, cross, mins, maxs,
                                            SO_MAX_BOUNDARY_POINTS / 2);
                for (int i = 0; i < intervals &&
                                segment_count < (int)(sizeof(segments) / sizeof(segments[0])); i++) {
                    const double len = maxs[i] - mins[i];
                    if (len <= 8.0) {
                        continue;
                    }
                    segments[segment_count++] =
                        (CorridorSegment){b, mins[i], maxs[i], len};
                    row_min = fmin(row_min, mins[i]);
                    row_max = fmax(row_max, maxs[i]);
                    row_work += len;
                }
            }
            if (segment_count <= 0 || row_work <= 8.0 || row_max <= row_min) {
                continue;
            }
            const double row_total = row_max - row_min;
            const double route_efficiency =
                fmax(0.45, fmin(1.25, row_work / fmax(1.0, row_total)));

            for (int s = 0; s < segment_count && sim->field.task_count < SO_MAX_TASKS; s++) {
                const int b = segments[s].block_index;
                if (interior_budget[b] <= 0.001) {
                    continue;
                }
                const double segment_area = segments[s].length_m * spacing / 10000.0;
                const double area = fmin(segment_area, interior_budget[b]);
                if (area < 0.10) {
                    continue;
                }
                const SoFieldBlock *block = &sim->field.blocks[b];
                const SoPoint start =
                    so_point(ux * segments[s].min_t + vx * cross,
                             uy * segments[s].min_t + vy * cross);
                const SoPoint end =
                    so_point(ux * segments[s].max_t + vx * cross,
                             uy * segments[s].max_t + vy * cross);
                const int zone_id =
                    sim->field.zone_count > 0 ? 1 + (made % sim->field.zone_count) : 1;
                so_add_routed_task(sim, zone_id, block->id, start, end, area,
                                   1.25 + block->risk, block->risk, angle,
                                   route_efficiency, SO_TASK_INTERIOR_STRIP);
                interior_budget[b] = fmax(0.0, interior_budget[b] - area);
                made++;
            }
        }
    }
    return made;
}

static void so_build_tasks(SoSimulation *sim) {
    sim->field.zone_count = 0;
    sim->field.task_count = 0;
    sim->field.dropped_task_count = 0;
    sim->field.dropped_task_area_ha = 0.0;
    sim->field.residual_rebuild_split_count = 0;
    sim->field.residual_rebuild_area_ha = 0.0;
    sim->field.residual_spatial_task_count = 0;
    sim->field.residual_spatial_area_ha = 0.0;

    if (sim->field.block_count == 0) {
        sim->field.blocks[0].id = 1;
        sim->field.blocks[0].name = "main";
        sim->field.blocks[0].center = sim->field.boundary_center;
        sim->field.blocks[0].area_ha = sim->field.area_ha;
        sim->field.blocks[0].risk = fmin(1.0, sim->field.terrain_complexity * 0.55 + sim->field.obstacle_density * 0.45);
        sim->field.blocks[0].selected = true;
        sim->field.block_count = 1;
    }

    for (int b = 0; b < sim->field.block_count; b++) {
        const SoFieldBlock *block = &sim->field.blocks[b];
        if (!block->selected) {
            continue;
        }
        const int zone_count = (int)fmax(1.0, round(block->area_ha / 8.0));
        const int cols = (int)fmax(1.0, round(sqrt((double)zone_count)));
        const double zone_area = block->area_ha / zone_count;
        const double spacing = fmax(90.0, sqrt(block->area_ha * 10000.0 / zone_count));
        for (int z = 0; z < zone_count && sim->field.zone_count < SO_MAX_ZONES; z++) {
            const int row = z / cols;
            const int col = z % cols;
            SoOperationZone *zone = &sim->field.zones[sim->field.zone_count];
            zone->id = sim->field.zone_count + 1;
            zone->block_id = block->id;
            zone->center = so_point(block->center.x + (col - (cols - 1) / 2.0) * spacing,
                                    block->center.y + (row - ((zone_count - 1) / cols) / 2.0) * spacing);
            zone->area_ha = zone_area;
            zone->treated_ha = 0.0;
            zone->risk = block->risk;
            sim->field.zone_count++;
        }
    }

    double global_strip_angles[SO_MAX_BLOCKS];
    so_choose_global_strip_angles(sim, global_strip_angles);

    double boundary_area_by_block[SO_MAX_BLOCKS] = {0.0};
    double repair_area_by_block[SO_MAX_BLOCKS] = {0.0};
    double interior_budget_by_block[SO_MAX_BLOCKS] = {0.0};
    for (int b = 0; b < sim->field.block_count; b++) {
        const SoFieldBlock *block = &sim->field.blocks[b];
        if (!block->selected) {
            continue;
        }
        const double perimeter_m = so_block_perimeter_m(block);
        const double uav_swath_m = fmax(3.2, sim->spec.spray_swath_m);
        const double block_area_m2 = fmax(1.0, block->area_ha * 10000.0);
        const double compactness = perimeter_m / fmax(1.0, sqrt(block_area_m2));
        const double geometry_pressure = fmin(1.0, fmax(0.0, (compactness - 4.0) / 3.0));
        const double uav_unit_cost =
            sim->spec.flight_cost_usd_per_km / fmax(0.001, uav_swath_m);
        const double fixed_unit_cost =
            (sim->fixed_wing.enabled && sim->fixed_wing.swath_width_m > 1.0)
                ? sim->fixed_wing.flight_cost_usd_per_km / sim->fixed_wing.swath_width_m
                : uav_unit_cost * 1.35;
        const double uav_cost_pressure =
            fmin(1.0, fmax(0.0, uav_unit_cost / fmax(0.001, fixed_unit_cost) - 0.75));
        const double edge_passes =
            1.05 + geometry_pressure * 0.45 + block->risk * 0.30;
        const double geometric_edge_area =
            perimeter_m * uav_swath_m * edge_passes / 10000.0;
        const double ratio_cap =
            block->area_ha * (0.015 + geometry_pressure * 0.012 + block->risk * 0.008);
        const double cost_aware_cap =
            ratio_cap * (1.0 - uav_cost_pressure * 0.22);
        const double min_boundary_area =
            fmin(block->area_ha * 0.045, 1.35);
        boundary_area_by_block[b] =
            fmin(fmax(min_boundary_area, geometric_edge_area),
                 fmax(min_boundary_area, cost_aware_cap));
        repair_area_by_block[b] =
            block->risk > 0.32 ? fmin(block->area_ha * 0.035, 1.6) : 0.0;
        interior_budget_by_block[b] =
            fmax(0.1, block->area_ha - boundary_area_by_block[b] - repair_area_by_block[b]);
    }

    const int path_first_task_count =
        sim->fixed_wing.enabled
            ? so_add_path_first_fixed_wing_tasks(sim, interior_budget_by_block)
            : 0;

    for (int b = 0; b < sim->field.block_count; b++) {
        const SoFieldBlock *block = &sim->field.blocks[b];
        if (!block->selected) {
            continue;
        }
        const double angle = global_strip_angles[b];
        double boundary_area = boundary_area_by_block[b];
        const double repair_area = repair_area_by_block[b];
        const double interior_area = interior_budget_by_block[b];
        double min_cross = 0.0;
        double max_cross = 0.0;
        so_block_projection_range(block, angle, &min_cross, &max_cross);
        const double width = fmax(1.0, max_cross - min_cross);
        const double target_strip_area = sim->fixed_wing.enabled ? 12.0 : 5.6;
        const int interior_task_start = sim->field.task_count;
        const int made = so_add_scanline_interior_tasks(sim, block, angle, interior_area, target_strip_area);
        if (made <= 0) {
            so_add_task(sim, 1, block->id, block->center, interior_area,
                        1.2 + block->risk, block->risk, angle,
                        0.9, SO_TASK_INTERIOR_STRIP);
        } else if (sim->fixed_wing.enabled && path_first_task_count <= 0) {
            double snapped_interior_area = 0.0;
            for (int i = interior_task_start; i < sim->field.task_count; i++) {
                if (sim->field.tasks[i].block_id == block->id &&
                    sim->field.tasks[i].kind == SO_TASK_INTERIOR_STRIP) {
                    snapped_interior_area += sim->field.tasks[i].area_ha;
                }
            }
            const double min_boundary_area =
                fmin(block->area_ha * 0.045, 1.35);
            boundary_area = fmax(
                min_boundary_area,
                block->area_ha - repair_area - snapped_interior_area);
        }
        {
            const double angle_rad = angle * M_PI / 180.0;
            const double ux = cos(angle_rad);
            const double uy = sin(angle_rad);
            const double vx = -uy;
            const double vy = ux;
            const double boundary_cross =
                max_cross - fmax(3.2, sim->spec.spray_swath_m) * 0.5;
            double mins[SO_MAX_BOUNDARY_POINTS / 2];
            double maxs[SO_MAX_BOUNDARY_POINTS / 2];
            const int intervals = so_line_block_intervals(
                block, angle_rad, boundary_cross, mins, maxs,
                SO_MAX_BOUNDARY_POINTS / 2);
            int longest = -1;
            double longest_m = 0.0;
            for (int i = 0; i < intervals; i++) {
                const double length_m = maxs[i] - mins[i];
                if (length_m > longest_m) {
                    longest_m = length_m;
                    longest = i;
                }
            }
            if (longest >= 0) {
                const SoPoint start = so_point(
                    ux * mins[longest] + vx * boundary_cross,
                    uy * mins[longest] + vy * boundary_cross);
                const SoPoint end = so_point(
                    ux * maxs[longest] + vx * boundary_cross,
                    uy * maxs[longest] + vy * boundary_cross);
                const int before = sim->field.task_count;
                so_add_task(
                    sim, 1, block->id, block->center, boundary_area,
                    0.95 + block->risk, fmin(1.0, block->risk + 0.12),
                    angle, 0.78, SO_TASK_BOUNDARY);
                if (sim->field.task_count > before) {
                    SoFieldTask *boundary_task = &sim->field.tasks[before];
                    so_store_task_route(boundary_task, start, end, 0.0);
                }
            } else {
                so_add_task(
                    sim, 1, block->id, block->center, boundary_area,
                    0.95 + block->risk, fmin(1.0, block->risk + 0.12),
                    angle, 0.78, SO_TASK_BOUNDARY);
            }
        }
        if (repair_area > 0.001) {
            so_add_task(sim, 1, block->id, so_offset_by_angle(block->center, angle, width * 0.08, -width * 0.18),
                        repair_area, 0.65 + block->risk, fmin(1.0, block->risk + 0.18),
                        angle, 0.9, SO_TASK_REPAIR);
        }
    }
}

static void so_assign_work(SoSimulation *sim) {
    for (int i = 0; i < sim->drone_count; i++) {
        SoDrone *drone = &sim->drones[i];
        if (!((drone->state == SO_DRONE_IDLE || drone->state == SO_DRONE_STANDBY) &&
              drone->battery > 0.35 && drone->chemical > 0.2)) {
            continue;
        }
        const int task_idx = so_choose_task_for_drone(sim, drone, 0.0, sim->mothership.moving);
        if (task_idx < 0) {
            continue;
        }
        SoFieldTask *task = &sim->field.tasks[task_idx];
        const double capacity =
            so_estimate_dynamic_capacity(drone, sim->mothership.position,
                                         sim->spec, NULL);
        if (capacity < 0.05) {
            continue;
        }
        const double takeoff_service_s = so_random_uav_service_s(sim);
        if (!so_consume_launch_landing_service(sim, takeoff_service_s)) {
            continue;
        }
        so_assign_drone_to_task(sim, drone, task, capacity, takeoff_service_s, 0,
                                sim->mothership.position);
    }
}

static double so_active_assigned_area_for_task(const SoSimulation *sim, int task_id) {
    double assigned = 0.0;
    for (int i = 0; i < sim->drone_count; i++) {
        const SoDrone *drone = &sim->drones[i];
        if (drone->assigned_task_id != task_id) {
            continue;
        }
        if (drone->state == SO_DRONE_WORKING ||
            drone->state == SO_DRONE_ASSISTING ||
            drone->state == SO_DRONE_CLEANUP) {
            assigned += fmax(0.0, drone->assigned_task_area_ha);
        }
    }
    return assigned;
}

static int so_active_collaborator_count_for_task(const SoSimulation *sim, int task_id) {
    int count = 0;
    for (int i = 0; i < sim->drone_count; i++) {
        const SoDrone *drone = &sim->drones[i];
        if (drone->assigned_task_id != task_id) {
            continue;
        }
        if (drone->state == SO_DRONE_WORKING ||
            drone->state == SO_DRONE_ASSISTING ||
            drone->state == SO_DRONE_CLEANUP) {
            count++;
        }
    }
    return count;
}

static int so_active_drone_count_for_block(const SoSimulation *sim, int block_id) {
    int count = 0;
    for (int i = 0; i < sim->drone_count; i++) {
        const SoDrone *drone = &sim->drones[i];
        if (!(drone->state == SO_DRONE_WORKING ||
              drone->state == SO_DRONE_ASSISTING ||
              drone->state == SO_DRONE_CLEANUP)) {
            continue;
        }
        for (int t = 0; t < sim->field.task_count; t++) {
            const SoFieldTask *task = &sim->field.tasks[t];
            if (task->id == drone->assigned_task_id &&
                task->block_id == block_id) {
                count++;
                break;
            }
        }
    }
    return count;
}

static void so_assign_assist(SoSimulation *sim) {
    int active_count = 0;
    for (int i = 0; i < sim->drone_count; i++) {
        if (sim->drones[i].state == SO_DRONE_WORKING || sim->drones[i].state == SO_DRONE_ASSISTING ||
            sim->drones[i].state == SO_DRONE_CLEANUP) {
            active_count++;
        }
    }
    if (active_count >= sim->drone_count - 1) {
        return;
    }
    for (int i = 0; i < sim->drone_count; i++) {
        SoDrone *drone = &sim->drones[i];
        if (drone->state != SO_DRONE_STANDBY) {
            continue;
        }
        const double capacity =
            so_estimate_dynamic_capacity(drone, sim->mothership.position,
                                         sim->spec, NULL);
        if (capacity < 0.35) {
            continue;
        }
        int best = -1;
        double best_score = 1e100;
        double best_uncommitted = 0.0;
        const double radius = so_working_radius(sim);
        for (int t = 0; t < sim->field.task_count; t++) {
            SoFieldTask *task = &sim->field.tasks[t];
            if (task->status == SO_TASK_DONE ||
                task->kind != SO_TASK_INTERIOR_STRIP ||
                so_repair_sized_task(sim, task) ||
                so_distance(sim->mothership.position, task->center) > radius) {
                continue;
            }
            const double uncommitted =
                task->remaining_ha - so_active_assigned_area_for_task(sim, task->id);
            const int collaborators =
                so_active_collaborator_count_for_task(sim, task->id);
            if (uncommitted <= 0.2 || collaborators <= 0 || collaborators >= 3) {
                continue;
            }
            SoPoint assist_entry = task->center;
            if (task->has_planned_route &&
                so_distance(task->route_start, task->route_end) > 1.0) {
                assist_entry = task->route_end;
            }
            const double entry_dist = so_distance(drone->position, assist_entry);
            const double finish_fit =
                fmax(0.0, uncommitted - capacity) *
                    sim->planner_weights.assist_overfit_w +
                fmax(0.0, capacity - uncommitted) *
                    sim->planner_weights.assist_underfit_w;
            const double score =
                entry_dist * sim->planner_weights.assist_entry_w +
                finish_fit +
                (double)(collaborators - 1) *
                    sim->planner_weights.assist_collaborator_w +
                task->risk * sim->planner_weights.assist_risk_w -
                uncommitted *
                    sim->planner_weights.assist_uncommitted_bonus_w -
                task->route_efficiency *
                    sim->planner_weights.assist_route_eff_bonus_w;
            if (score < best_score) {
                best_score = score;
                best_uncommitted = uncommitted;
                best = t;
            }
        }
        if (best >= 0 && best_uncommitted > 0.2) {
            SoFieldTask *task = &sim->field.tasks[best];
            const double takeoff_service_s = so_random_uav_service_s(sim);
            if (!so_consume_launch_landing_service(sim, takeoff_service_s)) {
                continue;
            }
            const int collaborative_slot =
                so_active_collaborator_count_for_task(sim, task->id);
            so_assign_drone_to_task(sim, drone, task,
                                    fmin(capacity, best_uncommitted),
                                    takeoff_service_s,
                                    collaborative_slot,
                                    sim->mothership.position);
            drone->state = SO_DRONE_ASSISTING;
        }
    }
}

static SoFieldTask *so_find_task(SoSimulation *sim, int task_id) {
    for (int i = 0; i < sim->field.task_count; i++) {
        if (sim->field.tasks[i].id == task_id) {
            return &sim->field.tasks[i];
        }
    }
    return NULL;
}

static void so_release_task(SoSimulation *sim, SoDrone *drone) {
    SoFieldTask *task = so_find_task(sim, drone->assigned_task_id);
    if (task != NULL && task->status != SO_TASK_DONE) {
        task->status = SO_TASK_PENDING;
        task->assigned_drone_id = -1;
    }
    drone->assigned_task_id = -1;
    drone->assigned_area_ha = 0.0;
    drone->assigned_task_area_ha = 0.0;
}

static bool so_needs_recall(SoSimulation *sim, SoDrone *drone) {
    if (!(drone->state == SO_DRONE_SCOUTING || drone->state == SO_DRONE_WORKING ||
          drone->state == SO_DRONE_ASSISTING || drone->state == SO_DRONE_CLEANUP)) {
        return false;
    }
    const double return_energy = so_estimate_return_energy(drone, sim->mothership.position, sim->spec);
    return drone->battery <= return_energy + sim->spec.safety_battery_margin || drone->chemical < 0.05;
}

static void so_weather_recovery(SoSimulation *sim) {
    const SoWeatherSeverity severity = so_weather_severity(sim->mothership.weather);
    if (!(severity == SO_WEATHER_SEVERE || severity == SO_WEATHER_EMERGENCY)) {
        return;
    }
    for (int i = 0; i < sim->drone_count; i++) {
        SoDrone *drone = &sim->drones[i];
        const bool active = drone->state == SO_DRONE_SCOUTING || drone->state == SO_DRONE_WORKING ||
                            drone->state == SO_DRONE_ASSISTING || drone->state == SO_DRONE_CLEANUP;
        if (!active) {
            continue;
        }
        so_release_task(sim, drone);
        if (severity == SO_WEATHER_SEVERE || so_distance(drone->position, sim->mothership.position) <= 260.0) {
            drone->state = SO_DRONE_RETURNING;
            drone->target = sim->mothership.position;
            drone->has_target = true;
        } else {
            if (sim->field.emergency_landing_spot_count < sim->drone_count) {
                const int cols = 3;
                for (int s = 0; s < sim->drone_count && s < SO_MAX_LANDING_SPOTS; s++) {
                    sim->field.emergency_landing_spots[s] =
                        so_point(sim->mothership.position.x + (s % cols - 1) * 22.0,
                                 sim->mothership.position.y + (s / cols - 1) * 22.0);
                }
                sim->field.emergency_landing_spot_count = sim->drone_count;
            }
            drone->state = SO_DRONE_EMERGENCY_LANDING;
            drone->target = sim->field.emergency_landing_spots[i];
            drone->has_target = true;
        }
    }
}

static int so_drone_index_by_id(const SoSimulation *sim, int drone_id) {
    for (int i = 0; i < sim->drone_count; i++) {
        if (sim->drones[i].id == drone_id) {
            return i;
        }
    }
    return -1;
}

static bool so_drone_in_slot(const int *slots, int slot_count, int drone_id) {
    for (int s = 0; s < slot_count; s++) {
        if (slots[s] == drone_id) {
            return true;
        }
    }
    return false;
}

static int so_active_charger_count(const SoSimulation *sim) {
    if (sim->mothership.fast_chargers < 1) {
        return 1;
    }
    return sim->mothership.fast_chargers < SO_MAX_FAST_CHARGERS
               ? sim->mothership.fast_chargers
               : SO_MAX_FAST_CHARGERS;
}

static int so_active_charger_slot_count(const SoSimulation *sim) {
    return so_active_charger_count(sim) * SO_BATTERY_SLOTS_PER_CHARGER;
}

static int so_charger_slot_for_drone(const SoSimulation *sim, int drone_id) {
    const int slot_count = so_active_charger_slot_count(sim);
    for (int s = 0; s < slot_count; s++) {
        if (sim->queues.charger_slots[s] == drone_id) {
            return s;
        }
    }
    return -1;
}

static void so_reset_service_budgets(SoSimulation *sim) {
    sim->launch_landing_service_used_s = 0.0;
    sim->charger_handling_service_used_s = 0.0;
}

static double so_random_uav_service_s(SoSimulation *sim) {
    if (sim->service_rng_state == 0u) {
        sim->service_rng_state = 0x6D2B79F5u;
    }
    sim->service_rng_state =
        sim->service_rng_state * 1664525u + 1013904223u;
    const double unit =
        (double)(sim->service_rng_state & 0x00FFFFFFu) / 16777215.0;
    const double min_s = sim->uav_service_time_min_s > 0.0
                             ? sim->uav_service_time_min_s
                             : 5.0;
    const double max_s = sim->uav_service_time_max_s > min_s
                             ? sim->uav_service_time_max_s
                             : 8.0;
    return min_s + unit * (max_s - min_s);
}

static bool so_consume_service_budget(double *used_s, int slots, double dt_s, double seconds) {
    if (seconds <= 0.001) {
        return true;
    }
    const double capacity_s = fmax(1.0, (double)slots) * fmax(0.001, dt_s);
    if (*used_s + seconds > capacity_s + 1e-6) {
        return false;
    }
    *used_s += seconds;
    return true;
}

static bool so_consume_launch_landing_service(SoSimulation *sim, double seconds) {
    return so_consume_service_budget(&sim->launch_landing_service_used_s,
                                     sim->uav_launch_landing_slots,
                                     sim->dt_s,
                                     seconds);
}

static bool so_consume_charger_handling_service(SoSimulation *sim, double seconds) {
    return so_consume_service_budget(&sim->charger_handling_service_used_s,
                                     sim->charger_handling_slots,
                                     sim->dt_s,
                                     seconds);
}

static double so_charge_rate_h_for_slot(const SoSimulation *sim, int slot) {
    if (slot < 0) {
        return 0.0;
    }
    const int charger = slot / SO_BATTERY_SLOTS_PER_CHARGER;
    int occupied = 0;
    for (int s = charger * SO_BATTERY_SLOTS_PER_CHARGER;
         s < (charger + 1) * SO_BATTERY_SLOTS_PER_CHARGER &&
         s < so_active_charger_slot_count(sim);
         s++) {
        if (sim->queues.charger_slots[s] > 0) {
            occupied++;
        }
    }
    const double minutes_full = occupied >= 2 ? 12.5 : 7.5;
    return 60.0 / minutes_full;
}

static void so_update_service_queues(SoSimulation *sim) {
    const int charger_slot_count = so_active_charger_slot_count(sim);
    const int refill_count = sim->mothership.refill_ports < SO_MAX_REFILL_PORTS
                                 ? sim->mothership.refill_ports
                                 : SO_MAX_REFILL_PORTS;

    for (int s = 0; s < SO_MAX_CHARGER_SLOTS; s++) {
        if (s >= charger_slot_count) {
            sim->queues.charger_slots[s] = -1;
            continue;
        }
        const int idx = so_drone_index_by_id(sim, sim->queues.charger_slots[s]);
        if (idx < 0 || sim->drones[idx].state != SO_DRONE_CHARGING ||
            sim->drones[idx].service_remaining_s > 0.001) {
            sim->queues.charger_slots[s] = -1;
        }
    }

    for (int s = 0; s < SO_MAX_REFILL_PORTS; s++) {
        if (s >= refill_count) {
            sim->queues.refill_slots[s] = -1;
            continue;
        }
        const int idx = so_drone_index_by_id(sim, sim->queues.refill_slots[s]);
        if (idx < 0 || (sim->drones[idx].state != SO_DRONE_REFILLING && sim->drones[idx].state != SO_DRONE_CHARGING) ||
            sim->drones[idx].chemical >= 0.995) {
            sim->queues.refill_slots[s] = -1;
        }
    }

    for (int s = 0; s < charger_slot_count; s++) {
        if (sim->queues.charger_slots[s] != -1) {
            continue;
        }

        int best_id = -1;
        double lowest_battery = 2.0;
        for (int i = 0; i < sim->drone_count; i++) {
            SoDrone *drone = &sim->drones[i];
            if (drone->state != SO_DRONE_CHARGING ||
                drone->battery >= drone->target_charge ||
                so_drone_in_slot(sim->queues.charger_slots, SO_MAX_CHARGER_SLOTS, drone->id)) {
                continue;
            }
            if (drone->battery < lowest_battery) {
                lowest_battery = drone->battery;
                best_id = drone->id;
            }
        }
        if (best_id > 0) {
            const double insert_service_s = so_random_uav_service_s(sim);
            if (!so_consume_charger_handling_service(sim, insert_service_s)) {
                best_id = -1;
            }
        }
        sim->queues.charger_slots[s] = best_id;
    }

    for (int s = 0; s < refill_count; s++) {
        if (sim->queues.refill_slots[s] != -1) {
            continue;
        }

        int best_id = -1;
        double lowest_chemical = 2.0;
        for (int i = 0; i < sim->drone_count; i++) {
            SoDrone *drone = &sim->drones[i];
            const bool needs_refill = drone->chemical < 0.995 &&
                                      (drone->state == SO_DRONE_REFILLING || drone->state == SO_DRONE_CHARGING);
            if (!needs_refill ||
                so_drone_in_slot(sim->queues.refill_slots, SO_MAX_REFILL_PORTS, drone->id)) {
                continue;
            }
            if (drone->chemical < lowest_chemical) {
                lowest_chemical = drone->chemical;
                best_id = drone->id;
            }
        }
        sim->queues.refill_slots[s] = best_id;
    }
}

static void so_update_active_drones(SoSimulation *sim, SoWeatherAdjustedSpec weather) {
    const double dt_h = sim->dt_s / 3600.0;
    const double refill_rate_h = 8.0;

    so_update_service_queues(sim);

    for (int i = 0; i < sim->drone_count; i++) {
        SoDrone *drone = &sim->drones[i];

        if (drone->state == SO_DRONE_SCOUTING) {
            drone->assigned_area_ha = fmax(0.0, drone->assigned_area_ha - weather.scout_rate_ha_h * dt_h);
            const double energy_used = so_drone_scout_drain_h(drone, &sim->spec) *
                                       weather.battery_scout_multiplier * dt_h;
            so_add_uav_drone_electricity_cost(sim, drone, energy_used);
            drone->battery = fmax(0.0, drone->battery - energy_used);
            if (drone->assigned_area_ha <= 0.001) {
                drone->state = SO_DRONE_RETURNING;
                so_event(sim, "scout completed");
            }
        } else if (drone->state == SO_DRONE_WORKING || drone->state == SO_DRONE_ASSISTING ||
                   drone->state == SO_DRONE_CLEANUP) {
            if (drone->travel_remaining_s > 0.001) {
                const double turn_dt_s = fmin(drone->travel_remaining_s, sim->dt_s);
                SoFieldTask *task = so_find_task(sim, drone->assigned_task_id);
                if (task != NULL && weather.spray_allowed &&
                    drone->chemical > 0.001 && drone->assigned_area_ha > 0.001) {
                    const double route_efficiency = fmax(0.5, task->route_efficiency);
                    const double planned_shift_area =
                        so_uav_track_change_spray_area_ha(sim, task, drone->assigned_area_ha);
                    const double time_fraction =
                        task->turn_time_s > 0.001 ? turn_dt_s / task->turn_time_s : 0.0;
                    const double shift_done =
                        planned_shift_area * fmax(0.0, fmin(1.0, time_fraction)) *
                        weather.spray_effectiveness * route_efficiency;
                    const double rate_done =
                        weather.spray_rate_ha_h * weather.spray_effectiveness *
                        route_efficiency * (turn_dt_s / 3600.0) * 0.55;
                    const double done =
                        fmin(task->remaining_ha,
                             fmin(drone->assigned_area_ha, fmin(shift_done, rate_done)));
                    if (done > 0.0) {
                        task->remaining_ha = fmax(0.0, task->remaining_ha - done);
                        drone->assigned_area_ha = fmax(0.0, drone->assigned_area_ha - done);
                        drone->assigned_task_area_ha =
                            fmax(0.0, drone->assigned_task_area_ha - done);
                        sim->field.treated_ha = fmin(sim->field.area_ha, sim->field.treated_ha + done);
                        if (drone->sortie_battery_modules >= 1 && drone->sortie_battery_modules <= 4) {
                            sim->uav_area_by_battery_modules[drone->sortie_battery_modules] += done;
                        }
                        drone->chemical =
                            fmax(0.0, drone->chemical - so_drone_chemical_per_ha(drone, &sim->spec) * done);
                    }
                }
                drone->travel_remaining_s = fmax(0.0, drone->travel_remaining_s - turn_dt_s);
                const double energy_used = so_drone_work_drain_h(drone, &sim->spec) *
                                           1.12 * weather.battery_work_multiplier *
                                           (turn_dt_s / 3600.0);
                so_add_uav_drone_electricity_cost(sim, drone, energy_used);
                drone->battery = fmax(0.0, drone->battery - energy_used);
                if (task != NULL && task->remaining_ha <= 0.001) {
                    task->status = SO_TASK_DONE;
                    drone->travel_remaining_s = 0.0;
                    so_continue_bundle_or_return(sim, drone);
                }
                continue;
            }
            if (!weather.spray_allowed) {
                const double energy_used = so_drone_work_drain_h(drone, &sim->spec) *
                                           0.25 * weather.battery_work_multiplier * dt_h;
                so_add_uav_drone_electricity_cost(sim, drone, energy_used);
                drone->battery = fmax(0.0, drone->battery - energy_used);
                continue;
            }
            SoFieldTask *task = so_find_task(sim, drone->assigned_task_id);
            if (task == NULL) {
                drone->state = SO_DRONE_RETURNING;
                continue;
            }
            const double route_efficiency = fmax(0.5, task->route_efficiency);
            const double done = fmin(task->remaining_ha,
                                     weather.spray_rate_ha_h * weather.spray_effectiveness * route_efficiency * dt_h);
            task->remaining_ha = fmax(0.0, task->remaining_ha - done);
            drone->assigned_area_ha = fmax(0.0, drone->assigned_area_ha - done);
            drone->assigned_task_area_ha =
                fmax(0.0, drone->assigned_task_area_ha - done);
            sim->field.treated_ha = fmin(sim->field.area_ha, sim->field.treated_ha + done);
            if (drone->sortie_battery_modules >= 1 && drone->sortie_battery_modules <= 4) {
                sim->uav_area_by_battery_modules[drone->sortie_battery_modules] += done;
            }
            const double energy_used = so_drone_work_drain_h(drone, &sim->spec) *
                                       weather.battery_work_multiplier /
                                       fmin(1.18, route_efficiency) * dt_h;
            so_add_uav_drone_electricity_cost(sim, drone, energy_used);
            drone->battery = fmax(0.0, drone->battery - energy_used);
            drone->chemical =
                fmax(0.0, drone->chemical - so_drone_chemical_per_ha(drone, &sim->spec) * done);

            if (task->remaining_ha <= 0.001) {
                task->status = SO_TASK_DONE;
                so_continue_bundle_or_return(sim, drone);
            } else if (drone->assigned_area_ha <= 0.001) {
                so_release_task(sim, drone);
                drone->state = SO_DRONE_RETURNING;
            }
        } else if (drone->state == SO_DRONE_RETURNING) {
            if (drone->travel_remaining_s <= 0.001 && drone->service_remaining_s <= 0.001) {
                const SoPoint recovery_point = sim->mothership.moving
                                                   ? sim->mothership.destination
                                                   : sim->mothership.position;
                const double dist = so_distance(drone->position, recovery_point);
                drone->travel_remaining_s = dist / fmax(0.001, weather.cruise_speed_mps);
                drone->return_energy_required = dist / 1000.0 * so_drone_empty_drain_km(drone, &sim->spec);
                so_log_drone_transfer_segment(drone, drone->position, recovery_point);
                so_add_uav_flight_cost(sim, dist);
            }
            if (drone->travel_remaining_s <= sim->dt_s) {
                so_add_uav_drone_electricity_cost(sim, drone, drone->return_energy_required);
                drone->battery = fmax(0.0, drone->battery - drone->return_energy_required);
                drone->travel_remaining_s = 0.0;
                drone->return_energy_required = 0.0;
                if (sim->mothership.moving) {
                    drone->position = sim->mothership.destination;
                } else {
                    drone->position = sim->mothership.position;
                    drone->service_remaining_s = so_random_uav_service_s(sim);
                }
            } else {
                const double fraction = sim->dt_s / drone->travel_remaining_s;
                const double energy_used = drone->return_energy_required * fraction;
                so_add_uav_drone_electricity_cost(sim, drone, energy_used);
                drone->battery = fmax(0.0, drone->battery - energy_used);
                drone->return_energy_required = fmax(0.0, drone->return_energy_required * (1.0 - fraction));
                drone->travel_remaining_s = fmax(0.0, drone->travel_remaining_s - sim->dt_s);
            }
            if (!sim->mothership.moving && drone->travel_remaining_s <= 0.001 &&
                drone->service_remaining_s > 0.001) {
                if (so_consume_launch_landing_service(sim, drone->service_remaining_s)) {
                    drone->service_remaining_s = 0.0;
                    drone->state = SO_DRONE_CHARGING;
                }
            }
        } else if (drone->state == SO_DRONE_CHARGING) {
            const int charger_slot = so_charger_slot_for_drone(sim, drone->id);
            if (charger_slot >= 0) {
                const double fast_charge_rate_h =
                    so_charge_rate_h_for_slot(sim, charger_slot);
                drone->battery = fmin(1.0, drone->battery + fast_charge_rate_h * dt_h);
            }
            if (so_drone_in_slot(sim->queues.refill_slots, SO_MAX_REFILL_PORTS, drone->id)) {
                drone->chemical = fmin(1.0, drone->chemical + refill_rate_h * dt_h);
            }
            if (drone->battery >= drone->target_charge) {
                const bool in_charger = charger_slot >= 0;
                const double remove_service_s =
                    in_charger ? so_random_uav_service_s(sim) : 0.0;
                if (!in_charger ||
                    so_consume_charger_handling_service(sim, remove_service_s)) {
                    drone->state = drone->chemical < 0.98 ? SO_DRONE_REFILLING : SO_DRONE_STANDBY;
                }
            }
        } else if (drone->state == SO_DRONE_REFILLING) {
            if (so_drone_in_slot(sim->queues.refill_slots, SO_MAX_REFILL_PORTS, drone->id)) {
                drone->chemical = fmin(1.0, drone->chemical + refill_rate_h * dt_h);
            }
            if (drone->chemical >= 0.995) {
                drone->state = SO_DRONE_STANDBY;
            }
        } else if (drone->state == SO_DRONE_EMERGENCY_LANDING) {
            drone->state = SO_DRONE_LANDED;
        }

        if (so_needs_recall(sim, drone)) {
            so_release_task(sim, drone);
            drone->state = SO_DRONE_RETURNING;
        }
    }

    so_update_service_queues(sim);
}

static void so_relocate_if_needed(SoSimulation *sim) {
    if (sim->mothership.moving) {
        return;
    }
    if (so_mothership_service_busy(sim)) {
        return;
    }
    const double active_cover_radius = so_depot_scarce(sim) ? 2200.0 : (so_regular_corridor_layout(sim) ? 1450.0 : 900.0);
    const int current_tasks = so_count_interior_tasks_covered(sim, sim->mothership.position, active_cover_radius);
    const int cleanup_tasks = so_cleanup_open_near(sim, sim->mothership.position, active_cover_radius);
    const int active_drones = so_active_field_drone_count(sim);
    const bool cleanup_relocation_window = cleanup_tasks <= 2 && active_drones <= 2;

    if (sim->mothership.operation_plan_index + 1 >= sim->mothership.operation_plan_count) {
        if (cleanup_relocation_window && current_tasks == 0 && so_pending_major_task_count(sim) > 0 &&
            sim->mothership.operation_plan_count < SO_MAX_DEPOTS) {
            int best_site = -1;
            int best_cover = 0;
            double best_travel = 1e100;
            for (int s = 0; s < sim->field.depot_count; s++) {
                const SoDepotSite *site = &sim->field.depots[s];
                if (!so_site_deployable(&sim->field, site)) {
                    continue;
                }
                int cover = 0;
                for (int t = 0; t < sim->field.task_count; t++) {
                    const SoFieldTask *task = &sim->field.tasks[t];
                    if (task->status != SO_TASK_DONE && task->remaining_ha > 0.001 &&
                        so_distance(site->point, task->center) <= active_cover_radius) {
                        cover++;
                    }
                }
                const double travel = so_hive_route_distance(sim, sim->mothership.position, site->point);
                if (travel >= 1e11) {
                    continue;
                }
                if (so_distance(sim->mothership.position, site->point) <= 140.0) {
                    continue;
                }
                if (cover > best_cover || (cover == best_cover && cover > 0 && travel < best_travel)) {
                    best_cover = cover;
                    best_travel = travel;
                    best_site = s;
                }
            }
            if (best_site >= 0 && best_cover > 0) {
                const SoPoint next = sim->field.depots[best_site].point;
                sim->mothership.operation_plan[sim->mothership.operation_plan_count++] = next;
                sim->mothership.stop_cost_usd += sim->mothership.deployment_stop_cost_usd;
                sim->mothership.operation_plan_index++;
                sim->mothership.destination = next;
                const double move_minutes = so_hive_travel_minutes(sim, sim->mothership.position, next);
                if (move_minutes >= 1e8) {
                    return;
                }
                sim->mothership.move_remaining_s = move_minutes * 60.0;
                sim->mothership.moving = true;
                so_event(sim, "mothership added dynamic cleanup depot");
            }
        }
        return;
    }

    const SoPoint next = sim->mothership.operation_plan[sim->mothership.operation_plan_index + 1];
    const int next_tasks = so_count_interior_tasks_covered(sim, next, active_cover_radius);
    const bool current_stop_clean = current_tasks == 0;
    const bool cleanup_can_finish = cleanup_relocation_window && current_tasks <= 2 && next_tasks >= current_tasks + 2;
    if (cleanup_relocation_window && (current_stop_clean || cleanup_can_finish)) {
        const double move_minutes = so_hive_travel_minutes(sim, sim->mothership.position, next);
        if (move_minutes >= 1e8) {
            return;
        }
        sim->mothership.operation_plan_index++;
        sim->mothership.destination = next;
        sim->mothership.move_remaining_s = move_minutes * 60.0;
        sim->mothership.moving = true;
        so_event(sim, "mothership relocating during cleanup window");
    }
}

static void so_update_mothership(SoSimulation *sim) {
    if (!sim->mothership.moving) {
        return;
    }
    const double move_dt_s = fmin(sim->dt_s, sim->mothership.move_remaining_s);
    const double move_m = fmax(0.0, move_dt_s * sim->mothership.move_speed_mps);
    sim->mothership.move_distance_m += move_m;
    sim->mothership.move_cost_usd += so_hive_move_cost_usd(sim, move_m);
    sim->mothership.move_remaining_s = fmax(0.0, sim->mothership.move_remaining_s - sim->dt_s);
    if (sim->mothership.move_remaining_s <= 0.001) {
        sim->mothership.position = sim->mothership.destination;
        sim->mothership.moving = false;
        so_event(sim, "mothership arrived");
    }
}

static bool so_mothership_service_busy(const SoSimulation *sim) {
    for (int i = 0; i < sim->drone_count; i++) {
        const SoDrone *drone = &sim->drones[i];
        if (drone->state == SO_DRONE_RETURNING ||
            drone->state == SO_DRONE_CHARGING ||
            drone->state == SO_DRONE_REFILLING) {
            return true;
        }
    }
    return false;
}

static double so_t200_tank_l_for_modules(const SoDroneSpec *spec, int modules) {
    const int extra_modules = modules > 1 ? modules - 1 : 0;
    const double available_kg =
        spec->modeled_payload_capacity_kg -
        spec->spray_system_weight_kg -
        (double)extra_modules * spec->battery_module_weight_kg;
    return fmax(40.0, fmin(spec->chemical_tank_max_l,
                           available_kg / fmax(0.001, spec->chemical_density_kg_per_l)));
}

static void so_apply_t200_battery_configuration(SoSimulation *sim, int modules) {
    if (modules < sim->spec.min_battery_modules) {
        modules = sim->spec.min_battery_modules;
    }
    if (modules > sim->spec.max_battery_modules) {
        modules = sim->spec.max_battery_modules;
    }

    sim->spec.battery_modules = modules;
    sim->spec.battery_capacity_kwh =
        (double)modules * sim->spec.battery_module_capacity_kwh;
    sim->spec.chemical_tank_l = so_t200_tank_l_for_modules(&sim->spec, modules);
    sim->spec.selected_payload_kg =
        sim->spec.chemical_tank_l * sim->spec.chemical_density_kg_per_l +
        sim->spec.spray_system_weight_kg +
        (double)(modules > 1 ? modules - 1 : 0) * sim->spec.battery_module_weight_kg;

    sim->spec.work_power_kw =
        48.0 + (double)modules * 1.8 + sim->spec.chemical_tank_l * 0.045;
    sim->spec.scout_power_kw =
        20.0 + (double)modules * 1.1;
    sim->spec.battery_drain_h_work =
        sim->spec.work_power_kw / fmax(0.001, sim->spec.battery_capacity_kwh);
    sim->spec.battery_drain_h_scout =
        sim->spec.scout_power_kw / fmax(0.001, sim->spec.battery_capacity_kwh);
    sim->spec.battery_drain_km_empty =
        (18.0 + (double)modules * 0.9) /
        fmax(0.001, sim->spec.battery_capacity_kwh) /
        fmax(0.001, sim->spec.cruise_speed_mps * 3.6);

    sim->spec.chemical_per_ha =
        sim->spec.chemical_l_per_ha / fmax(1.0, sim->spec.chemical_tank_l);
    sim->spec.chemical_tank_area_ha =
        sim->spec.chemical_tank_l / fmax(0.001, sim->spec.chemical_l_per_ha);
}

static void so_select_t200_battery_configuration(SoSimulation *sim) {
    int best_modules = sim->spec.min_battery_modules;
    double best_score = -1.0;
    const int charger_slot_count = so_active_charger_slot_count(sim);
    const double charge_pressure =
        (double)sim->drone_count / fmax(1.0, (double)charger_slot_count);

    for (int modules = sim->spec.min_battery_modules;
         modules <= sim->spec.max_battery_modules;
         modules++) {
        SoSimulation tmp = *sim;
        so_apply_t200_battery_configuration(&tmp, modules);

        const double available_battery =
            fmax(0.0, 0.92 - tmp.spec.safety_battery_margin);
        const double battery_area =
            available_battery /
            fmax(0.001, tmp.spec.battery_drain_h_work) *
            tmp.spec.spray_rate_ha_h;
        const double chemical_area =
            tmp.spec.chemical_tank_l /
            fmax(0.001, tmp.spec.chemical_l_per_ha);
        const double cycle_area = fmin(battery_area, chemical_area);
        const double work_h = cycle_area / fmax(0.001, tmp.spec.spray_rate_ha_h);
        const double charge_h = 0.78 * (12.5 / 60.0);
        const double refill_h = 0.85 / 8.0;
        const double payload_penalty =
            tmp.spec.selected_payload_kg > tmp.spec.modeled_payload_capacity_kg
                ? 1000.0
                : 0.0;
        const double cycle_h =
            work_h + charge_h * charge_pressure * 0.35 + refill_h * 0.20;
        const double score =
            cycle_area / fmax(0.001, cycle_h) - payload_penalty;
        if (score > best_score) {
            best_score = score;
            best_modules = modules;
        }
    }

    so_apply_t200_battery_configuration(sim, best_modules);
}

void so_init_default(SoSimulation *sim) {
    memset(sim, 0, sizeof(*sim));
    sim->drone_count = SO_MAX_DRONES;
    sim->dt_s = 60.0;
    sim->optimization_profile = SO_OPT_PROFILE_BALANCED;
    sim->next_weather_update_s = 0.0;
    sim->coverage_task_tolerance_ratio = 0.02;
    sim->coverage_final_error_limit_ratio = 0.03;
    sim->coverage_repair_cost_limit_ratio = 0.05;
    sim->uav_launch_landing_slots = 2;
    sim->charger_handling_slots = 2;
    sim->uav_service_time_min_s = 5.0;
    sim->uav_service_time_max_s = 8.0;
    sim->service_rng_state = 0xA341316Cu;
    sim->planner_weights = so_default_planner_weights();
    sim->planner_trial_count = SO_PLANNER_MAX_TRIALS;
    sim->planner_trial_index = 0;
    sim->selected_planner_trial = 0;
    sim->selected_planner_cost_usd = 0.0;
    sim->planner_seed = 0x51C0A7E5u;
    sim->spec.cruise_speed_mps = 12.0;
    sim->spec.scout_speed_mps = 6.0;
    sim->spec.scout_rate_ha_h = 25.0;
    sim->spec.spray_swath_m = 10.0;
    sim->spec.spray_radius_m = sim->spec.spray_swath_m * 0.5;
    sim->spec.spray_rate_ha_h = 30.0;
    sim->spec.turn_time_s = 8.0;
    sim->spec.turn_battery_cost = 0.004;
    sim->spec.flight_cost_usd_per_km = 0.45;
    sim->spec.launch_cost_usd = 2.70;
    sim->spec.effective_chemical_l_per_ha = so_random_effective_chemical_l_per_ha();
    sim->spec.deposition_efficiency = 0.85;
    sim->spec.chemical_l_per_ha =
        sim->spec.effective_chemical_l_per_ha /
        fmax(0.001, sim->spec.deposition_efficiency);
    sim->spec.chemical_cost_usd_per_l = 1.15;
    sim->spec.min_battery_modules = 1;
    sim->spec.max_battery_modules = 4;
    sim->spec.battery_module_capacity_kwh = 2.402;
    sim->spec.battery_module_weight_kg = 16.0;
    sim->spec.modeled_payload_capacity_kg = 200.0;
    sim->spec.spray_system_weight_kg = 15.0;
    sim->spec.chemical_density_kg_per_l = 1.0;
    sim->spec.chemical_tank_max_l = 200.0;
    sim->spec.fast_charger_power_kw = 45.0;
    sim->spec.electricity_price_usd_per_kwh = 0.12;
    sim->spec.turn_radius_m = 10.0;
    sim->spec.unfinished_penalty_usd_per_ha = 280.0;
    sim->spec.safety_battery_margin = 0.15;

    sim->field.area_ha = 72.0;
    sim->field.terrain_complexity = 0.45;
    sim->field.obstacle_density = 0.25;
    sim->field.boundary_center = so_point(0.0, 0.0);
    sim->field.block_count = 1;
    sim->field.blocks[0].id = 1;
    sim->field.blocks[0].name = "main";
    sim->field.blocks[0].center = so_point(0.0, 0.0);
    sim->field.blocks[0].area_ha = 72.0;
    sim->field.blocks[0].risk = 0.35;
    sim->field.blocks[0].selected = true;
    sim->field.depot_count = 3;
    sim->field.depots[0] = (SoDepotSite){1, so_point(-320.0, 0.0), 400.0, true, 0.1};
    sim->field.depots[1] = (SoDepotSite){2, so_point(0.0, -280.0), 400.0, true, 0.1};
    sim->field.depots[2] = (SoDepotSite){3, so_point(250.0, 80.0), 400.0, true, 0.15};

    sim->mothership.drone_slots = 8;
    sim->mothership.fast_chargers = 4;
    sim->mothership.refill_ports = 2;
    sim->mothership.position = so_point(-600.0, 0.0);
    sim->mothership.move_speed_mps = SO_HIVE_MOVE_SPEED_MPS;
    sim->mothership.truck_cost_usd_per_km = 2.60;
    sim->mothership.deployment_stop_cost_usd = 12.00;
    sim->mothership.weather = (SoWeather){2.5, 4.0, 26.0, 0.55, 0.0, 5000.0, 70.0, 0.0};
    so_select_t200_battery_configuration(sim);

    for (int i = 0; i < SO_MAX_CHARGER_SLOTS; i++) {
        sim->queues.charger_slots[i] = -1;
    }
    for (int i = 0; i < SO_MAX_REFILL_PORTS; i++) {
        sim->queues.refill_slots[i] = -1;
    }

    for (int i = 0; i < sim->drone_count; i++) {
        sim->drones[i].id = i + 1;
        sim->drones[i].state = SO_DRONE_IDLE;
        sim->drones[i].battery = 1.0;
        sim->drones[i].chemical = 1.0;
        sim->drones[i].position = sim->mothership.position;
        sim->drones[i].service_remaining_s = 0.0;
        sim->drones[i].assigned_task_area_ha = 0.0;
        sim->drones[i].target_charge = 0.8;
        sim->drones[i].assigned_task_id = -1;
        so_set_drone_sortie_spec(&sim->drones[i], &sim->spec);
        sim->drones[i].route_point_count = 0;
        sim->drones[i].route_segment_count = 0;
    }
}

void so_init_two_block_demo(SoSimulation *sim) {
    so_init_default(sim);
    sim->field.area_ha = 66.0;
    sim->field.block_count = 2;
    sim->field.blocks[0] = (SoFieldBlock){1, "west field", so_point(-350.0, 0.0), 32.0, 0.28, true, 0, {{0.0, 0.0}}};
    sim->field.blocks[1] = (SoFieldBlock){2, "east field", so_point(350.0, 0.0), 34.0, 0.35, true, 0, {{0.0, 0.0}}};
    sim->field.depot_count = 4;
    sim->field.depots[0] = (SoDepotSite){1, so_point(-430.0, -280.0), 520.0, true, 0.15};
    sim->field.depots[1] = (SoDepotSite){2, so_point(-250.0, 280.0), 90.0, true, 0.2};
    sim->field.depots[2] = (SoDepotSite){3, so_point(260.0, -280.0), 480.0, true, 0.18};
    sim->field.depots[3] = (SoDepotSite){4, so_point(460.0, 280.0), 420.0, false, 0.1};
}

void so_init_multi_block_demo(SoSimulation *sim, int block_count) {
    so_init_default(sim);
    if (block_count < 1) {
        block_count = 1;
    }
    if (block_count > SO_MAX_BLOCKS) {
        block_count = SO_MAX_BLOCKS;
    }

    sim->field.block_count = block_count;
    sim->field.area_ha = 0.0;
    sim->field.depot_count = 0;

    const int cols = (int)ceil(sqrt((double)block_count));
    const double spacing_x = 720.0;
    const double spacing_y = 560.0;
    double sum_x = 0.0;
    double sum_y = 0.0;

    for (int i = 0; i < block_count; i++) {
        const int row = i / cols;
        const int col = i % cols;
        const double x = (col - (cols - 1) / 2.0) * spacing_x;
        const double y = (row - ((block_count - 1) / cols) / 2.0) * spacing_y;
        const double area = 18.0 + (double)((i * 7) % 9);
        const double risk = 0.22 + 0.05 * (double)(i % 5);

        sim->field.blocks[i] = (SoFieldBlock){i + 1, "manual field", so_point(x, y), area, risk, true, 0, {{0.0, 0.0}}};
        sim->field.area_ha += area;
        sum_x += x;
        sum_y += y;

        if (sim->field.depot_count + 1 < SO_MAX_DEPOTS) {
            sim->field.depots[sim->field.depot_count++] =
                (SoDepotSite){sim->field.depot_count + 1, so_point(x - 210.0, y - 260.0), 460.0, true, 0.12 + risk * 0.15};
            sim->field.depots[sim->field.depot_count++] =
                (SoDepotSite){sim->field.depot_count + 1, so_point(x + 230.0, y + 245.0), 410.0, i % 3 != 1, 0.18};
        }
    }

    sim->field.boundary_center = so_point(sum_x / block_count, sum_y / block_count);
    sim->field.terrain_complexity = fmin(1.0, 0.28 + block_count * 0.035);
    sim->field.obstacle_density = fmin(1.0, 0.18 + block_count * 0.025);
    sim->mothership.position = so_point(sim->field.boundary_center.x - 760.0, sim->field.boundary_center.y - 120.0);
    for (int i = 0; i < sim->drone_count; i++) {
        sim->drones[i].position = sim->mothership.position;
    }
    so_reset_drone_route_logs(sim);
}

void so_init_ideal_layout_demo(SoSimulation *sim, int block_count) {
    so_init_default(sim);
    if (block_count < 1) {
        block_count = 1;
    }
    if (block_count > SO_MAX_BLOCKS) {
        block_count = SO_MAX_BLOCKS;
    }

    sim->field.block_count = block_count;
    sim->field.area_ha = 0.0;
    sim->field.depot_count = 0;
    sim->field.terrain_complexity = 0.18;
    sim->field.obstacle_density = 0.08;

    const int cols = (int)ceil(sqrt((double)block_count));
    const double spacing_x = 520.0;
    const double spacing_y = 430.0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (int i = 0; i < block_count; i++) {
        const int row = i / cols;
        const int col = i % cols;
        const double x = (col - (cols - 1) / 2.0) * spacing_x;
        const double y = (row - ((block_count - 1) / cols) / 2.0) * spacing_y;
        const double area = 22.0;
        const double risk = 0.16 + (i % 4) * 0.025;
        sim->field.blocks[i] = (SoFieldBlock){i + 1, "ideal strip field", so_point(x, y), area, risk, true, 0, {{0.0, 0.0}}};
        sim->field.area_ha += area;
        sum_x += x;
        sum_y += y;
    }

    for (int col = 0; col < cols && sim->field.depot_count < SO_MAX_DEPOTS; col++) {
        const double x = (col - (cols - 1) / 2.0) * spacing_x;
        sim->field.depots[sim->field.depot_count++] =
            (SoDepotSite){sim->field.depot_count + 1, so_point(x, -520.0), 620.0, true, 0.07};
        if (sim->field.depot_count < SO_MAX_DEPOTS) {
            sim->field.depots[sim->field.depot_count++] =
                (SoDepotSite){sim->field.depot_count + 1, so_point(x, 520.0), 580.0, true, 0.09};
        }
    }

    sim->field.boundary_center = so_point(sum_x / block_count, sum_y / block_count);
    sim->mothership.position = so_point(sim->field.boundary_center.x - 680.0, -620.0);
    sim->mothership.weather.wind_direction_deg = 82.0;
    for (int i = 0; i < sim->drone_count; i++) {
        sim->drones[i].position = sim->mothership.position;
    }
    so_reset_drone_route_logs(sim);
}

void so_init_irregular_layout_demo(SoSimulation *sim, int block_count) {
    so_init_default(sim);
    if (block_count < 1) {
        block_count = 1;
    }
    if (block_count > SO_MAX_BLOCKS) {
        block_count = SO_MAX_BLOCKS;
    }

    sim->field.block_count = block_count;
    sim->field.area_ha = 0.0;
    sim->field.depot_count = 0;
    sim->field.terrain_complexity = 0.68;
    sim->field.obstacle_density = 0.46;

    double sum_x = 0.0;
    double sum_y = 0.0;
    for (int i = 0; i < block_count; i++) {
        const double x = sin(i * 1.37) * 930.0 + (i % 4 - 1.5) * 280.0;
        const double y = cos(i * 0.91) * 720.0 + (i / 4) * 190.0;
        const double area = 10.0 + (double)((i * 11) % 24);
        const double risk = 0.30 + 0.055 * (double)((i * 5) % 8);
        sim->field.blocks[i] = (SoFieldBlock){i + 1, "irregular field", so_point(x, y), area, fmin(0.82, risk), true, 0, {{0.0, 0.0}}};
        sim->field.area_ha += area;
        sum_x += x;
        sum_y += y;

        if (i % 2 == 0 && sim->field.depot_count < SO_MAX_DEPOTS) {
            sim->field.depots[sim->field.depot_count++] =
                (SoDepotSite){sim->field.depot_count + 1, so_point(x - 320.0, y - 330.0), 420.0, true, 0.16 + risk * 0.08};
        }
        if (i % 5 == 0 && sim->field.depot_count < SO_MAX_DEPOTS) {
            sim->field.depots[sim->field.depot_count++] =
                (SoDepotSite){sim->field.depot_count + 1, so_point(x + 260.0, y + 310.0), 130.0, true, 0.20};
        }
        if (i % 3 == 1 && sim->field.depot_count < SO_MAX_DEPOTS) {
            sim->field.depots[sim->field.depot_count++] =
                (SoDepotSite){sim->field.depot_count + 1, so_point(x + 420.0, y - 250.0), 360.0, false, 0.12};
        }
    }

    if (sim->field.depot_count == 0) {
        sim->field.depots[sim->field.depot_count++] =
            (SoDepotSite){1, so_point(-450.0, -420.0), 440.0, true, 0.18};
    }

    sim->field.boundary_center = so_point(sum_x / block_count, sum_y / block_count);
    sim->mothership.position = so_point(sim->field.boundary_center.x - 820.0, sim->field.boundary_center.y - 360.0);
    sim->mothership.weather.wind_direction_deg = 35.0;
    for (int i = 0; i < sim->drone_count; i++) {
        sim->drones[i].position = sim->mothership.position;
    }
    so_reset_drone_route_logs(sim);
}

void so_init_hybrid_layout_demo(SoSimulation *sim, int block_count) {
    so_init_multi_block_demo(sim, block_count);
    so_enable_fixed_wing(sim);
}

void so_enable_fixed_wing(SoSimulation *sim) {
    sim->fixed_wing.enabled = true;
    sim->fixed_wing.planned = false;
    sim->fixed_wing.aircraft_count = 0;
    sim->fixed_wing.assigned_area_ha = 0.0;
    sim->fixed_wing.completed_area_ha = 0.0;
    sim->fixed_wing.planned_turn_non_spray_time_s = 0.0;
    sim->fixed_wing.turn_non_spray_time_s = 0.0;
    snprintf(sim->fixed_wing.path_strategy,
             sizeof(sim->fixed_wing.path_strategy),
             "%s",
             "unplanned");
    sim->fixed_wing.path_strategy_score = 0.0;
    for (int i = 0; i < 3; i++) {
        sim->fixed_wing.path_strategy_time_h[i] = 0.0;
        sim->fixed_wing.path_strategy_cost_usd[i] = 0.0;
        sim->fixed_wing.path_strategy_scoreboard[i] = 0.0;
    }
    sim->fixed_wing.airport = so_point(sim->mothership.position.x - 5000.0, sim->mothership.position.y);
    sim->fixed_wing.tank_l = 1893.0;
    sim->fixed_wing.fuel_l = 644.0;
    sim->fixed_wing.payload_kg = 2450.0;
    sim->fixed_wing.flight_cost_usd_per_km = 6.50;
    sim->fixed_wing.takeoff_cost_usd = 120.0;
    sim->fixed_wing.airport_service_cost_usd = 180.0;
    sim->fixed_wing.effective_chemical_l_per_ha = sim->spec.effective_chemical_l_per_ha;
    sim->fixed_wing.deposition_efficiency = 0.72;
    sim->fixed_wing.chemical_l_per_ha =
        sim->fixed_wing.effective_chemical_l_per_ha /
        fmax(0.001, sim->fixed_wing.deposition_efficiency);
    sim->fixed_wing.chemical_cost_usd_per_l = 1.15;
    sim->fixed_wing.fuel_burn_l_per_h = 205.0;
    sim->fixed_wing.fuel_price_usd_per_l = 1.03;
    sim->fixed_wing.fuel_cost_usd_per_h =
        sim->fixed_wing.fuel_burn_l_per_h *
        sim->fixed_wing.fuel_price_usd_per_l;
    sim->fixed_wing.turn_radius_m = 300.0;
    sim->fixed_wing.tank_area_ha =
        sim->fixed_wing.tank_l / fmax(0.001, sim->fixed_wing.chemical_l_per_ha);
    sim->fixed_wing.unfinished_penalty_usd_per_ha = 520.0;
}

static double so_block_perimeter_m(const SoFieldBlock *block) {
    if (block == NULL) {
        return 0.0;
    }
    if (block->boundary_count >= 3) {
        double perimeter = 0.0;
        for (int i = 0; i < block->boundary_count; i++) {
            perimeter += so_distance(block->boundary[i], block->boundary[(i + 1) % block->boundary_count]);
        }
        return perimeter;
    }
    return sqrt(fmax(1.0, block->area_ha) * 10000.0) * 4.0;
}

static bool so_operational_work_complete(const SoSimulation *sim) {
    const double tolerance_ratio =
        sim->final_repair_attempted
            ? fmax(sim->coverage_task_tolerance_ratio,
                   sim->coverage_final_error_limit_ratio)
            : sim->coverage_task_tolerance_ratio;
    const double coverage_tolerance_ha =
        fmax(0.05, sim->field.area_ha *
                       fmax(0.0, tolerance_ratio));
    if (!sim->field.scanned || sim->field.task_count <= 0 ||
        sim->field.area_ha - sim->field.treated_ha > coverage_tolerance_ha) {
        return false;
    }
    for (int b = 0; b < sim->field.block_count; b++) {
        const SoFieldBlock *block = &sim->field.blocks[b];
        double block_remaining_ha = 0.0;
        for (int t = 0; t < sim->field.task_count; t++) {
            const SoFieldTask *task = &sim->field.tasks[t];
            if (task->block_id == block->id && task->remaining_ha > 0.001) {
                block_remaining_ha += task->remaining_ha;
            }
        }
        const double block_limit_ha =
            fmax(0.05, block->area_ha *
                           fmax(0.0, sim->coverage_final_error_limit_ratio));
        if (block_remaining_ha > block_limit_ha) {
            return false;
        }
    }
    return !sim->fixed_wing.enabled ||
           sim->fixed_wing.completed_area_ha + 0.001 >= sim->fixed_wing.assigned_area_ha;
}

static bool so_ready_for_final_coverage_policy(const SoSimulation *sim) {
    if (!sim->field.scanned || sim->field.task_count <= 0 ||
        sim->final_repair_attempted || sim->mothership.moving) {
        return false;
    }
    if (sim->fixed_wing.enabled &&
        sim->fixed_wing.completed_area_ha + 0.001 < sim->fixed_wing.assigned_area_ha) {
        return false;
    }
    for (int i = 0; i < sim->drone_count; i++) {
        const SoDrone *drone = &sim->drones[i];
        if (drone->state == SO_DRONE_WORKING ||
            drone->state == SO_DRONE_ASSISTING ||
            drone->state == SO_DRONE_CLEANUP ||
            drone->state == SO_DRONE_SCOUTING) {
            return false;
        }
    }

    const double final_limit_ha =
        fmax(0.05, sim->field.area_ha *
                       fmax(0.0, sim->coverage_final_error_limit_ratio));
    if (sim->field.area_ha - sim->field.treated_ha > final_limit_ha) {
        return false;
    }
    for (int b = 0; b < sim->field.block_count; b++) {
        const SoFieldBlock *block = &sim->field.blocks[b];
        double block_remaining_ha = 0.0;
        for (int t = 0; t < sim->field.task_count; t++) {
            const SoFieldTask *task = &sim->field.tasks[t];
            if (task->block_id == block->id && task->remaining_ha > 0.001) {
                block_remaining_ha += task->remaining_ha;
            }
        }
        const double block_limit_ha =
            fmax(0.05, block->area_ha *
                           fmax(0.0, sim->coverage_final_error_limit_ratio));
        if (block_remaining_ha > block_limit_ha) {
            return false;
        }
    }
    return true;
}

static void so_finalize_coverage_error_policy(SoSimulation *sim) {
    const double area = fmax(0.0, sim->field.area_ha);
    const double uncovered =
        fmax(0.0, area - fmax(0.0, sim->field.treated_ha));
    sim->final_uncovered_ha = uncovered;
    sim->final_uncovered_ratio = area > 0.001 ? uncovered / area : 0.0;
    sim->final_repair_attempted = false;
    sim->final_repair_performed = false;
    sim->final_repair_area_ha = 0.0;
    sim->final_repair_cost_usd = 0.0;
    sim->final_repair_cost_ratio = 0.0;

    if (uncovered <= 0.001 ||
        sim->final_uncovered_ratio >
            fmax(0.0, sim->coverage_final_error_limit_ratio)) {
        return;
    }

    sim->final_repair_attempted = true;
    const double base_cost = fmax(1.0, so_direct_mission_cost_usd(sim));
    const double repair_cost = so_estimate_uav_final_repair_cost_usd(sim, uncovered);
    const double repair_ratio = repair_cost / base_cost;
    sim->final_repair_cost_usd = repair_cost;
    sim->final_repair_cost_ratio = repair_ratio;

    if (repair_ratio <= fmax(0.0, sim->coverage_repair_cost_limit_ratio)) {
        sim->final_repair_performed = true;
        sim->final_repair_area_ha = uncovered;
        sim->field.treated_ha = fmin(area, sim->field.treated_ha + uncovered);
        sim->final_uncovered_ha = fmax(0.0, area - sim->field.treated_ha);
        sim->final_uncovered_ratio =
            area > 0.001 ? sim->final_uncovered_ha / area : 0.0;
        sim->uav_flight_distance_m +=
            uncovered * 10000.0 / fmax(0.001, sim->spec.spray_swath_m);
        sim->uav_flight_cost_usd +=
            (uncovered * 10000.0 / fmax(0.001, sim->spec.spray_swath_m)) /
            1000.0 * sim->spec.flight_cost_usd_per_km;
        sim->uav_launch_cost_usd += sim->spec.launch_cost_usd;
        sim->uav_takeoffs++;
        const double work_h =
            uncovered / fmax(0.001, sim->spec.spray_rate_ha_h);
        const double spray_m =
            uncovered * 10000.0 / fmax(0.001, sim->spec.spray_swath_m);
        const double energy_units =
            work_h * sim->spec.battery_drain_h_work +
            spray_m / 1000.0 * sim->spec.battery_drain_km_empty * 0.35;
        sim->uav_energy_used_battery_units += energy_units;
        sim->uav_energy_used_kwh +=
            energy_units * sim->spec.battery_capacity_kwh;
        sim->uav_electricity_cost_usd +=
            energy_units * sim->spec.battery_capacity_kwh *
            sim->spec.electricity_price_usd_per_kwh;
        const int modules = sim->spec.battery_modules >= 0 &&
                                    sim->spec.battery_modules <= 4
                                ? sim->spec.battery_modules
                                : 4;
        sim->uav_sorties_by_battery_modules[modules]++;
        sim->uav_area_by_battery_modules[modules] += uncovered;
        so_event(sim, "final tolerated gap repaired by UAV cleanup");
    } else {
        so_event(sim, "final tolerated gap accepted without repair");
    }
}

static bool so_drone_recovered_or_safe(const SoSimulation *sim, const SoDrone *drone) {
    if (drone->state == SO_DRONE_LANDED) {
        return true;
    }
    const bool safe_service_state =
        drone->state == SO_DRONE_IDLE ||
        drone->state == SO_DRONE_STANDBY ||
        drone->state == SO_DRONE_CHARGING ||
        drone->state == SO_DRONE_REFILLING;
    return safe_service_state &&
           so_distance(drone->position, sim->mothership.position) <= 1.0;
}

static void so_recall_fleet_for_mission_end(SoSimulation *sim) {
    const SoPoint recovery_point = sim->mothership.moving
                                       ? sim->mothership.destination
                                       : sim->mothership.position;
    for (int i = 0; i < sim->drone_count; i++) {
        SoDrone *drone = &sim->drones[i];
        if (drone->state == SO_DRONE_LANDED ||
            (so_drone_recovered_or_safe(sim, drone) && !sim->mothership.moving)) {
            continue;
        }
        if (drone->state != SO_DRONE_RETURNING) {
            so_release_task(sim, drone);
            drone->state = SO_DRONE_RETURNING;
            drone->travel_remaining_s = 0.0;
            drone->return_energy_required = 0.0;
            drone->target = recovery_point;
            drone->has_target = true;
            so_event(sim, "mission work complete; drone recalled");
        }
    }
}

void so_step(SoSimulation *sim) {
    so_reset_service_budgets(sim);
    so_update_weather(sim);
    const SoWeatherAdjustedSpec weather = so_adjust_for_weather(sim->spec, sim->mothership.weather);
    so_weather_recovery(sim);
    so_update_mothership(sim);

    if (!sim->field.scanned) {
        so_build_tasks(sim);
        so_plan_fixed_wing_coverage(sim);
        so_rebuild_uav_residual_tasks_after_fixed_wing(sim);
        so_plan_depots(sim);
        if (sim->mothership.operation_plan_count > 0) {
            sim->mothership.destination = sim->mothership.operation_plan[0];
            const double move_minutes =
                so_hive_travel_minutes(sim, sim->mothership.position, sim->mothership.destination);
            sim->mothership.move_remaining_s = move_minutes >= 1e8 ? 0.0 : move_minutes * 60.0;
            sim->mothership.moving = sim->mothership.move_remaining_s > 1.0;
            if (!sim->mothership.moving) {
                sim->mothership.position = sim->mothership.destination;
            } else {
                so_event(sim, "mothership moving to first planned depot");
            }
        }
        sim->field.scanned = true;
        for (int i = 0; i < 2 && i < sim->drone_count; i++) {
            const double takeoff_service_s = so_random_uav_service_s(sim);
            if (!so_consume_launch_landing_service(sim, takeoff_service_s)) {
                break;
            }
            sim->drones[i].state = SO_DRONE_SCOUTING;
            sim->drones[i].assigned_area_ha = sim->field.area_ha / 2.0;
            so_add_uav_takeoff_cost(sim);
            const SoFieldBlock *block = i < sim->field.block_count ? &sim->field.blocks[i] : NULL;
            const SoPoint target = block != NULL ? block->center : sim->field.boundary_center;
            so_add_uav_flight_cost(sim,
                                   so_distance(sim->drones[i].position, target) +
                                   so_block_perimeter_m(block) +
                                   so_distance(target, sim->mothership.position));
        }
        so_event(sim, "scout assigned");
    }

    so_update_fixed_wing(sim, weather);
    so_update_active_drones(sim, weather);
    if (so_ready_for_final_coverage_policy(sim)) {
        so_finalize_coverage_error_policy(sim);
    }
    if (so_operational_work_complete(sim)) {
        so_recall_fleet_for_mission_end(sim);
    } else {
        so_relocate_if_needed(sim);
        if (sim->mothership.moving) {
            so_assign_relocation_cleanup(sim, weather);
        } else {
            so_assign_work(sim);
            so_assign_assist(sim);
        }
    }
    sim->now_s += sim->dt_s;
}

bool so_completed(const SoSimulation *sim) {
    if (!so_operational_work_complete(sim) || sim->mothership.moving) {
        return false;
    }
    const double final_limit_ha =
        fmax(0.05, sim->field.area_ha *
                       fmax(0.0, sim->coverage_final_error_limit_ratio));
    if (sim->field.area_ha - sim->field.treated_ha > final_limit_ha) {
        return false;
    }
    for (int i = 0; i < sim->drone_count; i++) {
        if (!so_drone_recovered_or_safe(sim, &sim->drones[i])) {
            return false;
        }
    }
    return true;
}

static void so_run_single(SoSimulation *sim, int max_steps) {
    for (int i = 0; i < max_steps && !so_completed(sim); i++) {
        so_step(sim);
    }
    if (!sim->final_repair_attempted && so_ready_for_final_coverage_policy(sim)) {
        so_finalize_coverage_error_policy(sim);
    } else if (!sim->final_repair_attempted && so_completed(sim)) {
        so_finalize_coverage_error_policy(sim);
    }
}

static double so_planner_candidate_selection_cost(const SoSimulation *sim) {
    double score = so_direct_mission_cost_usd(sim);
    const double area = fmax(0.0, sim->field.area_ha);
    const double uncovered_ha = fmax(0.0, area - fmax(0.0, sim->field.treated_ha));
    double open_task_ha = 0.0;
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *task = &sim->field.tasks[i];
        if (task->status != SO_TASK_DONE && task->remaining_ha > 0.001) {
            open_task_ha += task->remaining_ha;
        }
    }
    const double unfinished_ha = fmax(uncovered_ha, open_task_ha);
    const double penalty_per_ha =
        fmax(5000.0,
             fmax(sim->spec.unfinished_penalty_usd_per_ha,
                  sim->fixed_wing.unfinished_penalty_usd_per_ha) *
                 20.0);
    score += unfinished_ha * penalty_per_ha;
    if (!so_completed(sim)) {
        score += 10000000.0;
    }
    return score;
}

void so_run(SoSimulation *sim, int max_steps) {
    int trial_count = sim->planner_trial_count;
    if (trial_count < 1) {
        trial_count = 1;
    }
    if (trial_count > SO_PLANNER_MAX_TRIALS) {
        trial_count = SO_PLANNER_MAX_TRIALS;
    }
    if (trial_count <= 1 || sim->field.scanned || sim->now_s > 0.001) {
        so_run_single(sim, max_steps);
        sim->selected_planner_trial = sim->planner_trial_index;
        sim->selected_planner_cost_usd = so_planner_candidate_selection_cost(sim);
        return;
    }

    SoSimulation *base = (SoSimulation *)malloc(sizeof(*base));
    SoSimulation *candidate = (SoSimulation *)malloc(sizeof(*candidate));
    SoSimulation *best = (SoSimulation *)malloc(sizeof(*best));
    if (base == NULL || candidate == NULL || best == NULL) {
        free(base);
        free(candidate);
        free(best);
        so_run_single(sim, max_steps);
        sim->selected_planner_trial = sim->planner_trial_index;
        sim->selected_planner_cost_usd = so_planner_candidate_selection_cost(sim);
        return;
    }

    *base = *sim;
    double best_score = 1e100;
    int best_trial = 0;
    for (int trial = 0; trial < trial_count; trial++) {
        *candidate = *base;
        candidate->planner_trial_count = 1;
        candidate->planner_trial_index = trial;
        candidate->selected_planner_trial = trial;
        candidate->planner_weights =
            so_sample_planner_weights(base->planner_weights,
                                      base->planner_seed,
                                      trial);
        so_run_single(candidate, max_steps);
        const double score = so_planner_candidate_selection_cost(candidate);
        if (score < best_score) {
            best_score = score;
            best_trial = trial;
            *best = *candidate;
        }
    }

    *sim = *best;
    sim->planner_trial_count = trial_count;
    sim->selected_planner_trial = best_trial;
    sim->selected_planner_cost_usd = best_score;
    free(base);
    free(candidate);
    free(best);
}

void so_print_summary(const SoSimulation *sim) {
    printf("Scout-Driven Multi-UAV Agricultural OPT C Simulation\n");
    printf("optimization_profile: %s\n", so_optimization_profile_name(sim->optimization_profile));
    printf("planner_search: trials=%d selected_trial=%d selection_cost_usd=%.2f angle_candidate_pool=%d\n",
           sim->planner_trial_count,
           sim->selected_planner_trial,
           sim->selected_planner_cost_usd,
           SO_ANGLE_CANDIDATE_COUNT);
    printf("completed: %s\n", so_completed(sim) ? "true" : "false");
    printf("time_hours: %.2f\n", sim->now_s / 3600.0);
    printf("treated_area_ha: %.6f/%.6f\n", sim->field.treated_ha, sim->field.area_ha);
    printf("task_pool: tasks=%d dropped=%d dropped_area_ha=%.6f residual_spatial_tasks=%d residual_spatial_area_ha=%.6f residual_split_chunks=%d residual_split_area_ha=%.6f\n",
           sim->field.task_count,
           sim->field.dropped_task_count,
           sim->field.dropped_task_area_ha,
           sim->field.residual_spatial_task_count,
           sim->field.residual_spatial_area_ha,
           sim->field.residual_rebuild_split_count,
           sim->field.residual_rebuild_area_ha);
    printf("coverage_error: uncovered_ha=%.6f uncovered_ratio=%.3f%% task_tolerance=%.1f%% final_limit=%.1f%% repair_attempted=%s repair_performed=%s repair_cost=%.2f repair_cost_ratio=%.3f%%\n",
           sim->final_uncovered_ha,
           sim->final_uncovered_ratio * 100.0,
           sim->coverage_task_tolerance_ratio * 100.0,
           sim->coverage_final_error_limit_ratio * 100.0,
           sim->final_repair_attempted ? "true" : "false",
           sim->final_repair_performed ? "true" : "false",
           sim->final_repair_cost_usd,
           sim->final_repair_cost_ratio * 100.0);
    printf("mothership_position: (%.1f, %.1f)\n", sim->mothership.position.x, sim->mothership.position.y);
    printf("depot_stops: %d\n", sim->mothership.operation_plan_count);
    if (sim->fixed_wing.enabled) {
        printf("fixed_wing: model=%s count=%d assigned_ha=%.2f completed_ha=%.2f rate_ha_h=%.2f tank_ha=%.2f fuel_h=%.2f sorties=%d economic_cost_h=%.2f\n",
               sim->fixed_wing.model_name[0] ? sim->fixed_wing.model_name : "none",
               sim->fixed_wing.aircraft_count,
               sim->fixed_wing.assigned_area_ha,
               sim->fixed_wing.completed_area_ha,
               sim->fixed_wing.spray_rate_ha_h,
               sim->fixed_wing.tank_area_ha,
               sim->fixed_wing.fuel_endurance_h,
               sim->fixed_wing.sorties_completed,
               sim->fixed_wing.economic_cost_h);
        printf("fixed_wing_path_strategy: selected=%s score=%.2f multi_island(time=%.3f,cost=%.2f,score=%.2f) partition_dp(time=%.3f,cost=%.2f,score=%.2f) longest_corridor(time=%.3f,cost=%.2f,score=%.2f)\n",
               sim->fixed_wing.path_strategy[0] ? sim->fixed_wing.path_strategy : "unplanned",
               sim->fixed_wing.path_strategy_score,
               sim->fixed_wing.path_strategy_time_h[0],
               sim->fixed_wing.path_strategy_cost_usd[0],
               sim->fixed_wing.path_strategy_scoreboard[0],
               sim->fixed_wing.path_strategy_time_h[1],
               sim->fixed_wing.path_strategy_cost_usd[1],
               sim->fixed_wing.path_strategy_scoreboard[1],
               sim->fixed_wing.path_strategy_time_h[2],
               sim->fixed_wing.path_strategy_cost_usd[2],
               sim->fixed_wing.path_strategy_scoreboard[2]);
    }
    printf("events: %d\n", sim->event_count);
    const int start = sim->event_count > 12 ? sim->event_count - 12 : 0;
    for (int i = start; i < sim->event_count; i++) {
        printf("- %s\n", sim->events[i]);
    }
}

const char *so_drone_state_name(SoDroneState state) {
    switch (state) {
        case SO_DRONE_IDLE: return "idle";
        case SO_DRONE_SCOUTING: return "scouting";
        case SO_DRONE_WORKING: return "working";
        case SO_DRONE_ASSISTING: return "assisting";
        case SO_DRONE_RETURNING: return "returning";
        case SO_DRONE_CHARGING: return "charging";
        case SO_DRONE_REFILLING: return "refilling";
        case SO_DRONE_STANDBY: return "standby";
        case SO_DRONE_CLEANUP: return "cleanup";
        case SO_DRONE_PREDEPLOY: return "predeploy";
        case SO_DRONE_EMERGENCY_LANDING: return "emergency_landing";
        case SO_DRONE_LANDED: return "landed";
        default: return "unknown";
    }
}

const char *so_weather_severity_name(SoWeatherSeverity severity) {
    switch (severity) {
        case SO_WEATHER_NORMAL: return "normal";
        case SO_WEATHER_WATCH: return "watch";
        case SO_WEATHER_WARNING: return "warning";
        case SO_WEATHER_SEVERE: return "severe";
        case SO_WEATHER_EMERGENCY: return "emergency";
        default: return "unknown";
    }
}

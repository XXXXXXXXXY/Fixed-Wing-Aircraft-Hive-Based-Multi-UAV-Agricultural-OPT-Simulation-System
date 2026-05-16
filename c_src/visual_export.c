#include "scout_opt.h"

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *task_kind_name(SoTaskKind kind) {
    switch (kind) {
        case SO_TASK_INTERIOR_STRIP: return "interior_strip";
        case SO_TASK_BOUNDARY: return "boundary";
        case SO_TASK_REPAIR: return "repair";
        default: return "unknown";
    }
}

static double task_strip_length_m(const SoSimulation *sim, const SoFieldTask *task) {
    double swath = sim->spec.spray_swath_m > 0.1 ? sim->spec.spray_swath_m : 4.0;
    return task->area_ha * 10000.0 / swath;
}

static double fixed_wing_strip_length_m(const SoSimulation *sim, const SoFieldTask *task) {
    const double swath = sim->fixed_wing.swath_width_m > 1.0 ? sim->fixed_wing.swath_width_m : 22.0;
    return task->fixed_wing_area_ha * 10000.0 / swath;
}

static double angle_diff_rad(double a, double b) {
    double diff = fmod(fabs(a - b), 2.0 * M_PI);
    if (diff > M_PI) {
        diff = 2.0 * M_PI - diff;
    }
    return diff;
}

static bool task_fixed_wing_handled(const SoSimulation *sim, const SoFieldTask *task) {
    return sim->fixed_wing.enabled &&
           sim->fixed_wing.aircraft_count > 0 &&
           task->fixed_wing_area_ha > 0.001;
}

static const SoFieldBlock *find_block(const SoSimulation *sim, int block_id) {
    for (int i = 0; i < sim->field.block_count; i++) {
        if (sim->field.blocks[i].id == block_id) {
            return &sim->field.blocks[i];
        }
    }
    return NULL;
}

static void route_endpoints(const SoSimulation *sim,
                            const SoFieldTask *task,
                            double route_length,
                            SoPoint *start,
                            SoPoint *end) {
    if (task->has_planned_route) {
        *start = task->route_start;
        *end = task->route_end;
        return;
    }
    const double length = route_length;
    const double angle = task->strip_angle_deg * M_PI / 180.0;
    const double dx = cos(angle) * length * 0.5;
    const double dy = sin(angle) * length * 0.5;
    start->x = task->center.x - dx;
    start->y = task->center.y - dy;
    end->x = task->center.x + dx;
    end->y = task->center.y + dy;

    const SoFieldBlock *block = find_block(sim, task->block_id);
    if (block == NULL || block->boundary_count < 3) {
        return;
    }

    const double ux = cos(angle);
    const double uy = sin(angle);
    double best_neg = -1e100;
    double best_pos = 1e100;
    double min_t = 1e100;
    double max_t = -1e100;
    int intersections = 0;
    for (int i = 0; i < block->boundary_count; i++) {
        const SoPoint a = block->boundary[i];
        const SoPoint b = block->boundary[(i + 1) % block->boundary_count];
        const double vx = b.x - a.x;
        const double vy = b.y - a.y;
        const double wx = a.x - task->center.x;
        const double wy = a.y - task->center.y;
        const double denom = ux * vy - uy * vx;
        if (fabs(denom) < 1e-9) {
            continue;
        }
        const double t = (wx * vy - wy * vx) / denom;
        const double s = (wx * uy - wy * ux) / denom;
        if (s < -1e-6 || s > 1.0 + 1e-6) {
            continue;
        }
        if (t <= 0.0 && t > best_neg) {
            best_neg = t;
        }
        if (t >= 0.0 && t < best_pos) {
            best_pos = t;
        }
        if (t < min_t) {
            min_t = t;
        }
        if (t > max_t) {
            max_t = t;
        }
        intersections++;
    }
    if (intersections >= 2 && best_neg > -1e90 && best_pos < 1e90) {
        const double half = fmin(length * 0.5, fmin(-best_neg, best_pos) * 0.96);
        if (half > 8.0) {
            start->x = task->center.x - ux * half;
            start->y = task->center.y - uy * half;
            end->x = task->center.x + ux * half;
            end->y = task->center.y + uy * half;
        }
    } else if (intersections >= 2 && min_t < max_t) {
        const double margin = fmax(2.0, fmin(18.0, (max_t - min_t) * 0.02));
        start->x = task->center.x + ux * (min_t + margin);
        start->y = task->center.y + uy * (min_t + margin);
        end->x = task->center.x + ux * (max_t - margin);
        end->y = task->center.y + uy * (max_t - margin);
    }
}

static void emit_point(FILE *file, SoPoint p) {
    fprintf(file, "{\"x\": %.3f, \"y\": %.3f}", p.x, p.y);
}

typedef struct {
    char type[3];
    double a;
    double b;
    double c;
    double length;
    bool valid;
} VisualDubinsPath;

static double mod2pi(double value) {
    double out = fmod(value, 2.0 * M_PI);
    if (out < 0.0) {
        out += 2.0 * M_PI;
    }
    return out;
}

static void consider_dubins_path(VisualDubinsPath *best,
                                 const char *type,
                                 double a,
                                 double b,
                                 double c,
                                 double radius_m) {
    if (a < -1e-9 || b < -1e-9 || c < -1e-9) {
        return;
    }
    const double length = (a + b + c) * radius_m;
    if (!best->valid || length < best->length) {
        best->type[0] = type[0];
        best->type[1] = type[1];
        best->type[2] = type[2];
        best->a = a;
        best->b = b;
        best->c = c;
        best->length = length;
        best->valid = true;
    }
}

static VisualDubinsPath shortest_dubins_path(SoPoint from,
                                             double from_heading,
                                             SoPoint to,
                                             double to_heading,
                                             double radius_m) {
    VisualDubinsPath best = {{'S', 'S', 'S'}, 0.0, 0.0, 0.0, 0.0, false};
    if (radius_m <= 1.0) {
        return best;
    }

    const double dx = (to.x - from.x) / radius_m;
    const double dy = (to.y - from.y) / radius_m;
    const double d = hypot(dx, dy);
    if (d < 1e-6) {
        return best;
    }
    const double theta = atan2(dy, dx);
    const double alpha = mod2pi(from_heading - theta);
    const double beta = mod2pi(to_heading - theta);
    const double sa = sin(alpha);
    const double sb = sin(beta);
    const double ca = cos(alpha);
    const double cb = cos(beta);
    const double cab = cos(alpha - beta);

    double tmp0;
    double p2;
    double tmp1;

    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sa - sb);
    if (p2 >= 0.0) {
        tmp0 = d + sa - sb;
        tmp1 = atan2(cb - ca, tmp0);
        consider_dubins_path(&best, "LSL", mod2pi(-alpha + tmp1), sqrt(p2),
                             mod2pi(beta - tmp1), radius_m);
    }

    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (-sa + sb);
    if (p2 >= 0.0) {
        tmp0 = d - sa + sb;
        tmp1 = atan2(ca - cb, tmp0);
        consider_dubins_path(&best, "RSR", mod2pi(alpha - tmp1), sqrt(p2),
                             mod2pi(-beta + tmp1), radius_m);
    }

    p2 = -2.0 + d * d + 2.0 * cab + 2.0 * d * (sa + sb);
    if (p2 >= 0.0) {
        const double p = sqrt(p2);
        tmp0 = atan2(-ca - cb, d + sa + sb) - atan2(-2.0, p);
        consider_dubins_path(&best, "LSR", mod2pi(-alpha + tmp0), p,
                             mod2pi(-mod2pi(beta) + tmp0), radius_m);
    }

    p2 = -2.0 + d * d + 2.0 * cab - 2.0 * d * (sa + sb);
    if (p2 >= 0.0) {
        const double p = sqrt(p2);
        tmp0 = atan2(ca + cb, d - sa - sb) - atan2(2.0, p);
        consider_dubins_path(&best, "RSL", mod2pi(alpha - tmp0), p,
                             mod2pi(beta - tmp0), radius_m);
    }

    return best;
}

static void emit_arc_segment(FILE *file,
                             SoPoint *position,
                             double *heading,
                             double angle_rad,
                             double radius_m,
                             int direction) {
    if (angle_rad <= 1e-6) {
        return;
    }
    const double sign = direction >= 0 ? 1.0 : -1.0;
    const double nx = -sin(*heading) * sign;
    const double ny = cos(*heading) * sign;
    const SoPoint center = {position->x + nx * radius_m,
                            position->y + ny * radius_m};
    const double start_theta = atan2(position->y - center.y, position->x - center.x);
    const int steps = (int)fmax(2.0, ceil(angle_rad / (M_PI / 24.0)));
    for (int i = 1; i <= steps; i++) {
        const double step_angle = angle_rad * (double)i / (double)steps;
        const double theta = start_theta + sign * step_angle;
        const SoPoint p = {center.x + cos(theta) * radius_m,
                           center.y + sin(theta) * radius_m};
        fprintf(file, ", ");
        emit_point(file, p);
    }
    *heading = mod2pi(*heading + sign * angle_rad);
    position->x = center.x + cos(start_theta + sign * angle_rad) * radius_m;
    position->y = center.y + sin(start_theta + sign * angle_rad) * radius_m;
}

static void emit_straight_segment(FILE *file,
                                  SoPoint *position,
                                  double heading,
                                  double length_m) {
    if (length_m <= 1.0) {
        return;
    }
    const int steps = (int)fmax(1.0, ceil(length_m / 220.0));
    for (int i = 1; i <= steps; i++) {
        const double d = length_m * (double)i / (double)steps;
        const SoPoint p = {position->x + cos(heading) * d,
                           position->y + sin(heading) * d};
        fprintf(file, ", ");
        emit_point(file, p);
    }
    position->x += cos(heading) * length_m;
    position->y += sin(heading) * length_m;
}

static void emit_radius_limited_transition(FILE *file,
                                           SoPoint from,
                                           double from_heading,
                                           SoPoint to,
                                           double to_heading,
                                           double radius_m) {
    const double gap = hypot(to.x - from.x, to.y - from.y);
    if (gap <= 1.0) {
        return;
    }

    if (radius_m <= 1.0) {
        fprintf(file, ", ");
        emit_point(file, to);
        return;
    }

    const VisualDubinsPath path = shortest_dubins_path(from, from_heading, to, to_heading, radius_m);
    if (!path.valid) {
        fprintf(file, ", ");
        emit_point(file, to);
        return;
    }

    SoPoint position = from;
    double heading = from_heading;
    const double params[3] = {path.a, path.b, path.c};
    for (int i = 0; i < 3; i++) {
        if (path.type[i] == 'L') {
            emit_arc_segment(file, &position, &heading, params[i], radius_m, 1);
        } else if (path.type[i] == 'R') {
            emit_arc_segment(file, &position, &heading, params[i], radius_m, -1);
        } else {
            emit_straight_segment(file, &position, heading, params[i] * radius_m);
        }
    }
    fprintf(file, ", ");
    emit_point(file, to);
}

static void emit_fixed_wing_mission_trajectory(FILE *file, const SoSimulation *sim) {
    fprintf(file, "[");
    emit_point(file, sim->fixed_wing.airport);

    bool used[SO_MAX_TASKS] = {false};
    int remaining = 0;
    for (int i = 0; i < sim->field.task_count; i++) {
        if (task_fixed_wing_handled(sim, &sim->field.tasks[i])) {
            remaining++;
        }
    }

    SoPoint current = sim->fixed_wing.airport;
    double current_heading = 0.0;
    bool has_heading = false;
    double sortie_area = 0.0;
    const double tank_area = fmax(1.0, sim->fixed_wing.tank_area_ha);
    const double turn_radius = fmax(1.0, sim->fixed_wing.turn_radius_m);
    int wrote_strip = 0;
    while (remaining > 0) {
        int best = -1;
        SoPoint best_start = {0.0, 0.0};
        SoPoint best_end = {0.0, 0.0};
        double best_heading = 0.0;
        double best_score = 1e100;
        for (int i = 0; i < sim->field.task_count; i++) {
            if (used[i]) {
                continue;
            }
            const SoFieldTask *task = &sim->field.tasks[i];
            if (!task_fixed_wing_handled(sim, task)) {
                continue;
            }
            if (sortie_area > 0.001 && sortie_area + task->fixed_wing_area_ha > tank_area) {
                continue;
            }
            SoPoint start;
            SoPoint end;
            route_endpoints(sim, task, fixed_wing_strip_length_m(sim, task), &start, &end);
            for (int dir = 0; dir < 2; dir++) {
                const SoPoint candidate_start = dir == 0 ? start : end;
                const SoPoint candidate_end = dir == 0 ? end : start;
                const double heading = atan2(candidate_end.y - candidate_start.y,
                                             candidate_end.x - candidate_start.x);
                const double empty_m = hypot(current.x - candidate_start.x,
                                             current.y - candidate_start.y);
                const double turn_m = has_heading
                                          ? angle_diff_rad(current_heading, heading) * turn_radius
                                          : 0.0;
                const double score = empty_m + turn_m;
                if (score < best_score) {
                    best = i;
                    best_start = candidate_start;
                    best_end = candidate_end;
                    best_heading = heading;
                    best_score = score;
                }
            }
        }
        if (best < 0) {
            if (has_heading) {
                emit_radius_limited_transition(file, current, current_heading,
                                               sim->fixed_wing.airport, current_heading,
                                               turn_radius);
            } else {
                fprintf(file, ", ");
                emit_point(file, sim->fixed_wing.airport);
            }
            current = sim->fixed_wing.airport;
            current_heading = 0.0;
            has_heading = false;
            sortie_area = 0.0;
            continue;
        }

        const SoFieldTask *task = &sim->field.tasks[best];
        const double sx = best_end.x - best_start.x;
        const double sy = best_end.y - best_start.y;
        const double route_angle = atan2(sy, sx);
        const SoPoint approach = {best_start.x - cos(route_angle) * 220.0,
                                  best_start.y - sin(route_angle) * 220.0};
        if (sortie_area <= 0.001) {
            fprintf(file, ", ");
            emit_point(file, approach);
        } else if (has_heading) {
            emit_radius_limited_transition(file, current, current_heading,
                                           best_start, route_angle, turn_radius);
        }
        fprintf(file, ", ");
        emit_point(file, best_start);
        fprintf(file, ", ");
        emit_point(file, best_end);
        current = best_end;
        current_heading = best_heading;
        has_heading = true;
        sortie_area += task->fixed_wing_area_ha;
        used[best] = true;
        remaining--;
        wrote_strip = 1;
    }
    if (wrote_strip) {
        emit_radius_limited_transition(file, current, current_heading,
                                       sim->fixed_wing.airport, current_heading,
                                       turn_radius);
    }
    fprintf(file, "]");
}

static int line_block_intervals(const SoFieldBlock *block,
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

static int emit_task_coverage_route(FILE *file,
                                    const SoSimulation *sim,
                                    const SoFieldTask *task,
                                    double swath,
                                    int max_lines) {
    const SoFieldBlock *block = find_block(sim, task->block_id);
    if (block == NULL || block->boundary_count < 3 || task->area_ha <= 0.001) {
        return 0;
    }
    const double angle = task->strip_angle_deg * M_PI / 180.0;
    const double ux = cos(angle);
    const double uy = sin(angle);
    const double vx = -uy;
    const double vy = ux;
    double min_cross = 1e100;
    double max_cross = -1e100;
    for (int p = 0; p < block->boundary_count; p++) {
        const double c = block->boundary[p].x * vx + block->boundary[p].y * vy;
        if (c < min_cross) {
            min_cross = c;
        }
        if (c > max_cross) {
            max_cross = c;
        }
    }
    double remaining_m = task->area_ha * 10000.0 / fmax(0.001, swath);
    int written = 0;
    const double start_cross = min_cross + swath * 0.5;
    const double end_cross = max_cross - swath * 0.25;
    for (double cross = start_cross;
         cross <= end_cross && remaining_m > 0.001 && written < max_lines;
         cross += swath) {
        double mins[SO_MAX_BOUNDARY_POINTS / 2];
        double maxs[SO_MAX_BOUNDARY_POINTS / 2];
        const int intervals = line_block_intervals(block, angle, cross, mins, maxs, SO_MAX_BOUNDARY_POINTS / 2);
        for (int seg = 0; seg < intervals && remaining_m > 0.001 && written < max_lines; seg++) {
            const double full_len = maxs[seg] - mins[seg];
            const double use_len = fmin(full_len, remaining_m);
            const double start_t = mins[seg];
            const double end_t = mins[seg] + use_len;
            const SoPoint a = {ux * start_t + vx * cross, uy * start_t + vy * cross};
            const SoPoint b = {ux * end_t + vx * cross, uy * end_t + vy * cross};
            fprintf(file,
                    "%s[{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}]",
                    written ? ", " : "",
                    a.x, a.y, b.x, b.y);
            written++;
            remaining_m -= use_len;
        }
    }
    return written;
}

static double block_side_m(const SoFieldBlock *block) {
    return fmax(120.0, sqrt(fmax(1.0, block->area_ha) * 10000.0) * 1.05);
}

static void emit_block_boundary_points(FILE *file, const SoFieldBlock *block) {
    if (block->boundary_count >= 3) {
        for (int i = 0; i <= block->boundary_count; i++) {
            const SoPoint p = block->boundary[i % block->boundary_count];
            fprintf(file, "%s{\"x\": %.3f, \"y\": %.3f}", i == 0 ? "" : ", ", p.x, p.y);
        }
        return;
    }
    const double half = block_side_m(block) * 0.5;
    fprintf(file,
            "{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}, "
            "{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}",
            block->center.x - half, block->center.y - half,
            block->center.x + half, block->center.y - half,
            block->center.x + half, block->center.y + half,
            block->center.x - half, block->center.y + half,
            block->center.x - half, block->center.y - half);
}

bool so_export_visual_plan(const SoSimulation *sim, const char *path) {
    FILE *file = fopen(path, "wb");
    if (file == NULL) {
        return false;
    }

    int visual_drone_owner[SO_MAX_TASKS];
    double visual_drone_load[SO_MAX_DRONES];
    for (int i = 0; i < SO_MAX_TASKS; i++) {
        visual_drone_owner[i] = -1;
    }
    for (int i = 0; i < SO_MAX_DRONES; i++) {
        visual_drone_load[i] = 0.0;
    }
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *task = &sim->field.tasks[i];
        if (task->area_ha <= 0.001) {
            continue;
        }
        int best = 0;
        for (int d = 1; d < sim->drone_count; d++) {
            if (visual_drone_load[d] < visual_drone_load[best]) {
                best = d;
            }
        }
        visual_drone_owner[i] = best + 1;
        visual_drone_load[best] += task->area_ha / 8.0 + task->risk * 0.05;
    }

    fprintf(file, "{\n");
    fprintf(file, "  \"visual_plan_version\": 1,\n");
    fprintf(file, "  \"source\": \"scout_opt_c\",\n");
    fprintf(file, "  \"time_hours\": %.6f,\n", sim->now_s / 3600.0);
    fprintf(file, "  \"origin\": {\"lat\": %.12f, \"lon\": %.12f},\n",
            sim->field.has_origin ? sim->field.origin_lat : 32.085894448486876,
            sim->field.has_origin ? sim->field.origin_lon : 118.89885869105146);
    fprintf(file, "  \"hive\": {\n");
    fprintf(file, "    \"speed_kmh\": %.3f,\n", sim->mothership.move_speed_mps * 3.6);
    fprintf(file, "    \"can_move_while_service_busy\": false,\n");
    fprintf(file, "    \"move_policy\": \"cleanup_only_max_2_drones\",\n");
    fprintf(file, "    \"move_distance_m\": %.3f,\n", sim->mothership.move_distance_m);
    fprintf(file, "    \"truck_cost_usd_per_km\": %.3f,\n", sim->mothership.truck_cost_usd_per_km);
    fprintf(file, "    \"move_cost_usd\": %.3f,\n", sim->mothership.move_cost_usd);
    fprintf(file, "    \"deployment_stop_cost_usd\": %.3f,\n", sim->mothership.deployment_stop_cost_usd);
    fprintf(file, "    \"stop_cost_usd\": %.3f,\n", sim->mothership.stop_cost_usd);
    fprintf(file, "    \"start\": {\"x\": %.3f, \"y\": %.3f},\n",
            sim->mothership.operation_plan_count > 0 ? sim->mothership.operation_plan[0].x : sim->mothership.position.x,
            sim->mothership.operation_plan_count > 0 ? sim->mothership.operation_plan[0].y : sim->mothership.position.y);
    fprintf(file, "    \"stops\": [\n");
    for (int i = 0; i < sim->mothership.operation_plan_count; i++) {
        const SoPoint p = sim->mothership.operation_plan[i];
        fprintf(file, "      {\"index\": %d, \"x\": %.3f, \"y\": %.3f}%s\n",
                i, p.x, p.y, i + 1 == sim->mothership.operation_plan_count ? "" : ",");
    }
    fprintf(file, "    ]\n");
    fprintf(file, "  },\n");

    fprintf(file, "  \"drones\": {\"count\": %d, \"model\": \"dji_agras_t200\", \"payload_kg\": 200.0, \"rtk\": true, \"obstacle_sensing\": \"omnidirectional\", \"cruise_speed_mps\": %.3f, \"spray_speed_mps\": %.3f, \"spray_swath_m\": %.3f, \"spray_radius_m\": %.3f, \"spray_rate_ha_h\": %.3f, \"turn_time_s\": %.3f, \"turn_battery_cost\": %.5f, \"turn_radius_m\": %.3f, \"chemical_l_per_ha\": %.3f, \"chemical_cost_usd_per_l\": %.3f, \"battery_cost_usd_per_unit\": %.3f, \"unfinished_penalty_usd_per_ha\": %.3f, \"flight_cost_usd_per_km\": %.3f, \"launch_cost_usd\": %.3f, \"flight_distance_m\": %.3f, \"flight_cost_usd\": %.3f, \"takeoffs\": %d, \"launch_cost_total_usd\": %.3f, \"total_cost_usd\": %.3f, \"altitude_m\": 18.0},\n",
            sim->drone_count, sim->spec.cruise_speed_mps, sim->spec.cruise_speed_mps * 0.45,
            sim->spec.spray_swath_m, sim->spec.spray_radius_m, sim->spec.spray_rate_ha_h,
            sim->spec.turn_time_s, sim->spec.turn_battery_cost,
            sim->spec.turn_radius_m,
            sim->spec.chemical_l_per_ha,
            sim->spec.chemical_cost_usd_per_l,
            sim->spec.battery_cost_usd_per_unit,
            sim->spec.unfinished_penalty_usd_per_ha,
            sim->spec.flight_cost_usd_per_km, sim->spec.launch_cost_usd,
            sim->uav_flight_distance_m, sim->uav_flight_cost_usd,
            sim->uav_takeoffs, sim->uav_launch_cost_usd,
            sim->uav_flight_cost_usd + sim->uav_launch_cost_usd);

    fprintf(file, "  \"fixed_wing\": {\"enabled\": %s, \"count\": %d, \"model\": \"%s\", \"engine\": \"Pratt & Whitney PT6A-34AG\", \"power_shp\": 750.0, \"payload_kg\": %.1f, \"tank_l\": %.1f, \"fuel_l\": %.1f, \"tank_area_ha\": %.3f, \"fuel_endurance_h\": %.3f, \"cruise_speed_mps\": %.3f, \"work_speed_mps\": %.3f, \"swath_m\": %.3f, \"turn_time_s\": %.3f, \"turn_fuel_h\": %.6f, \"turn_radius_m\": %.3f, \"planned_turn_non_spray_time_s\": %.3f, \"turn_non_spray_time_s\": %.3f, \"turn_spraying_allowed\": false, \"chemical_l_per_ha\": %.3f, \"chemical_cost_usd_per_l\": %.3f, \"fuel_cost_usd_per_h\": %.3f, \"unfinished_penalty_usd_per_ha\": %.3f, \"planned_turns\": %d, \"corridor_count\": %d, \"corridor_work_m\": %.3f, \"corridor_empty_m\": %.3f, \"corridor_total_m\": %.3f, \"flight_cost_usd_per_km\": %.3f, \"takeoff_cost_usd\": %.3f, \"airport_service_cost_usd\": %.3f, \"flight_distance_m\": %.3f, \"flight_cost_usd\": %.3f, \"airport_cost_usd\": %.3f, \"total_cost_usd\": %.3f, \"economic_cost_h\": %.3f, \"airport\": {\"x\": %.3f, \"y\": %.3f}, \"return_point\": {\"x\": %.3f, \"y\": %.3f}, \"altitude_m\": 55.0},\n",
            sim->fixed_wing.enabled ? "true" : "false",
            sim->fixed_wing.aircraft_count,
            sim->fixed_wing.model_name[0] ? sim->fixed_wing.model_name : "none",
            sim->fixed_wing.payload_kg > 1.0 ? sim->fixed_wing.payload_kg : 2450.0,
            sim->fixed_wing.tank_l > 1.0 ? sim->fixed_wing.tank_l : 1893.0,
            sim->fixed_wing.fuel_l > 1.0 ? sim->fixed_wing.fuel_l : 644.0,
            sim->fixed_wing.tank_area_ha > 0.1 ? sim->fixed_wing.tank_area_ha : 189.3,
            sim->fixed_wing.fuel_endurance_h > 0.1 ? sim->fixed_wing.fuel_endurance_h : 3.2,
            sim->fixed_wing.cruise_speed_mps > 1.0 ? sim->fixed_wing.cruise_speed_mps : 42.0,
            sim->fixed_wing.work_speed_mps > 1.0 ? sim->fixed_wing.work_speed_mps : 59.0,
            sim->fixed_wing.swath_width_m > 1.0 ? sim->fixed_wing.swath_width_m : 36.0,
            sim->fixed_wing.turn_time_s,
            sim->fixed_wing.turn_fuel_h,
            sim->fixed_wing.turn_radius_m,
            sim->fixed_wing.planned_turn_non_spray_time_s,
            sim->fixed_wing.turn_non_spray_time_s,
            sim->fixed_wing.chemical_l_per_ha,
            sim->fixed_wing.chemical_cost_usd_per_l,
            sim->fixed_wing.fuel_cost_usd_per_h,
            sim->fixed_wing.unfinished_penalty_usd_per_ha,
            sim->fixed_wing.planned_turns,
            sim->fixed_wing.corridor_count,
            sim->fixed_wing.corridor_work_m,
            sim->fixed_wing.corridor_empty_m,
            sim->fixed_wing.corridor_total_m,
            sim->fixed_wing.flight_cost_usd_per_km,
            sim->fixed_wing.takeoff_cost_usd,
            sim->fixed_wing.airport_service_cost_usd,
            sim->fixed_wing.flight_distance_m,
            sim->fixed_wing.flight_cost_usd,
            sim->fixed_wing.airport_cost_usd,
            sim->fixed_wing.flight_cost_usd + sim->fixed_wing.airport_cost_usd,
            sim->fixed_wing.economic_cost_h,
            sim->fixed_wing.airport.x,
            sim->fixed_wing.airport.y,
            sim->fixed_wing.airport.x,
            sim->fixed_wing.airport.y);

    const double uav_total_cost = sim->uav_flight_cost_usd + sim->uav_launch_cost_usd;
    const double fixed_wing_total_cost = sim->fixed_wing.flight_cost_usd + sim->fixed_wing.airport_cost_usd;
    const double hive_total_cost = sim->mothership.move_cost_usd + sim->mothership.stop_cost_usd;
    fprintf(file,
            "  \"cost_summary\": {\"currency\": \"USD\", \"uav_total_usd\": %.3f, "
            "\"fixed_wing_total_usd\": %.3f, \"hive_total_usd\": %.3f, "
            "\"mission_total_usd\": %.3f, "
            "\"model\": \"C_total=C_spray+C_empty+C_turn+C_energy+C_risk+C_unfinished\", "
            "\"notes\": \"Task selection uses operational cost: chemical area cost, spray distance cost, empty flight/ferry cost, turn-radius energy cost, weather risk multiplier, and unfinished-area penalty; Hive cost uses truck movement plus deployment stops.\"},\n",
            uav_total_cost,
            fixed_wing_total_cost,
            hive_total_cost,
            uav_total_cost + fixed_wing_total_cost + hive_total_cost);

    fprintf(file, "  \"scout_routes\": [\n");
    int scout_written = 0;
    for (int i = 0; i < sim->field.block_count; i++) {
        const SoFieldBlock *block = &sim->field.blocks[i];
        if (!block->selected) {
            continue;
        }
        fprintf(file,
                "%s    {\"drone_id\": %d, \"block_id\": %d, \"altitude_m\": 28.0, \"speed_mps\": %.3f, "
                "\"route\": [",
                scout_written ? ",\n" : "",
                scout_written + 1,
                block->id,
                sim->spec.scout_speed_mps);
        emit_block_boundary_points(file, block);
        fprintf(file, "]}\n");
        scout_written++;
    }
    fprintf(file, "  ],\n");

    fprintf(file, "  \"work_area\": {\n");
    fprintf(file, "    \"source\": \"selected_field_blocks\",\n");
    fprintf(file, "    \"display\": \"field_boundary_overlay\",\n");
    fprintf(file, "    \"blocks\": [\n");
    int area_written = 0;
    for (int i = 0; i < sim->field.block_count; i++) {
        const SoFieldBlock *block = &sim->field.blocks[i];
        if (!block->selected) {
            continue;
        }
        fprintf(file,
                "%s      {\"block_id\": %d, \"name\": \"%s\", \"area_ha\": %.6f, "
                "\"center\": {\"x\": %.3f, \"y\": %.3f}, "
                "\"boundary\": [",
                area_written ? ",\n" : "",
                block->id,
                block->name != NULL ? block->name : "field block",
                block->area_ha,
                block->center.x,
                block->center.y);
        emit_block_boundary_points(file, block);
        fprintf(file, "]}\n");
        area_written++;
    }
    fprintf(file, "    ]\n");
    fprintf(file, "  },\n");

    fprintf(file, "  \"fixed_wing_routes\": [\n");
    int fixed_route_written = 0;
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *task = &sim->field.tasks[i];
        if (!task_fixed_wing_handled(sim, task)) {
            continue;
        }
        SoPoint start;
        SoPoint end;
        route_endpoints(sim, task, fixed_wing_strip_length_m(sim, task), &start, &end);
        fprintf(file,
                "%s    {\"task_id\": %d, \"block_id\": %d, \"area_ha\": %.6f, "
                "\"strip_angle_deg\": %.3f, \"turn_count\": %d, \"turn_time_s\": %.3f, "
                "\"center\": {\"x\": %.3f, \"y\": %.3f}, "
                "\"route\": [{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}]}\n",
                fixed_route_written ? "," : "",
                task->id, task->block_id, task->fixed_wing_area_ha,
                task->strip_angle_deg,
                task->turn_count,
                (double)task->turn_count * sim->fixed_wing.turn_time_s,
                task->center.x, task->center.y,
                start.x, start.y, end.x, end.y);
        fixed_route_written++;
    }
    fprintf(file, "  ],\n");

    fprintf(file, "  \"fixed_wing_trajectory\": ");
    emit_fixed_wing_mission_trajectory(file, sim);
    fprintf(file, ",\n");

    fprintf(file, "  \"tasks\": [\n");
    int task_written = 0;
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *task = &sim->field.tasks[i];
        if (task->area_ha <= 0.001) {
            continue;
        }
        SoPoint start;
        SoPoint end;
        route_endpoints(sim, task, task_strip_length_m(sim, task), &start, &end);
        fprintf(file,
                "%s    {\"id\": %d, \"block_id\": %d, \"kind\": \"%s\", \"handling\": \"%s\", "
                "\"assigned_drone_id\": %d, \"area_ha\": %.6f, \"strip_angle_deg\": %.3f, "
                "\"turn_count\": %d, \"turn_time_s\": %.3f, \"turn_energy_cost\": %.5f, "
                "\"center\": {\"x\": %.3f, \"y\": %.3f}, "
                "\"route\": [{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}], "
                "\"coverage_route\": [",
                task_written ? "," : "",
                task->id, task->block_id, task_kind_name(task->kind),
                "drone",
                visual_drone_owner[i],
                task->area_ha,
                task->strip_angle_deg,
                task->turn_count,
                task->turn_time_s,
                task->turn_energy_cost,
                task->center.x, task->center.y,
                start.x, start.y, end.x, end.y);
        emit_task_coverage_route(file, sim, task, sim->spec.spray_swath_m, 160);
        fprintf(file, "]}\n");
        task_written++;
    }
    fprintf(file, "  ]\n");
    fprintf(file, "}\n");

    fclose(file);
    return true;
}

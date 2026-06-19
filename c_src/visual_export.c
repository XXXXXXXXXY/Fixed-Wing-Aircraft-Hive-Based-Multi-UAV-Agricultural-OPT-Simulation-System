#include "scout_opt.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define VISUAL_UAV_MAX_TRACK_PASSES 240

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
    const double curve_rad = fabs(task->route_curve_deg) * M_PI / 180.0;
    const double curve_factor =
        curve_rad < 1e-6 ? 1.0 : curve_rad / fmax(1e-6, 2.0 * sin(curve_rad * 0.5));
    return task->area_ha * 10000.0 / swath * curve_factor;
}

static double fixed_wing_strip_length_m(const SoSimulation *sim, const SoFieldTask *task) {
    const double swath = sim->fixed_wing.swath_width_m > 1.0 ? sim->fixed_wing.swath_width_m : 22.0;
    const double curve_rad = fabs(task->route_curve_deg) * M_PI / 180.0;
    const double curve_factor =
        curve_rad < 1e-6 ? 1.0 : curve_rad / fmax(1e-6, 2.0 * sin(curve_rad * 0.5));
    return task->fixed_wing_area_ha * 10000.0 / swath * curve_factor;
}

static bool task_fixed_wing_handled(const SoSimulation *sim, const SoFieldTask *task) {
    return sim->fixed_wing.enabled &&
           sim->fixed_wing.aircraft_count > 0 &&
           task->fixed_wing_area_ha > 0.001;
}

static double visual_heading_diff_rad(double a, double b) {
    double diff = fmod(fabs(a - b), 2.0 * M_PI);
    if (diff > M_PI) {
        diff = 2.0 * M_PI - diff;
    }
    return diff;
}

static double visual_fixed_wing_route_choice_score(const SoSimulation *sim,
                                                   const SoFieldTask *task,
                                                   int current_block_id,
                                                   double transition_m,
                                                   double turn_m,
                                                   double strip_m) {
    if (strcmp(sim->fixed_wing.path_strategy, "partition_dp") == 0) {
        const double block_switch =
            current_block_id >= 0 && current_block_id != task->block_id ? 260.0 : -80.0;
        return transition_m + turn_m * 0.95 + block_switch;
    }
    if (strcmp(sim->fixed_wing.path_strategy, "longest_corridor") == 0) {
        return transition_m * 0.72 + turn_m * 0.90 - strip_m * 0.42;
    }
    return transition_m + turn_m * 0.85;
}

static const SoFieldBlock *find_block(const SoSimulation *sim, int block_id) {
    for (int i = 0; i < sim->field.block_count; i++) {
        if (sim->field.blocks[i].id == block_id) {
            return &sim->field.blocks[i];
        }
    }
    return NULL;
}

static int visual_uav_pass_count_for_width(double work_width_m, double swath_m) {
    const double swath = fmax(0.001, swath_m);
    if (work_width_m <= swath) {
        return 1;
    }
    const int full_passes = (int)floor(work_width_m / swath);
    const double residual_width = work_width_m - (double)full_passes * swath;
    return fmax(1, full_passes + (residual_width > swath * 0.05 ? 1 : 0));
}

static double visual_uav_effective_pass_route_length_m(const SoFieldTask *task,
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

static bool clip_route_to_block_boundary(const SoSimulation *sim,
                                         const SoFieldTask *task,
                                         SoPoint *start,
                                         SoPoint *end) {
    const SoFieldBlock *block = find_block(sim, task->block_id);
    if (block == NULL || block->boundary_count < 3) {
        return true;
    }
    const double dx = end->x - start->x;
    const double dy = end->y - start->y;
    const double length = hypot(dx, dy);
    if (length <= 1.0) {
        return false;
    }
    const double ux = dx / length;
    const double uy = dy / length;
    const SoPoint mid = {(start->x + end->x) * 0.5, (start->y + end->y) * 0.5};
    double min_t = 1e100;
    double max_t = -1e100;
    int intersections = 0;
    for (int i = 0; i < block->boundary_count; i++) {
        const SoPoint a = block->boundary[i];
        const SoPoint b = block->boundary[(i + 1) % block->boundary_count];
        const double vx = b.x - a.x;
        const double vy = b.y - a.y;
        const double wx = a.x - mid.x;
        const double wy = a.y - mid.y;
        const double denom = ux * vy - uy * vx;
        if (fabs(denom) < 1e-9) {
            continue;
        }
        const double t = (wx * vy - wy * vx) / denom;
        const double s = (wx * uy - wy * ux) / denom;
        if (s < -1e-6 || s > 1.0 + 1e-6) {
            continue;
        }
        min_t = fmin(min_t, t);
        max_t = fmax(max_t, t);
        intersections++;
    }
    if (intersections < 2 || max_t - min_t <= 8.0) {
        return false;
    }
    const double margin = fmin(12.0, fmax(0.0, (max_t - min_t) * 0.01));
    start->x = mid.x + ux * (min_t + margin);
    start->y = mid.y + uy * (min_t + margin);
    end->x = mid.x + ux * (max_t - margin);
    end->y = mid.y + uy * (max_t - margin);
    return true;
}

static void visual_block_projection_range(const SoFieldBlock *block,
                                          double angle_rad,
                                          double *out_min_cross,
                                          double *out_max_cross) {
    const double nx = -sin(angle_rad);
    const double ny = cos(angle_rad);
    double min_cross = 1e100;
    double max_cross = -1e100;
    if (block != NULL && block->boundary_count >= 3) {
        for (int i = 0; i < block->boundary_count; i++) {
            const double c = block->boundary[i].x * nx + block->boundary[i].y * ny;
            min_cross = fmin(min_cross, c);
            max_cross = fmax(max_cross, c);
        }
    }
    if (min_cross > max_cross) {
        min_cross = -100.0;
        max_cross = 100.0;
    }
    *out_min_cross = min_cross;
    *out_max_cross = max_cross;
}

static int visual_line_block_intervals(const SoFieldBlock *block,
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
    const double nx = -uy;
    const double ny = ux;
    double ts[SO_MAX_BOUNDARY_POINTS];
    int count = 0;
    for (int i = 0; i < block->boundary_count; i++) {
        const SoPoint a = block->boundary[i];
        const SoPoint b = block->boundary[(i + 1) % block->boundary_count];
        const double ca = a.x * nx + a.y * ny - cross;
        const double cb = b.x * nx + b.y * ny - cross;
        if (fabs(ca) <= 1e-9 && count < SO_MAX_BOUNDARY_POINTS) {
            ts[count++] = a.x * ux + a.y * uy;
        }
        if (ca * cb < -1e-9) {
            const double r = ca / (ca - cb);
            const SoPoint p = {a.x + (b.x - a.x) * r, a.y + (b.y - a.y) * r};
            if (count < SO_MAX_BOUNDARY_POINTS) {
                ts[count++] = p.x * ux + p.y * uy;
            }
        }
    }
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (ts[j] < ts[i]) {
                const double tmp = ts[i];
                ts[i] = ts[j];
                ts[j] = tmp;
            }
        }
    }
    int intervals = 0;
    for (int i = 0; i + 1 < count && intervals < max_intervals; i += 2) {
        if (ts[i + 1] - ts[i] > 1.0) {
            mins[intervals] = ts[i];
            maxs[intervals] = ts[i + 1];
            intervals++;
        }
    }
    return intervals;
}

static bool visual_uav_marked_pass_segment(const SoFieldBlock *block,
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
    visual_block_projection_range(block, angle_rad, &min_cross, &max_cross);
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
        visual_line_block_intervals(block, angle_rad, cross, mins, maxs,
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

static void emit_route_points(FILE *file,
                              const SoFieldTask *task,
                              SoPoint start,
                              SoPoint end) {
    if (task->has_route_mid && fabs(task->route_curve_deg) > 0.01) {
        fprintf(file,
                "[{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}]",
                start.x, start.y,
                task->route_mid.x, task->route_mid.y,
                end.x, end.y);
    } else {
        fprintf(file,
                "[{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}]",
                start.x, start.y, end.x, end.y);
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

    tmp0 = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sa - sb)) / 8.0;
    if (fabs(tmp0) <= 1.0) {
        const double p = mod2pi(2.0 * M_PI - acos(tmp0));
        const double t = mod2pi(alpha - atan2(ca - cb, d - sa + sb) + p * 0.5);
        const double q = mod2pi(alpha - beta - t + p);
        consider_dubins_path(&best, "RLR", t, p, q, radius_m);
    }

    tmp0 = (6.0 - d * d + 2.0 * cab + 2.0 * d * (-sa + sb)) / 8.0;
    if (fabs(tmp0) <= 1.0) {
        const double p = mod2pi(2.0 * M_PI - acos(tmp0));
        const double t = mod2pi(-alpha - atan2(ca - cb, d + sa - sb) + p * 0.5);
        const double q = mod2pi(beta - alpha - t + p);
        consider_dubins_path(&best, "LRL", t, p, q, radius_m);
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

static double dubins_transition_length(SoPoint from,
                                       double from_heading,
                                       SoPoint to,
                                       double to_heading,
                                       double radius_m) {
    const VisualDubinsPath path = shortest_dubins_path(from, from_heading, to, to_heading, radius_m);
    if (!path.valid) {
        return 1e100;
    }
    return path.length;
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

static void emit_fixed_wing_segment(FILE *file,
                                    int *written,
                                    const char *kind,
                                    int block_id,
                                    int task_id,
                                    int sortie_id,
                                    SoPoint start,
                                    SoPoint end,
                                    const SoFieldTask *task,
                                    double from_heading,
                                    double to_heading,
                                    double turn_radius_m) {
    if (strcmp(kind, "transfer") == 0 && hypot(end.x - start.x, end.y - start.y) <= 1.0) {
        return;
    }
    fprintf(file,
            "%s    {\"kind\": \"%s\", \"block_id\": %d, \"task_id\": %d, "
            "\"sortie_id\": %d, \"route\": [",
            *written ? ",\n" : "",
            kind,
            block_id,
            task_id,
            sortie_id);
    emit_point(file, start);
    if (strcmp(kind, "transfer") == 0) {
        emit_radius_limited_transition(file, start, from_heading,
                                       end, to_heading, turn_radius_m);
    } else {
        if (task != NULL && task->has_route_mid && fabs(task->route_curve_deg) > 0.01) {
            fprintf(file, ", ");
            emit_point(file, task->route_mid);
        }
        fprintf(file, ", ");
        emit_point(file, end);
    }
    fprintf(file, "]}");
    (*written)++;
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
    double sortie_area = 0.0;
    int current_block_id = -1;
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
                const double transition_m =
                    dubins_transition_length(current, current_heading,
                                             candidate_start, heading, turn_radius);
                const double turn_m = visual_heading_diff_rad(current_heading, heading) * turn_radius;
                const double strip_m = hypot(candidate_end.x - candidate_start.x,
                                             candidate_end.y - candidate_start.y);
                const double score =
                    visual_fixed_wing_route_choice_score(sim, task, current_block_id,
                                                         transition_m, turn_m, strip_m);
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
            emit_radius_limited_transition(file, current, current_heading,
                                           sim->fixed_wing.airport, 0.0,
                                           turn_radius);
            current = sim->fixed_wing.airport;
            current_heading = 0.0;
            sortie_area = 0.0;
            current_block_id = -1;
            continue;
        }

        const SoFieldTask *task = &sim->field.tasks[best];
        const double sx = best_end.x - best_start.x;
        const double sy = best_end.y - best_start.y;
        const double route_angle = atan2(sy, sx);
        emit_radius_limited_transition(file, current, current_heading,
                                       best_start, route_angle, turn_radius);
        fprintf(file, ", ");
        emit_point(file, best_start);
        fprintf(file, ", ");
        if (task->has_route_mid && fabs(task->route_curve_deg) > 0.01) {
            emit_point(file, task->route_mid);
            fprintf(file, ", ");
        }
        emit_point(file, best_end);
        current = best_end;
        current_heading = best_heading;
        sortie_area += task->fixed_wing_area_ha;
        current_block_id = task->block_id;
        used[best] = true;
        remaining--;
        wrote_strip = 1;
    }
    if (wrote_strip) {
        emit_radius_limited_transition(file, current, current_heading,
                                       sim->fixed_wing.airport, 0.0,
                                       turn_radius);
    }
    fprintf(file, "]");
}

static void emit_fixed_wing_actual_routes(FILE *file, const SoSimulation *sim) {
    fprintf(file, "[\n");

    bool used[SO_MAX_TASKS] = {false};
    int remaining = 0;
    for (int i = 0; i < sim->field.task_count; i++) {
        if (task_fixed_wing_handled(sim, &sim->field.tasks[i])) {
            remaining++;
        }
    }

    SoPoint current = sim->fixed_wing.airport;
    double current_heading = 0.0;
    double sortie_area = 0.0;
    int current_block_id = -1;
    const double tank_area = fmax(1.0, sim->fixed_wing.tank_area_ha);
    const double turn_radius = fmax(1.0, sim->fixed_wing.turn_radius_m);
    int written = 0;
    int sortie_id = 1;
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
                const double transition_m =
                    dubins_transition_length(current, current_heading,
                                             candidate_start, heading, turn_radius);
                const double turn_m = visual_heading_diff_rad(current_heading, heading) * turn_radius;
                const double strip_m = hypot(candidate_end.x - candidate_start.x,
                                             candidate_end.y - candidate_start.y);
                const double score =
                    visual_fixed_wing_route_choice_score(sim, task, current_block_id,
                                                         transition_m, turn_m, strip_m);
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
            if (hypot(current.x - sim->fixed_wing.airport.x,
                      current.y - sim->fixed_wing.airport.y) > 1.0) {
                emit_fixed_wing_segment(file, &written, "transfer", -1, -1, sortie_id,
                                        current, sim->fixed_wing.airport, NULL,
                                        current_heading, 0.0, turn_radius);
            }
            current = sim->fixed_wing.airport;
            current_heading = 0.0;
            sortie_area = 0.0;
            current_block_id = -1;
            sortie_id++;
            continue;
        }

        const SoFieldTask *task = &sim->field.tasks[best];
        const double route_angle = atan2(best_end.y - best_start.y, best_end.x - best_start.x);
        emit_fixed_wing_segment(file, &written, "transfer", task->block_id, task->id, sortie_id,
                                current, best_start, NULL,
                                current_heading, route_angle, turn_radius);
        emit_fixed_wing_segment(file, &written, "spray", task->block_id, task->id, sortie_id,
                                best_start, best_end, task,
                                route_angle, route_angle, turn_radius);
        current = best_end;
        current_heading = best_heading;
        sortie_area += task->fixed_wing_area_ha;
        current_block_id = task->block_id;
        used[best] = true;
        remaining--;
    }
    if (written > 0 &&
        hypot(current.x - sim->fixed_wing.airport.x,
              current.y - sim->fixed_wing.airport.y) > 1.0) {
        emit_fixed_wing_segment(file, &written, "transfer", -1, -1, sortie_id,
                                current, sim->fixed_wing.airport, NULL,
                                current_heading, 0.0, turn_radius);
    }

    fprintf(file, "\n  ]");
}

static int emit_task_coverage_route(FILE *file,
                                    const SoSimulation *sim,
                                    const SoFieldTask *task,
                                    double swath,
                                    int max_lines) {
    if (task->area_ha <= 0.001) {
        return 0;
    }
    SoPoint start;
    SoPoint end;
    route_endpoints(sim, task, task_strip_length_m(sim, task), &start, &end);
    const double length = hypot(end.x - start.x, end.y - start.y);
    if (length <= 1.0) {
        fprintf(file,
                "[{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}]",
                start.x, start.y, end.x, end.y);
        return 1;
    }
    const double effective_swath = fmax(0.1, swath);
    if (task->kind != SO_TASK_INTERIOR_STRIP) {
        const SoFieldBlock *block = find_block(sim, task->block_id);
        if (block != NULL && block->boundary_count >= 3) {
            const double ux = (end.x - start.x) / length;
            const double uy = (end.y - start.y) / length;
            const double nx = -uy;
            const double ny = ux;
            const double angle_rad = atan2(uy, ux);
            const double base_cross = ((start.x + end.x) * 0.5) * nx +
                                      ((start.y + end.y) * 0.5) * ny;
            double min_cross = 0.0;
            double max_cross = 0.0;
            visual_block_projection_range(block, angle_rad, &min_cross, &max_cross);
            const double step = fabs(base_cross - max_cross) <= fabs(base_cross - min_cross)
                                    ? -effective_swath
                                    : effective_swath;
            const double target_area_m2 = task->area_ha * 10000.0;
            double covered_m2 = 0.0;
            SoPoint prev_end = {0.0, 0.0};
            bool wrote_any = false;
            int segment_count = 0;
            const int limit = max_lines > 0 ? max_lines : VISUAL_UAV_MAX_TRACK_PASSES;
            for (int i = 0; i < limit; i++) {
                const double cross = base_cross + step * (double)i;
                if (cross < min_cross - effective_swath || cross > max_cross + effective_swath) {
                    break;
                }
                double mins[SO_MAX_BOUNDARY_POINTS / 2];
                double maxs[SO_MAX_BOUNDARY_POINTS / 2];
                const int intervals =
                    visual_line_block_intervals(block, angle_rad, cross, mins, maxs,
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
                SoPoint a = {ux * (mins[best] + margin) + nx * cross,
                             uy * (mins[best] + margin) + ny * cross};
                SoPoint b = {ux * (maxs[best] - margin) + nx * cross,
                             uy * (maxs[best] - margin) + ny * cross};
                if (i % 2 == 1) {
                    const SoPoint tmp = a;
                    a = b;
                    b = tmp;
                }
                if (wrote_any) {
                    fprintf(file,
                            ", [{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}]",
                            prev_end.x, prev_end.y, a.x, a.y);
                    segment_count++;
                }
                fprintf(file,
                        "%s[{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}]",
                        wrote_any ? ", " : "",
                        a.x, a.y, b.x, b.y);
                prev_end = b;
                wrote_any = true;
                segment_count++;
                covered_m2 += best_len * effective_swath;
                if (covered_m2 >= target_area_m2 * 0.98) {
                    return segment_count;
                }
            }
            if (wrote_any) {
                return segment_count;
            }
        }
    }
    const double pass_route_length_m =
        visual_uav_effective_pass_route_length_m(task, task->area_ha, length);
    const double work_width_m =
        task->area_ha * 10000.0 / fmax(1.0, pass_route_length_m);
    int pass_count = visual_uav_pass_count_for_width(work_width_m, effective_swath);
    if (max_lines > 0 && pass_count > max_lines) {
        pass_count = max_lines;
    }
    if (pass_count <= 1 && task->has_route_mid && fabs(task->route_curve_deg) > 0.01) {
        fprintf(file,
                "[{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}], "
                "[{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}]",
                start.x, start.y,
                task->route_mid.x, task->route_mid.y,
                task->route_mid.x, task->route_mid.y,
                end.x, end.y);
        return 2;
    }
    if (pass_count > 1) {
        const double ux = (end.x - start.x) / length;
        const double uy = (end.y - start.y) / length;
        const double nx = -uy;
        const double ny = ux;
        const double angle_rad = atan2(uy, ux);
        const double base_cross = ((start.x + end.x) * 0.5) * nx +
                                  ((start.y + end.y) * 0.5) * ny;
        const double route_min_t = fmin(start.x * ux + start.y * uy,
                                        end.x * ux + end.y * uy);
        const double route_max_t = fmax(start.x * ux + start.y * uy,
                                        end.x * ux + end.y * uy);
        const SoFieldBlock *block = find_block(sim, task->block_id);
        SoPoint prev_end = {0.0, 0.0};
        bool wrote_any = false;
        int segment_count = 0;
        for (int i = 0; i < pass_count; i++) {
            const double offset = ((double)i - ((double)pass_count - 1.0) * 0.5) * effective_swath;
            SoPoint a = {start.x + nx * offset, start.y + ny * offset};
            SoPoint b = {end.x + nx * offset, end.y + ny * offset};
            bool marked_segment = false;
            if (block != NULL && block->boundary_count >= 3) {
                marked_segment =
                    visual_uav_marked_pass_segment(block, angle_rad, effective_swath,
                                                   base_cross, route_min_t, route_max_t,
                                                   i, pass_count, &a, &b);
                if (!marked_segment) {
                    continue;
                }
            }
            if (i % 2 == 1) {
                const SoPoint tmp = a;
                a = b;
                b = tmp;
            }
            if (!marked_segment && !clip_route_to_block_boundary(sim, task, &a, &b)) {
                continue;
            }
            if (wrote_any) {
                fprintf(file,
                        ", [{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}]",
                        prev_end.x, prev_end.y, a.x, a.y);
                segment_count++;
            }
            fprintf(file,
                    "%s[{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}]",
                    wrote_any ? ", " : "",
                    a.x, a.y, b.x, b.y);
            prev_end = b;
            wrote_any = true;
            segment_count++;
        }
        return segment_count;
    }
    fprintf(file,
            "[{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}]",
            start.x, start.y, end.x, end.y);
    return 1;
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

    fprintf(file, "{\n");
    fprintf(file, "  \"visual_plan_version\": 1,\n");
    fprintf(file, "  \"source\": \"scout_opt_c\",\n");
    fprintf(file, "  \"optimization_profile\": \"%s\",\n",
            so_optimization_profile_name(sim->optimization_profile));
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

    const double fixed_chemical_area_ha =
        sim->fixed_wing.enabled
            ? fmax(0.0, fmin(sim->fixed_wing.assigned_area_ha,
                              sim->fixed_wing.completed_area_ha))
            : 0.0;
    const double fixed_chemical_cost_usd =
        fixed_chemical_area_ha *
        sim->fixed_wing.chemical_l_per_ha *
        sim->fixed_wing.chemical_cost_usd_per_l;
    const double uav_chemical_area_ha =
        fmax(0.0, fmin(sim->field.area_ha, sim->field.treated_ha) -
                       fixed_chemical_area_ha);
    const double uav_chemical_cost_usd =
        uav_chemical_area_ha *
        sim->spec.chemical_l_per_ha *
        sim->spec.chemical_cost_usd_per_l;
    const double uav_total_with_chemical =
        sim->uav_flight_cost_usd +
        sim->uav_launch_cost_usd +
        sim->uav_electricity_cost_usd +
        uav_chemical_cost_usd;

    fprintf(file, "  \"drones\": {\"count\": %d, \"model\": \"dji_agras_t200_style_abstraction\", \"modeled_payload_capacity_kg\": %.1f, \"manufacturer_validated_digital_twin\": false, \"rtk\": true, \"obstacle_sensing\": \"omnidirectional\", \"cruise_speed_mps\": %.3f, \"spray_speed_mps\": %.3f, \"spray_swath_m\": %.3f, \"spray_radius_m\": %.3f, \"spray_rate_ha_h\": %.3f, \"turn_time_s\": %.3f, \"turn_battery_cost\": %.5f, \"turn_radius_m\": %.3f, \"chemical_l_per_ha\": %.3f, \"chemical_cost_usd_per_l\": %.3f, \"chemical_area_ha\": %.6f, \"chemical_cost_usd\": %.6f, \"chemical_tank_l\": %.3f, \"chemical_tank_area_ha\": %.3f, \"selected_payload_kg\": %.3f, \"battery_modules\": %d, \"battery_module_capacity_kwh\": %.3f, \"battery_module_weight_kg\": %.3f, \"battery_capacity_kwh\": %.3f, \"work_power_kw\": %.3f, \"fast_chargers\": %d, \"battery_slots_per_charger\": %d, \"charger_battery_slots\": %d, \"single_battery_full_charge_min\": 7.5, \"parallel_two_battery_full_charge_min\": 12.5, \"fast_charger_power_kw\": %.3f, \"electricity_price_usd_per_kwh\": %.3f, \"battery_depreciation_included\": false, \"energy_used_battery_units\": %.6f, \"energy_used_kwh\": %.6f, \"electricity_cost_usd\": %.6f, \"unfinished_penalty_usd_per_ha\": %.3f, \"flight_cost_usd_per_km\": %.3f, \"launch_cost_usd\": %.3f, \"flight_distance_m\": %.3f, \"flight_cost_usd\": %.3f, \"takeoffs\": %d, \"launch_cost_total_usd\": %.3f, \"total_cost_usd\": %.3f, \"sortie_battery_options\": {\"allowed_modules\": [1, 2, 4], \"sorties_1\": %d, \"sorties_2\": %d, \"sorties_4\": %d, \"area_ha_1\": %.6f, \"area_ha_2\": %.6f, \"area_ha_4\": %.6f}, \"altitude_m\": 18.0},\n",
            sim->drone_count,
            sim->spec.modeled_payload_capacity_kg,
            sim->spec.cruise_speed_mps, sim->spec.cruise_speed_mps * 0.45,
            sim->spec.spray_swath_m, sim->spec.spray_radius_m, sim->spec.spray_rate_ha_h,
            sim->spec.turn_time_s, sim->spec.turn_battery_cost,
            sim->spec.turn_radius_m,
            sim->spec.chemical_l_per_ha,
            sim->spec.chemical_cost_usd_per_l,
            uav_chemical_area_ha,
            uav_chemical_cost_usd,
            sim->spec.chemical_tank_l,
            sim->spec.chemical_tank_area_ha,
            sim->spec.selected_payload_kg,
            sim->spec.battery_modules,
            sim->spec.battery_module_capacity_kwh,
            sim->spec.battery_module_weight_kg,
            sim->spec.battery_capacity_kwh,
            sim->spec.work_power_kw,
            sim->mothership.fast_chargers,
            SO_BATTERY_SLOTS_PER_CHARGER,
            sim->mothership.fast_chargers * SO_BATTERY_SLOTS_PER_CHARGER,
            sim->spec.fast_charger_power_kw,
            sim->spec.electricity_price_usd_per_kwh,
            sim->uav_energy_used_battery_units,
            sim->uav_energy_used_kwh,
            sim->uav_electricity_cost_usd,
            sim->spec.unfinished_penalty_usd_per_ha,
            sim->spec.flight_cost_usd_per_km, sim->spec.launch_cost_usd,
            sim->uav_flight_distance_m, sim->uav_flight_cost_usd,
            sim->uav_takeoffs, sim->uav_launch_cost_usd,
            uav_total_with_chemical,
            sim->uav_sorties_by_battery_modules[1],
            sim->uav_sorties_by_battery_modules[2],
            sim->uav_sorties_by_battery_modules[4],
            sim->uav_area_by_battery_modules[1],
            sim->uav_area_by_battery_modules[2],
            sim->uav_area_by_battery_modules[4]);

    fprintf(file,
            "  \"chemical_application_model\": {\"effective_required_l_per_ha\": %.3f, "
            "\"uav_deposition_efficiency\": %.3f, \"uav_applied_l_per_ha\": %.3f, "
            "\"uav_tank_l\": %.3f, \"uav_tank_area_ha\": %.3f, "
            "\"fixed_wing_deposition_efficiency\": %.3f, \"fixed_wing_applied_l_per_ha\": %.3f, "
            "\"fixed_wing_tank_l\": %.3f, \"fixed_wing_tank_area_ha\": %.3f},\n",
            sim->spec.effective_chemical_l_per_ha,
            sim->spec.deposition_efficiency,
            sim->spec.chemical_l_per_ha,
            sim->spec.chemical_tank_l,
            sim->spec.chemical_tank_area_ha,
            sim->fixed_wing.deposition_efficiency,
            sim->fixed_wing.chemical_l_per_ha,
            sim->fixed_wing.tank_l,
            sim->fixed_wing.tank_area_ha);

    const double fixed_work_h =
        sim->fixed_wing.corridor_work_m /
        fmax(0.001, sim->fixed_wing.work_speed_mps) / 3600.0;
    const double fixed_empty_h =
        sim->fixed_wing.corridor_empty_m /
        fmax(0.001, sim->fixed_wing.cruise_speed_mps) / 3600.0;
    const double fixed_turn_h =
        sim->fixed_wing.planned_turn_non_spray_time_s / 3600.0 *
        fmax(1.0, (double)sim->fixed_wing.aircraft_count);
    const double fixed_fuel_used_l =
        (fixed_work_h + fixed_empty_h + fixed_turn_h) *
        sim->fixed_wing.fuel_burn_l_per_h;
    const double fixed_fuel_cost_usd =
        fixed_fuel_used_l * sim->fixed_wing.fuel_price_usd_per_l;
    const double fixed_work_flight_cost_usd =
        sim->fixed_wing.corridor_work_m / 1000.0 *
        sim->fixed_wing.flight_cost_usd_per_km;
    const double fixed_empty_flight_cost_usd =
        sim->fixed_wing.corridor_empty_m / 1000.0 *
        sim->fixed_wing.flight_cost_usd_per_km;
    const double fixed_turn_flight_cost_usd =
        sim->fixed_wing.turn_radius_m > 0.0
            ? sim->fixed_wing.planned_turn_non_spray_time_s *
                  sim->fixed_wing.work_speed_mps / 1000.0 *
                  sim->fixed_wing.flight_cost_usd_per_km
            : 0.0;
    const double fixed_total_with_fuel =
        sim->fixed_wing.flight_cost_usd +
        sim->fixed_wing.airport_cost_usd +
        fixed_fuel_cost_usd +
        fixed_chemical_cost_usd;

    fprintf(file, "  \"fixed_wing\": {\"enabled\": %s, \"count\": %d, \"model\": \"%s\", \"engine\": \"Pratt & Whitney PT6A-34AG\", \"power_shp\": 750.0, \"payload_kg\": %.1f, \"tank_l\": %.1f, \"fuel_l\": %.1f, \"tank_area_ha\": %.3f, \"fuel_endurance_h\": %.3f, \"energy_coverage_ha\": %.3f, \"cruise_speed_mps\": %.3f, \"work_speed_mps\": %.3f, \"swath_m\": %.3f, \"spray_productivity_ha_h_per_aircraft\": %.3f, \"turn_time_s\": %.3f, \"turn_fuel_h\": %.6f, \"turn_radius_m\": %.3f, \"planned_turn_non_spray_time_s\": %.3f, \"turn_non_spray_time_s\": %.3f, \"turn_spraying_allowed\": false, \"chemical_l_per_ha\": %.3f, \"chemical_cost_usd_per_l\": %.3f, \"chemical_area_ha\": %.6f, \"chemical_cost_usd\": %.6f, \"fuel_burn_l_per_h\": %.3f, \"fuel_price_usd_per_l\": %.3f, \"fuel_cost_usd_per_h\": %.3f, \"energy_cost_usd_per_ha\": %.6f, \"fuel_used_l\": %.6f, \"fuel_cost_actual_usd\": %.6f, \"unfinished_penalty_usd_per_ha\": %.3f, \"planned_turns\": %d, \"corridor_count\": %d, \"corridor_work_m\": %.3f, \"corridor_empty_m\": %.3f, \"corridor_total_m\": %.3f, \"flight_cost_usd_per_km\": %.3f, \"takeoff_cost_usd\": %.3f, \"airport_service_cost_usd\": %.3f, \"flight_distance_m\": %.3f, \"flight_cost_usd\": %.3f, \"airport_cost_usd\": %.3f, \"total_cost_usd\": %.3f, \"economic_cost_h\": %.3f, \"airport\": {\"x\": %.3f, \"y\": %.3f}, \"return_point\": {\"x\": %.3f, \"y\": %.3f}, \"altitude_m\": 55.0},\n",
            sim->fixed_wing.enabled ? "true" : "false",
            sim->fixed_wing.aircraft_count,
            sim->fixed_wing.model_name[0] ? sim->fixed_wing.model_name : "none",
            sim->fixed_wing.payload_kg > 1.0 ? sim->fixed_wing.payload_kg : 2450.0,
            sim->fixed_wing.tank_l > 1.0 ? sim->fixed_wing.tank_l : 1893.0,
            sim->fixed_wing.fuel_l > 1.0 ? sim->fixed_wing.fuel_l : 644.0,
            sim->fixed_wing.tank_area_ha > 0.1 ? sim->fixed_wing.tank_area_ha : 189.3,
            sim->fixed_wing.fuel_endurance_h > 0.1 ? sim->fixed_wing.fuel_endurance_h : 3.2,
            sim->fixed_wing.fuel_endurance_h *
                sim->fixed_wing.swath_width_m *
                sim->fixed_wing.work_speed_mps *
                sim->fixed_wing.spray_efficiency * 3600.0 / 10000.0,
            sim->fixed_wing.cruise_speed_mps > 1.0 ? sim->fixed_wing.cruise_speed_mps : 42.0,
            sim->fixed_wing.work_speed_mps > 1.0 ? sim->fixed_wing.work_speed_mps : 59.0,
            sim->fixed_wing.swath_width_m > 1.0 ? sim->fixed_wing.swath_width_m : 36.0,
            sim->fixed_wing.swath_width_m *
                sim->fixed_wing.work_speed_mps *
                sim->fixed_wing.spray_efficiency * 3600.0 / 10000.0,
            sim->fixed_wing.turn_time_s,
            sim->fixed_wing.turn_fuel_h,
            sim->fixed_wing.turn_radius_m,
            sim->fixed_wing.planned_turn_non_spray_time_s,
            sim->fixed_wing.turn_non_spray_time_s,
            sim->fixed_wing.chemical_l_per_ha,
            sim->fixed_wing.chemical_cost_usd_per_l,
            fixed_chemical_area_ha,
            fixed_chemical_cost_usd,
            sim->fixed_wing.fuel_burn_l_per_h,
            sim->fixed_wing.fuel_price_usd_per_l,
            sim->fixed_wing.fuel_cost_usd_per_h,
            sim->fixed_wing.fuel_cost_usd_per_h /
                fmax(0.001,
                     sim->fixed_wing.swath_width_m *
                         sim->fixed_wing.work_speed_mps *
                         sim->fixed_wing.spray_efficiency * 3600.0 / 10000.0),
            fixed_fuel_used_l,
            fixed_fuel_cost_usd,
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
            fixed_total_with_fuel,
            sim->fixed_wing.economic_cost_h,
            sim->fixed_wing.airport.x,
            sim->fixed_wing.airport.y,
            sim->fixed_wing.airport.x,
            sim->fixed_wing.airport.y);

    fprintf(file,
            "  \"fixed_wing_path_strategy\": {\"selected\": \"%s\", \"selected_score\": %.6f, "
            "\"candidates\": ["
            "{\"name\": \"multi_island\", \"estimated_time_h\": %.6f, \"estimated_cost_usd\": %.6f, \"score\": %.6f}, "
            "{\"name\": \"partition_dp\", \"estimated_time_h\": %.6f, \"estimated_cost_usd\": %.6f, \"score\": %.6f}, "
            "{\"name\": \"longest_corridor\", \"estimated_time_h\": %.6f, \"estimated_cost_usd\": %.6f, \"score\": %.6f}"
            "]},\n",
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

    const double uav_total_cost = uav_total_with_chemical;
    const double fixed_wing_total_cost = fixed_total_with_fuel;
    const double hive_total_cost = sim->mothership.move_cost_usd + sim->mothership.stop_cost_usd;
    const double uncovered_ha =
        fmax(0.0, sim->field.area_ha - fmin(sim->field.area_ha, sim->field.treated_ha));
    const double uncovered_ratio =
        sim->field.area_ha > 0.001 ? uncovered_ha / sim->field.area_ha : 0.0;
    fprintf(file,
            "  \"coverage_policy\": {\"task_tolerance_ratio\": %.6f, "
            "\"final_error_limit_ratio\": %.6f, \"repair_cost_limit_ratio\": %.6f, "
            "\"treated_area_ha\": %.6f, \"target_area_ha\": %.6f, "
            "\"uncovered_area_ha\": %.6f, \"uncovered_ratio\": %.6f, "
            "\"repair_attempted\": %s, \"repair_performed\": %s, "
            "\"repair_area_ha\": %.6f, \"repair_cost_usd\": %.6f, "
            "\"repair_cost_ratio\": %.6f},\n",
            sim->coverage_task_tolerance_ratio,
            sim->coverage_final_error_limit_ratio,
            sim->coverage_repair_cost_limit_ratio,
            fmin(sim->field.area_ha, sim->field.treated_ha),
            sim->field.area_ha,
            uncovered_ha,
            uncovered_ratio,
            sim->final_repair_attempted ? "true" : "false",
            sim->final_repair_performed ? "true" : "false",
            sim->final_repair_area_ha,
            sim->final_repair_cost_usd,
            sim->final_repair_cost_ratio);
    fprintf(file,
            "  \"cost_breakdown\": {\"currency\": \"USD\", "
            "\"uav\": {\"chemical_usd\": %.6f, \"electricity_usd\": %.6f, "
            "\"flight_usd\": %.6f, \"launch_usd\": %.6f, \"total_usd\": %.6f}, "
            "\"fixed_wing\": {\"chemical_usd\": %.6f, \"fuel_usd\": %.6f, "
            "\"airport_usd\": %.6f, \"route_flight_usd\": %.6f, "
            "\"work_flight_usd_est\": %.6f, \"empty_flight_usd_est\": %.6f, "
            "\"turn_flight_usd_est\": %.6f, \"total_usd\": %.6f}, "
            "\"hive\": {\"move_usd\": %.6f, \"stop_usd\": %.6f, \"total_usd\": %.6f}},\n",
            uav_chemical_cost_usd,
            sim->uav_electricity_cost_usd,
            sim->uav_flight_cost_usd,
            sim->uav_launch_cost_usd,
            uav_total_cost,
            fixed_chemical_cost_usd,
            fixed_fuel_cost_usd,
            sim->fixed_wing.airport_cost_usd,
            sim->fixed_wing.flight_cost_usd,
            fixed_work_flight_cost_usd,
            fixed_empty_flight_cost_usd,
            fixed_turn_flight_cost_usd,
            fixed_wing_total_cost,
            sim->mothership.move_cost_usd,
            sim->mothership.stop_cost_usd,
            hive_total_cost);
    fprintf(file,
            "  \"cost_summary\": {\"currency\": \"USD\", \"uav_total_usd\": %.3f, "
            "\"fixed_wing_total_usd\": %.3f, \"hive_total_usd\": %.3f, "
            "\"mission_total_usd\": %.3f, "
            "\"uav_electricity_usd\": %.6f, \"uav_chemical_usd\": %.6f, "
            "\"fixed_wing_fuel_usd\": %.6f, \"fixed_wing_chemical_usd\": %.6f, "
            "\"model\": \"C_total=C_coverage+C_electricity+C_fuel+C_turn+C_empty+C_hive+C_risk+C_unfinished\", "
            "\"notes\": \"UAV and fixed-wing chemical costs are included using platform-specific deposition efficiency. UAV electricity uses the selected T200 battery module configuration, with fast charging limited by charger count and charger power; battery depreciation is excluded. Fixed-wing fuel uses 205 L/h at 1.03 USD/L.\"},\n",
            uav_total_cost,
            fixed_wing_total_cost,
            hive_total_cost,
            uav_total_cost + fixed_wing_total_cost + hive_total_cost,
            sim->uav_electricity_cost_usd,
            uav_chemical_cost_usd,
            fixed_fuel_cost_usd,
            fixed_chemical_cost_usd);

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
                "\"strip_angle_deg\": %.3f, \"route_curve_deg\": %.3f, "
                "\"spray_heading_change_limit_deg\": 3.000, "
                "\"turn_count\": %d, \"turn_time_s\": %.3f, "
                "\"center\": {\"x\": %.3f, \"y\": %.3f}, "
                "\"route\": ",
                fixed_route_written ? "," : "",
                task->id, task->block_id, task->fixed_wing_area_ha,
                task->strip_angle_deg,
                task->route_curve_deg,
                task->turn_count,
                (double)task->turn_count * sim->fixed_wing.turn_time_s,
                task->center.x, task->center.y);
        emit_route_points(file, task, start, end);
        fprintf(file, "}\n");
        fixed_route_written++;
    }
    fprintf(file, "  ],\n");

    fprintf(file, "  \"fixed_wing_trajectory\": ");
    emit_fixed_wing_mission_trajectory(file, sim);
    fprintf(file, ",\n");

    fprintf(file, "  \"fixed_wing_actual_routes\": ");
    emit_fixed_wing_actual_routes(file, sim);
    fprintf(file, ",\n");

    fprintf(file, "  \"uav_actual_routes\": [\n");
    int uav_route_written = 0;
    for (int d = 0; d < sim->drone_count; d++) {
        const SoDrone *drone = &sim->drones[d];
        if (drone->route_point_count < 2 || drone->route_segment_count <= 0) {
            continue;
        }
        fprintf(file, "%s    {\"drone_id\": %d, \"route\": [",
                uav_route_written ? ",\n" : "",
                drone->id);
        for (int p = 0; p < drone->route_point_count; p++) {
            fprintf(file, "%s{\"x\": %.3f, \"y\": %.3f}",
                    p == 0 ? "" : ", ",
                    drone->route_points[p].x,
                    drone->route_points[p].y);
        }
        fprintf(file, "], \"segments\": [");
        int segment_written = 0;
        for (int s = 0; s < drone->route_segment_count; s++) {
            const int start_idx = drone->route_segment_start[s];
            const int point_count = drone->route_segment_point_count[s];
            if (point_count < 2 || start_idx < 0 ||
                start_idx + point_count > drone->route_point_count) {
                continue;
            }
            fprintf(file,
                    "%s{\"kind\": \"%s\", \"block_id\": %d, \"task_id\": %d, \"route\": [",
                    segment_written ? ", " : "",
                    drone->route_segment_kind[s] == SO_DRONE_ROUTE_SPRAY ? "spray" : "transfer",
                    drone->route_segment_block_id[s],
                    drone->route_segment_task_id[s]);
            for (int p = 0; p < point_count; p++) {
                const SoPoint point = drone->route_points[start_idx + p];
                fprintf(file, "%s{\"x\": %.3f, \"y\": %.3f}",
                        p == 0 ? "" : ", ",
                        point.x,
                        point.y);
            }
            fprintf(file, "]}");
            segment_written++;
        }
        fprintf(file, "]}");
        uav_route_written++;
    }
    fprintf(file, "\n  ],\n");

    fprintf(file, "  \"tasks\": [\n");
    int task_written = 0;
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *task = &sim->field.tasks[i];
        if (task->area_ha <= 0.001 || task_fixed_wing_handled(sim, task)) {
            continue;
        }
        SoPoint start;
        SoPoint end;
        route_endpoints(sim, task, task_strip_length_m(sim, task), &start, &end);
        fprintf(file,
                "%s    {\"id\": %d, \"block_id\": %d, \"kind\": \"%s\", \"handling\": \"%s\", "
                "\"assigned_drone_id\": %d, \"area_ha\": %.6f, \"strip_angle_deg\": %.3f, "
                "\"route_curve_deg\": %.3f, \"spray_heading_change_limit_deg\": 0.000, "
                "\"uav_turn_model\": \"spraying_lateral_or_diagonal_track_change\", "
                "\"turn_count\": %d, \"turn_time_s\": %.3f, \"turn_energy_cost\": %.5f, "
                "\"center\": {\"x\": %.3f, \"y\": %.3f}, "
                "\"route\": ",
                task_written ? "," : "",
                task->id, task->block_id, task_kind_name(task->kind),
                "drone",
                task->assigned_drone_id,
                task->area_ha,
                task->strip_angle_deg,
                task->route_curve_deg,
                task->turn_count,
                task->turn_time_s,
                task->turn_energy_cost,
                task->center.x, task->center.y);
        emit_route_points(file, task, start, end);
        fprintf(file, ", \"coverage_route\": [");
        emit_task_coverage_route(file, sim, task, sim->spec.spray_swath_m,
                                 VISUAL_UAV_MAX_TRACK_PASSES);
        fprintf(file, "]}\n");
        task_written++;
    }
    fprintf(file, "  ]\n");
    fprintf(file, "}\n");

    fclose(file);
    return true;
}

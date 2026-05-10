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
    if (sim->fixed_wing.enabled && sim->fixed_wing.swath_width_m > 1.0 &&
        task->kind == SO_TASK_INTERIOR_STRIP) {
        swath = sim->fixed_wing.swath_width_m;
    }
    return task->area_ha * 10000.0 / swath;
}

static bool task_fixed_wing_handled(const SoSimulation *sim, const SoFieldTask *task) {
    if (!sim->fixed_wing.enabled || sim->fixed_wing.aircraft_count <= 0) {
        return false;
    }
    if (task->kind != SO_TASK_INTERIOR_STRIP) {
        return false;
    }
    const double length_m = task_strip_length_m(sim, task);
    return length_m >= 260.0 && task->area_ha >= 0.70;
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
                            SoPoint *start,
                            SoPoint *end) {
    const double length = task_strip_length_m(sim, task);
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
        if (task_fixed_wing_handled(sim, task)) {
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
    fprintf(file, "  \"origin\": {\"lat\": 32.085894448486876, \"lon\": 118.89885869105146},\n");
    fprintf(file, "  \"hive\": {\n");
    fprintf(file, "    \"speed_kmh\": %.3f,\n", sim->mothership.move_speed_mps * 3.6);
    fprintf(file, "    \"can_move_while_service_busy\": false,\n");
    fprintf(file, "    \"move_policy\": \"cleanup_only_max_2_drones\",\n");
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

    fprintf(file, "  \"drones\": {\"count\": %d, \"model\": \"dji_agras_t200\", \"payload_kg\": 200.0, \"rtk\": true, \"obstacle_sensing\": \"omnidirectional\", \"cruise_speed_mps\": %.3f, \"spray_speed_mps\": %.3f, \"spray_swath_m\": %.3f, \"spray_radius_m\": %.3f, \"spray_rate_ha_h\": %.3f, \"altitude_m\": 18.0},\n",
            sim->drone_count, sim->spec.cruise_speed_mps, sim->spec.cruise_speed_mps * 0.45,
            sim->spec.spray_swath_m, sim->spec.spray_radius_m, sim->spec.spray_rate_ha_h);

    fprintf(file, "  \"fixed_wing\": {\"enabled\": %s, \"count\": %d, \"model\": \"%s\", \"engine\": \"Pratt & Whitney PT6A-34AG\", \"power_shp\": 750.0, \"payload_kg\": %.1f, \"tank_l\": %.1f, \"fuel_l\": %.1f, \"tank_area_ha\": %.3f, \"fuel_endurance_h\": %.3f, \"cruise_speed_mps\": %.3f, \"work_speed_mps\": %.3f, \"swath_m\": %.3f, \"airport\": {\"x\": %.3f, \"y\": %.3f}, \"return_point\": {\"x\": %.3f, \"y\": %.3f}, \"altitude_m\": 55.0},\n",
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
            sim->fixed_wing.airport.x,
            sim->fixed_wing.airport.y,
            sim->fixed_wing.airport.x,
            sim->fixed_wing.airport.y);

    fprintf(file, "  \"scout_routes\": [\n");
    int scout_written = 0;
    for (int i = 0; i < sim->field.block_count && i < 3; i++) {
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

    fprintf(file, "  \"tasks\": [\n");
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *task = &sim->field.tasks[i];
        SoPoint start;
        SoPoint end;
        route_endpoints(sim, task, &start, &end);
        const bool fixed = task_fixed_wing_handled(sim, task);
        fprintf(file,
                "    {\"id\": %d, \"block_id\": %d, \"kind\": \"%s\", \"handling\": \"%s\", "
                "\"assigned_drone_id\": %d, \"area_ha\": %.6f, \"strip_angle_deg\": %.3f, "
                "\"center\": {\"x\": %.3f, \"y\": %.3f}, "
                "\"route\": [{\"x\": %.3f, \"y\": %.3f}, {\"x\": %.3f, \"y\": %.3f}]}%s\n",
                task->id, task->block_id, task_kind_name(task->kind),
                fixed ? "fixed_wing" : "drone",
                fixed ? -1 : visual_drone_owner[i],
                task->area_ha,
                task->strip_angle_deg,
                task->center.x, task->center.y,
                start.x, start.y, end.x, end.y,
                i + 1 == sim->field.task_count ? "" : ",");
    }
    fprintf(file, "  ]\n");
    fprintf(file, "}\n");

    fclose(file);
    return true;
}

#include "scout_opt.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double so_distance(SoPoint a, SoPoint b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

static bool so_mothership_service_busy(const SoSimulation *sim);
static int so_cleanup_open_near(const SoSimulation *sim, SoPoint point, double radius);
static int so_active_field_drone_count(const SoSimulation *sim);

static double so_angle_diff_rad(double a, double b) {
    double diff = fmod(fabs(a - b), M_PI);
    if (diff > M_PI / 2.0) {
        diff = M_PI - diff;
    }
    return diff;
}

static SoPoint so_point(double x, double y) {
    SoPoint p;
    p.x = x;
    p.y = y;
    return p;
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
    return so_distance(drone->position, depot) / 1000.0 * spec.battery_drain_km_empty;
}

static double so_dynamic_capacity(SoDrone *drone, SoPoint depot, SoDroneSpec spec) {
    const double return_energy = so_estimate_return_energy(drone, depot, spec);
    const double available_battery = fmax(0.0, drone->battery - return_energy - spec.safety_battery_margin);
    const double battery_area = available_battery / spec.battery_drain_h_work * spec.spray_rate_ha_h;
    const double chemical_area = drone->chemical / spec.chemical_per_ha;
    const double capacity = fmax(0.0, fmin(battery_area, chemical_area));
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
        SoDrone drone = sim->drones[i];
        if (drone.battery > 0.25 && drone.chemical > 0.1) {
            sum += so_dynamic_capacity(&drone, sim->mothership.position, sim->spec);
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

static double so_fixed_wing_suitability(const SoSimulation *sim, const SoFieldBlock *block) {
    if (block == NULL || block->area_ha < 10.0 || block->risk > 0.78 ||
        sim->field.obstacle_density > 0.72 || sim->field.terrain_complexity > 0.76) {
        return 0.0;
    }

    double score = 0.86 - block->risk * 0.30 -
                   sim->field.obstacle_density * 0.24 -
                   sim->field.terrain_complexity * 0.18;
    if (block->area_ha >= 22.0) {
        score += 0.08;
    } else if (block->area_ha < 16.0) {
        score -= 0.18;
    }
    return fmax(0.0, fmin(0.88, score));
}

static void so_select_fixed_wing_fleet(SoSimulation *sim, double eligible_area_ha, double weighted_round_trip_m) {
    if (!sim->fixed_wing.enabled || eligible_area_ha < 28.0) {
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
    } AircraftOption;

    const AircraftOption options[] = {
        {"light_fixed_wing", 15.0, 38.0, 38.0, 0.50, 18.0 * 60.0, 36.0, 720.0, 300.0, 1200.0, 2.2, 10.0 * 60.0, 8.0 * 60.0, 0.45},
        {"air_tractor_at_502b", 22.0, 68.9, 59.0, 0.54, 18.0 * 60.0, 189.3, 1893.0, 644.0, 2450.0, 3.2, 13.0 * 60.0, 10.0 * 60.0, 0.70},
        {"large_fixed_wing", 27.0, 50.0, 50.0, 0.56, 28.0 * 60.0, 86.0, 1700.0, 700.0, 2800.0, 3.0, 17.0 * 60.0, 12.0 * 60.0, 1.05},
    };

    double best_score = 1e100;
    int best_option = 0;
    int best_count = 1;
    for (int o = 0; o < 3; o++) {
        const double rate = options[o].swath_m * options[o].work_speed_mps * options[o].efficiency * 3600.0 / 10000.0;
        const int max_count = eligible_area_ha > 220.0 ? 2 : 1;
        for (int count = 1; count <= max_count; count++) {
            const double sortie_area_by_fuel = options[o].endurance_h * 0.82 * rate;
            const double sortie_area = fmin(options[o].tank_ha, sortie_area_by_fuel);
            const double sorties = ceil(eligible_area_ha / fmax(0.001, sortie_area * count));
            const double spray_h = eligible_area_ha / fmax(0.001, rate * count);
            const double service_h = fmax(0.0, sorties - 1.0) * options[o].turnaround_s / 3600.0;
            const double setup_h = options[o].setup_s / 3600.0;
            const double route_h = weighted_round_trip_m / fmax(0.001, options[o].speed_mps) / 3600.0;
            const double economic_penalty = options[o].cost_h * count;
            const double idle_penalty = count > 1 && eligible_area_ha < 360.0 ? 0.35 : 0.0;
            const double score = spray_h + service_h + setup_h + route_h + economic_penalty + idle_penalty;
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
    sim->fixed_wing.tank_area_ha = selected->tank_ha;
    sim->fixed_wing.tank_l = selected->tank_l;
    sim->fixed_wing.fuel_l = selected->fuel_l;
    sim->fixed_wing.payload_kg = selected->payload_kg;
    sim->fixed_wing.fuel_endurance_h = selected->endurance_h;
    sim->fixed_wing.ferry_time_s = selected->ferry_s;
    sim->fixed_wing.sortie_remaining_ha = 0.0;
    sim->fixed_wing.fuel_remaining_h = 0.0;
    sim->fixed_wing.service_remaining_s = 0.0;
    sim->fixed_wing.sorties_completed = 0;
    sim->fixed_wing.economic_cost_h = selected->cost_h * best_count;
    sim->fixed_wing.spray_rate_ha_h =
        selected->swath_m * selected->work_speed_mps * selected->efficiency * 3600.0 / 10000.0 * best_count;
}

static void so_plan_fixed_wing_coverage(SoSimulation *sim) {
    if (!sim->fixed_wing.enabled || sim->fixed_wing.planned) {
        return;
    }

    double eligible_area = 0.0;
    double weighted_round_trip_m = 0.0;
    double covered_area = 0.0;
    for (int i = 0; i < sim->field.task_count; i++) {
        SoFieldTask *task = &sim->field.tasks[i];
        if (task->kind != SO_TASK_INTERIOR_STRIP || task->status == SO_TASK_DONE) {
            continue;
        }
        const SoFieldBlock *block = so_find_block_const(sim, task->block_id);
        const double suitability = so_fixed_wing_suitability(sim, block);
        if (suitability <= 0.0) {
            continue;
        }
        const double fixed_area = task->remaining_ha * suitability;
        if (fixed_area >= 0.25) {
            eligible_area += fixed_area;
            weighted_round_trip_m += fixed_area * so_distance(sim->fixed_wing.airport, task->center) * 2.0;
        }
    }

    so_select_fixed_wing_fleet(sim, eligible_area,
                               eligible_area > 0.001 ? weighted_round_trip_m / eligible_area : 0.0);
    if (sim->fixed_wing.aircraft_count <= 0) {
        sim->fixed_wing.planned = true;
        return;
    }

    for (int i = 0; i < sim->field.task_count; i++) {
        SoFieldTask *task = &sim->field.tasks[i];
        if (task->kind != SO_TASK_INTERIOR_STRIP || task->status == SO_TASK_DONE) {
            continue;
        }
        const SoFieldBlock *block = so_find_block_const(sim, task->block_id);
        const double suitability = so_fixed_wing_suitability(sim, block);
        if (suitability <= 0.0) {
            continue;
        }

        const double fixed_area = task->remaining_ha * suitability;
        const double drone_repair_area = task->remaining_ha - fixed_area;
        if (fixed_area < 0.25) {
            continue;
        }
        covered_area += fixed_area;

        task->remaining_ha = drone_repair_area;
        task->area_ha = drone_repair_area;
        task->priority += drone_repair_area <= so_repair_threshold_ha(sim) ? 0.35 : 0.15;
        task->route_efficiency = fmin(task->route_efficiency, drone_repair_area <= so_repair_threshold_ha(sim) ? 0.92 : 0.96);
        if (drone_repair_area <= so_repair_threshold_ha(sim)) {
            task->kind = SO_TASK_REPAIR;
        }
        if (task->remaining_ha <= 0.001) {
            task->status = SO_TASK_DONE;
        }
    }

    sim->fixed_wing.assigned_area_ha = covered_area;
    if (covered_area > 0.001) {
        sim->fixed_wing.ferry_time_s =
            fmax(sim->fixed_wing.ferry_time_s,
                 (weighted_round_trip_m / fmax(0.001, eligible_area)) /
                     fmax(0.001, sim->fixed_wing.cruise_speed_mps));
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
    const double wind_penalty = fmin(0.22, fmax(0.0, sim->mothership.weather.wind_speed_mps - 4.0) * 0.035);
    if (sim->fixed_wing.sortie_remaining_ha <= 0.001 || sim->fixed_wing.fuel_remaining_h <= 0.001) {
        sim->fixed_wing.sortie_remaining_ha = sim->fixed_wing.tank_area_ha * sim->fixed_wing.aircraft_count;
        sim->fixed_wing.fuel_remaining_h = sim->fixed_wing.fuel_endurance_h;
        sim->fixed_wing.service_remaining_s = sim->fixed_wing.sorties_completed == 0
                                                  ? sim->fixed_wing.ferry_time_s
                                                  : sim->fixed_wing.turnaround_time_s;
        sim->fixed_wing.sorties_completed++;
        return;
    }

    const double possible_by_rate = sim->fixed_wing.spray_rate_ha_h * weather.spray_effectiveness *
                                    (1.0 - wind_penalty) * dt_h;
    const double possible_by_fuel = fmax(0.0, sim->fixed_wing.fuel_remaining_h) *
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

        const double capacity = so_dynamic_capacity(drone, sim->mothership.position, sim->spec);
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
        double phase_penalty = 0.0;
        if (so_zone_has_open_interior(sim, task->zone_id)) {
            if (task->kind == SO_TASK_BOUNDARY) {
                phase_penalty += 36.0;
            } else if (task->kind == SO_TASK_REPAIR) {
                phase_penalty += 54.0;
            }
        } else if (task->kind == SO_TASK_REPAIR) {
            phase_penalty += 12.0;
        }
        const double strip_bonus = task->kind == SO_TASK_INTERIOR_STRIP ? 16.0 * task->route_efficiency : 0.0;
        const double score = empty_dist / 12.0 + return_dist / 14.0 + over_capacity + underuse +
                             task->risk * 45.0 + queue_pressure + radius_penalty + push_penalty + phase_penalty -
                             task->priority * 8.0 - repair_bonus - next_depot_bonus - strip_bonus;

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
        if (task->kind == SO_TASK_INTERIOR_STRIP && task->status != SO_TASK_DONE &&
            task->remaining_ha > 0.001 && so_distance(point, task->center) <= radius) {
            count++;
        }
    }
    return count;
}

static int so_pending_major_task_count(const SoSimulation *sim) {
    int count = 0;
    for (int i = 0; i < sim->field.task_count; i++) {
        const SoFieldTask *task = &sim->field.tasks[i];
        if (task->status != SO_TASK_DONE && task->remaining_ha > 0.001 &&
            task->kind == SO_TASK_INTERIOR_STRIP) {
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
        const double score = dist + site->slope_risk * 120.0;
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
        covered[t] = sim->field.tasks[t].kind != SO_TASK_INTERIOR_STRIP;
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
                    cover++;
                    covered_area += task->remaining_ha;
                    covered_priority += task->priority;
                }
            }
            const double move_minutes = so_hive_travel_minutes(sim, previous, site->point);
            if (move_minutes >= 1e8) {
                continue;
            }
            double backtrack_penalty = 0.0;
            if (sim->mothership.operation_plan_count >= 2) {
                const SoPoint before = sim->mothership.operation_plan[sim->mothership.operation_plan_count - 2];
                const double prev_angle = atan2(previous.y - before.y, previous.x - before.x);
                const double next_angle = atan2(site->point.y - previous.y, site->point.x - previous.x);
                backtrack_penalty = so_angle_diff_rad(prev_angle, next_angle) * 18.0;
            }
            const double stop_cost = 30.0 + sim->mothership.operation_plan_count * 4.0;
            const double score = covered_area * 42.0 + covered_priority * 9.0 + cover * 6.0 -
                                 move_minutes * 3.5 - stop_cost - site->slope_risk * 18.0 - backtrack_penalty;
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
    so_event(sim, "depot plan fixed after scout");
}

static void so_assign_drone_to_task(SoSimulation *sim, SoDrone *drone, SoFieldTask *task, double capacity) {
    const double outbound_energy = so_distance(drone->position, task->center) / 1000.0 * sim->spec.battery_drain_km_empty;
    drone->battery = fmax(0.0, drone->battery - outbound_energy);
    drone->position = task->center;
    drone->state = task->remaining_ha <= capacity && task->remaining_ha <= 2.5 ? SO_DRONE_CLEANUP : SO_DRONE_WORKING;
    if (so_repair_sized_task(sim, task)) {
        drone->state = SO_DRONE_ASSISTING;
    }
    drone->assigned_task_id = task->id;
    drone->assigned_area_ha = task->kind == SO_TASK_INTERIOR_STRIP
                                  ? fmin(capacity * 0.92, task->remaining_ha + 5.5)
                                  : fmin(task->remaining_ha, capacity);
    drone->target = task->center;
    drone->has_target = true;
    drone->travel_remaining_s = 0.0;
    const double task_battery = (drone->assigned_area_ha / fmax(0.001, sim->spec.spray_rate_ha_h)) *
                                sim->spec.battery_drain_h_work;
    const double return_energy = so_estimate_return_energy(drone, sim->mothership.position, sim->spec);
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
        const double leg = so_distance(drone->position, task->center);
        if (leg > 260.0 && task->kind != SO_TASK_BOUNDARY) {
            continue;
        }
        double score = leg / 8.0 + task->risk * 20.0;
        if (task->kind == SO_TASK_INTERIOR_STRIP) {
            score -= 18.0 * task->route_efficiency;
        }
        if (task->kind == SO_TASK_BOUNDARY && so_zone_has_open_interior(sim, task->zone_id)) {
            score += 42.0;
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
        drone->state = SO_DRONE_RETURNING;
        return;
    }
    const int next_idx = so_choose_bundle_continuation(sim, drone);
    if (next_idx < 0) {
        drone->assigned_task_id = -1;
        drone->state = SO_DRONE_RETURNING;
        return;
    }
    SoFieldTask *next = &sim->field.tasks[next_idx];
    const double outbound_energy = so_distance(drone->position, next->center) / 1000.0 * sim->spec.battery_drain_km_empty;
    drone->battery = fmax(0.0, drone->battery - outbound_energy);
    drone->position = next->center;
    drone->assigned_task_id = next->id;
    drone->target = next->center;
    drone->state = next->kind == SO_TASK_BOUNDARY ? SO_DRONE_CLEANUP : SO_DRONE_WORKING;
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
    const double battery_area = available_battery / sim->spec.battery_drain_h_work * sim->spec.spray_rate_ha_h;
    const double chemical_area = drone->chemical / sim->spec.chemical_per_ha;
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
        const double score = empty_dist / 12.0 + destination_dist / 18.0 + task->risk * 25.0 -
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
            if (drone->state != SO_DRONE_STANDBY || drone->battery < 0.42 || drone->chemical < 0.18) {
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
        so_assign_drone_to_task(sim, drone, task, capacity);
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

static double so_best_strip_angle_deg(const SoSimulation *sim, const SoFieldBlock *block) {
    double best_angle = 0.0;
    double best_score = 1e100;
    const SoField *field = &sim->field;
    const double depot_axis = atan2(block->center.y - field->boundary_center.y,
                                   block->center.x - field->boundary_center.x);
    const double wind_rad = sim->mothership.weather.wind_direction_deg * M_PI / 180.0;
    const double wind_strength = fmin(1.0, sim->mothership.weather.wind_speed_mps / 8.0);
    for (int a = 0; a < 180; a += 15) {
        const double rad = (double)a * M_PI / 180.0;
        const double turn_proxy = fabs(sin(rad - depot_axis)) * 14.0;
        const double row_proxy = fabs(cos(rad)) * (0.8 + block->risk);
        const double crosswind = fabs(sin(rad - wind_rad));
        const double wind_proxy = crosswind * (4.0 + block->risk * 5.0) * wind_strength;
        const double score = turn_proxy + row_proxy + wind_proxy;
        if (score < best_score) {
            best_score = score;
            best_angle = (double)a;
        }
    }
    return best_angle;
}

static SoPoint so_offset_by_angle(SoPoint center, double angle_deg, double along, double cross) {
    const double rad = angle_deg * M_PI / 180.0;
    const double ux = cos(rad);
    const double uy = sin(rad);
    const double vx = -uy;
    const double vy = ux;
    return so_point(center.x + ux * along + vx * cross, center.y + uy * along + vy * cross);
}

static void so_build_tasks(SoSimulation *sim) {
    sim->field.zone_count = 0;
    sim->field.task_count = 0;

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

    for (int z = 0; z < sim->field.zone_count; z++) {
        SoOperationZone *zone = &sim->field.zones[z];
        const SoFieldBlock *block = NULL;
        for (int b = 0; b < sim->field.block_count; b++) {
            if (sim->field.blocks[b].id == zone->block_id) {
                block = &sim->field.blocks[b];
                break;
            }
        }
        const double angle = block != NULL ? so_best_strip_angle_deg(sim, block) : 0.0;
        const double boundary_area = fmin(zone->area_ha * 0.12, 0.9);
        const double repair_area = zone->risk > 0.32 ? fmin(zone->area_ha * 0.04, 0.35) : 0.0;
        const double interior_area = fmax(0.1, zone->area_ha - boundary_area - repair_area);
        const int pieces = (int)fmax(1.0, round(interior_area / 5.6));
        const double piece_area = interior_area / pieces;
        const double strip_spacing = fmax(70.0, sqrt(zone->area_ha * 10000.0 / fmax(1.0, (double)pieces)) * 0.38);
        for (int p = 0; p < pieces && sim->field.task_count < SO_MAX_TASKS; p++) {
            SoFieldTask *task = &sim->field.tasks[sim->field.task_count];
            task->id = sim->field.task_count + 1;
            task->zone_id = zone->id;
            task->block_id = zone->block_id;
            task->center = so_offset_by_angle(zone->center, angle, (p - (pieces - 1) / 2.0) * strip_spacing,
                                              ((p % 2) ? 24.0 : -24.0));
            task->area_ha = piece_area;
            task->remaining_ha = piece_area;
            task->priority = 1.2 + zone->risk;
            task->risk = zone->risk;
            task->strip_angle_deg = angle;
            task->route_efficiency = 1.10 + fmax(0.0, 0.35 - zone->risk) * 0.18;
            task->bundle_hint = zone->id;
            task->kind = SO_TASK_INTERIOR_STRIP;
            task->status = SO_TASK_PENDING;
            task->assigned_drone_id = -1;
            sim->field.task_count++;
        }
        if (boundary_area > 0.001 && sim->field.task_count < SO_MAX_TASKS) {
            SoFieldTask *task = &sim->field.tasks[sim->field.task_count];
            task->id = sim->field.task_count + 1;
            task->zone_id = zone->id;
            task->block_id = zone->block_id;
            task->center = so_offset_by_angle(zone->center, angle, 0.0, strip_spacing * 0.9);
            task->area_ha = boundary_area;
            task->remaining_ha = boundary_area;
            task->priority = 0.95 + zone->risk;
            task->risk = fmin(1.0, zone->risk + 0.12);
            task->strip_angle_deg = angle;
            task->route_efficiency = 0.78;
            task->bundle_hint = zone->id;
            task->kind = SO_TASK_BOUNDARY;
            task->status = SO_TASK_PENDING;
            task->assigned_drone_id = -1;
            sim->field.task_count++;
        }
        if (repair_area > 0.001 && sim->field.task_count < SO_MAX_TASKS) {
            SoFieldTask *task = &sim->field.tasks[sim->field.task_count];
            task->id = sim->field.task_count + 1;
            task->zone_id = zone->id;
            task->block_id = zone->block_id;
            task->center = so_offset_by_angle(zone->center, angle, strip_spacing * 0.45, -strip_spacing * 0.75);
            task->area_ha = repair_area;
            task->remaining_ha = repair_area;
            task->priority = 0.65 + zone->risk;
            task->risk = fmin(1.0, zone->risk + 0.18);
            task->strip_angle_deg = angle;
            task->route_efficiency = 0.9;
            task->bundle_hint = zone->id;
            task->kind = SO_TASK_REPAIR;
            task->status = SO_TASK_PENDING;
            task->assigned_drone_id = -1;
            sim->field.task_count++;
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
        const double capacity = so_dynamic_capacity(drone, sim->mothership.position, sim->spec);
        if (capacity < 0.05) {
            continue;
        }
        so_assign_drone_to_task(sim, drone, task, capacity);
    }
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
        const double capacity = so_dynamic_capacity(drone, sim->mothership.position, sim->spec);
        if (capacity < 0.35) {
            continue;
        }
        int best = -1;
        double largest_remaining = 0.0;
        const double radius = so_working_radius(sim);
        for (int t = 0; t < sim->field.task_count; t++) {
            SoFieldTask *task = &sim->field.tasks[t];
            if (task->remaining_ha > largest_remaining && task->status != SO_TASK_DONE &&
                so_distance(sim->mothership.position, task->center) <= radius) {
                largest_remaining = task->remaining_ha;
                best = t;
            }
        }
        if (best >= 0 && largest_remaining > 0.2) {
            SoFieldTask *task = &sim->field.tasks[best];
            so_assign_drone_to_task(sim, drone, task, capacity);
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

static bool so_drone_in_slot(const int slots[2], int drone_id) {
    return slots[0] == drone_id || slots[1] == drone_id;
}

static void so_update_service_queues(SoSimulation *sim) {
    const int charger_count = sim->mothership.fast_chargers < 2 ? sim->mothership.fast_chargers : 2;
    const int refill_count = sim->mothership.refill_ports < 2 ? sim->mothership.refill_ports : 2;

    for (int s = 0; s < 2; s++) {
        if (s >= charger_count) {
            sim->queues.charger_slots[s] = -1;
            continue;
        }
        const int idx = so_drone_index_by_id(sim, sim->queues.charger_slots[s]);
        if (idx < 0 || sim->drones[idx].state != SO_DRONE_CHARGING ||
            sim->drones[idx].battery >= sim->drones[idx].target_charge) {
            sim->queues.charger_slots[s] = -1;
        }
    }

    for (int s = 0; s < 2; s++) {
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

    for (int s = 0; s < charger_count; s++) {
        if (sim->queues.charger_slots[s] != -1) {
            continue;
        }

        int best_id = -1;
        double lowest_battery = 2.0;
        for (int i = 0; i < sim->drone_count; i++) {
            SoDrone *drone = &sim->drones[i];
            if (drone->state != SO_DRONE_CHARGING || so_drone_in_slot(sim->queues.charger_slots, drone->id)) {
                continue;
            }
            if (drone->battery < lowest_battery) {
                lowest_battery = drone->battery;
                best_id = drone->id;
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
            if (!needs_refill || so_drone_in_slot(sim->queues.refill_slots, drone->id)) {
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
    const double fast_charge_rate_h = 7.2;
    const double refill_rate_h = 8.0;

    so_update_service_queues(sim);

    for (int i = 0; i < sim->drone_count; i++) {
        SoDrone *drone = &sim->drones[i];

        if (drone->state == SO_DRONE_SCOUTING) {
            drone->assigned_area_ha = fmax(0.0, drone->assigned_area_ha - weather.scout_rate_ha_h * dt_h);
            drone->battery = fmax(0.0, drone->battery - sim->spec.battery_drain_h_scout *
                                                   weather.battery_scout_multiplier * dt_h);
            if (drone->assigned_area_ha <= 0.001) {
                drone->state = SO_DRONE_RETURNING;
                so_event(sim, "scout completed");
            }
        } else if (drone->state == SO_DRONE_WORKING || drone->state == SO_DRONE_ASSISTING ||
                   drone->state == SO_DRONE_CLEANUP) {
            if (!weather.spray_allowed) {
                drone->battery = fmax(0.0, drone->battery - sim->spec.battery_drain_h_work *
                                                       0.25 * weather.battery_work_multiplier * dt_h);
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
            sim->field.treated_ha = fmin(sim->field.area_ha, sim->field.treated_ha + done);
            drone->battery = fmax(0.0, drone->battery - sim->spec.battery_drain_h_work *
                                                   weather.battery_work_multiplier / fmin(1.18, route_efficiency) * dt_h);
            drone->chemical = fmax(0.0, drone->chemical - sim->spec.chemical_per_ha * done);

            if (task->remaining_ha <= 0.001) {
                task->status = SO_TASK_DONE;
                so_continue_bundle_or_return(sim, drone);
            } else if (drone->assigned_area_ha <= 0.001) {
                so_release_task(sim, drone);
                drone->state = SO_DRONE_RETURNING;
            }
        } else if (drone->state == SO_DRONE_RETURNING) {
            if (drone->travel_remaining_s <= 0.001) {
                const SoPoint recovery_point = sim->mothership.moving
                                                   ? sim->mothership.destination
                                                   : sim->mothership.position;
                const double dist = so_distance(drone->position, recovery_point);
                drone->travel_remaining_s = dist / fmax(0.001, weather.cruise_speed_mps);
                drone->return_energy_required = dist / 1000.0 * sim->spec.battery_drain_km_empty;
            }
            if (drone->travel_remaining_s <= sim->dt_s) {
                drone->battery = fmax(0.0, drone->battery - drone->return_energy_required);
                drone->travel_remaining_s = 0.0;
                drone->return_energy_required = 0.0;
                if (sim->mothership.moving) {
                    drone->position = sim->mothership.destination;
                } else {
                    drone->position = sim->mothership.position;
                    drone->state = SO_DRONE_CHARGING;
                }
            } else {
                const double fraction = sim->dt_s / drone->travel_remaining_s;
                drone->battery = fmax(0.0, drone->battery - drone->return_energy_required * fraction);
                drone->return_energy_required = fmax(0.0, drone->return_energy_required * (1.0 - fraction));
                drone->travel_remaining_s = fmax(0.0, drone->travel_remaining_s - sim->dt_s);
            }
        } else if (drone->state == SO_DRONE_CHARGING) {
            if (so_drone_in_slot(sim->queues.charger_slots, drone->id)) {
                drone->battery = fmin(1.0, drone->battery + fast_charge_rate_h * dt_h);
            }
            if (so_drone_in_slot(sim->queues.refill_slots, drone->id)) {
                drone->chemical = fmin(1.0, drone->chemical + refill_rate_h * dt_h);
            }
            if (drone->battery >= drone->target_charge) {
                drone->state = drone->chemical < 0.98 ? SO_DRONE_REFILLING : SO_DRONE_STANDBY;
            }
        } else if (drone->state == SO_DRONE_REFILLING) {
            if (so_drone_in_slot(sim->queues.refill_slots, drone->id)) {
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
                        task->kind == SO_TASK_INTERIOR_STRIP && so_distance(site->point, task->center) <= active_cover_radius) {
                        cover++;
                    }
                }
                const double travel = so_hive_route_distance(sim, sim->mothership.position, site->point);
                if (travel >= 1e11) {
                    continue;
                }
                if (so_depot_plan_contains(sim, site->point, 140.0)) {
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

void so_init_default(SoSimulation *sim) {
    memset(sim, 0, sizeof(*sim));
    sim->drone_count = SO_MAX_DRONES;
    sim->dt_s = 60.0;
    sim->next_weather_update_s = 0.0;
    sim->spec.cruise_speed_mps = 12.0;
    sim->spec.scout_speed_mps = 6.0;
    sim->spec.spray_rate_ha_h = 8.0;
    sim->spec.scout_rate_ha_h = 25.0;
    const double default_spray_speed_mps = sim->spec.cruise_speed_mps * 0.45;
    sim->spec.spray_swath_m =
        sim->spec.spray_rate_ha_h * 10000.0 / fmax(0.001, default_spray_speed_mps * 3600.0);
    sim->spec.spray_radius_m = sim->spec.spray_swath_m * 0.5;
    sim->spec.battery_drain_h_work = 0.38;
    sim->spec.battery_drain_h_scout = 0.24;
    sim->spec.battery_drain_km_empty = 0.025;
    sim->spec.chemical_per_ha = 0.11;
    sim->spec.chemical_tank_area_ha = 1.0 / sim->spec.chemical_per_ha;
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
    sim->mothership.fast_chargers = 2;
    sim->mothership.refill_ports = 2;
    sim->mothership.position = so_point(-600.0, 0.0);
    sim->mothership.move_speed_mps = SO_HIVE_MOVE_SPEED_MPS;
    sim->mothership.weather = (SoWeather){2.5, 4.0, 26.0, 0.55, 0.0, 5000.0, 70.0, 0.0};
    for (int i = 0; i < 2; i++) {
        sim->queues.charger_slots[i] = -1;
        sim->queues.refill_slots[i] = -1;
    }

    for (int i = 0; i < sim->drone_count; i++) {
        sim->drones[i].id = i + 1;
        sim->drones[i].state = SO_DRONE_IDLE;
        sim->drones[i].battery = 1.0;
        sim->drones[i].chemical = 1.0;
        sim->drones[i].position = sim->mothership.position;
        sim->drones[i].target_charge = 0.8;
        sim->drones[i].assigned_task_id = -1;
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
    sim->fixed_wing.airport = so_point(sim->mothership.position.x - 5000.0, sim->mothership.position.y);
    sim->fixed_wing.tank_l = 1893.0;
    sim->fixed_wing.fuel_l = 644.0;
    sim->fixed_wing.payload_kg = 2450.0;
}

void so_step(SoSimulation *sim) {
    so_update_weather(sim);
    const SoWeatherAdjustedSpec weather = so_adjust_for_weather(sim->spec, sim->mothership.weather);
    so_weather_recovery(sim);
    so_update_mothership(sim);

    if (!sim->field.scanned) {
        so_build_tasks(sim);
        so_plan_fixed_wing_coverage(sim);
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
            sim->drones[i].state = SO_DRONE_SCOUTING;
            sim->drones[i].assigned_area_ha = sim->field.area_ha / 2.0;
        }
        so_event(sim, "scout assigned");
    }

    so_update_fixed_wing(sim, weather);
    so_update_active_drones(sim, weather);
    so_relocate_if_needed(sim);
    if (sim->mothership.moving) {
        so_assign_relocation_cleanup(sim, weather);
    } else {
        so_assign_work(sim);
        so_assign_assist(sim);
    }
    sim->now_s += sim->dt_s;
}

bool so_completed(const SoSimulation *sim) {
    return sim->field.area_ha - sim->field.treated_ha <= 0.001;
}

void so_run(SoSimulation *sim, int max_steps) {
    for (int i = 0; i < max_steps && !so_completed(sim); i++) {
        so_step(sim);
    }
    if (so_completed(sim)) {
        sim->field.treated_ha = sim->field.area_ha;
    }
}

void so_print_summary(const SoSimulation *sim) {
    printf("Scout-Driven Multi-UAV Agricultural OPT C Simulation\n");
    printf("completed: %s\n", so_completed(sim) ? "true" : "false");
    printf("time_hours: %.2f\n", sim->now_s / 3600.0);
    printf("treated_area_ha: %.2f/%.2f\n", sim->field.treated_ha, sim->field.area_ha);
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

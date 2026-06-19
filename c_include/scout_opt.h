#ifndef SCOUT_OPT_H
#define SCOUT_OPT_H

#include <stdbool.h>
#include <stddef.h>

#define SO_MAX_DRONES 8
#define SO_MAX_BLOCKS 32
#define SO_MAX_ZONES 160
#define SO_MAX_TASKS 640
#define SO_MAX_DEPOTS 256
#define SO_MAX_EVENTS 4096
#define SO_MAX_LANDING_SPOTS 16
#define SO_MAX_BOUNDARY_POINTS 64
#define SO_MAX_FAST_CHARGERS 4
#define SO_BATTERY_SLOTS_PER_CHARGER 2
#define SO_MAX_CHARGER_SLOTS (SO_MAX_FAST_CHARGERS * SO_BATTERY_SLOTS_PER_CHARGER)
#define SO_MAX_REFILL_PORTS 2
#define SO_MAX_DRONE_ROUTE_POINTS 1024
#define SO_MAX_DRONE_ROUTE_SEGMENTS 256

#define SO_HIVE_MOVE_SPEED_KMH 30.0
#define SO_HIVE_MOVE_SPEED_MPS (SO_HIVE_MOVE_SPEED_KMH * 1000.0 / 3600.0)

typedef struct {
    double x;
    double y;
} SoPoint;

typedef enum {
    SO_DRONE_IDLE,
    SO_DRONE_SCOUTING,
    SO_DRONE_WORKING,
    SO_DRONE_ASSISTING,
    SO_DRONE_RETURNING,
    SO_DRONE_CHARGING,
    SO_DRONE_REFILLING,
    SO_DRONE_STANDBY,
    SO_DRONE_CLEANUP,
    SO_DRONE_PREDEPLOY,
    SO_DRONE_EMERGENCY_LANDING,
    SO_DRONE_LANDED
} SoDroneState;

typedef enum {
    SO_TASK_PENDING,
    SO_TASK_IN_PROGRESS,
    SO_TASK_DONE,
    SO_TASK_BLOCKED
} SoTaskStatus;

typedef enum {
    SO_TASK_INTERIOR_STRIP,
    SO_TASK_BOUNDARY,
    SO_TASK_REPAIR
} SoTaskKind;

typedef enum {
    SO_WEATHER_NORMAL,
    SO_WEATHER_WATCH,
    SO_WEATHER_WARNING,
    SO_WEATHER_SEVERE,
    SO_WEATHER_EMERGENCY
} SoWeatherSeverity;

typedef enum {
    SO_OPT_PROFILE_BALANCED,
    SO_OPT_PROFILE_TIME,
    SO_OPT_PROFILE_COST
} SoOptimizationProfile;

typedef enum {
    SO_DRONE_ROUTE_TRANSFER,
    SO_DRONE_ROUTE_SPRAY
} SoDroneRouteKind;

typedef struct {
    double wind_speed_mps;
    double wind_gust_mps;
    double temperature_c;
    double humidity;
    double precipitation_mmph;
    double visibility_m;
    double wind_direction_deg;
    double updated_at_s;
} SoWeather;

typedef struct {
    double scout_rate_ha_h;
    double spray_rate_ha_h;
    double cruise_speed_mps;
    double battery_work_multiplier;
    double battery_scout_multiplier;
    double spray_effectiveness;
    bool spray_allowed;
    bool flight_allowed;
} SoWeatherAdjustedSpec;

typedef struct {
    int id;
    const char *name;
    SoPoint center;
    double area_ha;
    double risk;
    bool selected;
    int boundary_count;
    SoPoint boundary[SO_MAX_BOUNDARY_POINTS];
} SoFieldBlock;

typedef struct {
    int id;
    int block_id;
    SoPoint center;
    double area_ha;
    double treated_ha;
    double risk;
} SoOperationZone;

typedef struct {
    int id;
    int zone_id;
    int block_id;
    SoPoint center;
    double area_ha;
    double remaining_ha;
    double priority;
    double risk;
    double strip_angle_deg;
    double route_efficiency;
    bool has_planned_route;
    SoPoint route_start;
    SoPoint route_mid;
    SoPoint route_end;
    bool has_route_mid;
    double route_curve_deg;
    double fixed_wing_area_ha;
    int turn_count;
    double turn_time_s;
    double turn_energy_cost;
    int bundle_hint;
    SoTaskKind kind;
    SoTaskStatus status;
    int assigned_drone_id;
} SoFieldTask;

typedef struct {
    int id;
    SoPoint point;
    double usable_area_m2;
    bool road_accessible;
    double slope_risk;
} SoDepotSite;

typedef struct {
    int id;
    SoDroneState state;
    double battery;
    double chemical;
    SoPoint position;
    SoPoint target;
    bool has_target;
    double assigned_area_ha;
    double remaining_capacity_ha;
    double return_energy_required;
    double travel_remaining_s;
    double target_charge;
    int assigned_task_id;
    int sortie_battery_modules;
    double sortie_battery_capacity_kwh;
    double sortie_chemical_tank_l;
    double sortie_chemical_per_ha;
    double sortie_chemical_tank_area_ha;
    double sortie_battery_drain_h_work;
    double sortie_battery_drain_h_scout;
    double sortie_battery_drain_km_empty;
    int route_point_count;
    SoPoint route_points[SO_MAX_DRONE_ROUTE_POINTS];
    int route_segment_count;
    int route_segment_start[SO_MAX_DRONE_ROUTE_SEGMENTS];
    int route_segment_point_count[SO_MAX_DRONE_ROUTE_SEGMENTS];
    int route_segment_block_id[SO_MAX_DRONE_ROUTE_SEGMENTS];
    int route_segment_task_id[SO_MAX_DRONE_ROUTE_SEGMENTS];
    SoDroneRouteKind route_segment_kind[SO_MAX_DRONE_ROUTE_SEGMENTS];
} SoDrone;

typedef struct {
    int drone_slots;
    int fast_chargers;
    int refill_ports;
    SoPoint position;
    SoPoint destination;
    bool moving;
    double move_remaining_s;
    double move_speed_mps;
    SoPoint operation_plan[SO_MAX_DEPOTS];
    int operation_plan_count;
    int operation_plan_index;
    SoWeather weather;
    double move_distance_m;
    double move_cost_usd;
    double stop_cost_usd;
    double truck_cost_usd_per_km;
    double deployment_stop_cost_usd;
} SoMothership;

typedef struct {
    double cruise_speed_mps;
    double scout_speed_mps;
    double spray_rate_ha_h;
    double scout_rate_ha_h;
    double spray_swath_m;
    double spray_radius_m;
    double battery_drain_h_work;
    double battery_drain_h_scout;
    double battery_drain_km_empty;
    double turn_time_s;
    double turn_battery_cost;
    double flight_cost_usd_per_km;
    double launch_cost_usd;
    double effective_chemical_l_per_ha;
    double deposition_efficiency;
    double chemical_tank_l;
    double chemical_l_per_ha;
    double chemical_cost_usd_per_l;
    double battery_capacity_kwh;
    int battery_modules;
    int min_battery_modules;
    int max_battery_modules;
    double battery_module_capacity_kwh;
    double battery_module_weight_kg;
    double modeled_payload_capacity_kg;
    double spray_system_weight_kg;
    double chemical_density_kg_per_l;
    double chemical_tank_max_l;
    double selected_payload_kg;
    double work_power_kw;
    double scout_power_kw;
    double fast_charger_power_kw;
    double electricity_price_usd_per_kwh;
    double turn_radius_m;
    double unfinished_penalty_usd_per_ha;
    double chemical_per_ha;
    double chemical_tank_area_ha;
    double safety_battery_margin;
} SoDroneSpec;

typedef struct {
    bool enabled;
    bool planned;
    int aircraft_count;
    char model_name[32];
    SoPoint airport;
    double tank_l;
    double fuel_l;
    double payload_kg;
    double work_speed_mps;
    double swath_width_m;
    double cruise_speed_mps;
    double spray_efficiency;
    double setup_time_s;
    double turnaround_time_s;
    double turn_time_s;
    double turn_fuel_h;
    int planned_turns;
    int corridor_count;
    double corridor_work_m;
    double corridor_empty_m;
    double corridor_total_m;
    double tank_area_ha;
    double fuel_endurance_h;
    double ferry_time_s;
    double planned_turn_non_spray_time_s;
    double turn_non_spray_time_s;
    double average_ferry_round_trip_m;
    double sortie_remaining_ha;
    double fuel_remaining_h;
    double service_remaining_s;
    int sorties_completed;
    double economic_cost_h;
    double flight_cost_usd_per_km;
    double takeoff_cost_usd;
    double airport_service_cost_usd;
    double effective_chemical_l_per_ha;
    double deposition_efficiency;
    double chemical_l_per_ha;
    double chemical_cost_usd_per_l;
    double fuel_burn_l_per_h;
    double fuel_price_usd_per_l;
    double fuel_cost_usd_per_h;
    double turn_radius_m;
    double unfinished_penalty_usd_per_ha;
    double flight_distance_m;
    double flight_cost_usd;
    double airport_cost_usd;
    double total_cost_usd;
    double spray_rate_ha_h;
    double assigned_area_ha;
    double completed_area_ha;
    char path_strategy[32];
    double path_strategy_score;
    double path_strategy_time_h[3];
    double path_strategy_cost_usd[3];
    double path_strategy_scoreboard[3];
} SoFixedWingFleet;

typedef struct {
    double area_ha;
    double treated_ha;
    double terrain_complexity;
    double obstacle_density;
    SoPoint boundary_center;
    bool has_origin;
    double origin_lat;
    double origin_lon;
    bool scanned;

    SoFieldBlock blocks[SO_MAX_BLOCKS];
    int block_count;

    SoOperationZone zones[SO_MAX_ZONES];
    int zone_count;

    SoFieldTask tasks[SO_MAX_TASKS];
    int task_count;

    SoDepotSite depots[SO_MAX_DEPOTS];
    int depot_count;

    SoPoint emergency_landing_spots[SO_MAX_LANDING_SPOTS];
    int emergency_landing_spot_count;
} SoField;

typedef struct {
    int charger_slots[SO_MAX_CHARGER_SLOTS];
    int refill_slots[SO_MAX_REFILL_PORTS];
} SoServiceQueues;

typedef struct {
    SoField field;
    SoMothership mothership;
    SoDrone drones[SO_MAX_DRONES];
    int drone_count;
    SoDroneSpec spec;
    SoFixedWingFleet fixed_wing;
    SoServiceQueues queues;
    double now_s;
    double dt_s;
    SoOptimizationProfile optimization_profile;
    double next_weather_update_s;
    double uav_flight_distance_m;
    double uav_flight_cost_usd;
    double uav_energy_used_battery_units;
    double uav_energy_used_kwh;
    double uav_electricity_cost_usd;
    double uav_launch_cost_usd;
    int uav_takeoffs;
    int uav_sorties_by_battery_modules[5];
    double uav_area_by_battery_modules[5];
    double coverage_task_tolerance_ratio;
    double coverage_final_error_limit_ratio;
    double coverage_repair_cost_limit_ratio;
    double final_uncovered_ha;
    double final_uncovered_ratio;
    bool final_repair_attempted;
    bool final_repair_performed;
    double final_repair_area_ha;
    double final_repair_cost_usd;
    double final_repair_cost_ratio;
    char events[SO_MAX_EVENTS][160];
    int event_count;
} SoSimulation;

void so_init_default(SoSimulation *sim);
void so_init_two_block_demo(SoSimulation *sim);
void so_init_multi_block_demo(SoSimulation *sim, int block_count);
void so_init_ideal_layout_demo(SoSimulation *sim, int block_count);
void so_init_irregular_layout_demo(SoSimulation *sim, int block_count);
void so_init_hybrid_layout_demo(SoSimulation *sim, int block_count);
void so_enable_fixed_wing(SoSimulation *sim);
void so_run(SoSimulation *sim, int max_steps);
void so_step(SoSimulation *sim);
bool so_completed(const SoSimulation *sim);
void so_print_summary(const SoSimulation *sim);
bool so_export_visual_plan(const SoSimulation *sim, const char *path);
const char *so_optimization_profile_name(SoOptimizationProfile profile);

const char *so_drone_state_name(SoDroneState state);
const char *so_weather_severity_name(SoWeatherSeverity severity);

#endif

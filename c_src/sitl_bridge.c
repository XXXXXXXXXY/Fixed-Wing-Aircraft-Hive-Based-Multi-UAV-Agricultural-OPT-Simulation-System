#include "scout_opt_sitl.h"

#include <stdio.h>

SoSitlConfig so_sitl_default_config(void) {
    SoSitlConfig config;
    config.ardupilot_root = "ardupilot";
    config.vehicle = "ArduCopter";
    config.frame = "quad";
    config.location = "CMAC";
    config.drone_count = SO_MAX_DRONES;
    config.base_instance = 0;
    config.auto_offset_bearing_deg = 90;
    config.auto_offset_distance_m = 10;
    config.base_mavlink_udp_port = 14550;
    config.use_mavproxy = false;
    config.use_map = false;
    config.use_console = false;
    config.no_rebuild = false;
    return config;
}

void so_sitl_build_command(const SoSitlConfig *config, char *buffer, size_t buffer_size) {
    const char *mavproxy = config->use_mavproxy ? "" : " --no-mavproxy";
    const char *map = (config->use_mavproxy && config->use_map) ? " --map" : "";
    const char *console = (config->use_mavproxy && config->use_console) ? " --console" : "";
    const char *no_rebuild = config->no_rebuild ? " --no-rebuild" : "";

    snprintf(buffer, buffer_size,
             "python \"%s\\Tools\\autotest\\sim_vehicle.py\" -v %s -f %s --count %d --auto-sysid "
             "--auto-offset-line %d,%d --location %s --mcast%s%s%s%s",
             config->ardupilot_root, config->vehicle, config->frame, config->drone_count,
             config->auto_offset_bearing_deg, config->auto_offset_distance_m, config->location,
             mavproxy, map, console, no_rebuild);
}

void so_sitl_print_link_plan(const SoSitlConfig *config, const SoSimulation *sim) {
    char command[640];
    so_sitl_build_command(config, command, sizeof(command));

    printf("ArduPilot SITL link plan\n");
    printf("full_name: ArduPilot Software In The Loop\n");
    printf("vehicle: %s frame=%s count=%d auto_sysid=true\n",
           config->vehicle, config->frame, config->drone_count);
    printf("start_command:\n%s\n", command);
    printf("mavlink:\n");
    printf("  multicast: 239.255.145.50:14550\n");
    printf("  qgroundcontrol: connect to UDP 14550 or use multicast output\n");
    printf("  direct-sitl-base: tcp/udp instance ports are managed by sim_vehicle.py from instance %d\n",
           config->base_instance);
    printf("mission_bridge:\n");
    printf("  C OPT stays authoritative for depot/zone/task scheduling.\n");
    printf("  SITL adapter should translate drone state hints into MAVLink arm/takeoff/goto/rtl commands.\n");
    if (sim != NULL) {
        printf("current_plan:\n");
        printf("  drones=%d blocks=%d tasks=%d depots=%d stops=%d\n",
               sim->drone_count, sim->field.block_count, sim->field.task_count,
               sim->field.depot_count, sim->mothership.operation_plan_count);
        for (int i = 0; i < sim->mothership.operation_plan_count; i++) {
            const SoPoint p = sim->mothership.operation_plan[i];
            printf("  depot_stop_%d=(%.1f,%.1f)\n", i + 1, p.x, p.y);
        }
    }
}

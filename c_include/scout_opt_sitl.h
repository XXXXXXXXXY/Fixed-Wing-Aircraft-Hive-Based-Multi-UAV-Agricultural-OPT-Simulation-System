#ifndef SCOUT_OPT_SITL_H
#define SCOUT_OPT_SITL_H

#include "scout_opt.h"

#include <stddef.h>

typedef struct {
    const char *ardupilot_root;
    const char *vehicle;
    const char *frame;
    const char *location;
    int drone_count;
    int base_instance;
    int auto_offset_bearing_deg;
    int auto_offset_distance_m;
    int base_mavlink_udp_port;
    bool use_mavproxy;
    bool use_map;
    bool use_console;
    bool no_rebuild;
} SoSitlConfig;

SoSitlConfig so_sitl_default_config(void);
void so_sitl_build_command(const SoSitlConfig *config, char *buffer, size_t buffer_size);
void so_sitl_print_link_plan(const SoSitlConfig *config, const SoSimulation *sim);

#endif

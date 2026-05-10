#ifndef SCOUT_OPT_CONFIG_H
#define SCOUT_OPT_CONFIG_H

#include "scout_opt.h"

#include <stdbool.h>
#include <stddef.h>

bool so_load_manual_scout_config(SoSimulation *sim, const char *path, char *error, size_t error_size);

#endif

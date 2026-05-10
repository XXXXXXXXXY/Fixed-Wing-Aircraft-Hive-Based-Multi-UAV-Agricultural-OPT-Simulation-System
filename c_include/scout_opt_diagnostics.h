#ifndef SCOUT_OPT_DIAGNOSTICS_H
#define SCOUT_OPT_DIAGNOSTICS_H

#include "scout_opt.h"

typedef struct {
    int errors;
    int warnings;
} SoValidationResult;

SoValidationResult so_validate_simulation(const SoSimulation *sim);
void so_print_fleet(const SoSimulation *sim);
void so_print_depot_plan(const SoSimulation *sim);
int so_run_acceptance_suite(void);

#endif

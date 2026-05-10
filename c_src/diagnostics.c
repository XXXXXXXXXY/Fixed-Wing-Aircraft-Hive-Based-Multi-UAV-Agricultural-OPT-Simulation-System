#include "scout_opt_diagnostics.h"
#include "scout_opt_config.h"

#include <math.h>
#include <stdio.h>

static int so_task_done_count(const SoSimulation *sim) {
    int count = 0;
    for (int i = 0; i < sim->field.task_count; i++) {
        if (sim->field.tasks[i].status == SO_TASK_DONE) {
            count++;
        }
    }
    return count;
}

SoValidationResult so_validate_simulation(const SoSimulation *sim) {
    SoValidationResult result = {0, 0};
    if (sim->drone_count <= 0 || sim->drone_count > SO_MAX_DRONES) {
        result.errors++;
    }
    if (sim->field.task_count <= 0 && sim->field.scanned) {
        result.errors++;
    }
    if (sim->mothership.operation_plan_count <= 0 && sim->field.scanned) {
        result.errors++;
    }
    if (sim->field.treated_ha < -0.001 || sim->field.treated_ha - sim->field.area_ha > 0.01) {
        result.errors++;
    }
    for (int i = 0; i < sim->drone_count; i++) {
        const SoDrone *drone = &sim->drones[i];
        if (drone->battery < -0.001 || drone->battery > 1.001) {
            result.errors++;
        }
        if (drone->chemical < -0.001 || drone->chemical > 1.001) {
            result.errors++;
        }
        if (drone->state == SO_DRONE_WORKING && drone->assigned_task_id < 0) {
            result.warnings++;
        }
    }
    return result;
}

void so_print_fleet(const SoSimulation *sim) {
    printf("fleet:\n");
    for (int i = 0; i < sim->drone_count; i++) {
        const SoDrone *drone = &sim->drones[i];
        printf("  drone=%d state=%s battery=%.2f chemical=%.2f task=%d pos=(%.1f,%.1f)\n",
               drone->id, so_drone_state_name(drone->state), drone->battery, drone->chemical,
               drone->assigned_task_id, drone->position.x, drone->position.y);
    }
}

void so_print_depot_plan(const SoSimulation *sim) {
    printf("depot_plan:\n");
    for (int i = 0; i < sim->mothership.operation_plan_count; i++) {
        const SoPoint p = sim->mothership.operation_plan[i];
        printf("  stop=%d point=(%.1f,%.1f)\n", i + 1, p.x, p.y);
    }
}

int so_run_acceptance_suite(void) {
    int failures = 0;

    SoSimulation single;
    so_init_default(&single);
    so_run(&single, 900);
    SoValidationResult single_validation = so_validate_simulation(&single);
    if (!so_completed(&single) || single_validation.errors != 0) {
        failures++;
        printf("acceptance default FAILED completed=%d errors=%d warnings=%d treated=%.2f/%.2f\n",
               so_completed(&single), single_validation.errors, single_validation.warnings,
               single.field.treated_ha, single.field.area_ha);
    } else {
        printf("acceptance default OK tasks_done=%d events=%d\n", so_task_done_count(&single), single.event_count);
    }

    SoSimulation two_blocks;
    so_init_two_block_demo(&two_blocks);
    so_run(&two_blocks, 900);
    SoValidationResult two_validation = so_validate_simulation(&two_blocks);
    if (!so_completed(&two_blocks) || two_validation.errors != 0) {
        failures++;
        printf("acceptance two-blocks FAILED completed=%d errors=%d warnings=%d treated=%.2f/%.2f\n",
               so_completed(&two_blocks), two_validation.errors, two_validation.warnings,
               two_blocks.field.treated_ha, two_blocks.field.area_ha);
    } else {
        printf("acceptance two-blocks OK tasks_done=%d events=%d\n", so_task_done_count(&two_blocks), two_blocks.event_count);
    }

    SoSimulation multi_blocks;
    so_init_multi_block_demo(&multi_blocks, 5);
    so_run(&multi_blocks, 1500);
    SoValidationResult multi_validation = so_validate_simulation(&multi_blocks);
    if (!so_completed(&multi_blocks) || multi_validation.errors != 0) {
        failures++;
        printf("acceptance multi-blocks FAILED completed=%d errors=%d warnings=%d treated=%.2f/%.2f blocks=%d\n",
               so_completed(&multi_blocks), multi_validation.errors, multi_validation.warnings,
               multi_blocks.field.treated_ha, multi_blocks.field.area_ha, multi_blocks.field.block_count);
    } else {
        printf("acceptance multi-blocks OK blocks=%d tasks_done=%d events=%d\n",
               multi_blocks.field.block_count, so_task_done_count(&multi_blocks), multi_blocks.event_count);
    }

    SoSimulation scenario;
    char error[160];
    if (!so_load_manual_scout_config(&scenario, "configs/manual_scout_two_blocks.example.json", error, sizeof(error))) {
        failures++;
        printf("acceptance scenario-load FAILED error=%s\n", error);
    } else {
        so_run(&scenario, 1200);
        SoValidationResult scenario_validation = so_validate_simulation(&scenario);
        if (!so_completed(&scenario) || scenario_validation.errors != 0) {
            failures++;
            printf("acceptance scenario-load FAILED completed=%d errors=%d warnings=%d treated=%.2f/%.2f blocks=%d\n",
                   so_completed(&scenario), scenario_validation.errors, scenario_validation.warnings,
                   scenario.field.treated_ha, scenario.field.area_ha, scenario.field.block_count);
        } else {
            printf("acceptance scenario-load OK blocks=%d tasks_done=%d events=%d\n",
                   scenario.field.block_count, so_task_done_count(&scenario), scenario.event_count);
        }
    }

    return failures == 0 ? 0 : 1;
}

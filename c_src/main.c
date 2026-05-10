#include "scout_opt.h"
#include "scout_opt_config.h"
#include "scout_opt_diagnostics.h"
#include "scout_opt_sitl.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void usage(const char *program) {
    printf("Usage: %s [--two-blocks] [--blocks N] [--ideal-blocks N] [--irregular-blocks N] [--hybrid-blocks N] [--fixed-wing] [--compare-layouts N] [--scenario FILE] [--steps N] [--diagnostics] [--sitl-plan] [--export-visual FILE] [--acceptance]\n", program);
}

static double run_layout_case(const char *name, int blocks, int steps, void (*init_fn)(SoSimulation *, int)) {
    SoSimulation sim;
    init_fn(&sim, blocks);
    so_run(&sim, steps);
    const double hours = sim.now_s / 3600.0;
    const double productivity = hours > 0.0 ? sim.field.area_ha / hours : 0.0;
    printf("layout=%s blocks=%d completed=%s area_ha=%.2f hours=%.2f ha_per_h=%.2f stops=%d events=%d\n",
           name, blocks, so_completed(&sim) ? "true" : "false", sim.field.area_ha,
           hours, productivity, sim.mothership.operation_plan_count, sim.event_count);
    if (sim.fixed_wing.enabled) {
        printf("  fixed_wing model=%s count=%d assigned_ha=%.2f completed_ha=%.2f rate_ha_h=%.2f tank_ha=%.2f fuel_h=%.2f sorties=%d economic_cost_h=%.2f\n",
               sim.fixed_wing.model_name[0] ? sim.fixed_wing.model_name : "none",
               sim.fixed_wing.aircraft_count,
               sim.fixed_wing.assigned_area_ha,
               sim.fixed_wing.completed_area_ha,
               sim.fixed_wing.spray_rate_ha_h,
               sim.fixed_wing.tank_area_ha,
               sim.fixed_wing.fuel_endurance_h,
               sim.fixed_wing.sorties_completed,
               sim.fixed_wing.economic_cost_h);
    }
    return so_completed(&sim) ? productivity : 0.0;
}

static void init_hybrid_normal(SoSimulation *sim, int blocks) {
    so_init_multi_block_demo(sim, blocks);
    so_enable_fixed_wing(sim);
}

static void init_hybrid_ideal(SoSimulation *sim, int blocks) {
    so_init_ideal_layout_demo(sim, blocks);
    so_enable_fixed_wing(sim);
}

static void init_hybrid_irregular(SoSimulation *sim, int blocks) {
    so_init_irregular_layout_demo(sim, blocks);
    so_enable_fixed_wing(sim);
}

int main(int argc, char **argv) {
    int steps = 900;
    bool two_blocks = false;
    bool diagnostics = false;
    bool acceptance = false;
    bool sitl_plan = false;
    bool fixed_wing = false;
    int block_count = 0;
    int ideal_block_count = 0;
    int irregular_block_count = 0;
    int hybrid_block_count = 0;
    int compare_layout_count = 0;
    const char *scenario = NULL;
    const char *export_visual = NULL;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--two-blocks") == 0) {
            two_blocks = true;
        } else if (strcmp(argv[i], "--blocks") == 0 && i + 1 < argc) {
            block_count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--ideal-blocks") == 0 && i + 1 < argc) {
            ideal_block_count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--irregular-blocks") == 0 && i + 1 < argc) {
            irregular_block_count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--hybrid-blocks") == 0 && i + 1 < argc) {
            hybrid_block_count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--compare-layouts") == 0 && i + 1 < argc) {
            compare_layout_count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
            scenario = argv[++i];
        } else if (strcmp(argv[i], "--diagnostics") == 0) {
            diagnostics = true;
        } else if (strcmp(argv[i], "--sitl-plan") == 0) {
            sitl_plan = true;
        } else if (strcmp(argv[i], "--fixed-wing") == 0) {
            fixed_wing = true;
        } else if (strcmp(argv[i], "--export-visual") == 0 && i + 1 < argc) {
            export_visual = argv[++i];
        } else if (strcmp(argv[i], "--acceptance") == 0) {
            acceptance = true;
        } else if (strcmp(argv[i], "--steps") == 0 && i + 1 < argc) {
            steps = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (acceptance) {
        return so_run_acceptance_suite();
    }

    if (compare_layout_count > 0) {
        const int compare_steps = steps > 900 ? steps : 5000;
        const double normal = run_layout_case("normal", compare_layout_count, compare_steps, so_init_multi_block_demo);
        const double ideal = run_layout_case("ideal", compare_layout_count, compare_steps, so_init_ideal_layout_demo);
        const double irregular = run_layout_case("irregular_scarce", compare_layout_count, compare_steps, so_init_irregular_layout_demo);
        const double hybrid = run_layout_case("hybrid_normal_fixedwing_drone", compare_layout_count, compare_steps, init_hybrid_normal);
        const double hybrid_ideal = run_layout_case("hybrid_ideal_fixedwing_drone", compare_layout_count, compare_steps, init_hybrid_ideal);
        const double hybrid_irregular = run_layout_case("hybrid_irregular_fixedwing_drone", compare_layout_count, compare_steps, init_hybrid_irregular);
        const char *best = "normal";
        double best_score = normal;
        if (ideal > best_score) {
            best = "ideal";
            best_score = ideal;
        }
        if (irregular > best_score) {
            best = "irregular_scarce";
            best_score = irregular;
        }
        if (hybrid > best_score) {
            best = "hybrid_normal_fixedwing_drone";
            best_score = hybrid;
        }
        if (hybrid_ideal > best_score) {
            best = "hybrid_ideal_fixedwing_drone";
            best_score = hybrid_ideal;
        }
        if (hybrid_irregular > best_score) {
            best = "hybrid_irregular_fixedwing_drone";
            best_score = hybrid_irregular;
        }
        printf("recommended_layout=%s ha_per_h=%.2f\n", best, best_score);
        return best_score > 0.0 ? 0 : 2;
    }

    SoSimulation sim;
    if (scenario != NULL) {
        char error[160];
        if (!so_load_manual_scout_config(&sim, scenario, error, sizeof(error))) {
            fprintf(stderr, "scenario load failed: %s\n", error);
            return 1;
        }
    } else if (ideal_block_count > 0) {
        so_init_ideal_layout_demo(&sim, ideal_block_count);
    } else if (irregular_block_count > 0) {
        so_init_irregular_layout_demo(&sim, irregular_block_count);
    } else if (hybrid_block_count > 0) {
        so_init_hybrid_layout_demo(&sim, hybrid_block_count);
    } else if (block_count > 0) {
        so_init_multi_block_demo(&sim, block_count);
    } else if (two_blocks) {
        so_init_two_block_demo(&sim);
    } else {
        so_init_default(&sim);
    }
    if (fixed_wing) {
        so_enable_fixed_wing(&sim);
    }
    so_run(&sim, steps);
    if (export_visual != NULL && !so_export_visual_plan(&sim, export_visual)) {
        fprintf(stderr, "visual plan export failed: %s\n", export_visual);
        return 1;
    }
    so_print_summary(&sim);
    if (sitl_plan) {
        SoSitlConfig sitl = so_sitl_default_config();
        sitl.drone_count = sim.drone_count;
        so_sitl_print_link_plan(&sitl, &sim);
    }
    if (diagnostics) {
        SoValidationResult validation = so_validate_simulation(&sim);
        so_print_depot_plan(&sim);
        so_print_fleet(&sim);
        printf("validation: errors=%d warnings=%d\n", validation.errors, validation.warnings);
    }
    return so_completed(&sim) ? 0 : 2;
}

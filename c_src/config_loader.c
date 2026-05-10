#include "scout_opt_config.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static SoPoint cfg_point(double x, double y) {
    SoPoint p;
    p.x = x;
    p.y = y;
    return p;
}

static void cfg_error(char *error, size_t error_size, const char *message) {
    if (error != NULL && error_size > 0) {
        snprintf(error, error_size, "%s", message);
    }
}

static char *cfg_read_file(const char *path, char *error, size_t error_size) {
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        cfg_error(error, error_size, "cannot open scenario file");
        return NULL;
    }
    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        cfg_error(error, error_size, "cannot seek scenario file");
        return NULL;
    }
    long size = ftell(file);
    if (size < 0) {
        fclose(file);
        cfg_error(error, error_size, "cannot read scenario file size");
        return NULL;
    }
    rewind(file);
    char *text = (char *)malloc((size_t)size + 1U);
    if (text == NULL) {
        fclose(file);
        cfg_error(error, error_size, "out of memory reading scenario");
        return NULL;
    }
    const size_t got = fread(text, 1U, (size_t)size, file);
    fclose(file);
    text[got] = '\0';
    return text;
}

static const char *cfg_find_key(const char *begin, const char *end, const char *key) {
    char pattern[96];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const size_t len = strlen(pattern);
    for (const char *p = begin; p != NULL && p + len <= end; p++) {
        p = strstr(p, pattern);
        if (p == NULL || p + len > end) {
            return NULL;
        }
        return p;
    }
    return NULL;
}

static const char *cfg_matching(const char *open, const char *end, char open_ch, char close_ch) {
    int depth = 0;
    bool in_string = false;
    bool escaped = false;
    for (const char *p = open; p < end && *p != '\0'; p++) {
        if (in_string) {
            if (escaped) {
                escaped = false;
            } else if (*p == '\\') {
                escaped = true;
            } else if (*p == '"') {
                in_string = false;
            }
            continue;
        }
        if (*p == '"') {
            in_string = true;
        } else if (*p == open_ch) {
            depth++;
        } else if (*p == close_ch) {
            depth--;
            if (depth == 0) {
                return p;
            }
        }
    }
    return NULL;
}

static bool cfg_number(const char *begin, const char *end, const char *key, double *out) {
    const char *k = cfg_find_key(begin, end, key);
    if (k == NULL) {
        return false;
    }
    const char *colon = memchr(k, ':', (size_t)(end - k));
    if (colon == NULL) {
        return false;
    }
    char *after = NULL;
    const double value = strtod(colon + 1, &after);
    if (after == colon + 1 || after > end) {
        return false;
    }
    *out = value;
    return true;
}

static bool cfg_bool(const char *begin, const char *end, const char *key, bool *out) {
    const char *k = cfg_find_key(begin, end, key);
    if (k == NULL) {
        return false;
    }
    const char *colon = memchr(k, ':', (size_t)(end - k));
    if (colon == NULL) {
        return false;
    }
    const char *p = colon + 1;
    while (p < end && isspace((unsigned char)*p)) {
        p++;
    }
    if (p + 4 <= end && strncmp(p, "true", 4) == 0) {
        *out = true;
        return true;
    }
    if (p + 5 <= end && strncmp(p, "false", 5) == 0) {
        *out = false;
        return true;
    }
    return false;
}

static bool cfg_point_value(const char *begin, const char *end, const char *key, SoPoint *out) {
    const char *k = cfg_find_key(begin, end, key);
    if (k == NULL) {
        return false;
    }
    const char *brace = memchr(k, '{', (size_t)(end - k));
    if (brace == NULL) {
        return false;
    }
    const char *close = cfg_matching(brace, end, '{', '}');
    if (close == NULL) {
        return false;
    }
    double x = 0.0;
    double y = 0.0;
    if (!cfg_number(brace, close, "x", &x) || !cfg_number(brace, close, "y", &y)) {
        return false;
    }
    *out = cfg_point(x, y);
    return true;
}

static int cfg_parse_boundary_points(const char *begin, const char *end, SoPoint *points, int max_points) {
    const char *k = cfg_find_key(begin, end, "boundary_points");
    if (k == NULL) {
        return 0;
    }
    const char *open = memchr(k, '[', (size_t)(end - k));
    if (open == NULL) {
        return 0;
    }
    const char *close = cfg_matching(open, end, '[', ']');
    if (close == NULL) {
        return 0;
    }

    int count = 0;
    const char *p = open + 1;
    while (p < close && count < max_points) {
        const char *obj = memchr(p, '{', (size_t)(close - p));
        if (obj == NULL) {
            break;
        }
        const char *obj_end = cfg_matching(obj, close, '{', '}');
        if (obj_end == NULL) {
            break;
        }
        double x = 0.0;
        double y = 0.0;
        if (cfg_number(obj, obj_end, "x", &x) && cfg_number(obj, obj_end, "y", &y)) {
            points[count++] = cfg_point(x, y);
        }
        p = obj_end + 1;
    }
    return count;
}

static bool cfg_boundary_area_center(const char *begin, const char *end, double *area_ha, SoPoint *center) {
    SoPoint points[SO_MAX_BOUNDARY_POINTS];
    const int count = cfg_parse_boundary_points(begin, end, points, SO_MAX_BOUNDARY_POINTS);
    if (count < 3) {
        return false;
    }

    double twice_area = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    for (int i = 0; i < count; i++) {
        const SoPoint a = points[i];
        const SoPoint b = points[(i + 1) % count];
        const double cross = a.x * b.y - b.x * a.y;
        twice_area += cross;
        cx += (a.x + b.x) * cross;
        cy += (a.y + b.y) * cross;
    }
    if (fabs(twice_area) < 1e-9) {
        return false;
    }
    const double signed_area_m2 = twice_area / 2.0;
    const double area_m2 = fabs(signed_area_m2);
    cx /= (6.0 * signed_area_m2);
    cy /= (6.0 * signed_area_m2);
    *area_ha = area_m2 / 10000.0;
    *center = cfg_point(cx, cy);
    return true;
}

static bool cfg_array_bounds(const char *text, const char *key, const char **open, const char **close) {
    const char *end = text + strlen(text);
    const char *k = cfg_find_key(text, end, key);
    if (k == NULL) {
        return false;
    }
    *open = memchr(k, '[', (size_t)(end - k));
    if (*open == NULL) {
        return false;
    }
    *close = cfg_matching(*open, end, '[', ']');
    return *close != NULL;
}

static void cfg_parse_blocks(SoSimulation *sim, const char *text) {
    const char *open = NULL;
    const char *close = NULL;
    if (!cfg_array_bounds(text, "field_blocks", &open, &close)) {
        return;
    }

    sim->field.block_count = 0;
    sim->field.area_ha = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    const char *p = open + 1;
    while (p < close && sim->field.block_count < SO_MAX_BLOCKS) {
        const char *obj = memchr(p, '{', (size_t)(close - p));
        if (obj == NULL) {
            break;
        }
        const char *obj_end = cfg_matching(obj, close, '{', '}');
        if (obj_end == NULL) {
            break;
        }

        double id_value = (double)(sim->field.block_count + 1);
        double area = 0.0;
        double risk = 0.32;
        bool selected = true;
        SoPoint center = cfg_point(0.0, 0.0);
        SoPoint boundary[SO_MAX_BOUNDARY_POINTS];
        int boundary_count = cfg_parse_boundary_points(obj, obj_end, boundary, SO_MAX_BOUNDARY_POINTS);
        bool has_center = cfg_point_value(obj, obj_end, "center", &center);
        bool has_area = cfg_number(obj, obj_end, "area_hectares", &area);
        cfg_number(obj, obj_end, "id", &id_value);
        cfg_number(obj, obj_end, "risk", &risk);
        cfg_bool(obj, obj_end, "selected", &selected);

        if ((!has_center || !has_area) && cfg_boundary_area_center(obj, obj_end, &area, &center)) {
            has_center = true;
            has_area = true;
        }
        if (has_center && has_area && area > 0.001) {
            SoFieldBlock *block = &sim->field.blocks[sim->field.block_count++];
            block->id = (int)id_value;
            block->name = "manual field";
            block->center = center;
            block->area_ha = area;
            block->risk = fmax(0.0, fmin(1.0, risk));
            block->selected = selected;
            block->boundary_count = boundary_count;
            for (int b = 0; b < boundary_count; b++) {
                block->boundary[b] = boundary[b];
            }
            if (selected) {
                sim->field.area_ha += area;
                sum_x += center.x;
                sum_y += center.y;
            }
        }
        p = obj_end + 1;
    }

    if (sim->field.block_count > 0) {
        sim->field.boundary_center = cfg_point(sum_x / sim->field.block_count, sum_y / sim->field.block_count);
    }
}

static void cfg_parse_depots(SoSimulation *sim, const char *text) {
    const char *open = NULL;
    const char *close = NULL;
    if (!cfg_array_bounds(text, "depot_sites", &open, &close)) {
        return;
    }

    sim->field.depot_count = 0;
    const char *p = open + 1;
    while (p < close && sim->field.depot_count < SO_MAX_DEPOTS) {
        const char *obj = memchr(p, '{', (size_t)(close - p));
        if (obj == NULL) {
            break;
        }
        const char *obj_end = cfg_matching(obj, close, '{', '}');
        if (obj_end == NULL) {
            break;
        }

        SoPoint point;
        if (cfg_point_value(obj, obj_end, "point", &point)) {
            double id_value = (double)(sim->field.depot_count + 1);
            double usable_area = 420.0;
            double slope_risk = 0.18;
            bool road_accessible = true;
            cfg_number(obj, obj_end, "id", &id_value);
            cfg_number(obj, obj_end, "usable_area_m2", &usable_area);
            cfg_number(obj, obj_end, "slope_risk", &slope_risk);
            cfg_bool(obj, obj_end, "road_accessible", &road_accessible);
            sim->field.depots[sim->field.depot_count++] =
                (SoDepotSite){(int)id_value, point, usable_area, road_accessible, slope_risk};
        }
        p = obj_end + 1;
    }
}

static void cfg_generate_missing_depots(SoSimulation *sim) {
    if (sim->field.depot_count > 0) {
        return;
    }
    for (int i = 0; i < sim->field.block_count && sim->field.depot_count < SO_MAX_DEPOTS; i++) {
        const SoFieldBlock *block = &sim->field.blocks[i];
        sim->field.depots[sim->field.depot_count++] =
            (SoDepotSite){sim->field.depot_count + 1, cfg_point(block->center.x - 240.0, block->center.y - 260.0),
                          440.0, true, 0.15};
        if (sim->field.depot_count < SO_MAX_DEPOTS) {
            sim->field.depots[sim->field.depot_count++] =
                (SoDepotSite){sim->field.depot_count + 1, cfg_point(block->center.x + 240.0, block->center.y + 260.0),
                              380.0, true, 0.2};
        }
    }
}

bool so_load_manual_scout_config(SoSimulation *sim, const char *path, char *error, size_t error_size) {
    char *text = cfg_read_file(path, error, error_size);
    if (text == NULL) {
        return false;
    }

    so_init_default(sim);
    cfg_parse_blocks(sim, text);
    if (sim->field.block_count <= 0) {
        free(text);
        cfg_error(error, error_size, "scenario has no usable field_blocks");
        return false;
    }

    cfg_parse_depots(sim, text);
    cfg_generate_missing_depots(sim);
    cfg_number(text, text + strlen(text), "terrain_complexity", &sim->field.terrain_complexity);
    cfg_number(text, text + strlen(text), "obstacle_density", &sim->field.obstacle_density);

    sim->mothership.position = cfg_point(sim->field.boundary_center.x - 760.0, sim->field.boundary_center.y - 160.0);
    for (int i = 0; i < sim->drone_count; i++) {
        sim->drones[i].position = sim->mothership.position;
    }

    free(text);
    cfg_error(error, error_size, "");
    return true;
}

#include "sim_shim.h"

#include "flood_capture.h"
#include "flood_curbcheck.h"
#include "flood_geometry.h"
#include "flood_veto.h"
#include "device_constants.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The firmware core reads scene geometry through this extern. On device it is
 * defined in main/ai_process/flood_geometry_config.c; here the simulator owns
 * it and callers change it through sim_set_geometry(). */
flood_geometry_t g_flood_geometry = SIM_GEOMETRY_DEFAULT_INIT;

static const float k_default_geometry[SIM_GEOMETRY_LEN] = SIM_GEOMETRY_DEFAULT_INIT;

/* ------------------------------------------------------------ ABI / names */

int32_t sim_abi_version(void)   { return SIM_ABI_VERSION; }
int32_t sim_sizeof_box(void)    { return (int32_t)sizeof(sim_box_t); }
int32_t sim_sizeof_score(void)  { return (int32_t)sizeof(sim_score_t); }
int32_t sim_sizeof_result(void) { return (int32_t)sizeof(sim_result_t); }

const char *sim_outcome_name(int32_t outcome)
{
    if (outcome == SIM_OUT_SKIPPED_NO_REF) return "skipped_no_ref";
    if (outcome < FLOOD_OUT_UPLOAD || outcome > FLOOD_OUT_HEARTBEAT) return "unknown";
    return flood_outcome_name((flood_outcome_t)outcome);
}

const char *sim_severity_name(int32_t severity)
{
    if (severity < FLOOD_SEV_NONE || severity > FLOOD_SEV_SEVERE) return "unknown";
    return flood_severity_name((flood_severity_t)severity);
}

const char *sim_class_name(int32_t cls)
{
    /* flood_class_t order, flood_veto.h */
    static const char *const names[] = {
        "other", "person", "car", "truck", "bus", "bicycle", "motorcycle"
    };
    if (cls < 0 || cls >= (int32_t)(sizeof(names) / sizeof(names[0]))) return "unknown";
    return names[cls];
}

float   sim_submerged_below(void)  { return FLOOD_SUBMERGED_BELOW; }
float   sim_veto_threshold(void)   { return FLOOD_VETO_THRESHOLD; }
float   sim_sample_period_s(void)  { return SIM_SAMPLE_PERIOD_S; }
int32_t sim_upload_downscale(void) { return SIM_UPLOAD_DOWNSCALE; }

/* ---------------------------------------------------------------- geometry */

static void geometry_from_array(flood_geometry_t *g, const float *a)
{
    g->curb_x0 = a[0]; g->curb_y0 = a[1]; g->curb_x1 = a[2]; g->curb_y1 = a[3];
    g->lane_x0 = a[4]; g->lane_y0 = a[5]; g->lane_x1 = a[6]; g->lane_y1 = a[7];
}

static void geometry_to_array(const flood_geometry_t *g, float *a)
{
    a[0] = g->curb_x0; a[1] = g->curb_y0; a[2] = g->curb_x1; a[3] = g->curb_y1;
    a[4] = g->lane_x0; a[5] = g->lane_y0; a[6] = g->lane_x1; a[7] = g->lane_y1;
}

void sim_default_geometry(float *out8) { memcpy(out8, k_default_geometry, sizeof(k_default_geometry)); }
void sim_get_geometry(float *out8)     { geometry_to_array(&g_flood_geometry, out8); }
void sim_set_geometry(const float *in8) { geometry_from_array(&g_flood_geometry, in8); }

static float eval_line_y(float x0, float y0, float x1, float y1, float x)
{
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

static int32_t geometry_error(char *err, int32_t err_size, const char *msg)
{
    if (err && err_size > 0) snprintf(err, (size_t)err_size, "%s", msg);
    return 0;
}

/* Mirrors flood_geometry_config_save()'s checks, same messages:
 * main/ai_process/flood_geometry_config.c:118-160. */
int32_t sim_validate_geometry(const float *in8, char *err, int32_t err_size)
{
    flood_geometry_t c;
    char msg[160];
    geometry_from_array(&c, in8);

    if (c.curb_x0 == c.curb_x1)
        return geometry_error(err, err_size, "curb line endpoints must not share the same x coordinate");
    if (c.lane_x0 == c.lane_x1)
        return geometry_error(err, err_size, "lane line endpoints must not share the same x coordinate");

    const float pts_x[4] = { c.curb_x0, c.curb_x1, c.lane_x0, c.lane_x1 };
    const float pts_y[4] = { c.curb_y0, c.curb_y1, c.lane_y0, c.lane_y1 };
    for (int i = 0; i < 4; i++) {
        if (pts_x[i] < 0.0f || pts_x[i] > (float)FLOOD_FRAME_W ||
            pts_y[i] < 0.0f || pts_y[i] > (float)FLOOD_FRAME_H) {
            snprintf(msg, sizeof msg, "line points must be within the frame (0-%d, 0-%d)",
                     FLOOD_FRAME_W, FLOOD_FRAME_H);
            return geometry_error(err, err_size, msg);
        }
    }

    const float curb_min = c.curb_x0 < c.curb_x1 ? c.curb_x0 : c.curb_x1;
    const float curb_max = c.curb_x0 > c.curb_x1 ? c.curb_x0 : c.curb_x1;
    const float lane_min = c.lane_x0 < c.lane_x1 ? c.lane_x0 : c.lane_x1;
    const float lane_max = c.lane_x0 > c.lane_x1 ? c.lane_x0 : c.lane_x1;
    const float domain_x0 = curb_min > lane_min ? curb_min : lane_min;
    const float domain_x1 = curb_max < lane_max ? curb_max : lane_max;

    if (domain_x1 - domain_x0 < SIM_GEOMETRY_MIN_DOMAIN_WIDTH) {
        snprintf(msg, sizeof msg,
                 "curb and lane lines must overlap horizontally by at least %d px",
                 (int)SIM_GEOMETRY_MIN_DOMAIN_WIDTH);
        return geometry_error(err, err_size, msg);
    }

    const float curb_y_lo = eval_line_y(c.curb_x0, c.curb_y0, c.curb_x1, c.curb_y1, domain_x0);
    const float curb_y_hi = eval_line_y(c.curb_x0, c.curb_y0, c.curb_x1, c.curb_y1, domain_x1);
    const float lane_y_lo = eval_line_y(c.lane_x0, c.lane_y0, c.lane_x1, c.lane_y1, domain_x0);
    const float lane_y_hi = eval_line_y(c.lane_x0, c.lane_y0, c.lane_x1, c.lane_y1, domain_x1);
    if (lane_y_lo <= curb_y_lo || lane_y_hi <= curb_y_hi)
        return geometry_error(err, err_size,
                              "lane line must lie below the curb line across their shared range");
    return 1;
}

float sim_domain_x0(void) { return flood_domain_x0(); }
float sim_domain_x1(void) { return flood_domain_x1(); }

void sim_align_patch(int32_t *out4)
{
    out4[0] = FLOOD_ALIGN_X0; out4[1] = FLOOD_ALIGN_Y0;
    out4[2] = FLOOD_ALIGN_X1; out4[3] = FLOOD_ALIGN_Y1;
}

/* The columns flood_reference_init() samples (flood_curbcheck.c): starting
 * FLOOD_STRIP_OFFSET px into the domain, every FLOOD_STRIP_XSTEP px, the same
 * usable-column count formula. For drawing only; scoring never uses this. */
int32_t sim_strip_outline(float *xs, float *y_lo, float *y_hi, int32_t max_points)
{
    const float start = flood_domain_x0() + FLOOD_STRIP_OFFSET;
    int usable = (int)((flood_domain_x1() - start) / FLOOD_STRIP_XSTEP);
    if (usable > FLOOD_STRIP_COLS) usable = FLOOD_STRIP_COLS;
    if (usable < 0) usable = 0;
    if (usable > max_points) usable = max_points;
    for (int c = 0; c < usable; ++c) {
        const float x    = start + (float)(c * FLOOD_STRIP_XSTEP);
        const float curb = flood_curb_y(x);
        const float span = flood_lane_y(x) - curb;
        xs[c]   = x;
        y_lo[c] = curb + FLOOD_STRIP_LO_PCT / 100.0f * span;
        y_hi[c] = curb + FLOOD_STRIP_HI_PCT / 100.0f * span;
    }
    return usable;
}

/* ------------------------------------------------------------------ frames */

void sim_rgb_to_gray(const uint8_t *rgb, int32_t width, int32_t height, uint8_t *gray_out)
{
    const size_t n = (size_t)width * (size_t)height;
    for (size_t i = 0; i < n; ++i) {
        const uint8_t *p = rgb + i * 3;
        gray_out[i] = (uint8_t)((p[0] * SIM_GRAY_R + p[1] * SIM_GRAY_G + p[2] * SIM_GRAY_B)
                                >> SIM_GRAY_SHIFT);
    }
}

#include "sim_shim.h"

#include "flood_capture.h"
#include "flood_curbcheck.h"
#include "flood_gallery.h"
#include "flood_geometry.h"
#include "flood_veto.h"
#include "flood_upload_roi.h"
#include "flood_upload_meta.h"
#include "device_constants.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

/* --------------------------------------------------------------- reference */

static flood_gray_t full_frame(const uint8_t *gray)
{
    flood_gray_t f = { gray, SIM_FRAME_W, SIM_FRAME_H, SIM_FRAME_W };
    return f;
}

/* flood_gallery_t is ~151KB: always heap-allocated (flood_gallery.h). */
void *sim_gallery_new(void)
{
    flood_gallery_t *g = malloc(sizeof(*g));
    if (g) flood_gallery_init(g);
    return g;
}

void sim_gallery_free(void *gallery) { free(gallery); }

void sim_gallery_clear(void *gallery)
{
    if (gallery) flood_gallery_init((flood_gallery_t *)gallery);
}

int32_t sim_gallery_max_slots(void) { return FLOOD_GALLERY_MAX_SLOTS; }

int32_t sim_gallery_count(const void *gallery)
{
    return gallery ? flood_gallery_count((const flood_gallery_t *)gallery) : 0;
}

/* Same two steps as flood_task.c's flood_do_calibration(): build the reference,
 * then flood_gallery_add_manual(). */
int32_t sim_gallery_add_manual(void *gallery, const uint8_t *gray)
{
    const int32_t slot = sim_gallery_add_manual_slot(gallery, gray);
    return slot >= 0 ? 1 : (slot == -1 ? -1 : 0);
}

static const flood_gallery_slot_t *slot_at(const void *gallery, int32_t index)
{
    if (!gallery || index < 0 || index >= FLOOD_GALLERY_MAX_SLOTS) return NULL;
    const flood_gallery_slot_t *s = &((const flood_gallery_t *)gallery)->slots[index];
    return s->in_use ? s : NULL;
}

int32_t sim_gallery_slot(const void *gallery, int32_t index, int32_t *source,
                         uint32_t *last_matched_cycle)
{
    const flood_gallery_slot_t *s = slot_at(gallery, index);
    if (!s) return 0;
    if (source) *source = (int32_t)s->source;
    if (last_matched_cycle) *last_matched_cycle = s->last_matched_cycle;
    return 1;
}

int32_t sim_gallery_delete_slot(void *gallery, int32_t index, int32_t source)
{
    return flood_gallery_delete_slot((flood_gallery_t *)gallery, index,
                                     (flood_ref_source_t)source) ? 1 : 0;
}

int32_t sim_gallery_clear_auto(void *gallery)
{
    return flood_gallery_clear_auto((flood_gallery_t *)gallery);
}

int32_t sim_gallery_add_manual_slot(void *gallery, const uint8_t *gray)
{
    if (!gallery || !gray) return -2;
    static flood_reference_t ref;   /* ~19KB; the shim is single-threaded */
    const flood_gray_t dry = full_frame(gray);
    if (!flood_reference_init(&ref, &dry)) return -2;
    return flood_gallery_add_manual((flood_gallery_t *)gallery, &ref);
}

typedef struct {
    const uint8_t *const *frames;
    const int32_t        *status;
} sim_rebuild_src_t;

static flood_frame_status_t sim_rebuild_load(int slot, uint8_t *gray_out, void *user)
{
    const sim_rebuild_src_t *src = user;
    if (src->status[slot] == 2) return FLOOD_FRAME_ERROR;
    if (src->status[slot] != 0 || !src->frames[slot]) return FLOOD_FRAME_ABSENT;
    memcpy(gray_out, src->frames[slot], (size_t)SIM_FRAME_W * SIM_FRAME_H);
    return FLOOD_FRAME_OK;
}

int32_t sim_gallery_rebuild(void *gallery, const uint8_t *const frames[8], const int32_t status[8])
{
    if (!gallery || !frames || !status) return 1;
    flood_gallery_t *scratch = malloc(sizeof(*scratch));
    uint8_t *buf = malloc((size_t)SIM_FRAME_W * SIM_FRAME_H);
    int32_t rc = 2;                 /* an allocation failure is a RETRY, as on the device */
    if (scratch && buf) {
        sim_rebuild_src_t src = { frames, status };
        rc = (int32_t)flood_gallery_rebuild((flood_gallery_t *)gallery, scratch, buf,
                                            sim_rebuild_load, &src);
    }
    free(buf);
    free(scratch);
    return rc;
}

int32_t sim_gallery_size(void) { return (int32_t)sizeof(flood_gallery_t); }

float sim_correlation_at(const void *gallery, int32_t slot, const uint8_t *gray,
                         int32_t dx, int32_t dy)
{
    const flood_gallery_slot_t *s = slot_at(gallery, slot);
    if (!s || !gray) return 0.0f;
    const flood_gray_t f = full_frame(gray);
    return flood_curb_correlation_at(&s->ref, &f, dx, dy);
}

/* ------------------------------------------------------------------- boxes */

static flood_box_t to_flood_box(const sim_box_t *b)
{
    flood_box_t f;
    f.x0 = b->x0; f.y0 = b->y0; f.x1 = b->x1; f.y1 = b->y1;
    f.confidence = b->confidence;
    f.cls = (flood_class_t)b->cls;
    return f;
}

static int32_t clamp_box_count(int32_t n)
{
    return n < 0 ? 0 : (n > SIM_MAX_BOXES ? SIM_MAX_BOXES : n);
}

void sim_box_from_accelerator(float x0, float y0, float x1, float y1, float confidence,
                              int32_t cls, int32_t input_w, int32_t input_h, sim_box_t *out)
{
    const flood_box_t b = flood_box_from_accelerator(x0, y0, x1, y1, confidence,
                                                     (flood_class_t)cls, input_w, input_h);
    out->x0 = b.x0; out->y0 = b.y0; out->x1 = b.x1; out->y1 = b.y1;
    out->confidence = b.confidence;
    out->cls = (int32_t)b.cls;
}

/* Fraction of the kerb line this one box hides (firmware flood_band_coverage);
 * the GUI shows it on each box. */
float sim_band_coverage(const sim_box_t *box)
{
    if (!box) return 0.0f;
    const flood_box_t b = to_flood_box(box);
    return flood_band_coverage(&b);
}

int32_t sim_heartbeat_every(void) { return FLOOD_HEARTBEAT_EVERY; }

/* Dense scoring: the two firmware stages on one frame, no capture state. */
int32_t sim_score(const void *gallery, const uint8_t *gray, const sim_box_t *boxes,
                  int32_t n_boxes, sim_score_t *out)
{
    if (!out) return 0;
    memset(out, 0, sizeof(*out));
    out->gallery_slot = -1;
    if (sim_gallery_count(gallery) == 0 || !gray) return 0;

    flood_box_t fb[SIM_MAX_BOXES];
    const int32_t n = boxes ? clamp_box_count(n_boxes) : 0;
    for (int32_t i = 0; i < n; ++i) fb[i] = to_flood_box(&boxes[i]);
    const flood_veto_t v = flood_veto_check(fb, n);

    const flood_gray_t f = full_frame(gray);
    int slot = -1;
    const flood_curb_state_t cs = flood_gallery_best((const flood_gallery_t *)gallery, &f, true, &slot);

    out->veto_usable      = v.usable ? 1 : 0;
    out->veto_coverage    = v.coverage;
    out->veto_worst_class = (int32_t)v.worst_class;
    out->curb_correlation = cs.correlation;
    out->submerged        = cs.submerged ? 1 : 0;
    out->severity         = (int32_t)cs.severity;
    out->shift_dx         = cs.shift_dx;
    out->shift_dy         = cs.shift_dy;
    out->gallery_slot     = slot;
    return 1;
}

/* ------------------------------------------------------------- device loop */

typedef struct {
    flood_capture_t capture;
    uint8_t        *gray;
    sim_box_t       boxes[SIM_MAX_BOXES];
    int32_t         n_boxes;
    int32_t         attempt;
    int32_t         last_frame_idx;
    flood_veto_t    last_veto;
    sim_provider_fn provider;
    void           *user;
    flood_gallery_learner_t *learner;   /* pending learning candidate, like flood_task.c's */
    int32_t learning;   /* flood_task.c: no learning while the gallery is stale */
    int32_t heartbeat_every;
} sim_device_t;

void *sim_device_new(void)
{
    sim_device_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    d->heartbeat_every = FLOOD_HEARTBEAT_EVERY;
    d->gray = malloc((size_t)SIM_FRAME_W * SIM_FRAME_H);
    if (!d->gray) { free(d); return NULL; }
    d->learner = malloc(sizeof(*d->learner));
    if (!d->learner) { free(d->gray); free(d); return NULL; }
    flood_gallery_learner_init(d->learner);
    d->learning = 1;
    flood_capture_init(&d->capture);
    return d;
}

void sim_device_free(void *dev)
{
    sim_device_t *d = dev;
    if (!d) return;
    free(d->gray);
    free(d->learner);
    free(d);
}

void sim_device_reset(void *dev)
{
    sim_device_t *d = dev;
    if (d) {
        flood_capture_init(&d->capture);
        d->capture.heartbeat_every = d->heartbeat_every;   /* the setting survives a reset */
        flood_gallery_learner_init(d->learner);
    }
}

void sim_device_forget_candidate(void *dev)
{
    sim_device_t *d = dev;
    if (d) flood_gallery_learner_init(d->learner);
}

void sim_device_set_learning(void *dev, int32_t on)
{
    sim_device_t *d = dev;
    if (!d) return;
    d->learning = on ? 1 : 0;
    if (!on) flood_gallery_learner_init(d->learner);
}

void sim_device_set_heartbeat(void *dev, int32_t every)
{
    sim_device_t *d = dev;
    if (!d) return;
    d->heartbeat_every = every < 0 ? 0 : every;
    d->capture.heartbeat_every = d->heartbeat_every;
}

/* flood_grab_fn: attempt N of flood_capture_sample() asks the provider for
 * attempt N's frame (0 = sample time, 1 = retry). */
static bool device_grab(flood_gray_t *out, void *user)
{
    sim_device_t *d = user;
    const int32_t attempt = d->attempt++;
    int32_t frame_idx = -1;
    d->n_boxes = 0;
    if (!d->provider(attempt, d->gray, d->boxes, &d->n_boxes, &frame_idx, d->user)) return false;
    d->n_boxes = clamp_box_count(d->n_boxes);
    d->last_frame_idx = frame_idx;
    *out = full_frame(d->gray);
    return true;
}

/* flood_detect_fn: returns the boxes the provider delivered with the frame,
 * and records the veto for reporting (flood_capture_sample() computes the
 * same veto itself and does not expose it). */
static int device_detect(const flood_gray_t *frame, flood_box_t *boxes, int max_boxes, void *user)
{
    (void)frame;
    sim_device_t *d = user;
    const int n = d->n_boxes < max_boxes ? d->n_boxes : max_boxes;
    for (int i = 0; i < n; ++i) boxes[i] = to_flood_box(&d->boxes[i]);
    d->last_veto = flood_veto_check(boxes, n);
    return n;
}

void sim_device_sample(void *dev, void *gallery, sim_provider_fn provider, void *user,
                       sim_result_t *out)
{
    sim_device_t *d = dev;
    flood_gallery_t *g = gallery;
    memset(out, 0, sizeof(*out));
    out->frame_idx = -1;
    out->gallery_slot = -1;
    out->gallery_source = -1;
    out->gallery_count = sim_gallery_count(g);
    if (!d || !provider) {
        out->outcome = SIM_OUT_SKIPPED_NO_REF;
        return;
    }
    /* flood_task.c:365 -- an empty gallery is never sampled: the firmware
     * would otherwise read correlation 0 as a flood on every sample. */
    if (out->gallery_count == 0) {
        out->outcome = SIM_OUT_SKIPPED_NO_REF;
        out->since_image = d->capture.since_image;
        out->candidate_pending = d->learner->pending ? 1 : 0;
        return;
    }

    d->capture.heartbeat_every = d->heartbeat_every;   /* flood_task.c:342 */
    d->attempt = 0;
    d->last_frame_idx = -1;
    d->last_veto.usable = true;
    d->last_veto.coverage = 0.0f;
    d->last_veto.worst_class = FLOOD_CLASS_OTHER;
    d->provider = provider;
    d->user = user;

    flood_gray_t sampled;
    const flood_decision_t dec = flood_capture_sample(&d->capture, g, device_grab, device_detect, d,
                                                      &sampled);

    out->outcome          = (int32_t)dec.outcome;
    out->frames_taken     = dec.frames_taken;
    out->has_reading      = dec.has_reading ? 1 : 0;
    out->curb_correlation = dec.curb_correlation;
    out->severity         = (int32_t)dec.severity;
    out->veto_usable      = d->last_veto.usable ? 1 : 0;
    out->veto_coverage    = d->last_veto.coverage;
    out->veto_worst_class = (int32_t)d->last_veto.worst_class;
    out->since_image      = d->capture.since_image;
    out->frame_idx        = d->last_frame_idx;
    if (dec.has_reading && dec.gallery_index >= 0) {
        const flood_gallery_slot_t *slot = &g->slots[dec.gallery_index];
        out->gallery_slot   = dec.gallery_index;
        out->gallery_source = (int32_t)slot->source;
        /* The decision does not carry the registration shift, so recompute it
         * with the same firmware function flood_curbcheck() used. Before
         * after_sample: admission may evict and overwrite this very slot. */
        flood_estimate_shift(&slot->ref, &sampled, &out->shift_dx, &out->shift_dy);

        /* flood_task.c:375-377 -- the gallery bookkeeping after every reading. */
        bool admitted = false, notice = false;
        float notice_corr = 0.0f;
        flood_gallery_after_sample(g, d->learning ? d->learner : NULL, &sampled, dec.curb_correlation, dec.gallery_index,
                                   &admitted, &notice, &notice_corr);
        out->admitted = admitted ? 1 : 0;
        out->notice   = notice ? 1 : 0;
    }
    out->gallery_count = sim_gallery_count(g);
    out->candidate_pending = d->learner->pending ? 1 : 0;
}

/* ----------------------------------------------------------------- uploads */

static flood_upload_roi_t roi_from_array(const int32_t *a)
{
    flood_upload_roi_t r;
    r.rect = (flood_rect_t){ a[0], a[1], a[2], a[3] };
    r.step = a[4];
    r.source = (flood_upload_src_t)a[5];
    return r;
}

int32_t sim_upload_max_pixels(void) { return FLOOD_UPLOAD_ROI_MAX_PIXELS; }
int32_t sim_upload_min_side(void)   { return FLOOD_UPLOAD_ROI_MIN_SIDE; }

int32_t sim_upload_rect_valid(const int32_t *rect4)
{
    if (!rect4) return 0;
    const flood_rect_t r = { rect4[0], rect4[1], rect4[2], rect4[3] };
    return flood_upload_rect_valid(&r) ? 1 : 0;
}

void sim_upload_roi(const int32_t *drawn4, int32_t *out8)
{
    flood_rect_t drawn;
    if (drawn4) drawn = (flood_rect_t){ drawn4[0], drawn4[1], drawn4[2], drawn4[3] };
    const flood_upload_roi_t r = flood_upload_roi_compute(drawn4 ? &drawn : NULL);
    out8[0] = r.rect.x; out8[1] = r.rect.y; out8[2] = r.rect.width; out8[3] = r.rect.height;
    out8[4] = r.step; out8[5] = (int32_t)r.source;
    out8[6] = flood_upload_roi_out_w(&r); out8[7] = flood_upload_roi_out_h(&r);
}

void sim_upload_crop(const uint8_t *rgb24, const int32_t *roi8, uint8_t *out)
{
    if (!rgb24 || !roi8 || !out) return;
    const flood_upload_roi_t r = roi_from_array(roi8);
    flood_upload_roi_copy(rgb24, SIM_FRAME_W, &r, out);
}

int32_t sim_upload_filename(char *out, int32_t size, const char *sn, int32_t heartbeat,
                            int32_t year, int32_t month, int32_t day,
                            int32_t hour, int32_t minute, int32_t second)
{
    struct tm local;
    memset(&local, 0, sizeof local);
    local.tm_year = year - 1900; local.tm_mon = month - 1; local.tm_mday = day;
    local.tm_hour = hour; local.tm_min = minute; local.tm_sec = second;
    return flood_upload_filename(out, (size_t)size, sn, heartbeat != 0, &local);
}

int32_t sim_upload_description(char *out, int32_t size, int32_t heartbeat, const char *id,
                               float correlation, const char *severity, int32_t slot,
                               const int32_t *roi8, int64_t captured_at)
{
    const flood_upload_meta_t m = {
        .heartbeat = heartbeat != 0, .id = id, .correlation = correlation, .severity = severity,
        .slot = slot, .roi = roi_from_array(roi8), .captured_at = (long long)captured_at,
    };
    return flood_upload_description(out, (size_t)size, &m);
}

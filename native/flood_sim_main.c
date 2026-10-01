/* flood_sim.exe: the firmware flood detector on PGM frames, no Python.
 * Output: one JSON line per device sample or dense frame, same fields as
 * floodsim's samples.csv. */
#include "sim_io.h"
#include "sim_shim.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#define MAX_REFERENCES 8   /* FLOOD_GALLERY_MAX_SLOTS */

typedef struct {
    const char *references[MAX_REFERENCES];   /* each a manual gallery slot */
    int n_references;
    const char *frames, *boxes, *geometry, *mode;
    double period, retry_gap, fps;
    int every;
} options_t;

static void usage(void)
{
    fputs("usage: flood_sim --reference dry.pgm [--reference dusk.pgm ...] --frames <dir|list>\n"
          "                 [--boxes boxes.csv]\n"
          "                 [--geometry geo.json] [--mode device|dense] [--period 60]\n"
          "                 [--retry-gap 1] [--every 1] [--fps 1]\n", stderr);
}

static int parse_args(int argc, char **argv, options_t *o)
{
    memset(o, 0, sizeof(*o));
    o->mode = "device";
    o->period = sim_sample_period_s();
    o->retry_gap = 1.0;
    o->fps = 1.0;
    o->every = 1;
    for (int i = 1; i < argc; i += 2) {
        const char *a = argv[i];
        const char *v = i + 1 < argc ? argv[i + 1] : NULL;
        if (!v) { fprintf(stderr, "missing value for %s\n", a); return 0; }
        if (!strcmp(a, "--reference")) {
            if (o->n_references == MAX_REFERENCES) {
                fprintf(stderr, "at most %d --reference images\n", MAX_REFERENCES);
                return 0;
            }
            o->references[o->n_references++] = v;
        }
        else if (!strcmp(a, "--frames")) o->frames = v;
        else if (!strcmp(a, "--boxes")) o->boxes = v;
        else if (!strcmp(a, "--geometry")) o->geometry = v;
        else if (!strcmp(a, "--mode")) o->mode = v;
        else if (!strcmp(a, "--period")) o->period = atof(v);
        else if (!strcmp(a, "--retry-gap")) o->retry_gap = atof(v);
        else if (!strcmp(a, "--every")) o->every = atoi(v);
        else if (!strcmp(a, "--fps")) o->fps = atof(v);
        else { fprintf(stderr, "unknown option %s\n", a); return 0; }
    }
    if (!o->n_references || !o->frames) { fputs("--reference and --frames are required\n", stderr); return 0; }
    if (strcmp(o->mode, "device") != 0 && strcmp(o->mode, "dense") != 0) {
        fputs("--mode must be device or dense\n", stderr);
        return 0;
    }
    if (o->period <= 0 || o->retry_gap < 0 || o->every < 1 || o->fps <= 0) {
        fputs("--period and --fps must be > 0, --retry-gap >= 0, --every >= 1\n", stderr);
        return 0;
    }
    return 1;
}

static const char *json_bool(int32_t v) { return v ? "true" : "false"; }

/* flood_ref_source_t: 0 = MANUAL, 1 = AUTO; -1 = no slot matched. */
static const char *source_name(int32_t source)
{
    return source == 0 ? "manual" : source == 1 ? "auto" : "none";
}

static void print_device(int sample, double t, const sim_result_t *r)
{
    printf("{\"mode\":\"device\",\"sample\":%d,\"t_sec\":%.3f,\"frame_idx\":%d,\"outcome\":\"%s\","
           "\"frames_taken\":%d,\"has_reading\":%s,\"curb_correlation\":%.6f,\"severity\":\"%s\","
           "\"shift_dx\":%d,\"shift_dy\":%d,\"veto_usable\":%s,\"veto_coverage\":%.6f,"
           "\"veto_worst_class\":\"%s\",\"since_image\":%d,\"gallery_slot\":%d,"
           "\"gallery_source\":\"%s\",\"admitted\":%s,\"notice\":%s,\"gallery_count\":%d,\"candidate_pending\":%s}\n",
           sample, t, r->frame_idx, sim_outcome_name(r->outcome), r->frames_taken,
           json_bool(r->has_reading), r->curb_correlation, sim_severity_name(r->severity),
           r->shift_dx, r->shift_dy, json_bool(r->veto_usable), r->veto_coverage,
           sim_class_name(r->veto_worst_class), r->since_image, r->gallery_slot,
           source_name(r->gallery_source), json_bool(r->admitted), json_bool(r->notice),
           r->gallery_count, json_bool(r->candidate_pending));
}

static void print_dense(int32_t frame_idx, double t, const sim_score_t *s)
{
    printf("{\"mode\":\"dense\",\"frame_idx\":%d,\"t_sec\":%.3f,\"veto_usable\":%s,"
           "\"veto_coverage\":%.6f,\"veto_worst_class\":\"%s\",\"curb_correlation\":%.6f,"
           "\"submerged\":%s,\"severity\":\"%s\",\"shift_dx\":%d,\"shift_dy\":%d,"
           "\"gallery_slot\":%d}\n",
           frame_idx, t, json_bool(s->veto_usable), s->veto_coverage,
           sim_class_name(s->veto_worst_class), s->curb_correlation, json_bool(s->submerged),
           sim_severity_name(s->severity), s->shift_dx, s->shift_dy, s->gallery_slot);
}

typedef struct {
    const sim_frame_t   *frames;
    int                  n_frames;
    const sim_box_rec_t *boxes;
    int                  n_boxes;
    double               times[2];   /* sample time, retry time */
} provider_ctx_t;

static int32_t provide_frame(int32_t attempt, uint8_t *gray_out, sim_box_t *boxes_out,
                             int32_t *n_boxes, int32_t *frame_idx, void *user)
{
    provider_ctx_t *c = user;
    char err[512];
    const int i = sim_frame_at(c->frames, c->n_frames, c->times[attempt < 1 ? 0 : 1]);
    if (i < 0) return 0;
    if (!sim_read_pgm(c->frames[i].path, gray_out, err, sizeof err)) {
        fprintf(stderr, "warning: %s\n", err);
        return 0;   /* a failed grab, as a camera timeout is on device */
    }
    *frame_idx = c->frames[i].frame_idx;
    *n_boxes = sim_boxes_for(c->boxes, c->n_boxes, *frame_idx, boxes_out, SIM_MAX_BOXES);
    return 1;
}

static int fail(const char *msg)
{
    fprintf(stderr, "error: %s\n", msg);
    return 2;
}

int main(int argc, char **argv)
{
#ifdef _WIN32
    _setmode(1, _O_BINARY);   /* stdout: LF endings, byte-identical to the golden files */
#endif
    options_t o;
    char err[512];
    if (!parse_args(argc, argv, &o)) {
        usage();
        return 2;
    }

    if (o.geometry) {
        float g[SIM_GEOMETRY_LEN];
        if (!sim_read_geometry_json(o.geometry, g, err, sizeof err)) return fail(err);
        if (!sim_validate_geometry(g, err, (int32_t)sizeof err)) return fail(err);
        sim_set_geometry(g);
    }

    uint8_t *gray = malloc((size_t)SIM_FRAME_W * SIM_FRAME_H);
    void *gallery = sim_gallery_new();
    if (!gray || !gallery) return fail("out of memory");
    for (int i = 0; i < o.n_references; ++i) {
        if (!sim_read_pgm(o.references[i], gray, err, sizeof err)) return fail(err);
        if (sim_gallery_add_manual(gallery, gray) != 1)
            return fail("geometry leaves no usable sampling window");
    }

    sim_frame_t *frames = NULL;
    const int n = sim_list_frames(o.frames, o.fps, &frames, err, sizeof err);
    if (n < 0) return fail(err);

    sim_box_rec_t *boxes = NULL;
    int n_boxes = 0;
    if (o.boxes) {
        n_boxes = sim_read_boxes(o.boxes, &boxes, err, sizeof err);
        if (n_boxes < 0) return fail(err);
    }

    if (strcmp(o.mode, "device") == 0) {
        void *dev = sim_device_new();
        if (!dev) return fail("out of memory");
        provider_ctx_t ctx = { frames, n, boxes, n_boxes, { 0.0, 0.0 } };
        const double last_t = frames[n - 1].t_sec;
        for (int k = 0;; ++k) {
            const double t = k * o.period;
            if (t > last_t + 1e-9) break;
            ctx.times[0] = t;
            ctx.times[1] = t + o.retry_gap;
            sim_result_t r;
            sim_device_sample(dev, gallery, provide_frame, &ctx, &r);
            print_device(k, t, &r);
        }
        sim_device_free(dev);
    } else {
        sim_box_t fb[SIM_MAX_BOXES];
        for (int i = 0; i < n; i += o.every) {
            if (!sim_read_pgm(frames[i].path, gray, err, sizeof err)) {
                fprintf(stderr, "warning: %s\n", err);
                continue;
            }
            const int nfb = sim_boxes_for(boxes, n_boxes, frames[i].frame_idx, fb, SIM_MAX_BOXES);
            sim_score_t s;
            sim_score(gallery, gray, fb, nfb, &s);
            print_dense(frames[i].frame_idx, frames[i].t_sec, &s);
        }
    }

    free(boxes);
    free(frames);
    sim_gallery_free(gallery);
    free(gray);
    return 0;
}

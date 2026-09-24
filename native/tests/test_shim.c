#include "sim_shim.h"
#include "synth.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_failures = 0;
#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    ++g_failures; } } while (0)

static void test_abi_and_names(void)
{
    CHECK(sim_abi_version() == SIM_ABI_VERSION);
    CHECK(sim_sizeof_box() == (int32_t)sizeof(sim_box_t));
    CHECK(sim_sizeof_score() == (int32_t)sizeof(sim_score_t));
    CHECK(sim_sizeof_result() == (int32_t)sizeof(sim_result_t));
    CHECK(strcmp(sim_outcome_name(0), "upload") == 0);
    CHECK(strcmp(sim_outcome_name(2), "blocked") == 0);
    CHECK(strcmp(sim_outcome_name(SIM_OUT_SKIPPED_NO_REF), "skipped_no_ref") == 0);
    CHECK(strcmp(sim_outcome_name(9), "unknown") == 0);
    CHECK(strcmp(sim_severity_name(2), "moderate") == 0);
    CHECK(strcmp(sim_class_name(2), "car") == 0);
    CHECK(strcmp(sim_class_name(99), "unknown") == 0);
    CHECK(sim_submerged_below() == 0.45f);
    CHECK(sim_veto_threshold() == 0.15f);
    CHECK(sim_sample_period_s() == 60.0f);
    CHECK(sim_upload_downscale() == 2);
}

static void test_gray(void)
{
    const uint8_t rgb[6] = { 255, 0, 0, 10, 200, 30 };
    uint8_t gray[2];
    sim_rgb_to_gray(rgb, 2, 1, gray);
    CHECK(gray[0] == ((255 * 77) >> 8));
    CHECK(gray[1] == ((10 * 77 + 200 * 151 + 30 * 28) >> 8));
}

static void test_geometry(void)
{
    float d[8], cur[8], bad[8];
    char err[128];
    sim_default_geometry(d);
    CHECK(d[0] == 560.0f && d[1] == 230.0f && d[6] == 1920.0f && d[7] == 446.0f);
    sim_get_geometry(cur);
    CHECK(memcmp(d, cur, sizeof d) == 0);
    CHECK(sim_validate_geometry(d, err, sizeof err) == 1);

    memcpy(bad, d, sizeof d);
    bad[2] = bad[0];
    CHECK(sim_validate_geometry(bad, err, sizeof err) == 0);
    CHECK(strstr(err, "curb line endpoints") != NULL);

    memcpy(bad, d, sizeof d);
    bad[5] = 100.0f;
    bad[7] = 100.0f;
    CHECK(sim_validate_geometry(bad, err, sizeof err) == 0);
    CHECK(strstr(err, "below the curb") != NULL);

    memcpy(bad, d, sizeof d);
    bad[1] = -5.0f;
    CHECK(sim_validate_geometry(bad, err, sizeof err) == 0);
    CHECK(strstr(err, "within the frame") != NULL);

    float moved[8];
    memcpy(moved, d, sizeof d);
    moved[1] = 240.0f;
    sim_set_geometry(moved);
    sim_get_geometry(cur);
    CHECK(cur[1] == 240.0f);
    sim_set_geometry(d);
}

static void test_strip_outline(void)
{
    float xs[512], lo[512], hi[512];
    const int32_t n = sim_strip_outline(xs, lo, hi, 512);
    CHECK(n == 275);
    CHECK(xs[0] == 700.0f);
    CHECK(lo[0] < hi[0]);
    int32_t a[4];
    sim_align_patch(a);
    CHECK(a[0] == 300 && a[1] == 0 && a[2] == 1700 && a[3] == 200);
    CHECK(sim_domain_x0() == 560.0f && sim_domain_x1() == 1920.0f);
}

static uint8_t *g_dry;
static uint8_t *g_flood;
static const sim_box_t k_car = { 0.0f, 0.0f, 1919.0f, 1079.0f, 0.9f, 2 };

static void test_reference_and_score(void)
{
    sim_score_t s;
    void *ref = sim_ref_new();
    CHECK(ref != NULL);
    CHECK(!sim_ref_ready(ref));
    CHECK(sim_score(ref, g_dry, NULL, 0, &s) == 0);
    CHECK(sim_score(NULL, g_dry, NULL, 0, &s) == 0);

    CHECK(sim_ref_build(ref, g_dry) == 1);
    CHECK(sim_ref_ready(ref));

    CHECK(sim_score(ref, g_dry, NULL, 0, &s) == 1);
    CHECK(fabsf(s.curb_correlation - 1.0f) < 1e-6f);
    CHECK(!s.submerged);
    CHECK(s.severity == 0);
    CHECK(s.shift_dx == 0 && s.shift_dy == 0);
    CHECK(s.veto_usable && s.veto_coverage == 0.0f && s.veto_worst_class == 0);

    CHECK(sim_score(ref, g_flood, NULL, 0, &s) == 1);
    CHECK(s.curb_correlation == 0.0f);
    CHECK(s.submerged);
    CHECK(s.severity == 2);

    CHECK(sim_score(ref, g_dry, &k_car, 1, &s) == 1);
    CHECK(!s.veto_usable);
    CHECK(s.veto_coverage == 1.0f);
    CHECK(s.veto_worst_class == 2);

    CHECK(fabsf(sim_correlation_at(ref, g_dry, 0, 0) - 1.0f) < 1e-6f);

    sim_box_t b;
    sim_box_from_accelerator(10.0f, 20.0f, 30.0f, 40.0f, 0.5f, 1, 640, 480, &b);
    CHECK(b.x0 == 30.0f && b.y0 == 45.0f && b.x1 == 90.0f && b.y1 == 90.0f);
    CHECK(b.confidence == 0.5f && b.cls == 1);
    sim_ref_free(ref);
}

typedef struct {
    const uint8_t *frames[2];   /* NULL = grab fails */
    int32_t n_boxes[2];         /* 0 or 1 (k_car) */
    int32_t idx[2];
    int calls;
} script_t;

static int32_t scripted(int32_t attempt, uint8_t *gray_out, sim_box_t *boxes_out,
                        int32_t *n_boxes, int32_t *frame_idx, void *user)
{
    script_t *s = user;
    s->calls++;
    if (attempt > 1 || s->frames[attempt] == NULL) return 0;
    memcpy(gray_out, s->frames[attempt], (size_t)SYNTH_W * SYNTH_H);
    *n_boxes = s->n_boxes[attempt];
    if (*n_boxes) boxes_out[0] = k_car;
    *frame_idx = s->idx[attempt];
    return 1;
}

static void test_device(void)
{
    sim_result_t r;
    void *ref = sim_ref_new();
    void *dev = sim_device_new();
    sim_ref_build(ref, g_dry);

    script_t clear = { { g_dry, g_dry }, { 0, 0 }, { 0, 1 }, 0 };
    sim_device_sample(dev, ref, scripted, &clear, &r);
    CHECK(r.outcome == 1 && r.frames_taken == 1 && r.has_reading);
    CHECK(r.since_image == 1 && r.frame_idx == 0 && clear.calls == 1);
    CHECK(fabsf(r.curb_correlation - 1.0f) < 1e-6f);

    script_t flood = { { g_flood, g_flood }, { 0, 0 }, { 2, 3 }, 0 };
    sim_device_sample(dev, ref, scripted, &flood, &r);
    CHECK(r.outcome == 0 && r.severity == 2 && r.since_image == 0 && r.frame_idx == 2);

    script_t retry = { { g_dry, g_dry }, { 1, 0 }, { 4, 5 }, 0 };
    sim_device_sample(dev, ref, scripted, &retry, &r);
    CHECK(r.outcome == 1 && r.frames_taken == 2 && r.frame_idx == 5);
    CHECK(r.veto_usable && r.veto_coverage == 0.0f);

    script_t blocked = { { g_dry, g_dry }, { 1, 1 }, { 6, 7 }, 0 };
    sim_device_sample(dev, ref, scripted, &blocked, &r);
    CHECK(r.outcome == 2 && !r.has_reading && r.frames_taken == 2);
    CHECK(!r.veto_usable && r.veto_worst_class == 2 && r.since_image == 2);

    script_t none = { { NULL, NULL }, { 0, 0 }, { 0, 0 }, 0 };
    sim_device_sample(dev, ref, scripted, &none, &r);
    CHECK(r.outcome == 2 && r.frames_taken == 2 && r.frame_idx == -1);

    /* Heartbeat: the 60th consecutive reading without a flood. */
    sim_device_reset(dev);
    for (int i = 0; i < 59; ++i) {
        sim_device_sample(dev, ref, scripted, &clear, &r);
        CHECK(r.outcome == 1);
    }
    sim_device_sample(dev, ref, scripted, &clear, &r);
    CHECK(r.outcome == 3 && r.since_image == 0);

    /* Unready reference: skipped, capture state untouched (flood_task.c:271). */
    void *unready = sim_ref_new();
    sim_device_sample(dev, unready, scripted, &clear, &r);
    CHECK(r.outcome == SIM_OUT_SKIPPED_NO_REF && r.since_image == 0 && r.frame_idx == -1);
    sim_device_sample(dev, NULL, scripted, &clear, &r);
    CHECK(r.outcome == SIM_OUT_SKIPPED_NO_REF);

    sim_ref_free(unready);
    sim_device_free(dev);
    sim_ref_free(ref);
}

int main(void)
{
    g_dry = malloc((size_t)SYNTH_W * SYNTH_H);
    g_flood = malloc((size_t)SYNTH_W * SYNTH_H);
    if (!g_dry || !g_flood) return 2;
    synth_dry(g_dry);
    synth_flooded(g_flood);

    test_abi_and_names();
    test_gray();
    test_geometry();
    test_strip_outline();
    test_reference_and_score();
    test_device();

    free(g_dry);
    free(g_flood);
    if (g_failures) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_shim: all checks passed");
    return 0;
}

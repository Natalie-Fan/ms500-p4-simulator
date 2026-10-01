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
static uint8_t *g_relit;   /* dry, different lighting: learnable */
static uint8_t *g_night;   /* dry, too different: ambiguous band */
static const sim_box_t k_car = { 0.0f, 0.0f, 1919.0f, 1079.0f, 0.9f, 2 };

static void test_reference_and_score(void)
{
    sim_score_t s;
    void *ref = sim_gallery_new();
    CHECK(ref != NULL);
    CHECK(sim_gallery_count(ref) == 0);
    CHECK(sim_gallery_max_slots() == 8);
    CHECK(sim_score(ref, g_dry, NULL, 0, &s) == 0 && s.gallery_slot == -1);
    CHECK(sim_score(NULL, g_dry, NULL, 0, &s) == 0);

    CHECK(sim_gallery_add_manual(ref, g_dry) == 1);
    CHECK(sim_gallery_count(ref) == 1);
    int32_t source = -1;
    uint32_t last = 99;
    CHECK(sim_gallery_slot(ref, 0, &source, &last) == 1 && source == 0 && last == 0);
    CHECK(sim_gallery_slot(ref, 1, &source, &last) == 0);
    CHECK(sim_gallery_slot(ref, -1, NULL, NULL) == 0 && sim_gallery_slot(ref, 8, NULL, NULL) == 0);

    CHECK(sim_score(ref, g_dry, NULL, 0, &s) == 1);
    CHECK(s.gallery_slot == 0);
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

    CHECK(fabsf(sim_correlation_at(ref, 0, g_dry, 0, 0) - 1.0f) < 1e-6f);
    CHECK(sim_correlation_at(ref, 3, g_dry, 0, 0) == 0.0f);

    /* Dense scoring keeps the best slot and never admits. */
    CHECK(sim_gallery_add_manual(ref, g_relit) == 1);
    CHECK(sim_score(ref, g_relit, NULL, 0, &s) == 1);
    CHECK(s.gallery_slot == 1 && fabsf(s.curb_correlation - 1.0f) < 1e-6f);
    CHECK(sim_score(ref, g_night, NULL, 0, &s) == 1);
    CHECK(sim_gallery_count(ref) == 2);

    sim_gallery_clear(ref);
    CHECK(sim_gallery_count(ref) == 0);

    sim_box_t b;
    sim_box_from_accelerator(10.0f, 20.0f, 30.0f, 40.0f, 0.5f, 1, 640, 480, &b);
    CHECK(b.x0 == 30.0f && b.y0 == 45.0f && b.x1 == 90.0f && b.y1 == 90.0f);
    CHECK(b.confidence == 0.5f && b.cls == 1);
    sim_gallery_free(ref);
}

static void test_gallery_full_of_manual(void)
{
    void *g = sim_gallery_new();
    for (int i = 0; i < 8; ++i) CHECK(sim_gallery_add_manual(g, g_dry) == 1);
    CHECK(sim_gallery_add_manual(g, g_dry) == -1);
    CHECK(sim_gallery_count(g) == 8);
    sim_gallery_free(g);
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
    void *ref = sim_gallery_new();
    void *dev = sim_device_new();
    sim_gallery_add_manual(ref, g_dry);

    script_t clear = { { g_dry, g_dry }, { 0, 0 }, { 0, 1 }, 0 };
    sim_device_sample(dev, ref, scripted, &clear, &r);
    CHECK(r.outcome == 1 && r.frames_taken == 1 && r.has_reading);
    CHECK(r.since_image == 1 && r.frame_idx == 0 && clear.calls == 1);
    CHECK(fabsf(r.curb_correlation - 1.0f) < 1e-6f);
    CHECK(r.gallery_slot == 0 && r.gallery_source == 0 && r.gallery_count == 1);
    CHECK(!r.admitted && !r.notice);   /* 1.0 >= FLOOD_GALLERY_NEAR_DUP_MAX: already covered */

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
    CHECK(r.gallery_slot == -1 && r.gallery_source == -1);

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

    /* Empty gallery: skipped, capture state untouched (flood_task.c:300). */
    void *empty = sim_gallery_new();
    sim_device_sample(dev, empty, scripted, &clear, &r);
    CHECK(r.outcome == SIM_OUT_SKIPPED_NO_REF && r.since_image == 0 && r.frame_idx == -1);
    CHECK(r.gallery_count == 0);
    sim_device_sample(dev, NULL, scripted, &clear, &r);
    CHECK(r.outcome == SIM_OUT_SKIPPED_NO_REF);

    sim_gallery_free(empty);
    sim_device_free(dev);
    sim_gallery_free(ref);
}

/* The flood_task.c gallery bookkeeping: a confidently-dry but different frame
 * is learned as an AUTO slot; a run of ambiguous readings raises one notice. */
static void test_device_learning(void)
{
    sim_result_t r;
    sim_score_t s;
    void *g = sim_gallery_new();
    void *dev = sim_device_new();
    sim_gallery_add_manual(g, g_dry);

    /* The fixtures must land in the bands they stand for. */
    sim_score(g, g_relit, NULL, 0, &s);
    CHECK(s.curb_correlation >= 0.65f && s.curb_correlation < 0.95f);
    sim_score(g, g_night, NULL, 0, &s);
    CHECK(s.curb_correlation >= 0.45f && s.curb_correlation < 0.65f);

    script_t relit = { { g_relit, g_relit }, { 0, 0 }, { 10, 11 }, 0 };
    sim_device_sample(dev, g, scripted, &relit, &r);            /* candidate only */
    CHECK(r.has_reading && r.gallery_slot == 0 && r.gallery_source == 0);
    CHECK(!r.admitted && r.candidate_pending && r.gallery_count == 1);
    sim_device_sample(dev, g, scripted, &relit, &r);            /* proven: becomes a slot */
    CHECK(r.admitted && !r.candidate_pending && r.gallery_count == 2);
    int32_t source = -1;
    CHECK(sim_gallery_slot(g, 1, &source, NULL) == 1 && source == 1);

    /* The same frame now matches the learned slot and is not learned again. */
    sim_device_sample(dev, g, scripted, &relit, &r);
    CHECK(r.gallery_slot == 1 && r.gallery_source == 1);
    CHECK(fabsf(r.curb_correlation - 1.0f) < 1e-6f);
    CHECK(!r.admitted && !r.candidate_pending && r.gallery_count == 2);

    /* FLOOD_GALLERY_AMBIGUOUS_STREAK_CYCLES (5) ambiguous readings in a row:
     * the notice fires on the 5th only, and nothing is learned. */
    script_t night = { { g_night, g_night }, { 0, 0 }, { 12, 13 }, 0 };
    int notices = 0;
    for (int i = 0; i < 5; ++i) {
        sim_device_sample(dev, g, scripted, &night, &r);
        CHECK(!r.admitted && r.outcome == 1);
        CHECK(r.notice == (i == 4));
        notices += r.notice;
    }
    sim_device_sample(dev, g, scripted, &night, &r);
    CHECK(!r.notice && notices == 1 && r.gallery_count == 2);

    sim_device_free(dev);
    sim_gallery_free(g);
}

#define ANCHOR 0.55f               /* FLOOD_GALLERY_MANUAL_ANCHOR_MIN */
#define LIP    (1.0f / 7.0f)       /* water level of the gutter lip alone */

/* Water climbs the curb face from the gutter lip in 2 % steps, one per sample. Each step
 * resembles the last, so without an anchor the gallery learns its way up the face and keeps
 * reporting clear. The property under test: nothing the manual reference already scores
 * below the anchor is ever learned, and the near-full level is reported as a flood.
 * A learned reference is the previous reading's frame (proven by this one), so the anchor is
 * checked against that frame's manual score.
 * Known limit, not a margin: the last admission is at ~70 % water, so levels ~82-94 % (which
 * the manual reference already calls submerged) are still reported clear, and the upload at
 * the final ~96 % step scores 0.438 against the 0.45 line. See the anchor spec's "Residual
 * limits" section; do not read this test as proof that a plateaued flood is caught. */
static void test_rising_flood_is_not_learned_away(void)
{
    float xs[512], lo[512], hi[512];
    const int32_t n = sim_strip_outline(xs, lo, hi, 512);
    uint8_t *frame = malloc((size_t)SYNTH_W * SYNTH_H);
    void *g = sim_gallery_new();
    void *manual = sim_gallery_new();     /* the manual reference alone, to read its score */
    void *dev = sim_device_new();
    sim_gallery_add_manual(g, g_dry);
    sim_gallery_add_manual(manual, g_dry);

    sim_result_t r;
    sim_score_t s;
    int admitted = 0, admitted_below_anchor = 0;
    float prev_manual = 1.0f;                            /* manual score of the candidate's frame */
    for (int k = 0; k < 42; ++k) {                       /* level 1/7 .. ~0.96 */
        memcpy(frame, g_dry, (size_t)SYNTH_W * SYNTH_H);
        synth_water(frame, xs, lo, hi, n, LIP + 0.02f * k, k, 8);
        script_t step = { { frame, frame }, { 0, 0 }, { k, k }, 0 };
        sim_device_sample(dev, g, scripted, &step, &r);
        sim_score(manual, frame, NULL, 0, &s);
        if (r.admitted) {
            ++admitted;
            if (prev_manual < ANCHOR) ++admitted_below_anchor;
        }
        prev_manual = s.curb_correlation;
    }
    CHECK(admitted > 0);                   /* the low, lip-only levels are still learnable */
    CHECK(admitted_below_anchor == 0);     /* no learning past the anchor */
    CHECK(s.curb_correlation < 0.45f);     /* the manual reference calls the last level submerged */
    CHECK(r.has_reading && r.outcome == 0);   /* ...and so does the device: upload */

    sim_device_free(dev);
    sim_gallery_free(manual);
    sim_gallery_free(g);
    free(frame);
}

/* Normal drain flow: only the gutter lip holds water, different in every frame. It is reported
 * clear and never learned: each candidate fails its proof. */
static void test_normal_drain_flow_is_not_learned(void)
{
    float xs[512], lo[512], hi[512];
    const int32_t n = sim_strip_outline(xs, lo, hi, 512);
    uint8_t *frame = malloc((size_t)SYNTH_W * SYNTH_H);
    void *g = sim_gallery_new();
    void *dev = sim_device_new();
    sim_gallery_add_manual(g, g_dry);

    sim_result_t r;
    int admitted = 0;
    for (int k = 0; k < 20; ++k) {
        memcpy(frame, g_dry, (size_t)SYNTH_W * SYNTH_H);
        synth_water(frame, xs, lo, hi, n, LIP, 1000 + 37 * k, 256);
        script_t step = { { frame, frame }, { 0, 0 }, { k, k }, 0 };
        sim_device_sample(dev, g, scripted, &step, &r);
        CHECK(r.has_reading && r.outcome == 1);
        admitted += r.admitted;
    }
    CHECK(admitted == 0);

    sim_device_free(dev);
    sim_gallery_free(g);
    free(frame);
}

/* The camera moves with the wind. A learnable frame shifted 8 px down must still be learned:
 * the anchor has to score the manual reference with registration, as detection does. */
static void test_anchor_uses_registration(void)
{
    uint8_t *frame = malloc((size_t)SYNTH_W * SYNTH_H);
    memcpy(frame, g_relit, (size_t)SYNTH_W * SYNTH_H);
    memmove(frame + (size_t)8 * SYNTH_W, frame, (size_t)(SYNTH_H - 8) * SYNTH_W);
    void *g = sim_gallery_new();
    void *dev = sim_device_new();
    sim_gallery_add_manual(g, g_dry);

    sim_result_t r;
    script_t shifted = { { frame, frame }, { 0, 0 }, { 0, 1 }, 0 };
    sim_device_sample(dev, g, scripted, &shifted, &r);
    CHECK(r.candidate_pending);
    sim_device_sample(dev, g, scripted, &shifted, &r);
    CHECK(r.has_reading && r.shift_dy == 8);
    CHECK(r.curb_correlation >= 0.65f && r.curb_correlation < 0.95f);
    CHECK(r.admitted);

    sim_device_free(dev);
    sim_gallery_free(g);
    free(frame);
}

/* Rain-like streaks: the kerb rows change differently every reading. Never proven. */
static void test_transient_noise_not_learned(void)
{
    uint8_t *frame = malloc((size_t)SYNTH_W * SYNTH_H);
    void *g = sim_gallery_new();
    void *dev = sim_device_new();
    sim_gallery_add_manual(g, g_dry);
    sim_result_t r;
    int admitted = 0, pending = 0;
    for (int k = 0; k < 10; ++k) {
        synth_relit(frame, 4, 3, 5000 + 97 * k);
        script_t step = { { frame, frame }, { 0, 0 }, { k, k }, 0 };
        sim_device_sample(dev, g, scripted, &step, &r);
        admitted += r.admitted;
        pending += r.candidate_pending;
    }
    CHECK(pending > 0);                     /* candidates do form: it is the proof that fails */
    CHECK(admitted == 0 && sim_gallery_count(g) == 1);
    sim_device_free(dev);
    sim_gallery_free(g);
    free(frame);
}

/* A car over the curb between candidate and proof must not drop the candidate. */
static void test_candidate_survives_blocked_sample(void)
{
    void *g = sim_gallery_new();
    void *dev = sim_device_new();
    sim_gallery_add_manual(g, g_dry);
    sim_result_t r;
    script_t relit = { { g_relit, g_relit }, { 0, 0 }, { 0, 1 }, 0 };
    script_t blocked = { { g_relit, g_relit }, { 1, 1 }, { 2, 3 }, 0 };
    sim_device_sample(dev, g, scripted, &relit, &r);
    CHECK(r.candidate_pending);
    sim_device_sample(dev, g, scripted, &blocked, &r);
    CHECK(!r.has_reading && r.candidate_pending && !r.admitted);
    sim_device_sample(dev, g, scripted, &relit, &r);
    CHECK(r.admitted && sim_gallery_count(g) == 2);
    sim_device_free(dev);
    sim_gallery_free(g);
}

/* An operator captures this lighting manually while a candidate is pending: the next reading is
 * already covered by the manual slot, so the candidate is not promoted as a duplicate. */
static void test_candidate_not_promoted_when_already_covered(void)
{
    void *g = sim_gallery_new();
    void *dev = sim_device_new();
    sim_gallery_add_manual(g, g_dry);
    sim_result_t r;
    script_t relit = { { g_relit, g_relit }, { 0, 0 }, { 0, 1 }, 0 };
    sim_device_sample(dev, g, scripted, &relit, &r);
    CHECK(r.candidate_pending);
    CHECK(sim_gallery_add_manual(g, g_relit) == 1);
    sim_device_sample(dev, g, scripted, &relit, &r);
    CHECK(!r.admitted && !r.candidate_pending && sim_gallery_count(g) == 2);
    sim_device_free(dev);
    sim_gallery_free(g);
}

/* A proven candidate with nowhere to go (all 8 slots MANUAL) is dropped quietly. */
static void test_full_manual_gallery_drops_proven_candidate(void)
{
    void *g = sim_gallery_new();
    void *dev = sim_device_new();
    for (int i = 0; i < 8; ++i) sim_gallery_add_manual(g, g_dry);
    sim_result_t r;
    script_t relit = { { g_relit, g_relit }, { 0, 0 }, { 0, 1 }, 0 };
    sim_device_sample(dev, g, scripted, &relit, &r);
    CHECK(r.candidate_pending);             /* a candidate forms; it just has nowhere to go */
    sim_device_sample(dev, g, scripted, &relit, &r);
    CHECK(!r.admitted && sim_gallery_count(g) == 8);
    sim_device_free(dev);
    sim_gallery_free(g);
}

/* Deleting references: the source must match, a freed slot is reused, clear_auto keeps every
 * MANUAL slot, and an emptied gallery is not sampled. */
static void test_gallery_delete(void)
{
    void *g = sim_gallery_new();
    void *dev = sim_device_new();
    sim_result_t r;
    int32_t source = -1;
    sim_gallery_add_manual(g, g_dry);                           /* slot 0, MANUAL */
    script_t relit = { { g_relit, g_relit }, { 0, 0 }, { 0, 1 }, 0 };
    sim_device_sample(dev, g, scripted, &relit, &r);
    sim_device_sample(dev, g, scripted, &relit, &r);            /* proven: slot 1, AUTO */
    CHECK(r.admitted && sim_gallery_count(g) == 2);

    CHECK(sim_gallery_delete_slot(g, 1, 0) == 0);               /* AUTO slot, MANUAL request */
    CHECK(sim_gallery_delete_slot(g, 0, 1) == 0);               /* MANUAL slot, AUTO request */
    CHECK(sim_gallery_delete_slot(g, 5, 0) == 0);               /* unused */
    CHECK(sim_gallery_delete_slot(g, -1, 0) == 0 && sim_gallery_delete_slot(g, 8, 0) == 0);
    CHECK(sim_gallery_delete_slot(NULL, 0, 0) == 0 && sim_gallery_clear_auto(NULL) == 0);
    CHECK(sim_gallery_count(g) == 2);

    CHECK(sim_gallery_delete_slot(g, 0, 0) == 1);               /* the manual one */
    CHECK(sim_gallery_count(g) == 1 && sim_gallery_slot(g, 0, NULL, NULL) == 0);
    CHECK(sim_gallery_add_manual(g, g_dry) == 1);               /* reuses the freed slot 0 */
    CHECK(sim_gallery_slot(g, 0, &source, NULL) == 1 && source == 0);

    sim_gallery_add_manual(g, g_flood);                         /* slot 2, MANUAL */
    CHECK(sim_gallery_clear_auto(g) == 1);
    CHECK(sim_gallery_count(g) == 2 && sim_gallery_slot(g, 1, NULL, NULL) == 0);
    CHECK(sim_gallery_clear_auto(g) == 0);

    CHECK(sim_gallery_delete_slot(g, 0, 0) == 1 && sim_gallery_delete_slot(g, 2, 0) == 1);
    CHECK(sim_gallery_count(g) == 0);
    sim_device_sample(dev, g, scripted, &relit, &r);            /* empty: skipped, not a flood */
    CHECK(r.outcome == SIM_OUT_SKIPPED_NO_REF && !r.has_reading);

    sim_device_free(dev);
    sim_gallery_free(g);
}

/* forget_candidate drops a pending candidate: the next reading forms a new one instead of
 * promoting the old. */
static void test_forget_candidate(void)
{
    void *g = sim_gallery_new();
    void *dev = sim_device_new();
    sim_result_t r;
    sim_gallery_add_manual(g, g_dry);
    script_t relit = { { g_relit, g_relit }, { 0, 0 }, { 0, 1 }, 0 };
    sim_device_sample(dev, g, scripted, &relit, &r);
    CHECK(r.candidate_pending);
    sim_device_forget_candidate(dev);
    sim_device_forget_candidate(NULL);
    sim_device_sample(dev, g, scripted, &relit, &r);
    CHECK(!r.admitted && r.candidate_pending && sim_gallery_count(g) == 1);
    sim_device_free(dev);
    sim_gallery_free(g);
}

/* A gallery with MANUAL dry (slot 0), a learned relit (slot 1) and MANUAL flood (slot 2),
 * after two readings (cycle_counter 2). */
static void *gallery_with_learned(void *dev)
{
    void *g = sim_gallery_new();
    sim_result_t r;
    sim_gallery_add_manual(g, g_dry);
    script_t relit = { { g_relit, g_relit }, { 0, 0 }, { 0, 1 }, 0 };
    sim_device_sample(dev, g, scripted, &relit, &r);
    sim_device_sample(dev, g, scripted, &relit, &r);
    sim_gallery_add_manual(g, g_flood);
    return g;
}

static void test_add_manual_returns_slot(void)
{
    void *g = sim_gallery_new();
    CHECK(sim_gallery_add_manual_slot(g, g_dry) == 0);
    CHECK(sim_gallery_add_manual_slot(g, g_dry) == 1);
    for (int i = 2; i < 8; ++i) CHECK(sim_gallery_add_manual_slot(g, g_dry) == i);
    CHECK(sim_gallery_add_manual_slot(g, g_dry) == -1);
    CHECK(sim_gallery_add_manual_slot(NULL, g_dry) == -2);
    CHECK(sim_gallery_add_manual(g, g_dry) == -1);              /* old contract unchanged */
    sim_gallery_free(g);
}

/* After the lines move, every MANUAL slot is rebuilt in place from its frame and samples where
 * the new lines put the strip, exactly as a fresh capture would; learned slots are dropped. */
static void test_rebuild_all_frames(void)
{
    float d[8];
    const float moved[8] = { 560.0f, 100.0f, 1920.0f, 150.0f, 560.0f, 160.0f, 1920.0f, 190.0f };
    sim_get_geometry(d);
    void *dev = sim_device_new();
    void *g = gallery_with_learned(dev);
    void *stale = gallery_with_learned(dev);
    uint32_t last0 = 0;
    sim_gallery_slot(g, 0, NULL, &last0);

    sim_set_geometry(moved);
    void *fresh = sim_gallery_new();
    sim_gallery_add_manual(fresh, g_dry);
    const uint8_t *frames[8] = { g_dry, NULL, g_flood, NULL, NULL, NULL, NULL, NULL };
    const int32_t status[8] = { 0, 1, 0, 1, 1, 1, 1, 1 };
    CHECK(sim_gallery_rebuild(g, frames, status) == 0);
    int32_t source = -1;
    uint32_t last = 0;
    CHECK(sim_gallery_count(g) == 2);
    CHECK(sim_gallery_slot(g, 0, &source, &last) == 1 && source == 0 && last == last0);
    CHECK(sim_gallery_slot(g, 1, NULL, NULL) == 0);
    CHECK(sim_gallery_slot(g, 2, &source, NULL) == 1 && source == 0);
    CHECK(fabsf(sim_correlation_at(g, 0, g_flood, 0, 0) -
                sim_correlation_at(fresh, 0, g_flood, 0, 0)) < 1e-6f);
    CHECK(fabsf(sim_correlation_at(stale, 0, g_flood, 0, 0) -
                sim_correlation_at(fresh, 0, g_flood, 0, 0)) > 1e-3f);

    /* cycle_counter carried over: the next reading stamps cycle 3, not 1. */
    sim_result_t r;
    script_t dry = { { g_dry, g_dry }, { 0, 0 }, { 5, 6 }, 0 };
    sim_device_sample(dev, g, scripted, &dry, &r);
    CHECK(r.gallery_slot == 0 && sim_gallery_slot(g, 0, NULL, &last) == 1 && last == 3);

    sim_set_geometry(d);
    sim_gallery_free(fresh);
    sim_gallery_free(stale);
    sim_gallery_free(g);
    sim_device_free(dev);
}

/* Absent frames drop their slots; no frame at all, or a read error anywhere (even after other
 * slots were rebuilt), leaves the gallery byte-identical; a learned-only gallery is NOTHING. */
static void test_rebuild_absent_none_and_errors(void)
{
    void *dev = sim_device_new();
    void *g = gallery_with_learned(dev);
    const size_t size = (size_t)sim_gallery_size();
    unsigned char *before = malloc(size);
    memcpy(before, g, size);
    const uint8_t *frames[8] = { g_dry, NULL, g_flood, NULL, NULL, NULL, NULL, NULL };

    const int32_t none[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };
    CHECK(sim_gallery_rebuild(g, frames, none) == 1 && memcmp(before, g, size) == 0);
    const int32_t err_first[8] = { 2, 1, 0, 1, 1, 1, 1, 1 };
    CHECK(sim_gallery_rebuild(g, frames, err_first) == 2 && memcmp(before, g, size) == 0);
    const int32_t err_last[8] = { 0, 1, 2, 1, 1, 1, 1, 1 };   /* slot 0 rebuilt, then slot 2 fails */
    CHECK(sim_gallery_rebuild(g, frames, err_last) == 2 && memcmp(before, g, size) == 0);

    const int32_t partial[8] = { 1, 1, 0, 1, 1, 1, 1, 1 };
    CHECK(sim_gallery_rebuild(g, frames, partial) == 0);
    CHECK(sim_gallery_count(g) == 1 && sim_gallery_slot(g, 2, NULL, NULL) == 1);

    void *dev2 = sim_device_new();
    void *learned = gallery_with_learned(dev2);
    sim_gallery_delete_slot(learned, 0, 0);
    sim_gallery_delete_slot(learned, 2, 0);                     /* only the learned slot 1 left */
    const int32_t all[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    CHECK(sim_gallery_rebuild(learned, frames, all) == 1 && sim_gallery_count(learned) == 1);

    free(before);
    sim_gallery_free(learned);
    sim_gallery_free(g);
    sim_device_free(dev2);
    sim_device_free(dev);
}

/* Learning off (a stale gallery): a qualifying frame forms no candidate and is never admitted;
 * switching off drops a pending candidate; on again, it learns. */
static void test_learning_switch(void)
{
    void *g = sim_gallery_new();
    void *dev = sim_device_new();
    sim_result_t r;
    sim_gallery_add_manual(g, g_dry);
    script_t relit = { { g_relit, g_relit }, { 0, 0 }, { 0, 1 }, 0 };
    sim_device_sample(dev, g, scripted, &relit, &r);
    CHECK(r.candidate_pending);
    sim_device_set_learning(dev, 0);
    sim_device_set_learning(NULL, 0);
    for (int i = 0; i < 3; ++i) {
        sim_device_sample(dev, g, scripted, &relit, &r);
        CHECK(!r.admitted && !r.candidate_pending);
    }
    sim_device_set_learning(dev, 1);
    sim_device_sample(dev, g, scripted, &relit, &r);
    sim_device_sample(dev, g, scripted, &relit, &r);
    CHECK(r.admitted && sim_gallery_count(g) == 2);
    sim_device_free(dev);
    sim_gallery_free(g);
}

static void test_band_coverage(void)
{
    const sim_box_t road = { 0.0f, 900.0f, 1919.0f, 1079.0f, 0.9f, 2 };  /* below the lane line */
    CHECK(sim_band_coverage(&k_car) == 1.0f);
    CHECK(sim_band_coverage(&road) == 0.0f);
    CHECK(sim_band_coverage(NULL) == 0.0f);
    CHECK(sim_heartbeat_every() == 60);
}

int main(void)
{
    g_dry = malloc((size_t)SYNTH_W * SYNTH_H);
    g_flood = malloc((size_t)SYNTH_W * SYNTH_H);
    g_relit = malloc((size_t)SYNTH_W * SYNTH_H);
    g_night = malloc((size_t)SYNTH_W * SYNTH_H);
    if (!g_dry || !g_flood || !g_relit || !g_night) return 2;
    synth_dry(g_dry);
    synth_flooded(g_flood);
    synth_relit(g_relit, 4, 3, 7777);   /* ~0.80: learnable */
    synth_relit(g_night, 2, 3, 4242);   /* ~0.55: ambiguous */

    test_abi_and_names();
    test_gray();
    test_geometry();
    test_strip_outline();
    test_reference_and_score();
    test_gallery_full_of_manual();
    test_device();
    test_device_learning();
    test_rising_flood_is_not_learned_away();
    test_normal_drain_flow_is_not_learned();
    test_anchor_uses_registration();
    test_transient_noise_not_learned();
    test_candidate_survives_blocked_sample();
    test_candidate_not_promoted_when_already_covered();
    test_full_manual_gallery_drops_proven_candidate();
    test_gallery_delete();
    test_forget_candidate();
    test_add_manual_returns_slot();
    test_rebuild_all_frames();
    test_rebuild_absent_none_and_errors();
    test_learning_switch();
    test_band_coverage();

    free(g_dry);
    free(g_flood);
    free(g_relit);
    free(g_night);
    if (g_failures) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_shim: all checks passed");
    return 0;
}

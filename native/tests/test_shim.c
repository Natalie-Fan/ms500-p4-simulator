#include "sim_shim.h"

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

int main(void)
{
    test_abi_and_names();
    test_gray();
    test_geometry();
    test_strip_outline();
    if (g_failures) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_shim: all checks passed");
    return 0;
}

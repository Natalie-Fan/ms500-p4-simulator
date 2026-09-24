# Flood Simulator — Part 1: Core (native + headless) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the MS500-P4 firmware's flood-detection C code, unmodified, on image/video files through a C shim, a standalone `flood_sim.exe`, and a headless Python package with `device` and `dense` modes, YOLO box caching, label scoring and a `floodsim` CLI.

**Architecture:** CMake compiles the firmware's `process_flood_detection/*.c` straight from the firmware repo together with `sim_shim.c` into one object library, linked into `flood_core.dll` (loaded by Python via ctypes) and `flood_sim.exe` (standalone C). The shim is the only code touching firmware structs; it exposes flat structs plus the device loop that mirrors `flood_task.c`. Python handles decoding, YOLO, the run loops' timing, reports and the CLI.

**Tech Stack:** C11, MinGW-w64 gcc 16 (UCRT), CMake ≥ 3.20, Ninja; Python 3.11 venv, numpy, opencv-python, ultralytics (optional extra), pytest.

**Spec:** `docs/superpowers/specs/2026-09-24-flood-simulator-design.md`

**Part 2** (`2026-09-24-flood-simulator-part2-gui.md`) builds the PySide6 GUI on the interfaces this plan produces.

**Deviations from the spec (intentional, small):**
- The reference is an opaque C handle (`sim_ref_new/free`) instead of a Python-owned byte buffer. That is safer for `int64_t` alignment and is equally opaque to Python.
- The shim adds `sim_validate_geometry()` (mirrors the firmware's web-save checks so bad geometry can't divide by zero), `sim_correlation_at()` (needed by the reference cross-check test), and `sim_strip_outline()/sim_align_patch()/sim_domain_*()` (needed by Part 2 overlays).
- The `dirty` flag in provenance covers the firmware's `process_flood_detection/` directory only, so unrelated firmware edits (web UI etc.) don't mark results dirty.

## Global Constraints

- Repo root: `D:\git\ms500-p4-simulator`. All commands run in Git Bash from the repo root.
- Firmware repo: `FIRMWARE_DIR` default `D:/git/NETWORK_P4_FLOOD_DETECTION`, env override `FLOOD_FIRMWARE_DIR`. Firmware sources are compiled in place and **never copied or edited**.
- Toolchain: MinGW gcc + CMake + Ninja (all on PATH); C standard C11 with extensions off.
- Python: `py -3.11 -m venv .venv`; run everything with `.venv/Scripts/python`.
- Frame size 1920×1080. Gray = `(r*77 + g*151 + b*28) >> 8` (firmware `flood_frame_tap.c:112`), computed in C.
- Defaults: period 60 s (from the shim), retry gap 1 s, dense every 1, YOLO `yolo26n.pt` conf 0.25 at 640×480.
- `BLOCKED` (dense: `veto_usable == false`) is an abstention, never a negative.
- Output formatting everywhere (CSV, JSON lines, exe): `t_sec` `%.3f`, `curb_correlation`/`veto_coverage` `%.6f`, booleans `true`/`false`, missing label empty.
- `floodsim compare` tolerance 1e-6 on correlation; reference cross-check tolerance 1e-5.
- Every commit message ends with a blank line then `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **Non-ASCII paths on Windows** (e.g. a clip folder named `测试`): `cv2.imread`/`imwrite` fail silently on them. The fix is to decode from bytes, and this is tested in Task 5 (`test_non_ascii_folder`).
2. **A labels CSV saved from Excel** (UTF-8 BOM, CRLF) should parse normally; a user would never suspect the BOM. Tested in Task 8 (`test_labels_with_bom_and_crlf`).
3. **`--out` pointing at an existing, non-empty folder** must be refused, because mixing two runs' `samples.csv`/`uploads` is silent corruption. Tested in Task 8 (`test_prepare_out_dir_refuses_non_empty`).
4. **A box cache cut off mid-write** (process killed while YOLO ran) must lose only the broken line, and that frame is re-detected. Tested in Task 6 (`test_truncated_cache_line_is_skipped`).
5. **Rebuild while another floodsim process has the DLL loaded** (Windows locks it; the link fails with "Permission denied") needs an actionable message. Tested in Task 4 (`test_build_error_message_hints_locked_dll`).

---

## File Structure

```
native/
  CMakeLists.txt          object lib (firmware + shim) → flood_core.dll, flood_sim.exe, tests
  device_constants.h      every value mirrored from firmware device glue, with file:line
  sim_shim.h / sim_shim.c flat C API over the firmware core (only file touching firmware structs)
  sim_io.h / sim_io.c     PGM / frame list / boxes CSV / geometry JSON readers for the exe
  flood_sim_main.c        flood_sim.exe
  tests/synth.h, synth.c  deterministic synthetic dry / flooded frames
  tests/test_shim.c       C unit tests (CTest)
  tests/make_fixtures.c   writes PGM frames + boxes/geometry for golden tests
  tests/run_golden.cmake  runs flood_sim and diffs against a golden file
  tests/golden/device.jsonl, dense.jsonl
floodsim/
  __init__.py  errors.py  results.py  geometry.py  core.py
  source.py  boxes.py  modes.py  labels.py  report.py  cli.py
tools/truth_to_labels.py
tests/  __init__.py conftest.py synth.py test_*.py
pyproject.toml  README.md  .gitignore  .gitattributes
```

---

### Task 1: Native scaffold and shim basics

**Files:**
- Create: `native/CMakeLists.txt`, `native/device_constants.h`, `native/sim_shim.h`, `native/sim_shim.c`, `native/tests/test_shim.c`, `.gitattributes`
- Modify: `.gitignore`

**Interfaces:**
- Produces (C, all `SIM_API`): `sim_abi_version`, `sim_sizeof_box/score/result`, `sim_outcome_name`, `sim_severity_name`, `sim_class_name`, `sim_submerged_below`, `sim_veto_threshold`, `sim_sample_period_s`, `sim_upload_downscale`, `sim_default_geometry`, `sim_get_geometry`, `sim_set_geometry`, `sim_validate_geometry`, `sim_rgb_to_gray`, `sim_domain_x0/x1`, `sim_align_patch`, `sim_strip_outline`. Header also declares Task 2's functions.
- Geometry arrays are `float[8]` in order `curb_x0, curb_y0, curb_x1, curb_y1, lane_x0, lane_y0, lane_x1, lane_y1`.

- [ ] **Step 1: Repo hygiene files**

`.gitignore` (replace contents):
```
.superpowers/
build/
.venv/
out/
__pycache__/
*.egg-info/
.pytest_cache/
```

`.gitattributes`:
```
*.jsonl text eol=lf
```

- [ ] **Step 2: Write `native/device_constants.h`**

```c
/* Values mirrored from firmware device glue that the simulator cannot compile
 * (it needs FreeRTOS, the camera, SPIFFS). Each names the firmware file and
 * line it was copied from, at firmware commit 7c9bed2. When one of those
 * files changes, re-check the matching value here -- nothing else notices.
 *
 * Mirrored on the Python side, listed here so this stays the one index:
 *   floodsim/boxes.py  COCO_TO_FLOOD  <- main/ai_process/flood_task.c:101-112 map_traffic_class
 *   floodsim/modes.py  UploadWriter label "%06X", sequence per upload
 *                                       <- main/ai_process/flood_task.c:221
 *   sim_shim.c sim_device_sample() reference-ready gate
 *                                       <- main/ai_process/flood_task.c:271
 *   sim_shim.c sim_validate_geometry()  <- main/ai_process/flood_geometry_config.c:118-160
 */
#ifndef DEVICE_CONSTANTS_H
#define DEVICE_CONSTANTS_H

/* main/ai_process/flood_task.c:27  FLOOD_SAMPLE_PERIOD_MS 60000 */
#define SIM_SAMPLE_PERIOD_S 60.0f

/* main/bsp_video/flood_frame_tap.c:112  (r * 77 + g * 151 + b * 28) >> 8 */
#define SIM_GRAY_R     77
#define SIM_GRAY_G     151
#define SIM_GRAY_B     28
#define SIM_GRAY_SHIFT 8

/* main/ai_process/flood_geometry_config.c:28-31  g_flood_geometry defaults,
 * in flood_geometry_t field order. */
#define SIM_GEOMETRY_DEFAULT_INIT \
    { 560.0f, 230.0f, 1920.0f, 355.0f, 560.0f, 378.0f, 1920.0f, 446.0f }

/* main/ai_process/flood_geometry_config.c:22  FLOOD_GEOMETRY_MIN_DOMAIN_WIDTH
 * (needs flood_geometry.h included first). */
#define SIM_GEOMETRY_MIN_DOMAIN_WIDTH (FLOOD_STRIP_OFFSET + 40.0f)

/* main/bsp_video/flood_roi.h:38  FLOOD_ROI_DOWNSCALE */
#define SIM_UPLOAD_DOWNSCALE 2

#endif /* DEVICE_CONSTANTS_H */
```

- [ ] **Step 3: Write `native/sim_shim.h`** (complete; Task 2 implements the second half)

```c
/* Flat, stable C API over the firmware flood-detection core. This is the only
 * code that touches firmware structs: if the firmware changes a struct or a
 * signature, the compiler flags it here and Python is unaffected. */
#ifndef SIM_SHIM_H
#define SIM_SHIM_H

#include <stdint.h>

#ifdef _WIN32
#define SIM_API __declspec(dllexport)
#else
#define SIM_API
#endif

#define SIM_ABI_VERSION        1
#define SIM_FRAME_W            1920
#define SIM_FRAME_H            1080
#define SIM_MAX_BOXES          32     /* FLOOD_VETO_MAX_BOXES */
#define SIM_GEOMETRY_LEN       8
#define SIM_OUT_SKIPPED_NO_REF 4      /* after FLOOD_OUT_UPLOAD..HEARTBEAT (0..3) */

typedef struct {
    float   x0, y0, x1, y1;           /* frame coordinates, 1920x1080 */
    float   confidence;
    int32_t cls;                      /* flood_class_t value */
} sim_box_t;

typedef struct {
    int32_t veto_usable;
    float   veto_coverage;
    int32_t veto_worst_class;
    float   curb_correlation;
    int32_t submerged;
    int32_t severity;
    int32_t shift_dx;
    int32_t shift_dy;
} sim_score_t;

typedef struct {
    int32_t outcome;                  /* flood_outcome_t, or SIM_OUT_SKIPPED_NO_REF */
    int32_t frames_taken;
    int32_t has_reading;
    float   curb_correlation;
    int32_t severity;
    int32_t shift_dx;
    int32_t shift_dy;
    int32_t veto_usable;              /* veto of the last attempt that ran detect */
    float   veto_coverage;
    int32_t veto_worst_class;
    int32_t since_image;              /* capture state after this sample */
    int32_t frame_idx;                /* last frame grabbed successfully, -1 if none */
} sim_result_t;

/* Fills the frame for `attempt` (0 = sample time, 1 = retry). gray_out holds
 * SIM_FRAME_W*SIM_FRAME_H bytes, boxes_out SIM_MAX_BOXES entries. Returns 1 if
 * a frame was delivered, 0 if none (the firmware treats that as a failed grab). */
typedef int32_t (*sim_provider_fn)(int32_t attempt, uint8_t *gray_out, sim_box_t *boxes_out,
                                   int32_t *n_boxes, int32_t *frame_idx, void *user);

/* ---- ABI, names, constants ---- */
SIM_API int32_t     sim_abi_version(void);
SIM_API int32_t     sim_sizeof_box(void);
SIM_API int32_t     sim_sizeof_score(void);
SIM_API int32_t     sim_sizeof_result(void);
SIM_API const char *sim_outcome_name(int32_t outcome);
SIM_API const char *sim_severity_name(int32_t severity);
SIM_API const char *sim_class_name(int32_t cls);
SIM_API float       sim_submerged_below(void);
SIM_API float       sim_veto_threshold(void);
SIM_API float       sim_sample_period_s(void);
SIM_API int32_t     sim_upload_downscale(void);

/* ---- geometry ---- */
SIM_API void    sim_default_geometry(float *out8);
SIM_API void    sim_get_geometry(float *out8);
SIM_API void    sim_set_geometry(const float *in8);
SIM_API int32_t sim_validate_geometry(const float *in8, char *err, int32_t err_size);
SIM_API float   sim_domain_x0(void);
SIM_API float   sim_domain_x1(void);
SIM_API void    sim_align_patch(int32_t *out4);
SIM_API int32_t sim_strip_outline(float *xs, float *y_lo, float *y_hi, int32_t max_points);

/* ---- frames ---- */
SIM_API void sim_rgb_to_gray(const uint8_t *rgb, int32_t width, int32_t height, uint8_t *gray_out);

/* ---- reference and scoring (Task 2) ---- */
SIM_API void   *sim_ref_new(void);
SIM_API void    sim_ref_free(void *ref);
SIM_API int32_t sim_ref_build(void *ref, const uint8_t *gray);
SIM_API int32_t sim_ref_ready(const void *ref);
SIM_API float   sim_correlation_at(const void *ref, const uint8_t *gray, int32_t dx, int32_t dy);
SIM_API void    sim_box_from_accelerator(float x0, float y0, float x1, float y1, float confidence,
                                         int32_t cls, int32_t input_w, int32_t input_h,
                                         sim_box_t *out);
SIM_API int32_t sim_score(const void *ref, const uint8_t *gray, const sim_box_t *boxes,
                          int32_t n_boxes, sim_score_t *out);

/* ---- device loop (Task 2) ---- */
SIM_API void *sim_device_new(void);
SIM_API void  sim_device_free(void *dev);
SIM_API void  sim_device_reset(void *dev);
SIM_API void  sim_device_sample(void *dev, const void *ref, sim_provider_fn provider,
                                void *user, sim_result_t *out);

#endif /* SIM_SHIM_H */
```

- [ ] **Step 4: Write the failing C test `native/tests/test_shim.c`**

```c
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
```

- [ ] **Step 5: Write `native/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.20)
project(floodsim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(FIRMWARE_DIR "D:/git/NETWORK_P4_FLOOD_DETECTION" CACHE PATH "Firmware repo root")
set(FLOOD_DIR "${FIRMWARE_DIR}/components/as_code/process_flood_detection")
if(NOT EXISTS "${FLOOD_DIR}/flood_capture.h")
  message(FATAL_ERROR
    "Firmware flood sources not found in ${FLOOD_DIR}. "
    "Set -DFIRMWARE_DIR=... (floodsim passes FLOOD_FIRMWARE_DIR).")
endif()
# Recorded so floodsim can tell when the build points at a different firmware checkout.
file(WRITE "${CMAKE_BINARY_DIR}/firmware_dir.txt" "${FIRMWARE_DIR}")

# Firmware sources are compiled in place, never copied. CONFIGURE_DEPENDS
# picks up added/renamed firmware .c files on the next build.
file(GLOB FLOOD_SOURCES CONFIGURE_DEPENDS "${FLOOD_DIR}/*.c")

add_library(flood_objs OBJECT ${FLOOD_SOURCES} sim_shim.c)
target_include_directories(flood_objs PUBLIC ${FLOOD_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(flood_objs PRIVATE -O2 -Wall)
set_target_properties(flood_objs PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(flood_core SHARED $<TARGET_OBJECTS:flood_objs>)
set_target_properties(flood_core PROPERTIES PREFIX "")
target_link_options(flood_core PRIVATE -static-libgcc)

enable_testing()

add_executable(test_shim tests/test_shim.c)
target_link_libraries(test_shim PRIVATE flood_objs)
add_test(NAME shim COMMAND test_shim)
```

- [ ] **Step 6: Configure and build to verify the test fails to link**

Run: `cmake -S native -B build/native -G Ninja -DCMAKE_BUILD_TYPE=Release -DFIRMWARE_DIR=D:/git/NETWORK_P4_FLOOD_DETECTION && cmake --build build/native`
Expected: FAIL. The compile fails because `native/sim_shim.c` doesn't exist yet ("Cannot find source file: sim_shim.c").

- [ ] **Step 7: Write `native/sim_shim.c` (part A)**

```c
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
```

- [ ] **Step 8: Build and run the tests**

Run: `cmake --build build/native && ctest --test-dir build/native --output-on-failure`
Expected: build succeeds (unused-function warnings about Task 2 declarations are fine: declarations only), `1/1 Test #1: shim ... Passed`.

- [ ] **Step 9: Commit**

```bash
git add .gitignore .gitattributes native
git commit -m "feat(native): firmware object library, DLL and shim basics" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 2: Shim reference, scoring and device loop

**Files:**
- Create: `native/tests/synth.h`, `native/tests/synth.c`
- Modify: `native/sim_shim.c` (append part B), `native/tests/test_shim.c` (add tests), `native/CMakeLists.txt` (test_shim sources)

**Interfaces:**
- Consumes: Task 1 header.
- Produces: `sim_ref_new/free/build/ready`, `sim_correlation_at`, `sim_box_from_accelerator`, `sim_score` (returns 0 and zeroes `out` when the reference is missing or not ready), `sim_device_new/free/reset/sample`. `sim_device_sample` with a NULL or unready reference returns `outcome = SIM_OUT_SKIPPED_NO_REF`, `frame_idx = -1`, and leaves the capture state untouched.
- Produces (tests): `synth_dry(uint8_t*)`, `synth_flooded(uint8_t*)`, `SYNTH_W`, `SYNTH_H`, `SYNTH_FLOOD_Y0 = 195`, `SYNTH_FLOOD_Y1 = 500`.

- [ ] **Step 1: Write `native/tests/synth.h` and `synth.c`**

```c
/* Deterministic synthetic frames. Dry: a hashed texture everywhere, so the
 * kerb strip and the alignment patch both have structure. Flooded: the same
 * frame with rows [SYNTH_FLOOD_Y0, SYNTH_FLOOD_Y1) set to a constant -- the
 * whole kerb strip (y ~200..300 under the default geometry) loses its
 * structure, while the alignment patch (y 0..190) is untouched. */
#ifndef SYNTH_H
#define SYNTH_H

#include <stdint.h>

#define SYNTH_W 1920
#define SYNTH_H 1080
#define SYNTH_FLOOD_Y0 195
#define SYNTH_FLOOD_Y1 500

void synth_dry(uint8_t *gray);
void synth_flooded(uint8_t *gray);

#endif
```

```c
#include "synth.h"

#include <string.h>

static uint8_t texture(int x, int y)
{
    uint32_t h = (uint32_t)x * 73856093u ^ (uint32_t)y * 19349663u;
    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return (uint8_t)(h >> 8);
}

void synth_dry(uint8_t *gray)
{
    for (int y = 0; y < SYNTH_H; ++y)
        for (int x = 0; x < SYNTH_W; ++x)
            gray[(size_t)y * SYNTH_W + x] = texture(x, y);
}

void synth_flooded(uint8_t *gray)
{
    synth_dry(gray);
    for (int y = SYNTH_FLOOD_Y0; y < SYNTH_FLOOD_Y1; ++y)
        memset(gray + (size_t)y * SYNTH_W, 128, SYNTH_W);
}
```

- [ ] **Step 2: Add the failing tests to `native/tests/test_shim.c`**

Add `#include "synth.h"` after `#include "sim_shim.h"`, add these functions above `main`, and call them from `main` after the existing tests (allocate frames first):

```c
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
```

`main` becomes:

```c
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
```

In `native/CMakeLists.txt` change the test target to:

```cmake
add_executable(test_shim tests/test_shim.c tests/synth.c)
target_include_directories(test_shim PRIVATE tests)
target_link_libraries(test_shim PRIVATE flood_objs)
```

- [ ] **Step 3: Build to verify the tests fail**

Run: `cmake --build build/native`
Expected: FAIL at link with `undefined reference to 'sim_ref_new'` (and the other Task 2 functions).

- [ ] **Step 4: Append part B to `native/sim_shim.c`**

```c
/* --------------------------------------------------------------- reference */

static flood_gray_t full_frame(const uint8_t *gray)
{
    flood_gray_t f = { gray, SIM_FRAME_W, SIM_FRAME_H, SIM_FRAME_W };
    return f;
}

void *sim_ref_new(void)      { return calloc(1, sizeof(flood_reference_t)); }
void  sim_ref_free(void *ref) { free(ref); }

int32_t sim_ref_build(void *ref, const uint8_t *gray)
{
    if (!ref || !gray) return 0;
    const flood_gray_t dry = full_frame(gray);
    return flood_reference_init((flood_reference_t *)ref, &dry) ? 1 : 0;
}

int32_t sim_ref_ready(const void *ref)
{
    return (ref && ((const flood_reference_t *)ref)->ready) ? 1 : 0;
}

float sim_correlation_at(const void *ref, const uint8_t *gray, int32_t dx, int32_t dy)
{
    const flood_gray_t f = full_frame(gray);
    return flood_curb_correlation_at((const flood_reference_t *)ref, &f, dx, dy);
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

/* Dense scoring: the two firmware stages on one frame, no capture state. */
int32_t sim_score(const void *ref, const uint8_t *gray, const sim_box_t *boxes,
                  int32_t n_boxes, sim_score_t *out)
{
    if (!out) return 0;
    memset(out, 0, sizeof(*out));
    if (!sim_ref_ready(ref) || !gray) return 0;

    flood_box_t fb[SIM_MAX_BOXES];
    const int32_t n = boxes ? clamp_box_count(n_boxes) : 0;
    for (int32_t i = 0; i < n; ++i) fb[i] = to_flood_box(&boxes[i]);
    const flood_veto_t v = flood_veto_check(fb, n);

    const flood_gray_t f = full_frame(gray);
    const flood_curb_state_t cs = flood_curbcheck((const flood_reference_t *)ref, &f, true);

    out->veto_usable      = v.usable ? 1 : 0;
    out->veto_coverage    = v.coverage;
    out->veto_worst_class = (int32_t)v.worst_class;
    out->curb_correlation = cs.correlation;
    out->submerged        = cs.submerged ? 1 : 0;
    out->severity         = (int32_t)cs.severity;
    out->shift_dx         = cs.shift_dx;
    out->shift_dy         = cs.shift_dy;
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
} sim_device_t;

void *sim_device_new(void)
{
    sim_device_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    d->gray = malloc((size_t)SIM_FRAME_W * SIM_FRAME_H);
    if (!d->gray) { free(d); return NULL; }
    flood_capture_init(&d->capture);
    return d;
}

void sim_device_free(void *dev)
{
    sim_device_t *d = dev;
    if (!d) return;
    free(d->gray);
    free(d);
}

void sim_device_reset(void *dev)
{
    sim_device_t *d = dev;
    if (d) flood_capture_init(&d->capture);
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

void sim_device_sample(void *dev, const void *ref, sim_provider_fn provider, void *user,
                       sim_result_t *out)
{
    sim_device_t *d = dev;
    memset(out, 0, sizeof(*out));
    out->frame_idx = -1;
    if (!d || !provider) {
        out->outcome = SIM_OUT_SKIPPED_NO_REF;
        return;
    }
    /* flood_task.c:271 -- an unready reference is never sampled: the firmware
     * would otherwise read correlation 0 as a flood on every sample. */
    if (!sim_ref_ready(ref)) {
        out->outcome = SIM_OUT_SKIPPED_NO_REF;
        out->since_image = d->capture.since_image;
        return;
    }

    d->attempt = 0;
    d->last_frame_idx = -1;
    d->last_veto.usable = true;
    d->last_veto.coverage = 0.0f;
    d->last_veto.worst_class = FLOOD_CLASS_OTHER;
    d->provider = provider;
    d->user = user;

    const flood_reference_t *r = ref;
    const flood_decision_t dec = flood_capture_sample(&d->capture, r, device_grab, device_detect, d);

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
    if (dec.has_reading) {
        /* A reading always comes from the last grab, still in d->gray. The
         * decision does not carry the registration shift, so recompute it
         * with the same firmware function flood_curbcheck() used. */
        const flood_gray_t f = full_frame(d->gray);
        flood_estimate_shift(r, &f, &out->shift_dx, &out->shift_dy);
    }
}
```

- [ ] **Step 5: Build and run the tests**

Run: `cmake --build build/native && ctest --test-dir build/native --output-on-failure`
Expected: PASS, `test_shim: all checks passed`.

- [ ] **Step 6: Commit**

```bash
git add native
git commit -m "feat(native): reference, dense scoring and device loop in the shim" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: `flood_sim.exe` with golden tests

**Files:**
- Create: `native/sim_io.h`, `native/sim_io.c`, `native/flood_sim_main.c`, `native/tests/make_fixtures.c`, `native/tests/run_golden.cmake`, `native/tests/golden/device.jsonl`, `native/tests/golden/dense.jsonl`
- Modify: `native/CMakeLists.txt`

**Interfaces:**
- Consumes: the shim (Tasks 1–2), `synth_dry/synth_flooded`.
- Produces: `flood_sim.exe --reference R.pgm --frames <dir|list> [--boxes B.csv] [--geometry G.json] [--mode device|dense] [--period 60] [--retry-gap 1] [--every 1] [--fps 1]`. It prints JSON lines with the key orders below; exit code 2 on usage/input errors.
  - device: `mode, sample, t_sec, frame_idx, outcome, frames_taken, has_reading, curb_correlation, severity, shift_dx, shift_dy, veto_usable, veto_coverage, veto_worst_class, since_image`
  - dense: `mode, frame_idx, t_sec, veto_usable, veto_coverage, veto_worst_class, curb_correlation, submerged, severity, shift_dx, shift_dy`
- Frame names: `f<idx>_t<ms>.pgm` (both parts optional; the index defaults to the list position, the time to `position / fps`). Frames are sorted by time, then index. The frame "at t" is the latest frame with `t_sec <= t`.
- Boxes CSV: lines starting with `#` or `frame_idx` are skipped; rows `frame_idx,t_sec,x0,y0,x1,y1,conf,coco_class,flood_class`; malformed rows (including a no-detection marker row with empty fields) are skipped.

- [ ] **Step 1: Write the golden files**

`native/tests/golden/device.jsonl`:
```
{"mode":"device","sample":0,"t_sec":0.000,"frame_idx":0,"outcome":"clear","frames_taken":1,"has_reading":true,"curb_correlation":1.000000,"severity":"none","shift_dx":0,"shift_dy":0,"veto_usable":true,"veto_coverage":0.000000,"veto_worst_class":"other","since_image":1}
{"mode":"device","sample":1,"t_sec":60.000,"frame_idx":2,"outcome":"upload","frames_taken":1,"has_reading":true,"curb_correlation":0.000000,"severity":"moderate","shift_dx":0,"shift_dy":0,"veto_usable":true,"veto_coverage":0.000000,"veto_worst_class":"other","since_image":0}
{"mode":"device","sample":2,"t_sec":120.000,"frame_idx":5,"outcome":"clear","frames_taken":2,"has_reading":true,"curb_correlation":1.000000,"severity":"none","shift_dx":0,"shift_dy":0,"veto_usable":true,"veto_coverage":0.000000,"veto_worst_class":"other","since_image":1}
```

`native/tests/golden/dense.jsonl`:
```
{"mode":"dense","frame_idx":0,"t_sec":0.000,"veto_usable":true,"veto_coverage":0.000000,"veto_worst_class":"other","curb_correlation":1.000000,"submerged":false,"severity":"none","shift_dx":0,"shift_dy":0}
{"mode":"dense","frame_idx":1,"t_sec":1.000,"veto_usable":true,"veto_coverage":0.000000,"veto_worst_class":"other","curb_correlation":1.000000,"submerged":false,"severity":"none","shift_dx":0,"shift_dy":0}
{"mode":"dense","frame_idx":2,"t_sec":60.000,"veto_usable":true,"veto_coverage":0.000000,"veto_worst_class":"other","curb_correlation":0.000000,"submerged":true,"severity":"moderate","shift_dx":0,"shift_dy":0}
{"mode":"dense","frame_idx":3,"t_sec":61.000,"veto_usable":true,"veto_coverage":0.000000,"veto_worst_class":"other","curb_correlation":0.000000,"submerged":true,"severity":"moderate","shift_dx":0,"shift_dy":0}
{"mode":"dense","frame_idx":4,"t_sec":120.000,"veto_usable":false,"veto_coverage":1.000000,"veto_worst_class":"car","curb_correlation":1.000000,"submerged":false,"severity":"none","shift_dx":0,"shift_dy":0}
{"mode":"dense","frame_idx":5,"t_sec":121.000,"veto_usable":true,"veto_coverage":0.000000,"veto_worst_class":"other","curb_correlation":1.000000,"submerged":false,"severity":"none","shift_dx":0,"shift_dy":0}
```

(Scenario: frames 0,1 dry at 0 s/1 s; 2,3 flooded at 60 s/61 s; 4 dry with a car box across the whole frame at 120 s; 5 dry at 121 s. Device samples at 0/60/120 s; at 120 s the car vetoes attempt 1 and the retry at 121 s reads clear.)

- [ ] **Step 2: Write `native/tests/make_fixtures.c`**

```c
/* Writes the golden-test inputs: reference.pgm, frames/f<idx>_t<ms>.pgm,
 * boxes.csv, geometry.json (firmware defaults) and bad_geometry.json. */
#include "synth.h"

#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#include <direct.h>
#define MAKE_DIR(p) _mkdir(p)
#else
#include <sys/stat.h>
#define MAKE_DIR(p) mkdir((p), 0755)
#endif

static int write_pgm(const char *path, const uint8_t *gray)
{
    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    fprintf(f, "P5\n%d %d\n255\n", SYNTH_W, SYNTH_H);
    const size_t n = (size_t)SYNTH_W * SYNTH_H;
    const int ok = fwrite(gray, 1, n, f) == n;
    fclose(f);
    return ok;
}

static int write_text(const char *path, const char *text)
{
    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    const int ok = fputs(text, f) >= 0;
    fclose(f);
    return ok;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fputs("usage: make_fixtures <out_dir>\n", stderr);
        return 2;
    }
    const char *dir = argv[1];
    char path[1024];
    MAKE_DIR(dir);
    snprintf(path, sizeof path, "%s/frames", dir);
    MAKE_DIR(path);

    uint8_t *dry = malloc((size_t)SYNTH_W * SYNTH_H);
    uint8_t *flood = malloc((size_t)SYNTH_W * SYNTH_H);
    if (!dry || !flood) return 2;
    synth_dry(dry);
    synth_flooded(flood);

    const struct { int idx; long long t_ms; const uint8_t *gray; } frames[] = {
        { 0, 0, dry }, { 1, 1000, dry }, { 2, 60000, flood },
        { 3, 61000, flood }, { 4, 120000, dry }, { 5, 121000, dry },
    };

    int ok = 1;
    snprintf(path, sizeof path, "%s/reference.pgm", dir);
    ok &= write_pgm(path, dry);
    for (size_t i = 0; i < sizeof frames / sizeof frames[0]; ++i) {
        snprintf(path, sizeof path, "%s/frames/f%06d_t%010lld.pgm", dir, frames[i].idx, frames[i].t_ms);
        ok &= write_pgm(path, frames[i].gray);
    }
    snprintf(path, sizeof path, "%s/boxes.csv", dir);
    ok &= write_text(path,
        "# fixture: frame 4 has a car across the whole kerb\n"
        "frame_idx,t_sec,x0,y0,x1,y1,conf,coco_class,flood_class\n"
        "4,120.000,0,0,1919,1079,0.9,car,2\n"
        "5,121.000,,,,,,,\n");
    snprintf(path, sizeof path, "%s/geometry.json", dir);
    ok &= write_text(path,
        "{\"curb\": {\"x0\": 560, \"y0\": 230, \"x1\": 1920, \"y1\": 355},\n"
        " \"lane\": {\"x0\": 560, \"y0\": 378, \"x1\": 1920, \"y1\": 446}}\n");
    snprintf(path, sizeof path, "%s/bad_geometry.json", dir);
    ok &= write_text(path,
        "{\"curb\": {\"x0\": 560, \"y0\": 230, \"x1\": 560, \"y1\": 355},\n"
        " \"lane\": {\"x0\": 560, \"y0\": 378, \"x1\": 1920, \"y1\": 446}}\n");

    free(dry);
    free(flood);
    if (!ok) {
        fprintf(stderr, "make_fixtures: failed writing into %s\n", dir);
        return 1;
    }
    return 0;
}
```

- [ ] **Step 3: Write `native/tests/run_golden.cmake`**

```cmake
# Usage: cmake -DEXE=<flood_sim> -DARGS=<a|b|c> -DGOLDEN=<file> -P run_golden.cmake
string(REPLACE "|" ";" args "${ARGS}")
execute_process(COMMAND "${EXE}" ${args}
                OUTPUT_VARIABLE out ERROR_VARIABLE err RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
  message(FATAL_ERROR "flood_sim exited with ${rc}:\n${err}")
endif()
file(READ "${GOLDEN}" expected)
if(NOT out STREQUAL expected)
  message(FATAL_ERROR "output differs from ${GOLDEN}\n--- got ---\n${out}--- expected ---\n${expected}")
endif()
```

- [ ] **Step 4: Add the exe and golden tests to `native/CMakeLists.txt`** (append)

```cmake
add_executable(flood_sim flood_sim_main.c sim_io.c)
target_link_libraries(flood_sim PRIVATE flood_objs)
target_compile_options(flood_sim PRIVATE -O2 -Wall)
target_link_options(flood_sim PRIVATE -static)

add_executable(make_fixtures tests/make_fixtures.c tests/synth.c)
target_include_directories(make_fixtures PRIVATE tests)

set(FIX ${CMAKE_BINARY_DIR}/fixtures)
add_test(NAME fixtures COMMAND make_fixtures ${FIX})
set_tests_properties(fixtures PROPERTIES FIXTURES_SETUP sim_fixtures)

function(golden_test name golden)
  string(REPLACE ";" "|" args "${ARGN}")
  add_test(NAME ${name}
           COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:flood_sim> "-DARGS=${args}"
                   -DGOLDEN=${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/${golden}
                   -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_golden.cmake)
  set_tests_properties(${name} PROPERTIES FIXTURES_REQUIRED sim_fixtures)
endfunction()

golden_test(golden_device device.jsonl
  --reference ${FIX}/reference.pgm --frames ${FIX}/frames --boxes ${FIX}/boxes.csv --mode device)
golden_test(golden_device_geometry device.jsonl
  --reference ${FIX}/reference.pgm --frames ${FIX}/frames --boxes ${FIX}/boxes.csv
  --geometry ${FIX}/geometry.json)
golden_test(golden_dense dense.jsonl
  --reference ${FIX}/reference.pgm --frames ${FIX}/frames --boxes ${FIX}/boxes.csv --mode dense)

add_test(NAME bad_geometry_rejected
         COMMAND flood_sim --reference ${FIX}/reference.pgm --frames ${FIX}/frames
                 --geometry ${FIX}/bad_geometry.json)
set_tests_properties(bad_geometry_rejected PROPERTIES WILL_FAIL TRUE FIXTURES_REQUIRED sim_fixtures)
```

- [ ] **Step 5: Build to verify it fails**

Run: `cmake --build build/native`
Expected: FAIL. CMake reports that `flood_sim_main.c` / `sim_io.c` can't be found.

- [ ] **Step 6: Write `native/sim_io.h`**

```c
/* File readers for flood_sim.exe. Standard C library only. */
#ifndef SIM_IO_H
#define SIM_IO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sim_shim.h"

#define SIM_PATH_MAX 1024

typedef struct {
    char    path[SIM_PATH_MAX];
    int32_t frame_idx;
    double  t_sec;
} sim_frame_t;

typedef struct {
    int32_t   frame_idx;
    sim_box_t box;
} sim_box_rec_t;

/* Reads a 1920x1080 8-bit binary PGM (P5, maxval 255) into gray_out. */
bool sim_read_pgm(const char *path, uint8_t *gray_out, char *err, size_t err_size);

/* `spec` is a directory of *.pgm or a text file with one path per line.
 * Returns the frame count (sorted by t_sec, then frame_idx) or -1; caller
 * frees *out. */
int sim_list_frames(const char *spec, double fps, sim_frame_t **out, char *err, size_t err_size);

/* Latest frame with t_sec <= t, or -1. */
int sim_frame_at(const sim_frame_t *frames, int n, double t);

/* Returns the number of box rows (malformed rows skipped) or -1; caller frees *out. */
int sim_read_boxes(const char *path, sim_box_rec_t **out, char *err, size_t err_size);

/* Copies the boxes of frame_idx into out (at most max_out); returns the count. */
int sim_boxes_for(const sim_box_rec_t *recs, int n_recs, int32_t frame_idx,
                  sim_box_t *out, int max_out);

/* Reads {"curb":{"x0","y0","x1","y1"},"lane":{...}} (the firmware web API shape). */
bool sim_read_geometry_json(const char *path, float out[SIM_GEOMETRY_LEN], char *err, size_t err_size);

#endif /* SIM_IO_H */
```

- [ ] **Step 7: Write `native/sim_io.c`**

```c
#include "sim_io.h"

#include <ctype.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void set_err(char *err, size_t err_size, const char *fmt, const char *arg)
{
    if (err && err_size) snprintf(err, err_size, fmt, arg);
}

static char *copy_string(const char *s)
{
    const size_t n = strlen(s) + 1;
    char *c = malloc(n);
    if (c) memcpy(c, s, n);
    return c;
}

/* ------------------------------------------------------------------- PGM */

/* Next header integer, skipping whitespace and # comments. Consumes the one
 * whitespace byte after the number, as the PGM format requires. */
static int pgm_next_int(FILE *f)
{
    int c = fgetc(f);
    for (;;) {
        while (c != EOF && isspace(c)) c = fgetc(f);
        if (c != '#') break;
        while (c != EOF && c != '\n') c = fgetc(f);
    }
    if (c == EOF || !isdigit(c)) return -1;
    int v = 0;
    while (c != EOF && isdigit(c)) {
        v = v * 10 + (c - '0');
        c = fgetc(f);
    }
    return v;
}

bool sim_read_pgm(const char *path, uint8_t *gray_out, char *err, size_t err_size)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        set_err(err, err_size, "cannot open %s", path);
        return false;
    }
    char magic[2];
    const bool is_p5 = fread(magic, 1, 2, f) == 2 && magic[0] == 'P' && magic[1] == '5';
    const int w = is_p5 ? pgm_next_int(f) : -1;
    const int h = is_p5 ? pgm_next_int(f) : -1;
    const int maxval = is_p5 ? pgm_next_int(f) : -1;
    if (!is_p5 || w != SIM_FRAME_W || h != SIM_FRAME_H || maxval != 255) {
        fclose(f);
        set_err(err, err_size, "%s is not a 1920x1080 8-bit binary PGM", path);
        return false;
    }
    const size_t n = (size_t)SIM_FRAME_W * SIM_FRAME_H;
    const bool ok = fread(gray_out, 1, n, f) == n;
    fclose(f);
    if (!ok) set_err(err, err_size, "%s: truncated pixel data", path);
    return ok;
}

/* ------------------------------------------------------------ frame list */

typedef struct {
    char **items;
    int    n, cap;
} strlist_t;

static bool strlist_push(strlist_t *l, const char *s)
{
    if (l->n == l->cap) {
        const int cap = l->cap ? l->cap * 2 : 64;
        char **grown = realloc(l->items, (size_t)cap * sizeof(char *));
        if (!grown) return false;
        l->items = grown;
        l->cap = cap;
    }
    l->items[l->n] = copy_string(s);
    return l->items[l->n++] != NULL;
}

static void strlist_free(strlist_t *l)
{
    for (int i = 0; i < l->n; ++i) free(l->items[i]);
    free(l->items);
}

static int cmp_strings(const void *a, const void *b)
{
    return strcmp(*(char *const *)a, *(char *const *)b);
}

static bool has_pgm_suffix(const char *name)
{
    const size_t n = strlen(name);
    if (n < 4) return false;
    const char *s = name + n - 4;
    return s[0] == '.' && tolower((unsigned char)s[1]) == 'p' &&
           tolower((unsigned char)s[2]) == 'g' && tolower((unsigned char)s[3]) == 'm';
}

/* f<idx>_t<ms>.pgm as written by `floodsim export`; either part optional. */
static void parse_frame_name(const char *path, int32_t *idx, long long *t_ms, bool *has_t)
{
    const char *base = path;
    for (const char *p = path; *p; ++p)
        if (*p == '/' || *p == '\\') base = p + 1;
    *idx = -1;
    *has_t = false;
    if (base[0] == 'f' && isdigit((unsigned char)base[1])) *idx = (int32_t)strtol(base + 1, NULL, 10);
    const char *t = strstr(base, "_t");
    if (t && isdigit((unsigned char)t[2])) {
        *t_ms = strtoll(t + 2, NULL, 10);
        *has_t = true;
    }
}

static int cmp_frames(const void *a, const void *b)
{
    const sim_frame_t *x = a, *y = b;
    if (x->t_sec != y->t_sec) return x->t_sec < y->t_sec ? -1 : 1;
    return (x->frame_idx > y->frame_idx) - (x->frame_idx < y->frame_idx);
}

int sim_list_frames(const char *spec, double fps, sim_frame_t **out, char *err, size_t err_size)
{
    strlist_t names = { 0 };
    DIR *dir = opendir(spec);
    if (dir) {
        struct dirent *e;
        char full[SIM_PATH_MAX];
        while ((e = readdir(dir)) != NULL) {
            if (!has_pgm_suffix(e->d_name)) continue;
            snprintf(full, sizeof full, "%s/%s", spec, e->d_name);
            if (!strlist_push(&names, full)) break;
        }
        closedir(dir);
        if (names.n > 1) qsort(names.items, (size_t)names.n, sizeof(char *), cmp_strings);
    } else {
        FILE *f = fopen(spec, "r");
        if (!f) {
            set_err(err, err_size, "cannot open frames %s", spec);
            return -1;
        }
        char line[SIM_PATH_MAX];
        while (fgets(line, sizeof line, f)) {
            size_t len = strlen(line);
            while (len && isspace((unsigned char)line[len - 1])) line[--len] = '\0';
            if (len == 0 || line[0] == '#') continue;
            if (!strlist_push(&names, line)) break;
        }
        fclose(f);
    }
    if (names.n == 0) {
        strlist_free(&names);
        set_err(err, err_size, "no .pgm frames in %s", spec);
        return -1;
    }

    sim_frame_t *frames = calloc((size_t)names.n, sizeof(*frames));
    if (!frames) {
        strlist_free(&names);
        set_err(err, err_size, "out of memory listing %s", spec);
        return -1;
    }
    for (int i = 0; i < names.n; ++i) {
        int32_t idx;
        long long t_ms = 0;
        bool has_t;
        snprintf(frames[i].path, sizeof frames[i].path, "%s", names.items[i]);
        parse_frame_name(names.items[i], &idx, &t_ms, &has_t);
        frames[i].frame_idx = idx >= 0 ? idx : i;
        frames[i].t_sec = has_t ? (double)t_ms / 1000.0 : (double)i / fps;
    }
    const int n = names.n;
    strlist_free(&names);
    qsort(frames, (size_t)n, sizeof(*frames), cmp_frames);
    *out = frames;
    return n;
}

int sim_frame_at(const sim_frame_t *frames, int n, double t)
{
    for (int i = n - 1; i >= 0; --i)
        if (frames[i].t_sec <= t + 1e-9) return i;
    return -1;
}

/* ----------------------------------------------------------------- boxes */

int sim_read_boxes(const char *path, sim_box_rec_t **out, char *err, size_t err_size)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        set_err(err, err_size, "cannot open boxes %s", path);
        return -1;
    }
    sim_box_rec_t *recs = NULL;
    int n = 0, cap = 0;
    char line[512];
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#' || strncmp(line, "frame_idx", 9) == 0) continue;
        int idx, cls;
        double t;
        float x0, y0, x1, y1, conf;
        char coco[64];
        /* A no-detection marker row has empty fields and fails here: skipped. */
        if (sscanf(line, "%d,%lf,%f,%f,%f,%f,%f,%63[^,],%d",
                   &idx, &t, &x0, &y0, &x1, &y1, &conf, coco, &cls) != 9) continue;
        if (n == cap) {
            cap = cap ? cap * 2 : 64;
            sim_box_rec_t *grown = realloc(recs, (size_t)cap * sizeof(*recs));
            if (!grown) break;
            recs = grown;
        }
        recs[n].frame_idx = idx;
        recs[n].box.x0 = x0; recs[n].box.y0 = y0;
        recs[n].box.x1 = x1; recs[n].box.y1 = y1;
        recs[n].box.confidence = conf;
        recs[n].box.cls = cls;
        ++n;
    }
    fclose(f);
    *out = recs;
    return n;
}

int sim_boxes_for(const sim_box_rec_t *recs, int n_recs, int32_t frame_idx,
                  sim_box_t *out, int max_out)
{
    int n = 0;
    for (int i = 0; i < n_recs && n < max_out; ++i)
        if (recs[i].frame_idx == frame_idx) out[n++] = recs[i].box;
    return n;
}

/* --------------------------------------------------------------- geometry */

static const char *find_object(const char *json, const char *key)
{
    char pat[32];
    snprintf(pat, sizeof pat, "\"%s\"", key);
    const char *p = strstr(json, pat);
    return p ? strchr(p + strlen(pat), '{') : NULL;
}

static bool read_number(const char *obj, const char *key, float *out)
{
    const char *end = strchr(obj, '}');
    char pat[16];
    snprintf(pat, sizeof pat, "\"%s\"", key);
    const char *p = strstr(obj, pat);
    if (!end || !p || p > end) return false;
    p = strchr(p + strlen(pat), ':');
    if (!p || p > end) return false;
    char *num_end;
    const double v = strtod(p + 1, &num_end);
    if (num_end == p + 1) return false;
    *out = (float)v;
    return true;
}

bool sim_read_geometry_json(const char *path, float out[SIM_GEOMETRY_LEN], char *err, size_t err_size)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        set_err(err, err_size, "cannot open geometry %s", path);
        return false;
    }
    fseek(f, 0, SEEK_END);
    const long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = size > 0 ? malloc((size_t)size + 1) : NULL;
    const bool read_ok = buf && fread(buf, 1, (size_t)size, f) == (size_t)size;
    fclose(f);
    if (!read_ok) {
        free(buf);
        set_err(err, err_size, "cannot read geometry %s", path);
        return false;
    }
    buf[size] = '\0';

    static const char *const lines[2] = { "curb", "lane" };
    static const char *const keys[4] = { "x0", "y0", "x1", "y1" };
    bool ok = true;
    for (int l = 0; l < 2 && ok; ++l) {
        const char *obj = find_object(buf, lines[l]);
        if (!obj) {
            set_err(err, err_size, "geometry: missing \"%s\" object", lines[l]);
            ok = false;
            break;
        }
        for (int k = 0; k < 4 && ok; ++k) {
            if (!read_number(obj, keys[k], &out[l * 4 + k])) {
                set_err(err, err_size, "geometry: %s is missing a number", lines[l]);
                ok = false;
            }
        }
    }
    free(buf);
    return ok;
}
```

- [ ] **Step 8: Write `native/flood_sim_main.c`**

```c
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

typedef struct {
    const char *reference, *frames, *boxes, *geometry, *mode;
    double period, retry_gap, fps;
    int every;
} options_t;

static void usage(void)
{
    fputs("usage: flood_sim --reference dry.pgm --frames <dir|list> [--boxes boxes.csv]\n"
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
        if (!strcmp(a, "--reference")) o->reference = v;
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
    if (!o->reference || !o->frames) { fputs("--reference and --frames are required\n", stderr); return 0; }
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

static void print_device(int sample, double t, const sim_result_t *r)
{
    printf("{\"mode\":\"device\",\"sample\":%d,\"t_sec\":%.3f,\"frame_idx\":%d,\"outcome\":\"%s\","
           "\"frames_taken\":%d,\"has_reading\":%s,\"curb_correlation\":%.6f,\"severity\":\"%s\","
           "\"shift_dx\":%d,\"shift_dy\":%d,\"veto_usable\":%s,\"veto_coverage\":%.6f,"
           "\"veto_worst_class\":\"%s\",\"since_image\":%d}\n",
           sample, t, r->frame_idx, sim_outcome_name(r->outcome), r->frames_taken,
           json_bool(r->has_reading), r->curb_correlation, sim_severity_name(r->severity),
           r->shift_dx, r->shift_dy, json_bool(r->veto_usable), r->veto_coverage,
           sim_class_name(r->veto_worst_class), r->since_image);
}

static void print_dense(int32_t frame_idx, double t, const sim_score_t *s)
{
    printf("{\"mode\":\"dense\",\"frame_idx\":%d,\"t_sec\":%.3f,\"veto_usable\":%s,"
           "\"veto_coverage\":%.6f,\"veto_worst_class\":\"%s\",\"curb_correlation\":%.6f,"
           "\"submerged\":%s,\"severity\":\"%s\",\"shift_dx\":%d,\"shift_dy\":%d}\n",
           frame_idx, t, json_bool(s->veto_usable), s->veto_coverage,
           sim_class_name(s->veto_worst_class), s->curb_correlation, json_bool(s->submerged),
           sim_severity_name(s->severity), s->shift_dx, s->shift_dy);
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
    void *ref = sim_ref_new();
    if (!gray || !ref) return fail("out of memory");
    if (!sim_read_pgm(o.reference, gray, err, sizeof err)) return fail(err);
    if (!sim_ref_build(ref, gray)) return fail("geometry leaves no usable sampling window");

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
            sim_device_sample(dev, ref, provide_frame, &ctx, &r);
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
            sim_score(ref, gray, fb, nfb, &s);
            print_dense(frames[i].frame_idx, frames[i].t_sec, &s);
        }
    }

    free(boxes);
    free(frames);
    sim_ref_free(ref);
    free(gray);
    return 0;
}
```

- [ ] **Step 9: Build and run all native tests**

Run: `cmake --build build/native && ctest --test-dir build/native --output-on-failure`
Expected: 6/6 pass (`shim`, `fixtures`, `golden_device`, `golden_device_geometry`, `golden_dense`, `bad_geometry_rejected`).

- [ ] **Step 10: Commit**

```bash
git add native
git commit -m "feat(native): flood_sim.exe with golden device/dense tests" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: Python package, geometry and the ctypes core

**Files:**
- Create: `pyproject.toml`, `floodsim/__init__.py`, `floodsim/errors.py`, `floodsim/results.py`, `floodsim/geometry.py`, `floodsim/core.py`, `tests/__init__.py`, `tests/conftest.py`, `tests/synth.py`, `tests/test_geometry.py`, `tests/test_core.py`

**Interfaces:**
- Consumes: `build/native/flood_core.dll` and the shim C API (Tasks 1–3).
- Produces:
  - `floodsim.errors`: `FloodSimError(Exception)`, `BuildError(FloodSimError)`.
  - `floodsim.results`: `Box(x0, y0, x1, y1, confidence, cls:int)`, `ScoreResult(veto_usable, veto_coverage, veto_worst_class:str, curb_correlation, submerged, severity:str, shift_dx, shift_dy)`, `DeviceResult(outcome:str, frames_taken, has_reading, curb_correlation, severity:str, shift_dx, shift_dy, veto_usable, veto_coverage, veto_worst_class:str, since_image, frame_idx)`, `DEVICE_FIELDS`, `DENSE_FIELDS` (lists of column names in the exe's key order, minus `mode`).
  - `floodsim.geometry`: `Geometry` (frozen dataclass, 8 floats; `to_list()`, `from_list(v)`, `to_json_dict()`), `load_geometry(path) -> Geometry`, `save_geometry(g, path)`.
  - `floodsim.core`: constants `REPO_ROOT, NATIVE_DIR, BUILD_DIR, DLL_NAME="flood_core.dll", EXE_NAME="flood_sim.exe", ABI_VERSION=1, FRAME_W=1920, FRAME_H=1080, MAX_BOXES=32`; `firmware_dir() -> Path`; `source_files(fw) -> list[Path]`; `needs_rebuild(dll, sources) -> bool`; `configured_firmware(build_dir) -> Path | None`; `build(fw)`; `STAMP_NAME = "floodsim.stamp"` (touched after each successful build; `needs_rebuild(stamp, sources)` compares against it); `build_error_message(cmd, stdout, stderr) -> str`; `firmware_provenance(fw) -> dict(commit, dirty)`; `load_core(*, rebuild=True) -> tuple[Core, dict]` (cached per process); `Core` with `default_geometry() -> list[float]`, `get_geometry() -> list[float]`, `set_geometry(g: Geometry)`, `validate_geometry(g) -> str | None`, `rgb_to_gray(rgb) -> np.ndarray`, `new_reference() -> Reference`, `score(ref, gray, boxes) -> ScoreResult`, `correlation_at(ref, gray, dx, dy) -> float`, `new_device() -> Device`, `box_from_accelerator(x0, y0, x1, y1, conf, cls, in_w, in_h) -> Box`, `submerged_below()`, `veto_threshold()`, `sample_period_s()`, `upload_downscale()`, `strip_outline() -> (xs, y_lo, y_hi)` numpy float32 arrays, `align_patch() -> (x0, y0, x1, y1)`, `domain() -> (x0, x1)`, `outcome_name(i)`, `severity_name(i)`, `class_name(i)`; `Reference` with `build(gray) -> bool`, `ready`, `geometry` (tuple or None), `is_stale() -> bool`, `close()`; `Device` with `sample(ref, provider) -> DeviceResult`, `reset()`, `close()`. The provider is `Callable[[int], tuple[np.ndarray, list[Box], int] | None]` (gray 1080×1920 uint8, boxes, frame_idx).
  - `tests.synth`: `dry_gray()`, `flooded_gray()`, `to_rgb(gray)`, `CAR` (a `Box` covering the frame, class car), `MemorySource` (Task 5 adds it).
  - `tests.conftest`: fixture `core` (the process-wide `Core`, geometry reset to defaults per test), fixture `build_info`.

- [ ] **Step 1: Create the venv and package skeleton**

`pyproject.toml`:
```toml
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "floodsim"
version = "0.1.0"
description = "Run the MS500-P4 flood-detection firmware code on image and video files"
requires-python = ">=3.11"
dependencies = ["numpy>=1.26", "opencv-python>=4.9"]

[project.optional-dependencies]
yolo = ["ultralytics>=8.3"]
dev = ["pytest>=8"]

[project.scripts]
floodsim = "floodsim.cli:main"

[tool.setuptools.packages.find]
include = ["floodsim*"]

[tool.pytest.ini_options]
testpaths = ["tests"]
```

`floodsim/__init__.py`:
```python
"""Flood-detection simulator for the MS500-P4 firmware."""

__version__ = "0.1.0"
```

`floodsim/errors.py`:
```python
class FloodSimError(Exception):
    """A user-facing error: printed as `error: <message>` by the CLI."""


class BuildError(FloodSimError):
    """The native build (cmake configure or build) failed."""
```

`tests/__init__.py`: empty file.

Run:
```bash
py -3.11 -m venv .venv
.venv/Scripts/python -m pip install --upgrade pip
.venv/Scripts/python -m pip install -e ".[dev]"
```
Expected: installs numpy, opencv-python, pytest; `Successfully installed ... floodsim-0.1.0`.

- [ ] **Step 2: Write `floodsim/results.py`**

```python
"""Plain result types shared by the core, the runners and the reports."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Box:
    """A detection in 1920x1080 frame coordinates; cls is a flood_class_t value."""
    x0: float
    y0: float
    x1: float
    y1: float
    confidence: float
    cls: int


@dataclass(frozen=True)
class ScoreResult:
    veto_usable: bool
    veto_coverage: float
    veto_worst_class: str
    curb_correlation: float
    submerged: bool
    severity: str
    shift_dx: int
    shift_dy: int


@dataclass(frozen=True)
class DeviceResult:
    outcome: str
    frames_taken: int
    has_reading: bool
    curb_correlation: float
    severity: str
    shift_dx: int
    shift_dy: int
    veto_usable: bool
    veto_coverage: float
    veto_worst_class: str
    since_image: int
    frame_idx: int


# Column order of samples.csv; identical to flood_sim.exe's JSON keys (minus "mode").
DEVICE_FIELDS = [
    "sample", "t_sec", "frame_idx", "outcome", "frames_taken", "has_reading",
    "curb_correlation", "severity", "shift_dx", "shift_dy", "veto_usable",
    "veto_coverage", "veto_worst_class", "since_image",
]
DENSE_FIELDS = [
    "frame_idx", "t_sec", "veto_usable", "veto_coverage", "veto_worst_class",
    "curb_correlation", "submerged", "severity", "shift_dx", "shift_dy",
]
```

- [ ] **Step 3: Write the failing geometry tests `tests/test_geometry.py`**

```python
import json

import pytest

from floodsim.errors import FloodSimError
from floodsim.geometry import Geometry, load_geometry, save_geometry

DEFAULT = [560.0, 230.0, 1920.0, 355.0, 560.0, 378.0, 1920.0, 446.0]


def test_round_trip(tmp_path):
    g = Geometry.from_list(DEFAULT)
    path = tmp_path / "geo.json"
    save_geometry(g, path)
    assert load_geometry(path) == g
    assert json.loads(path.read_text())["curb"] == {"x0": 560.0, "y0": 230.0, "x1": 1920.0, "y1": 355.0}


def test_accepts_firmware_web_response(tmp_path):
    # Shape returned by the firmware's GET handler, extra keys included.
    path = tmp_path / "web.json"
    path.write_text(json.dumps({
        "code": 0,
        "curb": {"x0": 560, "y0": 230, "x1": 1920, "y1": 355},
        "lane": {"x0": 560, "y0": 378, "x1": 1920, "y1": 446},
        "geometry_version": 3, "calibrated": True, "stale": False,
    }))
    assert load_geometry(path).to_list() == DEFAULT


def test_missing_number_is_an_error(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text('{"curb": {"x0": 1, "y0": 2, "x1": 3}, "lane": {"x0": 1, "y0": 2, "x1": 3, "y1": 4}}')
    with pytest.raises(FloodSimError, match="curb.y1"):
        load_geometry(path)


def test_not_json_is_an_error(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text("not json")
    with pytest.raises(FloodSimError, match="cannot read geometry"):
        load_geometry(path)
```

Run: `.venv/Scripts/python -m pytest tests/test_geometry.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'floodsim.geometry'`.

- [ ] **Step 4: Write `floodsim/geometry.py`**

```python
"""Curb/lane geometry in the firmware web API's JSON shape."""
from __future__ import annotations

import json
from dataclasses import astuple, dataclass
from pathlib import Path

from .errors import FloodSimError

_LINES = ("curb", "lane")
_KEYS = ("x0", "y0", "x1", "y1")


@dataclass(frozen=True)
class Geometry:
    curb_x0: float
    curb_y0: float
    curb_x1: float
    curb_y1: float
    lane_x0: float
    lane_y0: float
    lane_x1: float
    lane_y1: float

    def to_list(self) -> list[float]:
        return list(astuple(self))

    @classmethod
    def from_list(cls, values) -> Geometry:
        values = [float(v) for v in values]
        if len(values) != 8:
            raise FloodSimError(f"geometry needs 8 numbers, got {len(values)}")
        return cls(*values)

    def to_json_dict(self) -> dict:
        v = self.to_list()
        return {line: dict(zip(_KEYS, v[i * 4:i * 4 + 4])) for i, line in enumerate(_LINES)}


def load_geometry(path: Path) -> Geometry:
    """Reads {"curb": {x0,y0,x1,y1}, "lane": {...}}; other keys are ignored."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as e:
        raise FloodSimError(f"cannot read geometry {path}: {e}") from e
    values = []
    for line in _LINES:
        obj = data.get(line) if isinstance(data, dict) else None
        if not isinstance(obj, dict):
            raise FloodSimError(f"{path}: missing \"{line}\" object")
        for key in _KEYS:
            v = obj.get(key)
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                raise FloodSimError(f"{path}: {line}.{key} must be a number")
            values.append(float(v))
    return Geometry.from_list(values)


def save_geometry(g: Geometry, path: Path) -> None:
    Path(path).write_text(json.dumps(g.to_json_dict(), indent=2) + "\n", encoding="utf-8")
```

Run: `.venv/Scripts/python -m pytest tests/test_geometry.py -v`
Expected: 4 passed.

- [ ] **Step 5: Write `tests/synth.py` and `tests/conftest.py`**

`tests/synth.py`:
```python
"""Synthetic frames, identical to native/tests/synth.c."""
from __future__ import annotations

from functools import lru_cache

import numpy as np

from floodsim.results import Box

FRAME_H, FRAME_W = 1080, 1920
FLOOD_Y0, FLOOD_Y1 = 195, 500
CAR = Box(0.0, 0.0, 1919.0, 1079.0, 0.9, 2)


@lru_cache(maxsize=1)
def _dry() -> np.ndarray:
    y, x = np.mgrid[0:FRAME_H, 0:FRAME_W].astype(np.uint32)
    h = (x * np.uint32(73856093)) ^ (y * np.uint32(19349663))
    h ^= h >> np.uint32(13)
    h *= np.uint32(0x5BD1E995)
    h ^= h >> np.uint32(15)
    g = ((h >> np.uint32(8)) & np.uint32(0xFF)).astype(np.uint8)
    g.flags.writeable = False
    return g


def dry_gray() -> np.ndarray:
    return _dry()


def flooded_gray() -> np.ndarray:
    g = _dry().copy()
    g[FLOOD_Y0:FLOOD_Y1, :] = 128
    return g


def to_rgb(gray: np.ndarray) -> np.ndarray:
    """Equal channels, so the firmware gray formula returns `gray` exactly."""
    return np.repeat(gray[:, :, None], 3, axis=2)
```

`tests/conftest.py`:
```python
import pytest

from floodsim.core import load_core
from floodsim.geometry import Geometry


@pytest.fixture(scope="session")
def _core_session():
    return load_core()


@pytest.fixture
def core(_core_session):
    core, _ = _core_session
    core.set_geometry(Geometry.from_list(core.default_geometry()))
    return core


@pytest.fixture
def build_info(_core_session):
    return _core_session[1]
```

- [ ] **Step 6: Write the failing core tests `tests/test_core.py`**

```python
import os
import time
from pathlib import Path

import numpy as np
import pytest

from floodsim import core as core_mod
from floodsim.core import build_error_message, firmware_dir, needs_rebuild
from floodsim.errors import FloodSimError
from floodsim.geometry import Geometry
from floodsim.results import Box
from tests.synth import CAR, dry_gray, flooded_gray


def test_build_info_records_dll_and_firmware(build_info):
    assert build_info["dll"]["abi"] == 1
    assert build_info["dll"]["stale"] is False
    assert Path(build_info["firmware"]["dir"]).name == "NETWORK_P4_FLOOD_DETECTION"
    assert "commit" in build_info["firmware"] and "dirty" in build_info["firmware"]


def test_rgb_to_gray_matches_firmware_formula(core):
    rng = np.random.default_rng(1)
    rgb = rng.integers(0, 256, size=(5, 7, 3), dtype=np.uint8)
    r, g, b = (rgb[..., i].astype(np.uint32) for i in range(3))
    expected = ((r * 77 + g * 151 + b * 28) >> 8).astype(np.uint8)
    assert np.array_equal(core.rgb_to_gray(rgb), expected)


def test_default_geometry_matches_firmware(core):
    assert core.default_geometry() == [560.0, 230.0, 1920.0, 355.0, 560.0, 378.0, 1920.0, 446.0]
    assert core.get_geometry() == core.default_geometry()


def test_validate_geometry_uses_firmware_messages(core):
    good = Geometry.from_list(core.default_geometry())
    assert core.validate_geometry(good) is None
    bad = Geometry.from_list([560, 230, 560, 355, 560, 378, 1920, 446])
    assert "curb line endpoints" in core.validate_geometry(bad)


def test_identical_frame_scores_one(core):
    ref = core.new_reference()
    assert ref.build(dry_gray())
    s = core.score(ref, dry_gray(), [])
    assert s.curb_correlation == pytest.approx(1.0, abs=1e-6)
    assert s.submerged is False and s.severity == "none"
    assert (s.shift_dx, s.shift_dy) == (0, 0)
    assert s.veto_usable is True and s.veto_worst_class == "other"


def test_flooded_frame_is_submerged(core):
    ref = core.new_reference()
    ref.build(dry_gray())
    s = core.score(ref, flooded_gray(), [])
    assert s.curb_correlation == 0.0
    assert s.submerged is True and s.severity == "moderate"


def test_box_across_kerb_vetoes(core):
    ref = core.new_reference()
    ref.build(dry_gray())
    s = core.score(ref, dry_gray(), [CAR])
    assert s.veto_usable is False
    assert s.veto_coverage == 1.0
    assert s.veto_worst_class == "car"


def test_score_without_reference_raises(core):
    with pytest.raises(FloodSimError, match="reference"):
        core.score(core.new_reference(), dry_gray(), [])


def test_reference_goes_stale_when_geometry_changes(core):
    ref = core.new_reference()
    ref.build(dry_gray())
    assert ref.is_stale() is False
    g = core.default_geometry()
    g[1] = 240.0
    core.set_geometry(Geometry.from_list(g))
    assert ref.is_stale() is True


def test_box_from_accelerator_scales_to_frame(core):
    b = core.box_from_accelerator(10, 20, 30, 40, 0.5, 1, 640, 480)
    assert b == Box(30.0, 45.0, 90.0, 90.0, 0.5, 1)


def test_strip_outline_and_patch(core):
    xs, lo, hi = core.strip_outline()
    assert len(xs) == 275 and xs[0] == 700.0
    assert np.all(lo < hi)
    assert core.align_patch() == (300, 0, 1700, 200)
    assert core.domain() == (560.0, 1920.0)


def _provider(frames):
    """frames: list per attempt of (gray | None, boxes, idx)."""
    def provide(attempt):
        gray, boxes, idx = frames[attempt]
        return None if gray is None else (gray, boxes, idx)
    return provide


def test_device_clear_upload_retry(core):
    ref = core.new_reference()
    ref.build(dry_gray())
    dev = core.new_device()
    r = dev.sample(ref, _provider([(dry_gray(), [], 0), (dry_gray(), [], 1)]))
    assert (r.outcome, r.frames_taken, r.since_image, r.frame_idx) == ("clear", 1, 1, 0)
    r = dev.sample(ref, _provider([(flooded_gray(), [], 2), (flooded_gray(), [], 3)]))
    assert (r.outcome, r.severity, r.since_image) == ("upload", "moderate", 0)
    r = dev.sample(ref, _provider([(dry_gray(), [CAR], 4), (dry_gray(), [], 5)]))
    assert (r.outcome, r.frames_taken, r.frame_idx, r.veto_usable) == ("clear", 2, 5, True)


def test_device_heartbeat_on_60th_reading(core):
    ref = core.new_reference()
    ref.build(dry_gray())
    dev = core.new_device()
    outcomes = [dev.sample(ref, _provider([(dry_gray(), [], 0)] * 2)).outcome for _ in range(60)]
    assert outcomes[:59] == ["clear"] * 59
    assert outcomes[59] == "heartbeat"


def test_device_skips_without_reference(core):
    dev = core.new_device()
    r = dev.sample(None, _provider([(dry_gray(), [], 0)] * 2))
    assert r.outcome == "skipped_no_ref" and r.frame_idx == -1
    r = dev.sample(core.new_reference(), _provider([(dry_gray(), [], 0)] * 2))
    assert r.outcome == "skipped_no_ref"


def test_provider_exception_propagates(core):
    ref = core.new_reference()
    ref.build(dry_gray())

    def broken(attempt):
        raise ValueError("boom")

    with pytest.raises(ValueError, match="boom"):
        core.new_device().sample(ref, broken)


def test_needs_rebuild(tmp_path):
    stamp = tmp_path / "floodsim.stamp"
    src = tmp_path / "a.c"
    src.write_text("int a;")
    assert needs_rebuild(stamp, [src]) is True
    stamp.write_bytes(b"")
    old = time.time() - 100
    os.utime(src, (old, old))
    assert needs_rebuild(stamp, [src]) is False
    src.write_text("int b;")
    os.utime(src, (time.time() + 5, time.time() + 5))
    assert needs_rebuild(stamp, [src]) is True


def test_firmware_dir_override_missing(monkeypatch, tmp_path):
    monkeypatch.setenv("FLOOD_FIRMWARE_DIR", str(tmp_path / "nope"))
    with pytest.raises(FloodSimError, match="FLOOD_FIRMWARE_DIR"):
        firmware_dir()


def test_build_error_message_hints_locked_dll():
    msg = build_error_message(["cmake", "--build", "x"], "", "ld: cannot open output file flood_core.dll: Permission denied")
    assert "Permission denied" in msg
    assert "close other floodsim" in msg
```

Run: `.venv/Scripts/python -m pytest tests/test_core.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'floodsim.core'`.

- [ ] **Step 7: Write `floodsim/core.py`**

```python
"""Builds and loads flood_core.dll (firmware C + shim) and wraps it with ctypes."""
from __future__ import annotations

import ctypes
import os
import subprocess
from ctypes import POINTER, byref, c_char_p, c_float, c_int32, c_uint8, c_void_p
from pathlib import Path
from typing import Callable

import numpy as np

from .errors import BuildError, FloodSimError
from .geometry import Geometry
from .results import Box, DeviceResult, ScoreResult

REPO_ROOT = Path(__file__).resolve().parent.parent
NATIVE_DIR = REPO_ROOT / "native"
BUILD_DIR = REPO_ROOT / "build" / "native"
DEFAULT_FIRMWARE_DIR = Path("D:/git/NETWORK_P4_FLOOD_DETECTION")
FLOOD_SUBDIR = "components/as_code/process_flood_detection"
DLL_NAME = "flood_core.dll"
EXE_NAME = "flood_sim.exe"
STAMP_NAME = "floodsim.stamp"
ABI_VERSION = 1
FRAME_W, FRAME_H = 1920, 1080
MAX_BOXES = 32

Provider = Callable[[int], "tuple[np.ndarray, list[Box], int] | None"]


# ------------------------------------------------------------------ building

def firmware_dir() -> Path:
    env = os.environ.get("FLOOD_FIRMWARE_DIR")
    fw = Path(env) if env else DEFAULT_FIRMWARE_DIR
    if not (fw / FLOOD_SUBDIR / "flood_capture.h").exists():
        raise FloodSimError(
            f"firmware flood sources not found under {fw} "
            f"(expected {FLOOD_SUBDIR}); set FLOOD_FIRMWARE_DIR to the firmware repo")
    return fw


def source_files(fw: Path) -> list[Path]:
    flood = fw / FLOOD_SUBDIR
    files = [*flood.glob("*.c"), *flood.glob("*.h")]
    files += [p for p in NATIVE_DIR.rglob("*") if p.suffix in {".c", ".h", ".txt", ".cmake"}]
    return files


def needs_rebuild(stamp: Path, sources: list[Path]) -> bool:
    """True if any source is newer than the last successful build (`stamp`)."""
    if not stamp.exists():
        return True
    built = stamp.stat().st_mtime
    return any(p.stat().st_mtime > built for p in sources)


def configured_firmware(build_dir: Path) -> Path | None:
    marker = build_dir / "firmware_dir.txt"
    return Path(marker.read_text().strip()) if marker.exists() else None


def build_error_message(cmd: list[str], stdout: str, stderr: str) -> str:
    msg = f"{' '.join(cmd)} failed:\n{stdout}\n{stderr}".rstrip()
    if "Permission denied" in stdout + stderr:
        msg += ("\n\nThe DLL is probably loaded by another process; "
                "close other floodsim windows/runs and try again.")
    return msg


def build(fw: Path, build_dir: Path = BUILD_DIR) -> None:
    commands = [
        ["cmake", "-S", str(NATIVE_DIR), "-B", str(build_dir), "-G", "Ninja",
         "-DCMAKE_BUILD_TYPE=Release", f"-DFIRMWARE_DIR={fw.as_posix()}"],
        ["cmake", "--build", str(build_dir)],
    ]
    for cmd in commands:
        p = subprocess.run(cmd, capture_output=True, text=True)
        if p.returncode != 0:
            raise BuildError(build_error_message(cmd, p.stdout, p.stderr))


def firmware_provenance(fw: Path) -> dict:
    def git(*args: str) -> str | None:
        try:
            p = subprocess.run(["git", "-C", str(fw), *args], capture_output=True, text=True)
        except OSError:
            return None
        return p.stdout.strip() if p.returncode == 0 else None

    commit = git("rev-parse", "HEAD")
    status = git("status", "--porcelain", "--", FLOOD_SUBDIR)
    return {"commit": commit, "dirty": None if status is None else bool(status)}


_LOADED: tuple["Core", dict] | None = None


def load_core(*, rebuild: bool = True) -> tuple["Core", dict]:
    """Rebuild if firmware or shim sources are newer than the DLL, then load it once per process."""
    global _LOADED
    if _LOADED is not None:
        return _LOADED
    fw = firmware_dir()
    dll = BUILD_DIR / DLL_NAME
    # A stamp, not the DLL's mtime: ninja only relinks the DLL when its own
    # inputs change, so an edited test file would otherwise look stale forever.
    stamp = BUILD_DIR / STAMP_NAME
    configured = configured_firmware(BUILD_DIR)
    stale = (not dll.exists() or needs_rebuild(stamp, source_files(fw))
             or configured is None or configured.resolve() != fw.resolve())
    if stale and rebuild:
        build(fw)
        stamp.touch()
        stale = False
    if not dll.exists():
        raise FloodSimError(f"{dll} does not exist; run without --no-rebuild to build it")
    info = {
        "dll": {"path": str(dll), "abi": ABI_VERSION, "stale": stale},
        "firmware": {"dir": str(fw), **firmware_provenance(fw)},
    }
    _LOADED = (Core(dll), info)
    return _LOADED


# ------------------------------------------------------------------ ctypes

class _SimBox(ctypes.Structure):
    _fields_ = [("x0", c_float), ("y0", c_float), ("x1", c_float), ("y1", c_float),
                ("confidence", c_float), ("cls", c_int32)]


class _SimScore(ctypes.Structure):
    _fields_ = [("veto_usable", c_int32), ("veto_coverage", c_float), ("veto_worst_class", c_int32),
                ("curb_correlation", c_float), ("submerged", c_int32), ("severity", c_int32),
                ("shift_dx", c_int32), ("shift_dy", c_int32)]


class _SimResult(ctypes.Structure):
    _fields_ = [("outcome", c_int32), ("frames_taken", c_int32), ("has_reading", c_int32),
                ("curb_correlation", c_float), ("severity", c_int32), ("shift_dx", c_int32),
                ("shift_dy", c_int32), ("veto_usable", c_int32), ("veto_coverage", c_float),
                ("veto_worst_class", c_int32), ("since_image", c_int32), ("frame_idx", c_int32)]


_PROVIDER_FN = ctypes.CFUNCTYPE(c_int32, c_int32, POINTER(c_uint8), POINTER(_SimBox),
                                POINTER(c_int32), POINTER(c_int32), c_void_p)
_F8 = c_float * 8
_U8P = POINTER(c_uint8)
_FP = POINTER(c_float)


def _u8(a: np.ndarray):
    return a.ctypes.data_as(_U8P)


def _check_gray(gray: np.ndarray) -> np.ndarray:
    gray = np.ascontiguousarray(gray)
    if gray.shape != (FRAME_H, FRAME_W) or gray.dtype != np.uint8:
        raise FloodSimError(f"expected a {FRAME_H}x{FRAME_W} uint8 gray frame, got {gray.shape} {gray.dtype}")
    return gray


def _sim_boxes(boxes: list[Box]):
    n = min(len(boxes), MAX_BOXES)
    arr = (_SimBox * max(n, 1))()
    for i, b in enumerate(boxes[:n]):
        arr[i] = _SimBox(b.x0, b.y0, b.x1, b.y1, b.confidence, b.cls)
    return arr, n


class Core:
    def __init__(self, dll_path: Path):
        self._lib = ctypes.CDLL(str(dll_path))
        self._declare()
        abi = self._lib.sim_abi_version()
        if abi != ABI_VERSION:
            raise FloodSimError(f"DLL ABI v{abi}, wrapper expects v{ABI_VERSION} -- rebuild")
        for fn, struct in (("sim_sizeof_box", _SimBox), ("sim_sizeof_score", _SimScore),
                           ("sim_sizeof_result", _SimResult)):
            got = getattr(self._lib, fn)()
            if got != ctypes.sizeof(struct):
                raise FloodSimError(f"{fn}: DLL {got} bytes, wrapper {ctypes.sizeof(struct)} -- rebuild")

    def _declare(self) -> None:
        lib = self._lib

        def fn(name, restype, *argtypes):
            f = getattr(lib, name)
            f.restype = restype
            f.argtypes = list(argtypes)

        for name in ("sim_abi_version", "sim_sizeof_box", "sim_sizeof_score", "sim_sizeof_result",
                     "sim_upload_downscale"):
            fn(name, c_int32)
        for name in ("sim_submerged_below", "sim_veto_threshold", "sim_sample_period_s",
                     "sim_domain_x0", "sim_domain_x1"):
            fn(name, c_float)
        for name in ("sim_outcome_name", "sim_severity_name", "sim_class_name"):
            fn(name, c_char_p, c_int32)
        fn("sim_default_geometry", None, _FP)
        fn("sim_get_geometry", None, _FP)
        fn("sim_set_geometry", None, _FP)
        fn("sim_validate_geometry", c_int32, _FP, c_char_p, c_int32)
        fn("sim_align_patch", None, POINTER(c_int32))
        fn("sim_strip_outline", c_int32, _FP, _FP, _FP, c_int32)
        fn("sim_rgb_to_gray", None, _U8P, c_int32, c_int32, _U8P)
        fn("sim_ref_new", c_void_p)
        fn("sim_ref_free", None, c_void_p)
        fn("sim_ref_build", c_int32, c_void_p, _U8P)
        fn("sim_ref_ready", c_int32, c_void_p)
        fn("sim_correlation_at", c_float, c_void_p, _U8P, c_int32, c_int32)
        fn("sim_box_from_accelerator", None, c_float, c_float, c_float, c_float, c_float,
           c_int32, c_int32, c_int32, POINTER(_SimBox))
        fn("sim_score", c_int32, c_void_p, _U8P, POINTER(_SimBox), c_int32, POINTER(_SimScore))
        fn("sim_device_new", c_void_p)
        fn("sim_device_free", None, c_void_p)
        fn("sim_device_reset", None, c_void_p)
        fn("sim_device_sample", None, c_void_p, c_void_p, _PROVIDER_FN, c_void_p, POINTER(_SimResult))

    # names and constants
    def outcome_name(self, v: int) -> str: return self._lib.sim_outcome_name(v).decode()
    def severity_name(self, v: int) -> str: return self._lib.sim_severity_name(v).decode()
    def class_name(self, v: int) -> str: return self._lib.sim_class_name(v).decode()
    def submerged_below(self) -> float: return self._lib.sim_submerged_below()
    def veto_threshold(self) -> float: return self._lib.sim_veto_threshold()
    def sample_period_s(self) -> float: return self._lib.sim_sample_period_s()
    def upload_downscale(self) -> int: return self._lib.sim_upload_downscale()

    # geometry
    def default_geometry(self) -> list[float]:
        a = _F8()
        self._lib.sim_default_geometry(a)
        return list(a)

    def get_geometry(self) -> list[float]:
        a = _F8()
        self._lib.sim_get_geometry(a)
        return list(a)

    def set_geometry(self, g: Geometry) -> None:
        self._lib.sim_set_geometry(_F8(*g.to_list()))

    def validate_geometry(self, g: Geometry) -> str | None:
        err = ctypes.create_string_buffer(256)
        ok = self._lib.sim_validate_geometry(_F8(*g.to_list()), err, len(err))
        return None if ok else err.value.decode()

    def domain(self) -> tuple[float, float]:
        return self._lib.sim_domain_x0(), self._lib.sim_domain_x1()

    def align_patch(self) -> tuple[int, int, int, int]:
        a = (c_int32 * 4)()
        self._lib.sim_align_patch(a)
        return tuple(a)

    def strip_outline(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        cap = 512
        xs, lo, hi = (np.zeros(cap, np.float32) for _ in range(3))
        n = self._lib.sim_strip_outline(xs.ctypes.data_as(_FP), lo.ctypes.data_as(_FP),
                                        hi.ctypes.data_as(_FP), cap)
        return xs[:n].copy(), lo[:n].copy(), hi[:n].copy()

    # frames and scoring
    def rgb_to_gray(self, rgb: np.ndarray) -> np.ndarray:
        rgb = np.ascontiguousarray(rgb)
        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
            raise FloodSimError(f"expected an HxWx3 uint8 RGB frame, got {rgb.shape} {rgb.dtype}")
        h, w = rgb.shape[:2]
        out = np.empty((h, w), np.uint8)
        self._lib.sim_rgb_to_gray(_u8(rgb), w, h, _u8(out))
        return out

    def new_reference(self) -> Reference:
        return Reference(self)

    def score(self, ref: Reference | None, gray: np.ndarray, boxes: list[Box]) -> ScoreResult:
        if ref is None or not ref.ready:
            raise FloodSimError("no dry reference: set one before scoring")
        gray = _check_gray(gray)
        arr, n = _sim_boxes(boxes)
        s = _SimScore()
        self._lib.sim_score(ref.ptr, _u8(gray), arr, n, byref(s))
        return ScoreResult(
            veto_usable=bool(s.veto_usable), veto_coverage=s.veto_coverage,
            veto_worst_class=self.class_name(s.veto_worst_class),
            curb_correlation=s.curb_correlation, submerged=bool(s.submerged),
            severity=self.severity_name(s.severity), shift_dx=s.shift_dx, shift_dy=s.shift_dy)

    def correlation_at(self, ref: Reference, gray: np.ndarray, dx: int, dy: int) -> float:
        return self._lib.sim_correlation_at(ref.ptr, _u8(_check_gray(gray)), dx, dy)

    def box_from_accelerator(self, x0, y0, x1, y1, conf, cls, input_w, input_h) -> Box:
        b = _SimBox()
        self._lib.sim_box_from_accelerator(x0, y0, x1, y1, conf, cls, input_w, input_h, byref(b))
        return Box(b.x0, b.y0, b.x1, b.y1, b.confidence, b.cls)

    def new_device(self) -> Device:
        return Device(self)

    def _device_result(self, r: _SimResult) -> DeviceResult:
        return DeviceResult(
            outcome=self.outcome_name(r.outcome), frames_taken=r.frames_taken,
            has_reading=bool(r.has_reading), curb_correlation=r.curb_correlation,
            severity=self.severity_name(r.severity), shift_dx=r.shift_dx, shift_dy=r.shift_dy,
            veto_usable=bool(r.veto_usable), veto_coverage=r.veto_coverage,
            veto_worst_class=self.class_name(r.veto_worst_class), since_image=r.since_image,
            frame_idx=r.frame_idx)


class Reference:
    """Opaque firmware flood_reference_t, built from a dry gray frame."""

    def __init__(self, core: Core):
        self._core = core
        self.ptr = core._lib.sim_ref_new()
        if not self.ptr:
            raise MemoryError("sim_ref_new failed")
        self.geometry: tuple[float, ...] | None = None

    def build(self, gray: np.ndarray) -> bool:
        ok = bool(self._core._lib.sim_ref_build(self.ptr, _u8(_check_gray(gray))))
        self.geometry = tuple(self._core.get_geometry()) if ok else None
        return ok

    @property
    def ready(self) -> bool:
        return bool(self.ptr) and bool(self._core._lib.sim_ref_ready(self.ptr))

    def is_stale(self) -> bool:
        """Built under different geometry than the current one (firmware: geometry_version)."""
        return self.ready and self.geometry != tuple(self._core.get_geometry())

    def close(self) -> None:
        if self.ptr:
            self._core._lib.sim_ref_free(self.ptr)
            self.ptr = None

    def __del__(self):
        self.close()


class Device:
    """Firmware capture state (retry, heartbeat) driven by a frame provider."""

    def __init__(self, core: Core):
        self._core = core
        self._ptr = core._lib.sim_device_new()
        if not self._ptr:
            raise MemoryError("sim_device_new failed")

    def reset(self) -> None:
        self._core._lib.sim_device_reset(self._ptr)

    def sample(self, ref: Reference | None, provider: Provider) -> DeviceResult:
        errors: list[BaseException] = []

        def callback(attempt, gray_out, boxes_out, n_out, idx_out, _user):
            try:
                got = provider(attempt)
                if got is None:
                    return 0
                gray, boxes, frame_idx = got
                gray = _check_gray(gray)
                ctypes.memmove(gray_out, gray.ctypes.data, gray.nbytes)
                n = min(len(boxes), MAX_BOXES)
                for i, b in enumerate(boxes[:n]):
                    boxes_out[i] = _SimBox(b.x0, b.y0, b.x1, b.y1, b.confidence, b.cls)
                n_out[0] = n
                idx_out[0] = frame_idx
                return 1
            except BaseException as e:  # re-raised after the C call returns
                errors.append(e)
                return 0

        cb = _PROVIDER_FN(callback)
        res = _SimResult()
        self._core._lib.sim_device_sample(self._ptr, ref.ptr if ref is not None else None,
                                          cb, None, byref(res))
        if errors:
            raise errors[0]
        return self._core._device_result(res)

    def close(self) -> None:
        if self._ptr:
            self._core._lib.sim_device_free(self._ptr)
            self._ptr = None

    def __del__(self):
        self.close()
```

- [ ] **Step 8: Run the tests**

Run: `.venv/Scripts/python -m pytest tests/test_core.py tests/test_geometry.py -v`
Expected: all pass. The first run may rebuild the DLL because the firmware marker mtimes differ; that's fine.

- [ ] **Step 9: Commit**

```bash
git add pyproject.toml floodsim tests
git commit -m "feat(python): ctypes core with auto-rebuild, geometry JSON" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 5: Frame sources (image, folder, video)

**Files:**
- Create: `floodsim/source.py`, `tests/test_source.py`
- Modify: `tests/synth.py` (add `MemorySource`, `write_frame_folder`)

**Interfaces:**
- Consumes: `FloodSimError`.
- Produces: `FRAME_W, FRAME_H, IMAGE_EXTS`; `read_image_bgr(path) -> np.ndarray | None`; `write_image(path, img)`; `Normalizer(allow_stretch)(bgr, where) -> rgb`; `FrameSource` base with `path`, `n_frames`, `timestamp(i)`, `read_rgb(i) -> np.ndarray | None` (read-only RGB 1080×1920×3), `frame_name(i) -> str | None`, `fingerprint() -> dict`, `duration`, `index_at(t) -> int | None` (latest frame with timestamp ≤ t); `ImageSequence`, `VideoSource`; `open_source(path, *, fps=None, allow_stretch=False) -> FrameSource`.
- Produces (tests): `MemorySource(frames: list[tuple[float, np.ndarray | None]])`, `write_frame_folder(dir, frames: list[tuple[int, np.ndarray]]) -> Path` (frames as `(t_ms, gray)`, written as PNG `f{i:06d}_t{ms:010d}.png`).

- [ ] **Step 1: Add the test helpers to `tests/synth.py`** (append)

```python
from pathlib import Path

from floodsim.source import FrameSource, write_image


class MemorySource(FrameSource):
    """In-memory frames for runner tests; None marks a frame that fails to decode."""

    def __init__(self, frames):
        self.path = Path("memory")
        self._times = [float(t) for t, _ in frames]
        self._rgb = [rgb for _, rgb in frames]
        self.n_frames = len(frames)

    def timestamp(self, idx):
        return self._times[idx]

    def read_rgb(self, idx):
        return self._rgb[idx]

    def fingerprint(self):
        return {"path": "memory", "n": self.n_frames}


def write_frame_folder(folder: Path, frames) -> Path:
    """frames: list of (t_ms, gray). Lossless PNGs named f<i>_t<ms>.png."""
    folder.mkdir(parents=True, exist_ok=True)
    for i, (t_ms, gray) in enumerate(frames):
        write_image(folder / f"f{i:06d}_t{t_ms:010d}.png", to_rgb(gray))
    return folder
```

(`to_rgb` output has equal channels, so writing it as BGR is the same image.)

- [ ] **Step 2: Write the failing tests `tests/test_source.py`**

```python
import logging

import cv2
import numpy as np
import pytest

from floodsim.errors import FloodSimError
from floodsim.source import open_source, write_image
from tests.synth import dry_gray, to_rgb, write_frame_folder


def _png(path, w, h, value=100):
    write_image(path, np.full((h, w, 3), value, np.uint8))
    return path


def test_single_image_is_one_frame_at_zero(tmp_path):
    src = open_source(_png(tmp_path / "a.png", 1920, 1080))
    assert src.n_frames == 1 and src.timestamp(0) == 0.0
    assert src.read_rgb(0).shape == (1080, 1920, 3)
    assert src.index_at(0.0) == 0 and src.index_at(500.0) == 0 and src.index_at(-1.0) is None


def test_folder_uses_t_ms_names(tmp_path):
    folder = write_frame_folder(tmp_path / "f", [(0, dry_gray()), (1000, dry_gray()), (60000, dry_gray())])
    src = open_source(folder)
    assert [src.timestamp(i) for i in range(3)] == [0.0, 1.0, 60.0]
    assert src.index_at(59.9) == 1 and src.index_at(60.0) == 2 and src.index_at(1e6) == 2
    assert src.frame_name(2) == "f000002_t0000060000.png"
    assert np.array_equal(src.read_rgb(1), to_rgb(dry_gray()))


def test_folder_without_times_needs_fps(tmp_path):
    for name in ("a.png", "b.png"):
        _png(tmp_path / name, 1920, 1080)
    with pytest.raises(FloodSimError, match="--fps"):
        open_source(tmp_path)
    src = open_source(tmp_path, fps=4.0)
    assert src.timestamp(1) == 0.25


def test_resize_720p_warns_once(tmp_path, caplog):
    for name in ("a.png", "b.png"):
        _png(tmp_path / name, 1280, 720)
    src = open_source(tmp_path, fps=1.0)
    with caplog.at_level(logging.WARNING):
        assert src.read_rgb(0).shape == (1080, 1920, 3)
        src.read_rgb(1)
    assert sum("resizing" in r.message for r in caplog.records) <= 1


def test_non_16_9_refused_unless_allow_stretch(tmp_path):
    path = _png(tmp_path / "square.png", 800, 600)
    with pytest.raises(FloodSimError, match="16:9"):
        open_source(path)
    assert open_source(path, allow_stretch=True).read_rgb(0).shape == (1080, 1920, 3)


def test_corrupt_image_returns_none(tmp_path):
    _png(tmp_path / "f000000_t0000000000.png", 1920, 1080)
    (tmp_path / "f000001_t0000001000.png").write_bytes(b"not a png")
    src = open_source(tmp_path)
    assert src.read_rgb(0) is not None
    assert src.read_rgb(1) is None


def test_video_frames_and_times(tmp_path):
    path = tmp_path / "clip.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 4.0, (640, 360))
    for v in (0, 80, 160, 240):
        writer.write(np.full((360, 640, 3), v, np.uint8))
    writer.release()
    src = open_source(path)
    assert src.n_frames == 4
    assert src.timestamp(3) == pytest.approx(0.75)
    assert src.read_rgb(3).shape == (1080, 1920, 3)
    later, earlier = src.read_rgb(3), src.read_rgb(1)  # backwards seek reopens the file
    assert earlier.mean() < later.mean()


def test_non_ascii_folder(tmp_path):
    folder = write_frame_folder(tmp_path / "测试 clips", [(0, dry_gray())])
    src = open_source(folder)
    assert src.read_rgb(0) is not None
```

Run: `.venv/Scripts/python -m pytest tests/test_source.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'floodsim.source'`. The `tests/synth.py` import also fails at this point, which is expected.

- [ ] **Step 3: Write `floodsim/source.py`**

```python
"""Image / image-folder / video inputs as RGB 1920x1080 frames with timestamps."""
from __future__ import annotations

import bisect
import logging
import re
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np

from .errors import FloodSimError

FRAME_W, FRAME_H = 1920, 1080
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".pgm", ".ppm", ".tif", ".tiff"}
_T_MS = re.compile(r"_t(\d+)")
log = logging.getLogger(__name__)


def read_image_bgr(path: Path) -> np.ndarray | None:
    """cv2.imread cannot open non-ASCII paths on Windows, so decode from bytes."""
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def write_image(path: Path, img: np.ndarray) -> None:
    ok, buf = cv2.imencode(Path(path).suffix, img)
    if not ok:
        raise FloodSimError(f"cannot encode {path}")
    buf.tofile(str(path))


class Normalizer:
    """BGR frame of any size -> RGB 1920x1080, refusing non-16:9 unless allowed."""

    def __init__(self, allow_stretch: bool):
        self.allow_stretch = allow_stretch
        self._warned = False

    def __call__(self, bgr: np.ndarray, where: str) -> np.ndarray:
        h, w = bgr.shape[:2]
        if (w, h) != (FRAME_W, FRAME_H):
            if abs(w / h - FRAME_W / FRAME_H) > 0.01 and not self.allow_stretch:
                raise FloodSimError(
                    f"{where}: {w}x{h} is not 16:9, so the curb geometry would not line up "
                    f"(use --allow-stretch to resize anyway)")
            if not self._warned:
                log.warning("%s: resizing %dx%d to %dx%d", where, w, h, FRAME_W, FRAME_H)
                self._warned = True
            bgr = cv2.resize(bgr, (FRAME_W, FRAME_H), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        return rgb


class FrameSource:
    """Frame i has timestamp(i) seconds; read_rgb(i) is read-only RGB 1080x1920x3, or None."""

    path: Path
    n_frames: int
    _times: list[float]

    def timestamp(self, idx: int) -> float:
        return self._times[idx]

    def read_rgb(self, idx: int) -> np.ndarray | None:
        raise NotImplementedError

    def frame_name(self, idx: int) -> str | None:
        return None

    def fingerprint(self) -> dict:
        raise NotImplementedError

    @property
    def duration(self) -> float:
        return self.timestamp(self.n_frames - 1)

    def index_at(self, t: float) -> int | None:
        """Latest frame at or before t -- what a camera polled at t would hand over."""
        i = bisect.bisect_right(self._times, t + 1e-9) - 1
        return i if i >= 0 else None


class ImageSequence(FrameSource):
    def __init__(self, path: Path, files: list[Path], fps: float | None, normalizer: Normalizer):
        self.path = path
        self._normalize = normalizer
        matches = [_T_MS.search(f.stem) for f in files]
        if all(matches):
            pairs = sorted(zip((int(m.group(1)) / 1000.0 for m in matches), files), key=lambda p: p[0])
            self._times = [t for t, _ in pairs]
            self._files = [f for _, f in pairs]
        elif len(files) == 1:
            self._times, self._files = [0.0], files
        elif fps:
            self._times, self._files = [i / fps for i in range(len(files))], files
        else:
            raise FloodSimError(
                f"{path}: image frames need --fps or file names containing _t<ms> "
                f"(e.g. f000001_t0000060000.png)")
        self.n_frames = len(self._files)

    def read_rgb(self, idx: int) -> np.ndarray | None:
        f = self._files[idx]
        bgr = read_image_bgr(f)
        if bgr is None:
            log.warning("cannot decode %s", f)
            return None
        return self._normalize(bgr, f.name)

    def frame_name(self, idx: int) -> str | None:
        return self._files[idx].name

    def fingerprint(self) -> dict:
        stats = [f.stat() for f in self._files]
        return {"path": str(self.path), "n": self.n_frames,
                "size": sum(s.st_size for s in stats), "mtime": max(s.st_mtime for s in stats)}


class VideoSource(FrameSource):
    _CACHE = 16

    def __init__(self, path: Path, fps: float | None, normalizer: Normalizer):
        self.path = path
        self._normalize = normalizer
        self._cap = self._open()
        fps = fps or self._cap.get(cv2.CAP_PROP_FPS)
        if not fps or fps <= 0:
            raise FloodSimError(f"{path}: video reports no frame rate; pass --fps")
        n = 0
        while self._cap.grab():  # count by walking: container frame counts lie for MJPEG AVIs
            n += 1
        if n == 0:
            raise FloodSimError(f"{path}: no decodable frames")
        self.n_frames = n
        self._times = [i / fps for i in range(n)]
        self._cap.release()
        self._cap = self._open()
        self._pos = 0
        self._cache: OrderedDict[int, np.ndarray] = OrderedDict()

    def _open(self) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(str(self.path))
        if not cap.isOpened():
            hint = "" if str(self.path).isascii() else \
                " (OpenCV cannot open videos at non-ASCII paths on Windows; move or rename it)"
            raise FloodSimError(f"cannot open video {self.path}{hint}")
        return cap

    def read_rgb(self, idx: int) -> np.ndarray | None:
        if idx in self._cache:
            self._cache.move_to_end(idx)
            return self._cache[idx]
        if idx < self._pos:
            self._cap.release()
            self._cap = self._open()
            self._pos = 0
        while self._pos < idx:
            if not self._cap.grab():
                self._pos = self.n_frames  # force a reopen next time
                return None
            self._pos += 1
        ok, bgr = self._cap.read()
        self._pos += 1
        if not ok or bgr is None:
            log.warning("%s: frame %d failed to decode", self.path.name, idx)
            return None
        rgb = self._normalize(bgr, f"{self.path.name} frame {idx}")
        self._cache[idx] = rgb
        if len(self._cache) > self._CACHE:
            self._cache.popitem(last=False)
        return rgb

    def fingerprint(self) -> dict:
        s = self.path.stat()
        return {"path": str(self.path), "size": s.st_size, "mtime": s.st_mtime}


def open_source(path: Path, *, fps: float | None = None, allow_stretch: bool = False) -> FrameSource:
    path = Path(path)
    if not path.exists():
        raise FloodSimError(f"input not found: {path}")
    normalizer = Normalizer(allow_stretch)
    if path.is_dir():
        files = sorted(f for f in path.iterdir() if f.suffix.lower() in IMAGE_EXTS)
        if not files:
            raise FloodSimError(f"{path}: no image files")
        src: FrameSource = ImageSequence(path, files, fps, normalizer)
    elif path.suffix.lower() in IMAGE_EXTS:
        src = ImageSequence(path, [path], fps, normalizer)
    else:
        src = VideoSource(path, fps, normalizer)
    src.read_rgb(0)  # fail fast on a bad aspect ratio
    return src
```

- [ ] **Step 4: Run the tests**

Run: `.venv/Scripts/python -m pytest tests/test_source.py -v`
Expected: 8 passed.

- [ ] **Step 5: Commit**

```bash
git add floodsim/source.py tests/synth.py tests/test_source.py
git commit -m "feat(python): image, folder and video frame sources" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 6: Stage-1 boxes (YOLO, cache, edits)

**Files:**
- Create: `floodsim/boxes.py`, `tests/test_boxes.py`

**Interfaces:**
- Consumes: `Core.box_from_accelerator`, `FrameSource.fingerprint/path`, `Box`, `FloodSimError`.
- Produces: `DETECTOR_INPUT = (640, 480)`, `DEFAULT_MODEL = "yolo26n.pt"`, `DEFAULT_CONF = 0.25`, `COCO_TO_FLOOD`, `CSV_FIELDS`; `CachedBox(x0, y0, x1, y1, conf, coco_class, flood_class)` with `to_box()`; `Detection(x0, y0, x1, y1, conf, coco_class)` in detector-input coords; `Detector` protocol (`model_name`, `conf`, `detect(rgb_small) -> list[Detection]`); `YoloDetector(model, conf)`; `cache_meta(fingerprint, model, conf) -> dict`; `BoxStore(input_path, meta)` with `cache_path`, `edited_path`, `has_cache`, `get(idx)`, `put_detected(idx, t, boxes)`, `set_edited(idx, t, boxes)`, `clear_edited(idx)`, `is_edited(idx)`; `write_boxes_csv(path, entries, meta)` where entries = `list[tuple[int, float, list[CachedBox]]]`; `BoxProvider(core, store, detector)` with `none(core)`, `description`, `cached_for(idx, t, load_rgb) -> list[CachedBox]`, `boxes_for(idx, t, load_rgb) -> list[Box]`; `make_box_provider(core, source, *, no_detector, model, conf) -> BoxProvider`.
- Cache file: `# meta: <json>` line, header `CSV_FIELDS`, one row per box; a frame with no detections gets a marker row with empty box fields. Floats are written `%.9g`, so a float32 round-trips exactly: first-run and cached-run results are identical.

- [ ] **Step 1: Write the failing tests `tests/test_boxes.py`**

```python
import pytest

from floodsim.boxes import (BoxProvider, BoxStore, CachedBox, Detection, cache_meta,
                            write_boxes_csv)
from floodsim.errors import FloodSimError
from floodsim.results import Box
from tests.synth import dry_gray, to_rgb


class FakeDetector:
    model_name = "fake.pt"
    conf = 0.25

    def __init__(self, dets):
        self.dets = dets
        self.calls = 0

    def detect(self, rgb_small):
        assert rgb_small.shape == (480, 640, 3)
        self.calls += 1
        return list(self.dets)


FINGERPRINT = {"path": "clip.avi", "size": 10, "mtime": 1.0}
RGB = to_rgb(dry_gray())


def _provider(core, tmp_path, dets, meta=None):
    det = FakeDetector(dets)
    store = BoxStore(tmp_path / "clip.avi", meta or cache_meta(FINGERPRINT, det.model_name, det.conf))
    return BoxProvider(core, store, det), det, store


def test_detector_boxes_scaled_to_frame(core, tmp_path):
    prov, det, _ = _provider(core, tmp_path, [Detection(10, 20, 30, 40, 0.8, "car")])
    assert prov.boxes_for(0, 0.0, lambda: RGB) == [Box(30.0, 45.0, 90.0, 90.0, pytest.approx(0.8), 2)]


def test_cache_reused_without_detector_call(core, tmp_path):
    prov, det, _ = _provider(core, tmp_path, [Detection(10, 20, 30, 40, 0.8, "car")])
    first = prov.boxes_for(0, 0.0, lambda: RGB)
    prov2, det2, store2 = _provider(core, tmp_path, [])
    assert store2.has_cache
    assert prov2.boxes_for(0, 0.0, lambda: RGB) == first
    assert det2.calls == 0


def test_frame_with_no_detections_is_cached(core, tmp_path):
    prov, det, _ = _provider(core, tmp_path, [])
    assert prov.boxes_for(3, 0.75, lambda: RGB) == []
    prov2, det2, _ = _provider(core, tmp_path, [Detection(0, 0, 1, 1, 0.9, "car")])
    assert prov2.boxes_for(3, 0.75, lambda: RGB) == []
    assert det2.calls == 0


def test_cache_invalidated_when_settings_change(core, tmp_path):
    prov, _, _ = _provider(core, tmp_path, [Detection(10, 20, 30, 40, 0.8, "car")])
    prov.boxes_for(0, 0.0, lambda: RGB)
    other = cache_meta(FINGERPRINT, "fake.pt", 0.5)
    prov2, det2, store2 = _provider(core, tmp_path, [], meta=other)
    assert not store2.has_cache
    assert prov2.boxes_for(0, 0.0, lambda: RGB) == []
    assert det2.calls == 1


def test_edited_boxes_win(core, tmp_path):
    prov, det, store = _provider(core, tmp_path, [Detection(10, 20, 30, 40, 0.8, "car")])
    prov.boxes_for(0, 0.0, lambda: RGB)
    store.set_edited(0, 0.0, [CachedBox(1, 2, 3, 4, 1.0, "person", 1)])
    prov2, _, store2 = _provider(core, tmp_path, [])
    assert store2.is_edited(0)
    assert prov2.boxes_for(0, 0.0, lambda: RGB) == [Box(1.0, 2.0, 3.0, 4.0, 1.0, 1)]
    store2.clear_edited(0)
    assert prov2.boxes_for(0, 0.0, lambda: RGB)[0].cls == 2


def test_missing_frame_without_detector_raises(core, tmp_path):
    store = BoxStore(tmp_path / "clip.avi", cache_meta(FINGERPRINT, "fake.pt", 0.25))
    with pytest.raises(FloodSimError, match="no detector"):
        BoxProvider(core, store, None).boxes_for(7, 1.0, lambda: RGB)


def test_none_provider_returns_nothing(core):
    prov = BoxProvider.none(core)
    assert prov.boxes_for(0, 0.0, lambda: RGB) == []
    assert prov.description == {"kind": "no-detector"}


def test_unknown_coco_class_maps_to_other(core, tmp_path):
    prov, _, _ = _provider(core, tmp_path, [Detection(0, 0, 10, 10, 0.9, "dog")])
    assert prov.boxes_for(0, 0.0, lambda: RGB)[0].cls == 0


def test_truncated_cache_line_is_skipped(core, tmp_path):
    prov, _, store = _provider(core, tmp_path, [Detection(10, 20, 30, 40, 0.8, "car")])
    prov.boxes_for(0, 0.0, lambda: RGB)
    prov.boxes_for(1, 0.25, lambda: RGB)
    text = store.cache_path.read_text()
    store.cache_path.write_text(text[: text.rstrip("\n").rfind(",")])  # cut the last row mid-way
    prov2, det2, _ = _provider(core, tmp_path, [])
    assert len(prov2.boxes_for(0, 0.0, lambda: RGB)) == 1
    assert prov2.boxes_for(1, 0.25, lambda: RGB) == []   # re-detected (fake now finds nothing)
    assert det2.calls == 1
    _, _, store3 = _provider(core, tmp_path, [])        # the append did not merge into the cut line
    assert store3.get(0)[0].flood_class == 2 and store3.get(1) == []


def test_write_boxes_csv_for_exe(tmp_path):
    path = tmp_path / "boxes.csv"
    write_boxes_csv(path, [(4, 120.0, [CachedBox(0, 0, 1919, 1079, 0.9, "car", 2)]), (5, 121.0, [])],
                    {"export": True})
    lines = path.read_text().splitlines()
    assert lines[1] == "frame_idx,t_sec,x0,y0,x1,y1,conf,coco_class,flood_class"
    assert lines[2] == "4,120.000,0,0,1919,1079,0.9,car,2"
    assert lines[3] == "5,121.000,,,,,,,"
```

Run: `.venv/Scripts/python -m pytest tests/test_boxes.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'floodsim.boxes'`.

- [ ] **Step 2: Write `floodsim/boxes.py`**

```python
"""Stage-1 boxes: YOLO on the PC standing in for the IMX500, with a per-frame cache."""
from __future__ import annotations

import csv
import json
import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol

import cv2
import numpy as np

from .errors import FloodSimError
from .results import Box

log = logging.getLogger(__name__)

DETECTOR_INPUT = (640, 480)   # IMX500 accelerator input (ms500/veto.py IMX500_INPUT)
DEFAULT_MODEL = "yolo26n.pt"
DEFAULT_CONF = 0.25
# COCO name -> flood_class_t. The firmware maps its traffic model's classes the
# same way in map_traffic_class(), main/ai_process/flood_task.c:101-112.
COCO_TO_FLOOD = {"person": 1, "car": 2, "truck": 3, "bus": 4, "bicycle": 5, "motorcycle": 6}
CSV_FIELDS = ["frame_idx", "t_sec", "x0", "y0", "x1", "y1", "conf", "coco_class", "flood_class"]


@dataclass(frozen=True)
class CachedBox:
    x0: float
    y0: float
    x1: float
    y1: float
    conf: float
    coco_class: str
    flood_class: int

    def to_box(self) -> Box:
        return Box(self.x0, self.y0, self.x1, self.y1, self.conf, self.flood_class)


@dataclass(frozen=True)
class Detection:
    """In detector-input (640x480) coordinates."""
    x0: float
    y0: float
    x1: float
    y1: float
    conf: float
    coco_class: str


class Detector(Protocol):
    model_name: str
    conf: float

    def detect(self, rgb_small: np.ndarray) -> list[Detection]: ...


class YoloDetector:
    def __init__(self, model: str = DEFAULT_MODEL, conf: float = DEFAULT_CONF):
        try:
            from ultralytics import YOLO  # heavy and optional
        except ImportError as e:
            raise FloodSimError(
                "ultralytics is not installed (pip install -e .[yolo]); "
                "use --no-detector to run without Stage-1 boxes") from e
        self.model_name = model
        self.conf = conf
        self._model = YOLO(model)

    def detect(self, rgb_small: np.ndarray) -> list[Detection]:
        out = self._model.predict(cv2.cvtColor(rgb_small, cv2.COLOR_RGB2BGR),
                                  verbose=False, conf=self.conf)[0]
        dets = []
        for b in out.boxes:
            x0, y0, x1, y1 = (float(v) for v in b.xyxy[0])
            dets.append(Detection(x0, y0, x1, y1, float(b.conf), out.names[int(b.cls)]))
        return dets


def cache_meta(fingerprint: dict, model: str, conf: float) -> dict:
    return {"model": model, "conf": conf, "input": list(DETECTOR_INPUT), "source": fingerprint}


def _num(v: float) -> str:
    return f"{v:.9g}"   # 9 significant digits round-trip a float32 exactly


def _row(idx: int, t: float, b: CachedBox | None) -> list[str]:
    if b is None:   # marker: frame detected, nothing found
        return [str(idx), f"{t:.3f}", "", "", "", "", "", "", ""]
    return [str(idx), f"{t:.3f}", _num(b.x0), _num(b.y0), _num(b.x1), _num(b.y1),
            _num(b.conf), b.coco_class, str(b.flood_class)]


def _rows_for(idx: int, t: float, boxes: list[CachedBox]) -> list[list[str]]:
    return [_row(idx, t, b) for b in boxes] if boxes else [_row(idx, t, None)]


def write_boxes_csv(path: Path, entries, meta: dict | None) -> None:
    """entries: list of (frame_idx, t_sec, list[CachedBox])."""
    with open(path, "w", newline="", encoding="utf-8") as f:
        f.write(f"# meta: {json.dumps(meta, sort_keys=True)}\n")
        w = csv.writer(f, lineterminator="\n")
        w.writerow(CSV_FIELDS)
        for idx, t, boxes in entries:
            w.writerows(_rows_for(idx, t, boxes))


def _ensure_trailing_newline(path: Path) -> None:
    """A run killed mid-write leaves a partial last line; never append onto it."""
    with open(path, "rb+") as f:
        f.seek(0, os.SEEK_END)
        if f.tell() == 0:
            return
        f.seek(-1, os.SEEK_END)
        if f.read(1) != b"\n":
            f.write(b"\n")


def _read_csv(path: Path) -> tuple[dict | None, dict[int, list[CachedBox]], dict[int, float]]:
    meta = None
    rows: dict[int, list[CachedBox]] = {}
    times: dict[int, float] = {}
    with open(path, newline="", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            if line.startswith("# meta: "):
                try:
                    meta = json.loads(line[len("# meta: "):])
                except json.JSONDecodeError:
                    meta = None
                continue
            if line.startswith("#") or line.startswith("frame_idx") or not line.strip():
                continue
            fields = next(csv.reader([line]))
            try:
                if len(fields) != len(CSV_FIELDS):
                    raise ValueError(f"{len(fields)} fields")
                idx, t = int(fields[0]), float(fields[1])
                boxes = rows.setdefault(idx, [])
                times[idx] = t
                if fields[2] != "":
                    x0, y0, x1, y1, conf = (float(v) for v in fields[2:7])
                    boxes.append(CachedBox(x0, y0, x1, y1, conf, fields[7], int(fields[8])))
            except ValueError as e:
                log.warning("%s:%d: skipping malformed box row (%s)", path, line_no, e)
    return meta, rows, times


class BoxStore:
    """<input>.boxes.csv (detector cache) + <input>.boxes.edited.csv (manual edits, which win)."""

    def __init__(self, input_path: Path, meta: dict):
        self.cache_path = Path(f"{input_path}.boxes.csv")
        self.edited_path = Path(f"{input_path}.boxes.edited.csv")
        self.meta = meta
        self._cache: dict[int, list[CachedBox]] = {}
        self._edited: dict[int, list[CachedBox]] = {}
        self._edited_times: dict[int, float] = {}
        if self.cache_path.exists():
            file_meta, rows, _ = _read_csv(self.cache_path)
            if file_meta == json.loads(json.dumps(meta)):
                self._cache = rows
            else:
                log.warning("%s was made with different settings; discarding it", self.cache_path)
                write_boxes_csv(self.cache_path, [], meta)
        if self.edited_path.exists():
            _, self._edited, self._edited_times = _read_csv(self.edited_path)

    @property
    def has_cache(self) -> bool:
        return bool(self._cache)

    def get(self, idx: int) -> list[CachedBox] | None:
        if idx in self._edited:
            return self._edited[idx]
        return self._cache.get(idx)

    def is_edited(self, idx: int) -> bool:
        return idx in self._edited

    def put_detected(self, idx: int, t: float, boxes: list[CachedBox]) -> None:
        self._cache[idx] = boxes
        if not self.cache_path.exists():
            write_boxes_csv(self.cache_path, [], self.meta)
        _ensure_trailing_newline(self.cache_path)
        with open(self.cache_path, "a", newline="", encoding="utf-8") as f:
            csv.writer(f, lineterminator="\n").writerows(_rows_for(idx, t, boxes))

    def set_edited(self, idx: int, t: float, boxes: list[CachedBox]) -> None:
        self._edited[idx] = boxes
        self._edited_times[idx] = t
        self._write_edited()

    def clear_edited(self, idx: int) -> None:
        self._edited.pop(idx, None)
        self._edited_times.pop(idx, None)
        self._write_edited()

    def _write_edited(self) -> None:
        entries = [(i, self._edited_times.get(i, 0.0), self._edited[i]) for i in sorted(self._edited)]
        write_boxes_csv(self.edited_path, entries, {"edited": True})


class BoxProvider:
    """Boxes for a frame: edited > cache > detector. `none()` gives no boxes at all."""

    def __init__(self, core, store: BoxStore | None, detector: Detector | None):
        self.core = core
        self.store = store
        self.detector = detector

    @classmethod
    def none(cls, core) -> BoxProvider:
        return cls(core, None, None)

    @property
    def description(self) -> dict:
        if self.store is None:
            return {"kind": "no-detector"}
        return {"kind": "yolo" if self.detector else "cache",
                "model": self.store.meta["model"], "conf": self.store.meta["conf"],
                "cache_file": str(self.store.cache_path), "edited_file": str(self.store.edited_path)}

    def cached_for(self, idx: int, t: float,
                   load_rgb: Callable[[], np.ndarray | None]) -> list[CachedBox]:
        if self.store is None:
            return []
        got = self.store.get(idx)
        if got is not None:
            return got
        if self.detector is None:
            raise FloodSimError(
                f"frame {idx} is not in {self.store.cache_path} and no detector is available "
                f"(install ultralytics or use --no-detector)")
        rgb = load_rgb()
        if rgb is None:
            return []
        small = cv2.resize(rgb, DETECTOR_INPUT, interpolation=cv2.INTER_AREA)
        got = []
        for d in self.detector.detect(small):
            fc = COCO_TO_FLOOD.get(d.coco_class, 0)
            b = self.core.box_from_accelerator(d.x0, d.y0, d.x1, d.y1, d.conf, fc, *DETECTOR_INPUT)
            got.append(CachedBox(b.x0, b.y0, b.x1, b.y1, b.confidence, d.coco_class, fc))
        self.store.put_detected(idx, t, got)
        return got

    def boxes_for(self, idx: int, t: float, load_rgb: Callable[[], np.ndarray | None]) -> list[Box]:
        return [c.to_box() for c in self.cached_for(idx, t, load_rgb)]


def make_box_provider(core, source, *, no_detector: bool, model: str = DEFAULT_MODEL,
                      conf: float = DEFAULT_CONF) -> BoxProvider:
    if no_detector:
        return BoxProvider.none(core)
    store = BoxStore(source.path, cache_meta(source.fingerprint(), model, conf))
    try:
        detector: Detector | None = YoloDetector(model, conf)
    except FloodSimError:
        if not store.has_cache:
            raise
        log.warning("ultralytics not installed; using cached boxes from %s only", store.cache_path)
        detector = None
    return BoxProvider(core, store, detector)
```

- [ ] **Step 3: Run the tests**

Run: `.venv/Scripts/python -m pytest tests/test_boxes.py -v`
Expected: 11 passed.

- [ ] **Step 4: Commit**

```bash
git add floodsim/boxes.py tests/test_boxes.py
git commit -m "feat(python): YOLO box provider with per-frame cache and edits" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 7: Device and dense runners

**Files:**
- Create: `floodsim/modes.py`, `tests/test_modes.py`

**Interfaces:**
- Consumes: `Core`, `Reference`, `Device`, `FrameSource`, anything with `boxes_for(idx, t, load_rgb) -> list[Box]`, `write_image`.
- Produces: `RunParams(mode="device", period=60.0, retry_gap=1.0, every=1)`; `RunOutput(rows, decode_failures, cancelled)`; `FrameFeeder(core, source, boxes)` with `frame(idx) -> (gray, boxes, idx) | None` and `decode_failures`; `sample_device_at(device, ref, feeder, t, retry_gap) -> DeviceResult`; `device_row(sample, t, r) -> dict`; `dense_row(idx, t, s) -> dict`; `run_device(core, source, boxes, ref, params, *, uploads=None, progress=None, cancel=None) -> RunOutput`; `run_dense(core, source, boxes, ref, params, *, progress=None, cancel=None) -> RunOutput`; `UploadWriter(out_dir, downscale)` with `write(sample, rgb) -> Path`.
- Row dicts hold typed values (bool/int/float/str) keyed by `DEVICE_FIELDS` / `DENSE_FIELDS`. Device `t_sec` is the sample time; dense `t_sec` is the frame time.

- [ ] **Step 1: Write the failing tests `tests/test_modes.py`**

```python
import numpy as np
import pytest

from floodsim.modes import RunParams, UploadWriter, run_dense, run_device
from floodsim.results import DENSE_FIELDS, DEVICE_FIELDS
from floodsim.source import read_image_bgr
from tests.synth import CAR, MemorySource, dry_gray, flooded_gray, to_rgb

DRY, FLOOD = to_rgb(dry_gray()), to_rgb(flooded_gray())


class StaticBoxes:
    def __init__(self, by_idx):
        self.by_idx = by_idx

    def boxes_for(self, idx, t, load_rgb):
        return self.by_idx.get(idx, [])


def _golden_source():
    return MemorySource([(0, DRY), (1, DRY), (60, FLOOD), (61, FLOOD), (120, DRY), (121, DRY)])


@pytest.fixture
def ref(core):
    r = core.new_reference()
    assert r.build(dry_gray())
    return r


def test_device_matches_golden_scenario(core, ref):
    out = run_device(core, _golden_source(), StaticBoxes({4: [CAR]}), ref, RunParams(period=60.0))
    assert [r["outcome"] for r in out.rows] == ["clear", "upload", "clear"]
    assert [r["frames_taken"] for r in out.rows] == [1, 1, 2]
    assert [r["frame_idx"] for r in out.rows] == [0, 2, 5]
    assert [r["t_sec"] for r in out.rows] == [0.0, 60.0, 120.0]
    assert list(out.rows[0]) == DEVICE_FIELDS


def test_device_writes_what_would_be_uploaded(core, ref, tmp_path):
    uploads = UploadWriter(tmp_path, core.upload_downscale())
    run_device(core, _golden_source(), StaticBoxes({}), ref, RunParams(), uploads=uploads)
    files = sorted((tmp_path / "uploads").iterdir())
    assert [f.name for f in files] == ["00001_000000.jpg"]
    assert read_image_bgr(files[0]).shape == (540, 960, 3)


def test_device_heartbeat_on_60th_sample(core, ref):
    src = MemorySource([(t, DRY) for t in range(60)])
    out = run_device(core, src, StaticBoxes({}), ref, RunParams(period=1.0))
    assert len(out.rows) == 60
    assert out.rows[58]["outcome"] == "clear" and out.rows[59]["outcome"] == "heartbeat"


def test_device_without_reference_skips(core):
    out = run_device(core, _golden_source(), StaticBoxes({}), None, RunParams())
    assert {r["outcome"] for r in out.rows} == {"skipped_no_ref"}


def test_decode_failure_falls_back_to_retry_frame(core, ref):
    src = MemorySource([(0, None), (1, DRY)])
    out = run_device(core, src, StaticBoxes({}), ref, RunParams())
    assert out.rows[0]["outcome"] == "clear" and out.rows[0]["frames_taken"] == 2
    assert out.decode_failures == 1


def test_dense_every_second_frame(core, ref):
    out = run_dense(core, _golden_source(), StaticBoxes({4: [CAR]}), ref, RunParams(mode="dense", every=2))
    assert [r["frame_idx"] for r in out.rows] == [0, 2, 4]
    assert [r["submerged"] for r in out.rows] == [False, True, False]
    assert out.rows[2]["veto_usable"] is False
    assert list(out.rows[0]) == DENSE_FIELDS


def test_cancel_stops_run(core, ref):
    calls = []
    out = run_dense(core, _golden_source(), StaticBoxes({}), ref, RunParams(mode="dense"),
                    progress=lambda done, total: calls.append(done), cancel=lambda: len(calls) >= 2)
    assert out.cancelled and len(out.rows) == 2
```

Run: `.venv/Scripts/python -m pytest tests/test_modes.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'floodsim.modes'`.

- [ ] **Step 2: Write `floodsim/modes.py`**

```python
"""Device-faithful and dense runners over a frame source."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import cv2
import numpy as np

from .results import DeviceResult, ScoreResult
from .source import write_image

Progress = Callable[[int, int], None]
Cancel = Callable[[], bool]


@dataclass
class RunParams:
    mode: str = "device"
    period: float = 60.0
    retry_gap: float = 1.0
    every: int = 1


@dataclass
class RunOutput:
    rows: list[dict] = field(default_factory=list)
    decode_failures: int = 0
    cancelled: bool = False


def device_row(sample: int, t: float, r: DeviceResult) -> dict:
    return {"sample": sample, "t_sec": t, "frame_idx": r.frame_idx, "outcome": r.outcome,
            "frames_taken": r.frames_taken, "has_reading": r.has_reading,
            "curb_correlation": r.curb_correlation, "severity": r.severity,
            "shift_dx": r.shift_dx, "shift_dy": r.shift_dy, "veto_usable": r.veto_usable,
            "veto_coverage": r.veto_coverage, "veto_worst_class": r.veto_worst_class,
            "since_image": r.since_image}


def dense_row(idx: int, t: float, s: ScoreResult) -> dict:
    return {"frame_idx": idx, "t_sec": t, "veto_usable": s.veto_usable,
            "veto_coverage": s.veto_coverage, "veto_worst_class": s.veto_worst_class,
            "curb_correlation": s.curb_correlation, "submerged": s.submerged,
            "severity": s.severity, "shift_dx": s.shift_dx, "shift_dy": s.shift_dy}


class FrameFeeder:
    """Gray frame + boxes for a frame index, counting failed decodes."""

    def __init__(self, core, source, boxes):
        self.core = core
        self.source = source
        self.boxes = boxes
        self.decode_failures = 0

    def frame(self, idx: int):
        rgb = self.source.read_rgb(idx)
        if rgb is None:
            self.decode_failures += 1
            return None
        boxes = self.boxes.boxes_for(idx, self.source.timestamp(idx), lambda: rgb)
        return self.core.rgb_to_gray(rgb), boxes, idx


def sample_device_at(device, ref, feeder: FrameFeeder, t: float, retry_gap: float) -> DeviceResult:
    """One firmware sample: attempt 0 is the frame at t, the retry the frame at t + retry_gap."""
    times = (t, t + retry_gap)

    def provider(attempt: int):
        idx = feeder.source.index_at(times[min(attempt, 1)])
        return None if idx is None else feeder.frame(idx)

    return device.sample(ref, provider)


class UploadWriter:
    """What the camera would send: the frame decimated by FLOOD_ROI_DOWNSCALE with a
    6-hex sequence label (flood_roi_composite + flood_report_upload, flood_task.c:221).
    Encoded by OpenCV and labelled with an OpenCV font, so not byte-identical to the P4."""

    def __init__(self, out_dir: Path, downscale: int):
        self.dir = Path(out_dir) / "uploads"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.downscale = downscale
        self._seq = 0

    def write(self, sample: int, rgb: np.ndarray) -> Path:
        label = f"{self._seq & 0xFFFFFF:06X}"
        self._seq += 1
        canvas = np.ascontiguousarray(rgb[::self.downscale, ::self.downscale, ::-1])
        cv2.putText(canvas, label, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        path = self.dir / f"{sample:05d}_{label}.jpg"
        write_image(path, canvas)
        return path


def run_device(core, source, boxes, ref, params: RunParams, *, uploads: UploadWriter | None = None,
               progress: Progress | None = None, cancel: Cancel | None = None) -> RunOutput:
    feeder = FrameFeeder(core, source, boxes)
    device = core.new_device()
    out = RunOutput()
    total = int((source.duration + 1e-9) // params.period) + 1
    for k in range(total):
        if cancel and cancel():
            out.cancelled = True
            break
        t = k * params.period
        r = sample_device_at(device, ref, feeder, t, params.retry_gap)
        out.rows.append(device_row(k, t, r))
        if uploads and r.outcome in ("upload", "heartbeat") and r.frame_idx >= 0:
            rgb = source.read_rgb(r.frame_idx)
            if rgb is not None:
                uploads.write(k, rgb)
        if progress:
            progress(k + 1, total)
    out.decode_failures = feeder.decode_failures
    return out


def run_dense(core, source, boxes, ref, params: RunParams, *,
              progress: Progress | None = None, cancel: Cancel | None = None) -> RunOutput:
    feeder = FrameFeeder(core, source, boxes)
    out = RunOutput()
    indices = range(0, source.n_frames, params.every)
    for n, idx in enumerate(indices):
        if cancel and cancel():
            out.cancelled = True
            break
        got = feeder.frame(idx)
        if got is not None:
            gray, frame_boxes, _ = got
            out.rows.append(dense_row(idx, source.timestamp(idx), core.score(ref, gray, frame_boxes)))
        if progress:
            progress(n + 1, len(indices))
    out.decode_failures = feeder.decode_failures
    return out
```

- [ ] **Step 3: Run the tests**

Run: `.venv/Scripts/python -m pytest tests/test_modes.py -v`
Expected: 7 passed.

- [ ] **Step 4: Commit**

```bash
git add floodsim/modes.py tests/test_modes.py
git commit -m "feat(python): device and dense runners with upload images" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 8: Labels, scoring, run folders and compare

**Files:**
- Create: `floodsim/labels.py`, `floodsim/report.py`, `tests/test_labels.py`, `tests/test_report.py`

**Interfaces:**
- Consumes: row dicts from Task 7, `DEVICE_FIELDS`/`DENSE_FIELDS`.
- Produces:
  - `labels`: `VALID_LABELS`; `Labels.load(path) -> Labels` with `warnings: list[str]` and `label_for(t, name) -> str | None`; `score_rows(rows, mode) -> dict` with keys `tp, fp, tn, fn, abstained, indeterminate, unlabelled, skipped, abstention_rate, flooded_max_corr, dry_wet_min_corr, false_alarm_upper_95`; `format_score(score) -> list[str]`.
  - `report`: `format_value(key, v) -> str`; `write_samples(path, rows, fields)`; `read_samples(path) -> list[dict[str, str]]` (run folder, `samples.csv`, or `.jsonl`); `compare_runs(a, b, tol=1e-6) -> list[str]`; `prepare_out_dir(path) -> Path`; `summarize(rows, mode, decode_failures, score) -> str`; `write_run(out, run_info, rows, fields, summary)`.

- [ ] **Step 1: Write the failing tests `tests/test_labels.py`**

```python
import pytest

from floodsim.errors import FloodSimError
from floodsim.labels import Labels, score_rows


def _write(tmp_path, text, name="labels.csv", encoding="utf-8"):
    p = tmp_path / name
    p.write_bytes(text.encode(encoding))
    return p


def test_interval_labels_and_lookup(tmp_path):
    labels = Labels.load(_write(tmp_path, "start_sec,end_sec,label\n0,10,DRY\n10,20,flooded\n"))
    assert labels.label_for(0.0, None) == "DRY"
    assert labels.label_for(10.0, None) == "FLOODED"
    assert labels.label_for(20.0, None) is None
    assert labels.warnings == []


def test_labels_with_bom_and_crlf(tmp_path):
    labels = Labels.load(_write(tmp_path, "start_sec,end_sec,label\r\n0,5,WET\r\n", encoding="utf-8-sig"))
    assert labels.label_for(1.0, None) == "WET"


def test_file_labels(tmp_path):
    labels = Labels.load(_write(tmp_path, "file,label\na.png,DRY\nb.png,INDETERMINATE\n"))
    assert labels.label_for(0.0, "b.png") == "INDETERMINATE"
    assert labels.label_for(0.0, "c.png") is None


def test_gap_and_overlap_warnings(tmp_path):
    labels = Labels.load(_write(tmp_path, "start_sec,end_sec,label\n0,5,DRY\n4,8,WET\n10,12,DRY\n"))
    assert any("overlap" in w for w in labels.warnings)
    assert any("gap" in w for w in labels.warnings)


def test_unknown_label_rejected(tmp_path):
    with pytest.raises(FloodSimError, match="unknown label"):
        Labels.load(_write(tmp_path, "start_sec,end_sec,label\n0,5,DAMP\n"))


def test_wrong_columns_rejected(tmp_path):
    with pytest.raises(FloodSimError, match="expected columns"):
        Labels.load(_write(tmp_path, "a,b\n1,2\n"))


def _dev(outcome, corr, label, reading=True):
    return {"outcome": outcome, "curb_correlation": corr, "has_reading": reading, "label": label}


def test_score_device_rows_blocked_is_abstention():
    rows = [_dev("upload", 0.1, "FLOODED"), _dev("clear", 0.9, "DRY"), _dev("blocked", 0.0, "DRY", False),
            _dev("upload", 0.3, "WET"), _dev("clear", 0.8, "INDETERMINATE"), _dev("clear", 0.8, None),
            _dev("skipped_no_ref", 0.0, "DRY", False), _dev("heartbeat", 0.7, "FLOODED")]
    s = score_rows(rows, "device")
    assert (s["tp"], s["fp"], s["tn"], s["fn"]) == (1, 1, 1, 1)
    assert s["abstained"] == 1 and s["indeterminate"] == 1 and s["unlabelled"] == 1 and s["skipped"] == 1
    assert s["abstention_rate"] == pytest.approx(1 / 5)
    assert s["flooded_max_corr"] == pytest.approx(0.7)
    assert s["dry_wet_min_corr"] == pytest.approx(0.3)
    assert s["false_alarm_upper_95"] is None


def test_rule_of_three_when_no_false_alarms():
    rows = [_dev("clear", 0.9, "DRY")] * 30 + [_dev("upload", 0.1, "FLOODED")]
    s = score_rows(rows, "device")
    assert s["fp"] == 0 and s["false_alarm_upper_95"] == pytest.approx(0.1)


def test_dense_veto_is_abstention():
    rows = [{"veto_usable": False, "submerged": True, "curb_correlation": 0.1, "label": "DRY"},
            {"veto_usable": True, "submerged": True, "curb_correlation": 0.1, "label": "FLOODED"}]
    s = score_rows(rows, "dense")
    assert s["abstained"] == 1 and s["tp"] == 1 and s["fp"] == 0
```

- [ ] **Step 2: Write the failing tests `tests/test_report.py`**

```python
import json

import pytest

from floodsim.errors import FloodSimError
from floodsim.report import (compare_runs, format_value, prepare_out_dir, read_samples,
                             write_run, write_samples)
from floodsim.results import DENSE_FIELDS

ROW = {"frame_idx": 0, "t_sec": 0.0, "veto_usable": True, "veto_coverage": 0.0,
       "veto_worst_class": "other", "curb_correlation": 0.99999994, "submerged": False,
       "severity": "none", "shift_dx": 0, "shift_dy": 0}


def test_format_values():
    assert format_value("t_sec", 60.0) == "60.000"
    assert format_value("curb_correlation", 0.99999994) == "1.000000"
    assert format_value("veto_usable", True) == "true"
    assert format_value("label", None) == ""
    assert format_value("frame_idx", 3) == "3"


def test_write_and_read_run(tmp_path):
    out = prepare_out_dir(tmp_path / "run")
    write_run(out, {"mode": "dense"}, [ROW], DENSE_FIELDS, "summary\n")
    assert json.loads((out / "run.json").read_text())["mode"] == "dense"
    assert (out / "summary.txt").read_text() == "summary\n"
    rows = read_samples(out)
    assert rows[0]["curb_correlation"] == "1.000000" and rows[0]["submerged"] == "false"


def test_prepare_out_dir_refuses_non_empty(tmp_path):
    (tmp_path / "run").mkdir()
    (tmp_path / "run" / "samples.csv").write_text("x")
    with pytest.raises(FloodSimError, match="not empty"):
        prepare_out_dir(tmp_path / "run")


def test_compare_identical_and_different(tmp_path):
    a, b = tmp_path / "a.csv", tmp_path / "b.csv"
    write_samples(a, [ROW], DENSE_FIELDS)
    write_samples(b, [ROW], DENSE_FIELDS)
    assert compare_runs(a, b) == []
    write_samples(b, [dict(ROW, curb_correlation=0.5, submerged=True)], DENSE_FIELDS)
    diffs = compare_runs(a, b)
    assert any("curb_correlation" in d for d in diffs) and any("submerged" in d for d in diffs)
    write_samples(b, [ROW, dict(ROW, frame_idx=1)], DENSE_FIELDS)
    assert any("only in B" in d for d in compare_runs(a, b))


def test_compare_reads_jsonl(tmp_path):
    a = tmp_path / "a.csv"
    write_samples(a, [ROW], DENSE_FIELDS)
    j = tmp_path / "exe.jsonl"
    j.write_text('{"mode":"dense","frame_idx":0,"t_sec":0.000,"veto_usable":true,"veto_coverage":0.000000,'
                 '"veto_worst_class":"other","curb_correlation":1.000000,"submerged":false,'
                 '"severity":"none","shift_dx":0,"shift_dy":0}\n')
    assert compare_runs(a, j) == []
```

Run: `.venv/Scripts/python -m pytest tests/test_labels.py tests/test_report.py -v`
Expected: FAIL with `ModuleNotFoundError` for `floodsim.labels` / `floodsim.report`.

- [ ] **Step 3: Write `floodsim/labels.py`**

```python
"""Ground-truth labels and scoring. BLOCKED is an abstention, never a negative."""
from __future__ import annotations

import csv
import io
from pathlib import Path

from .errors import FloodSimError

VALID_LABELS = {"DRY", "WET", "FLOODED", "INDETERMINATE"}


def _label(value: str, path: Path) -> str:
    v = value.strip().upper()
    if v not in VALID_LABELS:
        raise FloodSimError(f"{path}: unknown label {value!r} (expected {sorted(VALID_LABELS)})")
    return v


class Labels:
    def __init__(self, intervals=None, files=None, warnings=None):
        self.intervals: list[tuple[float, float, str]] = intervals or []
        self.files: dict[str, str] = files or {}
        self.warnings: list[str] = warnings or []

    @classmethod
    def load(cls, path: Path) -> Labels:
        path = Path(path)
        text = path.read_text(encoding="utf-8-sig")
        reader = csv.DictReader(io.StringIO(text))
        fields = {f.strip() for f in reader.fieldnames or []}
        rows = [{(k or "").strip(): (v or "").strip() for k, v in r.items()} for r in reader]
        if {"start_sec", "end_sec", "label"} <= fields:
            try:
                intervals = sorted((float(r["start_sec"]), float(r["end_sec"]), _label(r["label"], path))
                                   for r in rows)
            except ValueError as e:
                raise FloodSimError(f"{path}: {e}") from e
            return cls(intervals=intervals, warnings=_interval_warnings(intervals))
        if {"file", "label"} <= fields:
            return cls(files={r["file"]: _label(r["label"], path) for r in rows})
        raise FloodSimError(f"{path}: expected columns start_sec,end_sec,label or file,label")

    def label_for(self, t: float, name: str | None) -> str | None:
        if self.files:
            return self.files.get(name) if name else None
        for start, end, label in self.intervals:
            if start <= t < end:
                return label
        return None


def _interval_warnings(intervals) -> list[str]:
    gaps = overlaps = 0
    for (_, a_end, _), (b_start, _, _) in zip(intervals, intervals[1:]):
        if b_start < a_end - 1e-9:
            overlaps += 1
        elif b_start > a_end + 1e-9:
            gaps += 1
    out = []
    if overlaps:
        out.append(f"labels: {overlaps} overlap(s) between intervals; the earliest interval wins")
    if gaps:
        out.append(f"labels: {gaps} gap(s) between intervals; samples in gaps are unlabelled")
    return out


def score_rows(rows: list[dict], mode: str) -> dict:
    c = dict(tp=0, fp=0, tn=0, fn=0, abstained=0, indeterminate=0, unlabelled=0, skipped=0)
    pos_corr: list[float] = []
    neg_corr: list[float] = []
    for r in rows:
        if mode == "device":
            if r["outcome"] == "skipped_no_ref":
                c["skipped"] += 1
                continue
            abstain = r["outcome"] == "blocked"
            predicted = r["outcome"] == "upload"
            reading = r["has_reading"]
        else:
            abstain = not r["veto_usable"]
            predicted = r["submerged"]
            reading = True
        label = r.get("label")
        if label is None:
            c["unlabelled"] += 1
            continue
        if label == "INDETERMINATE":
            c["indeterminate"] += 1
            continue
        if abstain:
            c["abstained"] += 1
            continue
        positive = label == "FLOODED"
        if reading:
            (pos_corr if positive else neg_corr).append(r["curb_correlation"])
        c[("tp" if predicted else "fn") if positive else ("fp" if predicted else "tn")] += 1

    labelled = c["tp"] + c["fp"] + c["tn"] + c["fn"] + c["abstained"]
    negatives = c["fp"] + c["tn"]
    c["abstention_rate"] = c["abstained"] / labelled if labelled else None
    c["flooded_max_corr"] = max(pos_corr) if pos_corr else None
    c["dry_wet_min_corr"] = min(neg_corr) if neg_corr else None
    c["false_alarm_upper_95"] = 3 / negatives if c["fp"] == 0 and negatives else None
    return c


def format_score(s: dict) -> list[str]:
    lines = [f"labelled: TP={s['tp']} FP={s['fp']} TN={s['tn']} FN={s['fn']}",
             f"abstained (BLOCKED, not counted as negatives): {s['abstained']}"
             + (f" ({s['abstention_rate']:.1%})" if s["abstention_rate"] is not None else ""),
             f"excluded: indeterminate={s['indeterminate']} unlabelled={s['unlabelled']} "
             f"skipped_no_ref={s['skipped']}"]
    hi, lo = s["flooded_max_corr"], s["dry_wet_min_corr"]
    if hi is not None or lo is not None:
        line = f"margin: flooded max corr={hi if hi is None else f'{hi:.3f}'}, " \
               f"dry/wet min corr={lo if lo is None else f'{lo:.3f}'}"
        if hi is not None and lo is not None:
            line += f", gap={lo - hi:+.3f}"
        lines.append(line)
    if s["false_alarm_upper_95"] is not None:
        n = s["fp"] + s["tn"]
        lines.append(f"0 false alarms in {n} negatives: 95% upper bound on the false-alarm rate "
                     f"is 3/{n} = {s['false_alarm_upper_95']:.1%} (rule of three), not zero")
    return lines
```

- [ ] **Step 4: Write `floodsim/report.py`**

```python
"""Run folders (run.json, samples.csv, summary.txt) and run comparison."""
from __future__ import annotations

import csv
import json
from collections import Counter
from pathlib import Path

from .errors import FloodSimError
from .labels import format_score

_FLOAT6 = {"curb_correlation", "veto_coverage"}
_CATEGORICAL = ("outcome", "severity", "submerged", "veto_usable")


def format_value(key: str, v) -> str:
    if v is None:
        return ""
    if isinstance(v, bool):
        return "true" if v else "false"
    if key == "t_sec":
        return f"{v:.3f}"
    if key in _FLOAT6:
        return f"{v:.6f}"
    return str(v)


def write_samples(path: Path, rows: list[dict], fields: list[str]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(fields)
        for r in rows:
            w.writerow([format_value(k, r.get(k)) for k in fields])


def read_samples(path: Path) -> list[dict[str, str]]:
    path = Path(path)
    if path.is_dir():
        path = path / "samples.csv"
    if path.suffix == ".jsonl":
        rows = []
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                d = json.loads(line)
                d.pop("mode", None)
                rows.append({k: ("true" if v is True else "false" if v is False else str(v))
                             for k, v in d.items()})
        return rows
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def compare_runs(a: Path, b: Path, tol: float = 1e-6) -> list[str]:
    ra, rb = read_samples(a), read_samples(b)
    key = "sample" if (ra or rb) and "sample" in (ra or rb)[0] else "frame_idx"
    ma, mb = {r[key]: r for r in ra}, {r[key]: r for r in rb}
    diffs = []
    for k in sorted(set(ma) | set(mb), key=int):
        if k not in mb:
            diffs.append(f"{key} {k}: only in A")
            continue
        if k not in ma:
            diffs.append(f"{key} {k}: only in B")
            continue
        x, y = ma[k], mb[k]
        for f in _CATEGORICAL:
            if f in x and f in y and x[f] != y[f]:
                diffs.append(f"{key} {k}: {f} {x[f]} -> {y[f]}")
        cx, cy = float(x["curb_correlation"]), float(y["curb_correlation"])
        if abs(cx - cy) > tol:
            diffs.append(f"{key} {k}: curb_correlation {cx:.6f} -> {cy:.6f}")
    return diffs


def prepare_out_dir(out: Path) -> Path:
    out = Path(out)
    if out.exists() and any(out.iterdir()):
        raise FloodSimError(f"{out} already exists and is not empty; choose another --out")
    out.mkdir(parents=True, exist_ok=True)
    return out


def summarize(rows: list[dict], mode: str, decode_failures: int, score: dict | None) -> str:
    lines = [f"mode: {mode}", f"rows: {len(rows)}", f"failed frame reads: {decode_failures}"]
    if mode == "device":
        counts = Counter(r["outcome"] for r in rows)
        for name in ("upload", "clear", "blocked", "heartbeat", "skipped_no_ref"):
            lines.append(f"{name}: {counts.get(name, 0)}")
    else:
        lines.append(f"submerged: {sum(r['submerged'] for r in rows)}")
        lines.append(f"vetoed (obstructed): {sum(not r['veto_usable'] for r in rows)}")
    if score:
        lines += format_score(score)
    return "\n".join(lines) + "\n"


def write_run(out: Path, run_info: dict, rows: list[dict], fields: list[str], summary: str) -> None:
    out = Path(out)
    (out / "run.json").write_text(json.dumps(run_info, indent=2, default=str) + "\n", encoding="utf-8")
    write_samples(out / "samples.csv", rows, fields)
    (out / "summary.txt").write_text(summary, encoding="utf-8")
```

- [ ] **Step 5: Run the tests**

Run: `.venv/Scripts/python -m pytest tests/test_labels.py tests/test_report.py -v`
Expected: 15 passed.

- [ ] **Step 6: Commit**

```bash
git add floodsim/labels.py floodsim/report.py tests/test_labels.py tests/test_report.py
git commit -m "feat(python): label scoring, run folders and compare" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 9: `floodsim` CLI and DLL-vs-exe parity

**Files:**
- Create: `floodsim/cli.py`, `tests/test_cli.py`

**Interfaces:**
- Consumes: everything above.
- Produces: `floodsim.cli.main(argv=None) -> int` (0 ok, 1 compare differences, 2 error); subcommands `run`, `detect`, `compare`, `export` exactly as the spec's CLI table; `build_reference(core, source, reference, reference_at, allow_stretch) -> (Reference | None, dict)` (reused by Part 2); `run_info(...)` assembly lives in `cmd_run`.

- [ ] **Step 1: Write the failing tests `tests/test_cli.py`**

```python
import subprocess

import pytest

from floodsim import core as core_mod
from floodsim.boxes import DEFAULT_CONF, DEFAULT_MODEL, BoxStore, CachedBox, cache_meta
from floodsim.cli import main
from floodsim.report import compare_runs, read_samples
from floodsim.source import open_source, write_image
from tests.synth import dry_gray, flooded_gray, to_rgb, write_frame_folder

CAR = CachedBox(0.0, 0.0, 1919.0, 1079.0, 0.9, "car", 2)


@pytest.fixture
def scenario(tmp_path, core):
    """The golden scenario as a PNG folder, a reference image and a complete box cache."""
    frames = [(0, dry_gray()), (1000, dry_gray()), (60000, flooded_gray()),
              (61000, flooded_gray()), (120000, dry_gray()), (121000, dry_gray())]
    folder = write_frame_folder(tmp_path / "clip", frames)
    ref = tmp_path / "dry.png"
    write_image(ref, to_rgb(dry_gray()))
    src = open_source(folder)
    store = BoxStore(folder, cache_meta(src.fingerprint(), DEFAULT_MODEL, DEFAULT_CONF))
    for i in range(src.n_frames):
        store.put_detected(i, src.timestamp(i), [CAR] if i == 4 else [])
    return folder, ref, tmp_path


def test_run_device_writes_run_folder(scenario):
    folder, ref, tmp = scenario
    assert main(["run", str(folder), "--reference", str(ref), "--out", str(tmp / "dev")]) == 0
    rows = read_samples(tmp / "dev")
    assert [r["outcome"] for r in rows] == ["clear", "upload", "clear"]
    assert (tmp / "dev" / "run.json").exists() and (tmp / "dev" / "summary.txt").exists()
    assert len(list((tmp / "dev" / "uploads").iterdir())) == 1


def test_run_dense_with_labels(scenario):
    folder, ref, tmp = scenario
    labels = tmp / "labels.csv"
    labels.write_text("start_sec,end_sec,label\n0,59,DRY\n59,100,FLOODED\n100,200,DRY\n")
    assert main(["run", str(folder), "--mode", "dense", "--reference", str(ref),
                 "--labels", str(labels), "--out", str(tmp / "dense")]) == 0
    rows = read_samples(tmp / "dense")
    assert [r["label"] for r in rows] == ["DRY", "DRY", "FLOODED", "FLOODED", "DRY", "DRY"]
    summary = (tmp / "dense" / "summary.txt").read_text()
    assert "TP=2 FP=0 TN=3 FN=0" in summary and "abstained" in summary


@pytest.mark.parametrize("mode", ["device", "dense"])
def test_export_then_exe_matches_python(scenario, mode):
    folder, ref, tmp = scenario
    assert main(["run", str(folder), "--mode", mode, "--reference", str(ref), "--out", str(tmp / "py")]) == 0
    assert main(["export", str(folder), "--reference", str(ref), "--out", str(tmp / "exp")]) == 0
    exe = core_mod.BUILD_DIR / core_mod.EXE_NAME
    p = subprocess.run([str(exe), "--reference", str(tmp / "exp" / "reference.pgm"),
                        "--frames", str(tmp / "exp" / "frames"), "--boxes", str(tmp / "exp" / "boxes.csv"),
                        "--mode", mode], capture_output=True, text=True)
    assert p.returncode == 0, p.stderr
    (tmp / "exe.jsonl").write_text(p.stdout)
    assert compare_runs(tmp / "py", tmp / "exe.jsonl") == []


def test_compare_exit_codes(scenario):
    folder, ref, tmp = scenario
    main(["run", str(folder), "--mode", "dense", "--reference", str(ref), "--out", str(tmp / "a")])
    main(["run", str(folder), "--mode", "dense", "--reference", str(ref), "--out", str(tmp / "b")])
    assert main(["compare", str(tmp / "a"), str(tmp / "b")]) == 0
    main(["run", str(folder), "--mode", "dense", "--reference-at", "60", "--out", str(tmp / "c")])
    assert main(["compare", str(tmp / "a"), str(tmp / "c")]) == 1


def test_run_rejects_both_reference_options(scenario, capsys):
    folder, ref, tmp = scenario
    assert main(["run", str(folder), "--reference", str(ref), "--reference-at", "0",
                 "--out", str(tmp / "x")]) == 2
    assert "not both" in capsys.readouterr().err


def test_dense_without_reference_errors(scenario, capsys):
    folder, _, tmp = scenario
    assert main(["run", str(folder), "--mode", "dense", "--out", str(tmp / "x")]) == 2
    assert "--reference" in capsys.readouterr().err


def test_no_detector_ignores_cache(scenario):
    folder, ref, tmp = scenario
    assert main(["run", str(folder), "--reference", str(ref), "--no-detector", "--out", str(tmp / "nd")]) == 0
    rows = read_samples(tmp / "nd")
    assert rows[2]["frames_taken"] == "1"   # no car box, so no retry at 120 s
```

Run: `.venv/Scripts/python -m pytest tests/test_cli.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'floodsim.cli'`.

- [ ] **Step 2: Write `floodsim/cli.py`**

```python
"""floodsim command line: run, detect, compare, export."""
from __future__ import annotations

import argparse
import datetime as dt
import logging
import sys
from dataclasses import asdict
from pathlib import Path

from . import __version__
from .boxes import DEFAULT_CONF, DEFAULT_MODEL, make_box_provider, write_boxes_csv
from .core import load_core
from .errors import FloodSimError
from .geometry import Geometry, load_geometry
from .labels import Labels, score_rows
from .modes import RunParams, UploadWriter, run_dense, run_device
from .report import compare_runs, prepare_out_dir, summarize, write_run
from .results import DENSE_FIELDS, DEVICE_FIELDS
from .source import Normalizer, open_source, read_image_bgr, write_image

log = logging.getLogger("floodsim")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="floodsim",
                                description="Run the MS500-P4 flood-detection firmware code on image/video files.")
    sub = p.add_subparsers(dest="command", required=True)

    def add_input(sp):
        sp.add_argument("input", type=Path, help="image, folder of images, or video")
        sp.add_argument("--fps", type=float, help="frame rate for image folders without _t<ms> names")
        sp.add_argument("--allow-stretch", action="store_true", help="resize non-16:9 input anyway")

    def add_boxes(sp):
        sp.add_argument("--no-detector", action="store_true", help="no Stage-1 boxes (veto always passes)")
        sp.add_argument("--model", default=DEFAULT_MODEL)
        sp.add_argument("--conf", type=float, default=DEFAULT_CONF)

    run = sub.add_parser("run", help="headless device or dense run")
    add_input(run)
    add_boxes(run)
    run.add_argument("--mode", choices=["device", "dense"], default="device")
    run.add_argument("--reference", type=Path, help="dry reference image")
    run.add_argument("--reference-at", type=float, help="use the input's frame at this time (s) as reference")
    run.add_argument("--geometry", type=Path, help="curb/lane geometry JSON")
    run.add_argument("--labels", type=Path, help="labels CSV to score against")
    run.add_argument("--period", type=float, help="device sample period in s (default: firmware's)")
    run.add_argument("--retry-gap", type=float, default=1.0)
    run.add_argument("--every", type=int, default=1, help="dense: score every Nth frame")
    run.add_argument("--no-rebuild", action="store_true", help="use the existing DLL even if stale")
    run.add_argument("--out", type=Path)

    detect = sub.add_parser("detect", help="fill the box cache ahead of time")
    add_input(detect)
    detect.add_argument("--model", default=DEFAULT_MODEL)
    detect.add_argument("--conf", type=float, default=DEFAULT_CONF)
    detect.add_argument("--every", type=int, default=1)

    cmp_ = sub.add_parser("compare", help="diff two runs (folders, samples.csv or exe .jsonl)")
    cmp_.add_argument("a", type=Path)
    cmp_.add_argument("b", type=Path)

    exp = sub.add_parser("export", help="gray PGM frames + boxes.csv for flood_sim.exe")
    add_input(exp)
    add_boxes(exp)
    exp.add_argument("--out", type=Path, required=True)
    exp.add_argument("--every", type=int, default=1)
    exp.add_argument("--reference", type=Path, help="also write reference.pgm from this image")
    exp.add_argument("--no-rebuild", action="store_true")
    return p


def build_reference(core, source, reference: Path | None, reference_at: float | None,
                    allow_stretch: bool):
    if reference is not None and reference_at is not None:
        raise FloodSimError("use --reference or --reference-at, not both")
    if reference is not None:
        bgr = read_image_bgr(reference)
        if bgr is None:
            raise FloodSimError(f"cannot read reference image {reference}")
        rgb = Normalizer(allow_stretch)(bgr, str(reference))
        info = {"kind": "image", "path": str(reference)}
    elif reference_at is not None:
        idx = source.index_at(reference_at)
        rgb = source.read_rgb(idx) if idx is not None else None
        if rgb is None:
            raise FloodSimError(f"no readable frame at {reference_at} s for the reference")
        info = {"kind": "frame", "t_sec": reference_at, "frame_idx": idx}
    else:
        return None, {"kind": "none"}
    ref = core.new_reference()
    if not ref.build(core.rgb_to_gray(rgb)):
        raise FloodSimError("geometry leaves no usable sampling window")
    return ref, info


def _apply_geometry(core, path: Path | None) -> Geometry:
    g = load_geometry(path) if path else Geometry.from_list(core.default_geometry())
    err = core.validate_geometry(g)
    if err:
        raise FloodSimError(f"invalid geometry: {err}")
    core.set_geometry(g)
    return g


def cmd_run(args) -> int:
    core, build_info = load_core(rebuild=not args.no_rebuild)
    geometry = _apply_geometry(core, args.geometry)
    source = open_source(args.input, fps=args.fps, allow_stretch=args.allow_stretch)
    ref, ref_info = build_reference(core, source, args.reference, args.reference_at, args.allow_stretch)
    if args.mode == "dense" and ref is None:
        raise FloodSimError("dense mode needs --reference or --reference-at")
    params = RunParams(args.mode, args.period or core.sample_period_s(), args.retry_gap, args.every)
    if params.period <= 0 or params.retry_gap < 0 or params.every < 1:
        raise FloodSimError("--period must be > 0, --retry-gap >= 0, --every >= 1")
    labels = Labels.load(args.labels) if args.labels else None
    for w in labels.warnings if labels else []:
        log.warning(w)
    boxes = make_box_provider(core, source, no_detector=args.no_detector, model=args.model, conf=args.conf)
    out = prepare_out_dir(args.out or Path("out") /
                          f"{args.input.stem}-{args.mode}-{dt.datetime.now():%Y%m%d-%H%M%S}")

    if args.mode == "device":
        result = run_device(core, source, boxes, ref, params,
                            uploads=UploadWriter(out, core.upload_downscale()))
        fields = list(DEVICE_FIELDS)
    else:
        result = run_dense(core, source, boxes, ref, params)
        fields = list(DENSE_FIELDS)

    score = None
    if labels:
        for row in result.rows:
            name = source.frame_name(row["frame_idx"]) if row["frame_idx"] >= 0 else None
            row["label"] = labels.label_for(row["t_sec"], name)
        fields.append("label")
        score = score_rows(result.rows, args.mode)

    run_info = {
        "floodsim": __version__, "created": dt.datetime.now().isoformat(timespec="seconds"),
        "input": source.fingerprint(), "mode": args.mode, "params": asdict(params),
        "geometry": geometry.to_json_dict(), "reference": ref_info, "boxes": boxes.description,
        "labels": str(args.labels) if args.labels else None, **build_info,
    }
    write_run(out, run_info, result.rows, fields,
              summarize(result.rows, args.mode, result.decode_failures, score))
    print(f"wrote {out}")
    return 0


def cmd_detect(args) -> int:
    core, _ = load_core()
    source = open_source(args.input, fps=args.fps, allow_stretch=args.allow_stretch)
    boxes = make_box_provider(core, source, no_detector=False, model=args.model, conf=args.conf)
    indices = range(0, source.n_frames, args.every)
    for n, idx in enumerate(indices, 1):
        boxes.cached_for(idx, source.timestamp(idx), lambda i=idx: source.read_rgb(i))
        if n % 50 == 0 or n == len(indices):
            print(f"detected {n}/{len(indices)} frames")
    print(f"cache: {boxes.store.cache_path}")
    return 0


def cmd_compare(args) -> int:
    diffs = compare_runs(args.a, args.b)
    for d in diffs:
        print(d)
    print(f"{len(diffs)} difference(s)")
    return 1 if diffs else 0


def cmd_export(args) -> int:
    core, _ = load_core(rebuild=not args.no_rebuild)
    source = open_source(args.input, fps=args.fps, allow_stretch=args.allow_stretch)
    boxes = make_box_provider(core, source, no_detector=args.no_detector, model=args.model, conf=args.conf)
    out = prepare_out_dir(args.out)
    frames_dir = out / "frames"
    frames_dir.mkdir()
    entries = []
    for idx in range(0, source.n_frames, args.every):
        rgb = source.read_rgb(idx)
        if rgb is None:
            log.warning("frame %d failed to decode; not exported", idx)
            continue
        t = source.timestamp(idx)
        write_image(frames_dir / f"f{idx:06d}_t{round(t * 1000):010d}.pgm", core.rgb_to_gray(rgb))
        entries.append((idx, t, boxes.cached_for(idx, t, lambda: rgb)))
    write_boxes_csv(out / "boxes.csv", entries, {"export": str(args.input)})
    if args.reference:
        ref_bgr = read_image_bgr(args.reference)
        if ref_bgr is None:
            raise FloodSimError(f"cannot read reference image {args.reference}")
        rgb = Normalizer(args.allow_stretch)(ref_bgr, str(args.reference))
        write_image(out / "reference.pgm", core.rgb_to_gray(rgb))
    print(f"wrote {len(entries)} frames to {frames_dir}")
    return 0


COMMANDS = {"run": cmd_run, "detect": cmd_detect, "compare": cmd_compare, "export": cmd_export}


def main(argv=None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    args = build_parser().parse_args(argv)
    try:
        return COMMANDS[args.command](args)
    except FloodSimError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 3: Run the tests**

Run: `.venv/Scripts/python -m pytest tests/test_cli.py -v`
Expected: 8 passed. (If `ultralytics` is installed, `make_box_provider` creates a detector, but the complete cache means it's never called.)

- [ ] **Step 4: Run the whole suite**

Run: `ctest --test-dir build/native --output-on-failure && .venv/Scripts/python -m pytest -v`
Expected: all native and Python tests pass.

- [ ] **Step 5: Commit**

```bash
git add floodsim/cli.py tests/test_cli.py
git commit -m "feat(cli): run/detect/compare/export with DLL-vs-exe parity test" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 10: Dataset labels tool, reference cross-check, README

**Files:**
- Create: `tools/truth_to_labels.py`, `tests/test_truth_to_labels.py`, `tests/test_reference_crosscheck.py`, `README.md`

**Interfaces:**
- Consumes: `Core.correlation_at`, `Core.rgb_to_gray`, `read_image_bgr`, `Normalizer`.
- Produces: `tools/truth_to_labels.py --truth truth.csv --bursts bursts.json --out DIR [--videos DIR] [--fps F]`. It writes `<clip>.labels.csv` per clip: `c{clip:02d}.labels.csv` without `--videos`, `<video stem>.labels.csv` with it. Each burst becomes one interval `[start/fps, (start + 16 + 1)/fps)`, per `dataset/build_bursts.py:35-36` (5 frames, step 4).

- [ ] **Step 1: Write the failing test `tests/test_truth_to_labels.py`**

```python
import json
import subprocess
import sys
from pathlib import Path

TOOL = Path(__file__).resolve().parent.parent / "tools" / "truth_to_labels.py"


def test_converts_bursts_to_intervals(tmp_path):
    (tmp_path / "truth.csv").write_text(
        "burst_id,clip,label,extent_pct,n_usable,curb_corr,trusted\n"
        "c01_b0,1,DRY,0,5,0.99,0\nc01_b1,1,WET,0,5,0.9,0\nc13_b0,13,FLOODED,0,5,0.1,1\n")
    (tmp_path / "bursts.json").write_text(json.dumps([
        {"id": "c01_b0", "clip": 1, "start_frame": 6},
        {"id": "c01_b1", "clip": 1, "start_frame": 100},
        {"id": "c13_b0", "clip": 13, "start_frame": 40},
    ]))
    p = subprocess.run([sys.executable, str(TOOL), "--truth", str(tmp_path / "truth.csv"),
                        "--bursts", str(tmp_path / "bursts.json"), "--out", str(tmp_path / "labels"),
                        "--fps", "4"], capture_output=True, text=True)
    assert p.returncode == 0, p.stderr
    assert (tmp_path / "labels" / "c01.labels.csv").read_text().splitlines() == [
        "start_sec,end_sec,label", "1.500,5.750,DRY", "25.000,29.250,WET"]
    assert (tmp_path / "labels" / "c13.labels.csv").read_text().splitlines()[1] == "10.000,14.250,FLOODED"
```

Run: `.venv/Scripts/python -m pytest tests/test_truth_to_labels.py -v`
Expected: FAIL with the tool exiting non-zero ("can't open file ... truth_to_labels.py").

- [ ] **Step 2: Write `tools/truth_to_labels.py`**

```python
"""Convert MS500_Flooding_Detection's burst ground truth into floodsim labels CSVs.

truth.csv (burst_id, clip, label) + bursts.json (id, clip, start_frame) ->
one <clip>.labels.csv per clip with start_sec,end_sec,label. A burst spans
FRAMES_PER_BURST frames FRAME_STEP apart (dataset/build_bursts.py:35-36).
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

FRAMES_PER_BURST = 5
FRAME_STEP = 4


def clip_fps(videos: Path | None, clip: int, override: float | None) -> tuple[float, str]:
    if videos is None:
        if override is None:
            sys.exit("--fps is required without --videos")
        return override, f"c{clip:02d}"
    vids = sorted(videos.glob("*.avi"))
    if clip > len(vids):
        sys.exit(f"clip {clip}: only {len(vids)} videos in {videos}")
    path = vids[clip - 1]  # build_bursts.py numbers sorted Videos/*.avi from 1
    if override is not None:
        return override, path.stem
    import cv2
    fps = cv2.VideoCapture(str(path)).get(cv2.CAP_PROP_FPS)
    if not fps or fps <= 0:
        sys.exit(f"{path}: no frame rate; pass --fps")
    return fps, path.stem


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--truth", type=Path, required=True)
    ap.add_argument("--bursts", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--videos", type=Path)
    ap.add_argument("--fps", type=float)
    args = ap.parse_args()

    starts = {b["id"]: b["start_frame"] for b in json.loads(args.bursts.read_text(encoding="utf-8"))}
    by_clip: dict[int, list[tuple[int, str]]] = defaultdict(list)
    with open(args.truth, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            if row["burst_id"] not in starts:
                print(f"warning: {row['burst_id']} not in {args.bursts}", file=sys.stderr)
                continue
            by_clip[int(row["clip"])].append((starts[row["burst_id"]], row["label"].strip().upper()))

    args.out.mkdir(parents=True, exist_ok=True)
    span = (FRAMES_PER_BURST - 1) * FRAME_STEP + 1
    for clip, bursts in sorted(by_clip.items()):
        fps, name = clip_fps(args.videos, clip, args.fps)
        with open(args.out / f"{name}.labels.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f, lineterminator="\n")
            w.writerow(["start_sec", "end_sec", "label"])
            for start, label in sorted(bursts):
                w.writerow([f"{start / fps:.3f}", f"{(start + span) / fps:.3f}", label])
    print(f"wrote {len(by_clip)} labels file(s) to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

Run: `.venv/Scripts/python -m pytest tests/test_truth_to_labels.py -v`
Expected: 1 passed.

- [ ] **Step 3: Write the cross-check test `tests/test_reference_crosscheck.py`**

```python
"""Firmware C vs the Python reference (ms500/curbcheck.py) at a fixed shift.

Skipped unless D:/git/MS500_Flooding_Detection with its dataset frames is present.
Registration is not compared: the firmware uses a direct vertical search, the
Python reference phase correlation (documented in flood_curbcheck.h)."""
import sys
from pathlib import Path

import pytest

from floodsim.source import Normalizer, read_image_bgr

REF_REPO = Path("D:/git/MS500_Flooding_Detection")
DRY = REF_REPO / "dataset" / "dry_reference_full.jpg"
FRAMES = sorted((REF_REPO / "dataset" / "frames").glob("*.jp*g")) if REF_REPO.exists() else []

pytestmark = pytest.mark.skipif(not DRY.exists() or not FRAMES, reason="reference repo/dataset not present")


def _gray(core, path):
    return core.rgb_to_gray(Normalizer(False)(read_image_bgr(path), path.name))


@pytest.mark.parametrize("frame", FRAMES, ids=lambda p: p.name)
def test_correlation_matches_python_reference(core, frame):
    sys.path.insert(0, str(REF_REPO))
    from ms500.curbcheck import CurbCheck

    ref_gray, gray = _gray(core, DRY), _gray(core, frame)
    ref = core.new_reference()
    assert ref.build(ref_gray)
    python = CurbCheck(ref_gray, register=False).check(gray).correlation
    assert core.correlation_at(ref, gray, 0, 0) == pytest.approx(python, abs=1e-5)
```

Run: `.venv/Scripts/python -m pytest tests/test_reference_crosscheck.py -v`
Expected: 2 passed (Dry-day, Dry-night). If it fails, report the two numbers; don't loosen the tolerance. A mismatch means the firmware port and the Python reference disagree, and that is a finding for the user.

- [ ] **Step 4: Write `README.md`**

````markdown
# ms500-p4-simulator

Runs the MS500-P4 firmware's flood-detection C code — unmodified, compiled straight from
`D:\git\NETWORK_P4_FLOOD_DETECTION` — on image and video files instead of the camera.

Design: `docs/superpowers/specs/2026-09-24-flood-simulator-design.md`.

## Setup (Windows, Git Bash)

Needs MinGW gcc, CMake ≥ 3.20, Ninja and Python 3.11 on PATH.

```bash
py -3.11 -m venv .venv
.venv/Scripts/python -m pip install -e ".[dev]"        # add ,yolo for Stage-1 detection (~2 GB)
```

The native build runs automatically the first time `floodsim` loads the core, and again whenever a
firmware or shim source is newer than `build/native/flood_core.dll`. To point at another firmware
checkout: `export FLOOD_FIRMWARE_DIR=/d/path/to/firmware`.

## Use

```bash
# device-faithful decisions (60 s samples, retry, heartbeat) + images the camera would upload
.venv/Scripts/floodsim run clip.avi --reference dry.jpg
# every frame: correlation / veto curve, scored against labels
.venv/Scripts/floodsim run clip.avi --mode dense --reference dry.jpg --labels clip.labels.csv
# regression: did a firmware edit change anything?
.venv/Scripts/floodsim compare out/before out/after
# pure C, no Python: export frames, then run the exe
.venv/Scripts/floodsim export clip.avi --reference dry.jpg --out out/exp
build/native/flood_sim.exe --reference out/exp/reference.pgm --frames out/exp/frames \
    --boxes out/exp/boxes.csv --mode dense
```

Without `ultralytics`, pass `--no-detector` (no obstruction veto) or have a complete
`<input>.boxes.csv` cache from an earlier `floodsim detect`.

Labels from the reference dataset:
`python tools/truth_to_labels.py --truth ../MS500_Flooding_Detection/dataset/truth.csv --bursts ../MS500_Flooding_Detection/dataset/bursts.json --videos ../MS500_Flooding_Detection/Videos --out labels/`

## What is mirrored rather than compiled

`native/device_constants.h` lists every value copied from firmware device glue (`flood_task.c`,
`flood_frame_tap.c`, `flood_roi.h`, `flood_geometry_config.c`) with its source line. Re-check it
when those files change.

## Tests

```bash
ctest --test-dir build/native --output-on-failure
.venv/Scripts/python -m pytest
```
````

- [ ] **Step 5: Run everything**

Run: `ctest --test-dir build/native --output-on-failure && .venv/Scripts/python -m pytest -v`
Expected: all pass. The cross-check passes when the reference repo is present and is skipped otherwise.

- [ ] **Step 6: Commit**

```bash
git add tools tests/test_truth_to_labels.py tests/test_reference_crosscheck.py README.md
git commit -m "feat: dataset labels tool, Python-reference cross-check, README" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

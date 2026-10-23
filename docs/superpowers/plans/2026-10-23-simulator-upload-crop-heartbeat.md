# Upload Crop and Heartbeat Setting Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the simulator upload the same crop, with the same file name and description, as the October 2026 firmware, and let the heartbeat interval be set in minutes from the CLI and the GUI.

**Architecture:** The firmware's `flood_upload_roi.c` (already compiled into `flood_core.dll`) and `flood_upload_meta.c` (added to the build) are exposed through new `sim_shim` functions; `floodsim/core.py` wraps them; `floodsim/modes.py` turns them into upload files; the CLI and the GUI session pass the new settings through; the GUI gets an upload-area overlay and draw tool plus heartbeat and serial-number fields. Last, the bilingual guide page is regenerated in the AglaiaSense design.

**Tech Stack:** C11 (MinGW gcc, CMake ≥ 3.20, Ninja), Python 3.11, ctypes, NumPy, OpenCV, PySide6, pytest/pytest-qt.

**Spec:** `docs/superpowers/specs/2026-10-23-simulator-upload-crop-heartbeat-design.md`

## Global Constraints

- Firmware repo `D:\git\NETWORK_P4_FLOOD_DETECTION` is read-only, except Task 8 adds `docs/flood-detection-guide.html` there. Firmware C is compiled unmodified.
- `SIM_ABI_VERSION` / `ABI_VERSION` = 5.
- Heartbeat: minutes 0–1440, default 60, 0 = off; readings = `minutes * 60 / period` rounded down; any non-zero minutes gives at least 1 reading.
- Upload area: firmware rules (side ≥ 32, inside 1920×1080, ≤ 518,400 px) decided by the firmware's `flood_upload_rect_valid()`; an invalid area falls back to the lines.
- Serial number: `^[A-Za-z0-9_-]{1,64}$`, default `SIM`.
- Upload JPEG: OpenCV, quality 90, no burned-in label.
- Upload names: firmware `flood_upload_filename()`; collisions get `_2`, `_3`, … before `.jpg`. Sidecar `<name>.json` holds the firmware description string unchanged.
- `samples.csv` device column `upload_file` is last; `compare_runs` does not compare it.
- Saved GUI area file: `<input>.upload_area.json` (same naming as `<input>.boxes.edited.csv`), firmware API shape `{"configured": bool, "roi": {x,y,width,height} | null}`.
- Python tests: `.venv/Scripts/python -m pytest`. Native tests: build via `.venv/Scripts/python -c "from floodsim.core import build, firmware_dir; build(firmware_dir())"`, then `ctest --test-dir build/native --output-on-failure`.
- Git Bash on Windows; repo uses `core.autocrlf=true`.

## Review Focus

1. A GUI export after the curb/lane lines were edited since the run: the uploads must be cropped with the geometry the run used, not the current one. (Task 5 test `test_export_uses_run_geometry`.)
2. A device run with heartbeat 0: no `heartbeat` rows at all, even on long clips. (Task 3 test `test_device_heartbeat_off`.)
3. `--upload-area` JSON with `"configured": false` (pasted from a camera with no drawn area): treated as no area, not an error. (Task 3 test `test_parse_upload_area_json_unconfigured`.)
4. A drawn area that breaks a rule given on the CLI: a warning, the run uses the lines, `run.json` records both. (Task 4 test `test_cli_invalid_upload_area_warns_and_uses_lines`.)
5. A non-1920×1080 frame reaching `Core.crop`: a clear `FloodSimError`, never a crash in C. (Task 2 test `test_crop_rejects_wrong_size`.)

---

### Task 1: Native shim, build and pure-C runner

**Files:**
- Create: `native/sdkconfig.h`
- Modify: `native/CMakeLists.txt`, `native/sim_shim.h`, `native/sim_shim.c`, `native/device_constants.h`, `native/flood_sim_main.c`
- Test: `native/tests/test_shim.c`, `native/CMakeLists.txt` (new ctest)

**Interfaces:**
- Consumes: firmware `flood_upload_roi_compute()`, `flood_upload_rect_valid()`, `flood_upload_roi_copy()`, `flood_upload_roi_out_w/h()`, `flood_upload_filename()`, `flood_upload_description()`, `FLOOD_UPLOAD_ROI_MAX_PIXELS`, `FLOOD_UPLOAD_ROI_MIN_SIDE`.
- Produces (C, exported):
  - `void sim_upload_roi(const int32_t *drawn4, int32_t *out8)` — out8 = {x,y,w,h,step,source,out_w,out_h}; source 0 drawn / 1 lines / 2 full; drawn4 may be NULL.
  - `int32_t sim_upload_rect_valid(const int32_t *rect4)`
  - `void sim_upload_crop(const uint8_t *rgb24, const int32_t *roi8, uint8_t *out)`
  - `int32_t sim_upload_filename(char *out, int32_t size, const char *sn, int32_t heartbeat, int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, int32_t second)`
  - `int32_t sim_upload_description(char *out, int32_t size, int32_t heartbeat, const char *id, float correlation, const char *severity, int32_t slot, const int32_t *roi8, int64_t captured_at)`
  - `void sim_device_set_heartbeat(void *dev, int32_t every)`
  - `int32_t sim_upload_max_pixels(void)`, `int32_t sim_upload_min_side(void)`
  - Removed: `sim_upload_downscale()`.
  - `flood_sim.exe --heartbeat-every N`.

Ruling carried from planning: `sim_upload_description` takes the severity **name** (the firmware's `flood_upload_meta_t.severity` is a string), not the enum the spec sketched — Python already holds names.

- [ ] **Step 1: Write the failing shim tests**

In `native/tests/test_shim.c`, in `test_abi_and_names()` replace `CHECK(sim_upload_downscale() == 2);` with:

```c
    CHECK(sim_upload_max_pixels() == 518400);
    CHECK(sim_upload_min_side() == 32);
```

Add before `int main(void)`:

```c
static void test_upload_roi(void)
{
    int32_t roi[8];
    float d[8];
    sim_default_geometry(d);
    sim_set_geometry(d);

    sim_upload_roi(NULL, roi);                      /* default lines: 1360x360, full resolution */
    CHECK(roi[0] == 560 && roi[1] == 110 && roi[2] == 1360 && roi[3] == 360);
    CHECK(roi[4] == 1 && roi[5] == 1 && roi[6] == 1360 && roi[7] == 360);

    const int32_t drawn[4] = { 100, 100, 640, 480 };
    CHECK(sim_upload_rect_valid(drawn) == 1);
    sim_upload_roi(drawn, roi);
    CHECK(roi[0] == 100 && roi[1] == 100 && roi[2] == 640 && roi[3] == 480);
    CHECK(roi[4] == 1 && roi[5] == 0 && roi[6] == 640 && roi[7] == 480);

    const int32_t outside[4] = { 1900, 0, 100, 100 };   /* invalid: falls back to the lines */
    CHECK(sim_upload_rect_valid(outside) == 0);
    sim_upload_roi(outside, roi);
    CHECK(roi[5] == 1 && roi[2] == 1360);

    float upside_down[8];                           /* lane above curb: whole frame at half size */
    memcpy(upside_down, d, sizeof d);
    upside_down[5] = 100.0f; upside_down[7] = 100.0f;
    sim_set_geometry(upside_down);
    sim_upload_roi(NULL, roi);
    CHECK(roi[5] == 2 && roi[4] == 2 && roi[6] == 960 && roi[7] == 536);
    sim_set_geometry(d);
}

static void test_upload_crop(void)
{
    uint8_t *rgb = malloc((size_t)SIM_FRAME_W * SIM_FRAME_H * 3);
    uint8_t *out = malloc(64 * 64 * 3);
    if (!rgb || !out) { ++g_failures; free(rgb); free(out); return; }
    for (int y = 0; y < SIM_FRAME_H; ++y)
        for (int x = 0; x < SIM_FRAME_W; ++x) {
            uint8_t *p = rgb + ((size_t)y * SIM_FRAME_W + x) * 3;
            p[0] = (uint8_t)x; p[1] = (uint8_t)y; p[2] = (uint8_t)(x + y);
        }
    const int32_t drawn[4] = { 8, 16, 64, 64 };
    int32_t roi[8];
    sim_upload_roi(drawn, roi);
    sim_upload_crop(rgb, roi, out);
    CHECK(out[0] == 8 && out[1] == 16);                         /* top-left = frame (8,16) */
    CHECK(out[(63 * 64 + 63) * 3] == (uint8_t)71 && out[(63 * 64 + 63) * 3 + 1] == (uint8_t)79);
    free(rgb);
    free(out);
}

static void test_upload_meta(void)
{
    char buf[512];
    CHECK(sim_upload_filename(buf, sizeof buf, "SN1", 1, 2026, 10, 22, 21, 46, 12) > 0);
    CHECK(strcmp(buf, "SN1_flood_hb_20261022_214612.jpg") == 0);
    CHECK(sim_upload_filename(buf, sizeof buf, "SN1", 0, 2026, 10, 22, 22, 1, 33) > 0);
    CHECK(strcmp(buf, "SN1_flood_20261022_220133.jpg") == 0);

    float d[8];
    int32_t roi[8];
    sim_default_geometry(d);
    sim_set_geometry(d);
    sim_upload_roi(NULL, roi);
    CHECK(sim_upload_description(buf, sizeof buf, 0, "00000A", 0.312f, "minor", 0, roi,
                                 1792731693LL) < (int32_t)sizeof buf);
    CHECK(strcmp(buf,
        "{\"purpose\":\"flood_detection\",\"kind\":\"flood\",\"id\":\"00000A\",\"correlation\":0.312,"
        "\"severity\":\"minor\",\"slot\":0,\"roi\":{\"x\":560,\"y\":110,\"width\":1360,\"height\":360},"
        "\"step\":1,\"roi_source\":\"lines\",\"source_frame\":{\"width\":1920,\"height\":1080},"
        "\"captured_at\":1792731693}") == 0);
    CHECK(sim_upload_description(buf, sizeof buf, 1, "00000B", NAN, "none", -1, roi, 0) > 0);
    CHECK(strstr(buf, "\"kind\":\"heartbeat\"") && strstr(buf, "\"correlation\":null"));
}

static void test_heartbeat_setting(void)
{
    sim_result_t r;
    void *ref = sim_gallery_new();
    void *dev = sim_device_new();
    sim_gallery_add_manual(ref, g_dry);
    script_t clear = { { g_dry, g_dry }, { 0, 0 }, { 0, 1 }, 0 };

    sim_device_set_heartbeat(dev, 0);                  /* off */
    for (int i = 0; i < 130; ++i) {
        sim_device_sample(dev, ref, scripted, &clear, &r);
        CHECK(r.outcome == 1);
    }
    sim_device_set_heartbeat(dev, 1);                  /* every dry reading */
    sim_device_sample(dev, ref, scripted, &clear, &r);
    CHECK(r.outcome == 3);
    sim_device_reset(dev);                             /* reset keeps the setting */
    sim_device_sample(dev, ref, scripted, &clear, &r);
    CHECK(r.outcome == 3);

    sim_device_free(dev);
    sim_gallery_free(ref);
}
```

Register them in `main()` after `test_band_coverage();`:

```c
    test_upload_roi();
    test_upload_crop();
    test_upload_meta();
    test_heartbeat_setting();
```

The `#include <math.h>` already present provides `NAN`. If `SIM_FRAME_W`/`SIM_FRAME_H` are not visible from `sim_shim.h`, use `1920`/`1080` literals.

- [ ] **Step 2: Run to verify it fails**

Run: `.venv/Scripts/python -c "from floodsim.core import build, firmware_dir; build(firmware_dir())"`
Expected: build FAILS (undefined references to `sim_upload_roi`, `sim_upload_max_pixels`, …).

- [ ] **Step 3: Add the build pieces**

`native/sdkconfig.h`:

```c
/* Stand-in for the ESP-IDF generated header: main/ai_process/flood_upload_meta.c only
 * compiles its body when the flood model is selected. */
#ifndef SIM_SDKCONFIG_H
#define SIM_SDKCONFIG_H
#define CONFIG_DNN_TYPE_FLOOD 1
#endif
```

`native/CMakeLists.txt`, replace the `add_library(flood_objs ...)` and `target_include_directories(flood_objs ...)` lines with:

```cmake
# Upload name/description (flood_upload_meta.c) lives in the firmware's main/, not the shared
# component; it is compiled in place too. native/ supplies the sdkconfig.h it includes.
set(FLOOD_META_DIR "${FIRMWARE_DIR}/main/ai_process")
add_library(flood_objs OBJECT ${FLOOD_SOURCES} ${FLOOD_META_DIR}/flood_upload_meta.c sim_shim.c)
target_include_directories(flood_objs PUBLIC ${FLOOD_DIR} ${FLOOD_META_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
```

`native/device_constants.h`, replace the `SIM_UPLOAD_DOWNSCALE` block with:

```c
/* Uploads: the crop comes from the firmware's flood_upload_roi.c (compiled, not mirrored).
 * Heartbeat: main/ai_process/flood_task.c:342 sets heartbeat_every to
 * minutes * 60000 / FLOOD_SAMPLE_PERIOD_MS before each reading; minutes is the Device Manager
 * setting, main/as_dm/dm_flood_settings.h (0-1440, default 60, 0 = off). floodsim applies the
 * same formula with the run's sample period (floodsim/modes.py heartbeat_readings). */
```

- [ ] **Step 4: Implement the shim functions**

`native/sim_shim.h`: set `#define SIM_ABI_VERSION        5`; delete the `sim_upload_downscale` prototype; add after `sim_heartbeat_every`:

```c
/* ---- uploads (firmware flood_upload_roi.c / flood_upload_meta.c) ---- */
/* drawn4 = {x,y,w,h} or NULL (derive from the lines). out8 = {x,y,w,h,step,source,out_w,out_h};
 * source 0 drawn, 1 lines, 2 full. Uses the current geometry. */
SIM_API void    sim_upload_roi(const int32_t *drawn4, int32_t *out8);
SIM_API int32_t sim_upload_rect_valid(const int32_t *rect4);
/* rgb24 is 1920x1080x3; out holds out_w*out_h*3 bytes. */
SIM_API void    sim_upload_crop(const uint8_t *rgb24, const int32_t *roi8, uint8_t *out);
SIM_API int32_t sim_upload_filename(char *out, int32_t size, const char *sn, int32_t heartbeat,
                                    int32_t year, int32_t month, int32_t day,
                                    int32_t hour, int32_t minute, int32_t second);
SIM_API int32_t sim_upload_description(char *out, int32_t size, int32_t heartbeat, const char *id,
                                       float correlation, const char *severity, int32_t slot,
                                       const int32_t *roi8, int64_t captured_at);
SIM_API int32_t sim_upload_max_pixels(void);
SIM_API int32_t sim_upload_min_side(void);
```

and after `sim_device_set_learning`:

```c
/* heartbeat_every in readings (0 = off), kept across sim_device_reset() and applied
 * before every reading, as flood_task.c:342 does. */
SIM_API void  sim_device_set_heartbeat(void *dev, int32_t every);
```

`native/sim_shim.c`:
- Add `#include "flood_upload_roi.h"`, `#include "flood_upload_meta.h"` and `#include <time.h>` with the other includes.
- Delete `int32_t sim_upload_downscale(void) { ... }`.
- Add `int32_t heartbeat_every;` to `sim_device_t`; in `sim_device_new()` after `calloc` succeeds set `d->heartbeat_every = FLOOD_HEARTBEAT_EVERY;`.
- `sim_device_reset()` becomes:

```c
void sim_device_reset(void *dev)
{
    sim_device_t *d = dev;
    if (d) {
        flood_capture_init(&d->capture);
        d->capture.heartbeat_every = d->heartbeat_every;   /* the setting survives a reset */
        flood_gallery_learner_init(d->learner);
    }
}
```

- In `sim_device_sample()`, immediately before the existing `d->attempt = 0;` line add `d->capture.heartbeat_every = d->heartbeat_every;   /* flood_task.c:342 */`.
- Add after `sim_device_set_learning()`:

```c
void sim_device_set_heartbeat(void *dev, int32_t every)
{
    sim_device_t *d = dev;
    if (!d) return;
    d->heartbeat_every = every < 0 ? 0 : every;
    d->capture.heartbeat_every = d->heartbeat_every;
}
```

- Add a new section at the end of the file:

```c
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
```

(`SIM_FRAME_W` is the shim's existing frame-width constant used by `sim_device_new()`; use it.)

- [ ] **Step 5: Add `--heartbeat-every` to `flood_sim.exe`**

In `native/flood_sim_main.c`:
- Add `int heartbeat_every;` to `options_t`.
- In `usage()`, change the last line to `"                 [--retry-gap 1] [--every 1] [--fps 1] [--heartbeat-every 60]\n"`.
- In `parse_args()` after `o->every = 1;` add `o->heartbeat_every = sim_heartbeat_every();`; add the branch `else if (!strcmp(a, "--heartbeat-every")) o->heartbeat_every = atoi(v);`; extend the range check to `o->every < 1 || o->fps <= 0 || o->heartbeat_every < 0` and its message to `"--period and --fps must be > 0, --retry-gap >= 0, --every >= 1, --heartbeat-every >= 0\n"`.
- After `void *dev = sim_device_new();` / its NULL check add `sim_device_set_heartbeat(dev, o.heartbeat_every);`.

In `native/CMakeLists.txt` after the `bad_geometry_rejected` test add:

```cmake
add_test(NAME heartbeat_every_negative_rejected
         COMMAND flood_sim --reference ${FIX}/reference.pgm --frames ${FIX}/frames --heartbeat-every -1)
set_tests_properties(heartbeat_every_negative_rejected PROPERTIES WILL_FAIL TRUE FIXTURES_REQUIRED sim_fixtures)
```

- [ ] **Step 6: Run the native tests**

Run: `.venv/Scripts/python -c "from floodsim.core import build, firmware_dir; build(firmware_dir())" && ctest --test-dir build/native --output-on-failure`
Expected: build succeeds; all ctest tests pass, including `shim` (prints `test_shim: all checks passed`), the three golden tests (unchanged output) and `heartbeat_every_negative_rejected`.

Python will not load the DLL until Task 2 (ABI 5 vs 4); do not run pytest in this task.

- [ ] **Step 7: Commit**

```bash
git add native/
git commit -m "feat(native): expose the firmware upload crop, names and heartbeat setting through the shim"
```

---

### Task 2: Python core wrappers

**Files:**
- Modify: `floodsim/core.py`, `floodsim/results.py`
- Test: `tests/test_core.py`

**Interfaces:**
- Consumes: Task 1 C functions.
- Produces:
  - `floodsim.results.UploadRoi` (frozen dataclass): `x:int, y:int, width:int, height:int, step:int, source:str, out_width:int, out_height:int`; method `to_array() -> list[int]` (8 ints, source as 0/1/2); `as_dict() -> dict`.
  - `Core.upload_roi(drawn: tuple[int,int,int,int] | None = None) -> UploadRoi`
  - `Core.upload_rect_valid(rect: tuple[int,int,int,int]) -> bool`
  - `Core.upload_limits() -> tuple[int, int]` → `(min_side, max_pixels)`
  - `Core.crop(rgb: np.ndarray, roi: UploadRoi) -> np.ndarray` (out_h, out_w, 3) uint8 RGB
  - `Core.upload_filename(sn: str, heartbeat: bool, local: datetime.datetime) -> str`
  - `Core.upload_description(*, heartbeat: bool, id: str, correlation: float, severity: str, slot: int, roi: UploadRoi, captured_at: int) -> str`
  - `Device.set_heartbeat(readings: int) -> None`
  - Removed: `Core.upload_downscale()`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_core.py`:

```python
import datetime as dt

from floodsim.errors import FloodSimError
from floodsim.results import UploadRoi


def test_abi_is_5():
    from floodsim.core import ABI_VERSION
    assert ABI_VERSION == 5


def test_upload_roi_from_default_lines(core):
    roi = core.upload_roi()
    assert roi == UploadRoi(560, 110, 1360, 360, 1, "lines", 1360, 360)
    assert core.upload_limits() == (32, 518400)


def test_upload_roi_drawn_and_invalid(core):
    assert core.upload_roi((100, 100, 640, 480)).source == "drawn"
    assert core.upload_rect_valid((100, 100, 640, 480))
    assert not core.upload_rect_valid((1900, 0, 100, 100))
    assert core.upload_roi((1900, 0, 100, 100)).source == "lines"


def test_crop_matches_frame(core):
    rgb = np.zeros((1080, 1920, 3), np.uint8)
    rgb[16, 8] = (1, 2, 3)
    out = core.crop(rgb, core.upload_roi((8, 16, 64, 64)))
    assert out.shape == (64, 64, 3) and tuple(out[0, 0]) == (1, 2, 3)


def test_crop_rejects_wrong_size(core):
    with pytest.raises(FloodSimError, match="1920x1080"):
        core.crop(np.zeros((720, 1280, 3), np.uint8), core.upload_roi())


def test_upload_name_and_description(core):
    local = dt.datetime(2026, 10, 22, 21, 46, 12)
    assert core.upload_filename("SN1", True, local) == "SN1_flood_hb_20261022_214612.jpg"
    desc = core.upload_description(heartbeat=False, id="00000A", correlation=0.312, severity="minor",
                                   slot=0, roi=core.upload_roi(), captured_at=1792731693)
    assert json.loads(desc) == {
        "purpose": "flood_detection", "kind": "flood", "id": "00000A", "correlation": 0.312,
        "severity": "minor", "slot": 0, "roi": {"x": 560, "y": 110, "width": 1360, "height": 360},
        "step": 1, "roi_source": "lines", "source_frame": {"width": 1920, "height": 1080},
        "captured_at": 1792731693}


def test_device_heartbeat_setting(core):
    from tests.synth import dry_gray
    ref = core.new_gallery()
    ref.add_manual(dry_gray())
    dev = core.new_device()
    dev.set_heartbeat(1)
    gray = dry_gray()
    assert dev.sample(ref, lambda attempt: (gray, [], 0)).outcome == "heartbeat"
    dev.set_heartbeat(0)
    assert all(dev.sample(ref, lambda attempt: (gray, [], 0)).outcome == "clear" for _ in range(70))
```

Make sure `tests/test_core.py` imports `json`, `numpy as np` and `pytest` at the top (add whichever are missing). If an existing test calls `core.upload_downscale()`, delete that assertion.

- [ ] **Step 2: Run to verify they fail**

Run: `.venv/Scripts/python -m pytest tests/test_core.py -q`
Expected: FAIL — `DLL ABI v5, wrapper expects v4 -- rebuild` (or `ImportError: cannot import name 'UploadRoi'`).

- [ ] **Step 3: Implement**

`floodsim/results.py`, add:

```python
_UPLOAD_SOURCES = ("drawn", "lines", "full")


@dataclass(frozen=True)
class UploadRoi:
    """The firmware's flood_upload_roi_t: the frame region uploaded, its step and where it came from."""
    x: int
    y: int
    width: int
    height: int
    step: int
    source: str          # "drawn" | "lines" | "full"
    out_width: int
    out_height: int

    @classmethod
    def from_array(cls, a) -> "UploadRoi":
        return cls(int(a[0]), int(a[1]), int(a[2]), int(a[3]), int(a[4]), _UPLOAD_SOURCES[a[5]],
                   int(a[6]), int(a[7]))

    def to_array(self) -> list[int]:
        return [self.x, self.y, self.width, self.height, self.step, _UPLOAD_SOURCES.index(self.source),
                self.out_width, self.out_height]

    def as_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "width": self.width, "height": self.height, "step": self.step,
                "source": self.source, "out_width": self.out_width, "out_height": self.out_height}
```

(If `results.py` does not import `dataclass` yet, add `from dataclasses import dataclass`.)

`floodsim/core.py`:
- `ABI_VERSION = 5`.
- Import `datetime as dt`, `c_int64` (from ctypes) and `UploadRoi` (from `.results`).
- `source_files()`: after the `files = [...]` line add
  `files += [fw / "main" / "ai_process" / name for name in ("flood_upload_meta.c", "flood_upload_meta.h")]`.
- In `_declare()`: remove `"sim_upload_downscale"` from the first tuple and add `"sim_upload_max_pixels", "sim_upload_min_side"`; add:

```python
        _IP = POINTER(c_int32)
        fn("sim_upload_roi", None, _IP, _IP)
        fn("sim_upload_rect_valid", c_int32, _IP)
        fn("sim_upload_crop", None, _U8P, _IP, _U8P)
        fn("sim_upload_filename", c_int32, c_char_p, c_int32, c_char_p, c_int32,
           c_int32, c_int32, c_int32, c_int32, c_int32, c_int32)
        fn("sim_upload_description", c_int32, c_char_p, c_int32, c_int32, c_char_p, c_float, c_char_p,
           c_int32, _IP, c_int64)
        fn("sim_device_set_heartbeat", None, c_void_p, c_int32)
```

- Remove `def upload_downscale(...)`; add to `Core`:

```python
    # uploads (firmware flood_upload_roi.c / flood_upload_meta.c)
    def upload_limits(self) -> tuple[int, int]:
        return self._lib.sim_upload_min_side(), self._lib.sim_upload_max_pixels()

    def upload_rect_valid(self, rect) -> bool:
        return bool(self._lib.sim_upload_rect_valid((c_int32 * 4)(*[int(v) for v in rect])))

    def upload_roi(self, drawn=None) -> UploadRoi:
        out = (c_int32 * 8)()
        arg = (c_int32 * 4)(*[int(v) for v in drawn]) if drawn is not None else None
        self._lib.sim_upload_roi(arg, out)
        return UploadRoi.from_array(list(out))

    def crop(self, rgb: np.ndarray, roi: UploadRoi) -> np.ndarray:
        if rgb.shape != (FRAME_H, FRAME_W, 3) or rgb.dtype != np.uint8:
            raise FloodSimError(f"crop needs a {FRAME_W}x{FRAME_H} RGB uint8 frame, got {rgb.shape} {rgb.dtype}")
        rgb = np.ascontiguousarray(rgb)
        out = np.empty((roi.out_height, roi.out_width, 3), np.uint8)
        self._lib.sim_upload_crop(_u8(rgb), (c_int32 * 8)(*roi.to_array()), _u8(out))
        return out

    def _text(self, call) -> str:
        size = 512
        while True:
            buf = ctypes.create_string_buffer(size)
            n = call(buf, size)
            if n < size:
                return buf.value.decode()
            size = n + 1

    def upload_filename(self, sn: str, heartbeat: bool, local: dt.datetime) -> str:
        return self._text(lambda buf, size: self._lib.sim_upload_filename(
            buf, size, sn.encode(), 1 if heartbeat else 0,
            local.year, local.month, local.day, local.hour, local.minute, local.second))

    def upload_description(self, *, heartbeat: bool, id: str, correlation: float, severity: str,
                           slot: int, roi: UploadRoi, captured_at: int) -> str:
        roi_arr = (c_int32 * 8)(*roi.to_array())
        return self._text(lambda buf, size: self._lib.sim_upload_description(
            buf, size, 1 if heartbeat else 0, id.encode(), float(correlation), severity.encode(),
            int(slot), roi_arr, int(captured_at)))
```

(`_u8` is the existing helper that returns a `uint8*` for a contiguous array; if it takes only gray arrays, it still works because it reads `.ctypes.data`.)

- Add to `Device`:

```python
    def set_heartbeat(self, readings: int) -> None:
        """Heartbeat interval in readings (0 = off); kept across reset(), as the device keeps its setting."""
        self._core._lib.sim_device_set_heartbeat(self._ptr, max(0, int(readings)))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv/Scripts/python -m pytest tests/test_core.py -q`
Expected: PASS. Other test files may now fail where they call `upload_downscale()` or `UploadWriter(out, downscale)` — those are fixed in Task 3; do not touch them here.

- [ ] **Step 5: Commit**

```bash
git add floodsim/core.py floodsim/results.py tests/test_core.py
git commit -m "feat(core): wrap the upload crop, names, descriptions and the heartbeat setting"
```

---

### Task 3: Upload area parsing, heartbeat conversion and upload files

**Files:**
- Create: `floodsim/upload_area.py`
- Modify: `floodsim/modes.py`, `floodsim/results.py`
- Test: `tests/test_upload_area.py` (new), `tests/test_modes.py`

**Interfaces:**
- Consumes: Task 2 `Core.upload_roi/crop/upload_filename/upload_description`, `Device.set_heartbeat`, `UploadRoi`.
- Produces:
  - `floodsim.upload_area`: `Rect = tuple[int, int, int, int]`; `parse_upload_area(value: str) -> Rect | None`; `load_upload_area(path: Path) -> Rect | None`; `save_upload_area(path: Path, rect: Rect | None) -> None` (None deletes the file); `upload_area_path(input_path: Path) -> Path`; `check_sn(sn: str) -> str`.
  - `floodsim.modes`: `heartbeat_readings(minutes: int, period_s: float) -> int`; `make_clock(source, input_path) -> Callable[[float], datetime]`; `RunParams.heartbeat_minutes: int = 60`; `UploadWriter(out_dir, core, *, drawn=None, sn="SIM", clock)` with `write(row: dict, rgb) -> str`; `upload_run_info(core, params, drawn, sn) -> dict`; `device_row(...)` adds `"upload_file": ""`; `run_device` sets the device heartbeat and fills `upload_file`.
  - `floodsim.results.DEVICE_FIELDS` ends with `"upload_file"`.

- [ ] **Step 1: Write the failing tests**

`tests/test_upload_area.py`:

```python
import json

import pytest

from floodsim.errors import FloodSimError
from floodsim.upload_area import (check_sn, load_upload_area, parse_upload_area, save_upload_area,
                                  upload_area_path)


def test_parse_upload_area_numbers():
    assert parse_upload_area("100,200,640,480") == (100, 200, 640, 480)
    assert parse_upload_area(" 1, 2 ,3,4 ") == (1, 2, 3, 4)


def test_parse_upload_area_json(tmp_path):
    p = tmp_path / "a.json"
    p.write_text(json.dumps({"configured": True, "roi": {"x": 1, "y": 2, "width": 300, "height": 400}}))
    assert parse_upload_area(str(p)) == (1, 2, 300, 400)


def test_parse_upload_area_json_unconfigured(tmp_path):
    p = tmp_path / "a.json"
    p.write_text(json.dumps({"configured": False, "roi": None, "effective": {}}))
    assert parse_upload_area(str(p)) is None


@pytest.mark.parametrize("bad", ["1,2,3", "a,b,c,d", "1.5,2,3,4", "", "missing.json"])
def test_parse_upload_area_rejects(bad):
    with pytest.raises(FloodSimError):
        parse_upload_area(bad)


def test_save_load_delete(tmp_path):
    p = upload_area_path(tmp_path / "clip.avi")
    assert p.name == "clip.avi.upload_area.json"
    save_upload_area(p, (10, 20, 300, 400))
    assert json.loads(p.read_text()) == {"configured": True,
                                         "roi": {"x": 10, "y": 20, "width": 300, "height": 400}}
    assert load_upload_area(p) == (10, 20, 300, 400)
    save_upload_area(p, None)
    assert not p.exists() and load_upload_area(p) is None


def test_check_sn():
    assert check_sn("MS500-H120_1") == "MS500-H120_1"
    for bad in ("", "has space", "a/b", "x" * 65):
        with pytest.raises(FloodSimError):
            check_sn(bad)
```

In `tests/test_modes.py`:
- Change the import line to `from floodsim.modes import RunParams, UploadWriter, heartbeat_readings, make_clock, run_dense, run_device` and add `import datetime as dt`, `import json`, `from pathlib import Path`.
- Replace `test_device_writes_what_would_be_uploaded` and `test_device_heartbeat_on_60th_sample` with:

```python
CLOCK = lambda t: dt.datetime(2026, 10, 22, 12, 0, 0) + dt.timedelta(seconds=t)


def test_device_writes_what_would_be_uploaded(core, ref, tmp_path):
    uploads = UploadWriter(tmp_path, core, sn="SN1", clock=CLOCK)
    out = run_device(core, _golden_source(), StaticBoxes({}), ref, RunParams(), uploads=uploads)
    files = sorted(f.name for f in (tmp_path / "uploads").iterdir())
    assert files == ["SN1_flood_20261022_120100.jpg", "SN1_flood_20261022_120100.json"]
    assert read_image_bgr(tmp_path / "uploads" / files[0]).shape == (360, 1360, 3)
    desc = json.loads((tmp_path / "uploads" / files[1]).read_text())
    assert desc["kind"] == "flood" and desc["id"] == "000000" and desc["roi_source"] == "lines"
    assert [r["upload_file"] for r in out.rows] == ["", "SN1_flood_20261022_120100.jpg", ""]


def test_device_upload_uses_drawn_area(core, ref, tmp_path):
    uploads = UploadWriter(tmp_path, core, drawn=(0, 0, 640, 480), clock=CLOCK)
    run_device(core, _golden_source(), StaticBoxes({}), ref, RunParams(), uploads=uploads)
    jpg = next((tmp_path / "uploads").glob("*.jpg"))
    assert jpg.name.startswith("SIM_flood_") and read_image_bgr(jpg).shape == (480, 640, 3)


def test_upload_name_collision_gets_suffix(core, ref, tmp_path):
    src = MemorySource([(0.0, FLOOD), (0.2, FLOOD), (0.4, FLOOD)])
    uploads = UploadWriter(tmp_path, core, clock=CLOCK)
    out = run_device(core, src, StaticBoxes({}), ref, RunParams(period=0.2), uploads=uploads)
    assert [r["upload_file"] for r in out.rows] == [
        "SIM_flood_20261022_120000.jpg", "SIM_flood_20261022_120000_2.jpg", "SIM_flood_20261022_120000_3.jpg"]


def test_device_heartbeat_follows_minutes(core, ref):
    src = MemorySource([(t, DRY) for t in range(60)])
    out = run_device(core, src, StaticBoxes({}), ref, RunParams(period=1.0, heartbeat_minutes=1))
    assert len(out.rows) == 60
    assert out.rows[58]["outcome"] == "clear" and out.rows[59]["outcome"] == "heartbeat"


def test_device_heartbeat_off(core, ref):
    src = MemorySource([(t, DRY) for t in range(200)])
    out = run_device(core, src, StaticBoxes({}), ref, RunParams(period=1.0, heartbeat_minutes=0))
    assert "heartbeat" not in {r["outcome"] for r in out.rows}


def test_default_heartbeat_is_hourly_in_video_time(core, ref):
    src = MemorySource([(t, DRY) for t in range(120)])
    out = run_device(core, src, StaticBoxes({}), ref, RunParams(period=1.0))
    assert "heartbeat" not in {r["outcome"] for r in out.rows}     # 60 min @ 1 s = every 3600th


@pytest.mark.parametrize("minutes,period,expected", [(0, 60, 0), (60, 60, 60), (60, 5, 720),
                                                     (1, 120, 1), (1440, 60, 1440), (1, 61, 1)])
def test_heartbeat_readings(minutes, period, expected):
    assert heartbeat_readings(minutes, period) == expected


@pytest.mark.parametrize("minutes", [-1, 1441])
def test_heartbeat_readings_range(minutes):
    with pytest.raises(FloodSimError):
        heartbeat_readings(minutes, 60)


def test_make_clock(tmp_path):
    clip = tmp_path / "record_20261022_080000.avi"
    clip.write_bytes(b"")
    assert make_clock(MemorySource([]), clip)(90) == dt.datetime(2026, 10, 22, 8, 1, 30)
    other = tmp_path / "x.avi"
    other.write_bytes(b"")
    start = dt.datetime.fromtimestamp(other.stat().st_mtime)
    assert make_clock(MemorySource([]), other)(0) == start
```

Add `from floodsim.errors import FloodSimError` to the imports if not present.

- [ ] **Step 2: Run to verify they fail**

Run: `.venv/Scripts/python -m pytest tests/test_upload_area.py tests/test_modes.py -q`
Expected: FAIL — `ModuleNotFoundError: floodsim.upload_area` and `ImportError: cannot import name 'heartbeat_readings'`.

- [ ] **Step 3: Implement `floodsim/upload_area.py`**

```python
"""The operator-drawn upload area and the serial number, in the firmware's formats."""
from __future__ import annotations

import json
import re
from pathlib import Path

from .errors import FloodSimError

Rect = tuple[int, int, int, int]
_SN = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
_KEYS = ("x", "y", "width", "height")


def upload_area_path(input_path) -> Path:
    """<input>.upload_area.json, next to <input>.boxes.edited.csv."""
    return Path(f"{input_path}.upload_area.json")


def _rect_from_roi(roi, where: str) -> Rect:
    if not isinstance(roi, dict):
        raise FloodSimError(f"{where}: \"roi\" must be an object with x, y, width, height")
    values = []
    for k in _KEYS:
        v = roi.get(k)
        if isinstance(v, bool) or not isinstance(v, int):
            raise FloodSimError(f"{where}: roi.{k} must be a whole number")
        values.append(v)
    return tuple(values)


def load_upload_area(path) -> Rect | None:
    """Reads the firmware GET /api/flood/upload_roi shape; missing file or configured=false is None."""
    path = Path(path)
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as e:
        raise FloodSimError(f"cannot read upload area {path}: {e}") from e
    if not isinstance(data, dict) or not data.get("configured"):
        return None
    return _rect_from_roi(data.get("roi"), str(path))


def save_upload_area(path, rect: Rect | None) -> None:
    path = Path(path)
    if rect is None:
        path.unlink(missing_ok=True)
        return
    path.write_text(json.dumps({"configured": True, "roi": dict(zip(_KEYS, map(int, rect)))}, indent=2) + "\n",
                    encoding="utf-8")


def parse_upload_area(value: str) -> Rect | None:
    """--upload-area: "x,y,w,h", or a JSON file in the firmware API shape."""
    text = value.strip()
    parts = [p.strip() for p in text.split(",")]
    if len(parts) == 4 and all(re.fullmatch(r"-?\d+", p) for p in parts):
        return tuple(int(p) for p in parts)
    if text.lower().endswith(".json"):
        if not Path(text).exists():
            raise FloodSimError(f"--upload-area file not found: {text}")
        return load_upload_area(text)
    raise FloodSimError(f"--upload-area must be x,y,w,h (whole numbers) or a .json file, got {value!r}")


def check_sn(sn: str) -> str:
    if not _SN.match(sn or ""):
        raise FloodSimError("serial number must be 1-64 letters, digits, '-' or '_'")
    return sn
```

- [ ] **Step 4: Implement the `modes.py` and `results.py` changes**

`floodsim/results.py`: append `"upload_file",` as the last entry of `DEVICE_FIELDS`.

`floodsim/modes.py`:
- Imports: add `import datetime as dt`, `import json`, `from .clips import clip_start`, `from .errors import FloodSimError`.
- `RunParams`: add field `heartbeat_minutes: int = 60` after `every`.
- `device_row(...)`: add `"upload_file": ""` as the last key of the returned dict.
- Add:

```python
def heartbeat_readings(minutes: int, period_s: float) -> int:
    """flood_task.c:342: minutes * 60000 / FLOOD_SAMPLE_PERIOD_MS, rounded down. 0 stays 0 (off);
    any other value gives at least 1, so a period longer than the interval still sends heartbeats."""
    if not 0 <= minutes <= 1440:
        raise FloodSimError(f"heartbeat must be 0-1440 minutes, got {minutes}")
    if minutes == 0:
        return 0
    return max(1, int(minutes * 60 // period_s))


def make_clock(source, input_path) -> Callable[[float], dt.datetime]:
    """Local wall time of a video offset: a record_YYYYMMDD_HHMMSS clip's start, else the input's
    modified time (2000-01-01 when there is no file, e.g. in-memory test sources)."""
    start = getattr(source, "start_clock", None) or clip_start(Path(input_path))
    if start is None:
        p = Path(input_path)
        start = dt.datetime.fromtimestamp(p.stat().st_mtime) if p.exists() else dt.datetime(2000, 1, 1)
    return lambda t: start + dt.timedelta(seconds=float(t))


def upload_run_info(core, params: RunParams, drawn, sn: str) -> dict:
    """run.json fields for the heartbeat and upload settings."""
    return {"heartbeat": {"minutes": params.heartbeat_minutes,
                          "readings": heartbeat_readings(params.heartbeat_minutes, params.period)},
            "upload_area": dict(zip(("x", "y", "width", "height"), drawn)) if drawn else None,
            "upload_area_used": core.upload_roi(drawn).as_dict(), "sn": sn}
```

- Replace `class UploadWriter` with:

```python
class UploadWriter:
    """What the camera would send (flood_upload.c): the upload crop of the frame, named and described
    by the firmware's flood_upload_meta.c. JPEG is OpenCV quality 90, so not byte-identical to the P4."""

    def __init__(self, out_dir: Path, core, *, drawn=None, sn: str = "SIM",
                 clock: Callable[[float], dt.datetime]):
        self.dir = Path(out_dir) / "uploads"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.core, self.drawn, self.sn, self.clock = core, drawn, sn, clock
        self._seq = 0
        self._used: set[str] = set()

    def _unique(self, name: str) -> str:
        stem, n = name[:-4], 2
        while name in self._used:
            name = f"{stem}_{n}.jpg"
            n += 1
        self._used.add(name)
        return name

    def write(self, row: dict, rgb: np.ndarray) -> str:
        heartbeat = row["outcome"] == "heartbeat"
        roi = self.core.upload_roi(self.drawn)
        crop = self.core.crop(rgb, roi)
        upload_id = f"{self._seq & 0xFFFFFF:06X}"
        self._seq += 1
        local = self.clock(row["t_sec"])
        name = self._unique(self.core.upload_filename(self.sn, heartbeat, local))
        desc = self.core.upload_description(
            heartbeat=heartbeat, id=upload_id, correlation=row["curb_correlation"],
            severity=row["severity"], slot=row["gallery_slot"], roi=roi,
            captured_at=int(local.timestamp()))
        cv2.imwrite(str(self.dir / name), np.ascontiguousarray(crop[:, :, ::-1]),
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
        (self.dir / f"{name[:-4]}.json").write_text(desc + "\n", encoding="utf-8")
        return name
```

- In `run_device`, after `device = core.new_device()` add `device.set_heartbeat(heartbeat_readings(params.heartbeat_minutes, params.period))`; replace the upload block with:

```python
        row = device_row(k, t, r)
        out.rows.append(row)
        if uploads and r.outcome in ("upload", "heartbeat") and r.frame_idx >= 0:
            rgb = source.read_rgb(r.frame_idx)
            if rgb is not None:
                row["upload_file"] = uploads.write(row, rgb)
```

(removing the old `out.rows.append(device_row(k, t, r))` line). `write_image` may now be unused in `modes.py`; remove its import only if nothing else uses it.

If `floodsim/clips.py` imports `floodsim.modes` (circular), import `clip_start` inside `make_clock` instead.

- [ ] **Step 5: Run tests to verify they pass**

Run: `.venv/Scripts/python -m pytest tests/test_upload_area.py tests/test_modes.py tests/test_core.py -q`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add floodsim/upload_area.py floodsim/modes.py floodsim/results.py tests/test_upload_area.py tests/test_modes.py
git commit -m "feat: camera-faithful upload files and a heartbeat interval in minutes"
```

---

### Task 4: CLI options and run.json

**Files:**
- Modify: `floodsim/cli.py`
- Test: `tests/test_cli.py`

**Interfaces:**
- Consumes: Task 3 `parse_upload_area`, `check_sn`, `heartbeat_readings`, `make_clock`, `UploadWriter`, `upload_run_info`, `RunParams.heartbeat_minutes`; Task 2 `Core.upload_rect_valid`.
- Produces: `floodsim run --heartbeat N --upload-area VALUE --sn TEXT`; `run.json` keys `heartbeat`, `upload_area`, `upload_area_used`, `sn`.

- [ ] **Step 1: Write the failing tests**

`tests/test_cli.py` already has the `scenario` fixture (a PNG folder dry→flooded→dry, a `dry.png` reference, a complete box cache) and calls `main([...])`, which returns 2 on a `FloodSimError`.

First fix the existing assertion in `test_run_device_writes_run_folder` (uploads now come with a `.json` sidecar):

```python
    assert len(list((tmp / "dev" / "uploads").glob("*.jpg"))) == 1
```

Then add (with `import json` at the top):

```python
def _run(scenario, name, *extra):
    folder, ref, tmp = scenario
    out = tmp / name
    assert main(["run", str(folder), "--reference", str(ref), "--out", str(out), *extra]) == 0
    return out, json.loads((out / "run.json").read_text())


def test_cli_upload_options_reach_run_json(scenario):
    out, info = _run(scenario, "dev", "--heartbeat", "0", "--upload-area", "0,0,640,480", "--sn", "CAM-1")
    assert info["heartbeat"] == {"minutes": 0, "readings": 0}
    assert info["upload_area"] == {"x": 0, "y": 0, "width": 640, "height": 480}
    assert info["upload_area_used"]["source"] == "drawn" and info["sn"] == "CAM-1"
    jpgs = list((out / "uploads").glob("*.jpg"))
    assert jpgs and all(p.name.startswith("CAM-1_flood_") for p in jpgs)
    assert "upload_file" in (out / "samples.csv").read_text().splitlines()[0]


def test_cli_invalid_upload_area_warns_and_uses_lines(scenario, caplog):
    _, info = _run(scenario, "dev", "--upload-area", "1900,0,100,100")
    assert info["upload_area"] == {"x": 1900, "y": 0, "width": 100, "height": 100}
    assert info["upload_area_used"]["source"] == "lines"
    assert "upload area" in caplog.text.lower()


@pytest.mark.parametrize("args", [["--heartbeat", "2000"], ["--sn", "bad sn"], ["--upload-area", "1,2,3"]])
def test_cli_rejects_bad_upload_options(scenario, args):
    folder, ref, tmp = scenario
    assert main(["run", str(folder), "--reference", str(ref), "--out", str(tmp / "o"), *args]) == 2


def test_cli_dense_ignores_upload_options(scenario):
    folder, ref, tmp = scenario
    out = tmp / "dense"
    assert main(["run", str(folder), "--mode", "dense", "--reference", str(ref),
                 "--heartbeat", "5", "--sn", "X", "--out", str(out)]) == 0
    assert not (out / "uploads").exists()


def test_compare_ignores_upload_names(scenario):
    a, _ = _run(scenario, "a", "--sn", "A")
    b, _ = _run(scenario, "b", "--sn", "B")
    assert main(["compare", str(a), str(b)]) == 0
    assert compare_runs(a, b) == []
```

- [ ] **Step 2: Run to verify they fail**

Run: `.venv/Scripts/python -m pytest tests/test_cli.py -q`
Expected: FAIL — `unrecognized arguments: --heartbeat`.

- [ ] **Step 3: Implement**

In the `run` subparser (after `--out`):

```python
    run.add_argument("--heartbeat", type=int, default=60,
                     help="device: heartbeat interval in minutes, 0-1440, 0 = off (Device Manager setting)")
    run.add_argument("--upload-area", help="device: drawn upload area x,y,w,h, or a JSON file from the "
                                           "camera's GET /api/flood/upload_roi")
    run.add_argument("--sn", default="SIM", help="device: serial number in upload file names")
```

Imports: `from .upload_area import check_sn, parse_upload_area` and add `UploadWriter, make_clock, upload_run_info, heartbeat_readings` to the `.modes` import.

In `cmd_run`:
- Replace the `params = RunParams(...)` line with
  `params = RunParams(args.mode, args.period or core.sample_period_s(), args.retry_gap, args.every, args.heartbeat)`.
- After the existing range check add:

```python
    heartbeat_readings(params.heartbeat_minutes, params.period)   # raises on 0-1440 violations
    sn = check_sn(args.sn)
    drawn = parse_upload_area(args.upload_area) if args.upload_area else None
    if args.mode == "dense" and (args.upload_area or args.sn != "SIM" or args.heartbeat != 60):
        log.info("dense mode never uploads: --heartbeat, --upload-area and --sn are ignored")
    if drawn is not None and not core.upload_rect_valid(drawn):
        log.warning("upload area %s breaks the firmware rules (sides >= 32, inside 1920x1080, "
                    "<= 518400 px); the camera would use the curb/lane lines, and so does this run", drawn)
```

- Replace the device branch's `uploads=UploadWriter(out, core.upload_downscale())` with
  `uploads=UploadWriter(out, core, drawn=drawn, sn=sn, clock=make_clock(source, args.input))`.
- After `run_info = make_run_info(...)` add `if args.mode == "device": run_info.update(upload_run_info(core, params, drawn, sn))`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv/Scripts/python -m pytest tests/test_cli.py tests/test_modes.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add floodsim/cli.py tests/test_cli.py
git commit -m "feat(cli): --heartbeat, --upload-area and --sn for device runs"
```

---

### Task 5: GUI session

**Files:**
- Modify: `floodsim/gui/session.py`
- Test: `tests/test_gui_session.py`

**Interfaces:**
- Consumes: Task 3 `upload_area_path`, `load_upload_area`, `save_upload_area`, `check_sn`, `heartbeat_readings`, `make_clock`, `UploadWriter`, `upload_run_info`; Task 2 `Core.upload_roi`, `Core.upload_rect_valid`, `Device.set_heartbeat`.
- Produces on `Session`:
  - attributes `heartbeat_minutes: int = 60`, `sn: str = "SIM"`, `upload_area: tuple | None`
  - `set_heartbeat(minutes: int) -> dict` (returns `overlay_geometry()`)
  - `set_sn(sn: str) -> str`
  - `set_upload_area(rect: tuple | None) -> dict` (saves/deletes `<input>.upload_area.json`, returns `overlay_geometry()`); raises `FloodSimError` for an area the firmware would reject
  - `upload_status() -> dict` — `{"roi": UploadRoi.as_dict(), "drawn": rect|None, "text": str}`
  - `overlay_geometry()` gains `"upload": upload_status()`, `"upload_limits": [min_side, max_pixels]`, and `"heartbeat_every"` now = `heartbeat_readings(heartbeat_minutes, sample_period_s)`
  - `RunRecord` gains `upload_area` and `sn` (defaults `None`, `"SIM"`)

- [ ] **Step 1: Write the failing tests**

First fix the existing assertion in `test_device_batch_export_writes_uploads` (uploads now come with a `.json` sidecar):

```python
    assert len(list((out / "uploads").glob("*.jpg"))) == 1
```

Then append to `tests/test_gui_session.py`:

```python
from floodsim.upload_area import upload_area_path


def test_upload_area_save_reload_delete(session, core, build_info, folder):
    overlay = session.set_upload_area((0, 0, 640, 480))
    assert overlay["upload"]["roi"]["source"] == "drawn"
    assert upload_area_path(folder).exists()
    again = Session(core, build_info)
    again.open(folder, no_detector=True)
    assert again.upload_area == (0, 0, 640, 480)
    session.set_upload_area(None)
    assert not upload_area_path(folder).exists()
    assert session.upload_status()["roi"]["source"] == "lines"


def test_upload_area_rejects_invalid(session):
    with pytest.raises(FloodSimError, match="upload area"):
        session.set_upload_area((1900, 0, 100, 100))


def test_line_move_changes_derived_area(session):
    before = session.upload_status()["roi"]
    g = session.geometry.to_list()
    g[1] -= 50.0; g[3] -= 50.0                     # curb line up 50 px
    session.set_geometry(Geometry.from_list(g))
    after = session.upload_status()["roi"]
    assert after["source"] == "lines" and after["y"] < before["y"]


def test_heartbeat_field_reaches_manual_sample(session):
    session.set_reference_frame(0)
    session.set_heartbeat(1)                       # 1 min @ 60 s device period = every reading
    assert session.overlay_geometry()["heartbeat_every"] == 1
    assert session.sample(0)["outcome"] == "heartbeat"
    session.reset_device()
    assert session.sample(0)["outcome"] == "heartbeat"     # Reset state keeps the setting
    with pytest.raises(FloodSimError):
        session.set_heartbeat(1441)


def test_sn_field(session):
    assert session.set_sn("CAM-7") == "CAM-7"
    with pytest.raises(FloodSimError):
        session.set_sn("bad sn")
    assert session.sn == "CAM-7"


def test_export_matches_cli_names(session, tmp_path):
    session.set_reference_frame(0)
    session.set_sn("CAM-7")
    session.set_upload_area((0, 0, 640, 480))
    session.run_batch(RunParams(mode="device", period=60.0, heartbeat_minutes=0))
    out = Path(session.export_run(tmp_path / "exp"))
    info = json.loads((out / "run.json").read_text())
    assert info["sn"] == "CAM-7" and info["upload_area_used"]["source"] == "drawn"
    assert info["heartbeat"] == {"minutes": 0, "readings": 0}
    rows = read_samples(out)
    names = [r["upload_file"] for r in rows if r["upload_file"]]
    assert names and all(n.startswith("CAM-7_flood_") for n in names)
    assert all((out / "uploads" / n).exists() for n in names)


def test_export_uses_run_geometry(session, tmp_path):
    session.set_reference_frame(0)
    session.run_batch(RunParams(mode="device", period=60.0))
    used = session.upload_status()["roi"]
    g = session.geometry.to_list()
    g[1] -= 50.0; g[3] -= 50.0
    session.set_geometry(Geometry.from_list(g))     # edit after the run
    out = Path(session.export_run(tmp_path / "exp"))
    assert json.loads((out / "run.json").read_text())["upload_area_used"] == used
    assert session.upload_status()["roi"] != used    # the session keeps the edited lines
```

- [ ] **Step 2: Run to verify they fail**

Run: `.venv/Scripts/python -m pytest tests/test_gui_session.py -q`
Expected: FAIL — `AttributeError: 'Session' object has no attribute 'set_upload_area'`.

- [ ] **Step 3: Implement**

`floodsim/gui/session.py`:
- Imports: add `heartbeat_readings, make_clock, upload_run_info` to the `..modes` import; `from ..upload_area import check_sn, load_upload_area, save_upload_area, upload_area_path`.
- `RunRecord`: add `upload_area: tuple | None = None` and `sn: str = "SIM"` as the last fields.
- `Session.__init__`: after `self.device = core.new_device()` add:

```python
        self.heartbeat_minutes = 60
        self.sn = "SIM"
        self.upload_area: tuple | None = None
        self.device.set_heartbeat(heartbeat_readings(self.heartbeat_minutes, core.sample_period_s()))
```

- `open()`: after `self.source = src` add `self.upload_area = load_upload_area(upload_area_path(src.path))`.
- Add methods (a new `# ------------------------------------------------------- uploads` section):

```python
    def upload_status(self) -> dict:
        roi = self.core.upload_roi(self.upload_area)
        names = {"drawn": "drawn", "lines": "from lines", "full": "whole frame"}
        scale = "full resolution" if roi.step == 1 else f"1/{roi.step} size"
        return {"roi": roi.as_dict(), "drawn": self.upload_area,
                "text": f"{names[roi.source]}, {roi.out_width}×{roi.out_height}, {scale}"}

    def set_upload_area(self, rect) -> dict:
        src = self._src()
        if rect is not None:
            rect = tuple(int(round(v)) for v in rect)
            if not self.core.upload_rect_valid(rect):
                lo, hi = self.core.upload_limits()
                raise FloodSimError(f"upload area must have sides of at least {lo} px, lie inside "
                                    f"1920x1080 and cover at most {hi} px")
        save_upload_area(upload_area_path(src.path), rect)
        self.upload_area = rect
        return self.overlay_geometry()

    def set_heartbeat(self, minutes: int) -> dict:
        readings = heartbeat_readings(int(minutes), self.core.sample_period_s())
        self.heartbeat_minutes = int(minutes)
        self.device.set_heartbeat(readings)
        return self.overlay_geometry()

    def set_sn(self, sn: str) -> str:
        self.sn = check_sn(sn)
        return self.sn
```

- `overlay_geometry()`: replace `"heartbeat_every": self.core.heartbeat_every(),` with
  `"heartbeat_every": heartbeat_readings(self.heartbeat_minutes, self.core.sample_period_s()),` and add
  `"upload": self.upload_status(), "upload_limits": list(self.core.upload_limits()),`.
- `run_batch()`: change `self.run = RunRecord(params, out, self.geometry, reference, {...})` to pass `upload_area=self.upload_area, sn=self.sn` as keywords.
- `export_run()`: replace the device upload block with:

```python
        rows = [dict(r) for r in result.rows]
        if params.mode == "device":
            uploads = UploadWriter(out, self.core, drawn=run.upload_area, sn=run.sn,
                                   clock=make_clock(src, src.path))
            self.core.set_geometry(run.geometry)          # crop with the lines the run used
            try:
                for row in rows:
                    if row["outcome"] in ("upload", "heartbeat") and row["frame_idx"] >= 0:
                        rgb = src.read_rgb(row["frame_idx"])
                        if rgb is not None:
                            row["upload_file"] = uploads.write(row, rgb)
            finally:
                self.core.set_geometry(self.geometry)
```

  then pass `rows` instead of `result.rows` to `write_run(...)` and to `summarize(...)`; after `info["cancelled"] = result.cancelled` add:

```python
        if params.mode == "device":
            self.core.set_geometry(run.geometry)
            try:
                info.update(upload_run_info(self.core, params, run.upload_area, run.sn))
            finally:
                self.core.set_geometry(self.geometry)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv/Scripts/python -m pytest tests/test_gui_session.py tests/gui/test_batch.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add floodsim/gui/session.py tests/test_gui_session.py
git commit -m "feat(gui): session keeps the upload area, heartbeat and serial number"
```

---

### Task 6: GUI widgets

**Files:**
- Create: `floodsim/gui/upload_area.py`
- Modify: `floodsim/gui/frame_view.py`, `floodsim/gui/panels.py`, `floodsim/gui/main_window.py`
- Test: `tests/gui/test_frame_view.py`, `tests/gui/test_panels.py`, `tests/gui/test_main_window.py`

**Interfaces:**
- Consumes: Task 5 `Session.set_upload_area`, `set_heartbeat`, `set_sn`, `upload_status`, overlay keys `upload`, `upload_limits`, `heartbeat_every`.
- Produces:
  - `floodsim.gui.upload_area`: `UPLOAD_COLOR = "#ff9500"`, `INVALID_COLOR = "#ff3b30"`; `rect_problem(x, y, w, h, min_side, max_pixels) -> str | None`.
  - `Tool.DRAW_UPLOAD = "Draw upload area"` (appears automatically as a radio button in `ToolsPanel`); `FrameView.uploadAreaDrawn = Signal(QRectF)`; `OverlayToggles.upload: bool = True`.
  - `VerdictPanel.heartbeatChanged = Signal(int)`, `VerdictPanel.snChanged = Signal(str)`, widgets `heartbeat_spin` (QSpinBox 0–1440, suffix " min"), `sn_edit` (QLineEdit), `upload_label` (QLabel); method `show_upload(text: str)`.
  - `RunDialog(defaults: RunParams)` shows a heartbeat spin box in device mode; `params()` returns it as `heartbeat_minutes`.
  - Menu **Upload area** ▸ **Draw** (selects `Tool.DRAW_UPLOAD`), **Delete**.

- [ ] **Step 1: Write the failing tests**

`tests/gui/test_panels.py`, add:

```python
def test_verdict_panel_heartbeat_and_sn_fields(qtbot):
    from floodsim.gui.panels import VerdictPanel
    p = VerdictPanel()
    qtbot.addWidget(p)
    assert p.heartbeat_spin.minimum() == 0 and p.heartbeat_spin.maximum() == 1440
    assert p.heartbeat_spin.value() == 60 and p.sn_edit.text() == "SIM"
    with qtbot.waitSignal(p.heartbeatChanged) as hb:
        p.heartbeat_spin.setValue(5)
    assert hb.args == [5]
    with qtbot.waitSignal(p.snChanged) as sn:
        p.sn_edit.setText("CAM-1")
        p.sn_edit.editingFinished.emit()
    assert sn.args == ["CAM-1"]
    p.show_upload("from lines, 1360×360, full resolution")
    assert "1360" in p.upload_label.text()


def test_rect_problem():
    from floodsim.gui.upload_area import rect_problem
    assert rect_problem(0, 0, 960, 540, 32, 518400) is None
    assert "32" in rect_problem(0, 0, 31, 100, 32, 518400)
    assert "inside" in rect_problem(1900, 0, 100, 100, 32, 518400)
    assert "518400" in rect_problem(0, 0, 1000, 600, 32, 518400)
```

`tests/gui/test_frame_view.py`, add (reuse the file's existing view fixture/pattern for creating a `FrameView` with an image; the overlay dict needs the keys `set_geometry_overlay` already reads plus the new ones):

```python
def test_upload_area_overlay_and_draw(qtbot):
    from PySide6.QtCore import QPointF
    from floodsim.gui.frame_view import FrameView, Tool
    view = FrameView()
    qtbot.addWidget(view)
    view.set_image(np.zeros((1080, 1920, 3), np.uint8))
    overlay = {"geometry": [560, 230, 1920, 355, 560, 378, 1920, 446], "strip": ([], [], []),
               "align": (0, 0, 0, 0), "domain": (560, 1920),
               "upload": {"roi": {"x": 560, "y": 110, "width": 1360, "height": 360, "step": 1,
                                  "source": "lines", "out_width": 1360, "out_height": 360},
                          "drawn": None, "text": ""},
               "upload_limits": [32, 518400]}
    view.set_geometry_overlay(overlay)
    assert view.upload_rect_item() is not None and view.upload_rect_item().pen().style() != Qt.SolidLine
    view.set_tool(Tool.DRAW_UPLOAD)
    with qtbot.waitSignal(view.uploadAreaDrawn) as sig:
        assert view._press(QPointF(100, 100))
        view._move(QPointF(740, 580))
        view._release(QPointF(740, 580))
    r = sig.args[0]
    assert (round(r.x()), round(r.y()), round(r.width()), round(r.height())) == (100, 100, 640, 480)
```

(If `strip` is consumed as numpy arrays, pass `(np.array([]), np.array([]), np.array([]))`; import `Qt` from `PySide6.QtCore` and `numpy as np` at the top if missing.)

`tests/gui/test_main_window.py` (it already has the `window` fixture, `_open(qtbot, win, path)`, `T`, `write_image`, `to_rgb`, `dry_gray`), add:

```python
def test_upload_area_menu_heartbeat_and_sn(qtbot, tmp_path, window, monkeypatch):
    from PySide6.QtCore import QRectF
    from PySide6.QtWidgets import QMessageBox
    img = tmp_path / "dry.png"
    write_image(img, to_rgb(dry_gray()))
    _open(qtbot, window, img)
    monkeypatch.setattr(QMessageBox, "question", lambda *a, **k: QMessageBox.Save)
    window.upload_draw_action.trigger()
    assert window.frame_view.tool is Tool.DRAW_UPLOAD
    window.frame_view.uploadAreaDrawn.emit(QRectF(0, 0, 640, 480))
    qtbot.waitUntil(lambda: window.session.upload_area == (0, 0, 640, 480))
    window.upload_delete_action.trigger()
    qtbot.waitUntil(lambda: window.session.upload_area is None)
    window.verdict.heartbeat_spin.setValue(5)
    qtbot.waitUntil(lambda: window.session.heartbeat_minutes == 5)
    window.verdict.sn_edit.setText("CAM-9")
    window.verdict.sn_edit.editingFinished.emit()
    qtbot.waitUntil(lambda: window.session.sn == "CAM-9")


def test_run_dialog_heartbeat(qtbot):
    from floodsim.gui.main_window import RunDialog
    from floodsim.modes import RunParams
    d = RunDialog(RunParams(mode="device", heartbeat_minutes=7))
    qtbot.addWidget(d)
    assert d.heartbeat.value() == 7
    d.heartbeat.setValue(0)
    assert d.params().heartbeat_minutes == 0
```

- [ ] **Step 2: Run to verify they fail**

Run: `.venv/Scripts/python -m pytest tests/gui -q`
Expected: FAIL — `AttributeError: ... 'heartbeat_spin'`, `ModuleNotFoundError: floodsim.gui.upload_area`, `AttributeError: DRAW_UPLOAD`.

- [ ] **Step 3: Implement `floodsim/gui/upload_area.py`**

```python
"""Upload-area drawing rules and colours for the GUI (the camera web page's orange box)."""
from __future__ import annotations

UPLOAD_COLOR = "#ff9500"
INVALID_COLOR = "#ff3b30"
FRAME_W, FRAME_H = 1920, 1080


def rect_problem(x: float, y: float, w: float, h: float, min_side: int, max_pixels: int) -> str | None:
    """Why a drawn rectangle breaks the firmware rules (flood_upload_rect_valid), or None."""
    if w < min_side or h < min_side:
        return f"Each side must be at least {min_side} px"
    if x < 0 or y < 0 or x + w > FRAME_W or y + h > FRAME_H:
        return "The area must lie inside the 1920x1080 frame"
    if w * h > max_pixels:
        return f"The area must cover at most {max_pixels} px"
    return None
```

- [ ] **Step 4: Implement `frame_view.py`**

- Import `from .upload_area import INVALID_COLOR, UPLOAD_COLOR, rect_problem`.
- `Tool`: add `DRAW_UPLOAD = "Draw upload area"` after `DELETE_BOX`.
- `OverlayToggles`: add `upload: bool = True`.
- `FrameView`: add `uploadAreaDrawn = Signal(QRectF)`; in `__init__` add `self._upload = None`, `self._upload_limits = (32, 518400)`, `self._upload_item = None`.
- `set_geometry_overlay()`: add `self._upload = overlay.get("upload")` and `self._upload_limits = tuple(overlay.get("upload_limits", self._upload_limits))` before its existing redraw call.
- In `redraw()`, after the curb/lane lines are added to `group` (and before boxes), add:

```python
        self._upload_item = None
        if self._upload and self.toggles.upload:
            r = self._upload["roi"]
            item = QGraphicsRectItem(QRectF(r["x"], r["y"], r["width"], r["height"]))
            item.setPen(_pen(UPLOAD_COLOR, 2.0, dashed=r["source"] != "drawn"))
            group.addToGroup(item)
            self._upload_item = item
```

  (Use the attribute name the class already uses for `OverlayToggles`; if it is not `self.toggles`, match it.)
- Add `def upload_rect_item(self): return self._upload_item`.
- `_press()`: before the `if self.tool is Tool.DRAW_BOX:` branch add a branch for `Tool.DRAW_UPLOAD` identical to the box branch except the pen is `_pen(UPLOAD_COLOR, 2.0)`.
- `_move()`: in the rubber-band branch, after `setRect(...)`, when `self.tool is Tool.DRAW_UPLOAD` recolour:

```python
            if self.tool is Tool.DRAW_UPLOAD:
                r = self._rubber.rect()
                bad = rect_problem(r.x(), r.y(), r.width(), r.height(), *self._upload_limits)
                self._rubber.setPen(_pen(INVALID_COLOR if bad else UPLOAD_COLOR, 2.0))
```

- `_release()`: in the rubber-band branch, replace the final `if rect.width() >= MIN_BOX ...: self.boxDrawn.emit(rect)` with:

```python
            if self.tool is Tool.DRAW_UPLOAD:
                self.uploadAreaDrawn.emit(rect)
            elif rect.width() >= MIN_BOX and rect.height() >= MIN_BOX:
                self.boxDrawn.emit(rect)
```

- [ ] **Step 5: Implement `panels.py`**

- Import `QLineEdit`, `QSpinBox` from `PySide6.QtWidgets` if missing.
- `VerdictPanel`: add class signals `heartbeatChanged = Signal(int)` and `snChanged = Signal(str)`. In `__init__`, in the "State" form after `state.addRow("Since image", self.since_label)` add:

```python
        self.heartbeat_spin = QSpinBox()
        self.heartbeat_spin.setRange(0, 1440)
        self.heartbeat_spin.setSuffix(" min")
        self.heartbeat_spin.setValue(60)
        self.heartbeat_spin.setToolTip("Flood Heartbeat Interval: minutes between dry-weather images; 0 = off")
        self.heartbeat_spin.valueChanged.connect(self.heartbeatChanged)
        self.sn_edit = QLineEdit("SIM")
        self.sn_edit.setToolTip("Serial number used in upload file names")
        self.sn_edit.editingFinished.connect(lambda: self.snChanged.emit(self.sn_edit.text().strip()))
        self.upload_label = QLabel(DASH)
        self.upload_label.setWordWrap(True)
        state.addRow("Heartbeat", self.heartbeat_spin)
        state.addRow("Serial number", self.sn_edit)
        state.addRow("Upload area", self.upload_label)
```

- Add `def show_upload(self, text: str) -> None: self.upload_label.setText(text)`.

- [ ] **Step 6: Implement `main_window.py`**

- `RunDialog.__init__`: after `self.every` setup add

```python
        self.heartbeat = QSpinBox()
        self.heartbeat.setRange(0, 1440)
        self.heartbeat.setSuffix(" min")
        self.heartbeat.setValue(defaults.heartbeat_minutes)
```

  and in the device branch add `form.addRow("Heartbeat", self.heartbeat)`. `params()` returns
  `RunParams(self._mode, self.period.value(), self.retry.value(), self.every.value(), self.heartbeat.value())`.
- `_build_menus()`: after the `&Boxes` menu add

```python
        m = bar.addMenu("&Upload area")
        self.upload_draw_action = self._action(m, "Draw", lambda: self.tools.tool_buttons[Tool.DRAW_UPLOAD].setChecked(True))
        self.upload_delete_action = self._action(m, "Delete", self._delete_upload_area)
```

  (Ensure `Tool` is imported from `.frame_view`; the radio button's `toggled` already routes to `frame_view.set_tool` through `toolChanged`.)
- `_connect()`: add

```python
        self.frame_view.uploadAreaDrawn.connect(self._on_upload_drawn)
        self.verdict.heartbeatChanged.connect(self._on_heartbeat_changed)
        self.verdict.snChanged.connect(self._on_sn_changed)
```

- `_on_overlay(overlay)`: after `set_geometry_overlay(overlay)` add
  `self.verdict.show_upload(overlay["upload"]["text"]); self.upload_delete_action.setEnabled(overlay["upload"]["drawn"] is not None)`.
- `_on_opened(...)`: after the session opens, call `self.verdict.heartbeat_spin.setValue(self.session.heartbeat_minutes)` and `self.verdict.sn_edit.setText(self.session.sn)` (these read plain attributes and are safe on the GUI thread).
- Add slots:

```python
    def _on_upload_drawn(self, rect: QRectF) -> None:
        w, h = int(round(rect.width())), int(round(rect.height()))
        answer = QMessageBox.question(self, "Upload area", f"Save the upload area {w}×{h}?",
                                      QMessageBox.Save | QMessageBox.Cancel)
        if answer != QMessageBox.Save:
            return
        r = (int(round(rect.x())), int(round(rect.y())), w, h)
        self.worker.submit(lambda: self.session.set_upload_area(r), self._on_overlay, self._show_error)

    def _delete_upload_area(self) -> None:
        self.worker.submit(lambda: self.session.set_upload_area(None), self._on_overlay, self._show_error)

    def _on_heartbeat_changed(self, minutes: int) -> None:
        self.worker.submit(lambda: self.session.set_heartbeat(minutes), self._on_overlay, self._show_error)

    def _on_sn_changed(self, sn: str) -> None:
        def failed(message: str) -> None:
            self.verdict.sn_edit.setText(self.session.sn)
            self._show_error(message)
        self.worker.submit(lambda: self.session.set_sn(sn), lambda _: None, failed)
```

  (`_on_overlay` stores `self._overlay` already; check its body and keep that behaviour. If `worker.submit` takes the error callback under another name, match the existing calls such as in `_start_run`.)
- `_start_run()`: build the dialog with `RunParams(mode=mode, period=period, heartbeat_minutes=self.session.heartbeat_minutes)`.

- [ ] **Step 7: Run tests to verify they pass**

Run: `.venv/Scripts/python -m pytest tests/gui tests/test_gui_session.py -q`
Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add floodsim/gui/ tests/gui/
git commit -m "feat(gui): upload-area overlay and draw tool, heartbeat and serial number fields"
```

---

### Task 7: Simulator docs and full suite

**Files:**
- Modify: `README.md`, `docs/HANDBOOK.md`; rebuild `docs/HANDBOOK.pdf` when possible

**Interfaces:**
- Consumes: everything above (names, options, file formats).
- Produces: user docs matching the code.

- [ ] **Step 1: Update `README.md`**

- In the "Use" block, after the first `floodsim run` example add:

```bash
# a camera's settings: heartbeat every 15 min, its drawn upload area, its serial number
.venv/Scripts/floodsim run clip.avi --reference dry.jpg --heartbeat 15 --upload-area 400,120,960,540 --sn MS500-H120-EP-2614-1001
```

- Add a paragraph after the gallery paragraph:

  "Uploads match the October 2026 firmware: each `upload` or `heartbeat` writes `uploads/<SN>_flood[_hb]_YYYYMMDD_HHMMSS.jpg` — the crop the camera sends (a drawn `--upload-area`, else derived from the curb/lane lines) — and a `.json` beside it with the camera's description. Names and descriptions come from the firmware's own `flood_upload_meta.c`; the JPEG is OpenCV quality 90, so it is not byte-identical to the camera's. Times are the clip's real start (`record_YYYYMMDD_HHMMSS`) or the input's modified time, plus the video offset. `samples.csv` names each upload in `upload_file`. `--heartbeat` is in minutes (0 = off, default 60) and is converted to readings with `--period`, as the camera converts it with its 60 s period."

- In the GUI bullets add: "**Upload area / heartbeat / serial number:** Upload area ▸ Draw (or the *Draw upload area* tool) and ▸ Delete, saved to `<input>.upload_area.json`; the orange box shows the area in use (dashed when derived from the lines). *Heartbeat* and *Serial number* sit in the State panel; the device run dialog starts from the heartbeat value."
- In "What is mirrored rather than compiled", note that the upload crop and names are compiled from `flood_upload_roi.c` and `flood_upload_meta.c`, and the heartbeat formula is mirrored from `flood_task.c:342`.
- Change `flood_sim.exe` usage to mention `--heartbeat-every N`.

- [ ] **Step 2: Update `docs/HANDBOOK.md`**

- §2 outcomes table: `heartbeat` row → "Dry, and the heartbeat interval (Device Manager *Flood Heartbeat Interval*, minutes; 0 = off) has passed since the last image".
- §4 GUI: add an "Upload area, heartbeat and serial number" subsection with the same content as the README GUI bullet.
- §5 `floodsim run`: document `--heartbeat`, `--upload-area` (both forms, invalid → warning and lines) and `--sn`.
- §6 `flood_sim.exe`: `--heartbeat-every`.
- §7 run folder: `uploads/` naming + sidecar, `upload_file` column, `run.json` keys `heartbeat`, `upload_area`, `upload_area_used`, `sn`.

- [ ] **Step 3: Rebuild the PDF if the tools exist**

Run: `.venv/Scripts/python -c "import reportlab, matplotlib" && .venv/Scripts/python tools/build_handbook_pdf.py`
Expected: `docs/HANDBOOK.pdf` rewritten. If the import fails, skip and record "HANDBOOK.pdf not rebuilt: reportlab/matplotlib not installed" in the task report.

- [ ] **Step 4: Run every test**

Run: `.venv/Scripts/python -c "from floodsim.core import build, firmware_dir; build(firmware_dir())" && ctest --test-dir build/native --output-on-failure && .venv/Scripts/python -m pytest -q`
Expected: all ctest tests pass; pytest all pass (skips only where they already skipped, e.g. no `onnxruntime`/model).

- [ ] **Step 5: Commit**

```bash
git add README.md docs/HANDBOOK.md docs/HANDBOOK.pdf
git commit -m "docs: upload crop, upload names and heartbeat settings"
```

---

### Task 8: Flood detection guide page (both repos)

**Files:**
- Create: `ms500-p4-simulator/docs/flood-detection-guide.html`, `NETWORK_P4_FLOOD_DETECTION/docs/flood-detection-guide.html` (identical)
- Read: `D:\git\aglaiasense-design\design-system\readme.md`, `tokens/colors.css`, `tokens/typography.css`, `tokens/spacing.css`, `assets/logo.png`, `assets/logo-white.png`; the existing draft `C:\Users\natal\AppData\Local\Temp\claude\D--git-NETWORK-P4-FLOOD-DETECTION\80b4130e-42c7-46c2-954e-0a767a21a2f9\scratchpad\ms500-flood-readme.html` (content source)

**Interfaces:**
- Consumes: the simulator behaviour from Tasks 1–7; firmware facts already in the draft.
- Produces: one self-contained HTML file; the artifact https://claude.ai/artifact/WomxpWr4yUY2yLo6qXfYSb republished from it.

- [ ] **Step 1: Read the design tokens**

Read the four token files and the readme sections "Visual foundations", "Content fundamentals", "Accessibility". Copy the CSS custom properties actually used (colors for light and `[data-theme="dark"]`, `--font-*`, `--text-*`, `--space-*`, `--radius-*`, `--shadow-card`, `--color-focus-ring`) into the page's `<style>` verbatim; do not invent values.

- [ ] **Step 2: Write the page**

Start from the draft's content and structure (language switch, sections Concepts / Firmware / Simulator, tables, warning box, the to-scale SVG of the default lines and the 1360×360 crop). Changes:

- Full HTML document (`<!doctype html>`, `<html lang="en" data-lang="en">`, charset and `width=device-width, initial-scale=1` viewport metas, `<title>MS500-P4 flood detection guide</title>`), since it is opened as a local file.
- Design system: Inter (body/UI) and Outfit (page title only) from Google Fonts with system fallbacks; light theme default on `:root`, dark under `[data-theme="dark"]` plus `@media (prefers-color-scheme: dark)` guarded by `:root:not([data-theme="light"])`; a top-bar theme toggle (Light / Dark, `aria-pressed`) next to the language switch (English / 中文); AA teal `#0b7a67` light / `#1e7b69` dark for links, active toggles and section labels; neutral grays; 6 px control / 10 px card radius light, 4 px dark; sentence case; section labels 11 px uppercase mono with 0.12em tracking; no emoji; visible `:focus-visible` ring; `prefers-reduced-motion` respected.
- Logo: embed `assets/logo.png` (light) and `assets/logo-white.png` (dark) as base64 data URIs (generate with `python -c "import base64,sys;print(base64.b64encode(open(sys.argv[1],'rb').read()).decode())" <file>`), shown top-left, swapping with the theme.
- Language and theme choices remembered in `localStorage` inside `try/catch`.
- Simulator section: add `--heartbeat`, `--upload-area`, `--sn`, the upload naming and sidecar, `upload_file`, the GUI Upload area menu / tool and the Heartbeat and Serial number fields. Replace the "Differences from the camera" table with one row: the JPEG encoder (OpenCV quality 90 vs the P4 hardware encoder) — crop, names and descriptions are identical.
- Keep every firmware fact from the draft (it was checked against the code on 2026-10-23); update the "Updated" date to the day this task runs.
- Every English block has its Chinese counterpart and vice versa.

Write it to `ms500-p4-simulator/docs/flood-detection-guide.html`.

- [ ] **Step 3: Check the file**

Run: `.venv/Scripts/python - <<'EOF'
from html.parser import HTMLParser
import pathlib, re
p = pathlib.Path("docs/flood-detection-guide.html"); s = p.read_text(encoding="utf-8")
class P(HTMLParser):
    def __init__(self): super().__init__(); self.stack = []
    def handle_starttag(self, t, a):
        if t not in {"meta","link","br","img","input","hr","source","path","line","rect","circle"}: self.stack.append(t)
    def handle_endtag(self, t):
        if self.stack and self.stack[-1] == t: self.stack.pop()
P().feed(s)
en, zh = len(re.findall(r'class="en"', s)), len(re.findall(r'class="zh"', s))
print("bytes", len(s.encode()), "en", en, "zh", zh)
assert en == zh, "every English block needs a Chinese one"
assert "http" not in re.sub(r'https://(fonts\.googleapis\.com|fonts\.gstatic\.com|claude\.ai)[^"\s]*', "", s).replace("http://www.w3.org", ""), "no other remote resources"
EOF`
Expected: prints sizes with equal `en`/`zh` counts; no assertion error. Open the file in a browser once and toggle language and theme.

- [ ] **Step 4: Copy to the firmware repo and commit both**

```bash
cp docs/flood-detection-guide.html /d/git/NETWORK_P4_FLOOD_DETECTION/docs/flood-detection-guide.html
git add docs/flood-detection-guide.html
git commit -m "docs: bilingual flood detection guide (AglaiaSense design)"
cd /d/git/NETWORK_P4_FLOOD_DETECTION
git add docs/flood-detection-guide.html
git commit -m "docs: bilingual flood detection guide (AglaiaSense design)"
```

In the firmware repo never stage `CMakeLists.txt` or `docs/task.md`.

- [ ] **Step 5: Republish the artifact**

The controller (not a subagent) republishes the page to https://claude.ai/artifact/WomxpWr4yUY2yLo6qXfYSb with the Artifact tool (`url` set to that link, `file_path` the simulator copy). The artifact wrapper supplies its own document skeleton, so publish a copy with the `<!doctype>`, `<html>`, `<head>` and `<body>` wrapper tags stripped (keep `<title>`, `<style>`, content and `<script>`).

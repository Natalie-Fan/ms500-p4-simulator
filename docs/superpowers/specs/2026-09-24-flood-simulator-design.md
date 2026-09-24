# MS500-P4 Flood Detection Simulator — Design

## Goal

A Windows desktop simulator that runs the flood-detection C code from
`D:\git\NETWORK_P4_FLOOD_DETECTION` against **image or video files** instead of the ESP32-P4
camera, so the detector can be:

1. **(Primary) Evaluated on recorded footage** — run clips/images through the real firmware
   decision logic, tune thresholds and curb/lane geometry, and score accuracy against labels.
2. **(Secondary, equal) Regression-tested** — confirm firmware edits don't change decisions on
   known inputs before flashing.
3. **(Secondary, equal) Visually debugged** — step through frames and see verdict, correlation,
   overlays; edit inputs by hand and re-run.

Full device emulation (web UI, MQTT, real uploads) is **out of scope**.

### Success criteria

- Every flood decision the simulator reports comes from the firmware's own, unmodified
  `flood_capture.c` / `flood_curbcheck.c` / `flood_veto.c`, compiled from the firmware repo in place.
- After editing firmware C, re-running the simulator picks up the change with no manual steps.
- Device mode reproduces the device's sampling/retry/heartbeat behaviour on a video's timeline.
- A GUI run and a CLI run with identical settings produce identical `samples.csv`; the DLL path and
  `flood_sim.exe` produce identical results for identical inputs.

## Source facts this design depends on

Taken from the firmware repo at commit `7c9bed2`:

- **Portable core**: `components/as_code/process_flood_detection/` — `flood_capture.[ch]`,
  `flood_curbcheck.[ch]`, `flood_veto.[ch]`, `flood_geometry.h` (~650 lines, C99, no allocation, no
  ESP-IDF dependency). The only external symbol is `extern flood_geometry_t g_flood_geometry`
  (defined on-device in `main/ai_process/flood_geometry_config.c`).
- **Device glue (not compiled by the simulator)**: `main/ai_process/flood_task.c` (FreeRTOS loop,
  60 s sample period, reference-ready gate, upload/log per outcome), `main/bsp_video/flood_frame_tap.c`
  (RGB24→gray), `main/bsp_video/flood_roi.c` (÷2 composite + label + HW JPEG),
  `main/ai_process/flood_calibration.c` (reference persistence).
- **Stage-1 boxes** come from the IMX500 on-sensor DNN (`multi_detection_result`, already in
  1920×1080 coordinates), class order `PERSON, BICYCLE, CAR, MOTORCYCLE, BUS, TRUCK`
  (`map_traffic_class()` in `flood_task.c`).
- **Grayscale conversion** on device: `(r*77 + g*151 + b*28) >> 8` (`flood_frame_tap.c`).
- **Unready reference is unsafe to sample**: `flood_curb_correlation_at()` returns 0 when
  `!ref->ready`, which grades as a flood; `flood_task.c` gates on `ref.ready` before sampling.
- Python reference implementation and labelled dataset: `D:\git\MS500_Flooding_Detection`
  (`ms500/*.py`, `dataset/truth.csv`, `bursts.json`, `dry_reference_full.jpg`; footage via
  `scripts/fetch_dataset.py`, ~530 MB). YOLO veto there: `yolo26n.pt`, conf 0.25, 640×480 input.

## Decisions

| Topic | Decision |
|---|---|
| Architecture | Python app + firmware C compiled into `flood_core.dll` (ctypes), plus standalone `flood_sim.exe` from the same sources |
| Toolchain | MinGW gcc + CMake (already installed); Python **3.11+ venv** |
| Firmware sync | No copied sources; CMake reads `FIRMWARE_DIR`; auto-rebuild on stale DLL |
| Stage-1 boxes | YOLO on PC (ultralytics) with per-frame sidecar cache; cache replayed for repeatable runs |
| Modes | `device`, `dense`, and interactive `manual` (GUI) |
| GUI | PySide6, single workspace (layout A) |

## Architecture

```
D:\git\ms500-p4-simulator\
├── native\
│   ├── CMakeLists.txt         FIRMWARE_DIR (default D:\git\NETWORK_P4_FLOOD_DETECTION, env override
│   │                          FLOOD_FIRMWARE_DIR) → glob process_flood_detection/*.c (CONFIGURE_DEPENDS)
│   │                          → static lib flood_core_static → flood_core.dll + flood_sim.exe
│   ├── stubs\                 no-op sdkconfig.h / esp_log.h etc. — added only when firmware needs them
│   ├── device_constants.h     every value mirrored from device glue, each with firmware file:line
│   ├── sim_shim.h / .c        the ONLY code that touches firmware structs
│   └── flood_sim_main.c       flood_sim.exe
├── floodsim\                  Python package
│   ├── core.py                ctypes wrapper, auto-rebuild, ABI check, firmware provenance
│   ├── source.py              image / video / image-folder → RGB24 1920×1080 frames + timestamps
│   ├── boxes.py               YOLO detector + sidecar cache + edited-box overlay
│   ├── modes.py               device / dense runners
│   ├── geometry.py            geometry JSON load/save
│   ├── labels.py / report.py  labels parsing, scoring, run folder writing, compare
│   ├── cli.py                 `floodsim` entry point
│   └── gui\                   PySide6 app
├── tools\truth_to_labels.py   converts reference-repo truth.csv/bursts.json → labels CSV
├── tests\
└── pyproject.toml
```

### C shim (`sim_shim.c`) — stable, flat API for Python and the exe

- `g_flood_geometry` definition (initialised to the firmware defaults in `flood_geometry.h`),
  `sim_set_geometry(const float[8])`, `sim_get_geometry(float[8])`.
- `sim_rgb_to_gray(rgb24, w, h, gray_out)` — mirrors `flood_frame_tap.c` exactly.
- `sim_ref_size()`, `sim_ref_build(gray, w, h, ref_buf) -> bool` — wraps `flood_reference_init()`;
  Python holds the reference as an opaque byte buffer of `sim_ref_size()` bytes.
- `sim_score(ref_buf, gray, boxes, n, sim_score_t*)` — dense: `flood_veto_check()` then
  `flood_curbcheck(registered=true)`; returns flat veto + curb fields.
- `sim_device_*` — device loop state (capture state + reference-ready gate) mirroring `flood_task.c`:
  `sim_device_init`, `sim_device_reset`, `sim_device_sample(state, ref_buf, provider_cb, user,
  sim_result_t*)`. `provider_cb(attempt, gray_out, boxes_out, &n) -> bool` supplies the frame and
  boxes for attempt 0 (t) and attempt 1 (t + retry gap); the shim's `grab`/`detect` callbacks for
  `flood_capture_sample()` are built on it.
- `sim_box_from_accelerator(...)` — thin wrapper over `flood_box_from_accelerator()`.
- `sim_abi_version()`, `sim_sizeof_result()`, `sim_sizeof_score()` — checked by Python at load.

`sim_result_t` fields: `outcome` (UPLOAD/CLEAR/BLOCKED/HEARTBEAT/SKIPPED_NO_REF), `frames_taken`,
`has_reading`, `curb_correlation`, `severity`, `shift_dx`, `shift_dy`, `veto_usable`,
`veto_coverage`, `veto_worst_class`, `since_image`. Firmware struct changes surface as compile
errors in the shim, not as silent Python mismatches.

### Firmware sync

- On `floodsim` start (CLI or GUI), `core.py` compares mtimes of `FIRMWARE_DIR`'s
  `process_flood_detection/*` and `native/*` against the DLL; if newer, runs
  `cmake --build` (configuring first if needed).
- Build failure → hard stop with compiler output. `--no-rebuild` uses the existing DLL explicitly
  and records `dll_stale: true` in `run.json`.
- Every run records firmware `git rev-parse HEAD` and dirty flag.
- Device-glue behaviour (`flood_task.c`, `flood_frame_tap.c`, `flood_roi.c`) is reimplemented, with
  every mirrored value in `device_constants.h` annotated with its source file and line. Changes
  there require a manual update — accepted trade-off.

## Data flow

```
input → source.py (OpenCV decode, BGR→RGB, resize to 1920×1080)
      → sim_rgb_to_gray (C, firmware formula)                    → gray frame
boxes.py (edited CSV ▸ cache ▸ YOLO@640×480 → sim_box_from_accelerator) → flood_box_t[]
      → sim_shim → firmware flood_* code → result
```

- Non-1920×1080 input: resize, warn once; refuse non-16:9 unless `--allow-stretch`.
- Grayscale is computed in C with the firmware formula, **not** OpenCV `cvtColor` (different rounding
  could flip borderline correlations).

### Dry reference

Sources: `--reference <image>`, `--reference-at <time>` (frame of the input), or GUI "Set reference".
Built with `flood_reference_init()` under the current geometry. Geometry changes after that mark it
stale (mirrors the firmware's `geometry_version`): headless runs rebuild automatically and log it;
the GUI shows a banner with "Rebuild reference". If `flood_reference_init()` fails:
"geometry leaves no usable sampling window".

### Device mode

- Sample times `0, P, 2P …`; `--period` default **60 s** (`FLOOD_SAMPLE_PERIOD_MS`).
- Per sample: reference not ready → `SKIPPED_NO_REF`; else `flood_capture_sample()` where attempt 0
  is the frame at t and attempt 1 the frame at t + `--retry-gap` (default **1 s**). A frame that
  fails to decode makes `grab` return false, as a timed-out grab does on device.
- Capture state persists across samples → `HEARTBEAT` every 60th sample as on device.
- For `UPLOAD`/`HEARTBEAT`, write the composite the device would send to `uploads/<label>.jpg`:
  full frame ÷2 nearest-neighbour, 6-hex uppercase sequence label burned in top-left (mirrors
  `flood_roi_composite` + `flood_report_upload`; JPEG via OpenCV, not bit-identical to HW JPEG).

### Dense mode

Every Nth frame (`--every`, default 1): `sim_score()` — veto + curbcheck, no capture state, no retry,
no heartbeat.

### Manual mode (GUI)

One live device state. Actions on the current frame: **Sample** (full `flood_capture_sample`, retry
at t + gap, state advances, row added to samples table), **Score** (dense, state untouched),
**Set reference**, **Reset state**. Geometry and box edits apply to the next action.

### Still images

A single image is one frame at t = 0. A folder is a sequence ordered by filename, timed by `--fps`
or by a timestamp parsed from the filename when present.

## Outputs

### Run folder `out/<run-name>/`

- `run.json` — inputs (path, size, mtime), mode and parameters, geometry, reference source, firmware
  commit + dirty, DLL ABI version and staleness, box source (model, conf, cache file, or
  `no-detector`).
- `samples.csv` — one row per sample (device) or scored frame (dense), fields as in `sim_result_t` /
  `sim_score_t` plus `t_sec`, `frame_idx`, and `label` when labels are given.
- `summary.txt` — outcome counts, decode failures, and scoring (below).
- `uploads/` — device mode only.

### Box cache

- `<input>.boxes.csv` next to the input: `frame_idx, t_sec, x0, y0, x1, y1, conf, coco_class,
  flood_class`. Header comment records model, conf threshold, detector input size, input size + mtime;
  mismatch invalidates the cache.
- YOLO runs lazily, only on frames a run or the GUI needs; results accumulate per frame.
- Defaults: `yolo26n.pt`, conf 0.25, input 640×480, boxes scaled by the firmware's
  `flood_box_from_accelerator()`. COCO → `flood_class_t` mapping for person, bicycle, car,
  motorcycle, bus, truck; everything else `FLOOD_CLASS_OTHER`.
- Manual edits go to `<input>.boxes.edited.csv`, which overrides the cache for the frames it covers;
  the YOLO cache is never overwritten by edits.

### Labels and scoring (`--labels`)

- Format: `start_sec,end_sec,label` (video) or `file,label` (image folder); labels
  `DRY | WET | FLOODED | INDETERMINATE`.
- Positive = `FLOODED`; negative = `DRY`/`WET`; `INDETERMINATE` excluded; unlabelled samples counted
  separately. Predicted positive = `UPLOAD` (device) or `submerged` (dense).
- **`BLOCKED` is an abstention, never a negative** — reported as its own count and rate.
- `summary.txt` reports confusion counts, abstention rate, margin (max correlation of positives, min
  of negatives), and the rule-of-three 95 % upper bound on false-alarm rate when FP = 0.
- Gaps or overlaps in the labels file produce a warning.

### CLI

| Command | Purpose |
|---|---|
| `floodsim run <input> --mode device\|dense --reference … [--reference-at] [--geometry] [--labels] [--period] [--retry-gap] [--every] [--no-detector] [--no-rebuild] [--allow-stretch] [--out]` | headless run |
| `floodsim detect <input>` | pre-fill the box cache |
| `floodsim compare <runA> <runB>` | list samples whose outcome/severity differ or correlation differs by > 1e-6; exit 1 if any |
| `floodsim export <input> --out <dir> [--every N]` | gray `.pgm` frames (named `f<frame_idx>_t<ms>.pgm`, gray via the DLL's firmware formula) + `boxes.csv` for `flood_sim.exe` |
| `floodsim gui [<input>]` | GUI |

### `flood_sim.exe`

```
flood_sim.exe --reference dry.pgm --frames <dir|list> [--boxes boxes.csv] [--geometry geo.json]
              [--mode device|dense] [--period 60] [--retry-gap 1] [--every 1] [--fps 30]
```

Frame timestamps come from the `_t<ms>` filename suffix written by `floodsim export`, else from
`--fps`. C standard library only (PGM, CSV and minimal JSON parsing in-file). Emits one JSON line per
sample with the same field names as `samples.csv`. No video decoding or YOLO.

## GUI (PySide6, layout A)

- **Menu**: File (open video / image / folder), Reference, Geometry (load/save), Run (device /
  dense), Export run.
- **Left — tools**: Pan, Edit curb line, Edit lane line, Draw box (then class picker), Delete box;
  buttons Sample, Score, Set reference, Reset state.
- **Centre — frame** with toggleable overlays: curb line, lane line, sampled kerb strip + domain,
  align patch, boxes coloured by class with coverage % (edited boxes dashed), shift vector,
  reference comparison view.
- **Right — verdict and state**: outcome, severity, correlation vs threshold, shift, veto result,
  frames_taken; since_image / heartbeat, reference status (source, geometry version, stale banner),
  firmware commit + dirty.
- **Bottom — timeline**: transport controls and scrubber; correlation plot on the same time axis
  with threshold line, BLOCKED shading, device-sample markers, and a ground-truth strip when labels
  are loaded; clicking the plot seeks. Collapsible samples table.
- Seeking shows the cached result or scores the frame on demand (dense).
- Geometry editing drags line endpoints; strip redraws live; each edit bumps geometry version.
  Geometry JSON uses the firmware web API shape `{"curb":{"x0","y0","x1","y1"},"lane":{…}}`
  (`main/bsp_webserver/web_handle_flood_geometry.c`), so values can be pasted to/from a device.
  The device's persisted `flood_geometry.bin` is not read or written.
- Batch runs use the same runner as the CLI on a background thread with progress + cancel.
- **Threading**: all DLL calls are serialised on one worker thread (the firmware uses a global
  `g_flood_geometry` and is not re-entrant). YOLO runs on a background thread with progress.

## Error handling

| Situation | Behaviour |
|---|---|
| `FIRMWARE_DIR` missing | Stop; name the path and the override variable |
| Auto-rebuild fails | Stop with compiler output (dialog in GUI); never fall back silently |
| ABI / struct-size mismatch | Stop: "DLL ABI vX, wrapper expects vY — rebuild" |
| Frame decode failure | Skip + log + count; device mode treats as failed grab |
| Non-1920×1080 / non-16:9 | Resize + warn / refuse unless `--allow-stretch` |
| No `ultralytics` and no cache | Stop unless `--no-detector` (0 boxes, recorded in `run.json`) |
| `flood_reference_init()` false | Clear message, no run |
| Labels gaps / overlaps | Warn; unlabelled excluded from scoring and counted |

## Testing

1. **C / CTest**: `flood_sim.exe` on small synthetic PGMs (drawn kerb; kerb erased; box across kerb)
   → golden JSON. No Python needed.
2. **pytest units**: device loop (retry on blocked, heartbeat at 60th sample, skip before reference);
   gray conversion equals firmware formula; box-cache invalidation and edited-box precedence;
   label scoring incl. abstentions and rule-of-three; `compare` exit codes.
3. **DLL vs exe parity**: same inputs through both → identical results.
4. **Python-reference cross-check (optional)**: if `D:\git\MS500_Flooding_Detection` and footage are
   present, correlation at fixed shift vs `ms500/curbcheck.py` within 1e-5. Registration is not
   compared exactly (C uses direct search, Python uses phase correlation — documented in the firmware
   header). Skipped otherwise.
5. **GUI smoke (pytest-qt)**: open image → set reference → score → move a line → reference stale.

Automated tests use synthetic frames only; evaluating on the real footage requires fetching the
dataset in the reference repo.

## Out of scope

- Web UI, MQTT, real uploads, OTA, calibration persistence to SPIFFS.
- Emulating the IMX500 traffic model exactly (YOLO is an approximation of Stage-1 input).
- Compiling `flood_task.c` / FreeRTOS glue.
- Bit-identical JPEG output with the P4 hardware encoder.
- Linux/macOS support (not precluded; not tested).

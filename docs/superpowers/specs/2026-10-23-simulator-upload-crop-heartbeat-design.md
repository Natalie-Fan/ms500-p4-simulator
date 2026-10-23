# Upload Crop and Heartbeat Setting Design

Extends `2026-09-24-flood-simulator-design.md`. Brings the simulator in line with the October 2026
firmware (`NETWORK_P4_FLOOD_DETECTION` up to `9e4d4ac`), which:

- uploads a crop of the raw frame instead of the whole frame at half size, with no burned-in label
  (`flood_upload_roi.c`, `flood_upload.c`);
- names uploads `SN_flood[_hb]_YYYYMMDD_HHMMSS.jpg` and sends a JSON `i_description`
  (`flood_upload_meta.c`);
- takes the heartbeat interval from a Device Manager setting in minutes, 0 = off
  (`dm_flood_settings.c`, applied at `flood_task.c:342`).

Firmware code stays unmodified and is compiled as-is. Detection results do not change.

## Decisions

| Topic | Decision |
|---|---|
| Upload area | Lines-derived by default; an optional drawn area in the CLI and the GUI (draw, redraw, delete), as on the camera's web page |
| Heartbeat | Set in minutes (0–1440, default 60, 0 = off), converted to readings with the firmware formula and the run's sample period |
| Upload names | Firmware names and descriptions, produced by the firmware's own `flood_upload_meta.c`; time from the clip's real start or the file's modified time; serial number from `--sn` (default `SIM`) |
| Remaining gap | JPEG is encoded by OpenCV at quality 90, not by the P4 hardware encoder, so images are not byte-identical; crop, name and description are |

## 1. Native layer

### `native/CMakeLists.txt`

- `flood_upload_roi.c` is already in `FLOOD_SOURCES` (the glob over `process_flood_detection/`).
- Add `${FIRMWARE_DIR}/main/ai_process/flood_upload_meta.c` explicitly, with `main/ai_process` on
  its include path.
- Add `native/sdkconfig.h` containing only `#define CONFIG_DNN_TYPE_FLOOD 1`, so the file's
  `#ifdef CONFIG_DNN_TYPE_FLOOD` guard compiles its body. It is on the include path of
  `flood_upload_meta.c` only.
- `core.source_files()` (the rebuild check) includes `flood_upload_meta.c`, so editing it rebuilds
  the DLL.

### `native/sim_shim.h/.c`

`SIM_ABI_VERSION` goes from 4 to 5. New functions:

```c
/* drawn4 = {x,y,w,h} or NULL (derive from the lines). out8 = {x,y,w,h,step,source,out_w,out_h}.
 * source: 0 drawn, 1 lines, 2 full (flood_upload_src_t). Uses the current geometry. */
SIM_API void    sim_upload_roi(const int32_t *drawn4, int32_t *out8);
SIM_API int32_t sim_upload_rect_valid(const int32_t *rect4);
/* rgb24 is 1920x1080; out holds out_w*out_h*3 bytes. Calls flood_upload_roi_copy(). */
SIM_API void    sim_upload_crop(const uint8_t *rgb24, const int32_t *roi8, uint8_t *out);
/* Wrappers over flood_upload_filename()/flood_upload_description(); return the snprintf length. */
SIM_API int32_t sim_upload_filename(char *out, int32_t size, const char *sn, int32_t heartbeat,
                                    int32_t year, int32_t month, int32_t day,
                                    int32_t hour, int32_t minute, int32_t second);
SIM_API int32_t sim_upload_description(char *out, int32_t size, int32_t heartbeat, const char *id,
                                       float correlation, int32_t severity, int32_t slot,
                                       const int32_t *roi8, int64_t captured_at);
SIM_API void    sim_device_set_heartbeat(void *dev, int32_t every);   /* readings; 0 = off */
SIM_API int32_t sim_upload_max_pixels(void);
SIM_API int32_t sim_upload_min_side(void);
```

`sim_upload_roi` fills a `flood_rect_t` from `drawn4` and calls `flood_upload_roi_compute()`; the
firmware's validity check (`flood_upload_rect_valid()`) decides whether a drawn area is used, so an
invalid one falls back to the lines exactly as on the camera. `sim_upload_description` maps
`severity` through `flood_severity_name()`.

`sim_device_set_heartbeat` sets `heartbeat_every` on the device's `flood_capture_state_t`; the
device keeps it until set again (the firmware sets it before every reading; the simulator sets it
once per run or whenever the GUI field changes, which is equivalent).

`sim_upload_downscale()` and `SIM_UPLOAD_DOWNSCALE` are removed. `sim_heartbeat_every()` stays (the
firmware default, `FLOOD_HEARTBEAT_EVERY`).

### `native/device_constants.h`

- Remove `SIM_UPLOAD_DOWNSCALE`.
- Add a comment pointing at `flood_task.c:342` for the minutes-to-readings formula, and at
  `dm_flood_settings.h` for 0–1440 / default 60.

### `flood_sim.exe`

New option `--heartbeat-every N` (readings; default `FLOOD_HEARTBEAT_EVERY`; 0 = off). It writes no
upload images, so nothing else changes.

## 2. Python core (`floodsim/core.py`)

```python
@dataclass(frozen=True)
class UploadRoi:
    x: int; y: int; width: int; height: int
    step: int
    source: str          # "drawn" | "lines" | "full"
    out_width: int; out_height: int

Core.upload_roi(drawn: tuple[int, int, int, int] | None = None) -> UploadRoi
Core.upload_rect_valid(rect) -> bool
Core.crop(rgb: np.ndarray, roi: UploadRoi) -> np.ndarray          # (out_h, out_w, 3) uint8, RGB
Core.upload_filename(sn: str, heartbeat: bool, local: datetime) -> str
Core.upload_description(*, heartbeat, id, correlation, severity, slot, roi, captured_at) -> str
Core.upload_limits() -> tuple[int, int]                            # (min_side, max_pixels)
Device.set_heartbeat(readings: int) -> None
```

`Core.upload_downscale()` is removed. Output buffers are sized from the shim's returned lengths
(retry once with a larger buffer if the first call reports truncation).

New helper in `floodsim/modes.py`:

```python
def heartbeat_readings(minutes: int, period_s: float) -> int:
    """minutes * 60 / period, rounded down (flood_task.c:342); 0 stays 0 (off);
    any other value gives at least 1, so a period longer than the interval still sends heartbeats."""
```

`minutes` must be 0–1440; anything else is a `FloodSimError`.

## 3. Runs, CLI and output

### `floodsim run` options (device mode)

| Option | Meaning |
|---|---|
| `--heartbeat N` | Minutes, 0–1440, default 60, 0 = off |
| `--upload-area VALUE` | `x,y,w,h` in 1920×1080 coordinates, or a JSON file in the firmware's `GET /api/flood/upload_roi` shape (`{"configured": true, "roi": {...}}`; `configured: false` means none) |
| `--sn TEXT` | Serial number for upload names, default `SIM`; letters, digits, `-` and `_` only |

In dense mode these options are accepted and ignored with a one-line note (dense mode never
uploads). `--upload-area` that the firmware would reject is a warning, not an error: the run uses
the lines, as the camera would, and `run.json` records both.

### `UploadWriter` (`floodsim/modes.py`)

```python
UploadWriter(out_dir, core, *, drawn=None, sn="SIM", clock: Callable[[float], datetime])
UploadWriter.write(t_sec, rgb, result: DeviceResult, heartbeat: bool) -> str   # file name written
```

For each `upload` or `heartbeat` outcome:

1. `roi = core.upload_roi(drawn)` with the geometry in force at that reading.
2. `crop = core.crop(rgb, roi)`; encode with OpenCV, JPEG quality 90.
3. `id` = 6-hex sequence `000000, 000001, …` per run (as `flood_upload.c`).
4. `local = clock(t_sec)`; name from `core.upload_filename(sn, heartbeat, local)`. If the name is
   already used in this run, insert `_2`, `_3`, … before `.jpg`.
5. Write `uploads/<name>.jpg` and `uploads/<name>.json` (the description string, unchanged).

No label is drawn on the image.

### Clock

- Input is a `record_YYYYMMDD_HHMMSS.<ext>` clip or a folder of them: the clip start
  (`clips.clip_start`, already used by the timeline) plus the offset.
- Anything else: the input path's modified time plus the offset.
- `captured_at` in the description is that local time converted with this PC's time zone, matching
  the camera (local time in the name, UTC seconds in the description).

### Run folder

- `samples.csv` gains `upload_file` (empty when nothing was uploaded). `compare_runs` does not
  compare it.
- `run.json` gains `"heartbeat": {"minutes": N, "readings": R}`, `"upload_area": {x,y,w,h} | null`,
  `"upload_area_used": {UploadRoi fields}` (at the start of the run), and `"sn"`.

## 4. GUI

### Upload area (`floodsim/gui/upload_area.py`, new)

- `UploadAreaState`: the drawn area or `None`, load/save/delete to `<input>.upload_area.json`
  (firmware API shape), and `effective(core) -> UploadRoi`. Pure Python, unit-tested without Qt.
- Frame overlay: the area in use, orange; solid when drawn, dashed when from the lines or the whole
  frame. Follows line edits immediately.
- Menu **Upload area**: **Draw** (drag a rectangle; red while it breaks a rule — side under 32 px,
  outside the frame, over 518,400 px; Save / Cancel; drawing again replaces), **Delete** (back to
  the lines; disabled when nothing is drawn).
- Status panel: `Upload area: drawn | from lines | whole frame, W×H, full resolution | 1/N size`.

`main_window.py` only creates the menu and forwards mouse events while drawing; drawing state and
validation live in `upload_area.py`.

### Heartbeat

- **Heartbeat (minutes)** spin box in the device-state panel (0–1440, default 60).
- Manual **Sample** converts it with the device period (60 s) before each sample.
- The device-run dialog starts from it and converts with the dialog's period.
- **Reset state** keeps the setting and clears the counter.

### Export

**File ▸ Export run** uses the same `UploadWriter`, so names, sidecars and `upload_file` match the
CLI.

## 5. Docs

- `README.md`, `docs/HANDBOOK.md`: new options, upload naming and sidecar, GUI menu and field,
  remaining JPEG-encoder gap. Rebuild `docs/HANDBOOK.pdf` with `tools/build_handbook_pdf.py` when
  reportlab and matplotlib are available; otherwise leave it and say so.
- **Flood detection guide** `docs/flood-detection-guide.html`, identical in both repos
  (`NETWORK_P4_FLOOD_DETECTION/docs/` and `ms500-p4-simulator/docs/`): a single self-contained
  bilingual (English / 中文) page with a language switch, following `D:\git\aglaiasense-design`
  (Inter for text, Outfit for the title, AA teals `#0b7a67` light / `#1e7b69` dark, neutral grays,
  light default with a dark switch via `data-theme`, 6/10 px light and 4 px dark radii, sentence
  case, uppercase mono section labels, the logo embedded as a data URI, no emoji). Content: the
  current guide updated for the simulator changes. Republished to
  https://claude.ai/artifact/WomxpWr4yUY2yLo6qXfYSb.

## 6. Tests

| Where | What |
|---|---|
| `native/tests/test_shim.c` | default geometry gives 1360×360 step 1 from the lines; a valid drawn area is used at step 1; an invalid one falls back to the lines; crop copies the right pixels; file name and description match the firmware format; heartbeat 0 never reports `heartbeat`, 1 reports it on every dry reading |
| `tests/test_core.py` | `UploadRoi` fields, `crop` shape, name/description round-trip, ABI version 5 |
| `tests/test_modes.py` | `heartbeat_readings` (0, 60 min @ 60 s = 60, 60 min @ 5 s = 720, 1 min @ 120 s = 1, out of range); synthetic device run writes `uploads/*.jpg` + `.json`, sets `upload_file`, image size equals the area in use; name collision suffix |
| `tests/test_cli.py` | `--upload-area` both forms and bad input; `--heartbeat`; `--sn`; dense mode ignores them; `run.json` fields; `compare` of two runs of the same input passes |
| `tests/test_gui_session.py` | area save/reload/delete; line move changes a line-derived area; heartbeat field reaches manual Sample and device runs; Export run matches the CLI |
| existing | golden C tests and the full pytest suite stay green |

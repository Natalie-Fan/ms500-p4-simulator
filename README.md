# ms500-p4-simulator

Runs the MS500-P4 firmware's flood-detection C code — unmodified, compiled straight from
`D:\git\NETWORK_P4_FLOOD_DETECTION` — on image and video files instead of the camera.

Design: `docs/superpowers/specs/2026-09-24-flood-simulator-design.md`.
**How to use it:** [`docs/HANDBOOK.md`](docs/HANDBOOK.md), or the illustrated [`docs/HANDBOOK.pdf`](docs/HANDBOOK.pdf) (rebuild it after editing the handbook: `pip install reportlab matplotlib`, then `.venv/Scripts/python tools/build_handbook_pdf.py`).

## Setup (Windows, Git Bash)

Needs MinGW gcc, CMake ≥ 3.20, Ninja and Python 3.11 on PATH.

```bash
py -3.11 -m venv .venv
.venv/Scripts/python -m pip install -e ".[dev,onnx]"   # onnx: runs models/traffic.onnx (the device model)
```

The native build runs automatically the first time `floodsim` loads the core, and again whenever a
firmware or shim source is newer than `build/native/flood_core.dll`. To point at another firmware
checkout: `export FLOOD_FIRMWARE_DIR=/d/path/to/firmware`.

## Use

```bash
# device-faithful decisions (60 s samples, retry, heartbeat) + images the camera would upload
.venv/Scripts/floodsim run clip.avi --reference dry.jpg
# a camera's settings: heartbeat every 15 min, its drawn upload area, its serial number
.venv/Scripts/floodsim run clip.avi --reference dry.jpg --heartbeat 15 --upload-area 400,120,960,540 --sn MS500-H120-EP-2614-1001
# every frame: correlation / veto curve, scored against labels
.venv/Scripts/floodsim run clip.avi --mode dense --reference dry.jpg --labels clip.labels.csv
# regression: did a firmware edit change anything?
.venv/Scripts/floodsim compare out/before out/after
# pure C, no Python: export frames, then run the exe
.venv/Scripts/floodsim export clip.avi --reference dry.jpg --out out/exp
build/native/flood_sim.exe --reference out/exp/reference.pgm --frames out/exp/frames \
    --boxes out/exp/boxes.csv --mode dense
# lighting: several manual baselines, 5 s samples so a short clip shows the gallery learning
.venv/Scripts/floodsim run clip.avi --reference noon.jpg --reference night.jpg --period 5
# a day of recordings: every record_YYYYMMDD_HHMMSS.avi in the folder, at its real start time
.venv/Scripts/floodsim run day_folder/ --reference noon.jpg
```

References form the firmware's **reference gallery** (`flood_gallery.c`, up to 8): each frame is
scored against all of them and the best match counts. Device mode also learns references the way
the device does (confidently dry but not yet covered) and flags a run of unmatched samples; see the
`gallery_slot`, `admitted`, `notice` and `gallery_count` columns and `summary.txt`. Dense mode never
learns. On the command line every run starts from the manual references only. In the GUI a device
run also starts from them, then the session keeps the gallery it ended with, so later dense runs,
Score and the verdict use what was learned (Reset state drops it).

Uploads match the October 2026 firmware: each `upload` or `heartbeat` writes `uploads/<SN>_flood[_hb]_YYYYMMDD_HHMMSS.jpg`, the crop the camera sends (a drawn `--upload-area`, else derived from the curb/lane lines), and a `.json` beside it with the camera's description. Names and descriptions come from the firmware's own `flood_upload_meta.c`. The JPEG is OpenCV quality 90, so it is not byte-identical to the camera's. Times are the clip's real start (`record_YYYYMMDD_HHMMSS`) or the input's modified time, plus the video offset. `samples.csv` names each upload in `upload_file`. `--heartbeat` is in minutes (0 = off, default 60) and is converted to readings with `--period`, as the camera converts it with its 60 s period. Heartbeats follow video time, so on a short clip with a short `--period` use e.g. `--heartbeat 1` to see them. Runs made before this change at periods other than 60 s differ on heartbeat rows.

A folder of `record_YYYYMMDD_HHMMSS` videos opens as one timeline at the clips' real start times.
Device checks that fall between clips are skipped (counted in `summary.txt`), the gallery carries
from clip to clip, and each clip keeps its own `.boxes.csv`. `floodsim export` does not support
such folders yet.

Stage-1 boxes come from the device's traffic model at `models/traffic.onnx` (not in git;
copy it there). Without it or `onnxruntime`, pass `--no-detector` (no obstruction veto) or have a complete
`<input>.boxes.csv` cache from an earlier `floodsim detect`.

Labels from the reference dataset:
`python tools/truth_to_labels.py --truth ../MS500_Flooding_Detection/dataset/truth.csv --bursts ../MS500_Flooding_Detection/dataset/bursts.json --videos ../MS500_Flooding_Detection/Videos --out labels/`

## GUI

```bash
.venv/Scripts/python -m pip install -e ".[dev,gui]"
.venv/Scripts/floodsim gui clip.avi            # or an image, or a folder of images
```

Layout: tools (left) · frame with overlays (centre) · verdict and device state (right) ·
correlation-over-time plot aligned with the scrubber (bottom; click it to jump) · samples table.

- **Manual mode:** *Sample* runs the full firmware capture (retry on a frame 1 s later, heartbeat
  counter carries over, may learn a reference); *Score* scores this frame only; *Add reference*
  adds this frame to the gallery as a manual baseline; *Reset state* clears the heartbeat counter
  and forgets learned references. Reference ▸ Clear all references empties the gallery;
  View ▸ Show reference displays a manual reference (with overlays) until you move to a frame.
- **Editing:** drag curb/lane endpoints (the reference is then marked stale, as on the device, until
  you rebuild it); draw or delete obstruction boxes (saved to `<input>.boxes.edited.csv`, dashed).
- **Batch:** Run ▸ Device/Dense runs the same runners as the CLI (the device run dialog sets the
  sample period, default 60 s); File ▸ Export run writes the same run folder as `floodsim run`.
- **Upload area / heartbeat / serial number:** Upload area ▸ Draw (or the *Draw upload area* tool) and ▸ Delete, saved to `<input>.upload_area.json`. The orange box shows the area in use (dashed when derived from the lines). *Heartbeat* and *Serial number* sit in the State panel. The device run dialog starts from the heartbeat value.
- All simulator calls run on one background thread; a long batch run delays frame updates until it
  finishes or is cancelled.

## What is mirrored rather than compiled

`native/device_constants.h` lists every value copied from firmware device glue (`flood_task.c`,
`flood_frame_tap.c`, `flood_roi.h`, `flood_geometry_config.c`) with its source line. Re-check it
when those files change. The upload crop and names are compiled from `flood_upload_roi.c` and
`flood_upload_meta.c`. The heartbeat formula is mirrored from `flood_task.c:342`.

`flood_sim.exe` also takes `--heartbeat-every N` (readings between heartbeats, 0 = off). To match a Python device run, pass the `heartbeat.readings` value from its `run.json`.

## Tests

```bash
ctest --test-dir build/native --output-on-failure
.venv/Scripts/python -m pytest
```

# ms500-p4-simulator

Runs the MS500-P4 firmware's flood-detection C code — unmodified, compiled straight from
`D:\git\NETWORK_P4_FLOOD_DETECTION` — on image and video files instead of the camera.

Design: `docs/superpowers/specs/2026-09-24-flood-simulator-design.md`.
**How to use it:** [`docs/HANDBOOK.md`](docs/HANDBOOK.md), or the illustrated [`docs/HANDBOOK.pdf`](docs/HANDBOOK.pdf) (rebuild it after editing the handbook: `pip install reportlab matplotlib`, then `.venv/Scripts/python tools/build_handbook_pdf.py`).

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

## GUI

```bash
.venv/Scripts/python -m pip install -e ".[dev,gui]"
.venv/Scripts/floodsim gui clip.avi            # or an image, or a folder of images
```

Layout: tools (left) · frame with overlays (centre) · verdict and device state (right) ·
correlation-over-time plot aligned with the scrubber (bottom; click it to jump) · samples table.

- **Manual mode:** *Sample* runs the full firmware capture (retry on a frame 1 s later, heartbeat
  counter carries over); *Score* scores this frame only; *Set reference* makes this frame the dry
  reference; *Reset state* clears the heartbeat counter.
- **Editing:** drag curb/lane endpoints (the reference is then marked stale, as on the device, until
  you rebuild it); draw or delete obstruction boxes (saved to `<input>.boxes.edited.csv`, dashed).
- **Batch:** Run ▸ Device/Dense runs the same runners as the CLI; File ▸ Export run writes the same
  run folder as `floodsim run`.
- All simulator calls run on one background thread; a long batch run delays frame updates until it
  finishes or is cancelled.

## What is mirrored rather than compiled

`native/device_constants.h` lists every value copied from firmware device glue (`flood_task.c`,
`flood_frame_tap.c`, `flood_roi.h`, `flood_geometry_config.c`) with its source line. Re-check it
when those files change.

## Tests

```bash
ctest --test-dir build/native --output-on-failure
.venv/Scripts/python -m pytest
```

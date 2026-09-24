# ms500-p4-similator

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

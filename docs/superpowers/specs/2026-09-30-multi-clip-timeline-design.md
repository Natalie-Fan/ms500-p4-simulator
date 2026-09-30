# Multi-Clip Timeline Design

Extends `2026-09-24-flood-simulator-design.md`. Simulator repo only: the firmware flood code is
unchanged and still compiled as-is; only how frames are fed to it changes.

## Motivation

The site records many short clips named `record_YYYYMMDD_HHMMSS.avi` over a day. Testing the
reference gallery's learning and lighting behaviour needs a device run across a real day, but the
simulator opens one input at a time, and a folder is only read as images (a folder of videos fails
with "no image files", `floodsim/source.py:193-197`). Joining the clips with ffmpeg loses the real
time gaps between them.

## Understanding

Stated by the user:

- Open a folder of `record_YYYYMMDD_HHMMSS.avi` clips as one timeline, each clip at its real start
  time (approach "A: real clock time from the file names"), and run one device run across it.
- All clips are named `record_YYYYMMDD_HHMMSS.avi`.
- Device checks that fall in a gap with no footage are skipped, with no rows.
- Each clip's own obstruction-box cache is reused.
- `floodsim export` / `flood_sim.exe` do not need to support folders of clips yet.

Assumed:

- Clips normally do not overlap, and are 16:9 like current recordings (other sizes follow the
  existing `Normalizer` rules).
- File names are local clock time; no time-zone handling.
- The clips are not labelled; per-clip label loading is out of scope.

Success criteria:

1. A device run checks every `period` seconds of real time from the first clip's start to the last
   frame; checks inside footage run exactly as today; checks in gaps produce no rows.
2. A reference learned in one clip is used in later clips (one continuous device and gallery).
3. Each clip's `.boxes.csv` / `.boxes.edited.csv` is shared between folder mode and opening that
   clip alone.
4. Single-video and image-folder behaviour is unchanged, including the goldens and every existing test.

## Architecture

A new `FrameSource` subclass, `MultiClipSource`, presents the folder as one input with one
continuous frame numbering and one timeline. Runners, GUI, labels and reports keep talking to "a
source". Two additions to the source contract let the runners and box layer handle clips:

- `has_footage(t) -> bool`
- `clip_of(idx) -> (clip_index, local_idx, local_t)`

A box router, `ClipBoxes`, gives each clip its own existing `BoxProvider`/`BoxStore`. The device
runner skips checks where `has_footage` is false.

## Components

### 1. `MultiClipSource` (`floodsim/source.py`, or `floodsim/clips.py` if `source.py` would pass ~500 lines)

**When it applies.** `open_source(path)` on a **directory**:

- only image files → `ImageSequence`, as today;
- only video files (`.avi`, `.mp4`, `.mkv`, `.mov`) → `MultiClipSource`;
- both → `FloodSimError("<folder>: contains both videos and images; put them in separate folders")`;
- neither → the existing "no image files" error, reworded to "no image or video files".

A single file, image or video, opens exactly as today.

**Names.** Every video must match `^record_(\d{8})_(\d{6})\.[A-Za-z0-9]+$` and parse as a valid
date-time (`%Y%m%d%H%M%S`). Otherwise raise `FloodSimError` listing every offending file name.
Two clips with the same start time raise `FloodSimError` naming both.

**Timeline.**

- Each clip's start comes from its name. `t0` is the earliest start. `offset(clip)` is
  `(start - t0).total_seconds()`. Dates carry across midnight and across days.
- Each clip is a `VideoSource` (existing class: its own fps, frame count by walking, local times in
  whole milliseconds). A global frame's time is `offset + local_t`, rounded to whole milliseconds as
  `VideoSource` does.
- Frames are numbered continuously across clips in start-time order.
- **Overlap:** if a clip's frames reach or pass the next clip's start, drop that clip's frames with
  `local_t + offset >= next_offset`, and log a warning naming both clips. If that leaves a clip with
  no frames, raise `FloodSimError` naming it.
- A clip that cannot be opened or has no decodable frames: `FloodSimError` naming the clip (from
  `VideoSource`). No partial timeline.

**`has_footage(t)`** is true iff for some clip, `first_t <= t < last_t + 1/fps`. `first_t` and
`last_t` are that clip's first and last kept global frame times; `fps` is its frame rate.

**`index_at(t)`** returns the latest frame at or before `t` when `has_footage(t)`, else `None`. A
check in a gap can never be handed the previous clip's last frame.

**`read_rgb(i)`** delegates to the clip's `VideoSource` with the local index. At most 2 clips hold
an open `cv2.VideoCapture` at a time (least-recently-used released). `VideoSource` gains a `close()`
that releases its capture and clears its cache; the next read reopens lazily. Frame counts are taken
once at open, with an INFO log per clip ("counting frames: record_… (3/12)").

**`frame_name(i)`** returns `"<clip file name>#<local_idx>"`.

**`fingerprint()`** returns `{"name": <folder name>, "clips": [<each clip's VideoSource.fingerprint()>]}`.

**`clock(t) -> datetime`** returns `t0 + timedelta(seconds=t)`. `start_clock` is `t0`.

`duration` is the last frame's time, as for other sources.

### 2. Device runner (`floodsim/modes.py`)

`run_device`:

- The check grid is unchanged: `t = k * period`, `k = 0 .. floor(duration / period)`.
- Before sampling, if the source has `has_footage` and `has_footage(t)` is false: skip. No row, no
  decode, no `Device.sample()` call, so device state and gallery are untouched. Increment
  `RunOutput.skipped_no_footage`.
- The `sample` column keeps `k`, so gaps show as jumps in sample numbers.
- The retry at `t + retry_gap` goes through `index_at` as today. In a gap it returns `None`, a
  failed grab to the firmware. Documented edge case: an obstructed check in the last `retry_gap`
  seconds of a clip comes out `blocked`.
- Sources without `has_footage` (existing ones) behave exactly as today.

`RunOutput` gains `skipped_no_footage: int = 0`.

`run_dense` needs no change: it already iterates frame indices.

### 3. Reporting (`floodsim/report.py`, `floodsim/cli.py`, `floodsim/gui/session.py`)

- `summarize()` adds `checks skipped (no footage): N` only when N > 0. It takes the count as a new
  keyword argument, default 0, so existing outputs are byte-identical.
- `make_run_info()` adds `"start_clock": <t0 ISO 8601>` to `run.json` when the source has one.
  `input` already carries the fingerprint, which now lists every clip.
- GUI `run_batch` result gains `skipped_no_footage`. The status bar appends
  `; N check(s) skipped (no footage)` when N > 0.

### 4. Box routing: `ClipBoxes` (`floodsim/boxes.py`)

Same surface as `BoxProvider` as used by callers: `cached_for(idx, t, load_rgb)`,
`boxes_for(idx, t, load_rgb)`, `description`, `detector`, `store`.

- Maps a global `idx` to `(clip, local_idx, local_t)` via `source.clip_of(idx)`, and delegates to
  that clip's `BoxProvider(core, BoxStore(clip.path, cache_meta(clip.fingerprint(), model, conf)),
  detector)`. That is exactly the store `make_box_provider` builds for the clip opened alone, so the
  cache files and their meta are interchangeable.
- Clip providers are created lazily on first use. The detector is loaded once and shared.
- `store` is a facade with the calls the GUI uses (`set_edited`, `clear_edited`, `is_edited`,
  `has_cache`, `has_edits`, `edited_path`), each routed to the clip's store with the local index and
  local time.
- `description` = `{"kind": "clips", "detector": <bool>, "clips": [<each touched clip's provider description>]}`.
- `make_box_provider(core, source, ...)` returns a `ClipBoxes` when the source is a
  `MultiClipSource`. The GUI's no-detector fallback builds the same router with `detector=None`.
- Stale-cache handling stays per clip (existing `BoxStore` behaviour).
- `floodsim detect <folder>` fills every clip's cache through the router (no code beyond the above).

### 5. CLI (`floodsim/cli.py`)

- `run` and `detect` accept a folder of clips via `open_source`.
- `--reference-at` takes seconds on the combined timeline. At a gap time it fails with the existing
  "no readable frame at … s" error.
- `export` on a `MultiClipSource` raises
  `FloodSimError("export does not support folders of clips; export a single clip")`.

### 6. GUI (`floodsim/gui/`)

- File ▸ Open folder… opens a folder of clips as one timeline. While opening, the status bar shows
  `opening <N> clips…`; afterwards
  `<N> clips, <HH:MM>–<HH:MM>, <frames> frames`.
- Timeline (`timeline.py`): when the source has `start_clock`, the time axis shows clock time
  (`HH:MM`, adding the date when the timeline spans more than one day), and gaps (the complement of
  `has_footage` between the first and last frame) are shaded. Clicking a gap seeks the nearest frame
  that has footage.
- Scrubber readout: `<HH:MM:SS> · <clip name> · frame <i>/<N>`.
- Verdict panel, device rows: `device sample #<k> at <HH:MM:SS> (t = <t> s)`.
- Everything else is unchanged (Sample, Score, references, Show reference, boxes, geometry, device
  and dense runs, gallery adoption, Export run).

The session exposes `start_clock`, the gap list and `clock(t)` to the window through `open()`'s
return value, so the widgets stay Qt-only and do not reach into the source.

## Errors

| Situation | Behaviour |
|---|---|
| Video name not `record_YYYYMMDD_HHMMSS` | Refuse to open; list the offending files. |
| Videos and images in one folder | Refuse to open. |
| Clip cannot be opened / no decodable frames | Refuse to open; name the clip. |
| Two clips with the same start | Refuse to open; name both. |
| Clips overlap | Open; drop the earlier clip's overlapping frames; warn naming both. |
| Frame fails to decode mid-clip | As today: failed frame read; a failed grab for a check. |
| `floodsim export` on a folder of clips | Stop with the message above. |

## Testing

Simulator repo, pytest. The fixtures write tiny synthetic clips at test time: 3-5 frames each, MJPEG
`.avi` via `cv2.VideoWriter`, 1920×1080, named `record_…`, built from `tests/synth.py` dry, dusk
and flooded frames. No real footage.

- **Timeline:** offsets from names, including a clip after midnight; continuous numbering;
  timestamps; `has_footage` inside, at both edges and in a gap; `index_at` returns `None` in a gap;
  overlap truncation with the warning; `frame_name`; `fingerprint`; `clock`.
- **Device run:** no rows for gap checks; `skipped_no_footage` and the summary line; `sample`
  numbers keep grid positions; a dusk reference learned in clip 1 is matched in clip 2
  (`gallery_source == "auto"`); no row's `frame_idx` belongs to a clip whose footage does not cover
  its `t_sec`.
- **Dense run:** all frames of all clips, in order.
- **Boxes:** a folder run writes each clip's own `.boxes.csv`; opening that clip alone reuses it
  with no detection; an edit in folder mode appears in single-clip mode. The detector is a stub
  that counts calls.
- **Errors:** every row of the table.
- **GUI session:** open a folder of clips, device run, the adopted gallery, and the open() info
  (`start_clock`, gaps). A light widget test for the clock-time axis labels and gap shading.
- **Regression:** all existing C and Python tests pass unchanged.

## Documentation

README note; handbook section (opening a day of clips, gaps, the export limit, the last-second-retry
edge case); handbook PDF rebuild; one-line entry in the firmware repo's `docs/task.md`.

## Out of scope

- `floodsim export` / `flood_sim.exe` for folders of clips.
- Per-clip label files (a single labels CSV on the combined timeline still works).
- Time zones, and names other than `record_YYYYMMDD_HHMMSS`.
- Any firmware change.

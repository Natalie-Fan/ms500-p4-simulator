# Multi-Clip Timeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Open a folder of `record_YYYYMMDD_HHMMSS` video clips as one timeline, each clip at its real start time, so one device run spans a whole day; gap checks are skipped and each clip's box cache is reused.

**Architecture:** A new `MultiClipSource` (`floodsim/clips.py`) presents the clips as one `FrameSource` with continuous frame numbers and seconds since the first clip's start, plus `has_footage(t)` and `clip_of(idx)`. `run_device` skips checks where `has_footage` is false. A `ClipBoxes` router (`floodsim/boxes.py`) gives each clip its own existing `BoxProvider`/`BoxStore`. The GUI shows clock time, shaded gaps and the current clip.

**Tech Stack:** Python 3.11, OpenCV (`cv2.VideoCapture`/`VideoWriter`, FFV1 codec for lossless test clips), PySide6 + pyqtgraph (GUI), pytest (+ pytest-qt).

**Spec:** `docs/superpowers/specs/2026-09-30-multi-clip-timeline-design.md` (simulator repo). Read it first.

**Repo:** `D:\git\ms500-p4-simulator`, branch `main` (the user commits there). Run everything from Git Bash in that folder. Python: `.venv/Scripts/python`. The firmware repo `D:\git\NETWORK_P4_FLOOD_DETECTION` is only touched in Task 5 (one `docs/task.md` line); never run `idf.py`.

## Global Constraints

- Simulator repo only; no firmware code change.
- Clip names: `^record_(\d{8})_(\d{6})\.[A-Za-z0-9]+$`, parsed with `%Y%m%d%H%M%S`, local clock time, no time zones.
- Video extensions: `.avi`, `.mp4`, `.mkv`, `.mov`.
- Time 0 = earliest clip start; frame time = clip offset + local time, rounded to whole milliseconds.
- `has_footage(t)`: `first_t <= t < last_t + 1/fps` for some clip.
- At most 2 clips hold an open `cv2.VideoCapture` at once.
- Gap checks: no row, no decode, no `Device.sample()`; counted in `RunOutput.skipped_no_footage`.
- `summarize()` prints `checks skipped (no footage): N` only when N > 0; single-video and image-folder outputs stay byte-identical.
- `floodsim export` on a folder of clips: `FloodSimError("export does not support folders of clips; export a single clip")`.
- Every existing C and Python test passes unchanged, except the one message change "no image files" → "no image or video files" (no test asserts it today).
- Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **Clips with different frame rates** (a 1 fps clip next to a 2 fps clip): each clip's own frame interval sets its footage end and its timestamps. Pinned by `test_clips_with_different_frame_rates` in Task 1.
2. **An obstructed check in a clip's last second**: its retry lands in the gap, finds no frame, and the check is `blocked` (the documented edge case), never reusing another clip's frame. Pinned by `test_retry_past_clip_end_is_blocked` in Task 3.
3. **Labels keyed by frame file name** (the `Labels.files` form) against clip frame names `record_…avi#i`: they simply do not match, while interval labels on the combined timeline work. Not tested (labels are out of scope); the reviewer checks that `label_for(t, "record_…avi#3")` cannot raise.
4. **The GUI's no-detector fallback** must build a `ClipBoxes` router, not a single `BoxStore` on the folder (which would write `<folder>.boxes.csv`). Pinned by `test_session_opens_folder_of_clips` in Task 4 (`no_detector=True` takes the fallback path, and the test asserts no `<folder>.boxes.csv` is written).
5. **Many clips**: reading across clips must never hold more than 2 open captures. Pinned by `test_at_most_two_clips_open` in Task 1.

---

## File Structure

| File | Change | Responsibility |
|---|---|---|
| `floodsim/source.py` | modify | `VIDEO_EXTS`; `VideoSource.fps` + `close()` + lazy reopen; directory dispatch in `open_source`. |
| `floodsim/clips.py` | create | `MultiClipSource`: timeline, footage, clip mapping, capture LRU. |
| `floodsim/boxes.py` | modify | `ClipBoxes` router + `_ClipStores` facade; `make_box_provider` returns a router for clips. |
| `floodsim/modes.py` | modify | `RunOutput.skipped_no_footage`; gap skip in `run_device`. |
| `floodsim/report.py` | modify | `summarize(..., skipped_no_footage=0)`; `start_clock` in `run.json`. |
| `floodsim/cli.py` | modify | pass the skipped count; refuse `export` on clips. |
| `floodsim/gui/session.py` | modify | clip-aware open info and no-detector fallback; skipped count in results and export. |
| `floodsim/gui/timeline.py` | modify | `ClockAxis`, `set_clock()`, gap shading, clip readout, nearest-frame seek in gaps. |
| `floodsim/gui/panels.py` | modify | `VerdictPanel.set_start_clock()`, clock time in device rows. |
| `floodsim/gui/main_window.py` | modify | wire clock/gaps/clips on open; opening and run status text. |
| `tests/synth.py` | modify | `write_clip()`: lossless FFV1 `record_….avi`. |
| `tests/test_clips.py` | create | Task 1 tests. |
| `tests/test_boxes.py`, `tests/test_modes.py`, `tests/test_report.py`, `tests/test_cli.py`, `tests/test_gui_session.py`, `tests/gui/test_timeline.py`, `tests/gui/test_panels.py` | modify | new tests per task. |
| `README.md`, `docs/HANDBOOK.md`, `docs/HANDBOOK.pdf` | modify | Task 5. |

---

### Task 1: `MultiClipSource` and directory dispatch

**Files:**
- Modify: `floodsim/source.py` (add `VIDEO_EXTS` near `IMAGE_EXTS`; `VideoSource.__init__`, `read_rgb`, new `close()`; `open_source`)
- Create: `floodsim/clips.py`
- Modify: `tests/synth.py` (add `write_clip`, `import cv2`)
- Create: `tests/test_clips.py`

**Interfaces:**
- Consumes: `FrameSource`, `Normalizer`, `VideoSource(path, fps, normalizer)` (`floodsim/source.py`); `dry_gray()`, `dusk_gray()`, `to_rgb()` (`tests/synth.py`).
- Produces:
  - `floodsim.source.VIDEO_EXTS: set[str]`
  - `VideoSource.fps: float`, `VideoSource.close() -> None`
  - `floodsim.clips.clip_start(path: Path) -> datetime | None`
  - `class MultiClipSource(FrameSource)`: `__init__(path: Path, files: list[Path], fps: float | None, normalizer: Normalizer)`; attributes `path`, `n_frames`, `start_clock: datetime`; methods `timestamp(i)`, `read_rgb(i)`, `frame_name(i) -> str`, `fingerprint() -> dict`, `index_at(t) -> int | None`, `has_footage(t: float) -> bool`, `clip_of(idx: int) -> tuple[int, int, float]` (clip index, local index, local time), `clip_source(ci: int) -> VideoSource`, `clip_bounds() -> list[tuple[int, str]]` (first global index, file name), `gaps() -> list[tuple[float, float]]`, `clock(t: float) -> datetime`
  - `tests.synth.write_clip(folder: Path, start: str, grays: list[np.ndarray], fps: float = 1.0) -> Path`

- [ ] **Step 1: Add `write_clip` to `tests/synth.py`**

Add `import cv2` with the other imports, then append:

```python
def write_clip(folder: Path, start: str, grays, fps: float = 1.0) -> Path:
    """Lossless (FFV1) clip named record_<start>.avi, e.g. start="20260618_164320". Gray in all
    three channels, so the firmware gray formula returns each frame exactly."""
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"record_{start}.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"FFV1"), fps, (FRAME_W, FRAME_H))
    assert writer.isOpened(), "this OpenCV build cannot write FFV1"
    for gray in grays:
        writer.write(np.repeat(gray[:, :, None], 3, axis=2))
    writer.release()
    return path
```

- [ ] **Step 2: Write the failing tests in `tests/test_clips.py`**

```python
import datetime as dt
import logging
import shutil

import numpy as np
import pytest

from floodsim.clips import MultiClipSource
from floodsim.errors import FloodSimError
from floodsim.source import open_source, write_image
from tests.synth import dry_gray, dusk_gray, to_rgb, write_clip


def _day(tmp_path):
    """23:59:50 (3 dry frames, t 0..2) and 00:05:00 the next day (2 dusk frames, t 310..311)."""
    folder = tmp_path / "day"
    write_clip(folder, "20260618_235950", [dry_gray()] * 3)
    write_clip(folder, "20260619_000500", [dusk_gray()] * 2)
    return folder


def test_folder_of_clips_is_one_timeline(tmp_path):
    src = open_source(_day(tmp_path))
    assert isinstance(src, MultiClipSource)
    assert src.n_frames == 5
    assert [src.timestamp(i) for i in range(5)] == [0.0, 1.0, 2.0, 310.0, 311.0]
    assert src.start_clock == dt.datetime(2026, 6, 18, 23, 59, 50)
    assert src.clock(310.0) == dt.datetime(2026, 6, 19, 0, 5, 0)
    assert src.frame_name(3) == "record_20260619_000500.avi#0"
    assert src.clip_of(4) == (1, 1, 1.0)
    assert np.array_equal(src.read_rgb(3), to_rgb(dusk_gray()))
    assert np.array_equal(src.read_rgb(0), to_rgb(dry_gray()))
    assert [c["name"] for c in src.fingerprint()["clips"]] == [
        "record_20260618_235950.avi", "record_20260619_000500.avi"]
    assert src.clip_bounds() == [(0, "record_20260618_235950.avi"), (3, "record_20260619_000500.avi")]


def test_footage_and_gaps(tmp_path):
    src = open_source(_day(tmp_path))
    assert src.has_footage(0.0) and src.has_footage(2.9)
    assert not src.has_footage(3.0) and not src.has_footage(100.0) and not src.has_footage(-1.0)
    assert src.has_footage(310.0) and src.has_footage(311.5) and not src.has_footage(312.0)
    assert src.index_at(2.9) == 2 and src.index_at(100.0) is None and src.index_at(310.5) == 3
    assert src.gaps() == [(3.0, 310.0)]


def test_clips_with_different_frame_rates(tmp_path):
    folder = tmp_path / "fps"
    write_clip(folder, "20260618_120000", [dry_gray()] * 2, fps=2.0)   # t 0, 0.5; footage to 1.0
    write_clip(folder, "20260618_120010", [dry_gray()] * 2, fps=1.0)   # t 10, 11; footage to 12.0
    src = open_source(folder)
    assert [src.timestamp(i) for i in range(4)] == [0.0, 0.5, 10.0, 11.0]
    assert src.has_footage(0.9) and not src.has_footage(1.0)
    assert src.has_footage(11.9) and not src.has_footage(12.0)


def test_overlap_drops_the_earlier_clips_frames(tmp_path, caplog):
    folder = tmp_path / "overlap"
    write_clip(folder, "20260618_120000", [dry_gray()] * 5)    # t 0..4
    write_clip(folder, "20260618_120003", [dusk_gray()] * 2)   # starts at t 3
    with caplog.at_level(logging.WARNING):
        src = open_source(folder)
    assert [src.timestamp(i) for i in range(src.n_frames)] == [0.0, 1.0, 2.0, 3.0, 4.0]
    assert src.frame_name(3) == "record_20260618_120003.avi#0"
    assert "record_20260618_120000.avi" in caplog.text and "overlaps" in caplog.text


def test_at_most_two_clips_open(tmp_path):
    folder = tmp_path / "many"
    for minute in range(4):
        write_clip(folder, f"20260618_12{minute:02d}00", [dry_gray()] * 2)
    src = open_source(folder)
    for idx in (0, 2, 4, 6, 0, 7, 3):
        assert np.array_equal(src.read_rgb(idx), to_rgb(dry_gray()))
        assert sum(src.clip_source(i)._cap is not None for i in range(4)) <= 2


def test_bad_names_are_listed(tmp_path):
    folder = tmp_path / "bad"
    write_clip(folder, "20260618_120000", [dry_gray()])
    write_clip(folder, "20260618_120100", [dry_gray()]).rename(folder / "cam1.avi")
    write_clip(folder, "20260618_120200", [dry_gray()]).rename(folder / "record_20261399_120200.avi")
    with pytest.raises(FloodSimError) as e:
        open_source(folder)
    assert "cam1.avi" in str(e.value) and "record_20261399_120200.avi" in str(e.value)


def test_same_start_rejected(tmp_path):
    folder = tmp_path / "same"
    clip = write_clip(folder, "20260618_120000", [dry_gray()])
    shutil.copy(clip, folder / "record_20260618_120000.mkv")
    with pytest.raises(FloodSimError, match="same start time"):
        open_source(folder)


def test_videos_and_images_together_rejected(tmp_path):
    folder = tmp_path / "mixed"
    write_clip(folder, "20260618_120000", [dry_gray()])
    write_image(folder / "a.png", to_rgb(dry_gray()))
    with pytest.raises(FloodSimError, match="both videos and images"):
        open_source(folder)


def test_empty_folder_rejected(tmp_path):
    (tmp_path / "empty").mkdir()
    with pytest.raises(FloodSimError, match="no image or video files"):
        open_source(tmp_path / "empty")


def test_broken_clip_named(tmp_path):
    folder = tmp_path / "broken"
    write_clip(folder, "20260618_120000", [dry_gray()])
    (folder / "record_20260618_120100.avi").write_bytes(b"not a video")
    with pytest.raises(FloodSimError, match="record_20260618_120100.avi"):
        open_source(folder)
```

- [ ] **Step 3: Run them and watch them fail**

Run: `.venv/Scripts/python -m pytest -q tests/test_clips.py`
Expected: FAIL at collection with `ModuleNotFoundError: No module named 'floodsim.clips'`.

- [ ] **Step 4: Change `floodsim/source.py`**

Next to `IMAGE_EXTS`:

```python
VIDEO_EXTS = {".avi", ".mp4", ".mkv", ".mov"}
```

In `VideoSource.__init__`, right after the `if not fps or fps <= 0:` check, add `self.fps = float(fps)`.

Add to `VideoSource` (after `_open`):

```python
    def close(self) -> None:
        """Release the capture and cached frames; the next read reopens lazily."""
        if self._cap is not None:
            self._cap.release()
        self._cap = None
        self._pos = 0
        self._cache.clear()
```

At the top of `VideoSource.read_rgb`, after the cache lookup and before `if idx < self._pos:`, add:

```python
        if self._cap is None:
            self._cap = self._open()
            self._pos = 0
```

Replace the directory branch of `open_source`:

```python
    if path.is_dir():
        entries = [f for f in path.iterdir() if f.is_file()]
        images = sorted(f for f in entries if f.suffix.lower() in IMAGE_EXTS)
        videos = sorted(f for f in entries if f.suffix.lower() in VIDEO_EXTS)
        if images and videos:
            raise FloodSimError(f"{path}: contains both videos and images; put them in separate folders")
        if videos:
            from .clips import MultiClipSource   # clips imports this module
            src: FrameSource = MultiClipSource(path, videos, fps, normalizer)
        elif images:
            src = ImageSequence(path, images, fps, normalizer)
        else:
            raise FloodSimError(f"{path}: no image or video files")
```

- [ ] **Step 5: Create `floodsim/clips.py`**

```python
"""A folder of record_YYYYMMDD_HHMMSS clips as one timeline, each clip at its real start time."""
from __future__ import annotations

import bisect
import datetime as dt
import logging
import re
from pathlib import Path

import numpy as np

from .errors import FloodSimError
from .source import FrameSource, Normalizer, VideoSource

_RECORD = re.compile(r"^record_(\d{8})_(\d{6})\.[A-Za-z0-9]+$")
_MAX_OPEN = 2   # clips holding an open VideoCapture at once
log = logging.getLogger(__name__)


def clip_start(path: Path) -> dt.datetime | None:
    """Local start time from record_YYYYMMDD_HHMMSS.<ext>, or None if the name does not match."""
    m = _RECORD.match(path.name)
    if not m:
        return None
    try:
        return dt.datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S")
    except ValueError:
        return None


class MultiClipSource(FrameSource):
    """Frames numbered continuously across clips; times are seconds since the first clip's start."""

    def __init__(self, path: Path, files: list[Path], fps: float | None, normalizer: Normalizer):
        self.path = path
        starts = {f: clip_start(f) for f in files}
        bad = sorted(f.name for f, s in starts.items() if s is None)
        if bad:
            raise FloodSimError(f"{path}: clip names must be record_YYYYMMDD_HHMMSS.<ext>; "
                                f"not: {', '.join(bad)}")
        ordered = sorted(files, key=lambda f: starts[f])
        for a, b in zip(ordered, ordered[1:]):
            if starts[a] == starts[b]:
                raise FloodSimError(f"{path}: {a.name} and {b.name} have the same start time")
        self.start_clock = starts[ordered[0]]

        self._clips: list[VideoSource] = []
        offsets: list[float] = []
        for n, f in enumerate(ordered, 1):
            log.info("counting frames: %s (%d/%d)", f.name, n, len(ordered))
            clip = VideoSource(f, fps, normalizer)
            clip.close()
            self._clips.append(clip)
            offsets.append((starts[f] - self.start_clock).total_seconds())

        self._times: list[float] = []
        self._first: list[int] = []          # global index of each clip's first frame
        self._spans: list[tuple[float, float]] = []   # footage [start, end) per clip
        for i, clip in enumerate(self._clips):
            times = [round((offsets[i] + clip.timestamp(j)) * 1000) / 1000 for j in range(clip.n_frames)]
            if i + 1 < len(self._clips) and times[-1] >= offsets[i + 1]:
                kept = bisect.bisect_left(times, offsets[i + 1])   # >= 1: starts are distinct and sorted
                log.warning("%s overlaps %s; dropping its last %d frame(s)",
                            clip.path.name, self._clips[i + 1].path.name, len(times) - kept)
                times = times[:kept]
            self._first.append(len(self._times))
            self._times.extend(times)
            self._spans.append((times[0], times[-1] + 1.0 / clip.fps))
        self.n_frames = len(self._times)
        self._span_starts = [s for s, _ in self._spans]
        self._open: list[int] = []           # clip indices with an open capture, oldest first

    # ---------------------------------------------------------------- lookup
    def clip_of(self, idx: int) -> tuple[int, int, float]:
        ci = bisect.bisect_right(self._first, idx) - 1
        local = idx - self._first[ci]
        return ci, local, self._clips[ci].timestamp(local)

    def clip_source(self, ci: int) -> VideoSource:
        return self._clips[ci]

    def clip_bounds(self) -> list[tuple[int, str]]:
        return [(first, clip.path.name) for first, clip in zip(self._first, self._clips)]

    def has_footage(self, t: float) -> bool:
        ci = bisect.bisect_right(self._span_starts, t + 1e-9) - 1
        return ci >= 0 and t < self._spans[ci][1]

    def index_at(self, t: float) -> int | None:
        """Latest frame at or before t, but never across a gap."""
        return super().index_at(t) if self.has_footage(t) else None

    def gaps(self) -> list[tuple[float, float]]:
        return [(a[1], b[0]) for a, b in zip(self._spans, self._spans[1:]) if b[0] > a[1]]

    def clock(self, t: float) -> dt.datetime:
        return self.start_clock + dt.timedelta(seconds=t)

    # ---------------------------------------------------------------- frames
    def read_rgb(self, idx: int) -> np.ndarray | None:
        ci, local, _ = self.clip_of(idx)
        if ci in self._open:
            self._open.remove(ci)
        self._open.append(ci)
        while len(self._open) > _MAX_OPEN:
            self._clips[self._open.pop(0)].close()
        return self._clips[ci].read_rgb(local)

    def frame_name(self, idx: int) -> str:
        ci, local, _ = self.clip_of(idx)
        return f"{self._clips[ci].path.name}#{local}"

    def fingerprint(self) -> dict:
        return {"name": self.path.name, "clips": [clip.fingerprint() for clip in self._clips]}
```

- [ ] **Step 6: Run the Task 1 tests**

Run: `.venv/Scripts/python -m pytest -q tests/test_clips.py tests/test_source.py`
Expected: all pass (10 new + the existing `test_source.py` tests).

- [ ] **Step 7: Run the whole suite**

Run: `.venv/Scripts/python -m pytest -q > "$TMP/t1.log" 2>&1; tail -3 "$TMP/t1.log"`
Expected: `179 passed` (169 existing + 10 new).

- [ ] **Step 8: Commit**

```bash
git add floodsim/source.py floodsim/clips.py tests/synth.py tests/test_clips.py
git commit -m "feat(source): open a folder of record_ clips as one timeline" -m "MultiClipSource places each clip at its start time from record_YYYYMMDD_HHMMSS, numbers frames continuously, answers has_footage(t) and never hands over a frame across a gap; at most 2 captures stay open." -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 2: Per-clip box caches (`ClipBoxes`)

**Files:**
- Modify: `floodsim/boxes.py` (new classes before `make_box_provider`; `make_box_provider`)
- Modify: `tests/test_boxes.py`

**Interfaces:**
- Consumes: `MultiClipSource.clip_of`, `clip_source`, `clip_bounds`, `path` (Task 1); existing `BoxProvider`, `BoxStore`, `cache_meta`, `make_detector`.
- Produces:
  - `class ClipBoxes`: `__init__(core, source, model: str, conf: float, detector)`; attributes `core`, `source`, `detector`, `store: _ClipStores`; methods `provider(ci) -> BoxProvider`, `cached_for(idx, t, load_rgb) -> list[CachedBox]`, `boxes_for(idx, t, load_rgb) -> list[Box]`; property `description -> dict` (`{"kind": "clips", "detector": bool, "clips": [...]}`)
  - `_ClipStores` facade: `get(idx)`, `set_edited(idx, t, boxes)`, `clear_edited(idx)`, `is_edited(idx)`, properties `has_cache`, `has_edits`, `edited_path`, `cache_path`, `meta`
  - `make_box_provider(...)` returns `ClipBoxes` for a source with `clip_of`

- [ ] **Step 1: Write the failing tests** (append to `tests/test_boxes.py`; add `from floodsim.boxes import ClipBoxes, make_box_provider` and `import floodsim.boxes as boxes_mod` to the imports, and `from floodsim.source import open_source` and `from tests.synth import write_clip`)

```python
def _two_clips(tmp_path):
    folder = tmp_path / "day"
    write_clip(folder, "20260618_120000", [dry_gray()] * 2)   # global frames 0, 1 at t 0, 1
    write_clip(folder, "20260618_120100", [dry_gray()] * 2)   # global frames 2, 3 at t 60, 61
    return folder, open_source(folder)


def test_clip_boxes_use_each_clips_own_cache(core, tmp_path):
    folder, src = _two_clips(tmp_path)
    det = FakeDetector([Detection(10, 20, 30, 40, 0.8, "car")])
    boxes = ClipBoxes(core, src, det.model_name, det.conf, det)
    got = [boxes.boxes_for(i, src.timestamp(i), lambda i=i: src.read_rgb(i)) for i in range(4)]
    assert det.calls == 4
    assert (folder / "record_20260618_120000.avi.boxes.csv").exists()
    assert (folder / "record_20260618_120100.avi.boxes.csv").exists()
    assert not (tmp_path / "day.boxes.csv").exists()
    # The second clip opened alone reuses its cache: its frame 1 is global frame 3.
    single = open_source(folder / "record_20260618_120100.avi")
    det2 = FakeDetector([])
    alone = BoxProvider(core, BoxStore(single.path, cache_meta(single.fingerprint(), det2.model_name,
                                                                det2.conf)), det2)
    assert alone.boxes_for(1, 1.0, lambda: RGB) == got[3]
    assert det2.calls == 0
    assert boxes.description["kind"] == "clips" and len(boxes.description["clips"]) == 2


def test_clip_boxes_edits_land_in_the_clip(core, tmp_path):
    folder, src = _two_clips(tmp_path)
    det = FakeDetector([])
    boxes = ClipBoxes(core, src, det.model_name, det.conf, det)
    edit = [CachedBox(1, 2, 3, 4, 1.0, "person", 1)]
    boxes.store.set_edited(3, 61.0, edit)
    assert boxes.store.is_edited(3) and not boxes.store.is_edited(0) and boxes.store.has_edits
    assert boxes.boxes_for(3, 61.0, lambda: RGB) == [Box(1.0, 2.0, 3.0, 4.0, 1.0, 1)]
    single = open_source(folder / "record_20260618_120100.avi")
    alone = BoxStore(single.path, cache_meta(single.fingerprint(), det.model_name, det.conf))
    assert alone.is_edited(1) and alone.get(1) == edit
    boxes.store.clear_edited(3)
    assert not boxes.store.is_edited(3)


def test_make_box_provider_routes_clips(core, tmp_path, monkeypatch):
    _, src = _two_clips(tmp_path)
    monkeypatch.setattr(boxes_mod, "make_detector", lambda model, conf: FakeDetector([]))
    boxes = make_box_provider(core, src, no_detector=False, model="fake.pt", conf=0.25)
    assert isinstance(boxes, ClipBoxes) and boxes.detector is not None


def test_make_box_provider_clips_without_detector_or_cache_fails(core, tmp_path, monkeypatch):
    _, src = _two_clips(tmp_path)

    def no_detector(model, conf):
        raise FloodSimError("no runtime")

    monkeypatch.setattr(boxes_mod, "make_detector", no_detector)
    with pytest.raises(FloodSimError, match="no runtime"):
        make_box_provider(core, src, no_detector=False, model="fake.pt", conf=0.25)
```

- [ ] **Step 2: Run and watch them fail**

Run: `.venv/Scripts/python -m pytest -q tests/test_boxes.py`
Expected: FAIL at collection with `ImportError: cannot import name 'ClipBoxes'`.

- [ ] **Step 3: Implement in `floodsim/boxes.py`** (insert before `make_box_provider`)

```python
class _ClipStores:
    """Every clip's BoxStore seen as one store, addressed by global frame index (for the GUI)."""

    def __init__(self, boxes: ClipBoxes):
        self._boxes = boxes

    def _at(self, idx: int):
        ci, local, local_t = self._boxes.source.clip_of(idx)
        return self._boxes.provider(ci).store, local, local_t

    def get(self, idx: int) -> list[CachedBox] | None:
        store, local, _ = self._at(idx)
        return store.get(local)

    def is_edited(self, idx: int) -> bool:
        store, local, _ = self._at(idx)
        return store.is_edited(local)

    def set_edited(self, idx: int, t: float, boxes: list[CachedBox]) -> None:
        store, local, local_t = self._at(idx)
        store.set_edited(local, local_t, boxes)

    def clear_edited(self, idx: int) -> None:
        store, local, _ = self._at(idx)
        store.clear_edited(local)

    def _all(self):
        return [self._boxes.provider(ci).store for ci in range(len(self._boxes.source.clip_bounds()))]

    @property
    def has_cache(self) -> bool:
        return any(s.has_cache for s in self._all())

    @property
    def has_edits(self) -> bool:
        return any(s.has_edits for s in self._all())

    @property
    def cache_path(self) -> Path:
        return self._boxes.source.path / "*.boxes.csv"

    @property
    def edited_path(self) -> Path:
        return self._boxes.source.path / "*.boxes.edited.csv"

    @property
    def meta(self) -> dict:
        return {"model": self._boxes.model, "conf": self._boxes.conf}


class ClipBoxes:
    """Boxes for a folder of clips: each frame goes to its own clip's BoxProvider, whose cache is
    exactly the one used when that clip is opened alone."""

    def __init__(self, core, source, model: str, conf: float, detector: Detector | None):
        self.core = core
        self.source = source
        self.model = model
        self.conf = conf
        self.detector = detector
        self._providers: dict[int, BoxProvider] = {}
        self.store = _ClipStores(self)

    def provider(self, ci: int) -> BoxProvider:
        prov = self._providers.get(ci)
        if prov is None:
            clip = self.source.clip_source(ci)
            store = BoxStore(clip.path, cache_meta(clip.fingerprint(), self.model, self.conf))
            prov = self._providers[ci] = BoxProvider(self.core, store, self.detector)
        return prov

    @property
    def description(self) -> dict:
        return {"kind": "clips", "detector": self.detector is not None,
                "clips": [self._providers[ci].description for ci in sorted(self._providers)]}

    def cached_for(self, idx: int, t: float,
                   load_rgb: Callable[[], np.ndarray | None]) -> list[CachedBox]:
        ci, local, local_t = self.source.clip_of(idx)
        return self.provider(ci).cached_for(local, local_t, load_rgb)

    def boxes_for(self, idx: int, t: float, load_rgb: Callable[[], np.ndarray | None]) -> list[Box]:
        return [c.to_box() for c in self.cached_for(idx, t, load_rgb)]
```

In `make_box_provider`, right after the `if no_detector:` return, add:

```python
    if hasattr(source, "clip_of"):
        try:
            detector: Detector | None = make_detector(model, conf)
            error = None
        except FloodSimError as e:
            detector, error = None, e
        boxes = ClipBoxes(core, source, model, conf, detector)
        if error is not None:
            if not boxes.store.has_cache:
                raise error
            log.warning("%s; using the clips' cached boxes only", error)
        return boxes
```

Change the function's return annotation to `-> BoxProvider | ClipBoxes`.

- [ ] **Step 4: Run the box tests**

Run: `.venv/Scripts/python -m pytest -q tests/test_boxes.py`
Expected: all pass (4 new).

- [ ] **Step 5: Whole suite**

Run: `.venv/Scripts/python -m pytest -q > "$TMP/t2.log" 2>&1; tail -3 "$TMP/t2.log"`
Expected: `183 passed`.

- [ ] **Step 6: Commit**

```bash
git add floodsim/boxes.py tests/test_boxes.py
git commit -m "feat(boxes): route a folder of clips to each clip's own box cache" -m "ClipBoxes hands each frame to its clip's BoxProvider/BoxStore, the same cache used when the clip is opened alone; edits land in the clip's .boxes.edited.csv; one shared detector." -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: Device runs skip gaps; reports and CLI

**Files:**
- Modify: `floodsim/modes.py` (`RunOutput`, `run_device`)
- Modify: `floodsim/report.py` (`summarize`, `make_run_info`)
- Modify: `floodsim/cli.py` (`cmd_run` summarize call, `cmd_export`)
- Modify: `tests/test_modes.py`, `tests/test_report.py`, `tests/test_cli.py`

**Interfaces:**
- Consumes: `MultiClipSource` (Task 1; `has_footage`, `start_clock`, `clip_of`), `ClipBoxes` via `make_box_provider` (Task 2), `tests.synth.write_clip`.
- Produces: `RunOutput.skipped_no_footage: int`; `summarize(rows, mode, decode_failures, score, cancelled=False, skipped_no_footage=0) -> str`; `run.json` key `"start_clock"` (ISO 8601) for clip folders.

- [ ] **Step 1: Write the failing runner tests** (append to `tests/test_modes.py`; add `dusk_gray, write_clip` to the `tests.synth` import and `from floodsim.source import open_source`)

```python
def _day_of_clips(tmp_path):
    folder = tmp_path / "day"
    write_clip(folder, "20260618_160000", [dry_gray()] * 3)    # t 0..2
    write_clip(folder, "20260618_161000", [dusk_gray()] * 3)   # t 600..602
    write_clip(folder, "20260618_163000", [dusk_gray()] * 3)   # t 1800..1802
    return open_source(folder)


def test_device_run_skips_gaps_and_carries_learning(core, tmp_path):
    src = _day_of_clips(tmp_path)
    g = core.new_gallery()
    g.add_manual(dry_gray())
    out = run_device(core, src, StaticBoxes({}), g, RunParams(period=60.0))
    assert [r["sample"] for r in out.rows] == [0, 10, 30]
    assert [r["t_sec"] for r in out.rows] == [0.0, 600.0, 1800.0]
    assert out.skipped_no_footage == 28                       # 31 checks, 3 with footage
    assert out.rows[1]["admitted"] is True                    # dusk learned in the second clip
    assert out.rows[2]["gallery_source"] == "auto"            # ...and matched in the third
    assert out.rows[2]["curb_correlation"] == pytest.approx(1.0, abs=1e-6)
    assert [src.clip_of(r["frame_idx"])[0] for r in out.rows] == [0, 1, 2]


def test_retry_past_clip_end_is_blocked(core, tmp_path):
    folder = tmp_path / "edge"
    write_clip(folder, "20260618_160000", [dry_gray()] * 2)     # t 0, 1
    write_clip(folder, "20260618_160959", [dry_gray()] * 2)     # t 599, 600: footage to 601
    src = open_source(folder)
    g = core.new_gallery()
    g.add_manual(dry_gray())
    out = run_device(core, src, StaticBoxes({3: [CAR]}), g, RunParams(period=600.0))
    edge = out.rows[1]
    assert (edge["t_sec"], edge["frame_idx"]) == (600.0, 3)
    assert (edge["outcome"], edge["frames_taken"]) == ("blocked", 2)   # retry at 601 is in the gap


def test_dense_run_covers_every_clip_frame(core, tmp_path):
    src = _day_of_clips(tmp_path)
    g = core.new_gallery()
    g.add_manual(dry_gray())
    out = run_dense(core, src, StaticBoxes({}), g, RunParams(mode="dense"))
    assert [r["frame_idx"] for r in out.rows] == list(range(9))


def test_single_video_runs_are_unchanged(core, ref):
    out = run_device(core, _golden_source(), StaticBoxes({}), ref, RunParams(period=60.0))
    assert out.skipped_no_footage == 0
```

- [ ] **Step 2: Report and CLI tests**

Append to `tests/test_report.py` (`summarize` is already imported there; if not, add `from floodsim.report import summarize`):

```python
def test_summary_reports_skipped_checks_only_when_any():
    assert "checks skipped (no footage): 5" in summarize([], "device", 0, None, skipped_no_footage=5)
    assert "no footage" not in summarize([], "device", 0, None)
```

Append to `tests/test_cli.py` (add `dusk_gray, write_clip` to the `tests.synth` import):

```python
def test_run_on_a_folder_of_clips(tmp_path):
    import json
    folder = tmp_path / "day"
    write_clip(folder, "20260618_160000", [dry_gray()] * 3)
    write_clip(folder, "20260618_161000", [dusk_gray()] * 3)
    ref = tmp_path / "dry.png"
    write_image(ref, to_rgb(dry_gray()))
    assert main(["run", str(folder), "--reference", str(ref), "--no-detector", "--out", str(tmp_path / "r")]) == 0
    rows = read_samples(tmp_path / "r")
    assert [r["t_sec"] for r in rows] == ["0.000", "600.000"]
    info = json.loads((tmp_path / "r" / "run.json").read_text())
    assert info["start_clock"] == "2026-06-18T16:00:00"
    assert len(info["input"]["clips"]) == 2
    assert "checks skipped (no footage): 9" in (tmp_path / "r" / "summary.txt").read_text()


def test_export_refuses_a_folder_of_clips(tmp_path, capsys):
    folder = tmp_path / "day"
    write_clip(folder, "20260618_160000", [dry_gray()] * 2)
    assert main(["export", str(folder), "--no-detector", "--out", str(tmp_path / "e")]) == 2
    assert "export does not support folders of clips" in capsys.readouterr().err
```

(Checks at 0, 60, …, 600 = 11; footage at 0 and 600, so 9 skipped.)

- [ ] **Step 3: Run and watch them fail**

Run: `.venv/Scripts/python -m pytest -q tests/test_modes.py tests/test_report.py tests/test_cli.py`
Expected: FAIL: `AttributeError: 'RunOutput' object has no attribute 'skipped_no_footage'`, `TypeError: summarize() got an unexpected keyword argument 'skipped_no_footage'`, a `KeyError: 'start_clock'`, and the export test returning 0 or failing on something other than the message.

- [ ] **Step 4: Implement**

`floodsim/modes.py`, in `RunOutput` add the field:

```python
    skipped_no_footage: int = 0
```

In `run_device`, replace the body of the `for k in range(total):` loop's first lines so it reads:

```python
    has_footage = getattr(source, "has_footage", None)
    for k in range(total):
        if cancel and cancel():
            out.cancelled = True
            break
        t = k * params.period
        if has_footage is not None and not has_footage(t):
            out.skipped_no_footage += 1   # a gap between clips: nothing to show the device
            if progress:
                progress(k + 1, total)
            continue
        r = sample_device_at(device, ref, feeder, t, params.retry_gap)
```

(`has_footage = ...` goes just before the loop; the rest of the loop body is unchanged.)

`floodsim/report.py`, `summarize`: add the parameter and the line after `failed frame reads`:

```python
def summarize(rows: list[dict], mode: str, decode_failures: int, score: dict | None,
              cancelled: bool = False, skipped_no_footage: int = 0) -> str:
    lines = [f"mode: {mode}", f"rows: {len(rows)}", f"failed frame reads: {decode_failures}"]
    if skipped_no_footage:
        lines.append(f"checks skipped (no footage): {skipped_no_footage}")
```

`make_run_info`: build the dict into a variable, then before returning it:

```python
    start_clock = getattr(source, "start_clock", None)
    if start_clock is not None:
        info["start_clock"] = start_clock.isoformat()
    return info
```

`floodsim/cli.py`, in `cmd_run` change the summarize call to:

```python
              summarize(result.rows, args.mode, result.decode_failures, score,
                        skipped_no_footage=result.skipped_no_footage))
```

In `cmd_export`, right after `source = open_source(...)`:

```python
    if hasattr(source, "clip_of"):
        raise FloodSimError("export does not support folders of clips; export a single clip")
```

- [ ] **Step 5: Run the task's tests**

Run: `.venv/Scripts/python -m pytest -q tests/test_modes.py tests/test_report.py tests/test_cli.py`
Expected: all pass.

- [ ] **Step 6: Whole suite + C tests**

Run: `.venv/Scripts/python -m pytest -q > "$TMP/t3.log" 2>&1; tail -3 "$TMP/t3.log"; ctest --test-dir build/native 2>&1 | tail -2`
Expected: `190 passed`; `100% tests passed out of 6`.

- [ ] **Step 7: Commit**

```bash
git add floodsim/modes.py floodsim/report.py floodsim/cli.py tests/test_modes.py tests/test_report.py tests/test_cli.py
git commit -m "feat: device runs across clips skip gaps; run.json records the start clock" -m "Checks between clips produce no rows and are counted as 'checks skipped (no footage)'; the gallery carries across clips; export refuses folders of clips." -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: GUI

**Files:**
- Modify: `floodsim/gui/session.py` (`open`, `run_batch`, `export_run`)
- Modify: `floodsim/gui/timeline.py` (`ClockAxis`, state in `__init__`, `set_times`, `set_frame`, `set_clock`, `seek_time`)
- Modify: `floodsim/gui/panels.py` (`VerdictPanel.__init__`, `set_start_clock`, `show_device`)
- Modify: `floodsim/gui/main_window.py` (`open_path`, `_on_opened`, `_on_run_done`)
- Modify: `tests/test_gui_session.py`, `tests/gui/test_timeline.py`, `tests/gui/test_panels.py`

**Interfaces:**
- Consumes: `MultiClipSource.start_clock`, `gaps()`, `clip_bounds()` (Task 1); `ClipBoxes` (Task 2); `RunOutput.skipped_no_footage`, `summarize(..., skipped_no_footage=)` (Task 3); `floodsim.source.VIDEO_EXTS`.
- Produces: `Session.open()` info keys `start_clock: datetime | None`, `gaps: list[tuple[float, float]]`, `clips: list[tuple[int, str]]`; `run_batch()` key `skipped_no_footage`; `TimelineWidget.set_clock(start, gaps, clips)`; `timeline.ClockAxis`; `VerdictPanel.set_start_clock(start)`.

- [ ] **Step 1: Session test** (append to `tests/test_gui_session.py`; add `import datetime as dt` and `write_clip` to the `tests.synth` import)

```python
def test_session_opens_folder_of_clips(core, build_info, tmp_path):
    folder = tmp_path / "day"
    write_clip(folder, "20260618_160000", [dry_gray()] * 3)
    write_clip(folder, "20260618_161000", [dusk_gray()] * 3)
    s = Session(core, build_info)
    info = s.open(folder, no_detector=True)
    assert info["start_clock"] == dt.datetime(2026, 6, 18, 16, 0, 0)
    assert info["gaps"] == [(3.0, 600.0)]
    assert info["clips"] == [(0, "record_20260618_160000.avi"), (3, "record_20260618_161000.avi")]
    s.set_reference_frame(0)
    res = s.run_batch(RunParams(mode="device", period=60.0))
    assert [r["t_sec"] for r in res["rows"]] == [0.0, 600.0]
    assert res["skipped_no_footage"] == 9
    assert "1 manual, 1 learned" in s.ref_status().description   # dusk learned, adopted
    assert not (tmp_path / "day.boxes.csv").exists()             # per-clip fallback, not a folder cache
    out = Path(s.export_run(tmp_path / "exp"))
    assert "checks skipped (no footage): 9" in (out / "summary.txt").read_text()
```

- [ ] **Step 2: Widget tests** (append to `tests/gui/test_timeline.py`; add `import datetime as dt` and `from floodsim.gui.timeline import ClockAxis`)

```python
def test_clock_axis_gaps_and_clip_readout(qtbot, tl):
    tl.set_times([0.0, 1.0, 2.0, 310.0, 311.0])
    tl.set_clock(dt.datetime(2026, 6, 18, 23, 59, 50), [(3.0, 310.0)], [(0, "a.avi"), (3, "b.avi")])
    axis = tl.plot.getAxis("bottom")
    assert isinstance(axis, ClockAxis)
    assert axis.tickStrings([0.0, 310.0], 1.0, 1.0) == ["06-18 23:59", "06-19 00:05"]   # spans two days
    assert len(tl._gap_items) == 1
    tl.set_frame(3)
    assert tl.time_label.text() == "00:05:00 · b.avi · frame 4/5"
    with qtbot.waitSignal(tl.seekFrame) as sig:
        tl.seek_time(200.0)          # in the gap: nearest frame with footage is t=310
    assert sig.args == [3]
    tl.set_clock(None, [], [])
    assert not isinstance(tl.plot.getAxis("bottom"), ClockAxis) and tl._gap_items == []
```

Append to `tests/gui/test_panels.py` (add `import datetime as dt`):

```python
def test_verdict_device_row_shows_clock_time(qtbot):
    p = VerdictPanel()
    qtbot.addWidget(p)
    p.set_start_clock(dt.datetime(2026, 6, 18, 16, 0, 0))
    p.show_device(dict(ROW, sample=165, t_sec=2920.0), 0.45)
    assert p.source_label.text() == "device sample #165 at 16:48:40 (t = 2920 s)"
```

- [ ] **Step 3: Run and watch them fail**

Run: `.venv/Scripts/python -m pytest -q tests/test_gui_session.py tests/gui/test_timeline.py tests/gui/test_panels.py`
Expected: FAIL: `KeyError: 'start_clock'`, `ImportError: cannot import name 'ClockAxis'`, `AttributeError: 'VerdictPanel' object has no attribute 'set_start_clock'`.

- [ ] **Step 4: Session** (`floodsim/gui/session.py`)

Add `ClipBoxes` to the `..boxes` import. In `open()`, replace the no-detector fallback:

```python
        if inner is None:
            if hasattr(src, "clip_of"):
                inner = ClipBoxes(self.core, src, self.model, DEFAULT_CONF, None)
            else:
                meta = cache_meta(src.fingerprint(), self.model, DEFAULT_CONF)
                inner = BoxProvider(self.core, BoxStore(src.path, meta), None)
```

and extend the returned dict:

```python
        clock = getattr(src, "start_clock", None)
        return {"name": src.path.name, "n_frames": src.n_frames,
                "times": [src.timestamp(i) for i in range(src.n_frames)],
                "detector": inner.detector is not None,
                "start_clock": clock,
                "gaps": src.gaps() if clock else [],
                "clips": src.clip_bounds() if clock else []}
```

In `run_batch`'s returned dict add `"skipped_no_footage": out.skipped_no_footage`. In `export_run`, pass it to `summarize`:

```python
                  summarize(result.rows, params.mode, result.decode_failures, score,
                            cancelled=result.cancelled, skipped_no_footage=result.skipped_no_footage))
```

- [ ] **Step 5: Timeline** (`floodsim/gui/timeline.py`)

Add `import datetime as dt` at the top. Before `class TimelineWidget`:

```python
class ClockAxis(pg.AxisItem):
    """Seconds on the timeline shown as the time of day, counted from `start`."""

    def __init__(self, start: dt.datetime, with_date: bool):
        super().__init__("bottom")
        self.start = start
        self.with_date = with_date

    def tickStrings(self, values, scale, spacing):
        fmt = "%m-%d %H:%M" if self.with_date else "%H:%M"
        return [(self.start + dt.timedelta(seconds=float(v))).strftime(fmt) for v in values]
```

In `TimelineWidget.__init__`, next to `self._times`:

```python
        self._start: dt.datetime | None = None
        self._gaps: list[tuple[float, float]] = []
        self._clips: list[tuple[int, str]] = []
        self._gap_items: list = []
```

At the start of `set_times`, add `self.set_clock(None, [], [])` (a new input starts without clock mode; `_on_opened` sets it again for clips).

Add the method:

```python
    def set_clock(self, start, gaps, clips) -> None:
        """Clock-time axis, shaded gaps and clip names for a folder of clips; start=None restores seconds."""
        for item in self._gap_items:
            self.plot.removeItem(item)
        self._gap_items = []
        was_clock = self._start is not None
        self._start, self._gaps, self._clips = start, list(gaps), list(clips)
        if start is None:
            if was_clock:
                self.plot.setAxisItems({"bottom": pg.AxisItem("bottom")})
                self.plot.setLabel("bottom", "time", units="s")
            return
        end = start + dt.timedelta(seconds=self._times[-1] if self._times else 0.0)
        self.plot.setAxisItems({"bottom": ClockAxis(start, end.date() != start.date())})
        self.plot.setLabel("bottom", "time of day")
        for a, b in self._gaps:
            item = pg.LinearRegionItem((a, b), movable=False, brush=pg.mkBrush(200, 200, 200, 90),
                                       pen=pg.mkPen(None))
            self.plot.addItem(item)
            self._gap_items.append(item)
```

In `set_frame`, replace the `time_label.setText(...)` line:

```python
        if self._start is not None and self._clips:
            clock = self._start + dt.timedelta(seconds=t)
            k = bisect.bisect_right([first for first, _ in self._clips], idx) - 1
            self.time_label.setText(f"{clock:%H:%M:%S} · {self._clips[k][1]} · frame {idx + 1}/{len(self._times)}")
        else:
            self.time_label.setText(f"t = {t:.2f} s   frame {idx + 1}/{len(self._times)}")
```

Replace `seek_time`:

```python
    def seek_time(self, t: float) -> None:
        idx = self.frame_at_time(t)
        if any(a <= t < b for a, b in self._gaps):   # in a gap: the nearest frame with footage
            j = bisect.bisect_left(self._times, t)
            candidates = [i for i in (j - 1, j) if 0 <= i < len(self._times)]
            idx = min(candidates, key=lambda i: abs(self._times[i] - t))
        if idx is not None:
            self.seekFrame.emit(idx)
```

- [ ] **Step 6: Verdict panel** (`floodsim/gui/panels.py`)

Add `import datetime as dt`. In `VerdictPanel.__init__` add `self._start = None`. Add:

```python
    def set_start_clock(self, start) -> None:
        """Clock time of t = 0 for a folder of clips; None for other inputs."""
        self._start = start
```

In `show_device`, replace the `source_label.setText(...)` line:

```python
        at = f"{row['t_sec']:.2f} s"
        if self._start is not None:
            at = f"{self._start + dt.timedelta(seconds=row['t_sec']):%H:%M:%S} (t = {row['t_sec']:.0f} s)"
        self.source_label.setText(f"device sample #{row['sample']} at {at}")
```

- [ ] **Step 7: Main window** (`floodsim/gui/main_window.py`)

Add `import datetime as dt` if absent and `from ..source import VIDEO_EXTS`. In `open_path`, before `self.worker.submit(...)`:

```python
        p = Path(path)
        n_clips = sum(1 for f in p.iterdir() if f.suffix.lower() in VIDEO_EXTS) if p.is_dir() else 0
        if n_clips:
            self.statusBar().showMessage(f"opening {n_clips} clips…")
```

In `_on_opened`, after `self.timeline.set_times(info["times"])`:

```python
        start = info.get("start_clock")
        self.timeline.set_clock(start, info.get("gaps", []), info.get("clips", []))
        self.verdict.set_start_clock(start)
```

and replace the `self.statusBar().showMessage(f"{info['n_frames']} frame(s)", 5000)` line with:

```python
        if start is not None:
            end = start + dt.timedelta(seconds=info["times"][-1])
            self.statusBar().showMessage(
                f"{len(info['clips'])} clips, {start:%H:%M}–{end:%H:%M}, {info['n_frames']} frames", 10000)
        else:
            self.statusBar().showMessage(f"{info['n_frames']} frame(s)", 5000)
```

In `_on_run_done`, after the `decode_failures` message line:

```python
        if result.get("skipped_no_footage"):
            message += f"; {result['skipped_no_footage']} check(s) skipped (no footage)"
```

- [ ] **Step 8: Run the task's tests**

Run: `.venv/Scripts/python -m pytest -q tests/test_gui_session.py tests/gui`
Expected: all pass.

- [ ] **Step 9: Whole suite**

Run: `.venv/Scripts/python -m pytest -q > "$TMP/t4.log" 2>&1; tail -3 "$TMP/t4.log"`
Expected: `193 passed`.

- [ ] **Step 10: Commit**

```bash
git add floodsim/gui/session.py floodsim/gui/timeline.py floodsim/gui/panels.py floodsim/gui/main_window.py tests/test_gui_session.py tests/gui/test_timeline.py tests/gui/test_panels.py
git commit -m "feat(gui): clock-time timeline, shaded gaps and clip readout for folders of clips" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 5: Docs

**Files:**
- Modify: `README.md`, `docs/HANDBOOK.md`, `docs/HANDBOOK.pdf` (simulator repo)
- Modify: `D:\git\NETWORK_P4_FLOOD_DETECTION\docs\task.md` (append one line)

**Interfaces:** consumes the behaviour and messages of Tasks 1-4; produces nothing code-facing.

- [ ] **Step 1: README** — in the `## Use` code block, after the lighting example, add:

```bash
# a day of recordings: every record_YYYYMMDD_HHMMSS.avi in the folder, at its real start time
.venv/Scripts/floodsim run day_folder/ --reference noon.jpg
```

and after the paragraph about the reference gallery add:

```markdown
A folder of `record_YYYYMMDD_HHMMSS` videos opens as one timeline at the clips' real start times.
Device checks that fall between clips are skipped (counted in `summary.txt`), the gallery carries
from clip to clip, and each clip keeps its own `.boxes.csv`. `floodsim export` does not support
such folders yet.
```

- [ ] **Step 2: Handbook** — in `docs/HANDBOOK.md`, in section 4 (the GUI) after "### First steps", add:

```markdown
### A day of recordings (folder of clips)

**File ▸ Open folder…** on a folder of videos named `record_YYYYMMDD_HHMMSS.avi` opens them as one
timeline, each clip at its real start time (dates carry across midnight).

- The time axis shows the time of day; stretches with no footage are shaded. The readout shows
  the time, the clip and the frame.
- A **Device run** checks every *period* seconds of real time. Checks that fall between clips are
  skipped: no row, and the count appears as "checks skipped (no footage)" in the status bar and
  `summary.txt`. The *sample* numbers keep their place, so a jump in them marks a gap.
- The learned gallery and the heartbeat counter carry across clips, as on one device.
- Each clip keeps its own `.boxes.csv` / `.boxes.edited.csv`, shared with opening that clip alone.
- One edge case: an obstructed check in a clip's last second has its retry fall in the gap, so it
  comes out `blocked` (on the device the retry would have had a frame).
- Clip names must be `record_YYYYMMDD_HHMMSS`; a folder mixing videos and images is refused; if
  two clips overlap, the earlier one's overlapping frames are dropped with a warning.
- `floodsim export` (frames for `flood_sim.exe`) does not support folders of clips.
```

In the Troubleshooting table add:

```markdown
| `clip names must be record_YYYYMMDD_HHMMSS.<ext>` | Rename the listed files; the start time comes from the name. |
| `export does not support folders of clips` | Export a single clip instead. |
```

- [ ] **Step 3: Rebuild the PDF**

Run: `.venv/Scripts/python tools/build_handbook_pdf.py`
Expected: `wrote D:\git\ms500-p4-simulator\docs\HANDBOOK.pdf`. (A reportlab "Parse error: saw </b> instead of expected </i>" means a sentence nests italics inside bold; reword it.) Close any open floodsim GUI first: a running GUI locks `build/native/flood_core.dll`.

- [ ] **Step 4: task.md** — append to `D:\git\NETWORK_P4_FLOOD_DETECTION\docs\task.md`:

```
[2026-09-30] 已完成（仿真器 ms500-p4-simulator）：支持将 record_YYYYMMDD_HHMMSS 视频文件夹作为一条时间线打开，各片段按真实开始时间排列；设备模式运行跨片段连续采样，无画面的时间段跳过并计数，参考图库跨片段延续；各片段复用自己的 .boxes.csv；GUI 显示时钟时间与空档阴影。floodsim export 暂不支持片段文件夹。设计见仿真器 docs/superpowers/specs/2026-09-30-multi-clip-timeline-design.md。
```

- [ ] **Step 5: Commit both repos**

```bash
git add README.md docs/HANDBOOK.md docs/HANDBOOK.pdf
git commit -m "docs: folder-of-clips timeline in the README and handbook" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
cd /d/git/NETWORK_P4_FLOOD_DETECTION
git add docs/task.md
git commit -m "docs: log the simulator's multi-clip timeline in task.md" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

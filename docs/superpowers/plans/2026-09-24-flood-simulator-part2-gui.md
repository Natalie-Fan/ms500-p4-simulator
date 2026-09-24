# Flood Simulator — Part 2: GUI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A PySide6 desktop GUI (layout A: tools | frame | verdict, timeline plot + samples table below) over the Part 1 core, supporting manual mode (Sample / Score / Set reference / Reset state, geometry and box editing) and batch device/dense runs.

**Architecture:** A Qt-free `Session` owns all simulator state and is called only on one worker `QThread` (the firmware core keeps geometry in a global and is not re-entrant). Widgets (`FrameView`, `TimelineWidget`, panels) are dumb views; `MainWindow` wires them to the session through `Worker.submit(call, on_done, on_error)`. Frame loading is latest-wins, so scrubbing never queues up work.

**Tech Stack:** PySide6 ≥ 6.6, pyqtgraph ≥ 0.13, pytest-qt ≥ 4.4, plus everything from Part 1.

**Spec:** `docs/superpowers/specs/2026-09-24-flood-simulator-design.md` (GUI section, layout A)

**Prerequisite:** Part 1 (`2026-09-24-flood-simulator-part1-core.md`) is complete and its tests pass.

**Deviation from the spec (intentional):** while a line endpoint is being dragged, the line itself moves live and the kerb strip is redrawn when the handle is released. The strip comes from the firmware's geometry, so each redraw is a worker round trip; redrawing on release keeps the strip coming from the firmware rather than being recomputed in Python.

## Global Constraints

- Everything from Part 1's Global Constraints still applies.
- GUI dependencies are an optional extra: `pip install -e ".[dev,gui]"`. `floodsim gui` without them fails with `error: GUI dependencies missing: pip install -e .[gui]`.
- Every `Session` method and every DLL call runs on the single `Worker` thread; widgets never call `Session` directly.
- `floodsim/gui/session.py` and `floodsim/gui/__init__.py` must not import Qt, so session tests run without a display.
- Qt tests run headless: `QT_QPA_PLATFORM=offscreen` is set in `tests/conftest.py`.
- Error dialogs are non-modal (`MainWindow.last_error` records the last message) so tests never block.

## Review Focus

1. **Fast scrubbing through a long video**, where each frame costs a decode plus a score. Requests must not pile up behind each other; the last frame asked for is the one shown. Tested in Task 5 (`test_scrubbing_is_latest_wins`).
2. **Opening an image folder whose names have no `_t<ms>`** should ask for a frame rate instead of just failing with a CLI-style `--fps` message. Tested in Task 5 (`test_folder_without_times_asks_for_fps`).
3. **Dragging the lane line above the curb** (invalid geometry): the firmware's error is shown and the drawing snaps back to the geometry actually in effect, not the rejected one. Tested in Task 5 (`test_invalid_geometry_drag_snaps_back`).
4. **A frame that fails to decode** while scrubbing should blank the image with a status message, not raise an error dialog or stop playback. Tested in Task 5 (`test_undecodable_frame_blanks_view`).
5. **Closing the window during a long batch run** should cancel it and close promptly, not hang on `QThread.wait()`. Tested in Task 6 (`test_close_during_run_cancels`).

---

## File Structure

```
native/sim_shim.h, sim_shim.c         + sim_band_coverage, sim_heartbeat_every (ABI 2)
floodsim/core.py                      + Core.band_coverage, Core.heartbeat_every, ABI_VERSION = 2
floodsim/report.py                    + make_run_info (moved out of cli.py)
floodsim/cli.py                       uses make_run_info; + `gui` subcommand
floodsim/gui/__init__.py              empty (no Qt import)
floodsim/gui/session.py               Qt-free state + operations (worker thread only)
floodsim/gui/worker.py                one QThread executor
floodsim/gui/frame_view.py            frame, overlays, editing tools
floodsim/gui/timeline.py              transport + correlation plot
floodsim/gui/panels.py                ToolsPanel, VerdictPanel, SamplesTable
floodsim/gui/main_window.py           layout A, menus, wiring, RunDialog
floodsim/gui/app.py                   run_gui()
tests/conftest.py                     + offscreen QPA
tests/test_gui_session.py             Session tests (no Qt)
tests/gui/__init__.py, conftest.py    importorskip PySide6, worker fixture
tests/gui/test_worker.py, test_frame_view.py, test_timeline.py, test_panels.py, test_main_window.py
```

---

### Task 1: Shim ABI 2 (per-box coverage, heartbeat), GUI dependencies, worker thread

**Files:**
- Modify: `native/sim_shim.h`, `native/sim_shim.c`, `native/tests/test_shim.c`, `floodsim/core.py`, `tests/test_core.py`, `tests/conftest.py`, `pyproject.toml`
- Create: `floodsim/gui/__init__.py`, `floodsim/gui/worker.py`, `tests/gui/__init__.py`, `tests/gui/conftest.py`, `tests/gui/test_worker.py`

**Interfaces:**
- Produces (C): `float sim_band_coverage(const sim_box_t *box)` (firmware `flood_band_coverage`), `int32_t sim_heartbeat_every(void)` (`FLOOD_HEARTBEAT_EVERY`); `SIM_ABI_VERSION 2`.
- Produces (Python): `Core.band_coverage(box: Box) -> float`, `Core.heartbeat_every() -> int`, `core.ABI_VERSION = 2`.
- Produces: `floodsim.gui.worker.Worker(QObject)` with `submit(call, on_done=None, on_error=None) -> int`, `busy` property, `stop()`, signals `progress(int, int)`, `busy_changed(bool)`, `error(str)`. `on_done(result)` and `on_error(message)` run on the GUI thread. A `FloodSimError` becomes its message; any other exception becomes `"<Type>: <msg>\n\n<traceback>"`. With no `on_error`, `error` is emitted.
- Produces (tests): fixture `worker` in `tests/gui/conftest.py`.

- [ ] **Step 1: Add the failing C test**

In `native/tests/test_shim.c` add above `main`:

```c
static void test_band_coverage(void)
{
    const sim_box_t road = { 0.0f, 900.0f, 1919.0f, 1079.0f, 0.9f, 2 };  /* below the lane line */
    CHECK(sim_band_coverage(&k_car) == 1.0f);
    CHECK(sim_band_coverage(&road) == 0.0f);
    CHECK(sim_band_coverage(NULL) == 0.0f);
    CHECK(sim_heartbeat_every() == 60);
}
```

and call `test_band_coverage();` in `main` after `test_device();`.

Run: `cmake --build build/native`
Expected: FAIL with `implicit declaration of function 'sim_band_coverage'` (error or warning), and a link error.

- [ ] **Step 2: Implement in the shim**

In `native/sim_shim.h` change `#define SIM_ABI_VERSION        1` to `#define SIM_ABI_VERSION        2` and add under the `/* ---- reference and scoring (Task 2) ---- */` block:

```c
SIM_API float   sim_band_coverage(const sim_box_t *box);
SIM_API int32_t sim_heartbeat_every(void);
```

In `native/sim_shim.c` add after `sim_box_from_accelerator`:

```c
/* Fraction of the kerb line this one box hides (firmware flood_band_coverage);
 * the GUI shows it on each box. */
float sim_band_coverage(const sim_box_t *box)
{
    if (!box) return 0.0f;
    const flood_box_t b = to_flood_box(box);
    return flood_band_coverage(&b);
}

int32_t sim_heartbeat_every(void) { return FLOOD_HEARTBEAT_EVERY; }
```

Run: `cmake --build build/native && ctest --test-dir build/native --output-on-failure`
Expected: all native tests pass.

- [ ] **Step 3: Update the Python core and its tests**

In `tests/test_core.py` change the first test's assertion `assert build_info["dll"]["abi"] == 1` to `assert build_info["dll"]["abi"] == core_mod.ABI_VERSION == 2`, and add:

```python
def test_band_coverage_and_heartbeat(core):
    assert core.band_coverage(CAR) == 1.0
    assert core.band_coverage(Box(0.0, 900.0, 1919.0, 1079.0, 0.9, 2)) == 0.0
    assert core.heartbeat_every() == 60
```

In `floodsim/core.py`:
- `ABI_VERSION = 1` → `ABI_VERSION = 2`
- in `_declare`, add `"sim_heartbeat_every"` to the `c_int32` name tuple, and add `fn("sim_band_coverage", c_float, POINTER(_SimBox))`
- add to `Core` (after `upload_downscale`):

```python
    def heartbeat_every(self) -> int: return self._lib.sim_heartbeat_every()

    def band_coverage(self, box: Box) -> float:
        b = _SimBox(box.x0, box.y0, box.x1, box.y1, box.confidence, box.cls)
        return self._lib.sim_band_coverage(byref(b))
```

Run: `.venv/Scripts/python -m pytest tests/test_core.py -v`
Expected: all pass (the ABI bump triggers an automatic rebuild).

- [ ] **Step 4: GUI dependencies and headless Qt**

In `pyproject.toml` replace the optional-dependencies block with:

```toml
[project.optional-dependencies]
yolo = ["ultralytics>=8.3"]
gui = ["PySide6>=6.6", "pyqtgraph>=0.13"]
dev = ["pytest>=8", "pytest-qt>=4.4"]
```

At the top of `tests/conftest.py` (before other imports) add:

```python
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")   # Qt tests run without a display
```

Run: `.venv/Scripts/python -m pip install -e ".[dev,gui]"`
Expected: installs PySide6, pyqtgraph, pytest-qt.

- [ ] **Step 5: Write the failing worker tests**

`floodsim/gui/__init__.py`: empty file. `tests/gui/__init__.py`: empty file.

`tests/gui/conftest.py`:
```python
import pytest

pytest.importorskip("PySide6")


@pytest.fixture
def worker(qtbot):
    from floodsim.gui.worker import Worker
    w = Worker()
    yield w
    w.stop()
```

`tests/gui/test_worker.py`:
```python
import threading

from floodsim.errors import FloodSimError


def test_runs_on_worker_thread(qtbot, worker):
    got = []
    worker.submit(threading.get_ident, got.append)
    qtbot.waitUntil(lambda: got, timeout=5000)
    assert got[0] != threading.get_ident()


def test_error_goes_to_handler(qtbot, worker):
    errors = []

    def boom():
        raise FloodSimError("bad input")

    worker.submit(boom, on_error=errors.append)
    qtbot.waitUntil(lambda: errors, timeout=5000)
    assert errors == ["bad input"]


def test_unhandled_error_signal(qtbot, worker):
    with qtbot.waitSignal(worker.error, timeout=5000) as blocker:
        worker.submit(lambda: 1 / 0)
    assert "ZeroDivisionError" in blocker.args[0]


def test_busy_changed(qtbot, worker):
    states, done = [], []
    worker.busy_changed.connect(states.append)
    worker.submit(lambda: 1, done.append)
    qtbot.waitUntil(lambda: done, timeout=5000)
    assert states == [True, False] and not worker.busy


def test_results_arrive_in_submission_order(qtbot, worker):
    got = []
    for i in range(5):
        worker.submit(lambda i=i: i, got.append)
    qtbot.waitUntil(lambda: len(got) == 5, timeout=5000)
    assert got == [0, 1, 2, 3, 4]
```

Run: `.venv/Scripts/python -m pytest tests/gui/test_worker.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'floodsim.gui.worker'`.

- [ ] **Step 6: Write `floodsim/gui/worker.py`**

```python
"""One worker thread for every Session/DLL call: the firmware core keeps its
geometry in a global and is not re-entrant, so calls must never overlap."""
from __future__ import annotations

import traceback
from typing import Any, Callable

from PySide6.QtCore import QObject, QThread, Signal, Slot

from ..errors import FloodSimError


class _Executor(QObject):
    """Lives on the worker thread."""
    done = Signal(int, object)
    error = Signal(int, str)

    @Slot(int, object)
    def run(self, job: int, call: object) -> None:
        try:
            result = call()
        except FloodSimError as e:
            self.error.emit(job, str(e))
            return
        except Exception as e:  # shown to the user with its traceback
            self.error.emit(job, f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}")
            return
        self.done.emit(job, result)


class Worker(QObject):
    """Lives on the GUI thread; callbacks run on the GUI thread."""
    _submit = Signal(int, object)
    progress = Signal(int, int)
    busy_changed = Signal(bool)
    error = Signal(str)

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self._thread = QThread()
        self._exec = _Executor()
        self._exec.moveToThread(self._thread)
        self._submit.connect(self._exec.run)
        self._exec.done.connect(self._on_done)
        self._exec.error.connect(self._on_error)
        self._pending: dict[int, tuple[Callable | None, Callable | None]] = {}
        self._next = 0
        self._thread.start()

    @property
    def busy(self) -> bool:
        return bool(self._pending)

    def submit(self, call: Callable[[], Any], on_done: Callable[[Any], None] | None = None,
               on_error: Callable[[str], None] | None = None) -> int:
        job = self._next
        self._next += 1
        self._pending[job] = (on_done, on_error)
        if len(self._pending) == 1:
            self.busy_changed.emit(True)
        self._submit.emit(job, call)
        return job

    @Slot(int, object)
    def _on_done(self, job: int, result: object) -> None:
        on_done, _ = self._pending.pop(job)
        self._idle_check()
        if on_done:
            on_done(result)

    @Slot(int, str)
    def _on_error(self, job: int, message: str) -> None:
        _, on_error = self._pending.pop(job)
        self._idle_check()
        if on_error:
            on_error(message)
        else:
            self.error.emit(message)

    def _idle_check(self) -> None:
        if not self._pending:
            self.busy_changed.emit(False)

    def stop(self) -> None:
        if self._thread.isRunning():
            self._thread.quit()
            self._thread.wait()
```

- [ ] **Step 7: Run the tests**

Run: `.venv/Scripts/python -m pytest tests/gui/test_worker.py tests/test_core.py -v`
Expected: all pass.

- [ ] **Step 8: Commit**

```bash
git add native floodsim tests pyproject.toml
git commit -m "feat(gui): shim ABI 2 (box coverage, heartbeat) and worker thread" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 2: Qt-free Session

**Files:**
- Create: `floodsim/gui/session.py`, `tests/test_gui_session.py`
- Modify: `floodsim/report.py` (add `make_run_info`), `floodsim/cli.py` (use it)

**Interfaces:**
- Consumes: Part 1 (`open_source`, `BoxProvider`, `BoxStore`, `cache_meta`, `make_box_provider`, `FrameFeeder`, `sample_device_at`, `run_device`, `run_dense`, `UploadWriter`, `device_row`, `Labels`, `score_rows`, `prepare_out_dir`, `summarize`, `write_run`) and `Core.band_coverage`/`heartbeat_every`.
- Produces:
  - `report.make_run_info(*, source, mode, params, geometry, reference, boxes, labels, build_info) -> dict`.
  - `session.FrameData(idx, t, rgb, boxes, score, label)` where `boxes` is `list[tuple[CachedBox, bool, float]]` (box, edited, band coverage).
  - `session.RefStatus(ready, stale, description, geometry_version, built_under)`.
  - `session.Session(core, build_info)` with attributes `geometry`, `geometry_version` and these methods (all worker-thread only):
    - `open(path, *, fps=None, allow_stretch=False, no_detector=False) -> {"name", "n_frames", "times", "detector"}`
    - `frame(idx, *, diff=False) -> FrameData`
    - `score_frame(idx) -> ScoreResult`
    - `set_reference_frame(idx) -> RefStatus`, `set_reference_image(path) -> RefStatus`, `rebuild_reference() -> RefStatus`, `ref_status() -> RefStatus`
    - `default_geometry() -> Geometry`, `set_geometry(g) -> dict`, `overlay_geometry() -> dict`. The overlay dict has keys `geometry, strip, align, domain, submerged_below, veto_threshold, heartbeat_every, sample_period_s`.
    - `set_boxes(idx, boxes)`, `clear_edits(idx)`
    - `sample(idx, retry_gap=1.0) -> dict` (a device row, labelled when labels are loaded), `reset_device()`
    - `run_batch(params, progress=None, cancel=None) -> {"mode", "rows", "cancelled", "decode_failures"}`
    - `load_labels(path) -> {"spans", "warnings", "manual_rows", "run_rows", "run_mode"}`, `label_spans()`
    - `export_run(out) -> str`
  - Staleness mirrors the firmware: the reference is stale when `built_under != geometry_version`.

- [ ] **Step 1: Write the failing tests `tests/test_gui_session.py`**

```python
from pathlib import Path

import pytest

from floodsim.boxes import CachedBox
from floodsim.errors import FloodSimError
from floodsim.geometry import Geometry
from floodsim.gui.session import Session
from floodsim.modes import RunParams
from floodsim.report import read_samples
from tests.synth import dry_gray, flooded_gray, write_frame_folder

CAR = CachedBox(0.0, 0.0, 1919.0, 1079.0, 1.0, "car", 2)


@pytest.fixture
def folder(tmp_path):
    return write_frame_folder(tmp_path / "clip", [(0, dry_gray()), (1000, dry_gray()),
                                                  (60000, flooded_gray()), (61000, flooded_gray())])


@pytest.fixture
def session(core, build_info, folder):
    s = Session(core, build_info)
    info = s.open(folder, no_detector=True)
    assert info["n_frames"] == 4 and info["times"] == [0.0, 1.0, 60.0, 61.0]
    assert info["detector"] is False
    return s


def test_frame_without_reference_has_no_score(session):
    fd = session.frame(0)
    assert fd.score is None and fd.rgb.shape == (1080, 1920, 3) and fd.boxes == []
    with pytest.raises(FloodSimError, match="reference"):
        session.score_frame(0)


def test_reference_then_scores(session):
    st = session.set_reference_frame(0)
    assert st.ready and not st.stale and st.description == "frame 0 (0.00 s)"
    assert session.frame(0).score.curb_correlation == pytest.approx(1.0, abs=1e-6)
    assert session.frame(2).score.submerged is True
    assert session.score_frame(2).severity == "moderate"


def test_geometry_change_marks_reference_stale_and_rebuild_clears(session):
    session.set_reference_frame(0)
    g = session.geometry.to_list()
    g[1] = 240.0
    ov = session.set_geometry(Geometry.from_list(g))
    assert ov["geometry"][1] == 240.0
    st = session.ref_status()
    assert st.stale and (st.built_under, st.geometry_version) == (0, 1)
    assert not session.rebuild_reference().stale


def test_invalid_geometry_rejected(session):
    with pytest.raises(FloodSimError, match="invalid geometry"):
        session.set_geometry(Geometry.from_list([560, 230, 560, 355, 560, 378, 1920, 446]))
    assert session.geometry_version == 0
    assert session.overlay_geometry()["geometry"] == session.default_geometry().to_list()


def test_box_edit_invalidates_score(session):
    session.set_reference_frame(0)
    assert session.frame(0).score.veto_usable
    session.set_boxes(0, [CAR])
    fd = session.frame(0)
    assert fd.boxes == [(CAR, True, 1.0)]
    assert fd.score.veto_usable is False
    session.clear_edits(0)
    assert session.frame(0).score.veto_usable


def test_manual_sample_uses_device_state(core, build_info, session, folder):
    session.set_reference_frame(0)
    row = session.sample(0)
    assert (row["sample"], row["outcome"], row["since_image"]) == (0, "clear", 1)
    assert session.sample(2)["outcome"] == "upload"
    session.reset_device()
    assert session.sample(0)["since_image"] == 1
    fresh = Session(core, build_info)
    fresh.open(folder, no_detector=True)
    assert fresh.sample(0)["outcome"] == "skipped_no_ref"


def test_dense_batch_fills_cache_and_exports(session, tmp_path):
    session.set_reference_frame(0)
    calls = []
    res = session.run_batch(RunParams(mode="dense"), progress=lambda d, t: calls.append(d))
    assert len(res["rows"]) == 4 and calls[-1] == 4 and not res["cancelled"]
    out = session.export_run(tmp_path / "run")
    assert [r["submerged"] for r in read_samples(out)] == ["false", "false", "true", "true"]


def test_device_batch_export_writes_uploads(session, tmp_path):
    session.set_reference_frame(0)
    res = session.run_batch(RunParams(mode="device", period=60.0))
    assert [r["outcome"] for r in res["rows"]] == ["clear", "upload"]
    out = Path(session.export_run(tmp_path / "run"))
    assert len(list((out / "uploads").iterdir())) == 1


def test_dense_batch_needs_reference(session):
    with pytest.raises(FloodSimError, match="reference"):
        session.run_batch(RunParams(mode="dense"))


def test_labels_spans_and_row_labels(session, tmp_path):
    labels = tmp_path / "labels.csv"
    labels.write_text("start_sec,end_sec,label\n0,30,DRY\n30,100,FLOODED\n")
    session.set_reference_frame(0)
    session.sample(2)
    res = session.load_labels(labels)
    assert res["spans"] == [(0.0, 60.0, "DRY"), (60.0, 62.0, "FLOODED")]
    assert res["manual_rows"][0]["label"] == "FLOODED"
    assert session.frame(0).label == "DRY"


def test_diff_view(session):
    session.set_reference_frame(0)
    assert session.frame(0, diff=True).rgb.max() == 0
    assert session.frame(2, diff=True).rgb.max() > 0


def test_export_without_run_errors(session, tmp_path):
    with pytest.raises(FloodSimError, match="run"):
        session.export_run(tmp_path / "run")


def test_overlay_geometry(session):
    ov = session.overlay_geometry()
    assert len(ov["strip"][0]) == 275
    assert ov["align"] == (300, 0, 1700, 200)
    assert ov["heartbeat_every"] == 60 and ov["sample_period_s"] == 60.0
    assert ov["submerged_below"] == pytest.approx(0.45)
```

Run: `.venv/Scripts/python -m pytest tests/test_gui_session.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'floodsim.gui.session'`.

- [ ] **Step 2: Move run-info assembly into `floodsim/report.py`**

Add to `floodsim/report.py` (new imports at the top: `import datetime as dt`, `from dataclasses import asdict`, `from . import __version__`):

```python
def make_run_info(*, source, mode: str, params, geometry, reference: dict, boxes: dict,
                  labels, build_info: dict) -> dict:
    """run.json content: inputs, settings and provenance (firmware commit, DLL)."""
    return {
        "floodsim": __version__, "created": dt.datetime.now().isoformat(timespec="seconds"),
        "input": source.fingerprint(), "mode": mode, "params": asdict(params),
        "geometry": geometry.to_json_dict(), "reference": reference, "boxes": boxes,
        "labels": str(labels) if labels else None, **build_info,
    }
```

In `floodsim/cli.py`, replace the `run_info = {...}` literal in `cmd_run` with:

```python
    run_info = make_run_info(source=source, mode=args.mode, params=params, geometry=geometry,
                             reference=ref_info, boxes=boxes.description, labels=args.labels,
                             build_info=build_info)
```

Add `make_run_info` to the `from .report import ...` line, and remove the now-unused `from . import __version__` and `from dataclasses import asdict` imports from `cli.py`.

Run: `.venv/Scripts/python -m pytest tests/test_cli.py -v`
Expected: 8 passed (behaviour unchanged).

- [ ] **Step 3: Write `floodsim/gui/session.py`**

```python
"""GUI state and operations. Qt-free; every method runs on the worker thread (worker.py)."""
from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from ..boxes import (DEFAULT_CONF, DEFAULT_MODEL, BoxProvider, BoxStore, CachedBox, cache_meta,
                     make_box_provider)
from ..errors import FloodSimError
from ..geometry import Geometry
from ..labels import Labels, score_rows
from ..modes import (FrameFeeder, RunParams, UploadWriter, device_row, run_dense, run_device,
                     sample_device_at)
from ..report import make_run_info, prepare_out_dir, summarize, write_run
from ..results import DENSE_FIELDS, DEVICE_FIELDS, ScoreResult
from ..source import Normalizer, open_source, read_image_bgr

log = logging.getLogger(__name__)
_SCORE_FIELDS = [f for f in DENSE_FIELDS if f not in ("frame_idx", "t_sec")]


class LenientBoxes:
    """The GUI's box provider: a frame the cache lacks, with no detector, just has no boxes."""

    def __init__(self, inner: BoxProvider):
        self.inner = inner
        self.store = inner.store
        self.description = inner.description

    def cached_for(self, idx, t, load_rgb) -> list[CachedBox]:
        try:
            return self.inner.cached_for(idx, t, load_rgb)
        except FloodSimError:
            return []

    def boxes_for(self, idx, t, load_rgb):
        return [c.to_box() for c in self.cached_for(idx, t, load_rgb)]


@dataclass(frozen=True)
class FrameData:
    idx: int
    t: float
    rgb: np.ndarray | None
    boxes: list          # (CachedBox, edited, band coverage)
    score: ScoreResult | None
    label: str | None


@dataclass(frozen=True)
class RefStatus:
    ready: bool
    stale: bool
    description: str
    geometry_version: int
    built_under: int | None


class Session:
    def __init__(self, core, build_info: dict):
        self.core = core
        self.build_info = build_info
        self.source = None
        self.boxes: LenientBoxes | None = None
        self.geometry = self.default_geometry()
        self.geometry_version = 0
        self.core.set_geometry(self.geometry)
        self.reference = None
        self.ref_gray: np.ndarray | None = None
        self.ref_info: dict = {"kind": "none"}
        self.ref_version: int | None = None
        self.device = core.new_device()
        self.dense_cache: dict[int, ScoreResult] = {}
        self.manual_rows: list[dict] = []
        self.run: tuple[RunParams, object] | None = None
        self.labels: Labels | None = None
        self.labels_path: Path | None = None

    # ------------------------------------------------------------ helpers
    def _src(self):
        if self.source is None:
            raise FloodSimError("open a video, image or folder first")
        return self.source

    def _label(self, t: float, idx: int) -> str | None:
        if self.labels is None or self.source is None:
            return None
        return self.labels.label_for(t, self.source.frame_name(idx) if idx >= 0 else None)

    def _label_rows(self, rows: list[dict]) -> None:
        if self.labels is not None:
            for r in rows:
                r["label"] = self._label(r["t_sec"], r["frame_idx"])

    # --------------------------------------------------------------- input
    def open(self, path, *, fps=None, allow_stretch=False, no_detector=False) -> dict:
        src = open_source(Path(path), fps=fps, allow_stretch=allow_stretch)
        inner = None
        if not no_detector:
            try:
                inner = make_box_provider(self.core, src, no_detector=False)
            except FloodSimError as e:
                log.warning("%s", e)
        if inner is None:
            meta = cache_meta(src.fingerprint(), DEFAULT_MODEL, DEFAULT_CONF)
            inner = BoxProvider(self.core, BoxStore(src.path, meta), None)
        self.source = src
        self.boxes = LenientBoxes(inner)
        self.dense_cache.clear()
        self.manual_rows.clear()
        self.run = None
        self.device.reset()
        return {"name": src.path.name, "n_frames": src.n_frames,
                "times": [src.timestamp(i) for i in range(src.n_frames)],
                "detector": inner.detector is not None}

    def _score(self, idx: int, rgb: np.ndarray, cached: list[CachedBox]) -> ScoreResult | None:
        if self.reference is None or not self.reference.ready:
            return None
        s = self.dense_cache.get(idx)
        if s is None:
            s = self.core.score(self.reference, self.core.rgb_to_gray(rgb), [b.to_box() for b in cached])
            self.dense_cache[idx] = s
        return s

    def frame(self, idx: int, *, diff: bool = False) -> FrameData:
        src = self._src()
        t = src.timestamp(idx)
        rgb = src.read_rgb(idx)
        cached = self.boxes.cached_for(idx, t, lambda: rgb) if rgb is not None else []
        edited = self.boxes.store.is_edited(idx)
        boxes = [(b, edited, self.core.band_coverage(b.to_box())) for b in cached]
        score = self._score(idx, rgb, cached) if rgb is not None else None
        shown = rgb
        if diff and rgb is not None and self.ref_gray is not None:
            d = cv2.absdiff(self.core.rgb_to_gray(rgb), self.ref_gray)
            d = np.minimum(d.astype(np.uint16) * 3, 255).astype(np.uint8)
            shown = np.repeat(d[:, :, None], 3, axis=2)
        return FrameData(idx, t, shown, boxes, score, self._label(t, idx))

    def score_frame(self, idx: int) -> ScoreResult:
        if self.reference is None or not self.reference.ready:
            raise FloodSimError("set a dry reference first")
        fd = self.frame(idx)
        if fd.score is None:
            raise FloodSimError(f"frame {idx} failed to decode")
        return fd.score

    # ----------------------------------------------------------- reference
    def _build_reference(self, rgb: np.ndarray, info: dict) -> RefStatus:
        gray = self.core.rgb_to_gray(rgb)
        ref = self.core.new_reference()
        if not ref.build(gray):
            raise FloodSimError("geometry leaves no usable sampling window")
        self.reference, self.ref_gray, self.ref_info = ref, gray, info
        self.ref_version = self.geometry_version
        self.dense_cache.clear()
        return self.ref_status()

    def set_reference_frame(self, idx: int) -> RefStatus:
        src = self._src()
        rgb = src.read_rgb(idx)
        if rgb is None:
            raise FloodSimError(f"frame {idx} failed to decode")
        return self._build_reference(rgb, {"kind": "frame", "frame_idx": idx, "t_sec": src.timestamp(idx)})

    def set_reference_image(self, path) -> RefStatus:
        bgr = read_image_bgr(Path(path))
        if bgr is None:
            raise FloodSimError(f"cannot read reference image {path}")
        rgb = Normalizer(False)(bgr, str(path))
        return self._build_reference(rgb, {"kind": "image", "path": str(path)})

    def rebuild_reference(self) -> RefStatus:
        if self.ref_gray is None:
            raise FloodSimError("no reference to rebuild; set one first")
        ref = self.core.new_reference()
        if not ref.build(self.ref_gray):
            raise FloodSimError("geometry leaves no usable sampling window")
        self.reference = ref
        self.ref_version = self.geometry_version
        self.dense_cache.clear()
        return self.ref_status()

    def ref_status(self) -> RefStatus:
        if self.reference is None:
            return RefStatus(False, False, "none", self.geometry_version, None)
        info = self.ref_info
        if info["kind"] == "frame":
            desc = f"frame {info['frame_idx']} ({info['t_sec']:.2f} s)"
        else:
            desc = f"image {Path(info['path']).name}"
        return RefStatus(True, self.ref_version != self.geometry_version, desc,
                         self.geometry_version, self.ref_version)

    # ------------------------------------------------------------ geometry
    def default_geometry(self) -> Geometry:
        return Geometry.from_list(self.core.default_geometry())

    def set_geometry(self, g: Geometry) -> dict:
        err = self.core.validate_geometry(g)
        if err:
            raise FloodSimError(f"invalid geometry: {err}")
        self.core.set_geometry(g)
        self.geometry = g
        self.geometry_version += 1
        self.dense_cache.clear()
        return self.overlay_geometry()

    def overlay_geometry(self) -> dict:
        return {"geometry": self.geometry.to_list(), "strip": self.core.strip_outline(),
                "align": self.core.align_patch(), "domain": self.core.domain(),
                "submerged_below": self.core.submerged_below(),
                "veto_threshold": self.core.veto_threshold(),
                "heartbeat_every": self.core.heartbeat_every(),
                "sample_period_s": self.core.sample_period_s()}

    # --------------------------------------------------------------- boxes
    def set_boxes(self, idx: int, boxes: list[CachedBox]) -> None:
        self.boxes.store.set_edited(idx, self._src().timestamp(idx), list(boxes))
        self.dense_cache.pop(idx, None)

    def clear_edits(self, idx: int) -> None:
        self.boxes.store.clear_edited(idx)
        self.dense_cache.pop(idx, None)

    # ---------------------------------------------------------- manual mode
    def sample(self, idx: int, retry_gap: float = 1.0) -> dict:
        src = self._src()
        t = src.timestamp(idx)
        r = sample_device_at(self.device, self.reference, FrameFeeder(self.core, src, self.boxes), t, retry_gap)
        row = device_row(len(self.manual_rows), t, r)
        self._label_rows([row])
        self.manual_rows.append(row)
        return dict(row)

    def reset_device(self) -> None:
        self.device.reset()

    # ---------------------------------------------------------------- runs
    def run_batch(self, params: RunParams, progress=None, cancel=None) -> dict:
        src = self._src()
        if params.mode == "dense":
            if self.reference is None or not self.reference.ready:
                raise FloodSimError("dense runs need a dry reference")
            out = run_dense(self.core, src, self.boxes, self.reference, params, progress=progress, cancel=cancel)
            for r in out.rows:
                self.dense_cache[r["frame_idx"]] = ScoreResult(**{k: r[k] for k in _SCORE_FIELDS})
        else:
            out = run_device(self.core, src, self.boxes, self.reference, params, progress=progress, cancel=cancel)
        self._label_rows(out.rows)
        self.run = (params, out)
        return {"mode": params.mode, "rows": [dict(r) for r in out.rows],
                "cancelled": out.cancelled, "decode_failures": out.decode_failures}

    def label_spans(self) -> list[tuple[float, float, str]]:
        src = self.source
        if src is None or self.labels is None:
            return []
        spans: list[list] = []
        n = src.n_frames
        for i in range(n):
            t = src.timestamp(i)
            end = src.timestamp(i + 1) if i + 1 < n else t + (t - src.timestamp(i - 1) if i else 1.0)
            label = self._label(t, i)
            if label is None:
                continue
            if spans and spans[-1][2] == label and abs(spans[-1][1] - t) < 1e-9:
                spans[-1][1] = end
            else:
                spans.append([t, end, label])
        return [tuple(s) for s in spans]

    def load_labels(self, path) -> dict:
        self.labels = Labels.load(Path(path))
        self.labels_path = Path(path)
        self._label_rows(self.manual_rows)
        run_rows = run_mode = None
        if self.run is not None:
            params, out = self.run
            self._label_rows(out.rows)
            run_rows, run_mode = [dict(r) for r in out.rows], params.mode
        return {"spans": self.label_spans(), "warnings": list(self.labels.warnings),
                "manual_rows": [dict(r) for r in self.manual_rows], "run_rows": run_rows,
                "run_mode": run_mode}

    def export_run(self, out) -> str:
        if self.run is None:
            raise FloodSimError("run a device or dense pass first (Run menu)")
        src = self._src()
        params, result = self.run
        out = prepare_out_dir(Path(out))
        fields = list(DEVICE_FIELDS if params.mode == "device" else DENSE_FIELDS)
        score = None
        if self.labels is not None:
            fields.append("label")
            score = score_rows(result.rows, params.mode)
        if params.mode == "device":
            uploads = UploadWriter(out, self.core.upload_downscale())
            for row in result.rows:
                if row["outcome"] in ("upload", "heartbeat") and row["frame_idx"] >= 0:
                    rgb = src.read_rgb(row["frame_idx"])
                    if rgb is not None:
                        uploads.write(row["sample"], rgb)
        info = make_run_info(source=src, mode=params.mode, params=params, geometry=self.geometry,
                             reference=self.ref_info, boxes=self.boxes.description,
                             labels=self.labels_path, build_info=self.build_info)
        write_run(out, info, result.rows, fields,
                  summarize(result.rows, params.mode, result.decode_failures, score))
        return str(out)
```

- [ ] **Step 4: Run the tests**

Run: `.venv/Scripts/python -m pytest tests/test_gui_session.py tests/test_cli.py -v`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add floodsim/gui/session.py floodsim/report.py floodsim/cli.py tests/test_gui_session.py
git commit -m "feat(gui): Qt-free session with manual mode, batch runs, labels, export" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: FrameView (frame, overlays, editing tools)

**Files:**
- Create: `floodsim/gui/frame_view.py`, `tests/gui/test_frame_view.py`

**Interfaces:**
- Consumes: the overlay dict from `Session.overlay_geometry()` (only the keys `geometry`, `strip`, `align`), `FrameData.boxes` tuples.
- Produces: `Tool` enum (`PAN, EDIT_CURB, EDIT_LANE, DRAW_BOX, DELETE_BOX`; `.value` is the button label); `OverlayToggles` (bool fields `curb, lane, strip, align, boxes, shift`); `FrameView(QGraphicsView)` with:
  - `set_image(rgb | None)`, `set_geometry_overlay(dict)`, `set_boxes(list)`, `set_shift((dx, dy) | None)`, `set_tool(Tool)`, `redraw()`, attributes `tool`, `toggles`
  - signals `geometryEdited(list)` (8 floats, on handle release), `boxDrawn(QRectF)` (normalized, ≥ 4×4 px, clamped to the frame), `boxDeleteRequested(int)` (index into the list passed to `set_boxes`; the smallest box containing the click)
  - test hooks `_press(QPointF) -> bool`, `_move(QPointF)`, `_release(QPointF)` that the mouse events call with scene coordinates.

- [ ] **Step 1: Write the failing tests `tests/gui/test_frame_view.py`**

```python
import pytest
from PySide6.QtCore import QPointF, QRectF

from floodsim.boxes import CachedBox
from floodsim.gui.frame_view import FrameView, Tool
from tests.synth import dry_gray, to_rgb


@pytest.fixture
def view(qtbot, core):
    v = FrameView()
    qtbot.addWidget(v)
    v.set_geometry_overlay({"geometry": core.default_geometry(), "strip": core.strip_outline(),
                            "align": core.align_patch()})
    return v


def _drag(v, a, b):
    v._press(QPointF(*a))
    v._move(QPointF(*b))
    v._release(QPointF(*b))


def test_image_and_overlays_drawn(view):
    view.set_image(to_rgb(dry_gray()))
    assert not view._pixmap.pixmap().isNull()
    assert len(view._overlay.childItems()) >= 3   # curb, lane, strip
    view.set_image(None)
    assert view._pixmap.pixmap().isNull()


def test_drag_curb_endpoint_emits_geometry(qtbot, view):
    view.set_tool(Tool.EDIT_CURB)
    with qtbot.waitSignal(view.geometryEdited) as b:
        _drag(view, (560, 230), (600, 240))
    assert b.args[0] == [600.0, 240.0, 1920.0, 355.0, 560.0, 378.0, 1920.0, 446.0]


def test_lane_tool_only_moves_lane(qtbot, view):
    view.set_tool(Tool.EDIT_LANE)
    assert view._press(QPointF(560, 230)) is False   # curb handle not active
    with qtbot.waitSignal(view.geometryEdited) as b:
        _drag(view, (1920, 446), (1900, 460))
    assert b.args[0][6:] == [1900.0, 460.0]


def test_drag_is_clamped_to_frame(qtbot, view):
    view.set_tool(Tool.EDIT_CURB)
    with qtbot.waitSignal(view.geometryEdited) as b:
        _drag(view, (1920, 355), (2500, -40))
    assert b.args[0][2:4] == [1920.0, 0.0]


def test_pan_tool_ignores_handles(view):
    assert view._press(QPointF(560, 230)) is False


def test_draw_box_emits_normalized_rect(qtbot, view):
    view.set_tool(Tool.DRAW_BOX)
    with qtbot.waitSignal(view.boxDrawn) as b:
        _drag(view, (300, 200), (100, 100))
    assert b.args[0] == QRectF(100, 100, 200, 100)


def test_tiny_box_ignored(qtbot, view):
    view.set_tool(Tool.DRAW_BOX)
    with qtbot.assertNotEmitted(view.boxDrawn):
        _drag(view, (100, 100), (102, 101))


def test_delete_picks_smallest_containing_box(qtbot, view):
    big = CachedBox(0, 0, 1000, 1000, 1.0, "bus", 4)
    small = CachedBox(100, 100, 200, 200, 1.0, "person", 1)
    view.set_boxes([(big, False, 0.5), (small, True, 0.0)])
    view.set_tool(Tool.DELETE_BOX)
    with qtbot.waitSignal(view.boxDeleteRequested) as b:
        view._press(QPointF(150, 150))
    assert b.args == [1]


def test_toggles_hide_overlays(view):
    before = len(view._overlay.childItems())
    view.toggles.strip = False
    view.redraw()
    assert len(view._overlay.childItems()) == before - 1
```

Run: `.venv/Scripts/python -m pytest tests/gui/test_frame_view.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'floodsim.gui.frame_view'`.

- [ ] **Step 2: Write `floodsim/gui/frame_view.py`**

```python
"""The frame with overlays, and the manual editing tools (layout A, centre)."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QImage, QPainter, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import (QGraphicsEllipseItem, QGraphicsItem, QGraphicsItemGroup,
                               QGraphicsLineItem, QGraphicsPixmapItem, QGraphicsPolygonItem,
                               QGraphicsRectItem, QGraphicsScene, QGraphicsSimpleTextItem,
                               QGraphicsView)

FRAME_W, FRAME_H = 1920, 1080
HANDLE_RADIUS = 12.0
GRAB_RADIUS = 24.0      # how close a press must be to an endpoint to grab it (scene px)
MIN_BOX = 4.0           # smaller rubber bands are treated as a click
SHIFT_SCALE = 4.0       # the registration shift is a few px; drawn 4x so it is visible

CLASS_COLORS = {"person": "#e6194b", "car": "#f58231", "truck": "#ffe119", "bus": "#bfef45",
                "bicycle": "#42d4f4", "motorcycle": "#911eb4", "other": "#a9a9a9"}
CURB_COLOR, LANE_COLOR = "#ff3333", "#33ccff"
STRIP_COLOR, ALIGN_COLOR, SHIFT_COLOR = "#ffee00", "#aaffaa", "#ff66ff"


class Tool(Enum):
    PAN = "Pan / zoom"
    EDIT_CURB = "Edit curb line"
    EDIT_LANE = "Edit lane line"
    DRAW_BOX = "Draw box"
    DELETE_BOX = "Delete box"


@dataclass
class OverlayToggles:
    curb: bool = True
    lane: bool = True
    strip: bool = True
    align: bool = False
    boxes: bool = True
    shift: bool = True


def _pen(color: str, width: float = 2.0, dashed: bool = False) -> QPen:
    pen = QPen(QColor(color))
    pen.setWidthF(width)
    pen.setCosmetic(True)   # constant on-screen width at any zoom
    if dashed:
        pen.setStyle(Qt.DashLine)
    return pen


def _clamp(p: QPointF) -> QPointF:
    return QPointF(min(max(p.x(), 0.0), float(FRAME_W)), min(max(p.y(), 0.0), float(FRAME_H)))


def _text(text: str, color: str, x: float, y: float) -> QGraphicsSimpleTextItem:
    item = QGraphicsSimpleTextItem(text)
    item.setBrush(QBrush(QColor(color)))
    item.setPos(x, y)
    item.setFlag(QGraphicsItem.ItemIgnoresTransformations)   # readable at any zoom
    return item


class FrameView(QGraphicsView):
    geometryEdited = Signal(list)
    boxDrawn = Signal(QRectF)
    boxDeleteRequested = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setScene(QGraphicsScene(0, 0, FRAME_W, FRAME_H, self))
        self.setRenderHint(QPainter.Antialiasing)
        self.setBackgroundBrush(QColor("#202020"))
        self._pixmap = QGraphicsPixmapItem()
        self.scene().addItem(self._pixmap)
        self._overlay: QGraphicsItemGroup | None = None
        self.tool = Tool.PAN
        self.toggles = OverlayToggles()
        self._geometry: list[float] | None = None
        self._strip = None
        self._align = None
        self._boxes: list = []
        self._shift: tuple[int, int] | None = None
        self._drag_endpoint: int | None = None
        self._rubber_start: QPointF | None = None
        self._rubber: QGraphicsRectItem | None = None
        self._fit = True
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.redraw()

    # -------------------------------------------------------------- content
    def set_image(self, rgb: np.ndarray | None) -> None:
        if rgb is None:
            self._pixmap.setPixmap(QPixmap())
            return
        rgb = np.ascontiguousarray(rgb)
        h, w = rgb.shape[:2]
        image = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()
        self._pixmap.setPixmap(QPixmap.fromImage(image))

    def set_geometry_overlay(self, overlay: dict) -> None:
        self._geometry = [float(v) for v in overlay["geometry"]]
        self._strip = overlay["strip"]
        self._align = overlay["align"]
        self.redraw()

    def set_boxes(self, boxes: list) -> None:
        self._boxes = list(boxes)
        self.redraw()

    def set_shift(self, shift: tuple[int, int] | None) -> None:
        self._shift = shift
        self.redraw()

    def set_tool(self, tool: Tool) -> None:
        self.tool = tool
        self.setDragMode(QGraphicsView.ScrollHandDrag if tool is Tool.PAN else QGraphicsView.NoDrag)
        self.redraw()

    def redraw(self) -> None:
        if self._overlay is not None:
            self.scene().removeItem(self._overlay)
        group = QGraphicsItemGroup()
        self._overlay = group
        self.scene().addItem(group)
        g = self._geometry
        if g:
            if self.toggles.curb:
                group.addToGroup(self._line(g[0:4], CURB_COLOR))
            if self.toggles.lane:
                group.addToGroup(self._line(g[4:8], LANE_COLOR))
            for k in self._active_endpoints():
                group.addToGroup(self._handle(g[2 * k], g[2 * k + 1], CURB_COLOR if k < 2 else LANE_COLOR))
        if self.toggles.strip and self._strip is not None and len(self._strip[0]):
            xs, lo, hi = self._strip
            pts = [QPointF(float(x), float(y)) for x, y in zip(xs, lo)]
            pts += [QPointF(float(x), float(y)) for x, y in zip(xs[::-1], hi[::-1])]
            strip = QGraphicsPolygonItem(QPolygonF(pts))
            strip.setPen(_pen(STRIP_COLOR, 1.5, dashed=True))
            group.addToGroup(strip)
        if self.toggles.align and self._align:
            x0, y0, x1, y1 = self._align
            rect = QGraphicsRectItem(QRectF(x0, y0, x1 - x0, y1 - y0))
            rect.setPen(_pen(ALIGN_COLOR, 1.5, dashed=True))
            group.addToGroup(rect)
        if self.toggles.boxes:
            for box, edited, coverage in self._boxes:
                color = CLASS_COLORS.get(box.coco_class, CLASS_COLORS["other"])
                rect = QGraphicsRectItem(QRectF(box.x0, box.y0, box.x1 - box.x0, box.y1 - box.y0))
                rect.setPen(_pen(color, 2.0, dashed=edited))
                group.addToGroup(rect)
                group.addToGroup(_text(f"{box.coco_class} {coverage:.0%}", color, box.x0, box.y0))
        if self.toggles.shift and self._shift is not None and self._align:
            dx, dy = self._shift
            x0, y0, x1, y1 = self._align
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            arrow = QGraphicsLineItem(cx, cy, cx + dx * SHIFT_SCALE, cy + dy * SHIFT_SCALE)
            arrow.setPen(_pen(SHIFT_COLOR, 2.5))
            group.addToGroup(arrow)
            group.addToGroup(_text(f"shift dx {dx:+d} dy {dy:+d}", SHIFT_COLOR, cx + 8, cy + 8))

    def _line(self, v: list[float], color: str) -> QGraphicsLineItem:
        item = QGraphicsLineItem(v[0], v[1], v[2], v[3])
        item.setPen(_pen(color, 2.5))
        return item

    def _handle(self, x: float, y: float, color: str) -> QGraphicsEllipseItem:
        r = HANDLE_RADIUS
        item = QGraphicsEllipseItem(x - r, y - r, 2 * r, 2 * r)
        item.setPen(_pen("#ffffff", 1.5))
        item.setBrush(QBrush(QColor(color)))
        return item

    def _active_endpoints(self) -> tuple[int, ...]:
        return {Tool.EDIT_CURB: (0, 1), Tool.EDIT_LANE: (2, 3)}.get(self.tool, ())

    def _box_at(self, p: QPointF) -> int | None:
        hits = [((b.x1 - b.x0) * (b.y1 - b.y0), i) for i, (b, _, _) in enumerate(self._boxes)
                if b.x0 <= p.x() <= b.x1 and b.y0 <= p.y() <= b.y1]
        return min(hits)[1] if hits else None

    # ----------------------------------------------------------- interaction
    def _press(self, p: QPointF) -> bool:
        if self._geometry and self._active_endpoints():
            for k in self._active_endpoints():
                x, y = self._geometry[2 * k], self._geometry[2 * k + 1]
                if (p.x() - x) ** 2 + (p.y() - y) ** 2 <= GRAB_RADIUS ** 2:
                    self._drag_endpoint = k
                    return True
            return False
        if self.tool is Tool.DRAW_BOX:
            self._rubber_start = _clamp(p)
            self._rubber = QGraphicsRectItem(QRectF(self._rubber_start, self._rubber_start))
            self._rubber.setPen(_pen("#ffffff", 1.5, dashed=True))
            self.scene().addItem(self._rubber)
            return True
        if self.tool is Tool.DELETE_BOX:
            idx = self._box_at(p)
            if idx is not None:
                self.boxDeleteRequested.emit(idx)
                return True
        return False

    def _move(self, p: QPointF) -> None:
        p = _clamp(p)
        if self._drag_endpoint is not None:
            k = self._drag_endpoint
            self._geometry[2 * k], self._geometry[2 * k + 1] = p.x(), p.y()
            self.redraw()
        elif self._rubber_start is not None and self._rubber is not None:
            self._rubber.setRect(QRectF(self._rubber_start, p).normalized())

    def _release(self, p: QPointF) -> None:
        if self._drag_endpoint is not None:
            self._move(p)
            self._drag_endpoint = None
            self.redraw()
            self.geometryEdited.emit(list(self._geometry))
        elif self._rubber_start is not None:
            rect = QRectF(self._rubber_start, _clamp(p)).normalized()
            if self._rubber is not None:
                self.scene().removeItem(self._rubber)
            self._rubber = None
            self._rubber_start = None
            if rect.width() >= MIN_BOX and rect.height() >= MIN_BOX:
                self.boxDrawn.emit(rect)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton and self._press(self.mapToScene(event.position().toPoint())):
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._drag_endpoint is not None or self._rubber_start is not None:
            self._move(self.mapToScene(event.position().toPoint()))
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if self._drag_endpoint is not None or self._rubber_start is not None:
            self._release(self.mapToScene(event.position().toPoint()))
            event.accept()
            return
        super().mouseReleaseEvent(event)

    # ------------------------------------------------------------- zooming
    def wheelEvent(self, event) -> None:
        factor = 1.15 ** (event.angleDelta().y() / 120)
        self.scale(factor, factor)
        self._fit = False

    def mouseDoubleClickEvent(self, event) -> None:
        self._fit = True
        self._fit_view()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if self._fit:
            self._fit_view()

    def showEvent(self, event) -> None:
        super().showEvent(event)
        if self._fit:
            self._fit_view()

    def _fit_view(self) -> None:
        self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)
```

- [ ] **Step 3: Run the tests**

Run: `.venv/Scripts/python -m pytest tests/gui/test_frame_view.py -v`
Expected: 9 passed.

- [ ] **Step 4: Commit**

```bash
git add floodsim/gui/frame_view.py tests/gui/test_frame_view.py
git commit -m "feat(gui): frame view with overlays and geometry/box editing tools" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: TimelineWidget (transport + correlation plot)

**Files:**
- Create: `floodsim/gui/timeline.py`, `tests/gui/test_timeline.py`

**Interfaces:**
- Produces: `TimelineWidget(QWidget)` with:
  - `set_times(list[float])`, `set_frame(idx)` (never emits), `set_threshold(float)`, `set_dense(list[(t, corr, veto_usable)])`, `set_device(list[row dict with t_sec, outcome])`, `set_labels(list[(start, end, label)])`, `frame_at_time(t) -> int | None`, `seek_time(t)`, `set_playing(bool)`
  - signals `seekFrame(int)`, `playToggled(bool)`
  - public children `slider`, `play_btn`, `back_btn`, `fwd_btn`, `time_label`, `curve`, `markers`, `threshold_line`, `playhead`; internal `_blocked_item`, `_labels_item`.

- [ ] **Step 1: Write the failing tests `tests/gui/test_timeline.py`**

```python
import pytest

from floodsim.gui.timeline import TimelineWidget


@pytest.fixture
def tl(qtbot):
    w = TimelineWidget()
    qtbot.addWidget(w)
    w.set_times([0.0, 1.0, 60.0, 61.0])
    return w


def test_set_frame_does_not_emit(qtbot, tl):
    with qtbot.assertNotEmitted(tl.seekFrame):
        tl.set_frame(2)
    assert tl.slider.value() == 2
    assert "60.00 s" in tl.time_label.text() and "3/4" in tl.time_label.text()
    assert tl.playhead.value() == 60.0


def test_slider_emits_seek(qtbot, tl):
    with qtbot.waitSignal(tl.seekFrame) as b:
        tl.slider.setValue(3)
    assert b.args == [3]


def test_seek_time_picks_frame_at_or_before(qtbot, tl):
    with qtbot.waitSignal(tl.seekFrame) as b:
        tl.seek_time(59.5)
    assert b.args == [1]


def test_seek_before_start_ignored(qtbot, tl):
    with qtbot.assertNotEmitted(tl.seekFrame):
        tl.seek_time(-5.0)


def test_step_buttons(qtbot, tl):
    tl.set_frame(1)
    with qtbot.waitSignal(tl.seekFrame) as b:
        tl.fwd_btn.click()
    assert b.args == [2]
    tl.set_frame(0)
    with qtbot.waitSignal(tl.seekFrame) as b:
        tl.back_btn.click()
    assert b.args == [0]


def test_play_button_emits(qtbot, tl):
    with qtbot.waitSignal(tl.playToggled) as b:
        tl.play_btn.click()
    assert b.args == [True]


def test_series(tl):
    tl.set_threshold(0.45)
    assert tl.threshold_line.isVisible() and tl.threshold_line.value() == pytest.approx(0.45)
    tl.set_dense([(61.0, 1.0, False), (0.0, 1.0, True), (60.0, 0.0, True)])
    x, y = tl.curve.getData()
    assert list(x) == [0.0, 60.0, 61.0] and list(y) == [1.0, 0.0, 1.0]
    assert tl._blocked_item is not None
    tl.set_device([{"t_sec": 0.0, "outcome": "clear"}, {"t_sec": 60.0, "outcome": "upload"},
                   {"t_sec": 120.0, "outcome": "skipped_no_ref"}])
    assert len(tl.markers.data) == 2
    tl.set_labels([(0.0, 60.0, "DRY"), (60.0, 62.0, "FLOODED")])
    assert tl._labels_item is not None
    tl.set_times([0.0])   # new input clears the series
    x, _ = tl.curve.getData()
    assert x is None or len(x) == 0
    assert tl._blocked_item is None and tl._labels_item is None
```

Run: `.venv/Scripts/python -m pytest tests/gui/test_timeline.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'floodsim.gui.timeline'`.

- [ ] **Step 2: Write `floodsim/gui/timeline.py`**

```python
"""Transport controls and the correlation-over-time plot, on one time axis."""
from __future__ import annotations

import bisect

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget

import pyqtgraph as pg  # after PySide6, so pyqtgraph binds to it

OUTCOME_COLORS = {"upload": "#d62728", "clear": "#2ca02c", "blocked": "#7f7f7f", "heartbeat": "#1f77b4"}
LABEL_COLORS = {"DRY": "#b8860b", "WET": "#4682b4", "FLOODED": "#d62728", "INDETERMINATE": "#999999"}
Y_MIN, Y_MAX = -1.35, 1.15
MARKER_Y = 1.08
LABEL_Y0, LABEL_H = -1.33, 0.06


class TimelineWidget(QWidget):
    seekFrame = Signal(int)
    playToggled = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._times: list[float] = []
        self._updating = False
        self._blocked_item = None
        self._labels_item = None

        self.back_btn = QPushButton("◀")
        self.play_btn = QPushButton("▶")
        self.play_btn.setCheckable(True)
        self.fwd_btn = QPushButton("▶|")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 0)
        self.time_label = QLabel("no input")
        self.back_btn.clicked.connect(lambda: self._step(-1))
        self.fwd_btn.clicked.connect(lambda: self._step(1))
        self.play_btn.toggled.connect(self._on_play)
        self.slider.valueChanged.connect(self._on_slider)
        controls = QHBoxLayout()
        for w in (self.back_btn, self.play_btn, self.fwd_btn):
            controls.addWidget(w)
        controls.addWidget(self.slider, 1)
        controls.addWidget(self.time_label)

        self.plot = pg.PlotWidget(background="w")
        self.plot.setMenuEnabled(False)
        self.plot.setMouseEnabled(x=True, y=False)
        self.plot.setYRange(Y_MIN, Y_MAX, padding=0)
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.setLabel("left", "curb corr")
        self.plot.setLabel("bottom", "time", units="s")
        self.plot.setFixedHeight(170)
        self.curve = self.plot.plot([], [], pen=pg.mkPen("#223366", width=1.5), symbol="o",
                                    symbolSize=4, symbolBrush="#223366", symbolPen=None)
        self.threshold_line = pg.InfiniteLine(angle=0, pos=0.0, pen=pg.mkPen("#d62728", style=Qt.DashLine))
        self.threshold_line.setVisible(False)
        self.playhead = pg.InfiniteLine(angle=90, pos=0.0, pen=pg.mkPen("#000000", width=1))
        self.markers = pg.ScatterPlotItem()
        for item in (self.threshold_line, self.playhead, self.markers):
            self.plot.addItem(item)
        self.plot.scene().sigMouseClicked.connect(self._on_plot_click)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(controls)
        layout.addWidget(self.plot)

    # ------------------------------------------------------------- frames
    def set_times(self, times) -> None:
        self._times = [float(t) for t in times]
        self._updating = True
        self.slider.setRange(0, max(len(self._times) - 1, 0))
        self.slider.setValue(0)
        self._updating = False
        self.curve.setData([], [])
        self.markers.clear()
        self._remove("_blocked_item")
        self._remove("_labels_item")
        end = self._times[-1] if self._times else 1.0
        self.plot.setXRange(0.0, max(end, 1.0), padding=0.02)

    def set_frame(self, idx: int) -> None:
        if not self._times:
            return
        self._updating = True
        self.slider.setValue(idx)
        self._updating = False
        t = self._times[idx]
        self.playhead.setValue(t)
        self.time_label.setText(f"t = {t:.2f} s   frame {idx + 1}/{len(self._times)}")

    def frame_at_time(self, t: float) -> int | None:
        if not self._times:
            return None
        i = bisect.bisect_right(self._times, t + 1e-9) - 1
        return i if i >= 0 else None

    def seek_time(self, t: float) -> None:
        idx = self.frame_at_time(t)
        if idx is not None:
            self.seekFrame.emit(idx)

    def set_playing(self, on: bool) -> None:
        self.play_btn.setChecked(on)

    # ------------------------------------------------------------- series
    def set_threshold(self, value: float) -> None:
        self.threshold_line.setValue(value)
        self.threshold_line.setVisible(True)

    def set_dense(self, points) -> None:
        pts = sorted(points)
        self.curve.setData(np.array([p[0] for p in pts], float), np.array([p[1] for p in pts], float))
        self._remove("_blocked_item")
        blocked = [p[0] for p in pts if not p[2]]
        if blocked:
            self._blocked_item = pg.BarGraphItem(
                x=np.array(blocked), height=Y_MAX - Y_MIN, y0=Y_MIN, width=self._frame_width(),
                brush=pg.mkBrush(128, 128, 128, 70), pen=pg.mkPen(None))
            self.plot.addItem(self._blocked_item)

    def set_device(self, rows) -> None:
        rows = [r for r in rows if r["outcome"] in OUTCOME_COLORS]
        self.markers.setData(x=[r["t_sec"] for r in rows], y=[MARKER_Y] * len(rows), symbol="t",
                             size=11, pen=None,
                             brush=[pg.mkBrush(OUTCOME_COLORS[r["outcome"]]) for r in rows])

    def set_labels(self, spans) -> None:
        self._remove("_labels_item")
        if spans:
            self._labels_item = pg.BarGraphItem(
                x0=np.array([s[0] for s in spans]), width=np.array([s[1] - s[0] for s in spans]),
                y0=LABEL_Y0, height=LABEL_H,
                brushes=[pg.mkBrush(LABEL_COLORS.get(s[2], "#999999")) for s in spans],
                pens=[pg.mkPen(None)] * len(spans))
            self.plot.addItem(self._labels_item)

    # ------------------------------------------------------------ helpers
    def _remove(self, attr: str) -> None:
        item = getattr(self, attr)
        if item is not None:
            self.plot.removeItem(item)
            setattr(self, attr, None)

    def _frame_width(self) -> float:
        if len(self._times) > 1:
            return float(np.median(np.diff(self._times))) or 1.0
        return 1.0

    def _step(self, delta: int) -> None:
        if self._times:
            self.seekFrame.emit(max(0, min(len(self._times) - 1, self.slider.value() + delta)))

    def _on_slider(self, value: int) -> None:
        if not self._updating:
            self.seekFrame.emit(value)

    def _on_play(self, on: bool) -> None:
        self.play_btn.setText("⏸" if on else "▶")
        self.playToggled.emit(on)

    def _on_plot_click(self, event) -> None:
        if event.button() != Qt.LeftButton:
            return
        vb = self.plot.getPlotItem().getViewBox()
        pos = event.scenePos()
        if vb.sceneBoundingRect().contains(pos):
            self.seek_time(vb.mapSceneToView(pos).x())
```

- [ ] **Step 3: Run the tests**

Run: `.venv/Scripts/python -m pytest tests/gui/test_timeline.py -v`
Expected: 7 passed.

- [ ] **Step 4: Commit**

```bash
git add floodsim/gui/timeline.py tests/gui/test_timeline.py
git commit -m "feat(gui): timeline with scrubber, correlation plot, markers and labels" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 5: Panels and MainWindow (manual mode)

**Files:**
- Create: `floodsim/gui/panels.py`, `floodsim/gui/main_window.py`, `tests/gui/test_panels.py`, `tests/gui/test_main_window.py`

**Interfaces:**
- Consumes: `Session`, `Worker`, `FrameView`, `Tool`, `TimelineWidget`, `RefStatus`, `format_value`, `COCO_TO_FLOOD`, `CachedBox`, `Geometry`, `load_geometry`, `save_geometry`, `RunParams`.
- Produces:
  - `panels.ToolsPanel` with signals `toolChanged(object)`, `sampleClicked`, `scoreClicked`, `setReferenceClicked`, `resetStateClicked`; `tool_buttons: dict[Tool, QRadioButton]`; buttons `sample_btn, score_btn, set_reference_btn, reset_btn`; `set_actions_enabled(has_source, has_reference)`.
  - `panels.VerdictPanel` with signal `rebuildClicked`; `show_score(score, label, threshold)`, `show_device(row, threshold)`, `show_state(since_image, heartbeat_every)`, `show_reference(RefStatus)`, `show_firmware(dict)`; labels `source_label, outcome_label, severity_label, corr_label, shift_label, veto_label, frames_label, label_label, since_label, reference_label, firmware_label`; `stale_banner`, `rebuild_btn`.
  - `panels.SamplesTable(QTableWidget)` with signal `frameRequested(int)`; `set_rows(rows, mode)`, `add_row(row, mode)`.
  - `main_window.RunDialog(defaults: RunParams)` with `params() -> RunParams`.
  - `main_window.MainWindow(session, worker, build_info)` with `open_path(path, *, fps=None, allow_stretch=False, no_detector=False)`, `show_frame(idx)`, attributes `last_error`, `_last_frame`, `frame_view`, `tools`, `verdict`, `timeline`, `samples`. Batch-run methods `_start_run(mode)`, `_cancel_run()`, and label/export actions are wired here but tested in Task 6.

- [ ] **Step 1: Write the failing panel tests `tests/gui/test_panels.py`**

```python
from floodsim.gui.frame_view import Tool
from floodsim.gui.panels import SamplesTable, ToolsPanel, VerdictPanel
from floodsim.gui.session import RefStatus
from floodsim.results import ScoreResult

ROW = {"sample": 3, "t_sec": 60.0, "frame_idx": 2, "outcome": "upload", "frames_taken": 1,
       "has_reading": True, "curb_correlation": 0.21, "severity": "severe", "shift_dx": -3,
       "shift_dy": 1, "veto_usable": True, "veto_coverage": 0.04, "veto_worst_class": "car",
       "since_image": 0, "label": None}


def test_verdict_device_row(qtbot):
    p = VerdictPanel()
    qtbot.addWidget(p)
    p.show_device(ROW, 0.45)
    assert p.outcome_label.text() == "UPLOAD"
    assert p.corr_label.text() == "0.210 (threshold 0.45)"
    assert p.veto_label.text() == "usable, coverage 4% (car)"
    assert p.shift_label.text() == "dx -3  dy +1"
    assert p.source_label.text() == "device sample #3 at 60.00 s"


def test_verdict_score_and_no_reading(qtbot):
    p = VerdictPanel()
    qtbot.addWidget(p)
    p.show_score(ScoreResult(False, 1.0, "car", 1.0, False, "none", 0, 0), "DRY", 0.45)
    assert p.outcome_label.text() == "VETOED" and p.label_label.text() == "DRY"
    p.show_score(None, None, 0.45)
    assert p.outcome_label.text() == "no score" and p.corr_label.text() == "–"
    p.show_device(dict(ROW, outcome="blocked", has_reading=False), 0.45)
    assert p.corr_label.text() == "– (no reading)"


def test_stale_banner(qtbot):
    p = VerdictPanel()
    qtbot.addWidget(p)
    p.show_reference(RefStatus(True, True, "frame 0 (0.00 s)", 2, 1))
    assert not p.stale_banner.isHidden() and "v1" in p.stale_text.text()
    p.show_reference(RefStatus(True, False, "frame 0 (0.00 s)", 2, 2))
    assert p.stale_banner.isHidden()
    p.show_reference(RefStatus(False, False, "none", 0, None))
    assert p.reference_label.text().startswith("none")


def test_firmware_label(qtbot):
    p = VerdictPanel()
    qtbot.addWidget(p)
    p.show_firmware({"commit": "7c9bed2abcdef", "dirty": False})
    assert p.firmware_label.text() == "7c9bed2 (clean)"


def test_tools_enable_and_signal(qtbot):
    t = ToolsPanel()
    qtbot.addWidget(t)
    t.set_actions_enabled(True, False)
    assert t.sample_btn.isEnabled() and t.set_reference_btn.isEnabled() and not t.score_btn.isEnabled()
    with qtbot.waitSignal(t.toolChanged) as b:
        t.tool_buttons[Tool.DRAW_BOX].click()
    assert b.args == [Tool.DRAW_BOX]


def test_samples_table(qtbot):
    s = SamplesTable()
    qtbot.addWidget(s)
    s.set_rows([ROW], "device")
    s.add_row(dict(ROW, sample=4, frame_idx=-1, outcome="blocked"), "device")
    assert s.rowCount() == 2 and s.item(0, 3).text() == "upload" and s.item(0, 4).text() == "0.210000"
    with qtbot.waitSignal(s.frameRequested) as b:
        s.cellDoubleClicked.emit(0, 0)
    assert b.args == [2]
    with qtbot.assertNotEmitted(s.frameRequested):
        s.cellDoubleClicked.emit(1, 0)   # frame_idx -1
    s.add_row({"frame_idx": 5, "t_sec": 1.0, "submerged": True, "curb_correlation": 0.1,
               "severity": "minor", "veto_usable": True, "veto_coverage": 0.0}, "dense")
    assert s.rowCount() == 1   # switching mode resets the table
```

Run: `.venv/Scripts/python -m pytest tests/gui/test_panels.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'floodsim.gui.panels'`.

- [ ] **Step 2: Write `floodsim/gui/panels.py`**

```python
"""Left tools panel, right verdict/state panel, and the samples table."""
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QAbstractItemView, QButtonGroup, QFormLayout, QFrame, QLabel,
                               QPushButton, QRadioButton, QTableWidget, QTableWidgetItem,
                               QVBoxLayout, QWidget)

from ..report import format_value
from .frame_view import Tool

OUTCOME_STYLE = {"upload": "#d62728", "clear": "#2ca02c", "blocked": "#7f7f7f",
                 "heartbeat": "#1f77b4", "skipped_no_ref": "#ff7f0e",
                 "submerged": "#d62728", "vetoed": "#7f7f7f"}
DASH = "–"


class ToolsPanel(QWidget):
    toolChanged = Signal(object)
    sampleClicked = Signal()
    scoreClicked = Signal()
    setReferenceClicked = Signal()
    resetStateClicked = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("<b>Tools</b>"))
        self._tools = list(Tool)
        self._group = QButtonGroup(self)
        self.tool_buttons: dict[Tool, QRadioButton] = {}
        for i, tool in enumerate(self._tools):
            button = QRadioButton(tool.value)
            self._group.addButton(button, i)
            self.tool_buttons[tool] = button
            layout.addWidget(button)
        self.tool_buttons[Tool.PAN].setChecked(True)
        self._group.idToggled.connect(self._on_tool)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        layout.addWidget(line)
        self.sample_btn = QPushButton("Sample")
        self.sample_btn.setToolTip("Full device sample here: retry on a later frame if obstructed; "
                                   "advances the heartbeat counter")
        self.score_btn = QPushButton("Score")
        self.score_btn.setToolTip("Veto + curb check on this frame only; device state untouched")
        self.set_reference_btn = QPushButton("Set reference")
        self.set_reference_btn.setToolTip("Use this frame as the dry reference")
        self.reset_btn = QPushButton("Reset state")
        self.reset_btn.setToolTip("Clear the heartbeat counter")
        for button, signal in ((self.sample_btn, self.sampleClicked), (self.score_btn, self.scoreClicked),
                               (self.set_reference_btn, self.setReferenceClicked),
                               (self.reset_btn, self.resetStateClicked)):
            button.clicked.connect(signal)
            layout.addWidget(button)
        layout.addStretch(1)

    def _on_tool(self, button_id: int, checked: bool) -> None:
        if checked:
            self.toolChanged.emit(self._tools[button_id])

    def set_actions_enabled(self, has_source: bool, has_reference: bool) -> None:
        for button in (self.sample_btn, self.set_reference_btn, self.reset_btn):
            button.setEnabled(has_source)
        self.score_btn.setEnabled(has_source and has_reference)


def _corr(value: float, threshold: float | None) -> str:
    return f"{value:.3f}" + (f" (threshold {threshold:.2f})" if threshold is not None else "")


def _veto(usable: bool, coverage: float, worst: str) -> str:
    text = f"{'usable' if usable else 'BLOCKED'}, coverage {coverage:.0%}"
    return text + (f" ({worst})" if worst != "other" else "")


class VerdictPanel(QWidget):
    rebuildClicked = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("<b>Verdict</b>"))
        form = QFormLayout()
        self.source_label = QLabel(DASH)
        self.outcome_label = QLabel(DASH)
        self.severity_label = QLabel(DASH)
        self.corr_label = QLabel(DASH)
        self.shift_label = QLabel(DASH)
        self.veto_label = QLabel(DASH)
        self.frames_label = QLabel(DASH)
        self.label_label = QLabel(DASH)
        for name, widget in (("", self.source_label), ("Outcome", self.outcome_label),
                             ("Severity", self.severity_label), ("Correlation", self.corr_label),
                             ("Shift", self.shift_label), ("Veto", self.veto_label),
                             ("Frames taken", self.frames_label), ("Label", self.label_label)):
            form.addRow(name, widget)
        layout.addLayout(form)

        layout.addWidget(QLabel("<b>State</b>"))
        state = QFormLayout()
        self.since_label = QLabel(DASH)
        self.reference_label = QLabel("none")
        self.firmware_label = QLabel(DASH)
        state.addRow("Since image", self.since_label)
        state.addRow("Reference", self.reference_label)
        state.addRow("Firmware", self.firmware_label)
        layout.addLayout(state)

        self.stale_banner = QFrame()
        self.stale_banner.setStyleSheet("background:#fff3cd; border:1px solid #d4a017;")
        banner = QVBoxLayout(self.stale_banner)
        self.stale_text = QLabel()
        self.stale_text.setWordWrap(True)
        self.rebuild_btn = QPushButton("Rebuild reference")
        self.rebuild_btn.clicked.connect(self.rebuildClicked)
        banner.addWidget(self.stale_text)
        banner.addWidget(self.rebuild_btn)
        self.stale_banner.hide()
        layout.addWidget(self.stale_banner)
        layout.addStretch(1)

    def _outcome(self, text: str, key: str | None) -> None:
        self.outcome_label.setText(text)
        color = OUTCOME_STYLE.get(key) if key else None
        self.outcome_label.setStyleSheet(
            f"color:white; background:{color}; padding:2px 6px; border-radius:3px;" if color else "")

    def show_score(self, score, label: str | None, threshold: float | None) -> None:
        self.source_label.setText("dense score (this frame)")
        self.label_label.setText(label or DASH)
        self.frames_label.setText(DASH)
        if score is None:
            self._outcome("no score", None)
            for w in (self.severity_label, self.corr_label, self.shift_label, self.veto_label):
                w.setText(DASH)
            return
        key = "vetoed" if not score.veto_usable else ("submerged" if score.submerged else "clear")
        self._outcome(key.upper(), key)
        self.severity_label.setText(score.severity)
        self.corr_label.setText(_corr(score.curb_correlation, threshold))
        self.shift_label.setText(f"dx {score.shift_dx:+d}  dy {score.shift_dy:+d}")
        self.veto_label.setText(_veto(score.veto_usable, score.veto_coverage, score.veto_worst_class))

    def show_device(self, row: dict, threshold: float | None) -> None:
        self.source_label.setText(f"device sample #{row['sample']} at {row['t_sec']:.2f} s")
        self._outcome(row["outcome"].upper(), row["outcome"])
        self.severity_label.setText(row["severity"])
        self.corr_label.setText(_corr(row["curb_correlation"], threshold) if row["has_reading"]
                                else f"{DASH} (no reading)")
        self.shift_label.setText(f"dx {row['shift_dx']:+d}  dy {row['shift_dy']:+d}")
        self.veto_label.setText(_veto(row["veto_usable"], row["veto_coverage"], row["veto_worst_class"]))
        self.frames_label.setText(str(row["frames_taken"]))
        self.label_label.setText(row.get("label") or DASH)

    def show_state(self, since_image: int, heartbeat_every: int | None) -> None:
        self.since_label.setText(f"{since_image} / {heartbeat_every}" if heartbeat_every else str(since_image))

    def show_reference(self, status) -> None:
        self.reference_label.setText(status.description if status.ready else "none (set a dry reference)")
        self.stale_banner.setVisible(status.stale)
        if status.stale:
            self.stale_text.setText(
                f"Reference was built under geometry v{status.built_under}; current is "
                f"v{status.geometry_version}. Scores still use the old reference, as the device "
                f"would until it recalibrates.")

    def show_firmware(self, fw: dict) -> None:
        commit = (fw.get("commit") or "unknown")[:7]
        dirty = fw.get("dirty")
        state = "dirty" if dirty else ("clean" if dirty is False else "?")
        self.firmware_label.setText(f"{commit} ({state})")


DEVICE_COLUMNS = [("#", "sample"), ("t (s)", "t_sec"), ("frame", "frame_idx"), ("outcome", "outcome"),
                  ("corr", "curb_correlation"), ("severity", "severity"), ("veto cov", "veto_coverage"),
                  ("label", "label")]
DENSE_COLUMNS = [("frame", "frame_idx"), ("t (s)", "t_sec"), ("submerged", "submerged"),
                 ("corr", "curb_correlation"), ("severity", "severity"), ("veto", "veto_usable"),
                 ("veto cov", "veto_coverage"), ("label", "label")]


class SamplesTable(QTableWidget):
    frameRequested = Signal(int)

    def __init__(self, parent=None):
        super().__init__(0, 0, parent)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.verticalHeader().hide()
        self.setMaximumHeight(200)
        self.cellDoubleClicked.connect(self._on_activated)
        self._rows: list[dict] = []
        self._mode: str | None = None

    def _columns(self):
        return DEVICE_COLUMNS if self._mode == "device" else DENSE_COLUMNS

    def set_rows(self, rows: list[dict], mode: str) -> None:
        self._mode = mode
        self._rows = []
        self.setRowCount(0)
        self.setColumnCount(len(self._columns()))
        self.setHorizontalHeaderLabels([c[0] for c in self._columns()])
        for row in rows:
            self.add_row(row, mode)

    def add_row(self, row: dict, mode: str) -> None:
        if mode != self._mode:
            self.set_rows([], mode)
        self._rows.append(row)
        i = self.rowCount()
        self.insertRow(i)
        for c, (_, key) in enumerate(self._columns()):
            self.setItem(i, c, QTableWidgetItem(format_value(key, row.get(key))))

    def _on_activated(self, row: int, _column: int) -> None:
        idx = self._rows[row].get("frame_idx", -1)
        if idx >= 0:
            self.frameRequested.emit(idx)
```

Run: `.venv/Scripts/python -m pytest tests/gui/test_panels.py -v`
Expected: 6 passed.

- [ ] **Step 3: Write the failing main-window tests `tests/gui/test_main_window.py`**

```python
import numpy as np
import pytest
from PySide6.QtCore import QPointF

from floodsim.gui import main_window as mw
from floodsim.gui.frame_view import Tool
from floodsim.gui.main_window import MainWindow
from floodsim.gui.session import Session
from floodsim.gui.worker import Worker
from floodsim.source import write_image
from tests.synth import dry_gray, flooded_gray, to_rgb, write_frame_folder

T = 15000


@pytest.fixture
def window(qtbot, core, build_info):
    worker = Worker()
    win = MainWindow(Session(core, build_info), worker, build_info)
    qtbot.addWidget(win)
    yield win
    worker.stop()


def _open(qtbot, win, path, **kw):
    win.open_path(path, **kw)
    qtbot.waitUntil(lambda: win._last_frame is not None, timeout=T)


def _set_reference(qtbot, win):
    win.tools.set_reference_btn.click()
    qtbot.waitUntil(lambda: win.verdict.reference_label.text().startswith("frame"), timeout=T)


def test_smoke_open_reference_score_edit_stale(qtbot, tmp_path, window):
    img = tmp_path / "dry.png"
    write_image(img, to_rgb(dry_gray()))
    _open(qtbot, window, img)
    _set_reference(qtbot, window)
    window.tools.score_btn.click()
    qtbot.waitUntil(lambda: window.verdict.corr_label.text().startswith("1.000"), timeout=T)
    window.frame_view.set_tool(Tool.EDIT_CURB)
    fv = window.frame_view
    fv._press(QPointF(560, 230))
    fv._move(QPointF(560, 250))
    fv._release(QPointF(560, 250))
    qtbot.waitUntil(lambda: not window.verdict.stale_banner.isHidden(), timeout=T)
    assert window.last_error is None


def test_manual_sample_adds_row_and_marker(qtbot, tmp_path, window):
    folder = write_frame_folder(tmp_path / "clip", [(0, dry_gray()), (60000, flooded_gray())])
    _open(qtbot, window, folder)
    _set_reference(qtbot, window)
    window.show_frame(1)
    qtbot.waitUntil(lambda: window._current == 1, timeout=T)
    window.tools.sample_btn.click()
    qtbot.waitUntil(lambda: window.samples.rowCount() == 1, timeout=T)
    assert window.verdict.outcome_label.text() == "UPLOAD"
    assert len(window.timeline.markers.data) == 1
    window.tools.reset_btn.click()
    qtbot.waitUntil(lambda: window.verdict.since_label.text() == "0 / 60", timeout=T)


def test_box_draw_and_delete(qtbot, tmp_path, window, monkeypatch):
    img = tmp_path / "dry.png"
    write_image(img, to_rgb(dry_gray()))
    _open(qtbot, window, img, no_detector=True)
    _set_reference(qtbot, window)
    monkeypatch.setattr(mw.QInputDialog, "getItem", lambda *a, **k: ("car", True))
    window.frame_view.set_tool(Tool.DRAW_BOX)
    fv = window.frame_view
    fv._press(QPointF(0, 0))
    fv._move(QPointF(1919, 1079))
    fv._release(QPointF(1919, 1079))
    qtbot.waitUntil(lambda: window._last_frame.boxes and window._last_frame.score
                    and not window._last_frame.score.veto_usable, timeout=T)
    window.frame_view.set_tool(Tool.DELETE_BOX)
    fv._press(QPointF(500, 500))
    qtbot.waitUntil(lambda: window._last_frame.boxes == [] and window._last_frame.score.veto_usable,
                    timeout=T)


def test_scrubbing_is_latest_wins(qtbot, tmp_path, window, monkeypatch):
    folder = write_frame_folder(tmp_path / "clip", [(i * 1000, dry_gray()) for i in range(6)])
    _open(qtbot, window, folder)
    calls = []
    real = window.session.frame
    monkeypatch.setattr(window.session, "frame", lambda idx, **kw: calls.append(idx) or real(idx, **kw))
    for _ in range(10):
        for idx in range(6):
            window.show_frame(idx)
    window.show_frame(3)
    qtbot.waitUntil(lambda: window._current == 3 and not window._loading, timeout=T)
    assert len(calls) <= 3 and calls[-1] == 3


def test_folder_without_times_asks_for_fps(qtbot, tmp_path, window, monkeypatch):
    folder = tmp_path / "plain"
    folder.mkdir()
    for name in ("a.png", "b.png"):
        write_image(folder / name, to_rgb(dry_gray()))
    monkeypatch.setattr(mw.QInputDialog, "getDouble", lambda *a, **k: (4.0, True))
    _open(qtbot, window, folder)
    assert window.timeline._times == [0.0, 0.25]
    assert window.last_error is None


def test_invalid_geometry_drag_snaps_back(qtbot, tmp_path, window):
    img = tmp_path / "dry.png"
    write_image(img, to_rgb(dry_gray()))
    _open(qtbot, window, img)
    qtbot.waitUntil(lambda: window.frame_view._geometry is not None, timeout=T)
    before = list(window.frame_view._geometry)
    window.frame_view.set_tool(Tool.EDIT_LANE)
    fv = window.frame_view
    fv._press(QPointF(560, 378))
    fv._move(QPointF(560, 100))
    fv._release(QPointF(560, 100))
    qtbot.waitUntil(lambda: window.last_error is not None, timeout=T)
    assert "below the curb" in window.last_error
    qtbot.waitUntil(lambda: window.frame_view._geometry == before, timeout=T)


def test_undecodable_frame_blanks_view(qtbot, tmp_path, window):
    folder = write_frame_folder(tmp_path / "clip", [(0, dry_gray())])
    (folder / "f000001_t0000001000.png").write_bytes(b"not a png")
    _open(qtbot, window, folder)
    window.show_frame(1)
    qtbot.waitUntil(lambda: window._current == 1, timeout=T)
    assert window._last_frame.rgb is None
    assert window.frame_view._pixmap.pixmap().isNull()
    assert window.last_error is None
```

Run: `.venv/Scripts/python -m pytest tests/gui/test_main_window.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'floodsim.gui.main_window'`.

- [ ] **Step 4: Write `floodsim/gui/main_window.py`**

```python
"""Layout A: tools | frame | verdict, with the timeline and samples table below."""
from __future__ import annotations

import datetime as dt
import threading
from pathlib import Path

from PySide6.QtCore import QRectF, Qt, QTimer
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (QDialog, QDialogButtonBox, QDoubleSpinBox, QFileDialog, QFormLayout,
                               QInputDialog, QLabel, QMainWindow, QMessageBox, QProgressBar,
                               QPushButton, QSpinBox, QSplitter, QToolButton, QVBoxLayout, QWidget)

from ..boxes import COCO_TO_FLOOD, CachedBox
from ..errors import FloodSimError
from ..geometry import Geometry, load_geometry, save_geometry
from ..modes import RunParams
from .frame_view import FrameView, OverlayToggles, Tool
from .panels import SamplesTable, ToolsPanel, VerdictPanel
from .timeline import TimelineWidget

BOX_CLASSES = list(COCO_TO_FLOOD)   # person, car, truck, bus, bicycle, motorcycle
VIDEO_FILTER = "Video (*.avi *.mp4 *.mkv *.mov);;All files (*)"
IMAGE_FILTER = "Images (*.jpg *.jpeg *.png *.bmp *.pgm *.tif *.tiff);;All files (*)"


class RunDialog(QDialog):
    def __init__(self, defaults: RunParams, parent=None):
        super().__init__(parent)
        self._mode = defaults.mode
        self.setWindowTitle(f"{defaults.mode.capitalize()} run")
        form = QFormLayout(self)
        self.period = QDoubleSpinBox()
        self.period.setRange(0.001, 86400.0)
        self.period.setDecimals(3)
        self.period.setSuffix(" s")
        self.period.setValue(defaults.period)
        self.retry = QDoubleSpinBox()
        self.retry.setRange(0.0, 3600.0)
        self.retry.setDecimals(3)
        self.retry.setSuffix(" s")
        self.retry.setValue(defaults.retry_gap)
        self.every = QSpinBox()
        self.every.setRange(1, 100000)
        self.every.setValue(defaults.every)
        if self._mode == "device":
            form.addRow("Sample period", self.period)
            form.addRow("Retry gap", self.retry)
        else:
            form.addRow("Score every Nth frame", self.every)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def params(self) -> RunParams:
        return RunParams(self._mode, self.period.value(), self.retry.value(), self.every.value())


class MainWindow(QMainWindow):
    def __init__(self, session, worker, build_info: dict):
        super().__init__()
        self.session = session
        self.worker = worker
        self.frame_view = FrameView()
        self.tools = ToolsPanel()
        self.verdict = VerdictPanel()
        self.timeline = TimelineWidget()
        self.samples = SamplesTable()
        self.last_error: str | None = None
        self._n = 0
        self._current = 0
        self._want: int | None = None
        self._loading = False
        self._diff = False
        self._last_frame = None
        self._overlay: dict | None = None
        self._dense_points: dict[int, tuple[float, float, bool]] = {}
        self._manual_rows: list[dict] = []
        self._run_device_rows: list[dict] = []
        self._cancel: threading.Event | None = None

        self._build_layout()
        self._build_menus()
        self._build_status(build_info)
        self._play = QTimer(self)
        self._play.setInterval(100)
        self._play.timeout.connect(self._play_tick)
        self._connect()
        self.verdict.show_firmware(build_info["firmware"])
        self.tools.set_actions_enabled(False, False)
        self.setWindowTitle("floodsim")
        self._refresh_overlay()
        self._refresh_reference()

    # -------------------------------------------------------------- layout
    def _build_layout(self) -> None:
        split = QSplitter(Qt.Horizontal)
        split.addWidget(self.tools)
        split.addWidget(self.frame_view)
        split.addWidget(self.verdict)
        split.setStretchFactor(1, 1)
        split.setSizes([180, 1100, 320])
        self.samples_toggle = QToolButton()
        self.samples_toggle.setText("Samples ▾")
        self.samples_toggle.setCheckable(True)
        self.samples.setVisible(False)
        self.samples_toggle.toggled.connect(self.samples.setVisible)
        central = QWidget()
        layout = QVBoxLayout(central)
        layout.addWidget(split, 1)
        layout.addWidget(self.timeline)
        layout.addWidget(self.samples_toggle)
        layout.addWidget(self.samples)
        self.setCentralWidget(central)

    def _action(self, menu, text, slot, shortcut=None, checkable=False, checked=False) -> QAction:
        action = QAction(text, self)
        if checkable:
            action.setCheckable(True)
            action.setChecked(checked)
            action.toggled.connect(slot)
        else:
            action.triggered.connect(lambda *_: slot())
        if shortcut:
            action.setShortcut(shortcut)
        menu.addAction(action)
        return action

    def _build_menus(self) -> None:
        bar = self.menuBar()
        m = bar.addMenu("&File")
        self._action(m, "Open video…", self._open_video, QKeySequence.Open)
        self._action(m, "Open image…", self._open_image)
        self._action(m, "Open folder…", self._open_folder)
        m.addSeparator()
        self._action(m, "Export run…", self._export_run)
        m.addSeparator()
        self._action(m, "Quit", self.close, QKeySequence.Quit)
        m = bar.addMenu("&Reference")
        self._action(m, "Set from current frame", self._set_reference)
        self._action(m, "Load image…", self._load_reference_image)
        self._action(m, "Rebuild under current geometry", self._rebuild_reference)
        m = bar.addMenu("&Geometry")
        self._action(m, "Load…", self._load_geometry)
        self._action(m, "Save…", self._save_geometry)
        self._action(m, "Reset to firmware default", self._reset_geometry)
        m = bar.addMenu("&Boxes")
        self._action(m, "Revert frame to detector boxes", self._revert_boxes)
        m = bar.addMenu("&Run")
        self._action(m, "Device run…", lambda: self._start_run("device"))
        self._action(m, "Dense run…", lambda: self._start_run("dense"))
        self._action(m, "Cancel run", self._cancel_run)
        m = bar.addMenu("&Labels")
        self._action(m, "Load labels…", self._load_labels)
        m = bar.addMenu("&View")
        defaults = OverlayToggles()
        for name in ("curb", "lane", "strip", "align", "boxes", "shift"):
            self._action(m, f"Show {name}", lambda on, n=name: self._toggle_overlay(n, on),
                         checkable=True, checked=getattr(defaults, name))
        m.addSeparator()
        self._action(m, "Difference from reference", self._toggle_diff, checkable=True)

    def _build_status(self, build_info: dict) -> None:
        bar = self.statusBar()
        self.detector_label = QLabel("")
        bar.addPermanentWidget(self.detector_label)
        self.progress = QProgressBar()
        self.progress.setMaximumWidth(240)
        self.progress.hide()
        bar.addPermanentWidget(self.progress)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.hide()
        self.cancel_btn.clicked.connect(self._cancel_run)
        bar.addPermanentWidget(self.cancel_btn)
        if build_info["dll"]["stale"]:
            bar.showMessage("Warning: the DLL is older than the firmware sources (--no-rebuild)")

    def _connect(self) -> None:
        self.worker.error.connect(self._show_error)
        self.worker.progress.connect(self._on_progress)
        self.tools.toolChanged.connect(self.frame_view.set_tool)
        self.tools.sampleClicked.connect(self._sample)
        self.tools.scoreClicked.connect(self._score)
        self.tools.setReferenceClicked.connect(self._set_reference)
        self.tools.resetStateClicked.connect(self._reset_state)
        self.verdict.rebuildClicked.connect(self._rebuild_reference)
        self.frame_view.geometryEdited.connect(self._on_geometry_edited)
        self.frame_view.boxDrawn.connect(self._on_box_drawn)
        self.frame_view.boxDeleteRequested.connect(self._on_box_delete)
        self.timeline.seekFrame.connect(self.show_frame)
        self.timeline.playToggled.connect(self._set_playing)
        self.samples.frameRequested.connect(self.show_frame)

    def _show_error(self, message: str) -> None:
        self.last_error = message
        box = QMessageBox(QMessageBox.Warning, "floodsim", message, parent=self)
        box.setModal(False)
        box.show()

    def _threshold(self) -> float | None:
        return self._overlay["submerged_below"] if self._overlay else None

    # ------------------------------------------------------------- opening
    def _open_video(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open video", "", VIDEO_FILTER)
        if path:
            self.open_path(Path(path))

    def _open_image(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open image", "", IMAGE_FILTER)
        if path:
            self.open_path(Path(path))

    def _open_folder(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "Open image folder")
        if path:
            self.open_path(Path(path))

    def open_path(self, path, *, fps=None, allow_stretch=False, no_detector=False) -> None:
        def failed(message: str) -> None:
            if "--fps" in message and fps is None:
                value, ok = QInputDialog.getDouble(
                    self, "Frame rate", "These images have no _t<ms> times in their names.\n"
                    "Frames per second:", 1.0, 0.001, 1000.0, 3)
                if ok:
                    self.open_path(path, fps=value, allow_stretch=allow_stretch, no_detector=no_detector)
                    return
            self._show_error(message)

        self.statusBar().showMessage(f"Opening {path}…")
        self.worker.submit(lambda: self.session.open(path, fps=fps, allow_stretch=allow_stretch,
                                                     no_detector=no_detector),
                           self._on_opened, failed)

    def _on_opened(self, info: dict) -> None:
        self._n = info["n_frames"]
        self._current = 0
        self._dense_points.clear()
        self._manual_rows = []
        self._run_device_rows = []
        self.timeline.set_times(info["times"])
        self.samples.set_rows([], "device")
        self.detector_label.setText("YOLO: on" if info["detector"] else "YOLO: off (cached/edited boxes only)")
        self.setWindowTitle(f"floodsim: {info['name']}")
        self.statusBar().showMessage(f"{info['n_frames']} frame(s)", 5000)
        self._refresh_reference()
        self.show_frame(0)

    # -------------------------------------------------------------- frames
    def show_frame(self, idx: int) -> None:
        if self._n == 0:
            return
        self._want = max(0, min(int(idx), self._n - 1))
        if not self._loading:
            self._load_next()

    def _load_next(self) -> None:
        if self._want is None:
            return
        idx, self._want = self._want, None
        self._loading = True
        diff = self._diff
        self.worker.submit(lambda: self.session.frame(idx, diff=diff), self._on_frame, self._on_frame_error)

    def _on_frame(self, fd) -> None:
        self._loading = False
        self._current = fd.idx
        self._last_frame = fd
        self.frame_view.set_image(fd.rgb)
        self.frame_view.set_boxes(fd.boxes)
        s = fd.score
        self.frame_view.set_shift((s.shift_dx, s.shift_dy) if s else None)
        self.timeline.set_frame(fd.idx)
        self.verdict.show_score(s, fd.label, self._threshold())
        if s is not None:
            self._dense_points[fd.idx] = (fd.t, s.curb_correlation, s.veto_usable)
            self._update_plot()
        if fd.rgb is None:
            self.statusBar().showMessage(f"frame {fd.idx} failed to decode", 5000)
        self._load_next()

    def _on_frame_error(self, message: str) -> None:
        self._loading = False
        self._show_error(message)
        self._load_next()

    def _update_plot(self) -> None:
        self.timeline.set_dense(list(self._dense_points.values()))
        self.timeline.set_device(self._run_device_rows + self._manual_rows)

    def _invalidate_scores(self) -> None:
        self._dense_points.clear()
        self._update_plot()
        self.show_frame(self._current)

    # ------------------------------------------------ overlay and reference
    def _refresh_overlay(self) -> None:
        self.worker.submit(self.session.overlay_geometry, self._on_overlay)

    def _on_overlay(self, overlay: dict) -> None:
        self._overlay = overlay
        self.frame_view.set_geometry_overlay(overlay)
        self.timeline.set_threshold(overlay["submerged_below"])

    def _refresh_reference(self) -> None:
        self.worker.submit(self.session.ref_status, self._on_ref_status)

    def _on_ref_status(self, status) -> None:
        self.verdict.show_reference(status)
        self.tools.set_actions_enabled(self._n > 0, status.ready)

    def _after_reference(self, status) -> None:
        self._on_ref_status(status)
        self._invalidate_scores()

    def _set_reference(self) -> None:
        if self._n == 0:
            return
        idx = self._current
        self.worker.submit(lambda: self.session.set_reference_frame(idx), self._after_reference)

    def _load_reference_image(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Dry reference image", "", IMAGE_FILTER)
        if path:
            self.worker.submit(lambda: self.session.set_reference_image(Path(path)), self._after_reference)

    def _rebuild_reference(self) -> None:
        self.worker.submit(self.session.rebuild_reference, self._after_reference)

    # ------------------------------------------------------------ geometry
    def _apply_geometry(self, g: Geometry) -> None:
        def done(overlay: dict) -> None:
            self._on_overlay(overlay)
            self._refresh_reference()
            self._invalidate_scores()

        def failed(message: str) -> None:
            self._show_error(message)
            self._refresh_overlay()   # snap the drawing back to the geometry in effect

        self.worker.submit(lambda: self.session.set_geometry(g), done, failed)

    def _on_geometry_edited(self, values: list) -> None:
        self._apply_geometry(Geometry.from_list(values))

    def _load_geometry(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Load geometry", "", "JSON (*.json);;All files (*)")
        if not path:
            return
        try:
            g = load_geometry(Path(path))
        except FloodSimError as e:
            self._show_error(str(e))
            return
        self._apply_geometry(g)

    def _save_geometry(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Save geometry", "geometry.json", "JSON (*.json)")
        if path:
            self.worker.submit(lambda: self.session.geometry, lambda g: save_geometry(g, Path(path)))

    def _reset_geometry(self) -> None:
        self.worker.submit(self.session.default_geometry, self._apply_geometry)

    # --------------------------------------------------------------- boxes
    def _current_boxes(self) -> list[CachedBox]:
        return [b for b, _, _ in self._last_frame.boxes] if self._last_frame else []

    def _on_box_drawn(self, rect: QRectF) -> None:
        if self._last_frame is None:
            return
        cls, ok = QInputDialog.getItem(self, "Box class", "Class:", BOX_CLASSES, 1, False)
        if not ok:
            return
        box = CachedBox(rect.left(), rect.top(), rect.right(), rect.bottom(), 1.0, cls, COCO_TO_FLOOD[cls])
        self._save_boxes(self._current_boxes() + [box])

    def _on_box_delete(self, index: int) -> None:
        boxes = self._current_boxes()
        if 0 <= index < len(boxes):
            del boxes[index]
            self._save_boxes(boxes)

    def _save_boxes(self, boxes: list[CachedBox]) -> None:
        idx = self._current
        self.worker.submit(lambda: self.session.set_boxes(idx, boxes), lambda _: self._after_box_edit(idx))

    def _revert_boxes(self) -> None:
        idx = self._current
        self.worker.submit(lambda: self.session.clear_edits(idx), lambda _: self._after_box_edit(idx))

    def _after_box_edit(self, idx: int) -> None:
        self._dense_points.pop(idx, None)
        self.show_frame(idx)

    # ---------------------------------------------------------- manual mode
    def _sample(self) -> None:
        if self._n == 0:
            return
        idx = self._current
        self.worker.submit(lambda: self.session.sample(idx), self._on_sample)

    def _on_sample(self, row: dict) -> None:
        self._manual_rows.append(row)
        self.verdict.show_device(row, self._threshold())
        self.verdict.show_state(row["since_image"], self._overlay["heartbeat_every"] if self._overlay else None)
        self.samples.add_row(row, "device")
        self._update_plot()

    def _score(self) -> None:
        if self._n == 0:
            return
        idx = self._current
        label = self._last_frame.label if self._last_frame else None
        self.worker.submit(lambda: self.session.score_frame(idx),
                           lambda s: self.verdict.show_score(s, label, self._threshold()))

    def _reset_state(self) -> None:
        every = self._overlay["heartbeat_every"] if self._overlay else None
        self.worker.submit(self.session.reset_device, lambda _: self.verdict.show_state(0, every))

    # ---------------------------------------------------------- batch runs
    def _start_run(self, mode: str) -> None:
        if self._n == 0 or self._cancel is not None:
            return
        period = self._overlay["sample_period_s"] if self._overlay else RunParams().period
        dialog = RunDialog(RunParams(mode=mode, period=period), self)
        if dialog.exec() != QDialog.Accepted:
            return
        params = dialog.params()
        cancel = self._cancel = threading.Event()
        self.progress.setRange(0, 0)
        self.progress.show()
        self.cancel_btn.show()
        progress = self.worker.progress.emit
        self.worker.submit(lambda: self.session.run_batch(params, progress=progress, cancel=cancel.is_set),
                           self._on_run_done, self._on_run_failed)

    def _on_progress(self, done: int, total: int) -> None:
        self.progress.setRange(0, total)
        self.progress.setValue(done)

    def _cancel_run(self) -> None:
        if self._cancel is not None:
            self._cancel.set()

    def _end_run(self) -> None:
        self.progress.hide()
        self.cancel_btn.hide()
        self._cancel = None

    def _on_run_done(self, result: dict) -> None:
        self._end_run()
        rows = result["rows"]
        if result["mode"] == "device":
            self._run_device_rows = rows
        else:
            for r in rows:
                self._dense_points[r["frame_idx"]] = (r["t_sec"], r["curb_correlation"], r["veto_usable"])
        self.samples.set_rows(rows, result["mode"])
        self.samples_toggle.setChecked(True)
        self._update_plot()
        message = f"{result['mode']} run: {len(rows)} row(s)"
        if result["cancelled"]:
            message += " (cancelled)"
        if result["decode_failures"]:
            message += f", {result['decode_failures']} failed frame reads"
        self.statusBar().showMessage(message)

    def _on_run_failed(self, message: str) -> None:
        self._end_run()
        self._show_error(message)

    # ------------------------------------------------------- labels/export
    def _load_labels(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Load labels", "", "CSV (*.csv);;All files (*)")
        if path:
            self.worker.submit(lambda: self.session.load_labels(Path(path)), self._on_labels)

    def _on_labels(self, result: dict) -> None:
        self.timeline.set_labels(result["spans"])
        self._manual_rows = result["manual_rows"]
        if result["run_rows"] is not None:
            self.samples.set_rows(result["run_rows"], result["run_mode"])
            if result["run_mode"] == "device":
                self._run_device_rows = result["run_rows"]
        if result["warnings"]:
            self.statusBar().showMessage("; ".join(result["warnings"]), 10000)
        self.show_frame(self._current)

    def _export_run(self) -> None:
        parent = QFileDialog.getExistingDirectory(self, "Export run into folder")
        if parent:
            out = Path(parent) / f"floodsim-{dt.datetime.now():%Y%m%d-%H%M%S}"
            self.worker.submit(lambda: self.session.export_run(out),
                               lambda path: self.statusBar().showMessage(f"wrote {path}"))

    # --------------------------------------------------------------- view
    def _toggle_overlay(self, name: str, on: bool) -> None:
        setattr(self.frame_view.toggles, name, on)
        self.frame_view.redraw()

    def _toggle_diff(self, on: bool) -> None:
        self._diff = on
        self.show_frame(self._current)

    def _set_playing(self, on: bool) -> None:
        if on:
            self._play.start()
        else:
            self._play.stop()

    def _play_tick(self) -> None:
        if self._loading:
            return
        if self._current >= self._n - 1:
            self.timeline.set_playing(False)
            return
        self.show_frame(self._current + 1)

    def keyPressEvent(self, event) -> None:
        if event.key() == Qt.Key_Left:
            self.show_frame(self._current - 1)
        elif event.key() == Qt.Key_Right:
            self.show_frame(self._current + 1)
        elif event.key() == Qt.Key_Space:
            self.timeline.set_playing(not self.timeline.play_btn.isChecked())
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event) -> None:
        self._play.stop()
        self._cancel_run()      # a running batch stops at its next frame
        self.worker.stop()
        super().closeEvent(event)
```

- [ ] **Step 5: Run the tests**

Run: `.venv/Scripts/python -m pytest tests/gui -v`
Expected: all pass (worker 5, frame view 9, timeline 7, panels 6, main window 7).

- [ ] **Step 6: Commit**

```bash
git add floodsim/gui/panels.py floodsim/gui/main_window.py tests/gui/test_panels.py tests/gui/test_main_window.py
git commit -m "feat(gui): main window with manual mode, geometry/box editing, verdict panel" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 6: Batch runs in the GUI, `floodsim gui`, README

**Files:**
- Create: `floodsim/gui/app.py`, `tests/gui/test_batch.py`
- Modify: `floodsim/cli.py` (add `gui`), `tests/test_cli.py` (gui help test), `README.md`

**Interfaces:**
- Consumes: `MainWindow._start_run`, `RunDialog`, `Session.run_batch/load_labels/export_run`.
- Produces: `floodsim.gui.app.run_gui(input_path=None, *, fps=None, allow_stretch=False, no_detector=False) -> int`; CLI `floodsim gui [input] [--fps] [--allow-stretch] [--no-detector]`.

- [ ] **Step 1: Write the failing tests `tests/gui/test_batch.py`**

```python
import time

import pytest
from PySide6.QtWidgets import QDialog

from floodsim.gui import main_window as mw
from floodsim.gui.main_window import MainWindow
from floodsim.gui.session import Session
from floodsim.gui.worker import Worker
from floodsim.report import read_samples
from tests.synth import dry_gray, flooded_gray, write_frame_folder

T = 20000


@pytest.fixture
def window(qtbot, core, build_info, monkeypatch):
    monkeypatch.setattr(mw.RunDialog, "exec", lambda self: QDialog.Accepted)
    worker = Worker()
    win = MainWindow(Session(core, build_info), worker, build_info)
    qtbot.addWidget(win)
    yield win
    worker.stop()


def _ready(qtbot, win, folder):
    win.open_path(folder, no_detector=True)
    qtbot.waitUntil(lambda: win._last_frame is not None, timeout=T)
    win.tools.set_reference_btn.click()
    qtbot.waitUntil(lambda: win.verdict.reference_label.text().startswith("frame"), timeout=T)


def test_dense_run_fills_plot_and_table(qtbot, tmp_path, window):
    folder = write_frame_folder(tmp_path / "clip", [(0, dry_gray()), (1000, dry_gray()),
                                                    (60000, flooded_gray()), (61000, flooded_gray())])
    _ready(qtbot, window, folder)
    window._start_run("dense")
    qtbot.waitUntil(lambda: window.samples.rowCount() == 4, timeout=T)
    x, y = window.timeline.curve.getData()
    assert list(x) == [0.0, 1.0, 60.0, 61.0] and list(y) == [pytest.approx(1.0, abs=1e-6)] * 2 + [0.0, 0.0]
    assert window.progress.isHidden()


def test_device_run_labels_and_export(qtbot, tmp_path, window, monkeypatch):
    folder = write_frame_folder(tmp_path / "clip", [(0, dry_gray()), (60000, flooded_gray())])
    labels = tmp_path / "labels.csv"
    labels.write_text("start_sec,end_sec,label\n0,30,DRY\n30,100,FLOODED\n")
    _ready(qtbot, window, folder)
    window._start_run("device")
    qtbot.waitUntil(lambda: len(window._run_device_rows) == 2, timeout=T)
    assert len(window.timeline.markers.data) == 2
    monkeypatch.setattr(mw.QFileDialog, "getOpenFileName", lambda *a, **k: (str(labels), ""))
    window._load_labels()
    qtbot.waitUntil(lambda: window.timeline._labels_item is not None, timeout=T)
    monkeypatch.setattr(mw.QFileDialog, "getExistingDirectory", lambda *a, **k: str(tmp_path / "exports"))
    (tmp_path / "exports").mkdir()
    window._export_run()
    qtbot.waitUntil(lambda: any((tmp_path / "exports").iterdir()), timeout=T)
    out = next((tmp_path / "exports").iterdir())
    qtbot.waitUntil(lambda: (out / "summary.txt").exists(), timeout=T)
    assert [r["label"] for r in read_samples(out)] == ["DRY", "FLOODED"]
    assert "TP=1" in (out / "summary.txt").read_text()


def test_close_during_run_cancels(qtbot, tmp_path, window):
    folder = write_frame_folder(tmp_path / "clip", [(i * 1000, dry_gray()) for i in range(3)])
    _ready(qtbot, window, folder)
    slow = window.session.run_batch

    def run_slowly(params, progress=None, cancel=None):
        def progress_and_wait(done, total):
            progress(done, total)
            time.sleep(0.5)
        return slow(params, progress=progress_and_wait, cancel=cancel)

    window.session.run_batch = run_slowly
    window.show()
    window._start_run("dense")
    qtbot.waitUntil(lambda: not window.progress.isHidden(), timeout=T)
    started = time.monotonic()
    window.close()
    assert time.monotonic() - started < 2.0
```

Run: `.venv/Scripts/python -m pytest tests/gui/test_batch.py -v`
Expected: PASS. These tests pin behaviour Task 5 already wired (run/labels/export actions; `closeEvent` cancels before `worker.stop()`); they are this task's regression net. If one fails, fix `main_window.py` before continuing. This task's red-green cycle is Steps 2–3.

- [ ] **Step 2: Write the failing CLI test**

Append to `tests/test_cli.py`:

```python
def test_gui_subcommand_is_registered():
    from floodsim.cli import build_parser
    args = build_parser().parse_args(["gui", "clip.avi", "--no-detector"])
    assert args.command == "gui" and args.no_detector is True and str(args.input) == "clip.avi"
```

Run: `.venv/Scripts/python -m pytest tests/test_cli.py::test_gui_subcommand_is_registered -v`
Expected: FAIL with `argument command: invalid choice: 'gui'`.

- [ ] **Step 3: Write `floodsim/gui/app.py` and the `gui` subcommand**

`floodsim/gui/app.py`:
```python
"""Entry point for `floodsim gui`."""
from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication

from ..core import load_core
from .main_window import MainWindow
from .session import Session
from .worker import Worker


def run_gui(input_path: Path | None = None, *, fps: float | None = None, allow_stretch: bool = False,
            no_detector: bool = False) -> int:
    app = QApplication.instance() or QApplication(sys.argv[:1])
    core, build_info = load_core()   # builds first if needed; a FloodSimError reaches the CLI
    worker = Worker()
    window = MainWindow(Session(core, build_info), worker, build_info)
    window.resize(1600, 950)
    window.show()
    if input_path is not None:
        window.open_path(Path(input_path), fps=fps, allow_stretch=allow_stretch, no_detector=no_detector)
    return app.exec()
```

In `floodsim/cli.py`, add to `build_parser()` before `return p`:

```python
    gui = sub.add_parser("gui", help="open the GUI")
    gui.add_argument("input", type=Path, nargs="?", help="image, folder of images, or video")
    gui.add_argument("--fps", type=float)
    gui.add_argument("--allow-stretch", action="store_true")
    gui.add_argument("--no-detector", action="store_true", help="do not load YOLO (cached/edited boxes only)")
```

add the command:

```python
def cmd_gui(args) -> int:
    try:
        from .gui.app import run_gui
    except ImportError as e:
        raise FloodSimError("GUI dependencies missing: pip install -e .[gui]") from e
    return run_gui(args.input, fps=args.fps, allow_stretch=args.allow_stretch, no_detector=args.no_detector)
```

and register it: `COMMANDS = {"run": cmd_run, "detect": cmd_detect, "compare": cmd_compare, "export": cmd_export, "gui": cmd_gui}`.

Run: `.venv/Scripts/python -m pytest tests/test_cli.py tests/gui -v`
Expected: all pass.

- [ ] **Step 4: Manual check of the real window**

Run: `.venv/Scripts/floodsim gui "D:/git/MS500_Flooding_Detection/dataset/frames/Dry-day.jpeg"`
Expected: the window opens (tools | frame | verdict, timeline below). Then check each of these:
1. Reference ▸ Set from current frame shows the reference as "frame 0 (0.00 s)".
2. Score shows CLEAR with correlation 1.000.
3. Edit curb line, drag an endpoint: the stale banner appears. Clicking Rebuild reference clears it.
4. View ▸ Show align shows the patch.
5. Close the window: the process exits.

Report anything that differs; this step is observation only.

- [ ] **Step 5: Add a GUI section to `README.md`** (after the "Use" section)

````markdown
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
````

- [ ] **Step 6: Run everything**

Run: `ctest --test-dir build/native --output-on-failure && .venv/Scripts/python -m pytest -v`
Expected: all pass.

- [ ] **Step 7: Commit**

```bash
git add floodsim/gui/app.py floodsim/cli.py tests/gui/test_batch.py tests/test_cli.py README.md
git commit -m "feat(gui): batch runs, labels, export, and the floodsim gui command" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

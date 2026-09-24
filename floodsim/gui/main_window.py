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

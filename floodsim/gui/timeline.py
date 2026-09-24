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

"""Transport controls and the correlation-over-time plot, on one time axis."""
from __future__ import annotations

import bisect
import datetime as dt

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget

import pyqtgraph as pg  # after PySide6, so pyqtgraph binds to it

OUTCOME_COLORS = {"upload": "#d62728", "clear": "#2ca02c", "blocked": "#7f7f7f", "heartbeat": "#1f77b4"}
LABEL_COLORS = {"DRY": "#b8860b", "WET": "#4682b4", "FLOODED": "#d62728", "INDETERMINATE": "#999999"}
Y_MIN, Y_MAX = -1.35, 1.15
MARKER_Y = 1.08
LABEL_Y0, LABEL_H = -1.33, 0.06


class ClockAxis(pg.AxisItem):
    """Seconds on the timeline shown as the time of day, counted from `start`."""

    def __init__(self, start: dt.datetime, with_date: bool):
        super().__init__("bottom")
        self.start = start
        self.with_date = with_date

    def tickStrings(self, values, scale, spacing):
        fmt = "%m-%d %H:%M" if self.with_date else "%H:%M"
        return [(self.start + dt.timedelta(seconds=float(v))).strftime(fmt) for v in values]


class TimelineWidget(QWidget):
    seekFrame = Signal(int)
    playToggled = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._times: list[float] = []
        self._start: dt.datetime | None = None
        self._gaps: list[tuple[float, float]] = []
        self._clips: list[tuple[int, str]] = []
        self._gap_items: list = []
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
        self.set_clock(None, [], [])   # a new input starts without clock mode; clips set it again
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
        if self._start is not None and self._clips:
            clock = self._start + dt.timedelta(seconds=t)
            k = bisect.bisect_right([first for first, _ in self._clips], idx) - 1
            self.time_label.setText(f"{clock:%H:%M:%S} · {self._clips[k][1]} · frame {idx + 1}/{len(self._times)}")
        else:
            self.time_label.setText(f"t = {t:.2f} s   frame {idx + 1}/{len(self._times)}")

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

    def frame_at_time(self, t: float) -> int | None:
        if not self._times:
            return None
        i = bisect.bisect_right(self._times, t + 1e-9) - 1
        return i if i >= 0 else None

    def seek_time(self, t: float) -> None:
        idx = self.frame_at_time(t)
        if any(a <= t < b for a, b in self._gaps):   # in a gap: the nearest frame with footage
            j = bisect.bisect_left(self._times, t)
            candidates = [i for i in (j - 1, j) if 0 <= i < len(self._times)]
            idx = min(candidates, key=lambda i: abs(self._times[i] - t))
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

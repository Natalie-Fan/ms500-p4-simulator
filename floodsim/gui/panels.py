"""Left tools panel, right verdict/state panel, and the samples table."""
from __future__ import annotations

import datetime as dt

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
                                   "advances the heartbeat counter; may learn a reference")
        self.score_btn = QPushButton("Score")
        self.score_btn.setToolTip("Veto + curb check on this frame only; device state untouched")
        self.set_reference_btn = QPushButton("Add reference")
        self.set_reference_btn.setToolTip("Add this frame as a manual dry baseline "
                                          "(the device's Capture Baseline)")
        self.reset_btn = QPushButton("Reset state")
        self.reset_btn.setToolTip("Clear the heartbeat counter and forget learned references")
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
        self._start = None
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
        self.matched_label = QLabel(DASH)
        self.matched_label.setWordWrap(True)
        self.label_label = QLabel(DASH)
        for name, widget in (("", self.source_label), ("Outcome", self.outcome_label),
                             ("Severity", self.severity_label), ("Correlation", self.corr_label),
                             ("Shift", self.shift_label), ("Veto", self.veto_label),
                             ("Frames taken", self.frames_label), ("Matched", self.matched_label),
                             ("Label", self.label_label)):
            form.addRow(name, widget)
        layout.addLayout(form)

        layout.addWidget(QLabel("<b>State</b>"))
        state = QFormLayout()
        self.since_label = QLabel(DASH)
        self.reference_label = QLabel("none")
        self.firmware_label = QLabel(DASH)
        state.addRow("Since image", self.since_label)
        state.addRow("Gallery", self.reference_label)
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
            for w in (self.severity_label, self.corr_label, self.shift_label, self.veto_label,
                      self.matched_label):
                w.setText(DASH)
            return
        key = "vetoed" if not score.veto_usable else ("submerged" if score.submerged else "clear")
        self._outcome(key.upper(), key)
        self.severity_label.setText(score.severity)
        self.corr_label.setText(_corr(score.curb_correlation, threshold))
        self.shift_label.setText(f"dx {score.shift_dx:+d}  dy {score.shift_dy:+d}")
        self.veto_label.setText(_veto(score.veto_usable, score.veto_coverage, score.veto_worst_class))
        self.matched_label.setText(f"slot {score.gallery_slot}" if score.gallery_slot >= 0 else DASH)

    def set_start_clock(self, start) -> None:
        """Clock time of t = 0 for a folder of clips; None for other inputs."""
        self._start = start

    def show_device(self, row: dict, threshold: float | None) -> None:
        at = f"{row['t_sec']:.2f} s"
        if self._start is not None:
            at = f"{self._start + dt.timedelta(seconds=row['t_sec']):%H:%M:%S} (t = {row['t_sec']:.0f} s)"
        self.source_label.setText(f"device sample #{row['sample']} at {at}")
        self._outcome(row["outcome"].upper(), row["outcome"])
        self.severity_label.setText(row["severity"])
        self.corr_label.setText(_corr(row["curb_correlation"], threshold) if row["has_reading"]
                                else f"{DASH} (no reading)")
        self.shift_label.setText(f"dx {row['shift_dx']:+d}  dy {row['shift_dy']:+d}")
        self.veto_label.setText(_veto(row["veto_usable"], row["veto_coverage"], row["veto_worst_class"]))
        self.frames_label.setText(str(row["frames_taken"]))
        matched = (f"slot {row['gallery_slot']} ({row['gallery_source']})"
                   if row["gallery_slot"] >= 0 else DASH)
        if row["admitted"]:
            matched += f"; learned a new reference ({row['gallery_count']} now)"
        if row["notice"]:
            matched += "; no confident match for a while: add a baseline for this lighting"
        if row.get("candidate_pending") and not row["admitted"]:
            matched += "; candidate pending: sample again to confirm"
        self.matched_label.setText(matched)
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
                  ("slot", "gallery_slot"), ("learned", "admitted"), ("notice", "notice"),
                  ("refs", "gallery_count"), ("pending", "candidate_pending"), ("label", "label")]
DENSE_COLUMNS = [("frame", "frame_idx"), ("t (s)", "t_sec"), ("submerged", "submerged"),
                 ("corr", "curb_correlation"), ("severity", "severity"), ("veto", "veto_usable"),
                 ("veto cov", "veto_coverage"), ("slot", "gallery_slot"), ("label", "label")]


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

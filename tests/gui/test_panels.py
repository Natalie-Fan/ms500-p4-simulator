import datetime as dt

from floodsim.gui.frame_view import Tool
from floodsim.gui.panels import SamplesTable, ToolsPanel, VerdictPanel
from floodsim.gui.session import RefStatus
from floodsim.results import ScoreResult

ROW = {"sample": 3, "t_sec": 60.0, "frame_idx": 2, "outcome": "upload", "frames_taken": 1,
       "has_reading": True, "curb_correlation": 0.21, "severity": "severe", "shift_dx": -3,
       "shift_dy": 1, "veto_usable": True, "veto_coverage": 0.04, "veto_worst_class": "car",
       "since_image": 0, "gallery_slot": 1, "gallery_source": "auto", "admitted": False,
       "notice": False, "gallery_count": 2, "label": None}


def test_verdict_device_row(qtbot):
    p = VerdictPanel()
    qtbot.addWidget(p)
    p.show_device(ROW, 0.45)
    assert p.outcome_label.text() == "UPLOAD"
    assert p.corr_label.text() == "0.210 (threshold 0.45)"
    assert p.veto_label.text() == "usable, coverage 4% (car)"
    assert p.shift_label.text() == "dx -3  dy +1"
    assert p.source_label.text() == "device sample #3 at 60.00 s"
    assert p.matched_label.text() == "slot 1 (auto)"
    p.show_device(dict(ROW, gallery_slot=0, gallery_source="manual", admitted=True, notice=True), 0.45)
    assert p.matched_label.text().startswith("slot 0 (manual); learned a new reference (2 now)")
    assert "add a baseline for this lighting" in p.matched_label.text()


def test_verdict_score_and_no_reading(qtbot):
    p = VerdictPanel()
    qtbot.addWidget(p)
    p.show_score(ScoreResult(False, 1.0, "car", 1.0, False, "none", 0, 0, 0), "DRY", 0.45)
    assert p.matched_label.text() == "slot 0"
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
    assert "Capture a new baseline" in p.stale_text.text()
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


def test_verdict_device_row_shows_clock_time(qtbot):
    p = VerdictPanel()
    qtbot.addWidget(p)
    p.set_start_clock(dt.datetime(2026, 6, 18, 16, 0, 0))
    p.show_device(dict(ROW, sample=165, t_sec=2920.0), 0.45)
    assert p.source_label.text() == "device sample #165 at 16:48:40 (t = 2920 s)"


def test_verdict_shows_pending_candidate(qtbot):
    p = VerdictPanel()
    qtbot.addWidget(p)
    p.show_device(dict(ROW, candidate_pending=True), 0.45)
    assert p.matched_label.text().endswith("; candidate pending: sample again to confirm")
    p.show_device(dict(ROW, candidate_pending=True, admitted=True), 0.45)
    assert "candidate pending" not in p.matched_label.text()


def test_verdict_panel_heartbeat_and_sn_fields(qtbot):
    from floodsim.gui.panels import VerdictPanel
    p = VerdictPanel()
    qtbot.addWidget(p)
    assert p.heartbeat_spin.minimum() == 0 and p.heartbeat_spin.maximum() == 1440
    assert p.heartbeat_spin.value() == 60 and p.sn_edit.text() == "SIM"
    with qtbot.waitSignal(p.heartbeatChanged) as hb:
        p.heartbeat_spin.setValue(5)
    assert hb.args == [5]
    with qtbot.waitSignal(p.snChanged) as sn:
        p.sn_edit.setText("CAM-1")
        p.sn_edit.editingFinished.emit()
    assert sn.args == ["CAM-1"]
    p.show_upload("from lines, 1360×360, full resolution")
    assert "1360" in p.upload_label.text()


def test_rect_problem():
    from floodsim.gui.upload_area import rect_problem
    assert rect_problem(0, 0, 960, 540, 32, 518400) is None
    assert "32" in rect_problem(0, 0, 31, 100, 32, 518400)
    assert "inside" in rect_problem(1900, 0, 100, 100, 32, 518400)
    assert "518400" in rect_problem(0, 0, 1000, 600, 32, 518400)

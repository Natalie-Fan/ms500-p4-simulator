import datetime as dt

import pytest

from floodsim.gui.timeline import ClockAxis, TimelineWidget


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

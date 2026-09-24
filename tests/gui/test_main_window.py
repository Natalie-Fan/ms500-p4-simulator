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
    qtbot.waitUntil(lambda: bool(window._last_frame.boxes and window._last_frame.score
                    and not window._last_frame.score.veto_usable), timeout=T)
    window.frame_view.set_tool(Tool.DELETE_BOX)
    fv._press(QPointF(500, 500))
    qtbot.waitUntil(lambda: bool(window._last_frame.boxes == [] and window._last_frame.score.veto_usable),
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


def test_save_geometry_error_is_shown(qtbot, tmp_path, window, monkeypatch):
    bad_path = str(tmp_path / "missing" / "geo.json")
    monkeypatch.setattr(mw.QFileDialog, "getSaveFileName", lambda *a, **k: (bad_path, ""))
    window._save_geometry()
    qtbot.waitUntil(lambda: window.last_error is not None, timeout=T)
    assert "FileNotFoundError" in window.last_error or "No such file" in window.last_error


def test_save_geometry_writes_current_geometry(qtbot, tmp_path, window, core, monkeypatch):
    from floodsim.geometry import Geometry, load_geometry

    out_path = tmp_path / "geo.json"
    monkeypatch.setattr(mw.QFileDialog, "getSaveFileName", lambda *a, **k: (str(out_path), ""))
    window._save_geometry()
    qtbot.waitUntil(lambda: out_path.exists(), timeout=T)
    saved = load_geometry(out_path)
    assert saved == Geometry.from_list(core.default_geometry())

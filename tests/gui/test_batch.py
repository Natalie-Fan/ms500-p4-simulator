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


def test_run_status_reports_rebuild_and_missing_boxes(qtbot, tmp_path, window):
    from floodsim.geometry import Geometry

    folder = write_frame_folder(tmp_path / "clip", [(0, dry_gray()), (60000, flooded_gray())])
    _ready(qtbot, window, folder)
    g = window.session.geometry.to_list()
    g[1] = 240.0
    window._apply_geometry(Geometry.from_list(g))
    qtbot.waitUntil(lambda: not window.verdict.stale_banner.isHidden(), timeout=T)
    window._start_run("dense")
    qtbot.waitUntil(lambda: window.samples.rowCount() == 2, timeout=T)
    message = window.statusBar().currentMessage()
    assert "reference rebuilt" in message and "2 frame(s) had no boxes" in message
    qtbot.waitUntil(lambda: window.verdict.stale_banner.isHidden(), timeout=T)

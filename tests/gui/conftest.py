import pytest

pytest.importorskip("PySide6")


@pytest.fixture
def worker(qtbot):
    from floodsim.gui.worker import Worker
    w = Worker()
    yield w
    w.stop()

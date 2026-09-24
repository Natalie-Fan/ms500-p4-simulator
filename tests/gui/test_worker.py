import threading

from floodsim.errors import FloodSimError


def test_runs_on_worker_thread(qtbot, worker):
    got = []
    worker.submit(threading.get_ident, got.append)
    qtbot.waitUntil(lambda: bool(got), timeout=5000)
    assert got[0] != threading.get_ident()


def test_error_goes_to_handler(qtbot, worker):
    errors = []

    def boom():
        raise FloodSimError("bad input")

    worker.submit(boom, on_error=errors.append)
    qtbot.waitUntil(lambda: bool(errors), timeout=5000)
    assert errors == ["bad input"]


def test_unhandled_error_signal(qtbot, worker):
    with qtbot.waitSignal(worker.error, timeout=5000) as blocker:
        worker.submit(lambda: 1 / 0)
    assert "ZeroDivisionError" in blocker.args[0]


def test_busy_changed(qtbot, worker):
    states, done = [], []
    worker.busy_changed.connect(states.append)
    worker.submit(lambda: 1, done.append)
    qtbot.waitUntil(lambda: bool(done), timeout=5000)
    assert states == [True, False] and not worker.busy


def test_results_arrive_in_submission_order(qtbot, worker):
    got = []
    for i in range(5):
        worker.submit(lambda i=i: i, got.append)
    qtbot.waitUntil(lambda: len(got) == 5, timeout=5000)
    assert got == [0, 1, 2, 3, 4]

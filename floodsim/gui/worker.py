"""One worker thread for every Session/DLL call: the firmware core keeps its
geometry in a global and is not re-entrant, so calls must never overlap."""
from __future__ import annotations

import traceback
from typing import Any, Callable

from PySide6.QtCore import QObject, QThread, Signal, Slot

from ..errors import FloodSimError


class _Executor(QObject):
    """Lives on the worker thread."""
    done = Signal(int, object)
    error = Signal(int, str)

    @Slot(int, object)
    def run(self, job: int, call: object) -> None:
        try:
            result = call()
        except FloodSimError as e:
            self.error.emit(job, str(e))
            return
        except Exception as e:  # shown to the user with its traceback
            self.error.emit(job, f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}")
            return
        self.done.emit(job, result)


class Worker(QObject):
    """Lives on the GUI thread; callbacks run on the GUI thread."""
    _submit = Signal(int, object)
    progress = Signal(int, int)
    busy_changed = Signal(bool)
    error = Signal(str)

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self._thread = QThread()
        self._exec = _Executor()
        self._exec.moveToThread(self._thread)
        self._submit.connect(self._exec.run)
        self._exec.done.connect(self._on_done)
        self._exec.error.connect(self._on_error)
        self._pending: dict[int, tuple[Callable | None, Callable | None]] = {}
        self._next = 0
        self._thread.start()

    @property
    def busy(self) -> bool:
        return bool(self._pending)

    def submit(self, call: Callable[[], Any], on_done: Callable[[Any], None] | None = None,
               on_error: Callable[[str], None] | None = None) -> int:
        job = self._next
        self._next += 1
        self._pending[job] = (on_done, on_error)
        if len(self._pending) == 1:
            self.busy_changed.emit(True)
        self._submit.emit(job, call)
        return job

    @Slot(int, object)
    def _on_done(self, job: int, result: object) -> None:
        on_done, _ = self._pending.pop(job)
        self._idle_check()
        if on_done:
            on_done(result)

    @Slot(int, str)
    def _on_error(self, job: int, message: str) -> None:
        _, on_error = self._pending.pop(job)
        self._idle_check()
        if on_error:
            on_error(message)
        else:
            self.error.emit(message)

    def _idle_check(self) -> None:
        if not self._pending:
            self.busy_changed.emit(False)

    def stop(self) -> None:
        if self._thread.isRunning():
            self._thread.quit()
            self._thread.wait()

"""Entry point for `floodsim gui`."""
from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication

from ..boxes import DEFAULT_MODEL
from ..core import load_core
from .main_window import MainWindow
from .session import Session
from .worker import Worker


def run_gui(input_path: Path | None = None, *, fps: float | None = None, allow_stretch: bool = False,
            no_detector: bool = False, model: str = DEFAULT_MODEL) -> int:
    app = QApplication.instance() or QApplication(sys.argv[:1])
    core, build_info = load_core()   # builds first if needed; a FloodSimError reaches the CLI
    worker = Worker()
    window = MainWindow(Session(core, build_info, model), worker, build_info)
    window.resize(1600, 950)
    window.show()
    if input_path is not None:
        window.open_path(Path(input_path), fps=fps, allow_stretch=allow_stretch, no_detector=no_detector)
    return app.exec()

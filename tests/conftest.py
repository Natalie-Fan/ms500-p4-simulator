import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")   # Qt tests run without a display

import pytest

from floodsim.core import load_core
from floodsim.geometry import Geometry


@pytest.fixture(scope="session")
def _core_session():
    return load_core()


@pytest.fixture
def core(_core_session):
    core, _ = _core_session
    core.set_geometry(Geometry.from_list(core.default_geometry()))
    return core


@pytest.fixture
def build_info(_core_session):
    return _core_session[1]

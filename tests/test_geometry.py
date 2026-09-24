import json

import pytest

from floodsim.errors import FloodSimError
from floodsim.geometry import Geometry, load_geometry, save_geometry

DEFAULT = [560.0, 230.0, 1920.0, 355.0, 560.0, 378.0, 1920.0, 446.0]


def test_round_trip(tmp_path):
    g = Geometry.from_list(DEFAULT)
    path = tmp_path / "geo.json"
    save_geometry(g, path)
    assert load_geometry(path) == g
    assert json.loads(path.read_text())["curb"] == {"x0": 560.0, "y0": 230.0, "x1": 1920.0, "y1": 355.0}


def test_accepts_firmware_web_response(tmp_path):
    # Shape returned by the firmware's GET handler, extra keys included.
    path = tmp_path / "web.json"
    path.write_text(json.dumps({
        "code": 0,
        "curb": {"x0": 560, "y0": 230, "x1": 1920, "y1": 355},
        "lane": {"x0": 560, "y0": 378, "x1": 1920, "y1": 446},
        "geometry_version": 3, "calibrated": True, "stale": False,
    }))
    assert load_geometry(path).to_list() == DEFAULT


def test_missing_number_is_an_error(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text('{"curb": {"x0": 1, "y0": 2, "x1": 3}, "lane": {"x0": 1, "y0": 2, "x1": 3, "y1": 4}}')
    with pytest.raises(FloodSimError, match="curb.y1"):
        load_geometry(path)


def test_not_json_is_an_error(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text("not json")
    with pytest.raises(FloodSimError, match="cannot read geometry"):
        load_geometry(path)

import json

import pytest

from floodsim.errors import FloodSimError
from floodsim.upload_area import (check_sn, load_upload_area, parse_upload_area, save_upload_area,
                                  upload_area_path)


def test_parse_upload_area_numbers():
    assert parse_upload_area("100,200,640,480") == (100, 200, 640, 480)
    assert parse_upload_area(" 1, 2 ,3,4 ") == (1, 2, 3, 4)


def test_parse_upload_area_json(tmp_path):
    p = tmp_path / "a.json"
    p.write_text(json.dumps({"configured": True, "roi": {"x": 1, "y": 2, "width": 300, "height": 400}}))
    assert parse_upload_area(str(p)) == (1, 2, 300, 400)


def test_parse_upload_area_json_unconfigured(tmp_path):
    p = tmp_path / "a.json"
    p.write_text(json.dumps({"configured": False, "roi": None, "effective": {}}))
    assert parse_upload_area(str(p)) is None


@pytest.mark.parametrize("bad", ["1,2,3", "a,b,c,d", "1.5,2,3,4", "", "missing.json"])
def test_parse_upload_area_rejects(bad):
    with pytest.raises(FloodSimError):
        parse_upload_area(bad)


def test_save_load_delete(tmp_path):
    p = upload_area_path(tmp_path / "clip.avi")
    assert p.name == "clip.avi.upload_area.json"
    save_upload_area(p, (10, 20, 300, 400))
    assert json.loads(p.read_text()) == {"configured": True,
                                         "roi": {"x": 10, "y": 20, "width": 300, "height": 400}}
    assert load_upload_area(p) == (10, 20, 300, 400)
    save_upload_area(p, None)
    assert not p.exists() and load_upload_area(p) is None


def test_check_sn():
    assert check_sn("MS500-H120_1") == "MS500-H120_1"
    for bad in ("", "has space", "a/b", "x" * 65):
        with pytest.raises(FloodSimError):
            check_sn(bad)

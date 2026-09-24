import pytest

from floodsim.boxes import (BoxProvider, BoxStore, CachedBox, Detection, cache_meta,
                            write_boxes_csv)
from floodsim.errors import FloodSimError
from floodsim.results import Box
from tests.synth import dry_gray, to_rgb


class FakeDetector:
    model_name = "fake.pt"
    conf = 0.25

    def __init__(self, dets):
        self.dets = dets
        self.calls = 0

    def detect(self, rgb_small):
        assert rgb_small.shape == (480, 640, 3)
        self.calls += 1
        return list(self.dets)


FINGERPRINT = {"path": "clip.avi", "size": 10, "mtime": 1.0}
RGB = to_rgb(dry_gray())


def _provider(core, tmp_path, dets, meta=None):
    det = FakeDetector(dets)
    store = BoxStore(tmp_path / "clip.avi", meta or cache_meta(FINGERPRINT, det.model_name, det.conf))
    return BoxProvider(core, store, det), det, store


def test_detector_boxes_scaled_to_frame(core, tmp_path):
    prov, det, _ = _provider(core, tmp_path, [Detection(10, 20, 30, 40, 0.8, "car")])
    assert prov.boxes_for(0, 0.0, lambda: RGB) == [Box(30.0, 45.0, 90.0, 90.0, pytest.approx(0.8), 2)]


def test_cache_reused_without_detector_call(core, tmp_path):
    prov, det, _ = _provider(core, tmp_path, [Detection(10, 20, 30, 40, 0.8, "car")])
    first = prov.boxes_for(0, 0.0, lambda: RGB)
    prov2, det2, store2 = _provider(core, tmp_path, [])
    assert store2.has_cache
    assert prov2.boxes_for(0, 0.0, lambda: RGB) == first
    assert det2.calls == 0


def test_frame_with_no_detections_is_cached(core, tmp_path):
    prov, det, _ = _provider(core, tmp_path, [])
    assert prov.boxes_for(3, 0.75, lambda: RGB) == []
    prov2, det2, _ = _provider(core, tmp_path, [Detection(0, 0, 1, 1, 0.9, "car")])
    assert prov2.boxes_for(3, 0.75, lambda: RGB) == []
    assert det2.calls == 0


def test_cache_invalidated_when_settings_change(core, tmp_path):
    prov, _, _ = _provider(core, tmp_path, [Detection(10, 20, 30, 40, 0.8, "car")])
    prov.boxes_for(0, 0.0, lambda: RGB)
    other = cache_meta(FINGERPRINT, "fake.pt", 0.5)
    prov2, det2, store2 = _provider(core, tmp_path, [], meta=other)
    assert not store2.has_cache
    assert prov2.boxes_for(0, 0.0, lambda: RGB) == []
    assert det2.calls == 1


def test_edited_boxes_win(core, tmp_path):
    prov, det, store = _provider(core, tmp_path, [Detection(10, 20, 30, 40, 0.8, "car")])
    prov.boxes_for(0, 0.0, lambda: RGB)
    store.set_edited(0, 0.0, [CachedBox(1, 2, 3, 4, 1.0, "person", 1)])
    prov2, _, store2 = _provider(core, tmp_path, [])
    assert store2.is_edited(0)
    assert prov2.boxes_for(0, 0.0, lambda: RGB) == [Box(1.0, 2.0, 3.0, 4.0, 1.0, 1)]
    store2.clear_edited(0)
    assert prov2.boxes_for(0, 0.0, lambda: RGB)[0].cls == 2


def test_missing_frame_without_detector_raises(core, tmp_path):
    store = BoxStore(tmp_path / "clip.avi", cache_meta(FINGERPRINT, "fake.pt", 0.25))
    with pytest.raises(FloodSimError, match="no detector"):
        BoxProvider(core, store, None).boxes_for(7, 1.0, lambda: RGB)


def test_none_provider_returns_nothing(core):
    prov = BoxProvider.none(core)
    assert prov.boxes_for(0, 0.0, lambda: RGB) == []
    assert prov.description == {"kind": "no-detector"}


def test_unknown_coco_class_maps_to_other(core, tmp_path):
    prov, _, _ = _provider(core, tmp_path, [Detection(0, 0, 10, 10, 0.9, "dog")])
    assert prov.boxes_for(0, 0.0, lambda: RGB)[0].cls == 0


def test_truncated_cache_line_is_skipped(core, tmp_path):
    prov, _, store = _provider(core, tmp_path, [Detection(10, 20, 30, 40, 0.8, "car")])
    prov.boxes_for(0, 0.0, lambda: RGB)
    prov.boxes_for(1, 0.25, lambda: RGB)
    text = store.cache_path.read_text()
    store.cache_path.write_text(text[: text.rstrip("\n").rfind(",")])  # cut the last row mid-way
    prov2, det2, _ = _provider(core, tmp_path, [])
    assert len(prov2.boxes_for(0, 0.0, lambda: RGB)) == 1
    assert prov2.boxes_for(1, 0.25, lambda: RGB) == []   # re-detected (fake now finds nothing)
    assert det2.calls == 1
    _, _, store3 = _provider(core, tmp_path, [])        # the append did not merge into the cut line
    assert store3.get(0)[0].flood_class == 2 and store3.get(1) == []


def test_write_boxes_csv_for_exe(tmp_path):
    path = tmp_path / "boxes.csv"
    write_boxes_csv(path, [(4, 120.0, [CachedBox(0, 0, 1919, 1079, 0.9, "car", 2)]), (5, 121.0, [])],
                    {"export": True})
    lines = path.read_text().splitlines()
    assert lines[1] == "frame_idx,t_sec,x0,y0,x1,y1,conf,coco_class,flood_class"
    assert lines[2] == "4,120.000,0,0,1919,1079,0.9,car,2"
    assert lines[3] == "5,121.000,,,,,,,"

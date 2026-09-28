import numpy as np
import pytest

from floodsim.modes import RunParams, UploadWriter, run_dense, run_device
from floodsim.results import DENSE_FIELDS, DEVICE_FIELDS
from floodsim.source import read_image_bgr
from tests.synth import CAR, MemorySource, dry_gray, flooded_gray, to_rgb

DRY, FLOOD = to_rgb(dry_gray()), to_rgb(flooded_gray())


class StaticBoxes:
    def __init__(self, by_idx):
        self.by_idx = by_idx

    def boxes_for(self, idx, t, load_rgb):
        return self.by_idx.get(idx, [])


def _golden_source():
    return MemorySource([(0, DRY), (1, DRY), (60, FLOOD), (61, FLOOD), (120, DRY), (121, DRY)])


@pytest.fixture
def ref(core):
    r = core.new_gallery()
    r.add_manual(dry_gray())
    return r


def test_device_matches_golden_scenario(core, ref):
    out = run_device(core, _golden_source(), StaticBoxes({4: [CAR]}), ref, RunParams(period=60.0))
    assert [r["outcome"] for r in out.rows] == ["clear", "upload", "clear"]
    assert [r["frames_taken"] for r in out.rows] == [1, 1, 2]
    assert [r["frame_idx"] for r in out.rows] == [0, 2, 5]
    assert [r["t_sec"] for r in out.rows] == [0.0, 60.0, 120.0]
    assert list(out.rows[0]) == DEVICE_FIELDS


def test_device_writes_what_would_be_uploaded(core, ref, tmp_path):
    uploads = UploadWriter(tmp_path, core.upload_downscale())
    run_device(core, _golden_source(), StaticBoxes({}), ref, RunParams(), uploads=uploads)
    files = sorted((tmp_path / "uploads").iterdir())
    assert [f.name for f in files] == ["00001_000000.jpg"]
    assert read_image_bgr(files[0]).shape == (540, 960, 3)


def test_device_heartbeat_on_60th_sample(core, ref):
    src = MemorySource([(t, DRY) for t in range(60)])
    out = run_device(core, src, StaticBoxes({}), ref, RunParams(period=1.0))
    assert len(out.rows) == 60
    assert out.rows[58]["outcome"] == "clear" and out.rows[59]["outcome"] == "heartbeat"


def test_device_without_reference_skips(core):
    out = run_device(core, _golden_source(), StaticBoxes({}), None, RunParams())
    assert {r["outcome"] for r in out.rows} == {"skipped_no_ref"}


def test_decode_failure_falls_back_to_retry_frame(core, ref):
    src = MemorySource([(0, None), (1, DRY)])
    out = run_device(core, src, StaticBoxes({}), ref, RunParams())
    assert out.rows[0]["outcome"] == "clear" and out.rows[0]["frames_taken"] == 2
    assert out.decode_failures == 1


def test_dense_every_second_frame(core, ref):
    out = run_dense(core, _golden_source(), StaticBoxes({4: [CAR]}), ref, RunParams(mode="dense", every=2))
    assert [r["frame_idx"] for r in out.rows] == [0, 2, 4]
    assert [r["submerged"] for r in out.rows] == [False, True, False]
    assert out.rows[2]["veto_usable"] is False
    assert list(out.rows[0]) == DENSE_FIELDS


def test_cancel_stops_run(core, ref):
    calls = []
    out = run_dense(core, _golden_source(), StaticBoxes({}), ref, RunParams(mode="dense"),
                    progress=lambda done, total: calls.append(done), cancel=lambda: len(calls) >= 2)
    assert out.cancelled and len(out.rows) == 2

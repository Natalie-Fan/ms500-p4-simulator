from pathlib import Path

import pytest

from floodsim.boxes import CachedBox
from floodsim.errors import FloodSimError
from floodsim.geometry import Geometry
from floodsim.gui.session import Session
from floodsim.modes import RunParams
from floodsim.report import read_samples
from tests.synth import dry_gray, flooded_gray, write_frame_folder

CAR = CachedBox(0.0, 0.0, 1919.0, 1079.0, 1.0, "car", 2)


@pytest.fixture
def folder(tmp_path):
    return write_frame_folder(tmp_path / "clip", [(0, dry_gray()), (1000, dry_gray()),
                                                  (60000, flooded_gray()), (61000, flooded_gray())])


@pytest.fixture
def session(core, build_info, folder):
    s = Session(core, build_info)
    info = s.open(folder, no_detector=True)
    assert info["n_frames"] == 4 and info["times"] == [0.0, 1.0, 60.0, 61.0]
    assert info["detector"] is False
    return s


def test_frame_without_reference_has_no_score(session):
    fd = session.frame(0)
    assert fd.score is None and fd.rgb.shape == (1080, 1920, 3) and fd.boxes == []
    with pytest.raises(FloodSimError, match="reference"):
        session.score_frame(0)


def test_reference_then_scores(session):
    st = session.set_reference_frame(0)
    assert st.ready and not st.stale and st.description == "frame 0 (0.00 s)"
    assert session.frame(0).score.curb_correlation == pytest.approx(1.0, abs=1e-6)
    assert session.frame(2).score.submerged is True
    assert session.score_frame(2).severity == "moderate"


def test_geometry_change_marks_reference_stale_and_rebuild_clears(session):
    session.set_reference_frame(0)
    g = session.geometry.to_list()
    g[1] = 240.0
    ov = session.set_geometry(Geometry.from_list(g))
    assert ov["geometry"][1] == 240.0
    st = session.ref_status()
    assert st.stale and (st.built_under, st.geometry_version) == (0, 1)
    assert not session.rebuild_reference().stale


def test_invalid_geometry_rejected(session):
    with pytest.raises(FloodSimError, match="invalid geometry"):
        session.set_geometry(Geometry.from_list([560, 230, 560, 355, 560, 378, 1920, 446]))
    assert session.geometry_version == 0
    assert session.overlay_geometry()["geometry"] == session.default_geometry().to_list()


def test_box_edit_invalidates_score(session):
    session.set_reference_frame(0)
    assert session.frame(0).score.veto_usable
    session.set_boxes(0, [CAR])
    fd = session.frame(0)
    assert fd.boxes == [(CAR, True, 1.0)]
    assert fd.score.veto_usable is False
    session.clear_edits(0)
    assert session.frame(0).score.veto_usable


def test_manual_sample_uses_device_state(core, build_info, session, folder):
    session.set_reference_frame(0)
    row = session.sample(0)
    assert (row["sample"], row["outcome"], row["since_image"]) == (0, "clear", 1)
    assert session.sample(2)["outcome"] == "upload"
    session.reset_device()
    assert session.sample(0)["since_image"] == 1
    fresh = Session(core, build_info)
    fresh.open(folder, no_detector=True)
    assert fresh.sample(0)["outcome"] == "skipped_no_ref"


def test_dense_batch_fills_cache_and_exports(session, tmp_path):
    session.set_reference_frame(0)
    calls = []
    res = session.run_batch(RunParams(mode="dense"), progress=lambda d, t: calls.append(d))
    assert len(res["rows"]) == 4 and calls[-1] == 4 and not res["cancelled"]
    out = session.export_run(tmp_path / "run")
    assert [r["submerged"] for r in read_samples(out)] == ["false", "false", "true", "true"]


def test_device_batch_export_writes_uploads(session, tmp_path):
    session.set_reference_frame(0)
    res = session.run_batch(RunParams(mode="device", period=60.0))
    assert [r["outcome"] for r in res["rows"]] == ["clear", "upload"]
    out = Path(session.export_run(tmp_path / "run"))
    assert len(list((out / "uploads").iterdir())) == 1


def test_dense_batch_needs_reference(session):
    with pytest.raises(FloodSimError, match="reference"):
        session.run_batch(RunParams(mode="dense"))


def test_labels_spans_and_row_labels(session, tmp_path):
    labels = tmp_path / "labels.csv"
    labels.write_text("start_sec,end_sec,label\n0,30,DRY\n30,100,FLOODED\n")
    session.set_reference_frame(0)
    session.sample(2)
    res = session.load_labels(labels)
    assert res["spans"] == [(0.0, 60.0, "DRY"), (60.0, 62.0, "FLOODED")]
    assert res["manual_rows"][0]["label"] == "FLOODED"
    assert session.frame(0).label == "DRY"


def test_diff_view(session):
    session.set_reference_frame(0)
    assert session.frame(0, diff=True).rgb.max() == 0
    assert session.frame(2, diff=True).rgb.max() > 0


def test_export_without_run_errors(session, tmp_path):
    with pytest.raises(FloodSimError, match="run"):
        session.export_run(tmp_path / "run")


def test_overlay_geometry(session):
    ov = session.overlay_geometry()
    assert len(ov["strip"][0]) == 275
    assert ov["align"] == (300, 0, 1700, 200)
    assert ov["heartbeat_every"] == 60 and ov["sample_period_s"] == 60.0
    assert ov["submerged_below"] == pytest.approx(0.45)

import json
from pathlib import Path

import numpy as np
import pytest

from floodsim.boxes import DEFAULT_CONF, DEFAULT_MODEL, BoxStore, CachedBox, cache_meta
from floodsim.cli import main as cli_main
from floodsim.errors import FloodSimError
from floodsim.geometry import Geometry
from floodsim.gui.session import Session
from floodsim.modes import RunParams
from floodsim.report import compare_runs, read_samples
from floodsim.source import open_source, write_image
from tests.synth import dry_gray, dusk_gray, flooded_gray, to_rgb, write_frame_folder

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
    assert st.ready and not st.stale
    assert st.description == "1 reference(s): 1 manual, 0 learned\n  manual: frame 0 (0.00 s)"
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


def _stale_geometry(session):
    g = session.geometry.to_list()
    g[1] = 240.0
    session.set_geometry(Geometry.from_list(g))


def test_batch_run_rebuilds_stale_reference(core, build_info, folder, session, tmp_path):
    # A flooded reference, then a curb moved above the flooded band: the stale
    # reference still holds the flat (flooded) strip, a rebuilt one the dry rows.
    moved = Geometry.from_list([560.0, 100.0, 1920.0, 150.0, 560.0, 160.0, 1920.0, 190.0])
    session.set_reference_frame(2)
    session.set_geometry(moved)
    assert session.ref_status().stale
    res = session.run_batch(RunParams(mode="dense"))
    fresh = Session(core, build_info)
    fresh.open(folder, no_detector=True)
    fresh.set_reference_frame(2)
    fresh.set_geometry(moved)
    fresh.rebuild_reference()
    explicit = fresh.run_batch(RunParams(mode="dense"))
    assert res["rows"] == explicit["rows"]
    assert res["reference_rebuilt"] is True and explicit["reference_rebuilt"] is False
    assert not session.ref_status().stale
    out = Path(session.export_run(tmp_path / "run"))
    assert json.loads((out / "run.json").read_text())["reference"]["rebuilt_for_run"] is True


def test_export_describes_the_run_not_the_current_state(session, tmp_path):
    session.set_reference_frame(0)
    used = session.geometry.to_json_dict()
    session.run_batch(RunParams(mode="dense"))
    _stale_geometry(session)
    out = Path(session.export_run(tmp_path / "run"))
    assert json.loads((out / "run.json").read_text())["geometry"] == used


def test_open_clears_labels_and_marks_frame_reference_as_previous_input(core, build_info, session, folder,
                                                                         tmp_path):
    labels = tmp_path / "labels.csv"
    labels.write_text("start_sec,end_sec,label\n0,100,DRY\n")
    session.set_reference_frame(0)
    session.load_labels(labels)
    other = write_frame_folder(tmp_path / "other", [(0, dry_gray()), (1000, flooded_gray())])
    session.open(other, no_detector=True)
    assert session.labels is None and session.labels_path is None
    assert session.frame(0).label is None
    assert session.ref_status().description.endswith("manual: frame 0 (0.00 s) of clip")
    session.run_batch(RunParams(mode="dense"))
    out = Path(session.export_run(tmp_path / "run"))
    info = json.loads((out / "run.json").read_text())
    assert info["reference"]["manual"][0]["input"] == str(folder) and info["labels"] is None


def test_boxes_description_without_detector_or_cache(session, tmp_path):
    session.set_reference_frame(0)
    res = session.run_batch(RunParams(mode="dense"))
    assert res["frames_without_boxes"] == 4
    out = Path(session.export_run(tmp_path / "run"))
    boxes = json.loads((out / "run.json").read_text())["boxes"]
    assert boxes["kind"] == "none" and boxes["frames_without_boxes"] == 4
    assert "edited_file" not in boxes


def test_boxes_description_with_partial_cache(core, build_info, folder, tmp_path):
    src = open_source(folder)
    store = BoxStore(folder, cache_meta(src.fingerprint(), DEFAULT_MODEL, DEFAULT_CONF))
    store.put_detected(0, 0.0, [])
    store.put_detected(1, 1.0, [CAR])
    s = Session(core, build_info)
    s.open(folder, no_detector=True)
    s.set_reference_frame(0)
    s.set_boxes(2, [CAR])
    res = s.run_batch(RunParams(mode="dense"))
    assert res["frames_without_boxes"] == 1
    out = Path(s.export_run(tmp_path / "run"))
    boxes = json.loads((out / "run.json").read_text())["boxes"]
    assert boxes["kind"] == "cache" and boxes["frames_without_boxes"] == 1
    assert boxes["edited_file"] == str(store.edited_path)


def test_export_of_cancelled_run_says_so(session, tmp_path):
    session.set_reference_frame(0)
    done = []
    res = session.run_batch(RunParams(mode="dense"), progress=lambda d, t: done.append(d),
                            cancel=lambda: len(done) >= 2)
    assert res["cancelled"] and len(res["rows"]) == 2
    out = Path(session.export_run(tmp_path / "run"))
    assert json.loads((out / "run.json").read_text())["cancelled"] is True
    assert "run cancelled: partial results" in (out / "summary.txt").read_text()


@pytest.mark.parametrize("mode", ["device", "dense"])
def test_gui_and_cli_runs_match(core, build_info, folder, tmp_path, mode):
    ref = tmp_path / "dry.png"
    write_image(ref, to_rgb(dry_gray()))
    s = Session(core, build_info)
    s.open(folder, no_detector=True)
    s.set_reference_image(ref)
    s.run_batch(RunParams(mode=mode))
    gui_out = Path(s.export_run(tmp_path / "gui"))
    cli_out = tmp_path / "cli"
    assert cli_main(["run", str(folder), "--mode", mode, "--reference", str(ref),
                     "--no-detector", "--out", str(cli_out)]) == 0
    assert read_samples(gui_out) and compare_runs(gui_out, cli_out) == []


def test_manual_sample_learns_and_reset_forgets(core, build_info, tmp_path):
    clip = write_frame_folder(tmp_path / "lit", [(0, dry_gray()), (60000, dusk_gray())])
    s = Session(core, build_info)
    s.open(clip, no_detector=True)
    s.set_reference_frame(0)
    row = s.sample(1)
    assert row["admitted"] and row["gallery_count"] == 2
    assert "2 reference(s): 1 manual, 1 learned" in s.ref_status().description
    assert s.frame(1).score.gallery_slot == 1      # frame scores see the learned reference
    # A batch run starts from the manual baselines only, and leaves the session gallery alone.
    res = s.run_batch(RunParams(mode="device", period=60.0))
    assert [r["admitted"] for r in res["rows"]] == [False, True]
    assert s.reference.count == 2
    s.reset_device()
    assert "1 reference(s): 1 manual, 0 learned" in s.ref_status().description
    s.set_reference_frame(1)
    assert "2 reference(s): 2 manual, 0 learned" in s.ref_status().description
    s.clear_references()
    assert not s.ref_status().ready


def test_reference_view_returns_stored_gray_as_rgb(session):
    session.set_reference_frame(0)
    session.set_reference_frame(2)
    assert session.ref_status().manual_names == ("frame 0 (0.00 s)", "frame 2 (60.00 s)")
    view = session.reference_view(1)
    assert view.shape == (1080, 1920, 3)
    assert np.array_equal(view[:, :, 0], flooded_gray()) and np.array_equal(view[:, :, 2], flooded_gray())
    with pytest.raises(FloodSimError, match="no manual reference 3"):
        session.reference_view(2)


def test_session_uses_its_model_for_the_box_cache(core, build_info, folder):
    s = Session(core, build_info, model="other.onnx")
    s.open(folder, no_detector=True)
    assert s.boxes.store.meta["model"] == "other.onnx"
    assert s.boxes.store.meta["input"] == [416, 416]

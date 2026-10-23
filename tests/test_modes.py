import datetime as dt
import json
from pathlib import Path

import numpy as np
import pytest

from floodsim.errors import FloodSimError
from floodsim.modes import RunParams, UploadWriter, heartbeat_readings, make_clock, run_dense, run_device
from floodsim.results import DENSE_FIELDS, DEVICE_FIELDS
from floodsim.source import open_source, read_image_bgr
from tests.synth import CAR, MemorySource, dry_gray, dusk_gray, flooded_gray, to_rgb, write_clip

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


CLOCK = lambda t: dt.datetime(2026, 10, 22, 12, 0, 0) + dt.timedelta(seconds=t)


def test_device_writes_what_would_be_uploaded(core, ref, tmp_path):
    uploads = UploadWriter(tmp_path, core, sn="SN1", clock=CLOCK)
    out = run_device(core, _golden_source(), StaticBoxes({}), ref, RunParams(), uploads=uploads)
    files = sorted(f.name for f in (tmp_path / "uploads").iterdir())
    assert files == ["SN1_flood_20261022_120100.jpg", "SN1_flood_20261022_120100.json"]
    assert read_image_bgr(tmp_path / "uploads" / files[0]).shape == (360, 1360, 3)
    desc = json.loads((tmp_path / "uploads" / files[1]).read_text())
    assert desc["kind"] == "flood" and desc["id"] == "000000" and desc["roi_source"] == "lines"
    assert [r["upload_file"] for r in out.rows] == ["", "SN1_flood_20261022_120100.jpg", ""]


def test_device_upload_uses_drawn_area(core, ref, tmp_path):
    uploads = UploadWriter(tmp_path, core, drawn=(0, 0, 640, 480), clock=CLOCK)
    run_device(core, _golden_source(), StaticBoxes({}), ref, RunParams(), uploads=uploads)
    jpg = next((tmp_path / "uploads").glob("*.jpg"))
    assert jpg.name.startswith("SIM_flood_") and read_image_bgr(jpg).shape == (480, 640, 3)


def test_upload_name_collision_gets_suffix(core, ref, tmp_path):
    src = MemorySource([(0.0, FLOOD), (0.2, FLOOD), (0.4, FLOOD)])
    uploads = UploadWriter(tmp_path, core, clock=CLOCK)
    out = run_device(core, src, StaticBoxes({}), ref, RunParams(period=0.2), uploads=uploads)
    assert [r["upload_file"] for r in out.rows] == [
        "SIM_flood_20261022_120000.jpg", "SIM_flood_20261022_120000_2.jpg", "SIM_flood_20261022_120000_3.jpg"]


def test_device_heartbeat_follows_minutes(core, ref):
    src = MemorySource([(t, DRY) for t in range(60)])
    out = run_device(core, src, StaticBoxes({}), ref, RunParams(period=1.0, heartbeat_minutes=1))
    assert len(out.rows) == 60
    assert out.rows[58]["outcome"] == "clear" and out.rows[59]["outcome"] == "heartbeat"


def test_device_heartbeat_off(core, ref):
    src = MemorySource([(t, DRY) for t in range(200)])
    out = run_device(core, src, StaticBoxes({}), ref, RunParams(period=1.0, heartbeat_minutes=0))
    assert "heartbeat" not in {r["outcome"] for r in out.rows}


def test_default_heartbeat_is_hourly_in_video_time(core, ref):
    src = MemorySource([(t, DRY) for t in range(120)])
    out = run_device(core, src, StaticBoxes({}), ref, RunParams(period=1.0))
    assert "heartbeat" not in {r["outcome"] for r in out.rows}     # 60 min @ 1 s = every 3600th


@pytest.mark.parametrize("minutes,period,expected", [(0, 60, 0), (60, 60, 60), (60, 5, 720),
                                                     (1, 120, 1), (1440, 60, 1440), (1, 61, 1)])
def test_heartbeat_readings(minutes, period, expected):
    assert heartbeat_readings(minutes, period) == expected


@pytest.mark.parametrize("minutes", [-1, 1441])
def test_heartbeat_readings_range(minutes):
    with pytest.raises(FloodSimError):
        heartbeat_readings(minutes, 60)


def test_make_clock(tmp_path):
    clip = tmp_path / "record_20261022_080000.avi"
    clip.write_bytes(b"")
    assert make_clock(MemorySource([]), clip)(90) == dt.datetime(2026, 10, 22, 8, 1, 30)
    other = tmp_path / "x.avi"
    other.write_bytes(b"")
    start = dt.datetime.fromtimestamp(other.stat().st_mtime)
    assert make_clock(MemorySource([]), other)(0) == start


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


def _day_of_clips(tmp_path):
    folder = tmp_path / "day"
    write_clip(folder, "20260618_160000", [dry_gray()] * 3)    # t 0..2
    write_clip(folder, "20260618_161000", [dusk_gray()] * 3)   # t 600..602
    write_clip(folder, "20260618_163000", [dusk_gray()] * 3)   # t 1800..1802
    return open_source(folder)


def test_device_run_skips_gaps_and_carries_learning(core, tmp_path):
    src = _day_of_clips(tmp_path)
    g = core.new_gallery()
    g.add_manual(dry_gray())
    out = run_device(core, src, StaticBoxes({}), g, RunParams(period=60.0))
    assert [r["sample"] for r in out.rows] == [0, 10, 30]
    assert [r["t_sec"] for r in out.rows] == [0.0, 600.0, 1800.0]
    assert out.skipped_no_footage == 28                       # 31 checks, 3 with footage
    assert out.rows[1]["admitted"] is False and out.rows[1]["candidate_pending"] is True
    assert out.rows[2]["admitted"] is True           # proven by the third clip, across the gap
    assert [src.clip_of(r["frame_idx"])[0] for r in out.rows] == [0, 1, 2]


def test_retry_past_clip_end_is_blocked(core, tmp_path):
    folder = tmp_path / "edge"
    write_clip(folder, "20260618_160000", [dry_gray()] * 2)     # t 0, 1
    write_clip(folder, "20260618_160959", [dry_gray()] * 2)     # t 599, 600: footage to 601
    src = open_source(folder)
    g = core.new_gallery()
    g.add_manual(dry_gray())
    out = run_device(core, src, StaticBoxes({3: [CAR]}), g, RunParams(period=600.0))
    edge = out.rows[1]
    assert (edge["t_sec"], edge["frame_idx"]) == (600.0, 3)
    assert (edge["outcome"], edge["frames_taken"]) == ("blocked", 2)   # retry at 601 is in the gap


def test_dense_run_covers_every_clip_frame(core, tmp_path):
    src = _day_of_clips(tmp_path)
    g = core.new_gallery()
    g.add_manual(dry_gray())
    out = run_dense(core, src, StaticBoxes({}), g, RunParams(mode="dense"))
    assert [r["frame_idx"] for r in out.rows] == list(range(9))


def test_single_video_runs_are_unchanged(core, ref):
    out = run_device(core, _golden_source(), StaticBoxes({}), ref, RunParams(period=60.0))
    assert out.skipped_no_footage == 0

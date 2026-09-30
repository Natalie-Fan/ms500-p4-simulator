import datetime as dt
import logging
import shutil

import numpy as np
import pytest

from floodsim.clips import MultiClipSource
from floodsim.errors import FloodSimError
from floodsim.source import open_source, write_image
from tests.synth import dry_gray, dusk_gray, to_rgb, write_clip


def _day(tmp_path):
    """23:59:50 (3 dry frames, t 0..2) and 00:05:00 the next day (2 dusk frames, t 310..311)."""
    folder = tmp_path / "day"
    write_clip(folder, "20260618_235950", [dry_gray()] * 3)
    write_clip(folder, "20260619_000500", [dusk_gray()] * 2)
    return folder


def test_folder_of_clips_is_one_timeline(tmp_path):
    src = open_source(_day(tmp_path))
    assert isinstance(src, MultiClipSource)
    assert src.n_frames == 5
    assert [src.timestamp(i) for i in range(5)] == [0.0, 1.0, 2.0, 310.0, 311.0]
    assert src.start_clock == dt.datetime(2026, 6, 18, 23, 59, 50)
    assert src.clock(310.0) == dt.datetime(2026, 6, 19, 0, 5, 0)
    assert src.frame_name(3) == "record_20260619_000500.avi#0"
    assert src.clip_of(4) == (1, 1, 1.0)
    assert np.array_equal(src.read_rgb(3), to_rgb(dusk_gray()))
    assert np.array_equal(src.read_rgb(0), to_rgb(dry_gray()))
    assert [c["name"] for c in src.fingerprint()["clips"]] == [
        "record_20260618_235950.avi", "record_20260619_000500.avi"]
    assert src.clip_bounds() == [(0, "record_20260618_235950.avi"), (3, "record_20260619_000500.avi")]


def test_footage_and_gaps(tmp_path):
    src = open_source(_day(tmp_path))
    assert src.has_footage(0.0) and src.has_footage(2.9)
    assert not src.has_footage(3.0) and not src.has_footage(100.0) and not src.has_footage(-1.0)
    assert src.has_footage(310.0) and src.has_footage(311.5) and not src.has_footage(312.0)
    assert src.index_at(2.9) == 2 and src.index_at(100.0) is None and src.index_at(310.5) == 3
    assert src.gaps() == [(3.0, 310.0)]


def test_clips_with_different_frame_rates(tmp_path):
    folder = tmp_path / "fps"
    write_clip(folder, "20260618_120000", [dry_gray()] * 2, fps=2.0)   # t 0, 0.5; footage to 1.0
    write_clip(folder, "20260618_120010", [dry_gray()] * 2, fps=1.0)   # t 10, 11; footage to 12.0
    src = open_source(folder)
    assert [src.timestamp(i) for i in range(4)] == [0.0, 0.5, 10.0, 11.0]
    assert src.has_footage(0.9) and not src.has_footage(1.0)
    assert src.has_footage(11.9) and not src.has_footage(12.0)


def test_overlap_drops_the_earlier_clips_frames(tmp_path, caplog):
    folder = tmp_path / "overlap"
    write_clip(folder, "20260618_120000", [dry_gray()] * 5)    # t 0..4
    write_clip(folder, "20260618_120003", [dusk_gray()] * 2)   # starts at t 3
    with caplog.at_level(logging.WARNING):
        src = open_source(folder)
    assert [src.timestamp(i) for i in range(src.n_frames)] == [0.0, 1.0, 2.0, 3.0, 4.0]
    assert src.frame_name(3) == "record_20260618_120003.avi#0"
    assert "record_20260618_120000.avi" in caplog.text and "overlaps" in caplog.text


def test_at_most_two_clips_open(tmp_path):
    folder = tmp_path / "many"
    for minute in range(4):
        write_clip(folder, f"20260618_12{minute:02d}00", [dry_gray()] * 2)
    src = open_source(folder)
    for idx in (0, 2, 4, 6, 0, 7, 3):
        assert np.array_equal(src.read_rgb(idx), to_rgb(dry_gray()))
        assert sum(src.clip_source(i)._cap is not None for i in range(4)) <= 2


def test_bad_names_are_listed(tmp_path):
    folder = tmp_path / "bad"
    write_clip(folder, "20260618_120000", [dry_gray()])
    write_clip(folder, "20260618_120100", [dry_gray()]).rename(folder / "cam1.avi")
    write_clip(folder, "20260618_120200", [dry_gray()]).rename(folder / "record_20261399_120200.avi")
    with pytest.raises(FloodSimError) as e:
        open_source(folder)
    assert "cam1.avi" in str(e.value) and "record_20261399_120200.avi" in str(e.value)


def test_same_start_rejected(tmp_path):
    folder = tmp_path / "same"
    clip = write_clip(folder, "20260618_120000", [dry_gray()])
    shutil.copy(clip, folder / "record_20260618_120000.mkv")
    with pytest.raises(FloodSimError, match="same start time"):
        open_source(folder)


def test_videos_and_images_together_rejected(tmp_path):
    folder = tmp_path / "mixed"
    write_clip(folder, "20260618_120000", [dry_gray()])
    write_image(folder / "a.png", to_rgb(dry_gray()))
    with pytest.raises(FloodSimError, match="both videos and images"):
        open_source(folder)


def test_empty_folder_rejected(tmp_path):
    (tmp_path / "empty").mkdir()
    with pytest.raises(FloodSimError, match="no image or video files"):
        open_source(tmp_path / "empty")


def test_broken_clip_named(tmp_path):
    folder = tmp_path / "broken"
    write_clip(folder, "20260618_120000", [dry_gray()])
    (folder / "record_20260618_120100.avi").write_bytes(b"not a video")
    with pytest.raises(FloodSimError, match="record_20260618_120100.avi"):
        open_source(folder)

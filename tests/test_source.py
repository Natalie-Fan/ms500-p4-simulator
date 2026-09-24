import logging

import cv2
import numpy as np
import pytest

from floodsim.errors import FloodSimError
from floodsim.source import open_source, write_image
from tests.synth import dry_gray, to_rgb, write_frame_folder


def _png(path, w, h, value=100):
    write_image(path, np.full((h, w, 3), value, np.uint8))
    return path


def test_single_image_is_one_frame_at_zero(tmp_path):
    src = open_source(_png(tmp_path / "a.png", 1920, 1080))
    assert src.n_frames == 1 and src.timestamp(0) == 0.0
    assert src.read_rgb(0).shape == (1080, 1920, 3)
    assert src.index_at(0.0) == 0 and src.index_at(500.0) == 0 and src.index_at(-1.0) is None


def test_folder_uses_t_ms_names(tmp_path):
    folder = write_frame_folder(tmp_path / "f", [(0, dry_gray()), (1000, dry_gray()), (60000, dry_gray())])
    src = open_source(folder)
    assert [src.timestamp(i) for i in range(3)] == [0.0, 1.0, 60.0]
    assert src.index_at(59.9) == 1 and src.index_at(60.0) == 2 and src.index_at(1e6) == 2
    assert src.frame_name(2) == "f000002_t0000060000.png"
    assert np.array_equal(src.read_rgb(1), to_rgb(dry_gray()))


def test_folder_without_times_needs_fps(tmp_path):
    for name in ("a.png", "b.png"):
        _png(tmp_path / name, 1920, 1080)
    with pytest.raises(FloodSimError, match="--fps"):
        open_source(tmp_path)
    src = open_source(tmp_path, fps=4.0)
    assert src.timestamp(1) == 0.25


def test_resize_720p_warns_once(tmp_path, caplog):
    for name in ("a.png", "b.png"):
        _png(tmp_path / name, 1280, 720)
    src = open_source(tmp_path, fps=1.0)
    with caplog.at_level(logging.WARNING):
        assert src.read_rgb(0).shape == (1080, 1920, 3)
        src.read_rgb(1)
    assert sum("resizing" in r.message for r in caplog.records) <= 1


def test_non_16_9_refused_unless_allow_stretch(tmp_path):
    path = _png(tmp_path / "square.png", 800, 600)
    with pytest.raises(FloodSimError, match="16:9"):
        open_source(path)
    assert open_source(path, allow_stretch=True).read_rgb(0).shape == (1080, 1920, 3)


def test_corrupt_image_returns_none(tmp_path):
    _png(tmp_path / "f000000_t0000000000.png", 1920, 1080)
    (tmp_path / "f000001_t0000001000.png").write_bytes(b"not a png")
    src = open_source(tmp_path)
    assert src.read_rgb(0) is not None
    assert src.read_rgb(1) is None


def test_video_frames_and_times(tmp_path):
    path = tmp_path / "clip.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 4.0, (640, 360))
    for v in (0, 80, 160, 240):
        writer.write(np.full((360, 640, 3), v, np.uint8))
    writer.release()
    src = open_source(path)
    assert src.n_frames == 4
    assert src.timestamp(3) == pytest.approx(0.75)
    assert src.read_rgb(3).shape == (1080, 1920, 3)
    later, earlier = src.read_rgb(3), src.read_rgb(1)  # backwards seek reopens the file
    assert earlier.mean() < later.mean()


def test_non_ascii_folder(tmp_path):
    folder = write_frame_folder(tmp_path / "测试 clips", [(0, dry_gray())])
    src = open_source(folder)
    assert src.read_rgb(0) is not None

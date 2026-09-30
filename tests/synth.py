"""Synthetic frames, identical to native/tests/synth.c."""
from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np

from floodsim.results import Box
from floodsim.source import FrameSource, write_image

FRAME_H, FRAME_W = 1080, 1920
FLOOD_Y0, FLOOD_Y1 = 195, 500
CAR = Box(0.0, 0.0, 1919.0, 1079.0, 0.9, 2)


def _texture(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    h = (x * np.uint32(73856093)) ^ (y * np.uint32(19349663))
    h ^= h >> np.uint32(13)
    h *= np.uint32(0x5BD1E995)
    h ^= h >> np.uint32(15)
    return ((h >> np.uint32(8)) & np.uint32(0xFF)).astype(np.uint8)


@lru_cache(maxsize=1)
def _dry() -> np.ndarray:
    y, x = np.mgrid[0:FRAME_H, 0:FRAME_W].astype(np.uint32)
    g = _texture(x, y)
    g.flags.writeable = False
    return g


def relit_gray(dry_weight: int, other_weight: int, seed: int) -> np.ndarray:
    """Dry, kerb rows blended with an unrelated texture (a lighting change); correlation
    against dry_gray() is about dry_weight / sqrt(dry_weight^2 + other_weight^2)."""
    g = _dry().copy()
    y, x = np.mgrid[FLOOD_Y0:FLOOD_Y1, 0:FRAME_W].astype(np.uint32)
    other = _texture(x + np.uint32(seed), y + np.uint32(2 * seed)).astype(np.uint32)
    rows = g[FLOOD_Y0:FLOOD_Y1].astype(np.uint32)
    g[FLOOD_Y0:FLOOD_Y1] = ((dry_weight * rows + other_weight * other) //
                            (dry_weight + other_weight)).astype(np.uint8)
    return g


def dusk_gray() -> np.ndarray:
    return relit_gray(4, 3, 7777)   # ~0.80: learnable


def night_gray() -> np.ndarray:
    return relit_gray(2, 3, 4242)   # ~0.55: ambiguous


def dry_gray() -> np.ndarray:
    return _dry()


def flooded_gray() -> np.ndarray:
    g = _dry().copy()
    g[FLOOD_Y0:FLOOD_Y1, :] = 128
    return g


def to_rgb(gray: np.ndarray) -> np.ndarray:
    """Equal channels, so the firmware gray formula returns `gray` exactly."""
    return np.repeat(gray[:, :, None], 3, axis=2)


class MemorySource(FrameSource):
    """In-memory frames for runner tests; None marks a frame that fails to decode."""

    def __init__(self, frames):
        self.path = Path("memory")
        self._times = [float(t) for t, _ in frames]
        self._rgb = [rgb for _, rgb in frames]
        self.n_frames = len(frames)

    def timestamp(self, idx):
        return self._times[idx]

    def read_rgb(self, idx):
        return self._rgb[idx]

    def fingerprint(self):
        return {"path": "memory", "n": self.n_frames}


def write_frame_folder(folder: Path, frames) -> Path:
    """frames: list of (t_ms, gray). Lossless PNGs named f<i>_t<ms>.png."""
    folder.mkdir(parents=True, exist_ok=True)
    for i, (t_ms, gray) in enumerate(frames):
        write_image(folder / f"f{i:06d}_t{t_ms:010d}.png", to_rgb(gray))
    return folder


def write_clip(folder: Path, start: str, grays, fps: float = 1.0) -> Path:
    """Lossless (FFV1) clip named record_<start>.avi, e.g. start="20260618_164320". Gray in all
    three channels, so the firmware gray formula returns each frame exactly."""
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"record_{start}.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"FFV1"), fps, (FRAME_W, FRAME_H))
    assert writer.isOpened(), "this OpenCV build cannot write FFV1"
    for gray in grays:
        writer.write(np.repeat(gray[:, :, None], 3, axis=2))
    writer.release()
    return path

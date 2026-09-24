"""Synthetic frames, identical to native/tests/synth.c."""
from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import numpy as np

from floodsim.results import Box
from floodsim.source import FrameSource, write_image

FRAME_H, FRAME_W = 1080, 1920
FLOOD_Y0, FLOOD_Y1 = 195, 500
CAR = Box(0.0, 0.0, 1919.0, 1079.0, 0.9, 2)


@lru_cache(maxsize=1)
def _dry() -> np.ndarray:
    y, x = np.mgrid[0:FRAME_H, 0:FRAME_W].astype(np.uint32)
    h = (x * np.uint32(73856093)) ^ (y * np.uint32(19349663))
    h ^= h >> np.uint32(13)
    h *= np.uint32(0x5BD1E995)
    h ^= h >> np.uint32(15)
    g = ((h >> np.uint32(8)) & np.uint32(0xFF)).astype(np.uint8)
    g.flags.writeable = False
    return g


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

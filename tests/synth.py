"""Synthetic frames, identical to native/tests/synth.c."""
from __future__ import annotations

from functools import lru_cache

import numpy as np

from floodsim.results import Box

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

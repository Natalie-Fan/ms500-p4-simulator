"""A folder of record_YYYYMMDD_HHMMSS clips as one timeline, each clip at its real start time."""
from __future__ import annotations

import bisect
import datetime as dt
import logging
import re
from pathlib import Path

import numpy as np

from .errors import FloodSimError
from .source import FrameSource, Normalizer, VideoSource

_RECORD = re.compile(r"^record_(\d{8})_(\d{6})\.[A-Za-z0-9]+$")
_MAX_OPEN = 2   # clips holding an open VideoCapture at once
log = logging.getLogger(__name__)


def clip_start(path: Path) -> dt.datetime | None:
    """Local start time from record_YYYYMMDD_HHMMSS.<ext>, or None if the name does not match."""
    m = _RECORD.match(path.name)
    if not m:
        return None
    try:
        return dt.datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S")
    except ValueError:
        return None


class MultiClipSource(FrameSource):
    """Frames numbered continuously across clips; times are seconds since the first clip's start."""

    def __init__(self, path: Path, files: list[Path], fps: float | None, normalizer: Normalizer):
        self.path = path
        starts = {f: clip_start(f) for f in files}
        bad = sorted(f.name for f, s in starts.items() if s is None)
        if bad:
            raise FloodSimError(f"{path}: clip names must be record_YYYYMMDD_HHMMSS.<ext>; "
                                f"not: {', '.join(bad)}")
        ordered = sorted(files, key=lambda f: starts[f])
        for a, b in zip(ordered, ordered[1:]):
            if starts[a] == starts[b]:
                raise FloodSimError(f"{path}: {a.name} and {b.name} have the same start time")
        self.start_clock = starts[ordered[0]]

        self._clips: list[VideoSource] = []
        offsets: list[float] = []
        for n, f in enumerate(ordered, 1):
            log.info("counting frames: %s (%d/%d)", f.name, n, len(ordered))
            clip = VideoSource(f, fps, normalizer)
            clip.read_rgb(0)   # fail fast on a bad aspect ratio, at open, not hours into a run
            clip.close()
            self._clips.append(clip)
            offsets.append((starts[f] - self.start_clock).total_seconds())

        self._times: list[float] = []
        self._first: list[int] = []                    # global index of each clip's first frame
        self._spans: list[tuple[float, float]] = []    # footage [start, end) per clip
        for i, clip in enumerate(self._clips):
            times = [round((offsets[i] + clip.timestamp(j)) * 1000) / 1000 for j in range(clip.n_frames)]
            if i + 1 < len(self._clips) and times[-1] >= offsets[i + 1]:
                # Keeps >= 1 frame: starts are distinct and sorted, and a clip's first frame
                # time is its own offset, which is below the next clip's offset.
                kept = bisect.bisect_left(times, offsets[i + 1])
                log.warning("%s overlaps %s; dropping its last %d frame(s)",
                            clip.path.name, self._clips[i + 1].path.name, len(times) - kept)
                times = times[:kept]
            self._first.append(len(self._times))
            self._times.extend(times)
            self._spans.append((times[0], times[-1] + 1.0 / clip.fps))
        self.n_frames = len(self._times)
        self._span_starts = [s for s, _ in self._spans]
        self._open: list[int] = []                     # clip indices with an open capture, oldest first

    # ---------------------------------------------------------------- lookup
    def clip_of(self, idx: int) -> tuple[int, int, float]:
        ci = bisect.bisect_right(self._first, idx) - 1
        local = idx - self._first[ci]
        return ci, local, self._clips[ci].timestamp(local)

    def clip_source(self, ci: int) -> VideoSource:
        return self._clips[ci]

    def clip_bounds(self) -> list[tuple[int, str]]:
        return [(first, clip.path.name) for first, clip in zip(self._first, self._clips)]

    def has_footage(self, t: float) -> bool:
        ci = bisect.bisect_right(self._span_starts, t + 1e-9) - 1
        return ci >= 0 and t < self._spans[ci][1]

    def index_at(self, t: float) -> int | None:
        """Latest frame at or before t, but never across a gap."""
        return super().index_at(t) if self.has_footage(t) else None

    def gaps(self) -> list[tuple[float, float]]:
        return [(a[1], b[0]) for a, b in zip(self._spans, self._spans[1:]) if b[0] > a[1]]

    def clock(self, t: float) -> dt.datetime:
        return self.start_clock + dt.timedelta(seconds=t)

    # ---------------------------------------------------------------- frames
    def read_rgb(self, idx: int) -> np.ndarray | None:
        ci, local, _ = self.clip_of(idx)
        if ci in self._open:
            self._open.remove(ci)
        self._open.append(ci)
        while len(self._open) > _MAX_OPEN:
            self._clips[self._open.pop(0)].close()
        return self._clips[ci].read_rgb(local)

    def frame_name(self, idx: int) -> str:
        ci, local, _ = self.clip_of(idx)
        return f"{self._clips[ci].path.name}#{local}"

    def fingerprint(self) -> dict:
        return {"name": self.path.name, "clips": [clip.fingerprint() for clip in self._clips]}

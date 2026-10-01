"""Device-faithful and dense runners over a frame source."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import cv2
import numpy as np

from .results import DeviceResult, ScoreResult
from .source import write_image

Progress = Callable[[int, int], None]
Cancel = Callable[[], bool]


@dataclass
class RunParams:
    mode: str = "device"
    period: float = 60.0
    retry_gap: float = 1.0
    every: int = 1


@dataclass
class RunOutput:
    rows: list[dict] = field(default_factory=list)
    decode_failures: int = 0
    cancelled: bool = False
    skipped_no_footage: int = 0


def device_row(sample: int, t: float, r: DeviceResult) -> dict:
    return {"sample": sample, "t_sec": t, "frame_idx": r.frame_idx, "outcome": r.outcome,
            "frames_taken": r.frames_taken, "has_reading": r.has_reading,
            "curb_correlation": r.curb_correlation, "severity": r.severity,
            "shift_dx": r.shift_dx, "shift_dy": r.shift_dy, "veto_usable": r.veto_usable,
            "veto_coverage": r.veto_coverage, "veto_worst_class": r.veto_worst_class,
            "since_image": r.since_image, "gallery_slot": r.gallery_slot,
            "gallery_source": r.gallery_source, "admitted": r.admitted, "notice": r.notice,
            "gallery_count": r.gallery_count, "candidate_pending": r.candidate_pending}


def dense_row(idx: int, t: float, s: ScoreResult) -> dict:
    return {"frame_idx": idx, "t_sec": t, "veto_usable": s.veto_usable,
            "veto_coverage": s.veto_coverage, "veto_worst_class": s.veto_worst_class,
            "curb_correlation": s.curb_correlation, "submerged": s.submerged,
            "severity": s.severity, "shift_dx": s.shift_dx, "shift_dy": s.shift_dy,
            "gallery_slot": s.gallery_slot}


class FrameFeeder:
    """Gray frame + boxes for a frame index, counting failed decodes."""

    def __init__(self, core, source, boxes):
        self.core = core
        self.source = source
        self.boxes = boxes
        self.decode_failures = 0

    def frame(self, idx: int):
        rgb = self.source.read_rgb(idx)
        if rgb is None:
            self.decode_failures += 1
            return None
        boxes = self.boxes.boxes_for(idx, self.source.timestamp(idx), lambda: rgb)
        return self.core.rgb_to_gray(rgb), boxes, idx


def sample_device_at(device, ref, feeder: FrameFeeder, t: float, retry_gap: float) -> DeviceResult:
    """One firmware sample: attempt 0 is the frame at t, the retry the frame at t + retry_gap."""
    times = (t, t + retry_gap)

    def provider(attempt: int):
        idx = feeder.source.index_at(times[min(attempt, 1)])
        return None if idx is None else feeder.frame(idx)

    return device.sample(ref, provider)


class UploadWriter:
    """What the camera would send: the frame decimated by FLOOD_ROI_DOWNSCALE with a
    6-hex sequence label (flood_roi_composite + flood_report_upload, flood_task.c:228).
    Encoded by OpenCV and labelled with an OpenCV font, so not byte-identical to the P4."""

    def __init__(self, out_dir: Path, downscale: int):
        self.dir = Path(out_dir) / "uploads"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.downscale = downscale
        self._seq = 0

    def write(self, sample: int, rgb: np.ndarray) -> Path:
        label = f"{self._seq & 0xFFFFFF:06X}"
        self._seq += 1
        canvas = np.ascontiguousarray(rgb[::self.downscale, ::self.downscale, ::-1])
        cv2.putText(canvas, label, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        path = self.dir / f"{sample:05d}_{label}.jpg"
        write_image(path, canvas)
        return path


def run_device(core, source, boxes, ref, params: RunParams, *, uploads: UploadWriter | None = None,
               progress: Progress | None = None, cancel: Cancel | None = None) -> RunOutput:
    feeder = FrameFeeder(core, source, boxes)
    device = core.new_device()
    out = RunOutput()
    total = int((source.duration + 1e-9) // params.period) + 1
    has_footage = getattr(source, "has_footage", None)
    for k in range(total):
        if cancel and cancel():
            out.cancelled = True
            break
        t = k * params.period
        if has_footage is not None and not has_footage(t):
            out.skipped_no_footage += 1   # a gap between clips: nothing to show the device
            if progress:
                progress(k + 1, total)
            continue
        r = sample_device_at(device, ref, feeder, t, params.retry_gap)
        out.rows.append(device_row(k, t, r))
        if uploads and r.outcome in ("upload", "heartbeat") and r.frame_idx >= 0:
            rgb = source.read_rgb(r.frame_idx)
            if rgb is not None:
                uploads.write(k, rgb)
        if progress:
            progress(k + 1, total)
    out.decode_failures = feeder.decode_failures
    return out


def run_dense(core, source, boxes, ref, params: RunParams, *,
              progress: Progress | None = None, cancel: Cancel | None = None) -> RunOutput:
    feeder = FrameFeeder(core, source, boxes)
    out = RunOutput()
    indices = range(0, source.n_frames, params.every)
    for n, idx in enumerate(indices):
        if cancel and cancel():
            out.cancelled = True
            break
        got = feeder.frame(idx)
        if got is not None:
            gray, frame_boxes, _ = got
            out.rows.append(dense_row(idx, source.timestamp(idx), core.score(ref, gray, frame_boxes)))
        if progress:
            progress(n + 1, len(indices))
    out.decode_failures = feeder.decode_failures
    return out

"""Device-faithful and dense runners over a frame source."""
from __future__ import annotations

import datetime as dt
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import cv2
import numpy as np

from .clips import clip_start
from .errors import FloodSimError
from .results import DeviceResult, ScoreResult

Progress = Callable[[int, int], None]
Cancel = Callable[[], bool]


@dataclass
class RunParams:
    mode: str = "device"
    period: float = 60.0
    retry_gap: float = 1.0
    every: int = 1
    heartbeat_minutes: int = 60


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
            "gallery_count": r.gallery_count, "candidate_pending": r.candidate_pending, "upload_file": ""}


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


def heartbeat_readings(minutes: int, period_s: float) -> int:
    """flood_task.c:342: minutes * 60000 / FLOOD_SAMPLE_PERIOD_MS, rounded down. 0 stays 0 (off);
    any other value gives at least 1, so a period longer than the interval still sends heartbeats."""
    if not 0 <= minutes <= 1440:
        raise FloodSimError(f"heartbeat must be 0-1440 minutes, got {minutes}")
    if minutes == 0:
        return 0
    return max(1, int(minutes * 60 // period_s))


def make_clock(source, input_path) -> Callable[[float], dt.datetime]:
    """Local wall time of a video offset: a record_YYYYMMDD_HHMMSS clip's start, else the input's
    modified time (2000-01-01 when there is no file, e.g. in-memory test sources)."""
    start = getattr(source, "start_clock", None) or clip_start(Path(input_path))
    if start is None:
        p = Path(input_path)
        start = dt.datetime.fromtimestamp(p.stat().st_mtime) if p.exists() else dt.datetime(2000, 1, 1)
    return lambda t: start + dt.timedelta(seconds=float(t))


def upload_run_info(core, params: RunParams, drawn, sn: str) -> dict:
    """run.json fields for the heartbeat and upload settings."""
    return {"heartbeat": {"minutes": params.heartbeat_minutes,
                          "readings": heartbeat_readings(params.heartbeat_minutes, params.period)},
            "upload_area": dict(zip(("x", "y", "width", "height"), drawn)) if drawn else None,
            "upload_area_used": core.upload_roi(drawn).as_dict(), "sn": sn}


class UploadWriter:
    """What the camera would send (flood_upload.c): the upload crop of the frame, named and described
    by the firmware's flood_upload_meta.c. JPEG is OpenCV quality 90, so not byte-identical to the P4."""

    def __init__(self, out_dir: Path, core, *, drawn=None, sn: str = "SIM",
                 clock: Callable[[float], dt.datetime]):
        self.dir = Path(out_dir) / "uploads"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.core, self.drawn, self.sn, self.clock = core, drawn, sn, clock
        self._seq = 0
        self._used: set[str] = set()

    def _unique(self, name: str) -> str:
        stem, n = name[:-4], 2
        while name in self._used:
            name = f"{stem}_{n}.jpg"
            n += 1
        self._used.add(name)
        return name

    def write(self, row: dict, rgb: np.ndarray) -> str:
        heartbeat = row["outcome"] == "heartbeat"
        roi = self.core.upload_roi(self.drawn)
        crop = self.core.crop(rgb, roi)
        upload_id = f"{self._seq & 0xFFFFFF:06X}"
        self._seq += 1
        local = self.clock(row["t_sec"])
        name = self._unique(self.core.upload_filename(self.sn, heartbeat, local))
        desc = self.core.upload_description(
            heartbeat=heartbeat, id=upload_id, correlation=row["curb_correlation"],
            severity=row["severity"], slot=row["gallery_slot"], roi=roi,
            captured_at=int(local.timestamp()))
        cv2.imwrite(str(self.dir / name), np.ascontiguousarray(crop[:, :, ::-1]),
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
        (self.dir / f"{name[:-4]}.json").write_text(desc + "\n", encoding="utf-8")
        return name


def run_device(core, source, boxes, ref, params: RunParams, *, uploads: UploadWriter | None = None,
               progress: Progress | None = None, cancel: Cancel | None = None) -> RunOutput:
    feeder = FrameFeeder(core, source, boxes)
    device = core.new_device()
    device.set_heartbeat(heartbeat_readings(params.heartbeat_minutes, params.period))
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
        row = device_row(k, t, r)
        out.rows.append(row)
        if uploads and r.outcome in ("upload", "heartbeat") and r.frame_idx >= 0:
            rgb = source.read_rgb(r.frame_idx)
            if rgb is not None:
                row["upload_file"] = uploads.write(row, rgb)
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

"""Plain result types shared by the core, the runners and the reports."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Box:
    """A detection in 1920x1080 frame coordinates; cls is a flood_class_t value."""
    x0: float
    y0: float
    x1: float
    y1: float
    confidence: float
    cls: int


@dataclass(frozen=True)
class ScoreResult:
    veto_usable: bool
    veto_coverage: float
    veto_worst_class: str
    curb_correlation: float
    submerged: bool
    severity: str
    shift_dx: int
    shift_dy: int
    gallery_slot: int          # best-matching gallery slot, -1 if none


@dataclass(frozen=True)
class DeviceResult:
    outcome: str
    frames_taken: int
    has_reading: bool
    curb_correlation: float
    severity: str
    shift_dx: int
    shift_dy: int
    veto_usable: bool
    veto_coverage: float
    veto_worst_class: str
    since_image: int
    frame_idx: int
    gallery_slot: int          # slot that matched, -1 unless has_reading
    gallery_source: str        # "manual", "auto" or "none"
    admitted: bool             # this sample learned a new AUTO reference
    notice: bool               # this sample raised the "no confident match" notice
    gallery_count: int         # references in the gallery after this sample


# Column order of samples.csv; identical to flood_sim.exe's JSON keys (minus "mode").
DEVICE_FIELDS = [
    "sample", "t_sec", "frame_idx", "outcome", "frames_taken", "has_reading",
    "curb_correlation", "severity", "shift_dx", "shift_dy", "veto_usable",
    "veto_coverage", "veto_worst_class", "since_image",
    "gallery_slot", "gallery_source", "admitted", "notice", "gallery_count",
]
DENSE_FIELDS = [
    "frame_idx", "t_sec", "veto_usable", "veto_coverage", "veto_worst_class",
    "curb_correlation", "submerged", "severity", "shift_dx", "shift_dy", "gallery_slot",
]

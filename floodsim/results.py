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
    candidate_pending: bool       # a learning candidate awaits its confirming reading


# Column order of samples.csv; identical to flood_sim.exe's JSON keys (minus "mode").
DEVICE_FIELDS = [
    "sample", "t_sec", "frame_idx", "outcome", "frames_taken", "has_reading",
    "curb_correlation", "severity", "shift_dx", "shift_dy", "veto_usable",
    "veto_coverage", "veto_worst_class", "since_image",
    "gallery_slot", "gallery_source", "admitted", "notice", "gallery_count",
    "candidate_pending", "upload_file",
]
DENSE_FIELDS = [
    "frame_idx", "t_sec", "veto_usable", "veto_coverage", "veto_worst_class",
    "curb_correlation", "submerged", "severity", "shift_dx", "shift_dy", "gallery_slot",
]


_UPLOAD_SOURCES = ("drawn", "lines", "full")


@dataclass(frozen=True)
class UploadRoi:
    """The firmware's flood_upload_roi_t: the frame region uploaded, its step and where it came from."""
    x: int
    y: int
    width: int
    height: int
    step: int
    source: str          # "drawn" | "lines" | "full"
    out_width: int
    out_height: int

    @classmethod
    def from_array(cls, a) -> "UploadRoi":
        return cls(int(a[0]), int(a[1]), int(a[2]), int(a[3]), int(a[4]), _UPLOAD_SOURCES[a[5]],
                   int(a[6]), int(a[7]))

    def to_array(self) -> list[int]:
        return [self.x, self.y, self.width, self.height, self.step, _UPLOAD_SOURCES.index(self.source),
                self.out_width, self.out_height]

    def as_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "width": self.width, "height": self.height, "step": self.step,
                "source": self.source, "out_width": self.out_width, "out_height": self.out_height}

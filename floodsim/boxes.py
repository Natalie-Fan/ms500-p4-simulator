"""Stage-1 boxes: the traffic model (or a YOLO) on the PC standing in for the IMX500, with a per-frame cache."""
from __future__ import annotations

import csv
import json
import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol

import cv2
import numpy as np

from .errors import FloodSimError
from .results import Box

log = logging.getLogger(__name__)

YOLO_INPUT = (640, 480)   # IMX500 accelerator input (ms500/veto.py IMX500_INPUT)
DEFAULT_MODEL = str(Path(__file__).resolve().parent.parent / "models" / "traffic.onnx")
# The device's detector settings, components/as_code/process_common/inference_common.h:
# modelScoreThreshold (0.5 with CONFIG_DNN_TYPE_FLOOD) and modelNMSThreshold.
DEFAULT_CONF = 0.5
DEFAULT_NMS = 0.20
# inference_yolo_traffic.cpp:167 drops motorcycles before tracking, drawing or flood see them.
DROPPED_CLASSES = {"motorcycle"}
# COCO name -> flood_class_t. The firmware maps its traffic model's classes the
# same way in map_traffic_class(), main/ai_process/flood_task.c:101-112.
COCO_TO_FLOOD = {"person": 1, "car": 2, "truck": 3, "bus": 4, "bicycle": 5, "motorcycle": 6}
CSV_FIELDS = ["frame_idx", "t_sec", "x0", "y0", "x1", "y1", "conf", "coco_class", "flood_class"]


@dataclass(frozen=True)
class CachedBox:
    x0: float
    y0: float
    x1: float
    y1: float
    conf: float
    coco_class: str
    flood_class: int

    def to_box(self) -> Box:
        return Box(self.x0, self.y0, self.x1, self.y1, self.conf, self.flood_class)


@dataclass(frozen=True)
class Detection:
    """In detector-input coordinates (see detector_input())."""
    x0: float
    y0: float
    x1: float
    y1: float
    conf: float
    coco_class: str


class Detector(Protocol):
    model_name: str
    conf: float

    def detect(self, rgb_small: np.ndarray) -> list[Detection]: ...


class YoloDetector:
    def __init__(self, model: str = DEFAULT_MODEL, conf: float = DEFAULT_CONF):
        try:
            from ultralytics import YOLO  # heavy and optional
        except ImportError as e:
            raise FloodSimError(
                "ultralytics is not installed (pip install -e .[yolo]); "
                "use --no-detector to run without Stage-1 boxes") from e
        self.model_name = model
        self.conf = conf
        self._model = YOLO(model)

    def detect(self, rgb_small: np.ndarray) -> list[Detection]:
        out = self._model.predict(cv2.cvtColor(rgb_small, cv2.COLOR_RGB2BGR),
                                  verbose=False, conf=self.conf, iou=DEFAULT_NMS,
                                  agnostic_nms=True)[0]   # the firmware's NMS ignores class
        dets = []
        for b in out.boxes:
            x0, y0, x1, y1 = (float(v) for v in b.xyxy[0])
            dets.append(Detection(x0, y0, x1, y1, float(b.conf), out.names[int(b.cls)]))
        return dets


def detector_input(model: str) -> tuple[int, int]:
    from .traffic_onnx import INPUT_SIZE
    return INPUT_SIZE if str(model).lower().endswith(".onnx") else YOLO_INPUT


def make_detector(model: str, conf: float) -> Detector:
    if str(model).lower().endswith(".onnx"):
        from .traffic_onnx import OnnxTrafficDetector
        return OnnxTrafficDetector(model, conf, DEFAULT_NMS)
    return YoloDetector(model, conf)


def cache_meta(fingerprint: dict, model: str, conf: float) -> dict:
    return {"model": model, "conf": conf, "nms": DEFAULT_NMS, "input": list(detector_input(model)),
            "source": fingerprint}


def _num(v: float) -> str:
    return f"{v:.9g}"   # 9 significant digits round-trip a float32 exactly


def _row(idx: int, t: float, b: CachedBox | None) -> list[str]:
    if b is None:   # marker: frame detected, nothing found
        return [str(idx), f"{t:.3f}", "", "", "", "", "", "", ""]
    return [str(idx), f"{t:.3f}", _num(b.x0), _num(b.y0), _num(b.x1), _num(b.y1),
            _num(b.conf), b.coco_class, str(b.flood_class)]


def _rows_for(idx: int, t: float, boxes: list[CachedBox]) -> list[list[str]]:
    return [_row(idx, t, b) for b in boxes] if boxes else [_row(idx, t, None)]


def write_boxes_csv(path: Path, entries, meta: dict | None) -> None:
    """entries: list of (frame_idx, t_sec, list[CachedBox])."""
    with open(path, "w", newline="", encoding="utf-8") as f:
        f.write(f"# meta: {json.dumps(meta, sort_keys=True)}\n")
        w = csv.writer(f, lineterminator="\n")
        w.writerow(CSV_FIELDS)
        for idx, t, boxes in entries:
            w.writerows(_rows_for(idx, t, boxes))


def _ensure_trailing_newline(path: Path) -> None:
    """A run killed mid-write leaves a partial last line; never append onto it."""
    with open(path, "rb+") as f:
        f.seek(0, os.SEEK_END)
        if f.tell() == 0:
            return
        f.seek(-1, os.SEEK_END)
        if f.read(1) != b"\n":
            f.write(b"\n")


def _read_csv(path: Path) -> tuple[dict | None, dict[int, list[CachedBox]], dict[int, float]]:
    meta = None
    rows: dict[int, list[CachedBox]] = {}
    times: dict[int, float] = {}
    with open(path, newline="", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            if line.startswith("# meta: "):
                try:
                    meta = json.loads(line[len("# meta: "):])
                except json.JSONDecodeError:
                    meta = None
                continue
            if line.startswith("#") or line.startswith("frame_idx") or not line.strip():
                continue
            fields = next(csv.reader([line]))
            try:
                last_idx = int(fields[0])
            except ValueError:
                last_idx = None
            try:
                if last_idx is None or len(fields) != len(CSV_FIELDS):
                    raise ValueError(f"{len(fields)} fields")
                idx, t = last_idx, float(fields[1])
                boxes = rows.setdefault(idx, [])
                times[idx] = t
                if fields[2] != "":
                    x0, y0, x1, y1, conf = (float(v) for v in fields[2:7])
                    boxes.append(CachedBox(x0, y0, x1, y1, conf, fields[7], int(fields[8])))
            except ValueError as e:
                # Appends are sequential, so an unparseable index means the last frame.
                bad = last_idx if last_idx is not None else next(reversed(rows), None)
                rows.pop(bad, None)
                times.pop(bad, None)
                log.warning("%s:%d: malformed box row (%s); frame %s will be re-detected",
                            path, line_no, e, bad)
    return meta, rows, times


class BoxStore:
    """<input>.boxes.csv (detector cache) + <input>.boxes.edited.csv (manual edits, which win)."""

    def __init__(self, input_path: Path, meta: dict):
        self.cache_path = Path(f"{input_path}.boxes.csv")
        self.edited_path = Path(f"{input_path}.boxes.edited.csv")
        self.meta = meta
        self._cache: dict[int, list[CachedBox]] = {}
        self._edited: dict[int, list[CachedBox]] = {}
        self._edited_times: dict[int, float] = {}
        if self.cache_path.exists():
            file_meta, rows, _ = _read_csv(self.cache_path)
            if file_meta == json.loads(json.dumps(meta)):
                self._cache = rows
            else:
                # Never destroy detection work: set the old cache aside, start a new one.
                stale = Path(f"{self.cache_path}.stale")
                self.cache_path.replace(stale)
                log.warning("%s was made with different settings; moved it to %s",
                            self.cache_path, stale)
        if self.edited_path.exists():
            _, self._edited, self._edited_times = _read_csv(self.edited_path)

    @property
    def has_cache(self) -> bool:
        return bool(self._cache)

    @property
    def has_edits(self) -> bool:
        return bool(self._edited)

    def get(self, idx: int) -> list[CachedBox] | None:
        if idx in self._edited:
            return self._edited[idx]
        return self._cache.get(idx)

    def is_edited(self, idx: int) -> bool:
        return idx in self._edited

    def put_detected(self, idx: int, t: float, boxes: list[CachedBox]) -> None:
        self._cache[idx] = boxes
        if not self.cache_path.exists():
            write_boxes_csv(self.cache_path, [], self.meta)
        _ensure_trailing_newline(self.cache_path)
        with open(self.cache_path, "a", newline="", encoding="utf-8") as f:
            csv.writer(f, lineterminator="\n").writerows(_rows_for(idx, t, boxes))

    def set_edited(self, idx: int, t: float, boxes: list[CachedBox]) -> None:
        self._edited[idx] = boxes
        self._edited_times[idx] = t
        self._write_edited()

    def clear_edited(self, idx: int) -> None:
        self._edited.pop(idx, None)
        self._edited_times.pop(idx, None)
        self._write_edited()

    def _write_edited(self) -> None:
        entries = [(i, self._edited_times.get(i, 0.0), self._edited[i]) for i in sorted(self._edited)]
        write_boxes_csv(self.edited_path, entries, {"edited": True})


class BoxProvider:
    """Boxes for a frame: edited > cache > detector. `none()` gives no boxes at all."""

    def __init__(self, core, store: BoxStore | None, detector: Detector | None):
        self.core = core
        self.store = store
        self.detector = detector

    @classmethod
    def none(cls, core) -> BoxProvider:
        return cls(core, None, None)

    @property
    def description(self) -> dict:
        if self.store is None:
            return {"kind": "no-detector"}
        return {"kind": "yolo" if self.detector else "cache",
                "model": self.store.meta["model"], "conf": self.store.meta["conf"],
                "cache_file": str(self.store.cache_path), "edited_file": str(self.store.edited_path)}

    def cached_for(self, idx: int, t: float,
                   load_rgb: Callable[[], np.ndarray | None]) -> list[CachedBox]:
        if self.store is None:
            return []
        got = self.store.get(idx)
        if got is not None:
            return got
        if self.detector is None:
            raise FloodSimError(
                f"frame {idx} is not in {self.store.cache_path} and no detector is available "
                f"(install the detector's runtime or use --no-detector)")
        rgb = load_rgb()
        if rgb is None:
            return []
        size = tuple(self.store.meta["input"])
        small = cv2.resize(rgb, size, interpolation=cv2.INTER_AREA)
        got = []
        for d in self.detector.detect(small):
            if d.coco_class in DROPPED_CLASSES:
                continue
            fc = COCO_TO_FLOOD.get(d.coco_class, 0)
            b = self.core.box_from_accelerator(d.x0, d.y0, d.x1, d.y1, d.conf, fc, *size)
            # Round-trip through the cache's text form so a fresh detection and
            # its cached copy are the same Python values, not just the same float32.
            x0, y0, x1, y1, conf = (float(_num(v)) for v in (b.x0, b.y0, b.x1, b.y1, b.confidence))
            got.append(CachedBox(x0, y0, x1, y1, conf, d.coco_class, fc))
        self.store.put_detected(idx, t, got)
        return got

    def boxes_for(self, idx: int, t: float, load_rgb: Callable[[], np.ndarray | None]) -> list[Box]:
        return [c.to_box() for c in self.cached_for(idx, t, load_rgb)]


def make_box_provider(core, source, *, no_detector: bool, model: str = DEFAULT_MODEL,
                      conf: float = DEFAULT_CONF) -> BoxProvider:
    if no_detector:
        return BoxProvider.none(core)
    store = BoxStore(source.path, cache_meta(source.fingerprint(), model, conf))
    try:
        detector: Detector | None = make_detector(model, conf)
    except FloodSimError as e:
        if not store.has_cache:
            raise
        log.warning("%s; using cached boxes from %s only", e, store.cache_path)
        detector = None
    return BoxProvider(core, store, detector)

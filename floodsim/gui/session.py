"""GUI state and operations. Qt-free; every method runs on the worker thread (worker.py)."""
from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from ..boxes import (DEFAULT_CONF, DEFAULT_MODEL, BoxProvider, BoxStore, CachedBox, cache_meta,
                     make_box_provider)
from ..errors import FloodSimError
from ..geometry import Geometry
from ..labels import Labels, score_rows
from ..modes import (FrameFeeder, RunParams, UploadWriter, device_row, run_dense, run_device,
                     sample_device_at)
from ..report import make_run_info, prepare_out_dir, summarize, write_run
from ..results import DENSE_FIELDS, DEVICE_FIELDS, ScoreResult
from ..source import Normalizer, open_source, read_image_bgr

log = logging.getLogger(__name__)
_SCORE_FIELDS = [f for f in DENSE_FIELDS if f not in ("frame_idx", "t_sec")]


class LenientBoxes:
    """The GUI's box provider: a frame the cache lacks, with no detector, just has no boxes."""

    def __init__(self, inner: BoxProvider):
        self.inner = inner
        self.store = inner.store
        self.description = inner.description

    def cached_for(self, idx, t, load_rgb) -> list[CachedBox]:
        try:
            return self.inner.cached_for(idx, t, load_rgb)
        except FloodSimError:
            return []

    def boxes_for(self, idx, t, load_rgb):
        return [c.to_box() for c in self.cached_for(idx, t, load_rgb)]


@dataclass(frozen=True)
class FrameData:
    idx: int
    t: float
    rgb: np.ndarray | None
    boxes: list          # (CachedBox, edited, band coverage)
    score: ScoreResult | None
    label: str | None


@dataclass(frozen=True)
class RefStatus:
    ready: bool
    stale: bool
    description: str
    geometry_version: int
    built_under: int | None


class Session:
    def __init__(self, core, build_info: dict):
        self.core = core
        self.build_info = build_info
        self.source = None
        self.boxes: LenientBoxes | None = None
        self.geometry = self.default_geometry()
        self.geometry_version = 0
        self.core.set_geometry(self.geometry)
        self.reference = None
        self.ref_gray: np.ndarray | None = None
        self.ref_info: dict = {"kind": "none"}
        self.ref_version: int | None = None
        self.device = core.new_device()
        self.dense_cache: dict[int, ScoreResult] = {}
        self.manual_rows: list[dict] = []
        self.run: tuple[RunParams, object] | None = None
        self.labels: Labels | None = None
        self.labels_path: Path | None = None

    # ------------------------------------------------------------ helpers
    def _src(self):
        if self.source is None:
            raise FloodSimError("open a video, image or folder first")
        return self.source

    def _label(self, t: float, idx: int) -> str | None:
        if self.labels is None or self.source is None:
            return None
        return self.labels.label_for(t, self.source.frame_name(idx) if idx >= 0 else None)

    def _label_rows(self, rows: list[dict]) -> None:
        if self.labels is not None:
            for r in rows:
                r["label"] = self._label(r["t_sec"], r["frame_idx"])

    # --------------------------------------------------------------- input
    def open(self, path, *, fps=None, allow_stretch=False, no_detector=False) -> dict:
        src = open_source(Path(path), fps=fps, allow_stretch=allow_stretch)
        inner = None
        if not no_detector:
            try:
                inner = make_box_provider(self.core, src, no_detector=False)
            except FloodSimError as e:
                log.warning("%s", e)
        if inner is None:
            meta = cache_meta(src.fingerprint(), DEFAULT_MODEL, DEFAULT_CONF)
            inner = BoxProvider(self.core, BoxStore(src.path, meta), None)
        self.source = src
        self.boxes = LenientBoxes(inner)
        self.dense_cache.clear()
        self.manual_rows.clear()
        self.run = None
        self.device.reset()
        return {"name": src.path.name, "n_frames": src.n_frames,
                "times": [src.timestamp(i) for i in range(src.n_frames)],
                "detector": inner.detector is not None}

    def _score(self, idx: int, rgb: np.ndarray, cached: list[CachedBox]) -> ScoreResult | None:
        if self.reference is None or not self.reference.ready:
            return None
        s = self.dense_cache.get(idx)
        if s is None:
            s = self.core.score(self.reference, self.core.rgb_to_gray(rgb), [b.to_box() for b in cached])
            self.dense_cache[idx] = s
        return s

    def frame(self, idx: int, *, diff: bool = False) -> FrameData:
        src = self._src()
        t = src.timestamp(idx)
        rgb = src.read_rgb(idx)
        cached = self.boxes.cached_for(idx, t, lambda: rgb) if rgb is not None else []
        edited = self.boxes.store.is_edited(idx)
        boxes = [(b, edited, self.core.band_coverage(b.to_box())) for b in cached]
        score = self._score(idx, rgb, cached) if rgb is not None else None
        shown = rgb
        if diff and rgb is not None and self.ref_gray is not None:
            d = cv2.absdiff(self.core.rgb_to_gray(rgb), self.ref_gray)
            d = np.minimum(d.astype(np.uint16) * 3, 255).astype(np.uint8)
            shown = np.repeat(d[:, :, None], 3, axis=2)
        return FrameData(idx, t, shown, boxes, score, self._label(t, idx))

    def score_frame(self, idx: int) -> ScoreResult:
        if self.reference is None or not self.reference.ready:
            raise FloodSimError("set a dry reference first")
        fd = self.frame(idx)
        if fd.score is None:
            raise FloodSimError(f"frame {idx} failed to decode")
        return fd.score

    # ----------------------------------------------------------- reference
    def _build_reference(self, rgb: np.ndarray, info: dict) -> RefStatus:
        gray = self.core.rgb_to_gray(rgb)
        ref = self.core.new_reference()
        if not ref.build(gray):
            raise FloodSimError("geometry leaves no usable sampling window")
        self.reference, self.ref_gray, self.ref_info = ref, gray, info
        self.ref_version = self.geometry_version
        self.dense_cache.clear()
        return self.ref_status()

    def set_reference_frame(self, idx: int) -> RefStatus:
        src = self._src()
        rgb = src.read_rgb(idx)
        if rgb is None:
            raise FloodSimError(f"frame {idx} failed to decode")
        return self._build_reference(rgb, {"kind": "frame", "frame_idx": idx, "t_sec": src.timestamp(idx)})

    def set_reference_image(self, path) -> RefStatus:
        bgr = read_image_bgr(Path(path))
        if bgr is None:
            raise FloodSimError(f"cannot read reference image {path}")
        rgb = Normalizer(False)(bgr, str(path))
        return self._build_reference(rgb, {"kind": "image", "path": str(path)})

    def rebuild_reference(self) -> RefStatus:
        if self.ref_gray is None:
            raise FloodSimError("no reference to rebuild; set one first")
        ref = self.core.new_reference()
        if not ref.build(self.ref_gray):
            raise FloodSimError("geometry leaves no usable sampling window")
        self.reference = ref
        self.ref_version = self.geometry_version
        self.dense_cache.clear()
        return self.ref_status()

    def ref_status(self) -> RefStatus:
        if self.reference is None:
            return RefStatus(False, False, "none", self.geometry_version, None)
        info = self.ref_info
        if info["kind"] == "frame":
            desc = f"frame {info['frame_idx']} ({info['t_sec']:.2f} s)"
        else:
            desc = f"image {Path(info['path']).name}"
        return RefStatus(True, self.ref_version != self.geometry_version, desc,
                         self.geometry_version, self.ref_version)

    # ------------------------------------------------------------ geometry
    def default_geometry(self) -> Geometry:
        return Geometry.from_list(self.core.default_geometry())

    def set_geometry(self, g: Geometry) -> dict:
        err = self.core.validate_geometry(g)
        if err:
            raise FloodSimError(f"invalid geometry: {err}")
        self.core.set_geometry(g)
        self.geometry = g
        self.geometry_version += 1
        self.dense_cache.clear()
        return self.overlay_geometry()

    def overlay_geometry(self) -> dict:
        return {"geometry": self.geometry.to_list(), "strip": self.core.strip_outline(),
                "align": self.core.align_patch(), "domain": self.core.domain(),
                "submerged_below": self.core.submerged_below(),
                "veto_threshold": self.core.veto_threshold(),
                "heartbeat_every": self.core.heartbeat_every(),
                "sample_period_s": self.core.sample_period_s()}

    # --------------------------------------------------------------- boxes
    def set_boxes(self, idx: int, boxes: list[CachedBox]) -> None:
        self.boxes.store.set_edited(idx, self._src().timestamp(idx), list(boxes))
        self.dense_cache.pop(idx, None)

    def clear_edits(self, idx: int) -> None:
        self.boxes.store.clear_edited(idx)
        self.dense_cache.pop(idx, None)

    # ---------------------------------------------------------- manual mode
    def sample(self, idx: int, retry_gap: float = 1.0) -> dict:
        src = self._src()
        t = src.timestamp(idx)
        r = sample_device_at(self.device, self.reference, FrameFeeder(self.core, src, self.boxes), t, retry_gap)
        row = device_row(len(self.manual_rows), t, r)
        self._label_rows([row])
        self.manual_rows.append(row)
        return dict(row)

    def reset_device(self) -> None:
        self.device.reset()

    # ---------------------------------------------------------------- runs
    def run_batch(self, params: RunParams, progress=None, cancel=None) -> dict:
        src = self._src()
        if params.mode == "dense":
            if self.reference is None or not self.reference.ready:
                raise FloodSimError("dense runs need a dry reference")
            out = run_dense(self.core, src, self.boxes, self.reference, params, progress=progress, cancel=cancel)
            for r in out.rows:
                self.dense_cache[r["frame_idx"]] = ScoreResult(**{k: r[k] for k in _SCORE_FIELDS})
        else:
            out = run_device(self.core, src, self.boxes, self.reference, params, progress=progress, cancel=cancel)
        self._label_rows(out.rows)
        self.run = (params, out)
        return {"mode": params.mode, "rows": [dict(r) for r in out.rows],
                "cancelled": out.cancelled, "decode_failures": out.decode_failures}

    def label_spans(self) -> list[tuple[float, float, str]]:
        src = self.source
        if src is None or self.labels is None:
            return []
        spans: list[list] = []
        n = src.n_frames
        for i in range(n):
            t = src.timestamp(i)
            end = src.timestamp(i + 1) if i + 1 < n else t + (t - src.timestamp(i - 1) if i else 1.0)
            label = self._label(t, i)
            if label is None:
                continue
            if spans and spans[-1][2] == label and abs(spans[-1][1] - t) < 1e-9:
                spans[-1][1] = end
            else:
                spans.append([t, end, label])
        return [tuple(s) for s in spans]

    def load_labels(self, path) -> dict:
        self.labels = Labels.load(Path(path))
        self.labels_path = Path(path)
        self._label_rows(self.manual_rows)
        run_rows = run_mode = None
        if self.run is not None:
            params, out = self.run
            self._label_rows(out.rows)
            run_rows, run_mode = [dict(r) for r in out.rows], params.mode
        return {"spans": self.label_spans(), "warnings": list(self.labels.warnings),
                "manual_rows": [dict(r) for r in self.manual_rows], "run_rows": run_rows,
                "run_mode": run_mode}

    def export_run(self, out) -> str:
        if self.run is None:
            raise FloodSimError("run a device or dense pass first (Run menu)")
        src = self._src()
        params, result = self.run
        out = prepare_out_dir(Path(out))
        fields = list(DEVICE_FIELDS if params.mode == "device" else DENSE_FIELDS)
        score = None
        if self.labels is not None:
            fields.append("label")
            score = score_rows(result.rows, params.mode)
        if params.mode == "device":
            uploads = UploadWriter(out, self.core.upload_downscale())
            for row in result.rows:
                if row["outcome"] in ("upload", "heartbeat") and row["frame_idx"] >= 0:
                    rgb = src.read_rgb(row["frame_idx"])
                    if rgb is not None:
                        uploads.write(row["sample"], rgb)
        info = make_run_info(source=src, mode=params.mode, params=params, geometry=self.geometry,
                             reference=self.ref_info, boxes=self.boxes.description,
                             labels=self.labels_path, build_info=self.build_info, input_path=src.path)
        write_run(out, info, result.rows, fields,
                  summarize(result.rows, params.mode, result.decode_failures, score))
        return str(out)

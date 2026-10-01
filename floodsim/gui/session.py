"""GUI state and operations. Qt-free; every method runs on the worker thread (worker.py)."""
from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from ..boxes import (DEFAULT_CONF, DEFAULT_MODEL, BoxProvider, BoxStore, CachedBox, ClipBoxes,
                     cache_meta, make_box_provider)
from ..errors import FloodSimError
from ..geometry import Geometry
from ..labels import Labels, score_rows
from ..modes import (FrameFeeder, RunOutput, RunParams, UploadWriter, device_row, run_dense,
                     run_device, sample_device_at)
from ..report import make_run_info, prepare_out_dir, summarize, write_run
from ..results import DENSE_FIELDS, DEVICE_FIELDS, ScoreResult
from ..source import Normalizer, open_source, read_image_bgr

log = logging.getLogger(__name__)
_SCORE_FIELDS = [f for f in DENSE_FIELDS if f not in ("frame_idx", "t_sec")]


class LenientBoxes:
    """The GUI's box provider: a frame the cache lacks, with no detector, just has no boxes.
    Such frames are counted in `missing_frames` so a run can say so."""

    def __init__(self, inner: BoxProvider):
        self.inner = inner
        self.store = inner.store
        self.missing_frames: set[int] = set()

    @property
    def description(self) -> dict:
        d = dict(self.inner.description)
        if self.inner.detector is None and not self.store.has_cache:
            d = {"kind": "none"}
        if self.store.has_edits:
            d["edited_file"] = str(self.store.edited_path)
        else:
            d.pop("edited_file", None)
        return d

    def cached_for(self, idx, t, load_rgb) -> list[CachedBox]:
        try:
            return self.inner.cached_for(idx, t, load_rgb)
        except FloodSimError:
            self.missing_frames.add(idx)
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
class RunRecord:
    """A finished batch run and the settings it ran under, for export."""
    params: RunParams
    output: RunOutput
    geometry: Geometry
    reference: dict
    boxes: dict


@dataclass(frozen=True)
class RefStatus:
    ready: bool
    stale: bool
    description: str
    geometry_version: int
    built_under: int | None
    manual_names: tuple[str, ...] = ()   # manual references in order, for View ▸ Show reference
    slots: tuple[tuple[int, str, str], ...] = ()   # (index, source, label) per in-use slot, for Delete reference


class Session:
    def __init__(self, core, build_info: dict, model: str = DEFAULT_MODEL):
        self.core = core
        self.model = model
        self.build_info = build_info
        self.source = None
        self.boxes: LenientBoxes | None = None
        self.geometry = self.default_geometry()
        self.geometry_version = 0
        self.core.set_geometry(self.geometry)
        # The live gallery (manual baselines + whatever manual Sample has learned),
        # and the manual baselines' frames, from which every batch run and reset
        # rebuilds a manual-only gallery.
        self.reference = None
        self.manual_refs: list[tuple[np.ndarray, dict]] = []
        self.manual_slots: list[int] = []   # gallery slot of each manual_refs entry, same order
        self.ref_version: int | None = None
        self.device = core.new_device()
        self.dense_cache: dict[int, ScoreResult] = {}
        self.manual_rows: list[dict] = []
        self.run: RunRecord | None = None
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
                inner = make_box_provider(self.core, src, no_detector=False, model=self.model)
            except FloodSimError as e:
                log.warning("%s", e)
        if inner is None:
            if hasattr(src, "clip_of"):
                inner = ClipBoxes(self.core, src, self.model, DEFAULT_CONF, None)
            else:
                meta = cache_meta(src.fingerprint(), self.model, DEFAULT_CONF)
                inner = BoxProvider(self.core, BoxStore(src.path, meta), None)
        if self.source is not None:
            for _, info in self.manual_refs:
                if info["kind"] == "frame":
                    info.setdefault("input", str(self.source.path))
        self.source = src
        self.boxes = LenientBoxes(inner)
        self.labels = None
        self.labels_path = None
        self.dense_cache.clear()
        self.manual_rows.clear()
        self.run = None
        self.device.reset()
        clock = getattr(src, "start_clock", None)
        return {"name": src.path.name, "n_frames": src.n_frames,
                "times": [src.timestamp(i) for i in range(src.n_frames)],
                "detector": inner.detector is not None,
                "start_clock": clock,
                "gaps": src.gaps() if clock else [],
                "clips": src.clip_bounds() if clock else []}

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
        if diff and rgb is not None and self.manual_refs:
            # Against the newest manual baseline: learned references keep no image.
            d = cv2.absdiff(self.core.rgb_to_gray(rgb), self.manual_refs[-1][0])
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
    def _manual_gallery(self):
        """A gallery of just the manual baselines, built under the current geometry."""
        gallery = self.core.new_gallery()
        for gray, _ in self.manual_refs:
            gallery.add_manual(gray)
        return gallery

    def _build_reference(self, rgb: np.ndarray, info: dict) -> RefStatus:
        # As Capture Baseline on the device: added to the live gallery, learned
        # references kept.
        gray = self.core.rgb_to_gray(rgb)
        if self.reference is None:
            self.reference = self.core.new_gallery()
        before = {i for i, src, _ in self.reference.slots() if src == "manual"}
        self.reference.add_manual(gray)
        self.manual_slots.append(next(i for i, src, _ in self.reference.slots()
                                      if src == "manual" and i not in before))
        self.manual_refs.append((gray, info))
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
        """Rebuild the manual baselines under the current geometry; learned ones are
        dropped (neither the device nor the simulator keeps their frames)."""
        if not self.manual_refs:
            raise FloodSimError("no reference to rebuild; set one first")
        self.reference = self._manual_gallery()
        self.manual_slots = list(range(len(self.manual_refs)))   # an empty gallery fills slots in order
        self.device.reset()      # a pending learning candidate belongs to the old gallery
        self.ref_version = self.geometry_version
        self.dense_cache.clear()
        return self.ref_status()

    def clear_references(self) -> RefStatus:
        self.reference = None
        self.device.reset()      # a pending learning candidate belongs to the old gallery
        self.manual_refs.clear()
        self.manual_slots.clear()
        self.ref_version = None
        self.dense_cache.clear()
        return self.ref_status()

    def delete_reference(self, index: int, source: str) -> RefStatus:
        """Delete one gallery slot, as the device's web UI does. A manual one also leaves the
        baselines that runs and rebuilds start from, and drops a pending learning candidate
        (it was anchored against the manual references)."""
        if self.reference is None or not self.reference.delete_slot(index, source):
            raise FloodSimError(f"no {source} reference in slot {index}")
        if source == "manual":
            pos = self.manual_slots.index(index)
            del self.manual_slots[pos]
            del self.manual_refs[pos]
            self.device.forget_candidate()
        self.dense_cache.clear()
        return self.ref_status()

    def clear_learned(self) -> RefStatus:
        """Free every learned slot and drop the pending learning candidate, as the device does."""
        if self.reference is not None:
            self.reference.clear_auto()
        self.device.forget_candidate()
        self.dense_cache.clear()
        return self.ref_status()

    @staticmethod
    def _describe(info: dict) -> str:
        if info["kind"] == "frame":
            desc = f"frame {info['frame_idx']} ({info['t_sec']:.2f} s)"
            if "input" in info:
                desc += f" of {Path(info['input']).name}"
            return desc
        return f"image {Path(info['path']).name}"

    def ref_status(self) -> RefStatus:
        if self.reference is None or not self.reference.ready:
            return RefStatus(False, False, "none", self.geometry_version, None)
        slots = self.reference.slots()
        learned = sum(source == "auto" for _, source, _ in slots)
        lines = [f"{len(slots)} reference(s): {len(slots) - learned} manual, {learned} learned"]
        names = tuple(self._describe(info) for _, info in self.manual_refs)
        lines += [f"  manual: {name}" for name in names]
        labels = dict(zip(self.manual_slots, names))
        listed = tuple((i, source, labels.get(i, f"manual, slot {i}") if source == "manual"
                        else f"learned, slot {i}") for i, source, _ in slots)
        return RefStatus(True, self.ref_version != self.geometry_version, "\n".join(lines),
                         self.geometry_version, self.ref_version, names, listed)

    def reference_view(self, index: int) -> np.ndarray:
        """Manual reference `index` as RGB, in the gray the firmware stores and compares."""
        if not 0 <= index < len(self.manual_refs):
            raise FloodSimError(f"no manual reference {index + 1}")
        return np.repeat(self.manual_refs[index][0][:, :, None], 3, axis=2)

    def _reference_info(self) -> dict:
        if not self.manual_refs:
            return {"kind": "none"}
        return {"kind": "gallery", "manual": [dict(info) for _, info in self.manual_refs]}

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
        if r.admitted:     # the live gallery changed: cached scores no longer hold
            self.dense_cache.clear()
        row = device_row(len(self.manual_rows), t, r)
        self._label_rows([row])
        self.manual_rows.append(row)
        return dict(row)

    def reset_device(self) -> RefStatus:
        """Clear the capture state and forget learned references (manual ones are
        rebuilt under the current geometry)."""
        self.device.reset()
        if self.manual_refs:
            self.rebuild_reference()
        return self.ref_status()

    # ---------------------------------------------------------------- runs
    def run_batch(self, params: RunParams, progress=None, cancel=None) -> dict:
        src = self._src()
        if params.mode == "dense" and (self.reference is None or not self.reference.ready):
            raise FloodSimError("dense runs need a dry reference")
        # Headless runs use a reference built under the current geometry, as the CLI does.
        rebuilt = self.reference is not None and self.ref_status().stale
        if rebuilt:
            self.rebuild_reference()
        self.boxes.missing_frames.clear()
        reference = self._reference_info()
        adopted = False
        if params.mode == "dense":
            # Read-only against the session's gallery, learned references included, so
            # the curve shows what the device would score with what it has learned.
            reference["learned"] = self.reference.count - len(self.manual_refs)
            out = run_dense(self.core, src, self.boxes, self.reference, params, progress=progress, cancel=cancel)
            for r in out.rows:
                self.dense_cache[r["frame_idx"]] = ScoreResult(**{k: r[k] for k in _SCORE_FIELDS})
        else:
            # A device run starts from the manual baselines only, so it is reproducible;
            # afterwards the session adopts the gallery it ended with, as the device would
            # keep what it learned.
            gallery = self._manual_gallery() if self.manual_refs else None
            out = run_device(self.core, src, self.boxes, gallery, params, progress=progress, cancel=cancel)
            if gallery is not None:
                self.reference = gallery
                self.manual_slots = list(range(len(self.manual_refs)))   # built by _manual_gallery(); runs never move manual slots
                self.device.reset()      # the session's pending candidate belongs to the old gallery
                self.dense_cache.clear()
                adopted = True
        self._label_rows(out.rows)
        missing = len(self.boxes.missing_frames)
        if self.reference is not None:
            reference["rebuilt_for_run"] = rebuilt
        self.run = RunRecord(params, out, self.geometry, reference,
                             {**self.boxes.description, "frames_without_boxes": missing})
        return {"mode": params.mode, "rows": [dict(r) for r in out.rows],
                "cancelled": out.cancelled, "decode_failures": out.decode_failures,
                "reference_rebuilt": rebuilt, "frames_without_boxes": missing,
                "gallery_adopted": adopted, "skipped_no_footage": out.skipped_no_footage}

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
            self._label_rows(self.run.output.rows)
            run_rows, run_mode = [dict(r) for r in self.run.output.rows], self.run.params.mode
        return {"spans": self.label_spans(), "warnings": list(self.labels.warnings),
                "manual_rows": [dict(r) for r in self.manual_rows], "run_rows": run_rows,
                "run_mode": run_mode}

    def export_run(self, out) -> str:
        if self.run is None:
            raise FloodSimError("run a device or dense pass first (Run menu)")
        src = self._src()
        run = self.run
        params, result = run.params, run.output
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
        info = make_run_info(source=src, mode=params.mode, params=params, geometry=run.geometry,
                             reference=run.reference, boxes=run.boxes,
                             labels=self.labels_path, build_info=self.build_info, input_path=src.path)
        info["cancelled"] = result.cancelled
        write_run(out, info, result.rows, fields,
                  summarize(result.rows, params.mode, result.decode_failures, score,
                            cancelled=result.cancelled, skipped_no_footage=result.skipped_no_footage))
        return str(out)

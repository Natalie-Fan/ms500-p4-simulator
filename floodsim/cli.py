"""floodsim command line: run, detect, compare, export."""
from __future__ import annotations

import argparse
import datetime as dt
import logging
import sys
from dataclasses import asdict
from pathlib import Path

from . import __version__
from .boxes import DEFAULT_CONF, DEFAULT_MODEL, make_box_provider, write_boxes_csv
from .core import load_core
from .errors import FloodSimError
from .geometry import Geometry, load_geometry
from .labels import Labels, score_rows
from .modes import RunParams, UploadWriter, run_dense, run_device
from .report import compare_runs, prepare_out_dir, summarize, write_run
from .results import DENSE_FIELDS, DEVICE_FIELDS
from .source import Normalizer, open_source, read_image_bgr, write_image

log = logging.getLogger("floodsim")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="floodsim",
                                description="Run the MS500-P4 flood-detection firmware code on image/video files.")
    sub = p.add_subparsers(dest="command", required=True)

    def add_input(sp):
        sp.add_argument("input", type=Path, help="image, folder of images, or video")
        sp.add_argument("--fps", type=float, help="frame rate for image folders without _t<ms> names")
        sp.add_argument("--allow-stretch", action="store_true", help="resize non-16:9 input anyway")

    def add_boxes(sp):
        sp.add_argument("--no-detector", action="store_true", help="no Stage-1 boxes (veto always passes)")
        sp.add_argument("--model", default=DEFAULT_MODEL)
        sp.add_argument("--conf", type=float, default=DEFAULT_CONF)

    run = sub.add_parser("run", help="headless device or dense run")
    add_input(run)
    add_boxes(run)
    run.add_argument("--mode", choices=["device", "dense"], default="device")
    run.add_argument("--reference", type=Path, help="dry reference image")
    run.add_argument("--reference-at", type=float, help="use the input's frame at this time (s) as reference")
    run.add_argument("--geometry", type=Path, help="curb/lane geometry JSON")
    run.add_argument("--labels", type=Path, help="labels CSV to score against")
    run.add_argument("--period", type=float, help="device sample period in s (default: firmware's)")
    run.add_argument("--retry-gap", type=float, default=1.0)
    run.add_argument("--every", type=int, default=1, help="dense: score every Nth frame")
    run.add_argument("--no-rebuild", action="store_true", help="use the existing DLL even if stale")
    run.add_argument("--out", type=Path)

    detect = sub.add_parser("detect", help="fill the box cache ahead of time")
    add_input(detect)
    detect.add_argument("--model", default=DEFAULT_MODEL)
    detect.add_argument("--conf", type=float, default=DEFAULT_CONF)
    detect.add_argument("--every", type=int, default=1)

    cmp_ = sub.add_parser("compare", help="diff two runs (folders, samples.csv or exe .jsonl)")
    cmp_.add_argument("a", type=Path)
    cmp_.add_argument("b", type=Path)

    exp = sub.add_parser("export", help="gray PGM frames + boxes.csv for flood_sim.exe")
    add_input(exp)
    add_boxes(exp)
    exp.add_argument("--out", type=Path, required=True)
    exp.add_argument("--every", type=int, default=1)
    exp.add_argument("--reference", type=Path, help="also write reference.pgm from this image")
    exp.add_argument("--no-rebuild", action="store_true")
    return p


def build_reference(core, source, reference: Path | None, reference_at: float | None,
                    allow_stretch: bool):
    if reference is not None and reference_at is not None:
        raise FloodSimError("use --reference or --reference-at, not both")
    if reference is not None:
        bgr = read_image_bgr(reference)
        if bgr is None:
            raise FloodSimError(f"cannot read reference image {reference}")
        rgb = Normalizer(allow_stretch)(bgr, str(reference))
        info = {"kind": "image", "path": str(reference)}
    elif reference_at is not None:
        idx = source.index_at(reference_at)
        rgb = source.read_rgb(idx) if idx is not None else None
        if rgb is None:
            raise FloodSimError(f"no readable frame at {reference_at} s for the reference")
        info = {"kind": "frame", "t_sec": reference_at, "frame_idx": idx}
    else:
        return None, {"kind": "none"}
    ref = core.new_reference()
    if not ref.build(core.rgb_to_gray(rgb)):
        raise FloodSimError("geometry leaves no usable sampling window")
    return ref, info


def _apply_geometry(core, path: Path | None) -> Geometry:
    g = load_geometry(path) if path else Geometry.from_list(core.default_geometry())
    err = core.validate_geometry(g)
    if err:
        raise FloodSimError(f"invalid geometry: {err}")
    core.set_geometry(g)
    return g


def cmd_run(args) -> int:
    core, build_info = load_core(rebuild=not args.no_rebuild)
    geometry = _apply_geometry(core, args.geometry)
    source = open_source(args.input, fps=args.fps, allow_stretch=args.allow_stretch)
    ref, ref_info = build_reference(core, source, args.reference, args.reference_at, args.allow_stretch)
    if args.mode == "dense" and ref is None:
        raise FloodSimError("dense mode needs --reference or --reference-at")
    params = RunParams(args.mode, args.period or core.sample_period_s(), args.retry_gap, args.every)
    if params.period <= 0 or params.retry_gap < 0 or params.every < 1:
        raise FloodSimError("--period must be > 0, --retry-gap >= 0, --every >= 1")
    labels = Labels.load(args.labels) if args.labels else None
    for w in labels.warnings if labels else []:
        log.warning(w)
    boxes = make_box_provider(core, source, no_detector=args.no_detector, model=args.model, conf=args.conf)
    out = prepare_out_dir(args.out or Path("out") /
                          f"{args.input.stem}-{args.mode}-{dt.datetime.now():%Y%m%d-%H%M%S}")

    if args.mode == "device":
        result = run_device(core, source, boxes, ref, params,
                            uploads=UploadWriter(out, core.upload_downscale()))
        fields = list(DEVICE_FIELDS)
    else:
        result = run_dense(core, source, boxes, ref, params)
        fields = list(DENSE_FIELDS)

    score = None
    if labels:
        for row in result.rows:
            name = source.frame_name(row["frame_idx"]) if row["frame_idx"] >= 0 else None
            row["label"] = labels.label_for(row["t_sec"], name)
        fields.append("label")
        score = score_rows(result.rows, args.mode)

    run_info = {
        "floodsim": __version__, "created": dt.datetime.now().isoformat(timespec="seconds"),
        "input": source.fingerprint(), "mode": args.mode, "params": asdict(params),
        "geometry": geometry.to_json_dict(), "reference": ref_info, "boxes": boxes.description,
        "labels": str(args.labels) if args.labels else None, **build_info,
    }
    write_run(out, run_info, result.rows, fields,
              summarize(result.rows, args.mode, result.decode_failures, score))
    print(f"wrote {out}")
    return 0


def cmd_detect(args) -> int:
    core, _ = load_core()
    source = open_source(args.input, fps=args.fps, allow_stretch=args.allow_stretch)
    boxes = make_box_provider(core, source, no_detector=False, model=args.model, conf=args.conf)
    indices = range(0, source.n_frames, args.every)
    for n, idx in enumerate(indices, 1):
        boxes.cached_for(idx, source.timestamp(idx), lambda i=idx: source.read_rgb(i))
        if n % 50 == 0 or n == len(indices):
            print(f"detected {n}/{len(indices)} frames")
    print(f"cache: {boxes.store.cache_path}")
    return 0


def cmd_compare(args) -> int:
    diffs = compare_runs(args.a, args.b)
    for d in diffs:
        print(d)
    print(f"{len(diffs)} difference(s)")
    return 1 if diffs else 0


def cmd_export(args) -> int:
    core, _ = load_core(rebuild=not args.no_rebuild)
    source = open_source(args.input, fps=args.fps, allow_stretch=args.allow_stretch)
    boxes = make_box_provider(core, source, no_detector=args.no_detector, model=args.model, conf=args.conf)
    out = prepare_out_dir(args.out)
    frames_dir = out / "frames"
    frames_dir.mkdir()
    entries = []
    for idx in range(0, source.n_frames, args.every):
        rgb = source.read_rgb(idx)
        if rgb is None:
            log.warning("frame %d failed to decode; not exported", idx)
            continue
        t = source.timestamp(idx)
        write_image(frames_dir / f"f{idx:06d}_t{round(t * 1000):010d}.pgm", core.rgb_to_gray(rgb))
        entries.append((idx, t, boxes.cached_for(idx, t, lambda: rgb)))
    write_boxes_csv(out / "boxes.csv", entries, {"export": str(args.input)})
    if args.reference:
        ref_bgr = read_image_bgr(args.reference)
        if ref_bgr is None:
            raise FloodSimError(f"cannot read reference image {args.reference}")
        rgb = Normalizer(args.allow_stretch)(ref_bgr, str(args.reference))
        write_image(out / "reference.pgm", core.rgb_to_gray(rgb))
    print(f"wrote {len(entries)} frames to {frames_dir}")
    return 0


COMMANDS = {"run": cmd_run, "detect": cmd_detect, "compare": cmd_compare, "export": cmd_export}


def main(argv=None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    args = build_parser().parse_args(argv)
    try:
        return COMMANDS[args.command](args)
    except FloodSimError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())

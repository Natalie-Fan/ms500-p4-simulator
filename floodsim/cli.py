"""floodsim command line: run, detect, compare, export."""
from __future__ import annotations

import argparse
import datetime as dt
import logging
import sys
from pathlib import Path

from .boxes import DEFAULT_CONF, DEFAULT_MODEL, make_box_provider, write_boxes_csv
from .core import load_core
from .errors import FloodSimError
from .geometry import Geometry, load_geometry
from .labels import Labels, score_rows
from .modes import (RunParams, UploadWriter, heartbeat_readings, make_clock, run_dense, run_device,
                    upload_run_info)
from .report import compare_runs, make_run_info, prepare_out_dir, summarize, write_run
from .results import DENSE_FIELDS, DEVICE_FIELDS
from .source import Normalizer, open_source, read_image_bgr, write_image
from .upload_area import check_sn, parse_upload_area

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
    run.add_argument("--reference", type=Path, action="append", default=[],
                     help="dry reference image; repeat for more manual baselines (e.g. day, dusk, night)")
    run.add_argument("--reference-at", type=float, action="append", default=[],
                     help="use the input's frame at this time (s) as a manual baseline; repeatable")
    run.add_argument("--geometry", type=Path, help="curb/lane geometry JSON")
    run.add_argument("--labels", type=Path, help="labels CSV to score against")
    run.add_argument("--period", type=float,
                     help="device sample period in s (default: firmware's 60); shorten it to get "
                          "more samples, and so more gallery learning, out of a short clip")
    run.add_argument("--retry-gap", type=float, default=1.0)
    run.add_argument("--every", type=int, default=1, help="dense: score every Nth frame")
    run.add_argument("--no-rebuild", action="store_true", help="use the existing DLL even if stale")
    run.add_argument("--out", type=Path)
    run.add_argument("--heartbeat", type=int, default=60,
                     help="device: heartbeat interval in minutes, 0-1440, 0 = off (Device Manager setting)")
    run.add_argument("--upload-area", help="device: drawn upload area x,y,w,h, or a JSON file from the "
                                           "camera's GET /api/flood/upload_roi")
    run.add_argument("--sn", default="SIM", help="device: serial number in upload file names")

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
    exp.add_argument("--reference", type=Path, action="append", default=[],
                     help="also write reference.pgm from this image; repeat for reference_1.pgm, ...")
    exp.add_argument("--no-rebuild", action="store_true")

    gui = sub.add_parser("gui", help="open the GUI")
    gui.add_argument("input", type=Path, nargs="?", help="image, folder of images, or video")
    gui.add_argument("--fps", type=float)
    gui.add_argument("--allow-stretch", action="store_true")
    gui.add_argument("--no-detector", action="store_true", help="do not load the detector (cached/edited boxes only)")
    gui.add_argument("--model", default=DEFAULT_MODEL)
    return p


def _reference_image_rgb(path: Path, allow_stretch: bool):
    bgr = read_image_bgr(path)
    if bgr is None:
        raise FloodSimError(f"cannot read reference image {path}")
    return Normalizer(allow_stretch)(bgr, str(path))


def build_reference(core, source, references: list[Path], reference_at: list[float],
                    allow_stretch: bool):
    """A gallery with one manual baseline per --reference image, then per --reference-at frame."""
    manual = []
    for path in references:
        manual.append((_reference_image_rgb(path, allow_stretch), {"kind": "image", "path": str(path)}))
    for t in reference_at:
        idx = source.index_at(t)
        rgb = source.read_rgb(idx) if idx is not None else None
        if rgb is None:
            raise FloodSimError(f"no readable frame at {t} s for the reference")
        manual.append((rgb, {"kind": "frame", "t_sec": t, "frame_idx": idx}))
    if not manual:
        return None, {"kind": "none"}
    gallery = core.new_gallery()
    for rgb, _ in manual:
        gallery.add_manual(core.rgb_to_gray(rgb))
    return gallery, {"kind": "gallery", "manual": [info for _, info in manual]}


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
    params = RunParams(args.mode, args.period or core.sample_period_s(), args.retry_gap, args.every, args.heartbeat)
    if params.period <= 0 or params.retry_gap < 0 or params.every < 1:
        raise FloodSimError("--period must be > 0, --retry-gap >= 0, --every >= 1")
    heartbeat_readings(params.heartbeat_minutes, params.period)   # raises on 0-1440 violations
    sn = check_sn(args.sn)
    drawn = parse_upload_area(args.upload_area) if args.upload_area else None
    if args.mode == "dense" and (args.upload_area or args.sn != "SIM" or args.heartbeat != 60):
        log.info("dense mode never uploads: --heartbeat, --upload-area and --sn are ignored")
    if drawn is not None and not core.upload_rect_valid(drawn):
        log.warning("upload area %s breaks the firmware rules (sides >= 32, inside 1920x1080, "
                    "<= 518400 px); the camera would use the curb/lane lines, and so does this run", drawn)
    labels = Labels.load(args.labels) if args.labels else None
    for w in labels.warnings if labels else []:
        log.warning(w)
    boxes = make_box_provider(core, source, no_detector=args.no_detector, model=args.model, conf=args.conf)
    out = prepare_out_dir(args.out or Path("out") /
                          f"{args.input.stem}-{args.mode}-{dt.datetime.now():%Y%m%d-%H%M%S}")

    if args.mode == "device":
        result = run_device(core, source, boxes, ref, params,
                            uploads=UploadWriter(out, core, drawn=drawn, sn=sn,
                                                 clock=make_clock(source, args.input)))
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

    run_info = make_run_info(source=source, mode=args.mode, params=params, geometry=geometry,
                             reference=ref_info, boxes=boxes.description, labels=args.labels,
                             build_info=build_info, input_path=args.input)
    if args.mode == "device":
        run_info.update(upload_run_info(core, params, drawn, sn))
    write_run(out, run_info, result.rows, fields,
              summarize(result.rows, args.mode, result.decode_failures, score,
                        skipped_no_footage=result.skipped_no_footage))
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
    if hasattr(source, "clip_of"):
        raise FloodSimError("export does not support folders of clips; export a single clip")
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
    for i, path in enumerate(args.reference):
        name = "reference.pgm" if i == 0 else f"reference_{i}.pgm"
        write_image(out / name, core.rgb_to_gray(_reference_image_rgb(path, args.allow_stretch)))
    print(f"wrote {len(entries)} frames to {frames_dir}")
    return 0


def cmd_gui(args) -> int:
    try:
        from .gui.app import run_gui
    except ImportError as e:
        raise FloodSimError("GUI dependencies missing: pip install -e .[gui]") from e
    return run_gui(args.input, fps=args.fps, allow_stretch=args.allow_stretch, no_detector=args.no_detector,
                   model=args.model)


COMMANDS = {"run": cmd_run, "detect": cmd_detect, "compare": cmd_compare, "export": cmd_export, "gui": cmd_gui}


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

"""Convert MS500_Flooding_Detection's burst ground truth into floodsim labels CSVs.

truth.csv (burst_id, clip, label) + bursts.json (id, clip, start_frame) ->
one <clip>.labels.csv per clip with start_sec,end_sec,label. A burst spans
FRAMES_PER_BURST frames FRAME_STEP apart (dataset/build_bursts.py:35-36).
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

FRAMES_PER_BURST = 5
FRAME_STEP = 4


def clip_fps(videos: Path | None, clip: int, override: float | None) -> tuple[float, str]:
    if videos is None:
        if override is None:
            sys.exit("--fps is required without --videos")
        return override, f"c{clip:02d}"
    vids = sorted(videos.glob("*.avi"))
    if clip > len(vids):
        sys.exit(f"clip {clip}: only {len(vids)} videos in {videos}")
    path = vids[clip - 1]  # build_bursts.py numbers sorted Videos/*.avi from 1
    if override is not None:
        return override, path.stem
    import cv2
    fps = cv2.VideoCapture(str(path)).get(cv2.CAP_PROP_FPS)
    if not fps or fps <= 0:
        sys.exit(f"{path}: no frame rate; pass --fps")
    return fps, path.stem


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--truth", type=Path, required=True)
    ap.add_argument("--bursts", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--videos", type=Path)
    ap.add_argument("--fps", type=float)
    args = ap.parse_args()

    starts = {b["id"]: b["start_frame"] for b in json.loads(args.bursts.read_text(encoding="utf-8"))}
    by_clip: dict[int, list[tuple[int, str]]] = defaultdict(list)
    with open(args.truth, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            if row["burst_id"] not in starts:
                print(f"warning: {row['burst_id']} not in {args.bursts}", file=sys.stderr)
                continue
            by_clip[int(row["clip"])].append((starts[row["burst_id"]], row["label"].strip().upper()))

    args.out.mkdir(parents=True, exist_ok=True)
    span = (FRAMES_PER_BURST - 1) * FRAME_STEP + 1
    for clip, bursts in sorted(by_clip.items()):
        fps, name = clip_fps(args.videos, clip, args.fps)
        with open(args.out / f"{name}.labels.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f, lineterminator="\n")
            w.writerow(["start_sec", "end_sec", "label"])
            for start, label in sorted(bursts):
                w.writerow([f"{start / fps:.3f}", f"{(start + span) / fps:.3f}", label])
    print(f"wrote {len(by_clip)} labels file(s) to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

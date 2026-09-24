"""Ground-truth labels and scoring. BLOCKED is an abstention, never a negative."""
from __future__ import annotations

import csv
import io
from pathlib import Path

from .errors import FloodSimError

VALID_LABELS = {"DRY", "WET", "FLOODED", "INDETERMINATE"}


def _label(value: str, path: Path) -> str:
    v = value.strip().upper()
    if v not in VALID_LABELS:
        raise FloodSimError(f"{path}: unknown label {value!r} (expected {sorted(VALID_LABELS)})")
    return v


class Labels:
    def __init__(self, intervals=None, files=None, warnings=None):
        self.intervals: list[tuple[float, float, str]] = intervals or []
        self.files: dict[str, str] = files or {}
        self.warnings: list[str] = warnings or []

    @classmethod
    def load(cls, path: Path) -> Labels:
        path = Path(path)
        text = path.read_text(encoding="utf-8-sig")
        reader = csv.DictReader(io.StringIO(text))
        fields = {f.strip() for f in reader.fieldnames or []}
        rows = [{(k or "").strip(): (v or "").strip() for k, v in r.items()} for r in reader]
        rows = [r for r in rows if any(r.values())]   # Excel leaves ",," rows behind
        if {"start_sec", "end_sec", "label"} <= fields:
            try:
                intervals = sorted((float(r["start_sec"]), float(r["end_sec"]), _label(r["label"], path))
                                   for r in rows)
            except ValueError as e:
                raise FloodSimError(f"{path}: {e}") from e
            return cls(intervals=intervals, warnings=_interval_warnings(intervals))
        if {"file", "label"} <= fields:
            return cls(files={r["file"]: _label(r["label"], path) for r in rows})
        raise FloodSimError(f"{path}: expected columns start_sec,end_sec,label or file,label")

    def label_for(self, t: float, name: str | None) -> str | None:
        if self.files:
            return self.files.get(name) if name else None
        for start, end, label in self.intervals:
            if start <= t < end:
                return label
        return None


def _interval_warnings(intervals) -> list[str]:
    gaps = overlaps = 0
    for (_, a_end, _), (b_start, _, _) in zip(intervals, intervals[1:]):
        if b_start < a_end - 1e-9:
            overlaps += 1
        elif b_start > a_end + 1e-9:
            gaps += 1
    out = []
    if overlaps:
        out.append(f"labels: {overlaps} overlap(s) between intervals; the earliest interval wins")
    if gaps:
        out.append(f"labels: {gaps} gap(s) between intervals; samples in gaps are unlabelled")
    return out


def score_rows(rows: list[dict], mode: str) -> dict:
    c = dict(tp=0, fp=0, tn=0, fn=0, abstained=0, indeterminate=0, unlabelled=0, skipped=0)
    pos_corr: list[float] = []
    neg_corr: list[float] = []
    for r in rows:
        if mode == "device":
            if r["outcome"] == "skipped_no_ref":
                c["skipped"] += 1
                continue
            abstain = r["outcome"] == "blocked"
            predicted = r["outcome"] == "upload"
            reading = r["has_reading"]
        else:
            abstain = not r["veto_usable"]
            predicted = r["submerged"]
            reading = True
        label = r.get("label")
        if label is None:
            c["unlabelled"] += 1
            continue
        if label == "INDETERMINATE":
            c["indeterminate"] += 1
            continue
        if abstain:
            c["abstained"] += 1
            continue
        positive = label == "FLOODED"
        if reading:
            (pos_corr if positive else neg_corr).append(r["curb_correlation"])
        c[("tp" if predicted else "fn") if positive else ("fp" if predicted else "tn")] += 1

    labelled = c["tp"] + c["fp"] + c["tn"] + c["fn"] + c["abstained"]
    negatives = c["fp"] + c["tn"]
    c["abstention_rate"] = c["abstained"] / labelled if labelled else None
    c["flooded_max_corr"] = max(pos_corr) if pos_corr else None
    c["dry_wet_min_corr"] = min(neg_corr) if neg_corr else None
    c["false_alarm_upper_95"] = 3 / negatives if c["fp"] == 0 and negatives else None
    return c


def format_score(s: dict) -> list[str]:
    lines = [f"labelled: TP={s['tp']} FP={s['fp']} TN={s['tn']} FN={s['fn']}",
             f"abstained (BLOCKED, not counted as negatives): {s['abstained']}"
             + (f" ({s['abstention_rate']:.1%})" if s["abstention_rate"] is not None else ""),
             f"excluded: indeterminate={s['indeterminate']} unlabelled={s['unlabelled']} "
             f"skipped_no_ref={s['skipped']}"]
    hi, lo = s["flooded_max_corr"], s["dry_wet_min_corr"]
    if hi is not None or lo is not None:
        line = f"margin: flooded max corr={hi if hi is None else f'{hi:.3f}'}, " \
               f"dry/wet min corr={lo if lo is None else f'{lo:.3f}'}"
        if hi is not None and lo is not None:
            line += f", gap={lo - hi:+.3f}"
        lines.append(line)
    if s["false_alarm_upper_95"] is not None:
        n = s["fp"] + s["tn"]
        lines.append(f"0 false alarms in {n} negatives: 95% upper bound on the false-alarm rate "
                     f"is 3/{n} = {s['false_alarm_upper_95']:.1%} (rule of three), not zero")
    return lines

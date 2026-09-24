"""Run folders (run.json, samples.csv, summary.txt) and run comparison."""
from __future__ import annotations

import csv
import datetime as dt
import json
from collections import Counter
from dataclasses import asdict
from pathlib import Path

from . import __version__
from .errors import FloodSimError
from .labels import format_score

_FLOAT6 = {"curb_correlation", "veto_coverage"}
_CATEGORICAL = ("frame_idx", "outcome", "severity", "submerged", "veto_usable")


def format_value(key: str, v) -> str:
    if v is None:
        return ""
    if isinstance(v, bool):
        return "true" if v else "false"
    if key == "t_sec":
        return f"{v:.3f}"
    if key in _FLOAT6:
        return f"{v:.6f}"
    return str(v)


def write_samples(path: Path, rows: list[dict], fields: list[str]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(fields)
        for r in rows:
            w.writerow([format_value(k, r.get(k)) for k in fields])


def read_samples(path: Path) -> list[dict[str, str]]:
    path = Path(path)
    if path.is_dir():
        path = path / "samples.csv"
    if path.suffix == ".jsonl":
        rows = []
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                d = json.loads(line)
                d.pop("mode", None)
                rows.append({k: ("true" if v is True else "false" if v is False else str(v))
                             for k, v in d.items()})
        return rows
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def compare_runs(a: Path, b: Path, tol: float = 1e-6) -> list[str]:
    ra, rb = read_samples(a), read_samples(b)
    key = "sample" if (ra or rb) and "sample" in (ra or rb)[0] else "frame_idx"
    ma, mb = {r[key]: r for r in ra}, {r[key]: r for r in rb}
    diffs = []
    for k in sorted(set(ma) | set(mb), key=int):
        if k not in mb:
            diffs.append(f"{key} {k}: only in A")
            continue
        if k not in ma:
            diffs.append(f"{key} {k}: only in B")
            continue
        x, y = ma[k], mb[k]
        for f in _CATEGORICAL:
            if f in x and f in y and x[f] != y[f]:
                diffs.append(f"{key} {k}: {f} {x[f]} -> {y[f]}")
        cx, cy = float(x["curb_correlation"]), float(y["curb_correlation"])
        if abs(cx - cy) > tol:
            diffs.append(f"{key} {k}: curb_correlation {cx:.6f} -> {cy:.6f}")
    return diffs


def make_run_info(*, source, mode: str, params, geometry, reference: dict, boxes: dict,
                  labels, build_info: dict, input_path) -> dict:
    """run.json content: inputs, settings and provenance (firmware commit, DLL)."""
    return {
        "floodsim": __version__, "created": dt.datetime.now().isoformat(timespec="seconds"),
        "input": {"path": str(input_path), **source.fingerprint()}, "mode": mode, "params": asdict(params),
        "geometry": geometry.to_json_dict(), "reference": reference, "boxes": boxes,
        "labels": str(labels) if labels else None, **build_info,
    }


def prepare_out_dir(out: Path) -> Path:
    out = Path(out)
    if out.exists() and any(out.iterdir()):
        raise FloodSimError(f"{out} already exists and is not empty; choose another --out")
    out.mkdir(parents=True, exist_ok=True)
    return out


def summarize(rows: list[dict], mode: str, decode_failures: int, score: dict | None) -> str:
    lines = [f"mode: {mode}", f"rows: {len(rows)}", f"failed frame reads: {decode_failures}"]
    if mode == "device":
        counts = Counter(r["outcome"] for r in rows)
        for name in ("upload", "clear", "blocked", "heartbeat", "skipped_no_ref"):
            lines.append(f"{name}: {counts.get(name, 0)}")
    else:
        lines.append(f"submerged: {sum(r['submerged'] for r in rows)}")
        lines.append(f"vetoed (obstructed): {sum(not r['veto_usable'] for r in rows)}")
    if score:
        lines += format_score(score)
    return "\n".join(lines) + "\n"


def write_run(out: Path, run_info: dict, rows: list[dict], fields: list[str], summary: str) -> None:
    out = Path(out)
    (out / "run.json").write_text(json.dumps(run_info, indent=2, default=str) + "\n", encoding="utf-8")
    write_samples(out / "samples.csv", rows, fields)
    (out / "summary.txt").write_text(summary, encoding="utf-8")

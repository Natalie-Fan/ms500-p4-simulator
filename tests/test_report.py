import json

import pytest

from floodsim.errors import FloodSimError
from floodsim.report import (compare_runs, format_value, prepare_out_dir, read_samples,
                             summarize, write_run, write_samples)
from floodsim.results import DENSE_FIELDS

ROW = {"frame_idx": 0, "t_sec": 0.0, "veto_usable": True, "veto_coverage": 0.0,
       "veto_worst_class": "other", "curb_correlation": 0.99999994, "submerged": False,
       "severity": "none", "shift_dx": 0, "shift_dy": 0}


def test_format_values():
    assert format_value("t_sec", 60.0) == "60.000"
    assert format_value("curb_correlation", 0.99999994) == "1.000000"
    assert format_value("veto_usable", True) == "true"
    assert format_value("label", None) == ""
    assert format_value("frame_idx", 3) == "3"


def test_write_and_read_run(tmp_path):
    out = prepare_out_dir(tmp_path / "run")
    write_run(out, {"mode": "dense"}, [ROW], DENSE_FIELDS, "summary\n")
    assert json.loads((out / "run.json").read_text())["mode"] == "dense"
    assert (out / "summary.txt").read_text() == "summary\n"
    rows = read_samples(out)
    assert rows[0]["curb_correlation"] == "1.000000" and rows[0]["submerged"] == "false"


def test_prepare_out_dir_refuses_non_empty(tmp_path):
    (tmp_path / "run").mkdir()
    (tmp_path / "run" / "samples.csv").write_text("x")
    with pytest.raises(FloodSimError, match="not empty"):
        prepare_out_dir(tmp_path / "run")


def test_compare_identical_and_different(tmp_path):
    a, b = tmp_path / "a.csv", tmp_path / "b.csv"
    write_samples(a, [ROW], DENSE_FIELDS)
    write_samples(b, [ROW], DENSE_FIELDS)
    assert compare_runs(a, b) == []
    write_samples(b, [dict(ROW, curb_correlation=0.5, submerged=True)], DENSE_FIELDS)
    diffs = compare_runs(a, b)
    assert any("curb_correlation" in d for d in diffs) and any("submerged" in d for d in diffs)
    write_samples(b, [ROW, dict(ROW, frame_idx=1)], DENSE_FIELDS)
    assert any("only in B" in d for d in compare_runs(a, b))


def test_compare_reads_jsonl(tmp_path):
    a = tmp_path / "a.csv"
    write_samples(a, [ROW], DENSE_FIELDS)
    j = tmp_path / "exe.jsonl"
    j.write_text('{"mode":"dense","frame_idx":0,"t_sec":0.000,"veto_usable":true,"veto_coverage":0.000000,'
                 '"veto_worst_class":"other","curb_correlation":1.000000,"submerged":false,'
                 '"severity":"none","shift_dx":0,"shift_dy":0}\n')
    assert compare_runs(a, j) == []


def test_compare_reports_frame_choice(tmp_path):
    a, b = tmp_path / "a.csv", tmp_path / "b.csv"
    fields = ["sample", "t_sec", "frame_idx", "outcome", "severity", "curb_correlation"]
    row = {"sample": 0, "t_sec": 0.0, "frame_idx": 18011, "outcome": "clear", "severity": "none",
           "curb_correlation": 0.9}
    write_samples(a, [row], fields)
    write_samples(b, [dict(row, frame_idx=18012)], fields)
    assert any("frame_idx" in d for d in compare_runs(a, b))


def test_summary_reports_skipped_checks_only_when_any():
    assert "checks skipped (no footage): 5" in summarize([], "device", 0, None, skipped_no_footage=5)
    assert "no footage" not in summarize([], "device", 0, None)

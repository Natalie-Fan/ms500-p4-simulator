import json
import subprocess
import sys
from pathlib import Path

TOOL = Path(__file__).resolve().parent.parent / "tools" / "truth_to_labels.py"


def test_converts_bursts_to_intervals(tmp_path):
    (tmp_path / "truth.csv").write_text(
        "burst_id,clip,label,extent_pct,n_usable,curb_corr,trusted\n"
        "c01_b0,1,DRY,0,5,0.99,0\nc01_b1,1,WET,0,5,0.9,0\nc13_b0,13,FLOODED,0,5,0.1,1\n")
    (tmp_path / "bursts.json").write_text(json.dumps([
        {"id": "c01_b0", "clip": 1, "start_frame": 6},
        {"id": "c01_b1", "clip": 1, "start_frame": 100},
        {"id": "c13_b0", "clip": 13, "start_frame": 40},
    ]))
    p = subprocess.run([sys.executable, str(TOOL), "--truth", str(tmp_path / "truth.csv"),
                        "--bursts", str(tmp_path / "bursts.json"), "--out", str(tmp_path / "labels"),
                        "--fps", "4"], capture_output=True, text=True)
    assert p.returncode == 0, p.stderr
    assert (tmp_path / "labels" / "c01.labels.csv").read_text().splitlines() == [
        "start_sec,end_sec,label", "1.500,5.750,DRY", "25.000,29.250,WET"]
    assert (tmp_path / "labels" / "c13.labels.csv").read_text().splitlines()[1] == "10.000,14.250,FLOODED"

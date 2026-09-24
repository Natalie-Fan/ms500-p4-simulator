import subprocess

import pytest

from floodsim import core as core_mod
from floodsim.boxes import DEFAULT_CONF, DEFAULT_MODEL, BoxStore, CachedBox, cache_meta
from floodsim.cli import main
from floodsim.report import compare_runs, read_samples
from floodsim.source import open_source, write_image
from tests.synth import dry_gray, flooded_gray, to_rgb, write_frame_folder

CAR = CachedBox(0.0, 0.0, 1919.0, 1079.0, 0.9, "car", 2)


@pytest.fixture
def scenario(tmp_path, core):
    """The golden scenario as a PNG folder, a reference image and a complete box cache."""
    frames = [(0, dry_gray()), (1000, dry_gray()), (60000, flooded_gray()),
              (61000, flooded_gray()), (120000, dry_gray()), (121000, dry_gray())]
    folder = write_frame_folder(tmp_path / "clip", frames)
    ref = tmp_path / "dry.png"
    write_image(ref, to_rgb(dry_gray()))
    src = open_source(folder)
    store = BoxStore(folder, cache_meta(src.fingerprint(), DEFAULT_MODEL, DEFAULT_CONF))
    for i in range(src.n_frames):
        store.put_detected(i, src.timestamp(i), [CAR] if i == 4 else [])
    return folder, ref, tmp_path


def test_run_device_writes_run_folder(scenario):
    folder, ref, tmp = scenario
    assert main(["run", str(folder), "--reference", str(ref), "--out", str(tmp / "dev")]) == 0
    rows = read_samples(tmp / "dev")
    assert [r["outcome"] for r in rows] == ["clear", "upload", "clear"]
    assert (tmp / "dev" / "run.json").exists() and (tmp / "dev" / "summary.txt").exists()
    assert len(list((tmp / "dev" / "uploads").iterdir())) == 1


def test_run_dense_with_labels(scenario):
    folder, ref, tmp = scenario
    labels = tmp / "labels.csv"
    labels.write_text("start_sec,end_sec,label\n0,59,DRY\n59,100,FLOODED\n100,200,DRY\n")
    assert main(["run", str(folder), "--mode", "dense", "--reference", str(ref),
                 "--labels", str(labels), "--out", str(tmp / "dense")]) == 0
    rows = read_samples(tmp / "dense")
    assert [r["label"] for r in rows] == ["DRY", "DRY", "FLOODED", "FLOODED", "DRY", "DRY"]
    summary = (tmp / "dense" / "summary.txt").read_text()
    assert "TP=2 FP=0 TN=3 FN=0" in summary and "abstained" in summary


@pytest.mark.parametrize("mode", ["device", "dense"])
def test_export_then_exe_matches_python(scenario, mode):
    folder, ref, tmp = scenario
    assert main(["run", str(folder), "--mode", mode, "--reference", str(ref), "--out", str(tmp / "py")]) == 0
    assert main(["export", str(folder), "--reference", str(ref), "--out", str(tmp / "exp")]) == 0
    exe = core_mod.BUILD_DIR / core_mod.EXE_NAME
    p = subprocess.run([str(exe), "--reference", str(tmp / "exp" / "reference.pgm"),
                        "--frames", str(tmp / "exp" / "frames"), "--boxes", str(tmp / "exp" / "boxes.csv"),
                        "--mode", mode], capture_output=True, text=True)
    assert p.returncode == 0, p.stderr
    (tmp / "exe.jsonl").write_text(p.stdout)
    assert compare_runs(tmp / "py", tmp / "exe.jsonl") == []


def test_compare_exit_codes(scenario):
    folder, ref, tmp = scenario
    main(["run", str(folder), "--mode", "dense", "--reference", str(ref), "--out", str(tmp / "a")])
    main(["run", str(folder), "--mode", "dense", "--reference", str(ref), "--out", str(tmp / "b")])
    assert main(["compare", str(tmp / "a"), str(tmp / "b")]) == 0
    main(["run", str(folder), "--mode", "dense", "--reference-at", "60", "--out", str(tmp / "c")])
    assert main(["compare", str(tmp / "a"), str(tmp / "c")]) == 1


def test_run_rejects_both_reference_options(scenario, capsys):
    folder, ref, tmp = scenario
    assert main(["run", str(folder), "--reference", str(ref), "--reference-at", "0",
                 "--out", str(tmp / "x")]) == 2
    assert "not both" in capsys.readouterr().err


def test_dense_without_reference_errors(scenario, capsys):
    folder, _, tmp = scenario
    assert main(["run", str(folder), "--mode", "dense", "--out", str(tmp / "x")]) == 2
    assert "--reference" in capsys.readouterr().err


def test_no_detector_ignores_cache(scenario):
    folder, ref, tmp = scenario
    assert main(["run", str(folder), "--reference", str(ref), "--no-detector", "--out", str(tmp / "nd")]) == 0
    rows = read_samples(tmp / "nd")
    assert rows[2]["frames_taken"] == "1"   # no car box, so no retry at 120 s


def test_run_json_records_input_path(scenario):
    import json
    folder, ref, tmp = scenario
    assert main(["run", str(folder), "--reference", str(ref), "--out", str(tmp / "rj")]) == 0
    info = json.loads((tmp / "rj" / "run.json").read_text())
    assert info["input"]["path"] == str(folder) and info["input"]["name"] == "clip"

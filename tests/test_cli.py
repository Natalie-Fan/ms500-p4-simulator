import json
import subprocess

import pytest

from floodsim import core as core_mod
from floodsim.boxes import DEFAULT_CONF, DEFAULT_MODEL, BoxStore, CachedBox, cache_meta
from floodsim.cli import main
from floodsim.report import compare_runs, read_samples
from floodsim.source import open_source, write_image
from tests.synth import dry_gray, dusk_gray, flooded_gray, to_rgb, write_clip, write_frame_folder

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
    assert len(list((tmp / "dev" / "uploads").glob("*.jpg"))) == 1


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


def test_run_takes_several_manual_references(scenario):
    import json
    folder, ref, tmp = scenario
    dusk = tmp / "dusk.png"
    write_image(dusk, to_rgb(dusk_gray()))
    assert main(["run", str(folder), "--reference", str(dusk), "--reference-at", "0",
                 "--out", str(tmp / "x")]) == 0
    info = json.loads((tmp / "x" / "run.json").read_text())
    assert [m["kind"] for m in info["reference"]["manual"]] == ["image", "frame"]
    rows = read_samples(tmp / "x")
    # Dry frames match the dry baseline (slot 1) exactly, so nothing new is learned.
    assert [rows[0]["gallery_slot"], rows[2]["gallery_slot"]] == ["1", "1"]
    assert {r["gallery_count"] for r in rows} == {"2"}
    assert "references learned: 0" in (tmp / "x" / "summary.txt").read_text()


def test_export_and_exe_with_two_references(scenario):
    folder, ref, tmp = scenario
    dusk = tmp / "dusk.png"
    write_image(dusk, to_rgb(dusk_gray()))
    args = ["--reference", str(ref), "--reference", str(dusk)]
    assert main(["run", str(folder), *args, "--out", str(tmp / "py")]) == 0
    assert main(["export", str(folder), *args, "--out", str(tmp / "exp")]) == 0
    assert (tmp / "exp" / "reference.pgm").exists() and (tmp / "exp" / "reference_1.pgm").exists()
    exe = core_mod.BUILD_DIR / core_mod.EXE_NAME
    p = subprocess.run([str(exe), "--reference", str(tmp / "exp" / "reference.pgm"),
                        "--reference", str(tmp / "exp" / "reference_1.pgm"),
                        "--frames", str(tmp / "exp" / "frames"), "--boxes", str(tmp / "exp" / "boxes.csv")],
                       capture_output=True, text=True)
    assert p.returncode == 0, p.stderr
    (tmp / "exe.jsonl").write_text(p.stdout)
    assert compare_runs(tmp / "py", tmp / "exe.jsonl") == []


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


def test_gui_subcommand_is_registered():
    from floodsim.cli import build_parser
    args = build_parser().parse_args(["gui", "clip.avi", "--no-detector"])
    assert args.command == "gui" and args.no_detector is True and str(args.input) == "clip.avi"


def test_gui_accepts_model():
    from floodsim.cli import build_parser
    args = build_parser().parse_args(["gui", "--model", "models/other.onnx"])
    assert args.model == "models/other.onnx"


def test_run_on_a_folder_of_clips(tmp_path):
    import json
    folder = tmp_path / "day"
    write_clip(folder, "20260618_160000", [dry_gray()] * 3)
    write_clip(folder, "20260618_161000", [dusk_gray()] * 3)
    ref = tmp_path / "dry.png"
    write_image(ref, to_rgb(dry_gray()))
    assert main(["run", str(folder), "--reference", str(ref), "--no-detector", "--out", str(tmp_path / "r")]) == 0
    rows = read_samples(tmp_path / "r")
    assert [r["t_sec"] for r in rows] == ["0.000", "600.000"]
    info = json.loads((tmp_path / "r" / "run.json").read_text())
    assert info["start_clock"] == "2026-06-18T16:00:00"
    assert len(info["input"]["clips"]) == 2
    assert "checks skipped (no footage): 9" in (tmp_path / "r" / "summary.txt").read_text()


def test_export_refuses_a_folder_of_clips(tmp_path, capsys):
    folder = tmp_path / "day"
    write_clip(folder, "20260618_160000", [dry_gray()] * 2)
    assert main(["export", str(folder), "--no-detector", "--out", str(tmp_path / "e")]) == 2
    assert "export does not support folders of clips" in capsys.readouterr().err


def _run(scenario, name, *extra):
    folder, ref, tmp = scenario
    out = tmp / name
    assert main(["run", str(folder), "--reference", str(ref), "--out", str(out), *extra]) == 0
    return out, json.loads((out / "run.json").read_text())


def test_cli_upload_options_reach_run_json(scenario):
    out, info = _run(scenario, "dev", "--heartbeat", "0", "--upload-area", "0,0,640,480", "--sn", "CAM-1")
    assert info["heartbeat"] == {"minutes": 0, "readings": 0}
    assert info["upload_area"] == {"x": 0, "y": 0, "width": 640, "height": 480}
    assert info["upload_area_used"]["source"] == "drawn" and info["sn"] == "CAM-1"
    jpgs = list((out / "uploads").glob("*.jpg"))
    assert jpgs and all(p.name.startswith("CAM-1_flood_") for p in jpgs)
    assert "upload_file" in (out / "samples.csv").read_text().splitlines()[0]


def test_cli_invalid_upload_area_warns_and_uses_lines(scenario, caplog):
    _, info = _run(scenario, "dev", "--upload-area", "1900,0,100,100")
    assert info["upload_area"] == {"x": 1900, "y": 0, "width": 100, "height": 100}
    assert info["upload_area_used"]["source"] == "lines"
    assert "upload area" in caplog.text.lower()


@pytest.mark.parametrize("args", [["--heartbeat", "2000"], ["--sn", "bad sn"], ["--upload-area", "1,2,3"]])
def test_cli_rejects_bad_upload_options(scenario, args):
    folder, ref, tmp = scenario
    assert main(["run", str(folder), "--reference", str(ref), "--out", str(tmp / "o"), *args]) == 2


def test_cli_dense_ignores_upload_options(scenario):
    folder, ref, tmp = scenario
    out = tmp / "dense"
    assert main(["run", str(folder), "--mode", "dense", "--reference", str(ref),
                 "--heartbeat", "5", "--sn", "X", "--out", str(out)]) == 0
    assert not (out / "uploads").exists()


def test_compare_ignores_upload_names(scenario):
    a, _ = _run(scenario, "a", "--sn", "A")
    b, _ = _run(scenario, "b", "--sn", "B")
    assert main(["compare", str(a), str(b)]) == 0
    assert compare_runs(a, b) == []

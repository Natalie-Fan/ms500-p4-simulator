import os
import time
from pathlib import Path

import numpy as np
import pytest

from floodsim import core as core_mod
from floodsim.core import build_error_message, firmware_dir, needs_rebuild
from floodsim.errors import FloodSimError
from floodsim.geometry import Geometry
from floodsim.results import Box
from tests.synth import CAR, dry_gray, dusk_gray, flooded_gray, night_gray


def test_build_info_records_dll_and_firmware(build_info):
    assert build_info["dll"]["abi"] == core_mod.ABI_VERSION == 4
    assert build_info["dll"]["stale"] is False
    assert Path(build_info["firmware"]["dir"]).name == "NETWORK_P4_FLOOD_DETECTION"
    assert "commit" in build_info["firmware"] and "dirty" in build_info["firmware"]


def test_band_coverage_and_heartbeat(core):
    assert core.band_coverage(CAR) == 1.0
    assert core.band_coverage(Box(0.0, 900.0, 1919.0, 1079.0, 0.9, 2)) == 0.0
    assert core.heartbeat_every() == 60


def test_rgb_to_gray_matches_firmware_formula(core):
    rng = np.random.default_rng(1)
    rgb = rng.integers(0, 256, size=(5, 7, 3), dtype=np.uint8)
    r, g, b = (rgb[..., i].astype(np.uint32) for i in range(3))
    expected = ((r * 77 + g * 151 + b * 28) >> 8).astype(np.uint8)
    assert np.array_equal(core.rgb_to_gray(rgb), expected)


def test_default_geometry_matches_firmware(core):
    assert core.default_geometry() == [560.0, 230.0, 1920.0, 355.0, 560.0, 378.0, 1920.0, 446.0]
    assert core.get_geometry() == core.default_geometry()


def test_validate_geometry_uses_firmware_messages(core):
    good = Geometry.from_list(core.default_geometry())
    assert core.validate_geometry(good) is None
    bad = Geometry.from_list([560, 230, 560, 355, 560, 378, 1920, 446])
    assert "curb line endpoints" in core.validate_geometry(bad)


def test_identical_frame_scores_one(core):
    ref = core.new_gallery()
    ref.add_manual(dry_gray())
    s = core.score(ref, dry_gray(), [])
    assert s.curb_correlation == pytest.approx(1.0, abs=1e-6)
    assert s.submerged is False and s.severity == "none"
    assert (s.shift_dx, s.shift_dy) == (0, 0)
    assert s.veto_usable is True and s.veto_worst_class == "other"


def test_flooded_frame_is_submerged(core):
    ref = core.new_gallery()
    ref.add_manual(dry_gray())
    s = core.score(ref, flooded_gray(), [])
    assert s.curb_correlation == 0.0
    assert s.submerged is True and s.severity == "moderate"


def test_box_across_kerb_vetoes(core):
    ref = core.new_gallery()
    ref.add_manual(dry_gray())
    s = core.score(ref, dry_gray(), [CAR])
    assert s.veto_usable is False
    assert s.veto_coverage == 1.0
    assert s.veto_worst_class == "car"


def test_score_without_reference_raises(core):
    with pytest.raises(FloodSimError, match="reference"):
        core.score(core.new_gallery(), dry_gray(), [])


def test_reference_goes_stale_when_geometry_changes(core):
    ref = core.new_gallery()
    ref.add_manual(dry_gray())
    assert ref.is_stale() is False
    g = core.default_geometry()
    g[1] = 240.0
    core.set_geometry(Geometry.from_list(g))
    assert ref.is_stale() is True


def test_box_from_accelerator_scales_to_frame(core):
    b = core.box_from_accelerator(10, 20, 30, 40, 0.5, 1, 640, 480)
    assert b == Box(30.0, 45.0, 90.0, 90.0, 0.5, 1)


def test_strip_outline_and_patch(core):
    xs, lo, hi = core.strip_outline()
    assert len(xs) == 275 and xs[0] == 700.0
    assert np.all(lo < hi)
    assert core.align_patch() == (300, 0, 1700, 200)
    assert core.domain() == (560.0, 1920.0)


def _provider(frames):
    """frames: list per attempt of (gray | None, boxes, idx)."""
    def provide(attempt):
        gray, boxes, idx = frames[attempt]
        return None if gray is None else (gray, boxes, idx)
    return provide


def test_device_clear_upload_retry(core):
    ref = core.new_gallery()
    ref.add_manual(dry_gray())
    dev = core.new_device()
    r = dev.sample(ref, _provider([(dry_gray(), [], 0), (dry_gray(), [], 1)]))
    assert (r.outcome, r.frames_taken, r.since_image, r.frame_idx) == ("clear", 1, 1, 0)
    r = dev.sample(ref, _provider([(flooded_gray(), [], 2), (flooded_gray(), [], 3)]))
    assert (r.outcome, r.severity, r.since_image) == ("upload", "moderate", 0)
    r = dev.sample(ref, _provider([(dry_gray(), [CAR], 4), (dry_gray(), [], 5)]))
    assert (r.outcome, r.frames_taken, r.frame_idx, r.veto_usable) == ("clear", 2, 5, True)


def test_device_heartbeat_on_60th_reading(core):
    ref = core.new_gallery()
    ref.add_manual(dry_gray())
    dev = core.new_device()
    outcomes = [dev.sample(ref, _provider([(dry_gray(), [], 0)] * 2)).outcome for _ in range(60)]
    assert outcomes[:59] == ["clear"] * 59
    assert outcomes[59] == "heartbeat"


def test_device_skips_without_reference(core):
    dev = core.new_device()
    r = dev.sample(None, _provider([(dry_gray(), [], 0)] * 2))
    assert r.outcome == "skipped_no_ref" and r.frame_idx == -1
    r = dev.sample(core.new_gallery(), _provider([(dry_gray(), [], 0)] * 2))
    assert r.outcome == "skipped_no_ref"


def test_gallery_slots_and_full_of_manual(core):
    g = core.new_gallery()
    assert g.count == 0 and not g.ready and g.slots() == []
    for _ in range(8):
        g.add_manual(dry_gray())
    assert g.count == 8 and all(src == "manual" for _, src, _ in g.slots())
    with pytest.raises(FloodSimError, match="full of manual"):
        g.add_manual(dry_gray())
    g.clear()
    assert g.count == 0


def test_score_picks_best_slot_and_never_learns(core):
    g = core.new_gallery()
    g.add_manual(dry_gray())
    s = core.score(g, dusk_gray(), [])
    assert 0.65 <= s.curb_correlation < 0.95 and s.gallery_slot == 0
    assert g.count == 1          # dense scoring is read-only
    g.add_manual(dusk_gray())
    s = core.score(g, dusk_gray(), [])
    assert s.gallery_slot == 1 and s.curb_correlation == pytest.approx(1.0, abs=1e-6)


def test_device_learns_new_lighting_and_notices_unmatched(core):
    g = core.new_gallery()
    g.add_manual(dry_gray())
    dev = core.new_device()
    dusk = _provider([(dusk_gray(), [], 7)] * 2)
    r = dev.sample(g, dusk)
    assert (r.admitted, r.candidate_pending, r.gallery_count) == (False, True, 1)
    r = dev.sample(g, dusk)
    assert (r.gallery_slot, r.gallery_source, r.admitted, r.gallery_count) == (0, "manual", True, 2)
    assert [src for _, src, _ in g.slots()] == ["manual", "auto"]
    r = dev.sample(g, dusk)
    assert (r.gallery_slot, r.gallery_source, r.admitted) == (1, "auto", False)

    night = _provider([(night_gray(), [], 8)] * 2)
    notices = [dev.sample(g, night).notice for _ in range(6)]
    assert notices == [False] * 4 + [True, False]
    assert g.count == 2


def test_provider_exception_propagates(core):
    ref = core.new_gallery()
    ref.add_manual(dry_gray())

    def broken(attempt):
        raise ValueError("boom")

    with pytest.raises(ValueError, match="boom"):
        core.new_device().sample(ref, broken)


def test_needs_rebuild(tmp_path):
    stamp = tmp_path / "floodsim.stamp"
    src = tmp_path / "a.c"
    src.write_text("int a;")
    assert needs_rebuild(stamp, [src]) is True
    stamp.write_bytes(b"")
    old = time.time() - 100
    os.utime(src, (old, old))
    assert needs_rebuild(stamp, [src]) is False
    src.write_text("int b;")
    os.utime(src, (time.time() + 5, time.time() + 5))
    assert needs_rebuild(stamp, [src]) is True


def test_firmware_dir_override_missing(monkeypatch, tmp_path):
    monkeypatch.setenv("FLOOD_FIRMWARE_DIR", str(tmp_path / "nope"))
    with pytest.raises(FloodSimError, match="FLOOD_FIRMWARE_DIR"):
        firmware_dir()


def test_build_error_message_hints_locked_dll():
    msg = build_error_message(["cmake", "--build", "x"], "", "ld: cannot open output file flood_core.dll: Permission denied")
    assert "Permission denied" in msg
    assert "close other floodsim" in msg


def test_gallery_delete_slot_and_clear_auto(core):
    g = core.new_gallery()
    g.add_manual(dry_gray())
    dev = core.new_device()
    dusk = _provider([(dusk_gray(), [], 7)] * 2)
    dev.sample(g, dusk)
    dev.sample(g, dusk)                                      # proven: learned in slot 1
    assert [(i, src) for i, src, _ in g.slots()] == [(0, "manual"), (1, "auto")]
    assert not g.delete_slot(1, "manual") and not g.delete_slot(5, "auto")
    assert g.clear_auto() == 1 and g.clear_auto() == 0
    assert [(i, src) for i, src, _ in g.slots()] == [(0, "manual")]
    assert g.delete_slot(0, "manual") and g.count == 0
    assert dev.sample(g, dusk).outcome == "skipped_no_ref"


def test_device_forget_candidate(core):
    g = core.new_gallery()
    g.add_manual(dry_gray())
    dev = core.new_device()
    dusk = _provider([(dusk_gray(), [], 7)] * 2)
    assert dev.sample(g, dusk).candidate_pending
    dev.forget_candidate()
    r = dev.sample(g, dusk)
    assert (r.admitted, r.candidate_pending, r.gallery_count) == (False, True, 1)


def test_gallery_rebuild_follows_the_rule(core):
    g = core.new_gallery()
    g.add_manual(dry_gray())                                   # slot 0
    g.add_manual(dusk_gray())                                  # slot 1
    assert g.rebuild({}) == "nothing" and g.count == 2
    assert g.rebuild({1: dusk_gray()}) == "done"
    assert [(i, src) for i, src, _ in g.slots()] == [(1, "manual")]
    assert not g.is_stale()


def test_device_learning_switch(core):
    g = core.new_gallery()
    g.add_manual(dry_gray())
    dev = core.new_device()
    dusk = _provider([(dusk_gray(), [], 7)] * 2)
    dev.set_learning(False)
    rows = [dev.sample(g, dusk) for _ in range(3)]
    assert not any(r.admitted or r.candidate_pending for r in rows)
    dev.set_learning(True)
    dev.sample(g, dusk)
    assert dev.sample(g, dusk).admitted

"""Firmware C vs the Python reference (ms500/curbcheck.py) at a fixed shift.

Skipped unless D:/git/MS500_Flooding_Detection with its dataset frames is present.
Registration is not compared: the firmware uses a direct vertical search, the
Python reference phase correlation (documented in flood_curbcheck.h)."""
import sys
from pathlib import Path

import pytest

from floodsim.source import Normalizer, read_image_bgr

REF_REPO = Path("D:/git/MS500_Flooding_Detection")
DRY = REF_REPO / "dataset" / "dry_reference_full.jpg"
FRAMES = sorted((REF_REPO / "dataset" / "frames").glob("*.jp*g")) if REF_REPO.exists() else []

pytestmark = pytest.mark.skipif(not DRY.exists() or not FRAMES, reason="reference repo/dataset not present")


def _gray(core, path):
    return core.rgb_to_gray(Normalizer(False)(read_image_bgr(path), path.name))


@pytest.mark.parametrize("frame", FRAMES, ids=lambda p: p.name)
def test_correlation_matches_python_reference(core, frame):
    sys.path.insert(0, str(REF_REPO))
    from ms500.curbcheck import CurbCheck

    ref_gray, gray = _gray(core, DRY), _gray(core, frame)
    ref = core.new_reference()
    assert ref.build(ref_gray)
    python = CurbCheck(ref_gray, register=False).check(gray).correlation
    assert core.correlation_at(ref, gray, 0, 0) == pytest.approx(python, abs=1e-5)

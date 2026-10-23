"""The operator-drawn upload area and the serial number, in the firmware's formats."""
from __future__ import annotations

import json
import re
from pathlib import Path

from .errors import FloodSimError

Rect = tuple[int, int, int, int]
_SN = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
_KEYS = ("x", "y", "width", "height")


def upload_area_path(input_path) -> Path:
    """<input>.upload_area.json, next to <input>.boxes.edited.csv."""
    return Path(f"{input_path}.upload_area.json")


def _rect_from_roi(roi, where: str) -> Rect:
    if not isinstance(roi, dict):
        raise FloodSimError(f"{where}: \"roi\" must be an object with x, y, width, height")
    values = []
    for k in _KEYS:
        v = roi.get(k)
        if isinstance(v, bool) or not isinstance(v, int):
            raise FloodSimError(f"{where}: roi.{k} must be a whole number")
        values.append(v)
    return tuple(values)


def load_upload_area(path) -> Rect | None:
    """Reads the firmware GET /api/flood/upload_roi shape; missing file or configured=false is None."""
    path = Path(path)
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as e:
        raise FloodSimError(f"cannot read upload area {path}: {e}") from e
    if not isinstance(data, dict) or not data.get("configured"):
        return None
    return _rect_from_roi(data.get("roi"), str(path))


def save_upload_area(path, rect: Rect | None) -> None:
    path = Path(path)
    if rect is None:
        path.unlink(missing_ok=True)
        return
    path.write_text(json.dumps({"configured": True, "roi": dict(zip(_KEYS, map(int, rect)))}, indent=2) + "\n",
                    encoding="utf-8")


def parse_upload_area(value: str) -> Rect | None:
    """--upload-area: "x,y,w,h", or a JSON file in the firmware API shape."""
    text = value.strip()
    parts = [p.strip() for p in text.split(",")]
    if len(parts) == 4 and all(re.fullmatch(r"-?\d+", p) for p in parts):
        return tuple(int(p) for p in parts)
    if text.lower().endswith(".json"):
        if not Path(text).exists():
            raise FloodSimError(f"--upload-area file not found: {text}")
        return load_upload_area(text)
    raise FloodSimError(f"--upload-area must be x,y,w,h (whole numbers) or a .json file, got {value!r}")


def check_sn(sn: str) -> str:
    if not _SN.match(sn or ""):
        raise FloodSimError("serial number must be 1-64 letters, digits, '-' or '_'")
    return sn

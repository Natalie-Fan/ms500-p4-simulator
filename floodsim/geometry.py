"""Curb/lane geometry in the firmware web API's JSON shape."""
from __future__ import annotations

import json
from dataclasses import astuple, dataclass
from pathlib import Path

from .errors import FloodSimError

_LINES = ("curb", "lane")
_KEYS = ("x0", "y0", "x1", "y1")


@dataclass(frozen=True)
class Geometry:
    curb_x0: float
    curb_y0: float
    curb_x1: float
    curb_y1: float
    lane_x0: float
    lane_y0: float
    lane_x1: float
    lane_y1: float

    def to_list(self) -> list[float]:
        return list(astuple(self))

    @classmethod
    def from_list(cls, values) -> Geometry:
        values = [float(v) for v in values]
        if len(values) != 8:
            raise FloodSimError(f"geometry needs 8 numbers, got {len(values)}")
        return cls(*values)

    def to_json_dict(self) -> dict:
        v = self.to_list()
        return {line: dict(zip(_KEYS, v[i * 4:i * 4 + 4])) for i, line in enumerate(_LINES)}


def load_geometry(path: Path) -> Geometry:
    """Reads {"curb": {x0,y0,x1,y1}, "lane": {...}}; other keys are ignored."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as e:
        raise FloodSimError(f"cannot read geometry {path}: {e}") from e
    values = []
    for line in _LINES:
        obj = data.get(line) if isinstance(data, dict) else None
        if not isinstance(obj, dict):
            raise FloodSimError(f"{path}: missing \"{line}\" object")
        for key in _KEYS:
            v = obj.get(key)
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                raise FloodSimError(f"{path}: {line}.{key} must be a number")
            values.append(float(v))
    return Geometry.from_list(values)


def save_geometry(g: Geometry, path: Path) -> None:
    Path(path).write_text(json.dumps(g.to_json_dict(), indent=2) + "\n", encoding="utf-8")

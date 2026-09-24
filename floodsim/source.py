"""Image / image-folder / video inputs as RGB 1920x1080 frames with timestamps."""
from __future__ import annotations

import bisect
import logging
import re
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np

from .errors import FloodSimError

FRAME_W, FRAME_H = 1920, 1080
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".pgm", ".ppm", ".tif", ".tiff"}
_T_MS = re.compile(r"_t(\d+)")
log = logging.getLogger(__name__)


def read_image_bgr(path: Path) -> np.ndarray | None:
    """cv2.imread cannot open non-ASCII paths on Windows, so decode from bytes."""
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def write_image(path: Path, img: np.ndarray) -> None:
    ok, buf = cv2.imencode(Path(path).suffix, img)
    if not ok:
        raise FloodSimError(f"cannot encode {path}")
    buf.tofile(str(path))


class Normalizer:
    """BGR frame of any size -> RGB 1920x1080, refusing non-16:9 unless allowed."""

    def __init__(self, allow_stretch: bool):
        self.allow_stretch = allow_stretch
        self._warned = False

    def __call__(self, bgr: np.ndarray, where: str) -> np.ndarray:
        h, w = bgr.shape[:2]
        if (w, h) != (FRAME_W, FRAME_H):
            if abs(w / h - FRAME_W / FRAME_H) > 0.01 and not self.allow_stretch:
                raise FloodSimError(
                    f"{where}: {w}x{h} is not 16:9, so the curb geometry would not line up "
                    f"(use --allow-stretch to resize anyway)")
            if not self._warned:
                log.warning("%s: resizing %dx%d to %dx%d", where, w, h, FRAME_W, FRAME_H)
                self._warned = True
            bgr = cv2.resize(bgr, (FRAME_W, FRAME_H), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        return rgb


class FrameSource:
    """Frame i has timestamp(i) seconds; read_rgb(i) is read-only RGB 1080x1920x3, or None."""

    path: Path
    n_frames: int
    _times: list[float]

    def timestamp(self, idx: int) -> float:
        return self._times[idx]

    def read_rgb(self, idx: int) -> np.ndarray | None:
        raise NotImplementedError

    def frame_name(self, idx: int) -> str | None:
        return None

    def fingerprint(self) -> dict:
        raise NotImplementedError

    @property
    def duration(self) -> float:
        return self.timestamp(self.n_frames - 1)

    def index_at(self, t: float) -> int | None:
        """Latest frame at or before t -- what a camera polled at t would hand over."""
        i = bisect.bisect_right(self._times, t + 1e-9) - 1
        return i if i >= 0 else None


class ImageSequence(FrameSource):
    def __init__(self, path: Path, files: list[Path], fps: float | None, normalizer: Normalizer):
        self.path = path
        self._normalize = normalizer
        matches = [_T_MS.search(f.stem) for f in files]
        if all(matches):
            pairs = sorted(zip((int(m.group(1)) / 1000.0 for m in matches), files), key=lambda p: p[0])
            self._times = [t for t, _ in pairs]
            self._files = [f for _, f in pairs]
        elif len(files) == 1:
            self._times, self._files = [0.0], files
        elif fps:
            self._times, self._files = [i / fps for i in range(len(files))], files
        else:
            raise FloodSimError(
                f"{path}: image frames need --fps or file names containing _t<ms> "
                f"(e.g. f000001_t0000060000.png)")
        self.n_frames = len(self._files)

    def read_rgb(self, idx: int) -> np.ndarray | None:
        f = self._files[idx]
        bgr = read_image_bgr(f)
        if bgr is None:
            log.warning("cannot decode %s", f)
            return None
        return self._normalize(bgr, f.name)

    def frame_name(self, idx: int) -> str | None:
        return self._files[idx].name

    def fingerprint(self) -> dict:
        stats = [f.stat() for f in self._files]
        return {"path": str(self.path), "n": self.n_frames,
                "size": sum(s.st_size for s in stats), "mtime": max(s.st_mtime for s in stats)}


class VideoSource(FrameSource):
    _CACHE = 16

    def __init__(self, path: Path, fps: float | None, normalizer: Normalizer):
        self.path = path
        self._normalize = normalizer
        self._cap = self._open()
        fps = fps or self._cap.get(cv2.CAP_PROP_FPS)
        if not fps or fps <= 0:
            raise FloodSimError(f"{path}: video reports no frame rate; pass --fps")
        n = 0
        while self._cap.grab():  # count by walking: container frame counts lie for MJPEG AVIs
            n += 1
        if n == 0:
            raise FloodSimError(f"{path}: no decodable frames")
        self.n_frames = n
        self._times = [i / fps for i in range(n)]
        self._cap.release()
        self._cap = self._open()
        self._pos = 0
        self._cache: OrderedDict[int, np.ndarray] = OrderedDict()

    def _open(self) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(str(self.path))
        if not cap.isOpened():
            hint = "" if str(self.path).isascii() else \
                " (OpenCV cannot open videos at non-ASCII paths on Windows; move or rename it)"
            raise FloodSimError(f"cannot open video {self.path}{hint}")
        return cap

    def read_rgb(self, idx: int) -> np.ndarray | None:
        if idx in self._cache:
            self._cache.move_to_end(idx)
            return self._cache[idx]
        if idx < self._pos:
            self._cap.release()
            self._cap = self._open()
            self._pos = 0
        while self._pos < idx:
            if not self._cap.grab():
                self._pos = self.n_frames  # force a reopen next time
                return None
            self._pos += 1
        ok, bgr = self._cap.read()
        self._pos += 1
        if not ok or bgr is None:
            log.warning("%s: frame %d failed to decode", self.path.name, idx)
            return None
        rgb = self._normalize(bgr, f"{self.path.name} frame {idx}")
        self._cache[idx] = rgb
        if len(self._cache) > self._CACHE:
            self._cache.popitem(last=False)
        return rgb

    def fingerprint(self) -> dict:
        s = self.path.stat()
        return {"path": str(self.path), "size": s.st_size, "mtime": s.st_mtime}


def open_source(path: Path, *, fps: float | None = None, allow_stretch: bool = False) -> FrameSource:
    path = Path(path)
    if not path.exists():
        raise FloodSimError(f"input not found: {path}")
    normalizer = Normalizer(allow_stretch)
    if path.is_dir():
        files = sorted(f for f in path.iterdir() if f.suffix.lower() in IMAGE_EXTS)
        if not files:
            raise FloodSimError(f"{path}: no image files")
        src: FrameSource = ImageSequence(path, files, fps, normalizer)
    elif path.suffix.lower() in IMAGE_EXTS:
        src = ImageSequence(path, [path], fps, normalizer)
    else:
        src = VideoSource(path, fps, normalizer)
    src.read_rgb(0)  # fail fast on a bad aspect ratio
    return src

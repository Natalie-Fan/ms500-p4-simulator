"""Builds and loads flood_core.dll (firmware C + shim) and wraps it with ctypes."""
from __future__ import annotations

import ctypes
import os
import subprocess
from ctypes import POINTER, byref, c_char_p, c_float, c_int32, c_uint8, c_void_p
from pathlib import Path
from typing import Callable

import numpy as np

from .errors import BuildError, FloodSimError
from .geometry import Geometry
from .results import Box, DeviceResult, ScoreResult

REPO_ROOT = Path(__file__).resolve().parent.parent
NATIVE_DIR = REPO_ROOT / "native"
BUILD_DIR = REPO_ROOT / "build" / "native"
DEFAULT_FIRMWARE_DIR = Path("D:/git/NETWORK_P4_FLOOD_DETECTION")
FLOOD_SUBDIR = "components/as_code/process_flood_detection"
DLL_NAME = "flood_core.dll"
EXE_NAME = "flood_sim.exe"
STAMP_NAME = "floodsim.stamp"
ABI_VERSION = 2
FRAME_W, FRAME_H = 1920, 1080
MAX_BOXES = 32

Provider = Callable[[int], "tuple[np.ndarray, list[Box], int] | None"]


# ------------------------------------------------------------------ building

def firmware_dir() -> Path:
    env = os.environ.get("FLOOD_FIRMWARE_DIR")
    fw = Path(env) if env else DEFAULT_FIRMWARE_DIR
    if not (fw / FLOOD_SUBDIR / "flood_capture.h").exists():
        raise FloodSimError(
            f"firmware flood sources not found under {fw} "
            f"(expected {FLOOD_SUBDIR}); set FLOOD_FIRMWARE_DIR to the firmware repo")
    return fw


def source_files(fw: Path) -> list[Path]:
    flood = fw / FLOOD_SUBDIR
    files = [*flood.glob("*.c"), *flood.glob("*.h")]
    files += [p for p in NATIVE_DIR.rglob("*") if p.suffix in {".c", ".h", ".txt", ".cmake"}]
    return files


def needs_rebuild(stamp: Path, sources: list[Path]) -> bool:
    """True if any source is newer than the last successful build (`stamp`)."""
    if not stamp.exists():
        return True
    built = stamp.stat().st_mtime
    return any(p.stat().st_mtime > built for p in sources)


def configured_firmware(build_dir: Path) -> Path | None:
    marker = build_dir / "firmware_dir.txt"
    return Path(marker.read_text().strip()) if marker.exists() else None


def build_error_message(cmd: list[str], stdout: str, stderr: str) -> str:
    msg = f"{' '.join(cmd)} failed:\n{stdout}\n{stderr}".rstrip()
    if "Permission denied" in stdout + stderr:
        msg += ("\n\nThe DLL is probably loaded by another process; "
                "close other floodsim windows/runs and try again.")
    return msg


def build(fw: Path, build_dir: Path = BUILD_DIR) -> None:
    commands = [
        ["cmake", "-S", str(NATIVE_DIR), "-B", str(build_dir), "-G", "Ninja",
         "-DCMAKE_BUILD_TYPE=Release", f"-DFIRMWARE_DIR={fw.as_posix()}"],
        ["cmake", "--build", str(build_dir)],
    ]
    for cmd in commands:
        p = subprocess.run(cmd, capture_output=True, text=True)
        if p.returncode != 0:
            raise BuildError(build_error_message(cmd, p.stdout, p.stderr))


def firmware_provenance(fw: Path) -> dict:
    def git(*args: str) -> str | None:
        try:
            p = subprocess.run(["git", "-C", str(fw), *args], capture_output=True, text=True)
        except OSError:
            return None
        return p.stdout.strip() if p.returncode == 0 else None

    commit = git("rev-parse", "HEAD")
    status = git("status", "--porcelain", "--", FLOOD_SUBDIR)
    return {"commit": commit, "dirty": None if status is None else bool(status)}


_LOADED: tuple["Core", dict] | None = None


def load_core(*, rebuild: bool = True) -> tuple["Core", dict]:
    """Rebuild if firmware or shim sources are newer than the DLL, then load it once per process."""
    global _LOADED
    if _LOADED is not None:
        return _LOADED
    fw = firmware_dir()
    dll = BUILD_DIR / DLL_NAME
    # A stamp, not the DLL's mtime: ninja only relinks the DLL when its own
    # inputs change, so an edited test file would otherwise look stale forever.
    stamp = BUILD_DIR / STAMP_NAME
    configured = configured_firmware(BUILD_DIR)
    stale = (not dll.exists() or needs_rebuild(stamp, source_files(fw))
             or configured is None or configured.resolve() != fw.resolve())
    if stale and rebuild:
        build(fw)
        stamp.touch()
        stale = False
    if not dll.exists():
        raise FloodSimError(f"{dll} does not exist; run without --no-rebuild to build it")
    info = {
        "dll": {"path": str(dll), "abi": ABI_VERSION, "stale": stale},
        "firmware": {"dir": str(fw), **firmware_provenance(fw)},
    }
    _LOADED = (Core(dll), info)
    return _LOADED


# ------------------------------------------------------------------ ctypes

class _SimBox(ctypes.Structure):
    _fields_ = [("x0", c_float), ("y0", c_float), ("x1", c_float), ("y1", c_float),
                ("confidence", c_float), ("cls", c_int32)]


class _SimScore(ctypes.Structure):
    _fields_ = [("veto_usable", c_int32), ("veto_coverage", c_float), ("veto_worst_class", c_int32),
                ("curb_correlation", c_float), ("submerged", c_int32), ("severity", c_int32),
                ("shift_dx", c_int32), ("shift_dy", c_int32)]


class _SimResult(ctypes.Structure):
    _fields_ = [("outcome", c_int32), ("frames_taken", c_int32), ("has_reading", c_int32),
                ("curb_correlation", c_float), ("severity", c_int32), ("shift_dx", c_int32),
                ("shift_dy", c_int32), ("veto_usable", c_int32), ("veto_coverage", c_float),
                ("veto_worst_class", c_int32), ("since_image", c_int32), ("frame_idx", c_int32)]


_PROVIDER_FN = ctypes.CFUNCTYPE(c_int32, c_int32, POINTER(c_uint8), POINTER(_SimBox),
                                POINTER(c_int32), POINTER(c_int32), c_void_p)
_F8 = c_float * 8
_U8P = POINTER(c_uint8)
_FP = POINTER(c_float)


def _u8(a: np.ndarray):
    return a.ctypes.data_as(_U8P)


def _check_gray(gray: np.ndarray) -> np.ndarray:
    gray = np.ascontiguousarray(gray)
    if gray.shape != (FRAME_H, FRAME_W) or gray.dtype != np.uint8:
        raise FloodSimError(f"expected a {FRAME_H}x{FRAME_W} uint8 gray frame, got {gray.shape} {gray.dtype}")
    return gray


def _sim_boxes(boxes: list[Box]):
    n = min(len(boxes), MAX_BOXES)
    arr = (_SimBox * max(n, 1))()
    for i, b in enumerate(boxes[:n]):
        arr[i] = _SimBox(b.x0, b.y0, b.x1, b.y1, b.confidence, b.cls)
    return arr, n


class Core:
    def __init__(self, dll_path: Path):
        self._lib = ctypes.CDLL(str(dll_path))
        self._declare()
        abi = self._lib.sim_abi_version()
        if abi != ABI_VERSION:
            raise FloodSimError(f"DLL ABI v{abi}, wrapper expects v{ABI_VERSION} -- rebuild")
        for fn, struct in (("sim_sizeof_box", _SimBox), ("sim_sizeof_score", _SimScore),
                           ("sim_sizeof_result", _SimResult)):
            got = getattr(self._lib, fn)()
            if got != ctypes.sizeof(struct):
                raise FloodSimError(f"{fn}: DLL {got} bytes, wrapper {ctypes.sizeof(struct)} -- rebuild")

    def _declare(self) -> None:
        lib = self._lib

        def fn(name, restype, *argtypes):
            f = getattr(lib, name)
            f.restype = restype
            f.argtypes = list(argtypes)

        for name in ("sim_abi_version", "sim_sizeof_box", "sim_sizeof_score", "sim_sizeof_result",
                     "sim_upload_downscale", "sim_heartbeat_every"):
            fn(name, c_int32)
        for name in ("sim_submerged_below", "sim_veto_threshold", "sim_sample_period_s",
                     "sim_domain_x0", "sim_domain_x1"):
            fn(name, c_float)
        for name in ("sim_outcome_name", "sim_severity_name", "sim_class_name"):
            fn(name, c_char_p, c_int32)
        fn("sim_default_geometry", None, _FP)
        fn("sim_get_geometry", None, _FP)
        fn("sim_set_geometry", None, _FP)
        fn("sim_validate_geometry", c_int32, _FP, c_char_p, c_int32)
        fn("sim_align_patch", None, POINTER(c_int32))
        fn("sim_strip_outline", c_int32, _FP, _FP, _FP, c_int32)
        fn("sim_rgb_to_gray", None, _U8P, c_int32, c_int32, _U8P)
        fn("sim_ref_new", c_void_p)
        fn("sim_ref_free", None, c_void_p)
        fn("sim_ref_build", c_int32, c_void_p, _U8P)
        fn("sim_ref_ready", c_int32, c_void_p)
        fn("sim_correlation_at", c_float, c_void_p, _U8P, c_int32, c_int32)
        fn("sim_box_from_accelerator", None, c_float, c_float, c_float, c_float, c_float,
           c_int32, c_int32, c_int32, POINTER(_SimBox))
        fn("sim_score", c_int32, c_void_p, _U8P, POINTER(_SimBox), c_int32, POINTER(_SimScore))
        fn("sim_band_coverage", c_float, POINTER(_SimBox))
        fn("sim_device_new", c_void_p)
        fn("sim_device_free", None, c_void_p)
        fn("sim_device_reset", None, c_void_p)
        fn("sim_device_sample", None, c_void_p, c_void_p, _PROVIDER_FN, c_void_p, POINTER(_SimResult))

    # names and constants
    def outcome_name(self, v: int) -> str: return self._lib.sim_outcome_name(v).decode()
    def severity_name(self, v: int) -> str: return self._lib.sim_severity_name(v).decode()
    def class_name(self, v: int) -> str: return self._lib.sim_class_name(v).decode()
    def submerged_below(self) -> float: return self._lib.sim_submerged_below()
    def veto_threshold(self) -> float: return self._lib.sim_veto_threshold()
    def sample_period_s(self) -> float: return self._lib.sim_sample_period_s()
    def upload_downscale(self) -> int: return self._lib.sim_upload_downscale()

    def heartbeat_every(self) -> int: return self._lib.sim_heartbeat_every()

    def band_coverage(self, box: Box) -> float:
        b = _SimBox(box.x0, box.y0, box.x1, box.y1, box.confidence, box.cls)
        return self._lib.sim_band_coverage(byref(b))

    # geometry
    def default_geometry(self) -> list[float]:
        a = _F8()
        self._lib.sim_default_geometry(a)
        return list(a)

    def get_geometry(self) -> list[float]:
        a = _F8()
        self._lib.sim_get_geometry(a)
        return list(a)

    def set_geometry(self, g: Geometry) -> None:
        self._lib.sim_set_geometry(_F8(*g.to_list()))

    def validate_geometry(self, g: Geometry) -> str | None:
        err = ctypes.create_string_buffer(256)
        ok = self._lib.sim_validate_geometry(_F8(*g.to_list()), err, len(err))
        return None if ok else err.value.decode()

    def domain(self) -> tuple[float, float]:
        return self._lib.sim_domain_x0(), self._lib.sim_domain_x1()

    def align_patch(self) -> tuple[int, int, int, int]:
        a = (c_int32 * 4)()
        self._lib.sim_align_patch(a)
        return tuple(a)

    def strip_outline(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        cap = 512
        xs, lo, hi = (np.zeros(cap, np.float32) for _ in range(3))
        n = self._lib.sim_strip_outline(xs.ctypes.data_as(_FP), lo.ctypes.data_as(_FP),
                                        hi.ctypes.data_as(_FP), cap)
        return xs[:n].copy(), lo[:n].copy(), hi[:n].copy()

    # frames and scoring
    def rgb_to_gray(self, rgb: np.ndarray) -> np.ndarray:
        rgb = np.ascontiguousarray(rgb)
        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
            raise FloodSimError(f"expected an HxWx3 uint8 RGB frame, got {rgb.shape} {rgb.dtype}")
        h, w = rgb.shape[:2]
        out = np.empty((h, w), np.uint8)
        self._lib.sim_rgb_to_gray(_u8(rgb), w, h, _u8(out))
        return out

    def new_reference(self) -> Reference:
        return Reference(self)

    def score(self, ref: Reference | None, gray: np.ndarray, boxes: list[Box]) -> ScoreResult:
        if ref is None or not ref.ready:
            raise FloodSimError("no dry reference: set one before scoring")
        gray = _check_gray(gray)
        arr, n = _sim_boxes(boxes)
        s = _SimScore()
        self._lib.sim_score(ref.ptr, _u8(gray), arr, n, byref(s))
        return ScoreResult(
            veto_usable=bool(s.veto_usable), veto_coverage=s.veto_coverage,
            veto_worst_class=self.class_name(s.veto_worst_class),
            curb_correlation=s.curb_correlation, submerged=bool(s.submerged),
            severity=self.severity_name(s.severity), shift_dx=s.shift_dx, shift_dy=s.shift_dy)

    def correlation_at(self, ref: Reference, gray: np.ndarray, dx: int, dy: int) -> float:
        return self._lib.sim_correlation_at(ref.ptr, _u8(_check_gray(gray)), dx, dy)

    def box_from_accelerator(self, x0, y0, x1, y1, conf, cls, input_w, input_h) -> Box:
        b = _SimBox()
        self._lib.sim_box_from_accelerator(x0, y0, x1, y1, conf, cls, input_w, input_h, byref(b))
        return Box(b.x0, b.y0, b.x1, b.y1, b.confidence, b.cls)

    def new_device(self) -> Device:
        return Device(self)

    def _device_result(self, r: _SimResult) -> DeviceResult:
        return DeviceResult(
            outcome=self.outcome_name(r.outcome), frames_taken=r.frames_taken,
            has_reading=bool(r.has_reading), curb_correlation=r.curb_correlation,
            severity=self.severity_name(r.severity), shift_dx=r.shift_dx, shift_dy=r.shift_dy,
            veto_usable=bool(r.veto_usable), veto_coverage=r.veto_coverage,
            veto_worst_class=self.class_name(r.veto_worst_class), since_image=r.since_image,
            frame_idx=r.frame_idx)


class Reference:
    """Opaque firmware flood_reference_t, built from a dry gray frame."""

    def __init__(self, core: Core):
        self._core = core
        self.ptr = core._lib.sim_ref_new()
        if not self.ptr:
            raise MemoryError("sim_ref_new failed")
        self.geometry: tuple[float, ...] | None = None

    def build(self, gray: np.ndarray) -> bool:
        ok = bool(self._core._lib.sim_ref_build(self.ptr, _u8(_check_gray(gray))))
        self.geometry = tuple(self._core.get_geometry()) if ok else None
        return ok

    @property
    def ready(self) -> bool:
        return bool(self.ptr) and bool(self._core._lib.sim_ref_ready(self.ptr))

    def is_stale(self) -> bool:
        """Built under different geometry than the current one (firmware: geometry_version)."""
        return self.ready and self.geometry != tuple(self._core.get_geometry())

    def close(self) -> None:
        if self.ptr:
            self._core._lib.sim_ref_free(self.ptr)
            self.ptr = None

    def __del__(self):
        self.close()


class Device:
    """Firmware capture state (retry, heartbeat) driven by a frame provider."""

    def __init__(self, core: Core):
        self._core = core
        self._ptr = core._lib.sim_device_new()
        if not self._ptr:
            raise MemoryError("sim_device_new failed")

    def reset(self) -> None:
        self._core._lib.sim_device_reset(self._ptr)

    def sample(self, ref: Reference | None, provider: Provider) -> DeviceResult:
        errors: list[BaseException] = []

        def callback(attempt, gray_out, boxes_out, n_out, idx_out, _user):
            try:
                got = provider(attempt)
                if got is None:
                    return 0
                gray, boxes, frame_idx = got
                gray = _check_gray(gray)
                ctypes.memmove(gray_out, gray.ctypes.data, gray.nbytes)
                n = min(len(boxes), MAX_BOXES)
                for i, b in enumerate(boxes[:n]):
                    boxes_out[i] = _SimBox(b.x0, b.y0, b.x1, b.y1, b.confidence, b.cls)
                n_out[0] = n
                idx_out[0] = frame_idx
                return 1
            except BaseException as e:  # re-raised after the C call returns
                errors.append(e)
                return 0

        cb = _PROVIDER_FN(callback)
        res = _SimResult()
        self._core._lib.sim_device_sample(self._ptr, ref.ptr if ref is not None else None,
                                          cb, None, byref(res))
        if errors:
            raise errors[0]
        return self._core._device_result(res)

    def close(self) -> None:
        if self._ptr:
            self._core._lib.sim_device_free(self._ptr)
            self._ptr = None

    def __del__(self):
        self.close()

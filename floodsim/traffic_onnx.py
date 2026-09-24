"""The device's traffic model (YOLOv8, MCT export) on the PC via onnxruntime.

Decoding mirrors the firmware: Inference::runInference_traffic() in
components/as_code/process_traffic/inference_yolo_traffic.cpp, with the anchors
and strides of components/as_code/process_common/inference_common.h."""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from .errors import FloodSimError

INPUT_SIZE = (416, 416)
# inference_traffic.cpp:181 -- the model's class_id order.
TRAFFIC_CLASSES = ("person", "bicycle", "car", "motorcycle", "bus", "truck")


def _anchors() -> tuple[np.ndarray, np.ndarray]:
    pts, strides = [], []
    for s in (8, 16, 32):
        n = INPUT_SIZE[0] // s
        ys, xs = np.mgrid[0:n, 0:n]
        pts.append(np.stack([xs.ravel() + 0.5, ys.ravel() + 0.5], axis=1))
        strides.append(np.full(n * n, s))
    return np.concatenate(pts).astype(np.float32), np.concatenate(strides).astype(np.float32)


ANCHORS, STRIDES = _anchors()   # same values as spiffs_data/dnn/anchors.txt
N_ANCHORS = len(ANCHORS)        # 3549


def decode(box: np.ndarray, scores: np.ndarray, conf: float, nms: float) -> list:
    """box: (4, N) ltrb anchor offsets; scores: (classes, N). Returns Detections in input pixels."""
    from .boxes import Detection
    cls = scores.argmax(axis=0)
    best = scores.max(axis=0)
    keep = np.flatnonzero(best > conf)
    rects, confs = [], []
    for i in keep:
        l, t, r, b = (float(v) for v in box[:, i])
        ax, ay = float(ANCHORS[i, 0]), float(ANCHORS[i, 1])
        mul = float(STRIDES[i])
        x1, y1, x2, y2 = ax - l, ay - t, ax + r, ay + b
        x, y = (x1 + x2) / 2 * mul, (y1 + y2) / 2 * mul
        w, h = (x2 - x1) * mul, (y2 - y1) * mul
        rects.append([int(x - 0.5 * w), int(y - 0.5 * h), int(w), int(h)])   # C casts truncate
        confs.append(float(best[i]))
    if not rects:
        return []
    out = []
    for k in np.array(cv2.dnn.NMSBoxes(rects, confs, conf, nms)).flatten():
        left, top, width, height = rects[k]
        out.append(Detection(left, top, left + width, top + height, confs[k],
                             TRAFFIC_CLASSES[int(cls[keep[k]])]))
    return out


class OnnxTrafficDetector:
    def __init__(self, model: str, conf: float, nms: float = 0.20):
        try:
            import onnxruntime as ort
        except ImportError as e:
            raise FloodSimError(
                "onnxruntime is not installed (pip install -e .[onnx]); "
                "use --no-detector to run without Stage-1 boxes") from e
        if not Path(model).is_file():
            raise FloodSimError(f"detection model not found: {model}")
        self.model_name = model
        self.conf = conf
        self.nms = nms
        self._session = ort.InferenceSession(model, providers=["CPUExecutionProvider"])
        self._input = self._session.get_inputs()[0].name

    def detect(self, rgb_small: np.ndarray) -> list:
        x = rgb_small.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        box, scores = self._session.run(None, {self._input: x})
        return decode(box[0], scores[0], self.conf, self.nms)

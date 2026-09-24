from pathlib import Path

import numpy as np
import pytest

from floodsim.core import firmware_dir
from floodsim.traffic_onnx import ANCHORS, N_ANCHORS, STRIDES, TRAFFIC_CLASSES, decode


def _tensors():
    return np.zeros((4, N_ANCHORS), np.float32), np.zeros((len(TRAFFIC_CLASSES), N_ANCHORS), np.float32)


def test_classes_in_firmware_order():
    # components/as_code/process_traffic/inference_traffic.cpp:181
    assert TRAFFIC_CLASSES == ("person", "bicycle", "car", "motorcycle", "bus", "truck")


def test_anchors_match_firmware_file():
    path = firmware_dir() / "spiffs_data" / "dnn" / "anchors.txt"
    if not path.exists():
        pytest.skip(f"{path} not found")
    assert np.array_equal(ANCHORS, np.loadtxt(path, dtype=np.float32))


def test_strides_follow_firmware_index_ranges():
    # inference_common.h: stride{{2703, 8}, {3379, 16}, {3548, 32}}, chosen with `i <= end`
    assert (STRIDES[0], STRIDES[2703], STRIDES[2704], STRIDES[3379], STRIDES[3380], STRIDES[3548]) \
        == (8, 8, 16, 16, 32, 32)


def test_decode_anchor_offsets_to_pixels():
    box, sc = _tensors()
    box[:, 0] = [0.5, 0.5, 1.5, 1.5]      # anchor (0.5, 0.5), stride 8
    sc[2, 0] = 0.9
    [d] = decode(box, sc, conf=0.5, nms=0.2)
    assert (d.x0, d.y0, d.x1, d.y1, d.coco_class) == (0, 0, 16, 16, "car")
    assert d.conf == pytest.approx(0.9)


def test_score_must_exceed_threshold():
    box, sc = _tensors()
    box[:, 0] = [0.5, 0.5, 1.5, 1.5]
    sc[0, 0] = 0.5
    assert decode(box, sc, conf=0.5, nms=0.2) == []


def test_nms_is_across_classes():
    box, sc = _tensors()
    box[:, 0] = box[:, 1] = [0.5, 0.5, 1.5, 1.5]   # anchors (0.5,0.5) and (1.5,0.5): IoU 1/3
    sc[2, 0] = 0.9     # car
    sc[5, 1] = 0.8     # truck overlapping it -> suppressed like the firmware's NMSBoxes
    assert [d.coco_class for d in decode(box, sc, conf=0.5, nms=0.2)] == ["car"]


def test_real_model_loads_and_runs():
    from floodsim.boxes import DEFAULT_MODEL
    from floodsim.traffic_onnx import OnnxTrafficDetector
    if not Path(DEFAULT_MODEL).exists():
        pytest.skip("models/traffic.onnx not present")
    det = OnnxTrafficDetector(DEFAULT_MODEL, 0.5)
    assert det.detect(np.zeros((416, 416, 3), np.uint8)) == []

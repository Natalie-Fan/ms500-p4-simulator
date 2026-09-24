import pytest

from floodsim.errors import FloodSimError
from floodsim.labels import Labels, score_rows


def _write(tmp_path, text, name="labels.csv", encoding="utf-8"):
    p = tmp_path / name
    p.write_bytes(text.encode(encoding))
    return p


def test_interval_labels_and_lookup(tmp_path):
    labels = Labels.load(_write(tmp_path, "start_sec,end_sec,label\n0,10,DRY\n10,20,flooded\n"))
    assert labels.label_for(0.0, None) == "DRY"
    assert labels.label_for(10.0, None) == "FLOODED"
    assert labels.label_for(20.0, None) is None
    assert labels.warnings == []


def test_labels_with_bom_and_crlf(tmp_path):
    labels = Labels.load(_write(tmp_path, "start_sec,end_sec,label\r\n0,5,WET\r\n", encoding="utf-8-sig"))
    assert labels.label_for(1.0, None) == "WET"


def test_file_labels(tmp_path):
    labels = Labels.load(_write(tmp_path, "file,label\na.png,DRY\nb.png,INDETERMINATE\n"))
    assert labels.label_for(0.0, "b.png") == "INDETERMINATE"
    assert labels.label_for(0.0, "c.png") is None


def test_gap_and_overlap_warnings(tmp_path):
    labels = Labels.load(_write(tmp_path, "start_sec,end_sec,label\n0,5,DRY\n4,8,WET\n10,12,DRY\n"))
    assert any("overlap" in w for w in labels.warnings)
    assert any("gap" in w for w in labels.warnings)


def test_unknown_label_rejected(tmp_path):
    with pytest.raises(FloodSimError, match="unknown label"):
        Labels.load(_write(tmp_path, "start_sec,end_sec,label\n0,5,DAMP\n"))


def test_wrong_columns_rejected(tmp_path):
    with pytest.raises(FloodSimError, match="expected columns"):
        Labels.load(_write(tmp_path, "a,b\n1,2\n"))


def _dev(outcome, corr, label, reading=True):
    return {"outcome": outcome, "curb_correlation": corr, "has_reading": reading, "label": label}


def test_score_device_rows_blocked_is_abstention():
    rows = [_dev("upload", 0.1, "FLOODED"), _dev("clear", 0.9, "DRY"), _dev("blocked", 0.0, "DRY", False),
            _dev("upload", 0.3, "WET"), _dev("clear", 0.8, "INDETERMINATE"), _dev("clear", 0.8, None),
            _dev("skipped_no_ref", 0.0, "DRY", False), _dev("heartbeat", 0.7, "FLOODED")]
    s = score_rows(rows, "device")
    assert (s["tp"], s["fp"], s["tn"], s["fn"]) == (1, 1, 1, 1)
    assert s["abstained"] == 1 and s["indeterminate"] == 1 and s["unlabelled"] == 1 and s["skipped"] == 1
    assert s["abstention_rate"] == pytest.approx(1 / 5)
    assert s["flooded_max_corr"] == pytest.approx(0.7)
    assert s["dry_wet_min_corr"] == pytest.approx(0.3)
    assert s["false_alarm_upper_95"] is None


def test_rule_of_three_when_no_false_alarms():
    rows = [_dev("clear", 0.9, "DRY")] * 30 + [_dev("upload", 0.1, "FLOODED")]
    s = score_rows(rows, "device")
    assert s["fp"] == 0 and s["false_alarm_upper_95"] == pytest.approx(0.1)


def test_dense_veto_is_abstention():
    rows = [{"veto_usable": False, "submerged": True, "curb_correlation": 0.1, "label": "DRY"},
            {"veto_usable": True, "submerged": True, "curb_correlation": 0.1, "label": "FLOODED"}]
    s = score_rows(rows, "dense")
    assert s["abstained"] == 1 and s["tp"] == 1 and s["fp"] == 0


def test_blank_excel_rows_are_ignored(tmp_path):
    labels = Labels.load(_write(tmp_path, "start_sec,end_sec,label\r\n0,5,DRY\r\n,,\r\n,,\r\n",
                                encoding="utf-8-sig"))
    assert labels.intervals == [(0.0, 5.0, "DRY")]
    files = Labels.load(_write(tmp_path, "file,label\na.png,WET\n,\n", name="f.csv"))
    assert files.files == {"a.png": "WET"}

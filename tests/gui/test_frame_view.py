import pytest
from PySide6.QtCore import QPointF, QRectF
from PySide6.QtWidgets import QGraphicsLineItem

from floodsim.boxes import CachedBox
from floodsim.gui.frame_view import FRAME_H, FrameView, Tool
from tests.synth import dry_gray, to_rgb


@pytest.fixture
def view(qtbot, core):
    v = FrameView()
    qtbot.addWidget(v)
    v.set_geometry_overlay({"geometry": core.default_geometry(), "strip": core.strip_outline(),
                            "align": core.align_patch(), "domain": core.domain()})
    return v


def _drag(v, a, b):
    v._press(QPointF(*a))
    v._move(QPointF(*b))
    v._release(QPointF(*b))


def test_image_and_overlays_drawn(view):
    view.set_image(to_rgb(dry_gray()))
    assert not view._pixmap.pixmap().isNull()
    assert len(view._overlay.childItems()) >= 5   # curb, lane, strip, 2 domain lines
    view.set_image(None)
    assert view._pixmap.pixmap().isNull()


def test_domain_lines_at_domain_edges(view):
    lines = [item for item in view._overlay.childItems() if isinstance(item, QGraphicsLineItem)]
    vertical = [ln for ln in lines
                if ln.line().x1() == ln.line().x2()
                and ln.line().y1() == 0.0 and ln.line().y2() == float(FRAME_H)]
    xs = sorted(ln.line().x1() for ln in vertical)
    assert xs == [560.0, 1920.0]


def test_drag_curb_endpoint_emits_geometry(qtbot, view):
    view.set_tool(Tool.EDIT_CURB)
    with qtbot.waitSignal(view.geometryEdited) as b:
        _drag(view, (560, 230), (600, 240))
    assert b.args[0] == [600.0, 240.0, 1920.0, 355.0, 560.0, 378.0, 1920.0, 446.0]


def test_lane_tool_only_moves_lane(qtbot, view):
    view.set_tool(Tool.EDIT_LANE)
    assert view._press(QPointF(560, 230)) is False   # curb handle not active
    with qtbot.waitSignal(view.geometryEdited) as b:
        _drag(view, (1920, 446), (1900, 460))
    assert b.args[0][6:] == [1900.0, 460.0]


def test_drag_is_clamped_to_frame(qtbot, view):
    view.set_tool(Tool.EDIT_CURB)
    with qtbot.waitSignal(view.geometryEdited) as b:
        _drag(view, (1920, 355), (2500, -40))
    assert b.args[0][2:4] == [1920.0, 0.0]


def test_pan_tool_ignores_handles(view):
    assert view._press(QPointF(560, 230)) is False


def test_draw_box_emits_normalized_rect(qtbot, view):
    view.set_tool(Tool.DRAW_BOX)
    with qtbot.waitSignal(view.boxDrawn) as b:
        _drag(view, (300, 200), (100, 100))
    assert b.args[0] == QRectF(100, 100, 200, 100)


def test_tiny_box_ignored(qtbot, view):
    view.set_tool(Tool.DRAW_BOX)
    with qtbot.assertNotEmitted(view.boxDrawn):
        _drag(view, (100, 100), (102, 101))


def test_delete_picks_smallest_containing_box(qtbot, view):
    big = CachedBox(0, 0, 1000, 1000, 1.0, "bus", 4)
    small = CachedBox(100, 100, 200, 200, 1.0, "person", 1)
    view.set_boxes([(big, False, 0.5), (small, True, 0.0)])
    view.set_tool(Tool.DELETE_BOX)
    with qtbot.waitSignal(view.boxDeleteRequested) as b:
        view._press(QPointF(150, 150))
    assert b.args == [1]


def test_toggles_hide_overlays(view):
    before = len(view._overlay.childItems())
    view.toggles.strip = False
    view.redraw()
    assert len(view._overlay.childItems()) == before - 3   # strip polygon + 2 domain lines


def test_tool_switch_cancels_curb_drag(qtbot, view):
    view.set_tool(Tool.EDIT_CURB)
    original = list(view._geometry)
    view._press(QPointF(560, 230))
    view._move(QPointF(600, 240))
    view.set_tool(Tool.PAN)
    with qtbot.assertNotEmitted(view.geometryEdited):
        view._release(QPointF(600, 240))
    assert view._geometry == original


def test_tool_switch_cancels_rubber_band(qtbot, view):
    view.set_tool(Tool.DRAW_BOX)
    view._press(QPointF(300, 200))
    view._move(QPointF(100, 100))
    rubber = view._rubber
    view.set_tool(Tool.PAN)
    assert rubber not in view.scene().items()
    with qtbot.assertNotEmitted(view.boxDrawn):
        view._release(QPointF(100, 100))
    assert view._rubber is None

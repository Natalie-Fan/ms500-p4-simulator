"""The frame with overlays, and the manual editing tools (layout A, centre)."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QImage, QPainter, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import (QGraphicsEllipseItem, QGraphicsItem, QGraphicsItemGroup,
                               QGraphicsLineItem, QGraphicsPixmapItem, QGraphicsPolygonItem,
                               QGraphicsRectItem, QGraphicsScene, QGraphicsSimpleTextItem,
                               QGraphicsView)

FRAME_W, FRAME_H = 1920, 1080
HANDLE_RADIUS = 12.0
GRAB_RADIUS = 24.0      # how close a press must be to an endpoint to grab it (scene px)
MIN_BOX = 4.0           # smaller rubber bands are treated as a click
SHIFT_SCALE = 4.0       # the registration shift is a few px; drawn 4x so it is visible

CLASS_COLORS = {"person": "#e6194b", "car": "#f58231", "truck": "#ffe119", "bus": "#bfef45",
                "bicycle": "#42d4f4", "motorcycle": "#911eb4", "other": "#a9a9a9"}
CURB_COLOR, LANE_COLOR = "#ff3333", "#33ccff"
STRIP_COLOR, ALIGN_COLOR, SHIFT_COLOR = "#ffee00", "#aaffaa", "#ff66ff"


class Tool(Enum):
    PAN = "Pan / zoom"
    EDIT_CURB = "Edit curb line"
    EDIT_LANE = "Edit lane line"
    DRAW_BOX = "Draw box"
    DELETE_BOX = "Delete box"


@dataclass
class OverlayToggles:
    curb: bool = True
    lane: bool = True
    strip: bool = True
    align: bool = False
    boxes: bool = True
    shift: bool = True


def _pen(color: str, width: float = 2.0, dashed: bool = False) -> QPen:
    pen = QPen(QColor(color))
    pen.setWidthF(width)
    pen.setCosmetic(True)   # constant on-screen width at any zoom
    if dashed:
        pen.setStyle(Qt.DashLine)
    return pen


def _clamp(p: QPointF) -> QPointF:
    return QPointF(min(max(p.x(), 0.0), float(FRAME_W)), min(max(p.y(), 0.0), float(FRAME_H)))


def _text(text: str, color: str, x: float, y: float) -> QGraphicsSimpleTextItem:
    item = QGraphicsSimpleTextItem(text)
    item.setBrush(QBrush(QColor(color)))
    item.setPos(x, y)
    item.setFlag(QGraphicsItem.ItemIgnoresTransformations)   # readable at any zoom
    return item


class FrameView(QGraphicsView):
    geometryEdited = Signal(list)
    boxDrawn = Signal(QRectF)
    boxDeleteRequested = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setScene(QGraphicsScene(0, 0, FRAME_W, FRAME_H, self))
        self.setRenderHint(QPainter.Antialiasing)
        self.setBackgroundBrush(QColor("#202020"))
        self._pixmap = QGraphicsPixmapItem()
        self.scene().addItem(self._pixmap)
        self._overlay: QGraphicsItemGroup | None = None
        self.tool = Tool.PAN
        self.toggles = OverlayToggles()
        self._geometry: list[float] | None = None
        self._strip = None
        self._align = None
        self._domain: tuple[float, float] | None = None
        self._boxes: list = []
        self._shift: tuple[int, int] | None = None
        self._drag_endpoint: int | None = None
        self._geometry_at_press: list[float] | None = None
        self._rubber_start: QPointF | None = None
        self._rubber: QGraphicsRectItem | None = None
        self._fit = True
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.redraw()

    # -------------------------------------------------------------- content
    def set_image(self, rgb: np.ndarray | None) -> None:
        if rgb is None:
            self._pixmap.setPixmap(QPixmap())
            return
        rgb = np.ascontiguousarray(rgb)
        h, w = rgb.shape[:2]
        image = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()
        self._pixmap.setPixmap(QPixmap.fromImage(image))

    def set_geometry_overlay(self, overlay: dict) -> None:
        self._geometry = [float(v) for v in overlay["geometry"]]
        self._strip = overlay["strip"]
        self._align = overlay["align"]
        self._domain = overlay.get("domain")
        self.redraw()

    def set_boxes(self, boxes: list) -> None:
        self._boxes = list(boxes)
        self.redraw()

    def set_shift(self, shift: tuple[int, int] | None) -> None:
        self._shift = shift
        self.redraw()

    def set_tool(self, tool: Tool) -> None:
        if self._drag_endpoint is not None:
            self._geometry = self._geometry_at_press
            self._drag_endpoint = None
            self._geometry_at_press = None
        if self._rubber_start is not None or self._rubber is not None:
            if self._rubber is not None:
                self.scene().removeItem(self._rubber)
            self._rubber = None
            self._rubber_start = None
        self.tool = tool
        self.setDragMode(QGraphicsView.ScrollHandDrag if tool is Tool.PAN else QGraphicsView.NoDrag)
        self.redraw()

    def redraw(self) -> None:
        if self._overlay is not None:
            self.scene().removeItem(self._overlay)
        group = QGraphicsItemGroup()
        self._overlay = group
        self.scene().addItem(group)
        g = self._geometry
        if g:
            if self.toggles.curb:
                group.addToGroup(self._line(g[0:4], CURB_COLOR))
            if self.toggles.lane:
                group.addToGroup(self._line(g[4:8], LANE_COLOR))
            for k in self._active_endpoints():
                group.addToGroup(self._handle(g[2 * k], g[2 * k + 1], CURB_COLOR if k < 2 else LANE_COLOR))
        if self.toggles.strip and self._strip is not None and len(self._strip[0]):
            xs, lo, hi = self._strip
            pts = [QPointF(float(x), float(y)) for x, y in zip(xs, lo)]
            pts += [QPointF(float(x), float(y)) for x, y in zip(xs[::-1], hi[::-1])]
            strip = QGraphicsPolygonItem(QPolygonF(pts))
            strip.setPen(_pen(STRIP_COLOR, 1.5, dashed=True))
            group.addToGroup(strip)
        if self.toggles.strip and self._domain is not None:
            for x in self._domain:
                line = QGraphicsLineItem(x, 0, x, FRAME_H)
                line.setPen(_pen(STRIP_COLOR, 1.0, dashed=True))
                group.addToGroup(line)
        if self.toggles.align and self._align:
            x0, y0, x1, y1 = self._align
            rect = QGraphicsRectItem(QRectF(x0, y0, x1 - x0, y1 - y0))
            rect.setPen(_pen(ALIGN_COLOR, 1.5, dashed=True))
            group.addToGroup(rect)
        if self.toggles.boxes:
            for box, edited, coverage in self._boxes:
                color = CLASS_COLORS.get(box.coco_class, CLASS_COLORS["other"])
                rect = QGraphicsRectItem(QRectF(box.x0, box.y0, box.x1 - box.x0, box.y1 - box.y0))
                rect.setPen(_pen(color, 2.0, dashed=edited))
                group.addToGroup(rect)
                group.addToGroup(_text(f"{box.coco_class} {coverage:.0%}", color, box.x0, box.y0))
        if self.toggles.shift and self._shift is not None and self._align:
            dx, dy = self._shift
            x0, y0, x1, y1 = self._align
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            arrow = QGraphicsLineItem(cx, cy, cx + dx * SHIFT_SCALE, cy + dy * SHIFT_SCALE)
            arrow.setPen(_pen(SHIFT_COLOR, 2.5))
            group.addToGroup(arrow)
            group.addToGroup(_text(f"shift dx {dx:+d} dy {dy:+d}", SHIFT_COLOR, cx + 8, cy + 8))

    def _line(self, v: list[float], color: str) -> QGraphicsLineItem:
        item = QGraphicsLineItem(v[0], v[1], v[2], v[3])
        item.setPen(_pen(color, 2.5))
        return item

    def _handle(self, x: float, y: float, color: str) -> QGraphicsEllipseItem:
        r = HANDLE_RADIUS
        item = QGraphicsEllipseItem(x - r, y - r, 2 * r, 2 * r)
        item.setPen(_pen("#ffffff", 1.5))
        item.setBrush(QBrush(QColor(color)))
        return item

    def _active_endpoints(self) -> tuple[int, ...]:
        return {Tool.EDIT_CURB: (0, 1), Tool.EDIT_LANE: (2, 3)}.get(self.tool, ())

    def _box_at(self, p: QPointF) -> int | None:
        hits = [((b.x1 - b.x0) * (b.y1 - b.y0), i) for i, (b, _, _) in enumerate(self._boxes)
                if b.x0 <= p.x() <= b.x1 and b.y0 <= p.y() <= b.y1]
        return min(hits)[1] if hits else None

    # ----------------------------------------------------------- interaction
    def _press(self, p: QPointF) -> bool:
        if self._geometry and self._active_endpoints():
            for k in self._active_endpoints():
                x, y = self._geometry[2 * k], self._geometry[2 * k + 1]
                if (p.x() - x) ** 2 + (p.y() - y) ** 2 <= GRAB_RADIUS ** 2:
                    self._geometry_at_press = list(self._geometry)
                    self._drag_endpoint = k
                    return True
            return False
        if self.tool is Tool.DRAW_BOX:
            self._rubber_start = _clamp(p)
            self._rubber = QGraphicsRectItem(QRectF(self._rubber_start, self._rubber_start))
            self._rubber.setPen(_pen("#ffffff", 1.5, dashed=True))
            self.scene().addItem(self._rubber)
            return True
        if self.tool is Tool.DELETE_BOX:
            idx = self._box_at(p)
            if idx is not None:
                self.boxDeleteRequested.emit(idx)
                return True
        return False

    def _move(self, p: QPointF) -> None:
        p = _clamp(p)
        if self._drag_endpoint is not None:
            k = self._drag_endpoint
            self._geometry[2 * k], self._geometry[2 * k + 1] = p.x(), p.y()
            self.redraw()
        elif self._rubber_start is not None and self._rubber is not None:
            self._rubber.setRect(QRectF(self._rubber_start, p).normalized())

    def _release(self, p: QPointF) -> None:
        if self._drag_endpoint is not None:
            self._move(p)
            self._drag_endpoint = None
            self.redraw()
            self.geometryEdited.emit(list(self._geometry))
        elif self._rubber_start is not None:
            rect = QRectF(self._rubber_start, _clamp(p)).normalized()
            if self._rubber is not None:
                self.scene().removeItem(self._rubber)
            self._rubber = None
            self._rubber_start = None
            if rect.width() >= MIN_BOX and rect.height() >= MIN_BOX:
                self.boxDrawn.emit(rect)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton and self._press(self.mapToScene(event.position().toPoint())):
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._drag_endpoint is not None or self._rubber_start is not None:
            self._move(self.mapToScene(event.position().toPoint()))
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if self._drag_endpoint is not None or self._rubber_start is not None:
            self._release(self.mapToScene(event.position().toPoint()))
            event.accept()
            return
        super().mouseReleaseEvent(event)

    # ------------------------------------------------------------- zooming
    def wheelEvent(self, event) -> None:
        factor = 1.15 ** (event.angleDelta().y() / 120)
        self.scale(factor, factor)
        self._fit = False

    def mouseDoubleClickEvent(self, event) -> None:
        self._fit = True
        self._fit_view()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if self._fit:
            self._fit_view()

    def showEvent(self, event) -> None:
        super().showEvent(event)
        if self._fit:
            self._fit_view()

    def _fit_view(self) -> None:
        self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)

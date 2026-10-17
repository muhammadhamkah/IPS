"""Top-down plan canvas: draw the volume outline, trusses, tripod areas and
no-go areas; drag vertices; show planned cameras and coverage overlays.

Scene units are metres with +Y up (the view flips Y). macOS: trackpad
two-finger scroll pans, pinch zooms, Cmd+scroll zooms; Option disables
snapping, Shift constrains to horizontal/vertical.
"""
from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import QEvent, QLineF, QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QGraphicsItem, QGraphicsScene, QGraphicsView

from idps.coverage.volume import offset_ring
from idps.show.pads import MIN_PAD_GAP_M, PAD_SIDE_M, SLOT_UNITS, SLOTS
from idps.ui import theme
from idps.ui.planner.document import frame_of

NEEDS_AREA = ("origin", "truss", "zone", "nogo", "pad")      # tools that need the performance area first
NEEDS_MOUNTS = ("pad",)                                       # ... and the trusses or tripod areas

TOOLS = {
    "select": "Select / move",
    "volume": "Draw performance area",
    "origin": "Place origin",
    "truss": "Draw truss",
    "zone": "Draw tripod area",
    "pad": "Place charging pad",
    "nogo": "Draw no-go area",
    "measure": "Measure distance",
}
HANDLE_PX = 7
HIT_PX = 9


def _poly(pts) -> QPolygonF:
    return QPolygonF([QPointF(float(x), float(y)) for x, y in pts])


def _seg_dist(p, a, b) -> float:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _contains(pts, p) -> bool:
    return len(pts) >= 3 and _poly(pts).containsPoint(QPointF(*p), Qt.FillRule.OddEvenFill)


class OverlayItem(QGraphicsItem):
    """Coverage or accuracy cells for one altitude level (painted, not one item per cell)."""

    def __init__(self):
        super().__init__()
        self.cells: list[tuple[QRectF, QColor]] = []
        self.marks: list[tuple[QPointF, QColor, bool]] = []    # edge points: pos, colour, failing
        self.clip = QPainterPath()
        self._bounds = QRectF()
        self.setZValue(-5)

    def set_cells(self, cells, marks, clip_poly: QPolygonF):
        self.prepareGeometryChange()
        self.cells, self.marks = cells, marks
        self.clip = QPainterPath()
        self.clip.addPolygon(clip_poly)
        self.clip.closeSubpath()
        self._bounds = clip_poly.boundingRect().adjusted(-1, -1, 1, 1)
        self.update()

    def boundingRect(self):
        return self._bounds

    def paint(self, p: QPainter, opt, widget=None):
        tok = theme.tokens()
        p.save()
        p.setClipPath(self.clip)
        p.setPen(QPen(tok["surface"], 0))
        for r, c in self.cells:
            p.setBrush(c)
            p.drawRect(r)
        p.restore()
        scale = p.worldTransform().m11()
        s = 3.5 / max(scale, 1e-6)
        for pos, c, bad in self.marks:
            p.setBrush(c)
            p.setPen(QPen(QColor(theme.STATUS[0]) if bad else tok["surface"], 0))
            p.drawRect(QRectF(pos.x() - s, pos.y() - s, 2 * s, 2 * s))


class PlanCanvas(QGraphicsView):
    selectionChanged = Signal(object)        # (kind, index) or None
    cursorMoved = Signal(float, float)
    message = Signal(str)
    toolChanged = Signal(str)

    def __init__(self, doc, parent=None):
        super().__init__(parent)
        self.doc = doc
        self.setScene(QGraphicsScene(-20000, -20000, 40000, 40000, self))
        self.setRenderHints(QPainter.RenderHint.Antialiasing | QPainter.RenderHint.TextAntialiasing)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.FullViewportUpdate)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setMouseTracking(True)
        self.viewport().setAttribute(Qt.WidgetAttribute.WA_AcceptTouchEvents, False)
        self.grabGesture(Qt.GestureType.PinchGesture)
        self.scale(20, -20)

        self.tool = "select"
        self.snap_step = 0.5
        self.snap_enabled = True
        self.pending: list[tuple[float, float]] = []
        self.cursor_pt: tuple[float, float] | None = None
        self.selection = None
        self.drag = None                 # (kind, index, vertex) while dragging a handle
        self.drag_before = None
        self.pan_from = None

        self.result = None               # (plan, evaluation) from the last planning run
        self.result_stale = False
        self.overlay_mode = "count"      # "none" | "count" | "accuracy"
        self.overlay_level = 0
        self.highlight_camera: int | None = None
        self.show_dimensions = True      # install measurements once a plan exists
        self.preview = None              # advisor Recommendation being previewed
        self.ruler: list = []            # Measure tool points
        self.overlay = OverlayItem()

        doc.changed.connect(self.rebuild)
        self.rebuild()

    # ------------------------------------------------------------------ view
    def px(self) -> float:
        """Metres per screen pixel."""
        return 1.0 / max(abs(self.transform().m11()), 1e-9)

    def zoom(self, factor: float):
        s = abs(self.transform().m11()) * factor
        if 0.05 < s < 5000:
            self.scale(factor, factor)
            self.viewport().update()

    def zoom_to_fit(self):
        pts = list(self.doc.data["volume"]["polygon"])
        for t in self.doc.data["trusses"]:
            pts += [t["start"], t["end"]]
        for z in self.doc.data["tripod_zones"]:
            pts += z.get("polygon", [])
            if "offset_m" in z and len(self.doc.data["volume"]["polygon"]) >= 3:
                pts += offset_ring(np.array(self.doc.data["volume"]["polygon"], float),
                                   float(z["offset_m"][1]), 1e9).tolist()
        if self.result is not None:
            pts += [c.position_m[:2] for c in self.result[0].cameras]
        if self.preview is not None:
            pts.append(self.preview.new_camera.position_m[:2])
        if self.doc.data.get("origin"):
            pts.append(self.doc.data["origin"]["xy"])
        pts += self._pads()
        if not pts:
            pts = [(-10, -10), (10, 10)]
        a = np.array(pts, float)
        r = QRectF(QPointF(*a.min(0)), QPointF(*a.max(0))).normalized()
        m = max(r.width(), r.height(), 4.0) * 0.08
        r = r.adjusted(-m, -m, m, m)
        self.fitInView(r, Qt.AspectRatioMode.KeepAspectRatio)
        t = self.transform()
        if t.m22() > 0:                               # fitInView resets the Y flip
            self.scale(1, -1)
        self.viewport().update()

    def event(self, e):
        # macOS trackpad pinch arrives as a native zoom gesture.
        if e.type() == QEvent.Type.NativeGesture:
            if e.gestureType() == Qt.NativeGestureType.ZoomNativeGesture:
                self.zoom(1.0 + e.value())
                return True
        return super().event(e)

    def viewportEvent(self, e):
        if e.type() == QEvent.Type.NativeGesture:
            return self.event(e)
        return super().viewportEvent(e)

    def wheelEvent(self, e):
        pd = e.pixelDelta()
        cmd = e.modifiers() & Qt.KeyboardModifier.ControlModifier   # Cmd on macOS
        if not pd.isNull() and not cmd:
            # trackpad two-finger scroll: pan
            self._pan_pixels(pd.x(), pd.y())
        else:
            ang = e.angleDelta().y() or pd.y()
            self.zoom(1.0015 ** ang)
        e.accept()

    def _pan_pixels(self, dx: float, dy: float):
        c = self.mapToScene(self.viewport().rect().center())
        s = self.px()
        self.centerOn(c.x() - dx * s, c.y() + dy * s)
        self.viewport().update()

    # ------------------------------------------------------------- painting
    def _grid_step(self) -> tuple[float, float]:
        target = 60 * self.px()                     # aim for ~60 px between minor lines
        for step in (0.1, 0.25, 0.5, 1, 2, 5, 10, 25, 50, 100):
            if step >= target:
                return step, step * (5 if step in (1, 10, 100, 0.1) else 4 if step in (0.25, 25) else 2)
        return 100.0, 500.0

    def drawBackground(self, p: QPainter, rect: QRectF):
        tok = theme.tokens()
        p.fillRect(rect, tok["surface"])
        minor, major = self._grid_step()
        for step, colour in ((minor, tok["grid_minor"]), (major, tok["grid_major"])):
            p.setPen(QPen(colour, 0))
            x = math.floor(rect.left() / step) * step
            while x <= rect.right():
                p.drawLine(QLineF(x, rect.top(), x, rect.bottom()))
                x += step
            y = math.floor(rect.top() / step) * step
            while y <= rect.bottom():
                p.drawLine(QLineF(rect.left(), y, rect.right(), y))
                y += step

    def _text(self, p: QPainter, pos: QPointF, text: str, colour: QColor, dx=6, dy=-6, bold=False,
              size=11):
        """Screen-space text at a scene position (unaffected by zoom or the Y flip)."""
        v = self.mapFromScene(pos)
        p.save()
        p.resetTransform()
        f = QFont(self.font())
        f.setPointSizeF(size)
        f.setBold(bold)
        p.setFont(f)
        p.setPen(colour)
        p.drawText(QPointF(v.x() + dx, v.y() + dy), text)
        p.restore()

    def _handle(self, p: QPainter, pt, colour: QColor, selected=False):
        s = (HANDLE_PX if not selected else HANDLE_PX + 2) * self.px() / 2
        tok = theme.tokens()
        p.setPen(QPen(colour, 1.5 * self.px()))
        p.setBrush(colour if selected else tok["handle_fill"])
        p.drawRect(QRectF(pt[0] - s, pt[1] - s, 2 * s, 2 * s))

    def drawForeground(self, p: QPainter, rect: QRectF):
        tok = theme.tokens()
        d = self.doc.data
        px = self.px()
        sel = self.selection

        # no-go areas
        for i, r in enumerate(d["no_go_zones"]):
            c = QColor(tok["nogo"])
            fill = QColor(c)
            fill.setAlphaF(0.16)
            p.setPen(QPen(c, (2.5 if sel == ("nogo", i) else 1.2) * px, Qt.PenStyle.DashLine))
            p.setBrush(fill)
            p.drawPolygon(_poly(r))
            if r:
                self._text(p, QPointF(*np.mean(r, 0)), "No-go", c, dx=-16, dy=4, size=10)
            if sel == ("nogo", i):
                for v in r:
                    self._handle(p, v, c)

        # tripod areas
        vol = d["volume"]["polygon"]
        for i, z in enumerate(d["tripod_zones"]):
            c = QColor(tok["zone"])
            fill = QColor(c)
            fill.setAlphaF(0.10)
            w = (2.5 if sel == ("zone", i) else 1.2) * px
            if "polygon" in z:
                p.setPen(QPen(c, w, Qt.PenStyle.DashLine))
                p.setBrush(fill)
                p.drawPolygon(_poly(z["polygon"]))
                if z["polygon"]:
                    self._text(p, QPointF(*np.mean(z["polygon"], 0)), f"Tripods: {z['id']}", c,
                               dx=-24, dy=4, size=10)
                if sel == ("zone", i):
                    for v in z["polygon"]:
                        self._handle(p, v, c)
            elif "offset_m" in z and len(vol) >= 3:
                path = QPainterPath()
                rings = []
                for off in z["offset_m"]:
                    ring = offset_ring(np.array(vol, float), float(off), 1e9)
                    ring = ring[1::2] if len(ring) == 2 * len(vol) else ring
                    rings.append(ring)
                    path.addPolygon(_poly(ring))
                    path.closeSubpath()
                p.setPen(QPen(c, w, Qt.PenStyle.DashLine))
                p.setBrush(fill)
                path.setFillRule(Qt.FillRule.OddEvenFill)
                p.drawPath(path)
                self._text(p, QPointF(*rings[-1][0]), f"Tripod strip {z['id']} "
                           f"({z['offset_m'][0]:g}-{z['offset_m'][1]:g} m)", c, dx=4, dy=-4, size=10)

        # volume outline
        if vol:
            c = QColor(tok["volume"])
            fill = QColor(c)
            fill.setAlphaF(0.06 if self.result is None or self.overlay_mode == "none" else 0.0)
            p.setPen(QPen(c, (2.6 if sel == ("volume", None) else 1.8) * px))
            p.setBrush(fill if len(vol) >= 3 else Qt.BrushStyle.NoBrush)
            if len(vol) >= 3:
                p.drawPolygon(_poly(vol))
            centre = np.mean(vol, 0)
            for k, v in enumerate(vol):
                self._handle(p, v, c, selected=sel == ("vertex", k))
                off = np.array(v) - centre
                n = np.linalg.norm(off)
                off = off / n * 14 if n > 0 else np.array([10.0, 10.0])
                self._text(p, QPointF(*v), f"V{k}", tok["text"], dx=off[0] - 6, dy=-off[1] + 4, bold=True,
                           size=10)
            if len(vol) >= 3:
                for k in range(len(vol)):
                    a, b = np.array(vol[k], float), np.array(vol[(k + 1) % len(vol)], float)
                    mid = (a + b) / 2
                    self._text(p, QPointF(*mid), f"E{k}  {np.linalg.norm(b - a):.2f} m", tok["text2"],
                               dx=-24, dy=-6, size=9)

        # trusses
        for i, t in enumerate(d["trusses"]):
            a, b = np.array(t["start"], float), np.array(t["end"], float)
            L = float(np.linalg.norm(b - a))
            selected = sel == ("truss", i)
            p.setPen(QPen(tok["truss_gap"], 3 * px, Qt.PenStyle.DotLine))
            p.drawLine(QLineF(*a, *b))
            segs = [(0, L)] if t.get("usable") in (None, "all") else t["usable"]
            p.setPen(QPen(tok["select"] if selected else tok["truss"], (5 if selected else 4) * px,
                          Qt.PenStyle.SolidLine, Qt.PenCapStyle.FlatCap))
            for s0, s1 in segs:
                if L > 0:
                    pa, pb = a + (b - a) * (s0 / L), a + (b - a) * (min(s1, L) / L)
                    p.drawLine(QLineF(*pa, *pb))
            self._text(p, QPointF(*a), f"{t['id']}  {t['height_m']:g} m", tok["text2"], dx=-4, dy=-8,
                       size=10)
            if selected:
                self._handle(p, a, tok["select"])
                self._handle(p, b, tok["select"])

        self._draw_pads(p, tok)

        # planned cameras
        if self.result is not None:
            plan, _ = self.result
            alpha = 0.45 if self.result_stale else 1.0
            for k, cam in enumerate(plan.cameras):
                col = QColor(tok["cam_truss"] if cam.mount_type == "truss" else tok["cam_tripod"])
                col.setAlphaF(alpha)
                x, y = cam.position_m[0], cam.position_m[1]
                az, el = math.radians(cam.azimuth_deg), math.radians(cam.elevation_deg)
                L = 26 * px * max(0.25, math.cos(el))
                hl = self.highlight_camera == k or sel == ("camera", k)
                p.setPen(QPen(col, (2.4 if hl else 1.6) * px))
                if math.cos(el) > 0.05:
                    tip = QPointF(x + L * math.cos(az), y + L * math.sin(az))
                    p.drawLine(QPointF(x, y), tip)
                r = (6 if hl else 4.2) * px
                p.setBrush(col)
                p.setPen(QPen(tok["surface"], 1.2 * px))
                p.drawEllipse(QPointF(x, y), r, r)
                if hl or len(plan.cameras) <= 40:
                    self._text(p, QPointF(x, y), cam.id, tok["text2"], dx=6, dy=12, size=9, bold=hl)
            # several cameras sharing one mount spot: badge it, otherwise their aim lines look odd
            from idps.ui.planner.view3d import mount_counts
            for (x, y, _), n in mount_counts(plan.cameras).items():
                if n > 1:
                    self._text(p, QPointF(x, y), f"×{n}", tok["text"], dx=-22, dy=-6, size=10, bold=True)

        if self.result is not None and self.show_dimensions:
            self._draw_dimensions(p, tok)
        self._draw_ruler(p, tok)
        if self.preview is not None and self.result is not None:
            self._draw_preview(p, tok)

        if self.tool == "pad" and self.cursor_pt is not None:
            self._draw_pad(p, tok, self.cursor_pt, len(self._pads()), ghost=True)

        # drawing preview
        if self.tool not in ("select", "measure", "pad") and (self.pending or self.cursor_pt):
            c = QColor(tok["select"])
            p.setPen(QPen(c, 1.6 * px, Qt.PenStyle.DashLine))
            pts = list(self.pending) + ([self.cursor_pt] if self.cursor_pt else [])
            for a, b in zip(pts[:-1], pts[1:]):
                p.drawLine(QPointF(*a), QPointF(*b))
            if self.tool != "truss" and len(self.pending) >= 2 and self.cursor_pt:
                p.setPen(QPen(c, 1.0 * px, Qt.PenStyle.DotLine))
                p.drawLine(QPointF(*self.cursor_pt), QPointF(*self.pending[0]))
            for v in self.pending:
                self._handle(p, v, c)
            if self.cursor_pt:
                self._handle(p, self.cursor_pt, c, selected=True)
                if self.pending:
                    a, b = np.array(self.pending[-1]), np.array(self.cursor_pt)
                    self._text(p, QPointF(*b), f"{np.linalg.norm(b - a):.2f} m", c, dx=10, dy=18, size=10)

        self._draw_origin(p, tok)

        if not vol and self.tool in ("select", "volume") and not d["trusses"] and not self.pending:
            p.save()
            p.resetTransform()
            f = QFont(self.font())
            f.setPointSizeF(14)
            p.setFont(f)
            p.setPen(tok["text2"])
            p.drawText(self.viewport().rect(), Qt.AlignmentFlag.AlignCenter,
                       "Step 1: draw the performance area (P).\n"
                       "Click its floor outline corners; Return or click V0 to close.\n"
                       "Step 2: place the origin (the calibration square) and point its N arrow North.\n"
                       "Step 3: draw the trusses (T) or tripod areas (A).\n"
                       "Step 4: place the charging pads (C).\n"
                       "Step 5: Plan Cameras (⌘R).")
            p.restore()

    # ------------------------------------------------------------- charging pads
    def _pads(self) -> list:
        return (self.doc.data.get("pads") or {}).get("centres") or []

    def _pad_corners(self, centre, side=PAD_SIDE_M):
        """A pad's square, its sides along North and East."""
        f = frame_of(self.doc.data)
        c, h = np.asarray(centre, float), side / 2
        return [c + f.n_hat * a * h + f.e_hat * b * h for a, b in ((1, -1), (1, 1), (-1, 1), (-1, -1))]

    def _slot_xy(self, centre, slot):
        f = frame_of(self.doc.data)
        un, ue = SLOT_UNITS[slot]
        h = float((self.doc.data.get("pads") or {}).get("slot_spacing_m", 0.2)) / 2
        return np.asarray(centre, float) + f.n_hat * un * h + f.e_hat * ue * h

    def _draw_pads(self, p: QPainter, tok):
        close = {k for i, j, _ in _pads_too_close(self._pads()) for k in (i, j)}
        for i, c in enumerate(self._pads()):
            self._draw_pad(p, tok, c, i, selected=self.selection == ("pad", i), warn=i in close)

    def _draw_pad(self, p: QPainter, tok, centre, i, selected=False, ghost=False, warn=False):
        """A charging pad: its square, its 4 drone slots (drone ID = 4 x (pad - 1) + slot; the
        slot is the take-off batch) and its name."""
        px = self.px()
        col = QColor(tok["select"] if selected else tok["nogo"] if warn else tok["pad"])
        if ghost:
            col.setAlphaF(0.55)
        fill = QColor(col)
        fill.setAlphaF(0.14)
        p.setPen(QPen(col, (2.4 if selected else 1.5) * px, Qt.PenStyle.DashLine if ghost else Qt.PenStyle.SolidLine))
        p.setBrush(fill)
        p.drawPolygon(_poly(self._pad_corners(centre)))
        r = max(0.035, 2.5 * px)
        show_ids = PAD_SIDE_M / px > 70
        for slot in range(1, SLOTS + 1):
            q = self._slot_xy(centre, slot)
            p.setPen(QPen(tok["surface"], 0.8 * px))
            p.setBrush(col)
            p.drawEllipse(QPointF(*q), r, r)
            if show_ids and not ghost:
                self._text(p, QPointF(*q), str(SLOTS * i + slot), tok["text2"], dx=4, dy=-3, size=8)
        corner = self._pad_corners(centre)[0]
        self._text(p, QPointF(*corner), f"Pad {i + 1}", col, dx=2, dy=-4, size=10, bold=selected)
        if selected:
            self._handle(p, centre, col, selected=True)

    # ------------------------------------------------------------- origin
    ORIGIN_N_PX = 64                     # North arrow length on screen
    ORIGIN_E_PX = 40

    def _origin_geometry(self, origin=None):
        """(origin xy, N tip, E tip) in scene coordinates, or None."""
        o = origin if origin is not None else self.doc.data.get("origin")
        if o is None:
            return None
        from idps.core.frames import Frame
        f = Frame.from_dict(o)
        c = np.array(f.origin_xy, float)
        px = self.px()
        return c, c + f.n_hat * self.ORIGIN_N_PX * px, c + f.e_hat * self.ORIGIN_E_PX * px

    def _draw_origin(self, p: QPainter, tok):
        origin = self.doc.data.get("origin")
        ghost = False
        if origin is None and self.tool == "origin" and self.cursor_pt is not None:
            origin, ghost = {"xy": list(self.cursor_pt), "north_deg": 90.0}, True
        g = self._origin_geometry(origin)
        if g is None:
            return
        c, n_tip, e_tip = g
        px = self.px()
        sel = self.selection == ("origin", None)
        col = QColor(tok["select"] if sel or ghost else tok["text"])
        ncol = QColor(theme.STATUS[0])                         # North in red, like a compass needle
        ecol = QColor(tok["volume"])
        if ghost:
            for q in (col, ncol, ecol):
                q.setAlphaF(0.6)
        for tip, colour, w in ((n_tip, ncol, 2.6), (e_tip, ecol, 2.0)):
            p.setPen(QPen(colour, w * px))
            p.drawLine(QPointF(*c), QPointF(*tip))
            u = (tip - c) / max(np.linalg.norm(tip - c), 1e-12)
            nrm = np.array([-u[1], u[0]])
            head = 9 * px
            p.setBrush(colour)
            p.drawPolygon(_poly([tip, tip - u * head + nrm * head * 0.45, tip - u * head - nrm * head * 0.45]))
        self._text(p, QPointF(*n_tip), "N", ncol, dx=4, dy=-4, bold=True, size=12)
        self._text(p, QPointF(*e_tip), "E", ecol, dx=4, dy=-4, bold=True, size=11)
        p.setPen(QPen(col, 2 * px))
        p.setBrush(tok["surface"])
        p.drawEllipse(QPointF(*c), 6 * px, 6 * px)
        p.setBrush(col)
        p.drawEllipse(QPointF(*c), 2.2 * px, 2.2 * px)
        self._text(p, QPointF(*c), "Origin (D down)", col, dx=-30, dy=22, size=9, bold=sel)
        if sel:                                               # rotate handle at the N tip
            self._handle(p, n_tip, ncol, selected=True)

    # ------------------------------------------------------------- dimensions
    def _dim(self, p: QPainter, a, b, offset_px: float, label: str, colour: QColor):
        """Architectural dimension: line between a and b shifted offset_px to the
        left of a->b, extension lines back to the points, ticks, and a label."""
        px = self.px()
        a, b = np.asarray(a, float), np.asarray(b, float)
        d = b - a
        L = float(np.linalg.norm(d))
        if L < 1e-6:
            return
        u = d / L
        nrm = np.array([-u[1], u[0]]) * offset_px * px
        a2, b2 = a + nrm, b + nrm
        p.setPen(QPen(colour, 1.0 * px))
        p.drawLine(QPointF(*a2), QPointF(*b2))
        ext = nrm / max(np.linalg.norm(nrm), 1e-12) * 4 * px
        for q, q2 in ((a, a2), (b, b2)):
            p.drawLine(QPointF(*q), QPointF(*(q2 + ext)))
            t = (u + nrm / max(np.linalg.norm(nrm), 1e-12)) * 4 * px          # 45° tick
            p.setPen(QPen(colour, 1.6 * px))
            p.drawLine(QPointF(*(q2 - t)), QPointF(*(q2 + t)))
            p.setPen(QPen(colour, 1.0 * px))
        if L / px > 34:                                    # label only where it fits
            mid = (a2 + b2) / 2 + nrm / max(np.linalg.norm(nrm), 1e-12) * 7 * px
            v = self.mapFromScene(QPointF(*mid))
            p.save()
            p.resetTransform()
            f = QFont(self.font())
            f.setPointSizeF(9)
            p.setFont(f)
            w = p.fontMetrics().horizontalAdvance(label) + 6
            r = QRectF(v.x() - w / 2, v.y() - 8, w, 16)
            bg = QColor(theme.tokens()["surface"])
            bg.setAlphaF(0.85)
            p.fillRect(r, bg)
            p.setPen(colour)
            p.drawText(r, Qt.AlignmentFlag.AlignCenter, label)
            p.restore()

    def _draw_dimensions(self, p: QPainter, tok):
        """Chained install measurements: truss start -> each camera mount, and
        volume vertex -> each tripod along its nearest edge (+ offset out)."""
        from idps.plan.measure import chains
        plan, _ = self.result
        col = QColor(tok["text2"])
        for ch in chains(self.doc.data["trusses"], self.doc.data["volume"]["polygon"], plan.cameras):
            o, u = np.array(ch.origin), np.array(ch.direction)
            prev = 0.0
            side = 26 if ch.kind == "truss" else -22        # edges: inside the volume, away from tripods
            for s, ids, off in ch.stations:
                self._dim(p, o + u * prev, o + u * s, side, f"{s - prev:.2f}", col)
                if ch.kind == "edge" and off > 0:
                    foot = o + u * s
                    out = np.array([u[1], -u[0]])
                    cam = next(c for c in plan.cameras if c.id == ids[0])
                    if np.dot(np.array(cam.position_m[:2]) - foot, out) < 0:
                        out = -out
                    if s < 0 or s > ch.length:
                        p.setPen(QPen(col, 1.0 * self.px(), Qt.PenStyle.DotLine))     # edge line extended
                        p.drawLine(QPointF(*o), QPointF(*foot))
                    self._dim(p, foot, foot + out * off, 0, f"{off:.2f}", col)
                prev = s
            self._text(p, QPointF(*o), "0", col, dx=-10, dy=-10, size=8)

    def _draw_preview(self, p: QPainter, tok):
        """Advisor suggestion: dashed ghost camera at the new spot and aim, with an
        arrow from the old position for a move."""
        r = self.preview
        px = self.px()
        c = QColor(tok["select"])
        new = r.new_camera
        x, y = new.position_m[0], new.position_m[1]
        if r.camera_id:
            old = next((k for k in self.result[0].cameras if k.id == r.camera_id), None)
            if old is not None and r.kind == "move":
                p.setPen(QPen(c, 2 * px, Qt.PenStyle.DashLine))
                p.drawLine(QPointF(*old.position_m[:2]), QPointF(x, y))
                self._dim(p, old.position_m[:2], (x, y), 18,
                          f"{np.linalg.norm(np.subtract(new.position_m[:2], old.position_m[:2])):.2f} m", c)
        az, el = math.radians(new.azimuth_deg), math.radians(new.elevation_deg)
        L = 40 * px * max(0.25, math.cos(el))
        p.setPen(QPen(c, 2.4 * px))
        if math.cos(el) > 0.05:
            p.drawLine(QPointF(x, y), QPointF(x + L * math.cos(az), y + L * math.sin(az)))
        p.setPen(QPen(c, 2.2 * px, Qt.PenStyle.DashLine))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(x, y), 11 * px, 11 * px)
        self._text(p, QPointF(x, y), ("+ " if r.kind == "add" else "") + f"{new.id} (suggested)", c,
                   dx=14, dy=-12, size=10, bold=True)

    def _draw_ruler(self, p: QPainter, tok):
        pts = list(self.ruler)
        if self.tool == "measure" and len(pts) == 1 and self.cursor_pt:
            pts.append(self.cursor_pt)
        if len(pts) < 2:
            if pts:
                self._handle(p, pts[0], QColor(tok["select"]), selected=True)
            return
        a, b = np.array(pts[0]), np.array(pts[1])
        c = QColor(tok["select"])
        p.setPen(QPen(c, 2 * self.px()))
        p.drawLine(QPointF(*a), QPointF(*b))
        for q in (a, b):
            self._handle(p, q, c, selected=True)
        self._dim(p, a, b, 0, f"{np.linalg.norm(b - a):.3f} m", c)

    # ------------------------------------------------------------- results
    def set_result(self, plan, ev):
        self.result = (plan, ev)
        self.result_stale = False
        self.highlight_camera = None
        self.overlay_level = min(self.overlay_level, len(ev.grid.zs) - 1)
        if self.overlay.scene() is None:
            self.scene().addItem(self.overlay)
        self.update_overlay()

    def clear_result(self):
        self.result = None
        if self.overlay.scene() is not None:
            self.scene().removeItem(self.overlay)
        self.viewport().update()

    def update_overlay(self):
        if self.result is None:
            return
        plan, ev = self.result
        self.overlay.setVisible(self.overlay_mode != "none")
        g = ev.grid
        lvl = g.level_of()
        k = self.overlay_level
        xs, ys = g.xs, g.ys
        dx = (xs[1] - xs[0]) if len(xs) > 1 else 1.0
        dy = (ys[1] - ys[0]) if len(ys) > 1 else 1.0
        limit = None
        req = plan.summary.get("requirements", {})
        limit = float(req.get("p95_mm", 10.0))
        min_cams = int(req.get("min_cameras", 3))

        def colour(n, p95):
            if self.overlay_mode == "count":
                c = QColor(theme.STATUS[min(int(n), 3)])
            else:
                c = theme.ramp_color(p95 if np.isfinite(p95) else 2 * limit, 2 * limit)
            c.setAlphaF(0.55 if self.result_stale else 0.85)
            return c

        cells = []
        on = np.where((lvl == k))[0]
        grid_idx = on[on < g.n_grid]
        ii, jj = np.nonzero(g.inside)
        for (i, j), v in zip(zip(ii, jj), grid_idx):
            r = QRectF(xs[i] - dx / 2, ys[j] - dy / 2, dx, dy)
            cells.append((r, colour(ev.n_cams[v], ev.p95_mm[v])))
        marks = []
        for v in on[on >= g.n_grid]:
            bad = not (ev.n_cams[v] >= min_cams and ev.p95_mm[v] <= limit)
            marks.append((QPointF(*ev.points[v, :2]), colour(ev.n_cams[v], ev.p95_mm[v]), bad))
        vol = self.doc.data["volume"]["polygon"]
        self.overlay.set_cells(cells, marks, _poly(vol) if len(vol) >= 3 else _poly(g.ring_xy))
        self.viewport().update()

    # --------------------------------------------------------------- editing
    def set_tool(self, tool: str):
        if tool in NEEDS_AREA and len(self.doc.data["volume"]["polygon"]) < 3:
            self.message.emit("Draw the performance area first: click its floor outline corners, "
                              "then Return.")
            tool = "volume"
        elif tool in NEEDS_MOUNTS and not self.doc.data["trusses"] and not self.doc.data["tripod_zones"]:
            self.message.emit("Place the trusses (T) or tripod areas (A) first: the charging pads come after the "
                              "camera mounts, then Plan Cameras.")
            tool = "truss"
        self.tool = tool
        self.pending = []
        self.toolChanged.emit(tool)
        self.setCursor(Qt.CursorShape.ArrowCursor if tool == "select" else Qt.CursorShape.CrossCursor)
        hints = {
            "select": "Click to select; drag vertices to move them; double-click an outline edge to add a vertex.",
            "volume": "Click the performance area's floor corners. Return, double-click or click V0 to finish. "
                      "Esc cancels.",
            "origin": "Click where the calibration square's corner (the origin) will be. Then drag the red N "
                      "arrow to point North.",
            "truss": "Click the truss start, then its end.",
            "zone": "Click the tripod area corners. Return or double-click to finish.",
            "nogo": "Click the no-go area corners. Return or double-click to finish.",
            "pad": f"Click where each charging pad stands (4 drones, 2 × 2; pads at least {MIN_PAD_GAP_M:g} m "
                   "apart). Pad 1 holds drones 1–4, pad 2 drones 5–8, … Esc when done.",
            "measure": "Click two points to measure (snaps to vertices, truss ends and cameras). Esc clears.",
        }
        if tool != "measure":
            self.ruler = []
        self.message.emit(hints[tool] + "  (Shift: horizontal/vertical, Option: no snapping)")
        self.viewport().update()

    def set_selection(self, sel):
        self.selection = sel
        self.highlight_camera = sel[1] if sel and sel[0] == "camera" else None
        self.selectionChanged.emit(sel)
        self.viewport().update()

    def rebuild(self):
        if self.result is not None:
            self.result_stale = True
            self.update_overlay()
        # drop a selection that no longer exists
        s = self.selection
        d = self.doc.data
        if s and ((s[0] == "vertex" and s[1] >= len(d["volume"]["polygon"]))
                  or (s[0] == "truss" and s[1] >= len(d["trusses"]))
                  or (s[0] == "zone" and s[1] >= len(d["tripod_zones"]))
                  or (s[0] == "nogo" and s[1] >= len(d["no_go_zones"]))
                  or (s[0] == "pad" and s[1] >= len(self._pads()))):
            self.set_selection(None)
        self.viewport().update()

    def _snap(self, x: float, y: float, mods) -> tuple[float, float]:
        if self.tool != "select" and self.pending and mods & Qt.KeyboardModifier.ShiftModifier:
            lx, ly = self.pending[-1]
            if abs(x - lx) >= abs(y - ly):
                y = ly
            else:
                x = lx
        if not self.snap_enabled or mods & Qt.KeyboardModifier.AltModifier:
            return round(x, 3), round(y, 3)
        # snap to existing points first
        tol = HIT_PX * self.px()
        for pt in self._all_points():
            if math.hypot(pt[0] - x, pt[1] - y) < tol:
                return float(pt[0]), float(pt[1])
        s = self.snap_step
        return round(round(x / s) * s, 3), round(round(y / s) * s, 3)

    def _all_points(self):
        d = self.doc.data
        if d.get("origin"):
            yield d["origin"]["xy"]
        yield from d["volume"]["polygon"]
        for t in d["trusses"]:
            yield t["start"]
            yield t["end"]
        for z in d["tripod_zones"]:
            yield from z.get("polygon", [])
        for r in d["no_go_zones"]:
            yield from r
        yield from self._pads()
        if self.result is not None:             # measure from / to planned cameras
            for c in self.result[0].cameras:
                yield c.position_m[:2]

    def _hit(self, x: float, y: float):
        """What is under the cursor: (kind, index, vertex-or-None)."""
        d = self.doc.data
        tol = HIT_PX * self.px()
        sel = self.selection
        g = self._origin_geometry()
        if g is not None:                                  # origin: N tip turns it, the centre moves it
            c, n_tip, _ = g
            if math.hypot(n_tip[0] - x, n_tip[1] - y) < tol * 1.4:
                return ("origin", None, "rotate")
            if math.hypot(c[0] - x, c[1] - y) < tol * 1.2:
                return ("origin", None, "move")
        # handles of the selected item first
        if sel and sel[0] == "truss":
            t = d["trusses"][sel[1]]
            for k, key in enumerate(("start", "end")):
                if math.hypot(t[key][0] - x, t[key][1] - y) < tol:
                    return ("truss", sel[1], k)
        for kind, idx, pts in self._polygons():
            if kind == "volume" or sel == (kind, idx):
                for k, v in enumerate(pts):
                    if math.hypot(v[0] - x, v[1] - y) < tol:
                        return (kind, idx, k)
        f = frame_of(d)
        for i, c in enumerate(self._pads()):              # a pad: grab it anywhere to move it
            dn, de = np.abs(f.ne((x, y)) - f.ne(c))
            if max(dn, de) <= PAD_SIDE_M / 2 + tol:
                return ("pad", i, 0)
        if self.result is not None:
            for k, c in enumerate(self.result[0].cameras):
                if math.hypot(c.position_m[0] - x, c.position_m[1] - y) < tol:
                    return ("camera", k, None)
        for i, t in enumerate(d["trusses"]):
            if _seg_dist((x, y), t["start"], t["end"]) < tol:
                return ("truss", i, None)
        for i, r in enumerate(d["no_go_zones"]):
            if _contains(r, (x, y)):
                return ("nogo", i, None)
        for i, z in enumerate(d["tripod_zones"]):
            if "polygon" in z and _contains(z["polygon"], (x, y)):
                return ("zone", i, None)
        vol = d["volume"]["polygon"]
        if len(vol) >= 3 and _contains(vol, (x, y)):
            return ("volume", None, None)
        for i, z in enumerate(d["tripod_zones"]):
            if "offset_m" in z and len(vol) >= 3 and not _contains(vol, (x, y)):
                from idps.coverage.volume import distance_to_polygon
                dist = distance_to_polygon(np.array([[x, y]]), np.array(vol, float))[0][0]
                if z["offset_m"][0] - tol <= dist <= z["offset_m"][1] + tol:
                    return ("zone", i, None)
        return None

    def _polygons(self):
        d = self.doc.data
        yield "volume", None, d["volume"]["polygon"]
        for i, z in enumerate(d["tripod_zones"]):
            if "polygon" in z:
                yield "zone", i, z["polygon"]
        for i, r in enumerate(d["no_go_zones"]):
            yield "nogo", i, r

    @staticmethod
    def _points_of(data, kind, idx):
        if kind == "volume":
            return data["volume"]["polygon"]
        if kind == "zone":
            return data["tripod_zones"][idx]["polygon"]
        if kind == "nogo":
            return data["no_go_zones"][idx]
        raise KeyError(kind)

    def mousePressEvent(self, e):
        sp = self.mapToScene(e.position().toPoint())
        x, y = sp.x(), sp.y()
        if e.button() == Qt.MouseButton.MiddleButton or (
                e.button() == Qt.MouseButton.LeftButton and self.tool == "select" and self._hit(x, y) is None):
            self.pan_from = e.position()
            if e.button() == Qt.MouseButton.LeftButton:
                self.set_selection(None)
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            return
        if e.button() != Qt.MouseButton.LeftButton:
            return super().mousePressEvent(e)
        if self.tool == "select":
            hit = self._hit(x, y)
            kind, idx, vtx = hit
            if kind == "origin":
                self.drag = ("origin", None, vtx)
                self.drag_before = self.doc.snapshot()
                self.set_selection(("origin", None))
                return
            if vtx is not None:
                self.drag = (kind, idx, vtx)
                self.drag_before = self.doc.snapshot()
                self.set_selection(("vertex", vtx) if kind == "volume" else (kind, idx))
            else:
                self.set_selection((kind, idx))
            return
        pt = self._snap(x, y, e.modifiers())
        if self.tool == "origin":
            north = float((self.doc.data.get("origin") or {}).get("north_deg", 90.0))
            self.doc.edit("Place origin", lambda d: d.__setitem__("origin", {"xy": list(pt), "north_deg": north}))
            self.cursor_pt = None
            self.set_tool("select")
            self.set_selection(("origin", None))
            self.message.emit("Origin placed. Drag the red N arrow to point North (snaps to 15°; hold Option "
                              "for any angle). Drag the centre to move it. All coordinates are North-East-Down "
                              "from here.")
            return
        if self.tool == "pad":
            self.place_pad(pt)
            return
        if self.tool == "measure":
            # two clicks measure; a third click starts a new measurement
            self.ruler = [pt] if len(self.ruler) != 1 else self.ruler + [pt]
            if len(self.ruler) == 2:
                a, b = np.array(self.ruler[0]), np.array(self.ruler[1])
                dn, de = frame_of(self.doc.data).ne(b) - frame_of(self.doc.data).ne(a)
                self.message.emit(f"Distance {np.linalg.norm(b - a):.3f} m  (ΔN {dn:+.3f} m, ΔE {de:+.3f} m)")
            self.viewport().update()
            return
        if self.tool == "truss":
            self.pending.append(pt)
            if len(self.pending) == 2:
                self._finish()
        else:
            if len(self.pending) >= 3 and math.hypot(pt[0] - self.pending[0][0],
                                                     pt[1] - self.pending[0][1]) < HIT_PX * self.px():
                self._finish()
            elif not self.pending or pt != self.pending[-1]:
                self.pending.append(pt)
        self.viewport().update()

    def mouseMoveEvent(self, e):
        sp = self.mapToScene(e.position().toPoint())
        self.cursorMoved.emit(sp.x(), sp.y())
        if self.pan_from is not None:
            d = e.position() - self.pan_from
            self.pan_from = e.position()
            self._pan_pixels(d.x(), d.y())
            return
        if self.drag is not None:
            kind, idx, vtx = self.drag
            if kind == "origin" and vtx == "rotate":
                o = self.doc.data["origin"]["xy"]
                ang = math.degrees(math.atan2(sp.y() - o[1], sp.x() - o[0]))
                if not e.modifiers() & Qt.KeyboardModifier.AltModifier:
                    ang = round(ang / 15.0) * 15.0
                self.doc.set_live(lambda data: data["origin"].__setitem__("north_deg", round(ang % 360.0, 3)))
                self.message.emit(f"North points {(90 - ang) % 360:.0f}° clockwise from the top of the plan")
                return
            pt = list(self._snap(sp.x(), sp.y(), e.modifiers()))

            def move(data):
                if kind == "origin":
                    data["origin"]["xy"] = pt
                    return
                if kind == "truss":
                    data["trusses"][idx]["start" if vtx == 0 else "end"] = pt
                elif kind == "pad":
                    data["pads"]["centres"][idx] = pt
                else:
                    self._points_of(data, kind, idx)[vtx] = pt
            self.doc.set_live(move)
            return
        if self.tool != "select":
            self.cursor_pt = self._snap(sp.x(), sp.y(), e.modifiers())
            self.viewport().update()

    def mouseReleaseEvent(self, e):
        if self.pan_from is not None:
            self.pan_from = None
            self.setCursor(Qt.CursorShape.ArrowCursor if self.tool == "select" else Qt.CursorShape.CrossCursor)
            return
        if self.drag is not None:
            what = {"move": "Move origin", "rotate": "Turn North"}.get(self.drag[2], "Move vertex") \
                if self.drag[0] == "origin" else "Move charging pad" if self.drag[0] == "pad" else "Move vertex"
            if self.drag[0] == "pad":
                self._warn_close_pads()
            self.doc.commit(what, self.drag_before)
            self.drag = None
            self.drag_before = None

    def mouseDoubleClickEvent(self, e):
        if self.tool in ("volume", "zone", "nogo") and len(self.pending) >= 3:
            self._finish()
            return
        if self.tool == "select":
            sp = self.mapToScene(e.position().toPoint())
            self.insert_vertex_near(sp.x(), sp.y())

    def insert_vertex_near(self, x: float, y: float) -> bool:
        """Double-click on an outline edge inserts a vertex there."""
        tol = HIT_PX * self.px()
        for kind, idx, pts in self._polygons():
            for k in range(len(pts)):
                a, b = pts[k], pts[(k + 1) % len(pts)]
                if len(pts) >= 3 and _seg_dist((x, y), a, b) < tol:
                    pt = list(self._snap(x, y, Qt.KeyboardModifier.NoModifier))

                    def ins(data, kind=kind, idx=idx, k=k):
                        self._points_of(data, kind, idx).insert(k + 1, pt)
                    self.doc.edit("Insert vertex", ins)
                    if kind == "volume":
                        self.set_selection(("vertex", k + 1))
                    return True
        return False

    def keyPressEvent(self, e):
        key = e.key()
        if key == Qt.Key.Key_Escape:
            if self.ruler:
                self.ruler = []
            elif self.pending:
                self.pending = []
            else:
                self.set_tool("select")
            self.viewport().update()
            return
        if key in (Qt.Key.Key_Return, Qt.Key.Key_Enter) and self.tool in ("volume", "zone", "nogo"):
            self._finish()
            return
        if key in (Qt.Key.Key_Backspace, Qt.Key.Key_Delete):
            if self.tool != "select" and self.pending:
                self.pending.pop()
                self.viewport().update()
            else:
                self.delete_selection()
            return
        super().keyPressEvent(e)

    def _finish(self):
        pts = [list(p) for p in self.pending]
        tool = self.tool
        self.pending = []
        self.cursor_pt = None
        if tool == "truss" and len(pts) == 2:
            d = self.doc.data

            def add(data):
                ids = {t["id"] for t in data["trusses"]}
                n = len(data["trusses"]) + 1
                while f"T{n}" in ids:
                    n += 1
                data["trusses"].append({"id": f"T{n}", "start": pts[0], "end": pts[1],
                                        "height_m": float(d["volume"]["z"][1]) + 1.5})
            self.doc.edit("Add truss", add)
            self.set_selection(("truss", len(self.doc.data["trusses"]) - 1))
            self.set_tool("select")
        elif tool in ("volume", "zone", "nogo") and len(pts) >= 3:
            if tool == "volume":
                self.doc.edit("Draw performance area", lambda data: data["volume"].__setitem__("polygon", pts))
                self.zoom_to_fit()
                if "origin" not in self.doc.data:           # step 2: where is the calibration square?
                    self.set_selection(None)
                    self.set_tool("origin")
                else:
                    self.set_selection(("volume", None))
                    self.set_tool("select")
            elif tool == "zone":
                def add(data):
                    n = len(data["tripod_zones"]) + 1
                    data["tripod_zones"].append({"id": f"area{n}", "polygon": pts, "height_m": [2.0, 20.0]})
                self.doc.edit("Add tripod area", add)
                self.set_selection(("zone", len(self.doc.data["tripod_zones"]) - 1))
                self.set_tool("select")
            else:
                self.doc.edit("Add no-go area", lambda data: data["no_go_zones"].append(pts))
                self.set_selection(("nogo", len(self.doc.data["no_go_zones"]) - 1))
                self.set_tool("select")
        self.viewport().update()

    def delete_selection(self):
        if self.selection:
            self.delete_item(*self.selection)

    def item_name(self, kind, idx) -> str:
        d = self.doc.data
        if kind == "truss":
            return f"Truss {d['trusses'][idx]['id']}"
        if kind == "zone":
            z = d["tripod_zones"][idx]
            return ("Tripod strip " if "offset_m" in z else "Tripod area ") + z["id"]
        if kind == "nogo":
            return f"No-go area {idx + 1}"
        if kind == "vertex":
            return f"Vertex V{idx}"
        if kind == "pad":
            return f"Charging pad {idx + 1}"
        if kind == "origin":
            return "Origin"
        return "Performance area"

    def contextMenuEvent(self, e):
        """Right-click / two-finger click: delete the item under the pointer."""
        menu = self.context_menu_at(e.pos())
        if menu is not None:
            menu.exec(e.globalPos())

    def context_menu_at(self, pos):
        from PySide6.QtWidgets import QMenu
        sp = self.mapToScene(pos)
        hit = self._hit(sp.x(), sp.y())
        if hit is None or hit[0] in ("camera", "origin"):
            return None
        kind, idx, vtx = hit
        if kind == "volume" and vtx is not None:
            kind, idx = "vertex", vtx
        elif kind == "truss" or vtx is None:
            pass
        else:
            vtx = None                                   # zone / no-go vertex: act on the whole item
        self.set_selection((kind, idx))
        menu = QMenu(self)
        act = menu.addAction(f"Delete {self.item_name(kind, idx)}")
        act.triggered.connect(lambda: self.delete_item(kind, idx))
        return menu

    def delete_item(self, kind, idx):
        if kind == "vertex":
            if len(self.doc.data["volume"]["polygon"]) <= 3:
                self.message.emit("The volume outline needs at least 3 vertices.")
                return
            self.doc.edit("Delete vertex", lambda d: d["volume"]["polygon"].pop(idx))
        elif kind == "truss":
            self.doc.edit("Delete truss", lambda d: d["trusses"].pop(idx))
        elif kind == "zone":
            self.doc.edit("Delete tripod zone", lambda d: d["tripod_zones"].pop(idx))
        elif kind == "nogo":
            self.doc.edit("Delete no-go area", lambda d: d["no_go_zones"].pop(idx))
        elif kind == "pad":
            self.doc.edit("Delete charging pad", lambda d: d["pads"]["centres"].pop(idx))
            if idx < len(self._pads()):
                self.message.emit("Pads after it moved up one: their drone IDs are 4 lower now.")
        elif kind == "volume":
            self.doc.edit("Clear volume", lambda d: d["volume"].__setitem__("polygon", []))
        self.set_selection(None)

    def place_pad(self, pt) -> bool:
        """Add a charging pad at pt (drawing XY), unless it would stand too close to another."""
        pads = self._pads()
        near = [k for k, c in enumerate(pads) if math.hypot(c[0] - pt[0], c[1] - pt[1]) < MIN_PAD_GAP_M - 1e-9]
        if near:
            self.message.emit(f"Too close to pad {near[0] + 1}: pads stand at least {MIN_PAD_GAP_M:g} m apart "
                              "(centre to centre).")
            return False

        def add(data):
            data.setdefault("pads", {}).setdefault("centres", []).append([float(pt[0]), float(pt[1])])
        self.doc.edit("Place charging pad", add)
        k = len(self._pads())
        self.set_selection(("pad", k - 1))
        self.message.emit(f"Pad {k} placed: drones {SLOTS * (k - 1) + 1}–{SLOTS * k} (slot = take-off batch). "
                          "Click to place the next; Esc when done, then Plan Cameras (⌘R).")
        self.viewport().update()
        return True

    def _warn_close_pads(self):
        for i, j, dist in _pads_too_close(self._pads()):
            self.message.emit(f"Pads {i + 1} and {j + 1} are {dist:.2f} m apart: keep pads at least "
                              f"{MIN_PAD_GAP_M:g} m apart.")
            return


def _pads_too_close(centres):
    from idps.show.pads import too_close
    return too_close(centres)

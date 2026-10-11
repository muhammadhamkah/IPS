"""3D view of the venue: extruded flight volume, trusses at height, tripod masts,
planned cameras with view cones, and the coverage layer at the chosen altitude.

Drawn with QPainter and a perspective projection (no OpenGL), so it behaves the
same on every Mac and in headless tests. macOS: drag or two-finger scroll
orbits, Shift pans, pinch or Cmd+scroll zooms.
"""
from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import QEvent, QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget

from idps.coverage.volume import offset_ring
from idps.ui import theme


class View3D(QWidget):
    cameraClicked = Signal(int)

    def __init__(self, doc, canvas, parent=None):
        super().__init__(parent)
        self.doc, self.canvas = doc, canvas
        self.setMinimumSize(300, 200)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.yaw, self.pitch = math.radians(-60), math.radians(30)
        self.dist = 60.0
        self.target = np.zeros(3)
        self.fov = math.radians(45)
        self.show_cones = "selected"         # "none" | "selected" | "all"
        self.show_coverage = True
        self._drag = None
        self._fitted = False
        doc.changed.connect(self.update)
        canvas.selectionChanged.connect(lambda _: self.update())

    # ------------------------------------------------------------- camera
    def _basis(self):
        cp, sp, cy, sy = math.cos(self.pitch), math.sin(self.pitch), math.cos(self.yaw), math.sin(self.yaw)
        fwd = -np.array([cp * cy, cp * sy, sp])            # from eye towards target
        right = np.array([-sy, cy, 0.0])
        up = np.cross(right, fwd)
        eye = self.target - fwd * self.dist
        return eye, right, up, fwd

    def project(self, P):
        """World points (N, 3) -> screen (N, 2) and depth (N,); depth <= 0 is behind the eye."""
        P = np.atleast_2d(np.asarray(P, float))
        eye, right, up, fwd = self._basis()
        d = P - eye
        x, y, z = d @ right, d @ up, d @ fwd
        f = (self.height() / 2) / math.tan(self.fov / 2)
        zs = np.where(z > 1e-3, z, 1e-3)
        sx = self.width() / 2 + f * x / zs
        sy = self.height() / 2 - f * y / zs
        return np.stack([sx, sy], 1), z

    def set_view(self, name: str):
        presets = {"top": (-90, 89.9), "front": (-90, 0), "side": (0, 0), "iso": (-60, 30)}
        yaw, pitch = presets[name]
        self.yaw, self.pitch = math.radians(yaw), math.radians(pitch)
        self.fit()

    def fit(self):
        pts = self._scene_points()
        if len(pts) == 0:
            pts = np.array([[-10, -10, 0], [10, 10, 5]], float)
        lo, hi = pts.min(0), pts.max(0)
        self.target = (lo + hi) / 2
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        self.dist = max(np.linalg.norm(hi - lo), 4.0) * 3
        w, h = max(self.width(), 1), max(self.height(), 1)
        # Shrink the distance until the bounding box fills ~85 % of the view.
        for _ in range(40):
            P, z = self.project(corners)
            if (z <= 0.05).any():
                break
            ext = max((P[:, 0].max() - P[:, 0].min()) / w, (P[:, 1].max() - P[:, 1].min()) / h)
            if ext > 0.85:
                break
            self.dist *= 0.9
        self.dist /= 0.9
        self._fitted = True
        self.update()

    def _scene_points(self):
        d = self.doc.data
        vol = d["volume"]["polygon"]
        z0, z1 = d["volume"]["z"]
        pts = [[x, y, z] for x, y in vol for z in (z0, z1)]
        for t in d["trusses"]:
            pts += [[*t["start"], t["height_m"]], [*t["end"], t["height_m"]]]
        if self.canvas.result is not None:
            pts += [c.position_m for c in self.canvas.result[0].cameras]
        return np.array(pts, float) if pts else np.zeros((0, 3))

    # ------------------------------------------------------------- input
    def mousePressEvent(self, e):
        self._drag = (e.position(), e.modifiers() & Qt.KeyboardModifier.ShiftModifier
                      or e.button() == Qt.MouseButton.RightButton, False)

    def mouseMoveEvent(self, e):
        if self._drag is None:
            return
        start, pan, _ = self._drag
        d = e.position() - start
        self._drag = (e.position(), pan, True)
        if pan:
            self._pan(d.x(), d.y())
        else:
            self._orbit(d.x(), d.y())

    def mouseReleaseEvent(self, e):
        if self._drag is not None and not self._drag[2]:
            self._pick(e.position())
        self._drag = None

    def _orbit(self, dx, dy):
        self.yaw -= dx * 0.008
        self.pitch = min(math.radians(89.9), max(math.radians(-89.9), self.pitch + dy * 0.008))
        self.update()

    def _pan(self, dx, dy):
        _, right, up, _ = self._basis()
        s = self.dist * 2 * math.tan(self.fov / 2) / max(self.height(), 1)
        self.target = self.target - right * dx * s + up * dy * s
        self.update()

    def _zoom(self, factor):
        self.dist = min(5000.0, max(0.5, self.dist / factor))
        self.update()

    def wheelEvent(self, e):
        pd = e.pixelDelta()
        cmd = e.modifiers() & Qt.KeyboardModifier.ControlModifier          # Cmd on macOS
        if not pd.isNull() and not cmd:
            if e.modifiers() & Qt.KeyboardModifier.ShiftModifier:
                self._pan(pd.x(), pd.y())
            else:
                self._orbit(pd.x(), pd.y())
        else:
            self._zoom(1.0015 ** (e.angleDelta().y() or pd.y()))
        e.accept()

    def event(self, e):
        if e.type() == QEvent.Type.NativeGesture and e.gestureType() == Qt.NativeGestureType.ZoomNativeGesture:
            self._zoom(1.0 + e.value())
            return True
        return super().event(e)

    def _pick(self, pos):
        if self.canvas.result is None:
            return
        cams = self.canvas.result[0].cameras
        P, z = self.project([c.position_m for c in cams])
        d = np.hypot(P[:, 0] - pos.x(), P[:, 1] - pos.y())
        d[z <= 0] = np.inf
        k = int(np.argmin(d)) if len(d) else -1
        if k >= 0 and d[k] < 12:
            self.canvas.set_selection(("camera", k))
            self.cameraClicked.emit(k)

    # ------------------------------------------------------------- drawing
    def paintEvent(self, _):
        if not self._fitted and self.width() > 10:
            self.fit()
        tok = theme.tokens()
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), tok["surface"])
        prims = []                       # (depth, kind, payload)
        self._grid(prims, tok)
        self._volume(prims, tok)
        self._areas(prims, tok)
        self._trusses(prims, tok)
        self._coverage(prims)
        self._cameras(prims, tok)
        for _, kind, payload in sorted(prims, key=lambda q: -q[0]):
            if kind == "poly":
                pts, pen, brush = payload
                p.setPen(pen)
                p.setBrush(brush)
                p.drawPolygon(QPolygonF([QPointF(*q) for q in pts]))
            elif kind == "line":
                a, b, pen = payload
                p.setPen(pen)
                p.drawLine(QPointF(*a), QPointF(*b))
            elif kind == "dot":
                c, r, fill, edge = payload
                p.setPen(QPen(edge, 1.2))
                p.setBrush(fill)
                p.drawEllipse(QPointF(*c), r, r)
            elif kind == "text":
                pos, text, colour, bold, size = payload
                f = QFont(self.font())
                f.setPointSizeF(size)
                f.setBold(bold)
                p.setFont(f)
                p.setPen(colour)
                p.drawText(QPointF(pos[0] + 6, pos[1] - 6), text)
        self._overlay_text(p, tok)
        p.end()

    def _line(self, prims, a, b, pen):
        (P, z) = self.project([a, b])
        if (z > 0.05).all():
            prims.append((float(z.mean()), "line", (P[0], P[1], pen)))

    def _poly(self, prims, pts3, pen, brush):
        P, z = self.project(pts3)
        if (z > 0.05).all():
            prims.append((float(z.mean()), "poly", (P, pen, brush)))

    def _text(self, prims, at, text, colour, bold=False, size=10):
        P, z = self.project([at])
        if z[0] > 0.05:
            prims.append((float(z[0]) - 1e6, "text", (P[0], text, colour, bold, size)))   # text always on top

    def _grid(self, prims, tok):
        pts = self._scene_points()
        if len(pts):
            lo, hi = pts.min(0)[:2] - 5, pts.max(0)[:2] + 5
        else:
            lo, hi = np.array([-20.0, -20.0]), np.array([20.0, 20.0])
        span = max(hi - lo)
        step = 1 if span < 30 else 5 if span < 150 else 10
        pen = QPen(tok["grid_minor"], 1)
        for x in np.arange(math.floor(lo[0] / step) * step, hi[0] + step, step):
            self._line(prims, (x, lo[1], 0), (x, hi[1], 0), pen)
        for y in np.arange(math.floor(lo[1] / step) * step, hi[1] + step, step):
            self._line(prims, (lo[0], y, 0), (hi[0], y, 0), pen)
        L = max(2.0, step * 2)
        for axis, colour, label in ((0, "#d03b3b", "+X"), (1, "#0ca30c", "+Y"), (2, "#3987e5", "+Z")):
            end = [0.0, 0.0, 0.0]
            end[axis] = L
            self._line(prims, (0, 0, 0), end, QPen(QColor(colour), 2.5))
            self._text(prims, end, label, QColor(colour), True, 10)
        self._text(prims, (0, 0, 0), "origin", tok["text2"], False, 9)

    def _volume(self, prims, tok):
        d = self.doc.data
        vol = d["volume"]["polygon"]
        if len(vol) < 3:
            return
        z0, z1 = d["volume"]["z"]
        c = QColor(tok["volume"])
        edge = QPen(c, 1.6)
        fill = QColor(c)
        fill.setAlphaF(0.07)
        n = len(vol)
        for k in range(n):
            a, b = vol[k], vol[(k + 1) % n]
            self._poly(prims, [(*a, z0), (*b, z0), (*b, z1), (*a, z1)], QPen(Qt.PenStyle.NoPen), fill)
            self._line(prims, (*a, z0), (*b, z0), edge)
            self._line(prims, (*a, z1), (*b, z1), edge)
            self._line(prims, (*a, z0), (*a, z1), QPen(c, 1.0, Qt.PenStyle.DashLine))
            self._text(prims, (*a, z0), f"V{k}", tok["text"], True, 10)
        k = len(vol) // 2                    # away from V0, where origin labels usually sit
        self._text(prims, (*vol[k], z1), f"top {z1:g} m", tok["text2"], False, 9)

    def _areas(self, prims, tok):
        d = self.doc.data
        vol = d["volume"]["polygon"]
        for z in d["tripod_zones"]:
            c = QColor(tok["zone"])
            pen = QPen(c, 1.2, Qt.PenStyle.DashLine)
            rings = []
            if "polygon" in z:
                rings = [z["polygon"]]
            elif "offset_m" in z and len(vol) >= 3:
                for off in z["offset_m"]:
                    r = offset_ring(np.array(vol, float), float(off), 1e9)
                    rings.append(r[1::2] if len(r) == 2 * len(vol) else r)
            for r in rings:
                for k in range(len(r)):
                    self._line(prims, (*r[k], 0), (*r[(k + 1) % len(r)], 0), pen)
        for r in d["no_go_zones"]:
            c = QColor(tok["nogo"])
            fill = QColor(c)
            fill.setAlphaF(0.18)
            self._poly(prims, [(*q, 0.01) for q in r], QPen(c, 1.2), fill)

    def _trusses(self, prims, tok):
        sel = self.canvas.selection
        for i, t in enumerate(self.doc.data["trusses"]):
            a, b = np.array(t["start"], float), np.array(t["end"], float)
            h = float(t["height_m"])
            L = float(np.linalg.norm(b - a))
            colour = tok["select"] if sel == ("truss", i) else tok["truss"]
            self._line(prims, (*a, h), (*b, h), QPen(tok["truss_gap"], 2, Qt.PenStyle.DotLine))
            segs = [(0, L)] if t.get("usable") in (None, "all") else t["usable"]
            for s0, s1 in segs:
                if L > 0:
                    pa, pb = a + (b - a) * (s0 / L), a + (b - a) * (min(s1, L) / L)
                    self._line(prims, (*pa, h), (*pb, h), QPen(colour, 5, Qt.PenStyle.SolidLine, Qt.PenCapStyle.FlatCap))
            self._text(prims, (*a, h), f"{t['id']}  {h:g} m", tok["text2"], sel == ("truss", i), 10)

    def _coverage(self, prims):
        if not self.show_coverage or self.canvas.result is None or self.canvas.overlay_mode == "none":
            return
        plan, ev = self.canvas.result
        g = ev.grid
        k = self.canvas.overlay_level
        on = np.where(g.level_of() == k)[0]
        if not len(on):
            return
        req = plan.summary.get("requirements", {})
        limit = float(req.get("p95_mm", 10.0))
        P, z = self.project(ev.points[on])
        for (sx, sy), depth, v in zip(P, z, on):
            if depth <= 0.05:
                continue
            if self.canvas.overlay_mode == "count":
                c = QColor(theme.STATUS[min(int(ev.n_cams[v]), 3)])
            else:
                p95 = ev.p95_mm[v]
                c = theme.ramp_color(p95 if np.isfinite(p95) else 2 * limit, 2 * limit)
            c.setAlphaF(0.75)
            prims.append((float(depth), "dot", ((sx, sy), 3.2, c, c)))

    def _cameras(self, prims, tok):
        if self.canvas.result is None:
            return
        plan, _ = self.canvas.result
        sel = self.canvas.selection
        sel_k = sel[1] if sel and sel[0] == "camera" else None
        stale = self.canvas.result_stale
        # tripod masts: one per floor spot, up to its highest camera
        masts: dict = {}
        for c in plan.cameras:
            if c.mount_type == "tripod":
                key = (round(c.position_m[0], 2), round(c.position_m[1], 2))
                masts[key] = max(masts.get(key, 0.0), c.position_m[2])
        mast_pen = QPen(QColor(tok["cam_tripod"]), 2.2)
        for (x, y), top in masts.items():
            self._line(prims, (x, y, 0), (x, y, top), mast_pen)
            for dx, dy in ((0.6, 0), (-0.3, 0.52), (-0.3, -0.52)):            # tripod feet
                self._line(prims, (x, y, min(1.2, top)), (x + dx, y + dy, 0), QPen(QColor(tok["cam_tripod"]), 1.4))
        sensor_aspect = 800 / 1280
        for k, c in enumerate(plan.cameras):
            colour = QColor(tok["cam_truss"] if c.mount_type == "truss" else tok["cam_tripod"])
            if stale:
                colour.setAlphaF(0.45)
            pos = np.array(c.position_m, float)
            if c.mount_type == "truss":
                self._line(prims, pos, pos + (0, 0, 0.3), QPen(colour, 1.5))
            hl = k == sel_k
            if self.show_cones == "all" or (self.show_cones == "selected" and hl):
                self._cone(prims, c, pos, colour, sensor_aspect, hl)
            else:
                R = c.R
                self._line(prims, pos, pos + R[2] * 1.5, QPen(colour, 2))
            P, z = self.project([pos])
            if z[0] > 0.05:
                prims.append((float(z[0]) - 0.01, "dot", (P[0], 6.5 if hl else 4.5, colour, tok["surface"])))
            if hl or len(plan.cameras) <= 40:
                self._text(prims, pos, c.id, tok["text"] if hl else tok["text2"], hl, 10 if hl else 9)

    def _cone(self, prims, c, pos, colour, aspect, hl):
        """Field-of-view pyramid out to a range that keeps the picture readable."""
        R = c.R
        half_w = math.atan(1280 / 2 / (c.lens_mm * 1e-3 / 3e-6))
        half_h = math.atan(math.tan(half_w) * aspect)
        pts = self._scene_points()
        span = float(np.linalg.norm(pts.max(0) - pts.min(0))) if len(pts) else 20.0
        L = 0.35 * span
        corners = []
        for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            d = R[2] + R[0] * math.tan(half_w) * sx + R[1] * math.tan(half_h) * sy
            corners.append(pos + d / np.linalg.norm(d) * L)
        fill = QColor(colour)
        fill.setAlphaF(0.10 if hl else 0.05)
        pen = QPen(colour, 1.6 if hl else 0.8)
        for a, b in zip(corners, corners[1:] + corners[:1]):
            self._poly(prims, [pos, a, b], QPen(Qt.PenStyle.NoPen), fill)
            self._line(prims, pos, a, pen)
            self._line(prims, a, b, pen)

    def _overlay_text(self, p, tok):
        f = QFont(self.font())
        f.setPointSizeF(10)
        p.setFont(f)
        p.setPen(tok["text2"])
        hint = "Drag or two-finger scroll: orbit · ⇧: pan · pinch or ⌘-scroll: zoom · click a camera to select"
        p.drawText(QRectF(10, self.height() - 24, self.width() - 20, 20), Qt.AlignmentFlag.AlignLeft, hint)
        if self.canvas.result is not None and self.canvas.overlay_mode != "none" and self.show_coverage:
            z = self.canvas.result[1].grid.zs[self.canvas.overlay_level]
            what = "cameras per point" if self.canvas.overlay_mode == "count" else "predicted accuracy"
            p.drawText(QRectF(10, 8, self.width() - 20, 20), Qt.AlignmentFlag.AlignLeft,
                       f"Coverage layer: {what} at Z = {z:.1f} m (altitude slider below)")

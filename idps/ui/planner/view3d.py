"""3D view of the venue: extruded flight volume, trusses at height, tripod masts,
planned cameras with view cones, and the coverage layer at the chosen altitude.

Drawn with QPainter and a perspective projection (no OpenGL), so it behaves the
same on every Mac and in headless tests. macOS: drag or two-finger scroll
orbits, Shift pans, pinch or Cmd+scroll zooms.
"""
from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import QEvent, QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget

from idps.coverage.volume import offset_ring
from idps.ui import theme


def mount_counts(cameras) -> dict:
    """{(x, y, z): number of cameras} for cameras sharing a physical spot
    (same truss point, or same tripod; tripods counted at the mast top)."""
    out: dict = {}
    for c in cameras:
        x, y, z = (round(v, 2) for v in c.position_m)
        key = (x, y, z) if c.mount_type == "truss" else (x, y, None)
        out[key] = out.get(key, 0) + 1
    tops: dict = {}
    for c in cameras:
        if c.mount_type == "tripod":
            k = (round(c.position_m[0], 2), round(c.position_m[1], 2))
            tops[k] = max(tops.get(k, 0.0), c.position_m[2])
    return {(k[0], k[1], k[2] if k[2] is not None else tops[(k[0], k[1])]): n for k, n in out.items()}


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
        self.coverage_mode = "slice"         # "slice" | "volume" | "problems" | "off"
        self.preview = None                  # advisor Recommendation being previewed
        self._drag = None
        self._fitted = False
        self._gizmo_hits: list = []          # (QRectF or (centre, radius), action) built while painting
        self._repeat = QTimer(self)          # hold a gizmo button to keep panning/rotating
        self._repeat.timeout.connect(self._repeat_tick)
        self._repeat_action = None
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
        action = self._gizmo_action(e.position())
        if action == "orbit":                 # drag on the axis ball orbits; a click on an axis snaps
            self._drag = (e.position(), False, False, "ball")
            return
        if action is not None:
            self._run_gizmo(action)
            self._repeat_action = action
            if action.startswith(("pan", "rot", "zoom", "tilt")):
                self._repeat.start(350)       # first repeat after a pause, then fast
            return
        self._drag = (e.position(), e.modifiers() & Qt.KeyboardModifier.ShiftModifier
                      or e.button() == Qt.MouseButton.RightButton, False, "scene")

    def mouseMoveEvent(self, e):
        if self._drag is None:
            return
        start, pan, _, where = self._drag
        d = e.position() - start
        self._drag = (e.position(), pan, True, where)
        if pan:
            self._pan(d.x(), d.y())
        else:
            self._orbit(d.x(), d.y())

    def mouseReleaseEvent(self, e):
        self._repeat.stop()
        self._repeat_action = None
        if self._drag is not None and not self._drag[2]:
            if self._drag[3] == "ball":
                axis = self._axis_at(e.position())
                if axis:
                    self._run_gizmo("view" + axis)
            else:
                self._pick(e.position())
        self._drag = None

    # ------------------------------------------------------------- gizmo
    GIZMO_R = 44

    def _gizmo_geometry(self):
        cx, cy = self.width() - self.GIZMO_R - 18, self.GIZMO_R + 18
        return QPointF(cx, cy), self.GIZMO_R

    def _axis_points(self):
        """Screen positions of the six axis tips on the gizmo ball, with depth."""
        c, r = self._gizmo_geometry()
        _, right, up, fwd = self._basis()
        out = []
        for name, v in (("+X", (1, 0, 0)), ("-X", (-1, 0, 0)), ("+Y", (0, 1, 0)), ("-Y", (0, -1, 0)),
                        ("+Z", (0, 0, 1)), ("-Z", (0, 0, -1))):
            v = np.array(v, float)
            out.append((name, QPointF(c.x() + v @ right * r * 0.78, c.y() - v @ up * r * 0.78), float(v @ fwd)))
        return out

    def _axis_at(self, pos):
        best = None
        for name, p, depth in self._axis_points():
            dist = math.hypot(p.x() - pos.x(), p.y() - pos.y())
            if dist < 11 and (best is None or depth < best[1]):     # nearest-to-viewer tip wins
                best = (name, depth)
        return best[0] if best else None

    def _gizmo_action(self, pos):
        for region, action in self._gizmo_hits:
            if isinstance(region, QRectF):
                if region.contains(pos):
                    return action
            else:
                c, r = region
                if math.hypot(pos.x() - c.x(), pos.y() - c.y()) <= r:
                    return action
        return None

    def _run_gizmo(self, action: str):
        step_px = max(self.height(), 1) * 0.08
        views = {"+X": (0, 0), "-X": (180, 0), "+Y": (90, 0), "-Y": (-90, 0), "+Z": (None, 89.9), "-Z": (None, -89.9)}
        if action.startswith("view"):
            yaw, pitch = views[action[4:]]
            if yaw is not None:
                self.yaw = math.radians(yaw)
            else:                                     # top/bottom: keep heading, squared to the axes
                self.yaw = round(self.yaw / (math.pi / 2)) * (math.pi / 2)
            self.pitch = math.radians(pitch)
            self.update()
        elif action == "pan_up":
            self._pan(0, step_px)
        elif action == "pan_down":
            self._pan(0, -step_px)
        elif action == "pan_left":
            self._pan(step_px, 0)
        elif action == "pan_right":
            self._pan(-step_px, 0)
        elif action == "rot_left":
            self._orbit(-math.radians(15) / 0.008, 0)
        elif action == "rot_right":
            self._orbit(math.radians(15) / 0.008, 0)
        elif action == "tilt_up":
            self._orbit(0, math.radians(10) / 0.008)
        elif action == "tilt_down":
            self._orbit(0, -math.radians(10) / 0.008)
        elif action == "zoom_in":
            self._zoom(1.2)
        elif action == "zoom_out":
            self._zoom(1 / 1.2)
        elif action == "home":
            self.set_view("iso")

    def _repeat_tick(self):
        if self._repeat_action:
            self._run_gizmo(self._repeat_action)
            self._repeat.start(70)

    def _draw_gizmo(self, p: QPainter, tok):
        """Axis ball (click an axis to look along it, drag to orbit) plus pan,
        rotate, tilt and zoom buttons, top right of the view."""
        self._gizmo_hits = []
        c, r = self._gizmo_geometry()
        bg = QColor(tok["surface"])
        bg.setAlphaF(0.85)
        ring = QColor(tok["text2"])
        ring.setAlphaF(0.35)
        p.setPen(QPen(ring, 1))
        p.setBrush(bg)
        p.drawEllipse(c, r, r)
        colours = {"X": QColor("#d03b3b"), "Y": QColor("#0ca30c"), "Z": QColor("#3987e5")}
        f = QFont(self.font())
        f.setPointSizeF(8)
        f.setBold(True)
        p.setFont(f)
        for name, tip, depth in sorted(self._axis_points(), key=lambda q: -q[2]):
            col = QColor(colours[name[1]])
            positive = name[0] == "+"
            if positive:
                p.setPen(QPen(col, 2.2))
                p.drawLine(c, tip)
            p.setPen(QPen(col, 1.5))
            p.setBrush(col if positive else bg)
            p.drawEllipse(tip, 9, 9)
            p.setPen(QColor("#ffffff") if positive else col)
            p.drawText(QRectF(tip.x() - 9, tip.y() - 9, 18, 18), Qt.AlignmentFlag.AlignCenter,
                       name[1] if positive else "-" + name[1])
        self._gizmo_hits.append(((c, r), "orbit"))

        # Button pad under the ball: pan arrows around a home button, then rotate/tilt/zoom.
        size, gap = 24, 3
        x0 = c.x() - (3 * size + 2 * gap) / 2
        y0 = c.y() + r + 10
        pad = [("pan_up", "↑", 1, 0, "Pan up"), ("pan_left", "←", 0, 1, "Pan left"), ("home", "⌂", 1, 1, "Home view"),
               ("pan_right", "→", 2, 1, "Pan right"), ("pan_down", "↓", 1, 2, "Pan down"),
               ("rot_left", "⟲", 0, 3, "Rotate left"), ("tilt_up", "⤒", 1, 3, "Tilt up"),
               ("rot_right", "⟳", 2, 3, "Rotate right"), ("zoom_out", "−", 0, 4, "Zoom out"),
               ("tilt_down", "⤓", 1, 4, "Tilt down"), ("zoom_in", "+", 2, 4, "Zoom in")]
        f.setPointSizeF(11)
        f.setBold(False)
        p.setFont(f)
        for action, glyph, col_i, row_i, _tip in pad:
            rect = QRectF(x0 + col_i * (size + gap), y0 + row_i * (size + gap) + (6 if row_i >= 3 else 0), size, size)
            p.setPen(QPen(ring, 1))
            p.setBrush(bg)
            p.drawRoundedRect(rect, 5, 5)
            p.setPen(tok["text"])
            p.drawText(rect, Qt.AlignmentFlag.AlignCenter, glyph)
            self._gizmo_hits.append((rect, action))
        self._gizmo_tips = {a: t for a, *_, t in pad}

    def gizmo_tooltip(self, pos):
        a = self._gizmo_action(pos)
        if a == "orbit":
            return "Drag to orbit · click an axis to look along it"
        return getattr(self, "_gizmo_tips", {}).get(a)

    def event_tooltip(self, e):
        from PySide6.QtWidgets import QToolTip
        tip = self.gizmo_tooltip(e.position() if hasattr(e, "position") else QPointF(e.pos()))
        if tip:
            QToolTip.showText(e.globalPos(), tip, self)
        else:
            QToolTip.hideText()
        return True

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
        if e.type() == QEvent.Type.ToolTip:
            return self.event_tooltip(e)
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
        self._extra(prims, tok)
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
        self._draw_gizmo(p, tok)
        p.end()

    def _extra(self, prims, tok):
        """Subclasses add their own layers here (e.g. live drones and alerts)."""

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

    def _point_colour(self, ev, v, limit, measure):
        if measure == "accuracy":
            p95 = ev.p95_mm[v]
            return theme.ramp_color(p95 if np.isfinite(p95) else 2 * limit, 2 * limit)
        return QColor(theme.STATUS[min(int(ev.n_cams[v]), 3)])

    def coverage_stats(self):
        """(failing points, failing volume m3) for the current result, or None."""
        if self.canvas.result is None:
            return None
        plan, ev = self.canvas.result
        s = ev.summary
        return int((~ev.meets).sum()), float(s.get("volume_below_min_cameras_m3", 0.0)
                                             + s.get("volume_over_accuracy_limit_m3", 0.0))

    def _coverage(self, prims):
        """coverage_mode: 'slice' = solid layer at the altitude slider, 'volume' = every
        evaluation point, 'problems' = only points failing the requirement, 'off'."""
        if self.coverage_mode == "off" or self.canvas.result is None:
            return
        plan, ev = self.canvas.result
        g = ev.grid
        req = plan.summary.get("requirements", {})
        limit = float(req.get("p95_mm", 10.0))
        measure = "accuracy" if self.canvas.overlay_mode == "accuracy" else "count"
        if self.coverage_mode == "slice":
            k = self.canvas.overlay_level
            lvl = g.level_of()
            z = float(g.zs[k])
            xs, ys = g.xs, g.ys
            dx = (xs[1] - xs[0]) if len(xs) > 1 else 1.0
            dy = (ys[1] - ys[0]) if len(ys) > 1 else 1.0
            grid_idx = np.where(lvl[:g.n_grid] == k)[0]
            ii, jj = np.nonzero(g.inside)
            for (i, j), v in zip(zip(ii, jj), grid_idx):
                c = self._point_colour(ev, v, limit, measure)
                c.setAlphaF(0.55)
                x0, y0 = xs[i] - dx / 2, ys[j] - dy / 2
                self._poly(prims, [(x0, y0, z), (x0 + dx, y0, z), (x0 + dx, y0 + dy, z), (x0, y0 + dy, z)],
                           QPen(Qt.PenStyle.NoPen), c)
            ring = np.where(lvl == k)[0]
            ring = ring[ring >= g.n_grid]
            P, depth = self.project(ev.points[ring]) if len(ring) else (np.zeros((0, 2)), np.zeros(0))
            for (sx, sy), d, v in zip(P, depth, ring):
                if d > 0.05:
                    c = self._point_colour(ev, v, limit, measure)
                    prims.append((float(d), "dot", ((sx, sy), 3.0, c, c)))
            return
        sel = np.arange(len(ev.points)) if self.coverage_mode == "volume" else np.where(~ev.meets)[0]
        if not len(sel):
            return
        P, depth = self.project(ev.points[sel])
        for (sx, sy), d, v in zip(P, depth, sel):
            if d <= 0.05:
                continue
            bad = not ev.meets[v]
            c = self._point_colour(ev, v, limit, measure)
            if self.coverage_mode == "volume" and not bad:
                c.setAlphaF(0.30)
                prims.append((float(d), "dot", ((sx, sy), 2.2, c, c)))
            else:
                edge = QColor(theme.STATUS[0])
                prims.append((float(d) - 0.02, "dot", ((sx, sy), 5.0, c, edge)))

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
        if self.preview is not None:                     # advisor suggestion: ghost camera + view cone
            r = self.preview
            col = QColor(tok["select"])
            new_pos = np.array(r.new_camera.position_m, float)
            if r.kind == "move" and r.camera_id:
                old = next((c for c in plan.cameras if c.id == r.camera_id), None)
                if old is not None:
                    self._line(prims, old.position_m, new_pos, QPen(col, 2, Qt.PenStyle.DashLine))
            if r.new_camera.mount_type == "tripod":
                self._line(prims, (new_pos[0], new_pos[1], 0), new_pos, QPen(col, 2, Qt.PenStyle.DashLine))
            self._cone(prims, r.new_camera, new_pos, col, sensor_aspect, True)
            P, z = self.project([new_pos])
            if z[0] > 0.05:
                prims.append((float(z[0]) - 0.02, "dot", (P[0], 7, col, tok["surface"])))
            self._text(prims, new_pos, f"{r.new_camera.id} (suggested)", col, True, 10)
        # several cameras on one mount: say so, otherwise their aim lines look like one odd camera
        for (x, y, z), n in mount_counts(plan.cameras).items():
            if n > 1:
                self._text(prims, (x, y, z + 0.6), f"×{n}", tok["text"], True, 11)

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
        if self.canvas.result is not None and self.coverage_mode != "off":
            what = "predicted accuracy" if self.canvas.overlay_mode == "accuracy" else "cameras per point"
            n_bad, vol_bad = self.coverage_stats()
            if self.coverage_mode == "slice":
                z = self.canvas.result[1].grid.zs[self.canvas.overlay_level]
                text = f"Coverage: {what} on the layer at Z = {z:.1f} m (move it with the altitude slider)"
            elif self.coverage_mode == "volume":
                text = f"Coverage: {what} at every evaluation point; failing points are large with a red ring"
            else:
                text = ("Problem points: none, every point meets the requirement" if n_bad == 0 else
                        f"Problem points: {n_bad} failing (about {vol_bad:,.0f} m³), colour = "
                        + ("predicted error" if what == "predicted accuracy" else "cameras seeing it"))
            p.drawText(QRectF(10, 8, self.width() - 20, 20), Qt.AlignmentFlag.AlignLeft, text)

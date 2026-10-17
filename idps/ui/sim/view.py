"""Simulate tab views: the replay in 3D, and one camera's image.

3D: each drone's marker coloured by how far the engine's position is from the truth, a ring
where the engine put it, a short trail, and for the selected drone the lines of sight from the
cameras that have it in frame."""
from __future__ import annotations

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QRadialGradient
from PySide6.QtWidgets import QWidget

from idps.sim.fromplan import marker_in_view
from idps.ui import theme
from idps.ui.planner.view3d import View3D

TRAIL_S = 1.5
NO_FIX = "#8a8984"


def error_colour(err_mm: float, limit_mm: float) -> QColor:
    """Within the requirement: green; up to twice it: amber; worse: red; no position: grey."""
    if not np.isfinite(err_mm):
        return QColor(NO_FIX)
    if err_mm <= limit_mm:
        return QColor(theme.STATUS[3])
    if err_mm <= 2 * limit_mm:
        return QColor(theme.STATUS[2])
    return QColor(theme.STATUS[0])


class SimView3D(View3D):
    def __init__(self, doc, canvas, parent=None):
        super().__init__(doc, canvas, parent)
        self.coverage_mode = "off"
        self.show_cones = "none"
        self.res = None
        self.frame = 0
        self.selected: int | None = None          # drone index
        self.limit_mm = 10.0

    def set_result(self, res, limit_mm: float):
        self.res, self.limit_mm, self.frame = res, limit_mm, 0
        self.selected = None
        self.update()

    def set_frame(self, k: int):
        self.frame = k
        self.update()

    def _extra(self, prims, tok):
        r = self.res
        if r is None:
            return
        k = self.frame
        n_trail = int(TRAIL_S * len(r.t) / max(r.t[-1], 1e-9))
        for i, did in enumerate(r.drone_ids):
            err = r.error_mm[i, k]
            col = error_colour(err, self.limit_mm)
            faint = QColor(col)
            faint.setAlphaF(0.35)
            tr = r.centre[i, max(0, k - n_trail):k + 1]
            for a, b in zip(tr, tr[1:]):
                self._line(prims, a, b, QPen(faint, 1.2))
            if i == self.selected:
                pen = QPen(QColor(tok["volume"]), 1.0)
                pen.setColor(QColor(tok["volume"].red(), tok["volume"].green(), tok["volume"].blue(), 140))
                for st in r.stations:
                    if np.isfinite(marker_in_view(st, r.marker[i, k])[0, 0]):
                        self._line(prims, st.pos, r.marker[i, k], pen)
            P, z = self.project([r.marker[i, k]])
            if z[0] > 0.05:
                big = 7.0 if i == self.selected else 5.0
                prims.append((float(z[0]) - 0.03, "dot", (P[0], big, col, tok["surface"])))
            e = r.engine[i, k]
            if np.isfinite(e).all():
                Pe, ze = self.project([e])
                if ze[0] > 0.05:
                    prims.append((float(ze[0]) - 0.04, "dot", (Pe[0], 9.0, QColor(0, 0, 0, 0), QColor(tok["volume"]))))
            if i == self.selected or len(r.drone_ids) <= 12:
                self._text(prims, r.marker[i, k], str(did), tok["text2"], i == self.selected, 9)

    def _overlay_text(self, p, tok):
        f = QFont(self.font())
        f.setPointSizeF(10)
        p.setFont(f)
        p.setPen(tok["text2"])
        p.drawText(QRectF(10, self.height() - 24, self.width() - 20, 20), Qt.AlignmentFlag.AlignLeft,
                   "Drag: orbit · ⇧: pan · pinch or ⌘-scroll: zoom")
        if self.res is None:
            return
        x = 10
        lim = self.limit_mm
        for text, colour in ((f"● within {lim:g} mm", theme.STATUS[3]), (f"● {lim:g}–{2 * lim:g} mm", theme.STATUS[2]),
                             (f"● over {2 * lim:g} mm", theme.STATUS[0]), ("● no position", NO_FIX),
                             ("○ engine position", tok["volume"].name())):
            p.setPen(QColor(colour))
            p.drawText(QRectF(x, 8, 220, 20), Qt.AlignmentFlag.AlignLeft, text)
            x += p.fontMetrics().horizontalAdvance(text) + 16


class CameraImage(QWidget):
    """What one camera's sensor sees at the replay time: the bright marker dots the engine
    works from."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.res = None
        self.station = 0
        self.frame = 0
        self.selected: int | None = None
        self.setMinimumHeight(140)

    def set_state(self, res, station: int, frame: int, selected=None):
        self.res, self.station, self.frame, self.selected = res, station, frame, selected
        self.update()

    def heightForWidth(self, w):
        return int(w * 0.625)

    def hasHeightForWidth(self):
        return True

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        tok = theme.tokens()
        p.fillRect(self.rect(), tok["surface"])
        r = self.res
        W, H = 1280, 800
        if r is not None and r.stations:
            st = r.stations[self.station]
            W, H = st.W, st.H
        s = min(self.width() / W, self.height() / H)
        img = QRectF((self.width() - W * s) / 2, (self.height() - H * s) / 2, W * s, H * s)
        p.fillRect(img, QColor("#0d1420"))
        p.setPen(QPen(QColor(255, 255, 255, 18), 1))
        for x in range(0, W + 1, 160):
            p.drawLine(QPointF(img.left() + x * s, img.top()), QPointF(img.left() + x * s, img.bottom()))
        for y in range(0, H + 1, 160):
            p.drawLine(QPointF(img.left(), img.top() + y * s), QPointF(img.right(), img.top() + y * s))
        if r is None:
            return
        st = r.stations[self.station]
        uv = marker_in_view(st, r.marker[:, self.frame])
        f = QFont(self.font())
        f.setPointSizeF(8.5)
        p.setFont(f)
        for i, (u, v) in enumerate(uv):
            if not np.isfinite(u):
                continue
            rng = np.linalg.norm(st.pos - r.marker[i, self.frame])
            b = float(min(1.0, r.led[i, self.frame] * (8.0 / max(rng, 0.5)) ** 1.2))
            c = QPointF(img.left() + u * s, img.top() + v * s)
            rad = 3 + 6 * b
            g = QRadialGradient(c, rad)
            g.setColorAt(0, QColor(255, 255, 255, int(120 + 135 * b)))
            g.setColorAt(0.4, QColor(220, 228, 240, int(70 * b)))
            g.setColorAt(1, QColor(0, 0, 0, 0))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(g)
            p.drawEllipse(c, rad, rad)
            p.setPen(QColor(120, 170, 255) if i == self.selected else QColor(200, 210, 225, 200))
            p.drawText(c + QPointF(7, -6), f"#{r.drone_ids[i]}")

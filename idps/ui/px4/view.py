"""PX4 flight tab views: the flight in 3D, and one drone's distance from its file over time."""
from __future__ import annotations

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPixmap
from PySide6.QtWidgets import QWidget

from idps.ui import theme
from idps.ui.planner.view3d import View3D
from idps.ui.sim.view import error_colour

TRAIL_S = 2.0


class FlightView3D(View3D):
    """Each drone where it flew, coloured by its distance from the file; a ring where the file has
    it; a short trail; and for the selected drone the line from where it should be to where it is
    and its path in the file."""

    def __init__(self, doc, canvas, parent=None):
        super().__init__(doc, canvas, parent)
        self.coverage_mode = "off"
        self.show_cones = "none"
        self.run = None
        self.frame = 0
        self.selected: int | None = None
        self.limit_mm = 30.0
        self._plan_w = self._actual_w = None

    def set_run(self, run, limit_mm: float):
        keep = self.run is not None and run is not None and self.run.folder == run.folder
        self.run, self.limit_mm = run, limit_mm
        self._plan_w = run.world(run.plan) if run is not None else None
        self._actual_w = run.world(run.actual) if run is not None else None
        if not keep:
            self.selected = None
        self.update()

    def set_frame(self, k: int):
        self.frame = k
        self.update()

    def _extra(self, prims, tok):
        r = self.run
        if r is None:
            return
        k = min(self.frame, len(r.t) - 1)
        n_trail = int(TRAIL_S * 20)
        for i, did in enumerate(r.drone_ids):
            plan, act = self._plan_w[i], self._actual_w[i]
            err = r.error_mm[i, k]
            col = error_colour(err, self.limit_mm)
            if i == self.selected:                                   # its whole path in the file
                path = QColor(tok["volume"])
                path.setAlpha(90)
                self._polyline(prims, plan[::4], QPen(path, 1.0, Qt.PenStyle.DashLine))
            faint = QColor(col)
            faint.setAlphaF(0.45)
            tr = act[max(0, k - n_trail):k + 1]
            self._polyline(prims, tr, QPen(faint, 1.4))
            P, z = self.project([plan[k]])
            if z[0] > 0.05:
                prims.append((float(z[0]) - 0.04, "dot", (P[0], 8.0, QColor(0, 0, 0, 0), QColor(tok["volume"]))))
            if np.isfinite(act[k]).all():
                if i == self.selected:
                    self._line(prims, plan[k], act[k], QPen(col, 1.5))
                Pa, za = self.project([act[k]])
                if za[0] > 0.05:
                    big = 6.5 if i == self.selected else 5.0
                    prims.append((float(za[0]) - 0.03, "dot", (Pa[0], big, col, tok["surface"])))
            label = f"{did}" + (f"  {err:.0f} mm" if i == self.selected and np.isfinite(err) else "")
            self._text(prims, act[k] if np.isfinite(act[k]).all() else plan[k], label, tok["text2"],
                       i == self.selected, 9)

    def _overlay_text(self, p, tok):
        f = QFont(self.font())
        f.setPointSizeF(10)
        p.setFont(f)
        p.setPen(tok["text2"])
        p.drawText(QRectF(10, self.height() - 24, self.width() - 20, 20), Qt.AlignmentFlag.AlignLeft,
                   "Drag: orbit · ⇧: pan · pinch or ⌘-scroll: zoom")
        if self.run is None:
            return
        x, lim = 10, self.limit_mm
        for text, colour in ((f"● within {lim:g} mm of the file", theme.STATUS[3]),
                             (f"● {lim:g}–{2 * lim:g} mm", theme.STATUS[2]), (f"● over {2 * lim:g} mm", theme.STATUS[0]),
                             ("○ where the file has it", tok["volume"].name())):
            p.setPen(QColor(colour))
            p.drawText(QRectF(x, 8, 260, 20), Qt.AlignmentFlag.AlignLeft, text)
            x += p.fontMetrics().horizontalAdvance(text) + 16


class ErrorGraph(QWidget):
    """Distance from the file over the show: the selected drone bold, the others faint, the limit
    dashed, and the replay time as a cursor."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.run, self.selected, self.frame, self.limit_mm = None, None, 0, 30.0
        self.setMinimumHeight(130)
        self._plot = None               # the axes and curves, drawn once per run / selection / size
        self._plot_key = None

    def set_state(self, run, selected, frame, limit_mm):
        self.run, self.selected, self.frame, self.limit_mm = run, selected, frame, limit_mm
        self.update()

    L, R, T, B = 40, 8, 8, 18           # margins around the plot

    def _scales(self):
        r = self.run
        w, h = self.width() - self.L - self.R, self.height() - self.T - self.B
        fin = r.error_mm[np.isfinite(r.error_mm)]
        top = max(2.0 * self.limit_mm, float(fin.max()) * 1.05 if len(fin) else 0.0)
        return w, h, top, r.t[0], r.t[-1]

    def _draw_plot(self, tok) -> QPixmap:
        """Axes and every drone's curve. A flight has thousands of samples per drone and the widget
        only a thousand or so pixel columns, so each curve is drawn as its lowest and highest value
        per column. Drawing every sample as its own line on every frame took over 100 ms and made
        the live view jerky."""
        r = self.run
        L, T = self.L, self.T
        w, h, top, t0, t1 = self._scales()
        dpr = self.devicePixelRatioF()
        pm = QPixmap(int(self.width() * dpr), int(self.height() * dpr))
        pm.setDevicePixelRatio(dpr)
        pm.fill(tok["surface"])
        p = QPainter(pm)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        f = QFont(self.font())
        f.setPointSizeF(9)
        p.setFont(f)
        Y = lambda e: T + h * (1 - min(e, top) / top)                                       # noqa: E731
        p.setPen(QPen(tok["text2"], 0.6))
        p.drawLine(QPointF(L, T + h), QPointF(L + w, T + h))
        for v in (0, self.limit_mm, top):
            p.drawText(QRectF(0, Y(v) - 8, L - 4, 16), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       f"{v:.0f}")
        p.drawText(QRectF(L, T + h + 2, w, 16), Qt.AlignmentFlag.AlignRight, "mm  ·  show time →")
        p.setPen(QPen(QColor(theme.STATUS[0]), 1, Qt.PenStyle.DashLine))
        p.drawLine(QPointF(L, Y(self.limit_mm)), QPointF(L + w, Y(self.limit_mm)))
        cols = max(int(w), 2)
        col = np.clip(((r.t - t0) / max(t1 - t0, 1e-9) * (cols - 1)).astype(int), 0, cols - 1)
        xs = L + w * np.arange(cols) / (cols - 1)
        order = [i for i in range(len(r.drone_ids)) if i != self.selected] + \
                ([self.selected] if self.selected is not None else [])
        for i in order:
            e = np.minimum(r.error_mm[i], top)
            ok = np.isfinite(e)
            lo, hi = np.full(cols, np.inf), np.full(cols, -np.inf)
            np.minimum.at(lo, col[ok], e[ok])
            np.maximum.at(hi, col[ok], e[ok])
            colour = QColor(tok["volume"]) if i == self.selected else QColor(tok["text2"])
            colour.setAlpha(255 if i == self.selected else 70)
            p.setPen(QPen(colour, 1.6 if i == self.selected else 0.8))
            have = np.isfinite(lo)
            ylo, yhi = T + h * (1 - lo / top), T + h * (1 - hi / top)
            line = []
            for c in range(cols):
                if have[c]:
                    line.append(QPointF(xs[c], ylo[c]))
                    if hi[c] > lo[c]:
                        line.append(QPointF(xs[c], yhi[c]))
                elif line:                                  # a gap (no samples here): the curve breaks
                    p.drawPolyline(line)
                    line = []
            if line:
                p.drawPolyline(line)
        p.end()
        return pm

    def paintEvent(self, _):
        p = QPainter(self)
        tok = theme.tokens()
        r = self.run
        if r is None or not len(r.t):
            p.fillRect(self.rect(), tok["surface"])
            f = QFont(self.font())
            f.setPointSizeF(9)
            p.setFont(f)
            p.setPen(tok["text2"])
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Distance from the file over the show")
            return
        key = (self.selected, self.width(), self.height(), self.limit_mm, tok["surface"].name(), tok["text2"].name())
        if self._plot is None or self._plot_key is None or self._plot_key[0] is not r or self._plot_key[1] != key:
            self._plot, self._plot_key = self._draw_plot(tok), (r, key)
        p.drawPixmap(0, 0, self._plot)
        w, h, top, t0, t1 = self._scales()
        k = min(self.frame, len(r.t) - 1)
        x = self.L + w * (r.t[k] - t0) / max(t1 - t0, 1e-9)
        p.setPen(QPen(tok["text"], 1))
        p.drawLine(QPointF(x, self.T), QPointF(x, self.T + h))

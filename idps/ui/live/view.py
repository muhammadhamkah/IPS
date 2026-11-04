"""3D view of the live show: drones coloured by how many cameras see them, faulty
cameras marked, and the brain's problem regions drawn where they are."""
from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPen

from idps.monitor.model import BLIND_SPOT, LIVED_WITH, RESOLVED
from idps.ui import theme
from idps.ui.planner.view3d import View3D

TRAIL = 25


def drone_colour(n: int, need: int = 3) -> QColor:
    if n >= need:
        return QColor(theme.STATUS[3])
    if n == need - 1:
        return QColor(theme.STATUS[2])
    return QColor(theme.STATUS[0])


class LiveView3D(View3D):
    def __init__(self, doc, canvas, parent=None):
        super().__init__(doc, canvas, parent)
        self.coverage_mode = "off"
        self.show_cones = "none"
        self.drones: dict[int, tuple] = {}        # id -> (pos, n_cams)
        self.trails: dict[int, list] = {}
        self.offline: set = set()
        self.moved: set = set()
        self.issues: list = []
        self.occluders: list = []                 # simulation only: the injected obstructions
        self.highlight: str | None = None         # issue id being shown
        self.need = 3

    def set_state(self, drones, offline, moved, issues, occluders):
        self.drones = drones
        for did, (p, _) in drones.items():
            tr = self.trails.setdefault(did, [])
            tr.append(np.asarray(p, float))
            del tr[:-TRAIL]
        for did in list(self.trails):
            if did not in drones:
                self.trails.pop(did)
        self.offline, self.moved, self.issues, self.occluders = set(offline), set(moved), list(issues), list(occluders)
        self.update()

    def clear(self):
        self.drones, self.trails, self.issues, self.occluders = {}, {}, [], []
        self.offline, self.moved = set(), set()
        self.update()

    # ------------------------------------------------------------- layers
    def _extra(self, prims, tok):
        self._obstructions(prims, tok)
        self._issue_regions(prims, tok)
        self._camera_faults(prims, tok)
        self._drones(prims, tok)

    def _drones(self, prims, tok):
        for did, (p, n) in self.drones.items():
            col = drone_colour(n, self.need)
            tr = self.trails.get(did, [])
            faint = QColor(col)
            faint.setAlphaF(0.35)
            for a, b in zip(tr, tr[1:]):
                self._line(prims, a, b, QPen(faint, 1.2))
            P, z = self.project([p])
            if z[0] > 0.05:
                prims.append((float(z[0]) - 0.03, "dot", (P[0], 5.0, col, tok["surface"])))

    def _camera_faults(self, prims, tok):
        if self.canvas.result is None:
            return
        for c in self.canvas.result[0].cameras:
            if c.id in self.offline:
                col, text = QColor(theme.STATUS[0]), f"{c.id} offline"
            elif c.id in self.moved:
                col, text = QColor(theme.STATUS[2]), f"{c.id} knocked"
            else:
                continue
            P, z = self.project([c.position_m])
            if z[0] > 0.05:
                prims.append((float(z[0]) - 0.05, "dot", (P[0], 9.0, QColor(0, 0, 0, 0), col)))
                prims.append((float(z[0]) - 0.051, "dot", (P[0], 6.0, col, col)))
            self._text(prims, c.position_m, text, col, True, 10)

    def _ring(self, prims, centre, r, axis, pen, n=36):
        a = np.linspace(0, 2 * math.pi, n + 1)
        u, v = {0: ((0, 1, 0), (0, 0, 1)), 1: ((1, 0, 0), (0, 0, 1)), 2: ((1, 0, 0), (0, 1, 0))}[axis]
        pts = [centre + r * (math.cos(t) * np.array(u) + math.sin(t) * np.array(v)) for t in a]
        for p, q in zip(pts, pts[1:]):
            self._line(prims, p, q, pen)

    def _issue_regions(self, prims, tok):
        for iss in self.issues:
            if iss.kind != BLIND_SPOT or iss.location is None or iss.state == RESOLVED:
                continue
            if iss.state == LIVED_WITH:
                col = QColor(tok["text2"])
            else:
                col = QColor(theme.STATUS[0] if iss.severity == "critical" else theme.STATUS[2])
            hl = iss.id == self.highlight
            pen = QPen(col, 2.4 if hl else 1.4, Qt.PenStyle.SolidLine if hl else Qt.PenStyle.DashLine)
            c = np.asarray(iss.location, float)
            for axis in (0, 1, 2):
                self._ring(prims, c, iss.radius_m, axis, pen)
            self._text(prims, c + (0, 0, iss.radius_m + 0.4), f"{iss.id} tracking loss", col, True, 10)

    def _obstructions(self, prims, tok):
        col = QColor(tok["text2"])
        pen = QPen(col, 1.0, Qt.PenStyle.DotLine)
        fill = QColor(col)
        fill.setAlphaF(0.12)
        for lo, hi, _ in self.occluders:
            lo, hi = np.asarray(lo), np.asarray(hi)
            V = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
            for i, j in ((0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)):
                self._line(prims, V[i], V[j], pen)
            self._poly(prims, [V[1], V[3], V[7], V[5]], QPen(Qt.PenStyle.NoPen), fill)   # top face

    def _overlay_text(self, p, tok):
        f = QFont(self.font())
        f.setPointSizeF(10)
        p.setFont(f)
        p.setPen(tok["text2"])
        p.drawText(QRectF(10, self.height() - 24, self.width() - 20, 20), Qt.AlignmentFlag.AlignLeft,
                   "Drag: orbit · ⇧: pan · pinch or ⌘-scroll: zoom")
        if not self.drones:
            return
        lost = sum(n < 2 for _, n in self.drones.values())
        weak = sum(2 <= n < self.need for _, n in self.drones.values())
        x = 10
        for text, colour in ((f"● {len(self.drones) - lost - weak} tracked (≥{self.need} cameras)", theme.STATUS[3]),
                             (f"● {weak} weak (2 cameras)", theme.STATUS[2]),
                             (f"● {lost} lost (<2)", theme.STATUS[0])):
            p.setPen(QColor(colour))
            p.drawText(QRectF(x, 8, 260, 20), Qt.AlignmentFlag.AlignLeft, text)
            x += p.fontMetrics().horizontalAdvance(text) + 18
        if self.occluders:
            p.setPen(tok["text2"])
            p.drawText(QRectF(x, 8, 400, 20), Qt.AlignmentFlag.AlignLeft,
                       "dotted boxes: simulated obstructions")

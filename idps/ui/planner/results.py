"""Bottom panel: planning progress, summary, coverage overlay controls, camera table."""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (QAbstractItemView, QButtonGroup, QFrame, QHBoxLayout, QHeaderView, QLabel,
                               QProgressBar, QPushButton, QSizePolicy, QSlider, QTableWidget, QTableWidgetItem,
                               QVBoxLayout, QWidget)

from idps.ui import theme
from idps.ui.flow import flow_row

COLUMNS = ["ID", "Mount", "N (m)", "E (m)", "D (m)", "Base heading", "Pan", "Tilt", "Lens", "Spacing",
           "Critical", "Install note"]


class Tile(QFrame):
    def __init__(self, title: str):
        super().__init__()
        self.setFrameShape(QFrame.Shape.StyledPanel)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 6, 10, 6)
        lay.setSpacing(0)
        self.t = QLabel(title)
        self.t.setStyleSheet("color: palette(placeholder-text); font-size: 11px;")
        self.v = QLabel("–")
        f = self.v.font()
        f.setPointSizeF(f.pointSizeF() * 1.45)
        f.setBold(True)
        self.v.setFont(f)
        self.s = QLabel("")
        self.s.setStyleSheet("color: palette(placeholder-text); font-size: 11px;")
        for w in (self.t, self.v, self.s):
            lay.addWidget(w)

    def set(self, value: str, sub: str = "", ok: bool | None = None):
        self.v.setText(value)
        self.s.setText(sub)
        colour = "" if ok is None else ("color: #0ca30c;" if ok else "color: #d03b3b;")
        self.v.setStyleSheet(colour)


class ResultsPanel(QWidget):
    runRequested = Signal()
    cancelRequested = Signal()
    exportRequested = Signal()
    reportRequested = Signal()
    cameraSelected = Signal(int)

    def __init__(self, canvas, parent=None):
        super().__init__(parent)
        self.canvas = canvas
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)

        # --- run row
        self.run_btn = QPushButton("Plan cameras")
        self.run_btn.setDefault(True)
        self.run_btn.setToolTip("Find camera count, positions and pan/tilt (⌘R)")
        self.run_btn.clicked.connect(self.runRequested)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.clicked.connect(self.cancelRequested)
        self.cancel_btn.hide()
        self.progress = QProgressBar()
        self.progress.setRange(0, 0)
        self.progress.setMaximumWidth(160)
        self.progress.hide()
        self.status = QLabel("Draw the volume, add trusses or tripod zones, then plan.")
        self.status.setStyleSheet("color: palette(placeholder-text);")
        self.stale = QLabel("Plan is out of date: the site changed since it was computed.")
        self.stale.setStyleSheet("color: #b8860b;")
        self.stale.hide()
        self.export_btn = QPushButton("Export plan…")
        self.export_btn.setToolTip("Save plan.yaml, cameras.csv and report.html to a folder (⌘E)")
        self.export_btn.clicked.connect(self.exportRequested)
        self.report_btn = QPushButton("Open report")
        self.report_btn.clicked.connect(self.reportRequested)
        for b in (self.export_btn, self.report_btn):
            b.setEnabled(False)
        row_box, row = flow_row(self.run_btn, self.cancel_btn, self.progress, self.export_btn, self.report_btn)
        lay.addWidget(row_box)
        for w in (self.status, self.stale):        # long messages wrap instead of widening the window
            w.setWordWrap(True)
            w.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            lay.addWidget(w)

        # --- summary tiles
        tiles_box, tiles = flow_row()
        self.t_cams = Tile("Cameras")
        self.t_meets = Tile("Meets requirement")
        self.t_cov = Tile("≥ N cameras")
        self.t_n1 = Tile("Survives a lost camera")
        self.t_acc = Tile("Predicted p95 error")
        for t in (self.t_cams, self.t_meets, self.t_cov, self.t_n1, self.t_acc):
            t.setMinimumWidth(170)
            tiles.addWidget(t)
        lay.addWidget(tiles_box)
        # dense re-check of the finished layout, between the planning grid's samples
        self.fine = QLabel("")
        self.fine.setObjectName("fine_check")
        self.fine.setWordWrap(True)
        self.fine.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        self.fine.setToolTip("The planner samples the volume every few metres. Coverage only changes where a "
                             "point crosses the edge of a camera's view, so after planning the layout is "
                             "re-checked on a much finer grid to catch any gap between those samples.")
        self.fine.hide()
        lay.addWidget(self.fine)

        # --- overlay controls
        orow_box, orow = flow_row()
        orow.addWidget(QLabel("Show on plan:"))
        self.mode_group = QButtonGroup(self)
        for i, (key, label) in enumerate((("none", "Layout only"), ("count", "Cameras per point"),
                                          ("accuracy", "Predicted accuracy"))):
            b = QPushButton(label)
            b.setCheckable(True)
            b.setChecked(key == "count")
            b.setProperty("mode", key)
            self.mode_group.addButton(b, i)
            orow.addWidget(b)
        self.mode_group.buttonClicked.connect(self._mode)
        orow.addSpacing(16)
        orow.addWidget(QLabel("Altitude:"))
        self.level = QSlider(Qt.Orientation.Horizontal)
        self.level.setMinimumWidth(180)
        self.level.valueChanged.connect(self._level)
        self.level_label = QLabel("–")
        self.level_label.setMinimumWidth(70)
        orow.addWidget(self.level)
        orow.addWidget(self.level_label)
        self.legend = QLabel("")
        orow.addSpacing(16)
        orow.addWidget(self.legend)
        lay.addWidget(orow_box)

        # --- camera table
        self.table = QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.verticalHeader().hide()
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setAlternatingRowColors(True)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.itemSelectionChanged.connect(self._row)
        lay.addWidget(self.table, 1)
        self._legend()

    # ------------------------------------------------------------------
    def running(self, on: bool):
        self.run_btn.setEnabled(not on)
        self.cancel_btn.setVisible(on)
        self.progress.setVisible(on)
        if on:
            self.stale.hide()

    def log(self, text: str):
        self.status.setText(text.strip())

    def mark_stale(self, stale: bool):
        self.stale.setVisible(stale)

    def show_result(self, plan, ev):
        s = ev.summary
        req = s["requirements"]
        fc = s.get("fine_check") or plan.summary.get("fine_check")
        if fc:
            from idps.plan.planner import fine_check_text
            self.fine.setText(("✅ " if not fc["failing"] else "⚠️ ") + fine_check_text(fc))
            self.fine.setStyleSheet("" if not fc["failing"] else "color: #b8860b;")
        self.fine.setVisible(bool(fc))

        def pct(v):
            return f"{100 * v:.1f} %"
        self.t_cams.set(str(s["cameras"]), f"{s['cameras_on_trusses']} on trusses · {s['cameras_on_tripods']} on "
                                           f"{s['tripods']} tripods")
        meets = s["fraction_meeting_all_requirements"]
        self.t_meets.set(pct(meets), f"≥ {req['min_cameras']} cameras and ≤ {req['p95_mm']:g} mm", meets >= 0.9999)
        self.t_cov.title = f"≥ {req['min_cameras']} cameras"
        self.t_cov.t.setText(f"≥ {req['min_cameras']} cameras")
        self.t_cov.set(pct(s["fraction_with_min_cameras"]),
                       f"{s['volume_below_min_cameras_m3']:,.0f} m³ below {req['min_cameras']}",
                       s["fraction_with_min_cameras"] >= 0.9999)
        self.t_n1.set(pct(s["fraction_meeting_with_any_one_camera_lost"]), "points still meeting it")
        med, worst = s.get("p95_error_mm_median"), s.get("p95_error_mm_worst")
        self.t_acc.set(f"{med:.1f} mm" if med is not None else "n/a",
                       f"median · worst {worst:.1f} mm" if worst is not None else "worst: unbounded")
        self.export_btn.setEnabled(True)
        self.report_btn.setEnabled(True)
        zs = ev.grid.zs
        self.level.blockSignals(True)
        self.level.setRange(0, len(zs) - 1)
        self.level.setValue(min(self.canvas.overlay_level, len(zs) - 1))
        self.level.blockSignals(False)
        self._level(self.level.value())
        self.table.setRowCount(len(plan.cameras))
        for r, c in enumerate(plan.cameras):
            nf = plan.ned_fields(c)
            n, e, d = nf["ned_m"]
            vals = [c.id, f"{c.mount_type} {c.mount_id}", f"{n:.2f}", f"{e:.2f}", f"{d:.2f}",
                    f"{nf['base_heading_from_north_deg']:.0f}°", f"{nf['pan_clockwise_deg']:+.1f}°",
                    f"{c.tilt_deg:+.1f}°", f"{c.lens_mm:g} mm",
                    "" if c.neighbour_spacing_m is None else f"{c.neighbour_spacing_m:.2f} m",
                    str(int(ev.critical[r])), c.install_note]
            for col, v in enumerate(vals):
                it = QTableWidgetItem(v)
                if 2 <= col <= 10:
                    it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.table.setItem(r, col, it)
        opt = plan.summary.get("optimiser", {})
        self.status.setText(f"Planned in {opt.get('seconds', 0):.1f} s from {opt.get('candidates', 0):,} "
                            f"candidate placements ({opt.get('stopped_because', '')}).")
        self.mark_stale(False)

    def select_camera(self, k):
        self.table.blockSignals(True)
        self.table.selectRow(k)
        self.table.blockSignals(False)

    def _row(self):
        r = self.table.currentRow()
        if r >= 0:
            self.cameraSelected.emit(r)

    def _mode(self, b):
        self.canvas.overlay_mode = b.property("mode")
        self.canvas.update_overlay()
        self._legend()

    def _level(self, k):
        self.canvas.overlay_level = k
        if self.canvas.result is not None:
            z = self.canvas.result[1].grid.zs[k]
            self.level_label.setText(f"{z:.1f} m high (D {-z:.1f})")
        self.canvas.update_overlay()

    def _legend(self):
        mode = self.canvas.overlay_mode
        if mode == "count":
            items = [(theme.STATUS[0], "0"), (theme.STATUS[1], "1"), (theme.STATUS[2], "2"), (theme.STATUS[3], "3+")]
            html = " ".join(f'<span style="color:{c}">■</span> {t}' for c, t in items) + " cameras"
        elif mode == "accuracy":
            html = (" ".join(f'<span style="color:{c}">■</span>' for c in theme.BLUE_RAMP)
                    + " 0 → 2× limit (edge points outlined red if failing)")
        else:
            html = ""
        self.legend.setText(html)


def colour_hex(c: QColor) -> str:
    return c.name()

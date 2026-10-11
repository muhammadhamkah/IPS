"""Right-hand panel: properties of the selected item, and venue/requirement settings.

Every edit goes through SiteDocument.edit, so it is undoable (Cmd+Z)."""
from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout,
                               QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QPushButton,
                               QScrollArea, QSizePolicy, QSpinBox, QTableWidget, QTableWidgetItem,
                               QTabWidget, QVBoxLayout, QWidget)

from idps.core.noise import PRESETS


class _NoStrayWheel:
    """Number fields ignore scroll/trackpad gestures unless clicked into first,
    so scrolling the inspector never silently changes a value (a classic Qt
    pitfall that let the volume height drift to 0)."""

    def _setup(self):
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def wheelEvent(self, e):
        if self.hasFocus():
            super().wheelEvent(e)
        else:
            e.ignore()                   # let the scroll area scroll instead


class DSpin(_NoStrayWheel, QDoubleSpinBox):
    def __init__(self):
        super().__init__()
        self._setup()


class ISpin(_NoStrayWheel, QSpinBox):
    def __init__(self):
        super().__init__()
        self._setup()


class Combo(_NoStrayWheel, QComboBox):
    def __init__(self):
        super().__init__()
        self._setup()


def dspin(lo, hi, dec=2, suffix=" m", step=0.5) -> QDoubleSpinBox:
    w = DSpin()
    w.setRange(lo, hi)
    w.setDecimals(dec)
    w.setSingleStep(step)
    w.setSuffix(suffix)
    w.setKeyboardTracking(False)
    w.setMinimumWidth(110)
    return w


def ispin(lo, hi) -> QSpinBox:
    w = ISpin()
    w.setRange(lo, hi)
    w.setKeyboardTracking(False)
    return w


MIN_HEIGHT_M = 0.5


def sync_z_editors(doc, z0, z1):
    """Show the document's heights without emitting edits."""
    z = doc.data["volume"]["z"]
    for w in (z0, z1):
        w.blockSignals(True)
    z0.setMaximum(500)
    z1.setMinimum(0)
    z0.setValue(z[0])
    z1.setValue(z[1])
    z1.setMinimum(z[0] + MIN_HEIGHT_M)
    z0.setMaximum(max(z[1] - MIN_HEIGHT_M, z0.minimum()))
    for w in (z0, z1):
        w.blockSignals(False)


def volume_z_editors(doc):
    """Floor Z / Top Z spin boxes bound to the document; top is kept at least
    MIN_HEIGHT_M above the floor."""
    z0, z1 = dspin(-50, 500), dspin(0, 500)
    z0.setToolTip("Floor of the flight volume (minimum Z), usually 0 m")
    z1.setToolTip("Top of the flight volume (maximum Z): the height the outline is extruded to")
    sync_z_editors(doc, z0, z1)

    def set_floor():
        v = z0.value()
        if v != doc.data["volume"]["z"][0]:
            doc.edit("Volume floor", lambda d: d["volume"]["z"].__setitem__(0, v))

    def set_top():
        v = z1.value()
        if v != doc.data["volume"]["z"][1]:
            doc.edit("Volume height", lambda d: d["volume"]["z"].__setitem__(1, v))
    z0.editingFinished.connect(set_floor)
    z1.editingFinished.connect(set_top)
    return z0, z1


def parse_floats(text: str) -> list[float]:
    return [float(t) for t in text.replace(";", ",").replace(" ", ",").split(",") if t.strip()]


def fmt_floats(v) -> str:
    return ", ".join(f"{x:g}" for x in v)


def parse_segments(text: str):
    """'all' -> None; '0-20, 34-54' -> [[0, 20], [34, 54]] (metres along the truss)."""
    t = text.strip().lower()
    if t in ("", "all"):
        return None
    out = []
    for part in t.split(","):
        a, b = part.split("-")
        a, b = float(a), float(b)
        if b <= a:
            raise ValueError(part)
        out.append([a, b])
    return out


class VertexTable(QTableWidget):
    """Editable X/Y vertex list with edge lengths."""

    def __init__(self, on_edit, parent=None):
        super().__init__(0, 3, parent)
        self.on_edit = on_edit
        self.setHorizontalHeaderLabels(["X (m)", "Y (m)", "edge to next"])
        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.setAlternatingRowColors(True)
        self.setMinimumHeight(170)
        self._loading = False
        self.itemChanged.connect(self._changed)

    def load(self, pts, prefix="V"):
        self._loading = True
        self.setRowCount(len(pts))
        self.setVerticalHeaderLabels([f"{prefix}{k}" for k in range(len(pts))])
        for k, (x, y) in enumerate(pts):
            self.setItem(k, 0, QTableWidgetItem(f"{x:.3f}"))
            self.setItem(k, 1, QTableWidgetItem(f"{y:.3f}"))
            if len(pts) >= 2:
                nx, ny = pts[(k + 1) % len(pts)]
                it = QTableWidgetItem(f"E{k}: {np.hypot(nx - x, ny - y):.2f} m")
            else:
                it = QTableWidgetItem("")
            it.setFlags(Qt.ItemFlag.ItemIsEnabled)
            self.setItem(k, 2, it)
        self._loading = False

    def _changed(self, item):
        if self._loading or item.column() > 1:
            return
        try:
            v = float(item.text())
        except ValueError:
            return
        self.on_edit(item.row(), item.column(), v)


class Inspector(QTabWidget):
    def __init__(self, doc, canvas, parent=None):
        super().__init__(parent)
        self.doc, self.canvas = doc, canvas
        self.setDocumentMode(True)
        self.sel_page = QWidget()
        self.sel_layout = QVBoxLayout(self.sel_page)
        # No AlignTop here: with it, rows get their (underestimated) size hint and overlap;
        # a trailing stretch keeps content at the top while honouring minimum heights.
        self.addTab(self._scroll(self.sel_page), "Inspector")
        self.settings = SettingsPage(doc)
        self.addTab(self._scroll(self.settings), "Venue && requirements")
        self.setMinimumWidth(340)
        canvas.selectionChanged.connect(lambda _: self.refresh())
        doc.changed.connect(self.refresh)
        self.refresh()

    @staticmethod
    def _scroll(w):
        s = QScrollArea()
        s.setWidget(w)
        s.setWidgetResizable(True)
        s.setFrameShape(QScrollArea.Shape.NoFrame)
        s.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)   # content fits the panel width
        return s

    def _clear(self):
        while self.sel_layout.count():
            w = self.sel_layout.takeAt(0).widget()          # stretch items have no widget
            if w is not None:
                w.hide()                 # deleteLater is deferred; hide now so panels never overlap
                w.deleteLater()

    def refresh(self):
        if self.canvas.drag is not None:
            return                       # don't rebuild widgets mid-drag
        self._clear()
        sel = self.canvas.selection
        d = self.doc.data
        kind = sel[0] if sel else None
        if kind in (None, "volume", "vertex"):
            self.sel_layout.addWidget(self._volume_box(sel[1] if kind == "vertex" else None))
            self.sel_layout.addWidget(self._add_box())
        elif kind == "truss" and sel[1] < len(d["trusses"]):
            self.sel_layout.addWidget(self._truss_box(sel[1]))
        elif kind == "zone" and sel[1] < len(d["tripod_zones"]):
            self.sel_layout.addWidget(self._zone_box(sel[1]))
        elif kind == "nogo" and sel[1] < len(d["no_go_zones"]):
            self.sel_layout.addWidget(self._nogo_box(sel[1]))
        elif kind == "camera":
            self.sel_layout.addWidget(self._camera_box(sel[1]))
        self.sel_layout.addWidget(self._mounts_box())
        self.sel_layout.addStretch(1)
        self.settings.refresh()

    # ---------------------------------------------------------------- volume
    def _volume_box(self, vertex):
        d = self.doc.data
        vol = d["volume"]
        box = QGroupBox("Flight volume")
        form = QFormLayout(box)
        z0, z1 = volume_z_editors(self.doc)
        form.addRow("Floor Z (min)", z0)
        form.addRow("Top Z (max, extrude to)", z1)
        pts = vol["polygon"]
        if len(pts) >= 3:
            from idps.coverage.volume import Volume
            v = Volume(np.array(pts, float), vol["z"][0], max(vol["z"][1], vol["z"][0] + 1e-3))
            perim = float(v.edge_lengths().sum())
            bb = v.bbox_max - v.bbox_min
            for label, text in (("Footprint", f"{bb[0]:.2f} × {bb[1]:.2f} m (bounding box)"),
                                ("Floor area", f"{v.floor_area_m2:,.1f} m², perimeter {perim:.1f} m"),
                                ("Volume", f"{v.volume_m3:,.0f} m³")):
                lab = QLabel(text)
                lab.setWordWrap(True)
                form.addRow(label, lab)
        table = VertexTable(self._edit_vertex)
        table.load(pts)
        if vertex is not None and vertex < len(pts):
            table.selectRow(vertex)
        table.itemSelectionChanged.connect(
            lambda: self.canvas.set_selection(("vertex", table.currentRow())) if table.currentRow() >= 0 else None)
        form.addRow(table)
        row = QHBoxLayout()
        add, rem = QPushButton("Add vertex"), QPushButton("Remove vertex")
        add.clicked.connect(lambda: self._add_vertex(table.currentRow()))
        rem.clicked.connect(lambda: self._remove_vertex(table.currentRow()))
        rem.setEnabled(len(pts) > 3)
        row.addWidget(add)
        row.addWidget(rem)
        form.addRow(row)
        hint = QLabel("Draw with the Volume tool (P), drag vertices on the canvas, or type exact "
                      "coordinates here. Origin (0, 0) is where the calibration square goes; +X is "
                      "its X arm.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: palette(placeholder-text);")
        form.addRow(hint)
        return box

    def _edit_vertex(self, row, col, value):
        self.doc.edit("Edit vertex", lambda d: d["volume"]["polygon"][row].__setitem__(col, value))

    def _add_vertex(self, after):
        pts = self.doc.data["volume"]["polygon"]
        if len(pts) < 2:
            return

        def add(d):
            p = d["volume"]["polygon"]
            k = after if 0 <= after < len(p) else len(p) - 1
            a, b = np.array(p[k]), np.array(p[(k + 1) % len(p)])
            p.insert(k + 1, [round(float(x), 3) for x in (a + b) / 2])
        self.doc.edit("Add vertex", add)
        self.canvas.set_selection(("vertex", (after if after >= 0 else len(pts) - 1) + 1))

    def _remove_vertex(self, row):
        if row >= 0 and len(self.doc.data["volume"]["polygon"]) > 3:
            self.doc.edit("Remove vertex", lambda d: d["volume"]["polygon"].pop(row))

    def _add_box(self):
        box = QGroupBox("Add")
        lay = QVBoxLayout(box)
        strip = QPushButton("Tripod strip around the volume…")
        strip.setToolTip("Tripods may stand in a band outside the drawn outline")
        strip.clicked.connect(self._add_strip)
        strip.setEnabled(len(self.doc.data["volume"]["polygon"]) >= 3)
        lay.addWidget(strip)
        tips = QLabel("Trusses: T tool · Tripod areas: A tool · No-go areas: N tool")
        tips.setStyleSheet("color: palette(placeholder-text);")
        lay.addWidget(tips)
        return box

    def _delete_button(self, kind, idx):
        b = QPushButton(f"Delete {self.canvas.item_name(kind, idx)}")
        b.setToolTip("Remove it from the venue (⌫ on the canvas; ⌘Z to undo)")
        b.clicked.connect(lambda: self.canvas.delete_item(kind, idx))
        return b

    def _mounts_box(self):
        """Every truss, tripod zone and no-go area with one-click remove."""
        d = self.doc.data
        box = QGroupBox("Mounts && areas")
        lay = QVBoxLayout(box)
        rows = [("truss", i, f"Truss {t['id']}", f"{t['height_m']:g} m high") for i, t in enumerate(d["trusses"])]
        for i, z in enumerate(d["tripod_zones"]):
            what = (f"strip {z['offset_m'][0]:g}–{z['offset_m'][1]:g} m out" if "offset_m" in z else "drawn area")
            rows.append(("zone", i, f"Tripods {z['id']}", f"{what}, masts {z['height_m'][0]:g}–{z['height_m'][1]:g} m"))
        rows += [("nogo", i, f"No-go area {i + 1}", "") for i in range(len(d["no_go_zones"]))]
        if not rows:
            empty = QLabel("No trusses or tripod zones yet.")
            empty.setStyleSheet("color: palette(placeholder-text);")
            lay.addWidget(empty)
        sel = self.canvas.selection
        for kind, idx, name, detail in rows:
            row = QHBoxLayout()
            pick = QPushButton(name)
            pick.setFlat(True)
            pick.setStyleSheet("text-align: left;" + ("font-weight: bold;" if sel == (kind, idx) else ""))
            pick.setToolTip(f"Select {name}" + (f" ({detail})" if detail else ""))
            pick.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)   # shrink, never overflow
            pick.clicked.connect(lambda _=False, k=kind, i=idx: self.canvas.set_selection((k, i)))
            rm = QPushButton("Remove")
            rm.setToolTip(f"Remove {name} (⌘Z to undo)")
            rm.setObjectName(f"remove_{kind}_{idx}")
            rm.clicked.connect(lambda _=False, k=kind, i=idx: self.canvas.delete_item(k, i))
            row.addWidget(pick, 1)
            row.addWidget(rm)
            lay.addLayout(row)
        bulk = QHBoxLayout()
        all_t = QPushButton("Remove all trusses")
        all_t.setEnabled(bool(d["trusses"]))
        all_t.clicked.connect(lambda: self.doc.edit("Remove all trusses", lambda dd: dd["trusses"].clear()))
        all_z = QPushButton("Remove all tripod zones")
        all_z.setEnabled(bool(d["tripod_zones"]))
        all_z.clicked.connect(lambda: self.doc.edit("Remove all tripod zones", lambda dd: dd["tripod_zones"].clear()))
        for b in (all_t, all_z):
            b.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            bulk.addWidget(b)
        lay.addLayout(bulk)
        return box

    def _add_strip(self):
        def add(d):
            n = len(d["tripod_zones"]) + 1
            d["tripod_zones"].append({"id": f"strip{n}", "offset_m": [1.0, 3.0], "height_m": [2.0, 20.0]})
        self.doc.edit("Add tripod strip", add)
        self.canvas.set_selection(("zone", len(self.doc.data["tripod_zones"]) - 1))

    # ---------------------------------------------------------------- truss
    def _truss_box(self, i):
        t = self.doc.data["trusses"][i]
        box = QGroupBox(f"Truss {t['id']}")
        form = QFormLayout(box)
        name = QLineEdit(t["id"])
        name.editingFinished.connect(lambda: self._set(("trusses", i, "id"), name.text().strip() or t["id"]))
        form.addRow("Name", name)
        h = dspin(0, 500)
        h.setValue(t["height_m"])
        h.editingFinished.connect(lambda: self._set(("trusses", i, "height_m"), h.value()))
        form.addRow("Height", h)
        for key, label in (("start", "Start"), ("end", "End")):
            row = QHBoxLayout()
            for c in range(2):
                s = dspin(-10000, 10000)
                s.setPrefix("XY"[c] + " ")
                s.setValue(t[key][c])
                s.editingFinished.connect(lambda s=s, key=key, c=c: self._set(("trusses", i, key, c), s.value()))
                row.addWidget(s)
            form.addRow(label, row)
        L = float(np.hypot(*(np.array(t["end"]) - np.array(t["start"]))))
        form.addRow("Length", QLabel(f"{L:.2f} m"))
        usable = t.get("usable")
        seg = QLineEdit("all" if usable in (None, "all") else ", ".join(f"{a:g}-{b:g}" for a, b in usable))
        seg.setPlaceholderText("all   or   0-20, 34-54")
        seg.setToolTip("Usable sections in metres from the start end; gaps are missing truss")

        def set_seg():
            try:
                v = parse_segments(seg.text())
            except ValueError:
                seg.setStyleSheet("color: #d03b3b;")
                return

            def f(d):
                if v is None:
                    d["trusses"][i].pop("usable", None)
                else:
                    d["trusses"][i]["usable"] = v
            self.doc.edit("Truss usable sections", f)
        seg.editingFinished.connect(set_seg)
        form.addRow("Usable sections", seg)
        top = self.doc.data["volume"]["z"][1]
        if t["height_m"] - self.doc.data["candidates"].get("truss_drop_m", 0.3) < top:
            warn = QLabel(f"⚠︎ Cameras would hang inside the flight volume (top {top:g} m).")
            warn.setStyleSheet("color: #d03b3b;")
            warn.setWordWrap(True)
            form.addRow(warn)
        form.addRow(self._delete_button("truss", i))
        return box

    # ---------------------------------------------------------------- zones
    def _zone_box(self, i):
        z = self.doc.data["tripod_zones"][i]
        strip = "offset_m" in z
        box = QGroupBox(("Tripod strip " if strip else "Tripod area ") + z["id"])
        form = QFormLayout(box)
        name = QLineEdit(z["id"])
        name.editingFinished.connect(lambda: self._set(("tripod_zones", i, "id"), name.text().strip() or z["id"]))
        form.addRow("Name", name)
        h0, h1 = dspin(0.5, 20), dspin(0.5, 20)
        h0.setValue(z["height_m"][0])
        h1.setValue(z["height_m"][1])
        h0.editingFinished.connect(lambda: self._set(("tripod_zones", i, "height_m", 0), h0.value()))
        h1.editingFinished.connect(lambda: self._set(("tripod_zones", i, "height_m", 1), h1.value()))
        form.addRow("Tripod height from", h0)
        form.addRow("Tripod height up to", h1)
        if strip:
            o0, o1 = dspin(0, 100), dspin(0, 100)
            o0.setValue(z["offset_m"][0])
            o1.setValue(z["offset_m"][1])
            o0.editingFinished.connect(lambda: self._set(("tripod_zones", i, "offset_m", 0), o0.value()))
            o1.editingFinished.connect(lambda: self._set(("tripod_zones", i, "offset_m", 1), o1.value()))
            form.addRow("Distance from edge, min", o0)
            form.addRow("Distance from edge, max", o1)
            n = len(self.doc.data["volume"]["polygon"])
            edges = z.get("edges")
            grid = QWidget()
            gl = QHBoxLayout(grid)
            gl.setContentsMargins(0, 0, 0, 0)
            boxes = []
            for k in range(n):
                cb = QCheckBox(f"E{k}")
                cb.setChecked(edges is None or k in edges)
                boxes.append(cb)
                gl.addWidget(cb)

            def set_edges():
                chosen = [k for k, cb in enumerate(boxes) if cb.isChecked()]

                def f(d):
                    if len(chosen) == n:
                        d["tripod_zones"][i].pop("edges", None)
                    else:
                        d["tripod_zones"][i]["edges"] = chosen
                self.doc.edit("Tripod strip edges", f)
            for cb in boxes:
                cb.toggled.connect(lambda _=None: set_edges())
            form.addRow("Along edges", grid)
        else:
            table = VertexTable(lambda r, c, v: self._set(("tripod_zones", i, "polygon", r, c), v))
            table.load(z["polygon"], prefix="P")
            form.addRow(table)
        form.addRow(self._delete_button("zone", i))
        return box

    def _nogo_box(self, i):
        box = QGroupBox("No-go area")
        form = QFormLayout(box)
        table = VertexTable(lambda r, c, v: self._set(("no_go_zones", i, r, c), v))
        table.load(self.doc.data["no_go_zones"][i], prefix="P")
        form.addRow(table)
        form.addRow(QLabel("No cameras or tripods are placed here (doors, walkways, exits)."))
        form.addRow(self._delete_button("nogo", i))
        return box

    def _camera_box(self, k):
        box = QGroupBox("Planned camera")
        form = QFormLayout(box)
        if self.canvas.result is None or k >= len(self.canvas.result[0].cameras):
            return box
        plan, ev = self.canvas.result
        c = plan.cameras[k]
        x, y, z = c.position_m
        rows = [("ID", c.id), ("Mount", f"{c.mount_type} {c.mount_id}"),
                ("Position", f"X {x:.2f}  Y {y:.2f}  Z {z:.2f} m"),
                ("Gimbal base heading", f"{c.base_heading_deg:.0f}° (0° = +X, 90° = +Y)"),
                ("Pan / tilt", f"{c.pan_deg:+.1f}° / {c.tilt_deg:+.1f}°"), ("Lens", f"{c.lens_mm:g} mm"),
                ("Nearest neighbour", "" if c.neighbour_spacing_m is None else f"{c.neighbour_spacing_m:.2f} m"),
                ("Critical points", f"{int(ev.critical[k])} would fail without this camera")]
        for a, b in rows:
            form.addRow(a, QLabel(b))
        note = QLabel(c.install_note)
        note.setWordWrap(True)
        note.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        form.addRow("Install", note)
        return box

    # ---------------------------------------------------------------- helpers
    def _set(self, path, value):
        def f(d):
            o = d
            for p in path[:-1]:
                o = o[p]
            o[path[-1]] = value
        self.doc.edit("Edit", f)


class SettingsPage(QWidget):
    """Venue name, requirements, design margin, lenses and optimiser density."""

    FIELDS = [
        # (group, label, path, kind, extra)
        ("Venue", "Name", ("name",), "text", None),
        ("Requirement", "Min cameras per point", ("requirements", "min_cameras"), "int", (1, 10)),
        ("Requirement", "p95 accuracy", ("requirements", "p95_mm"), "float", (1, 100, " mm")),
        ("Requirement", "Error budget", ("requirements", "noise"), "preset", None),
        ("Design margin (optimiser target)", "Min cameras per point", ("design", "min_cameras"), "int", (1, 12)),
        ("Design margin (optimiser target)", "p95 accuracy", ("design", "p95_mm"), "float", (1, 100, " mm")),
        ("Cameras", "Lenses (mm)", ("lenses_mm",), "list", None),
        ("Cameras", "Max detection range", ("visibility", "max_range_m"), "float", (5, 300, " m")),
        ("Cameras", "Max cameras", ("optimiser", "max_cameras"), "int", (1, 2000)),
        ("Cameras", "Cameras per mount", ("candidates", "max_cameras_per_mount"), "int", (1, 6)),
        ("Cameras", "Min spacing between cameras", ("candidates", "min_camera_spacing_m"), "float", (0, 50, " m")),
        ("Candidate density", "Truss spacing", ("candidates", "truss_step_m"), "float", (0.5, 50, " m")),
        ("Candidate density", "Camera drop below truss", ("candidates", "truss_drop_m"), "float", (0, 5, " m")),
        ("Candidate density", "Truss tilt angles (°)", ("candidates", "truss_elevations_deg"), "list", None),
        ("Candidate density", "Tripod spacing", ("candidates", "tripod_step_m"), "float", (0.5, 50, " m")),
        ("Candidate density", "Tripod mast heights (m)", ("candidates", "tripod_heights_m"), "list", None),
        ("Candidate density", "Tripod tilt angles (°)", ("candidates", "tripod_elevations_deg"), "list", None),
        ("Candidate density", "Pan step", ("candidates", "azimuth_step_deg"), "float", (5, 90, "°")),
        ("Evaluation grid", "Spacing XY", ("evaluation", "spacing_m", 0), "float", (0.25, 50, " m")),
        ("Evaluation grid", "Spacing Z", ("evaluation", "spacing_m", 2), "float", (0.25, 50, " m")),
        ("Evaluation grid", "Inset from walls", ("evaluation", "inset_m"), "float", (0, 5, " m")),
    ]

    def __init__(self, doc, parent=None):
        super().__init__(parent)
        self.doc = doc
        lay = QVBoxLayout(self)
        lay.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.vol_box = QGroupBox("Flight volume height")
        vol_form = QFormLayout(self.vol_box)
        self.z0, self.z1 = volume_z_editors(doc)
        vol_form.addRow("Floor Z (min)", self.z0)
        vol_form.addRow("Top Z (max)", self.z1)
        lay.addWidget(self.vol_box)
        self.widgets = {}
        groups = {}
        for group, label, path, kind, extra in self.FIELDS:
            if group not in groups:
                g = QGroupBox(group)
                groups[group] = QFormLayout(g)
                lay.addWidget(g)
            w = self._make(path, kind, extra)
            self.widgets[path] = (w, kind)
            groups[group].addRow(label, w)
        hint = QLabel("Error budget presets: realistic = 0.1 px / 0.1 mrad / 3 mm / 0.1 ms; "
                      "tight = 0.05 px / 0.05 mrad / 2 mm / 0.05 ms; very_tight = 0.05 px / 0.03 mrad / "
                      "1 mm / 0.02 ms (see docs/RISKS.md).")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: palette(placeholder-text);")
        lay.addWidget(hint)
        self.refresh()

    def _make(self, path, kind, extra):
        if kind == "text":
            w = QLineEdit()
            w.editingFinished.connect(lambda: self._set(path, w.text()))
        elif kind == "int":
            w = ispin(*extra)
            w.editingFinished.connect(lambda: self._set(path, w.value()))
        elif kind == "float":
            w = dspin(extra[0], extra[1], 2, extra[2], 0.5)
            w.editingFinished.connect(lambda: self._set(path, w.value()))
        elif kind == "preset":
            w = Combo()
            w.addItems(list(PRESETS))
            w.activated.connect(lambda _: self._set(path, w.currentText()))
        else:
            w = QLineEdit()
            w.setPlaceholderText("comma separated")

            def commit():
                try:
                    v = parse_floats(w.text())
                except ValueError:
                    w.setStyleSheet("color: #d03b3b;")
                    return
                w.setStyleSheet("")
                if v:
                    self._set(path, v)
            w.editingFinished.connect(commit)
        return w

    def _get(self, path):
        o = self.doc.data
        for p in path:
            o = o[p]
        return o

    def _set(self, path, value):
        if path[:2] == ("evaluation", "spacing_m") and path[2] == 0:
            def f(d):
                sp = d["evaluation"]["spacing_m"]
                if not isinstance(sp, list):
                    d["evaluation"]["spacing_m"] = sp = [sp, sp, sp]
                sp[0] = sp[1] = value
            self.doc.edit("Evaluation spacing", f)
            return

        def f(d):
            o = d
            for p in path[:-1]:
                if isinstance(o, dict) and p not in o:
                    o[p] = {}
                o = o[p]
            if isinstance(o, (int, float)):
                return
            o[path[-1]] = value
        if path[:2] == ("evaluation", "spacing_m"):
            def g(d, f=f):
                sp = d["evaluation"]["spacing_m"]
                if not isinstance(sp, list):
                    d["evaluation"]["spacing_m"] = [sp, sp, sp]
                f(d)
            self.doc.edit("Evaluation spacing", g)
            return
        self.doc.edit("Settings", f)

    def refresh(self):
        sync_z_editors(self.doc, self.z0, self.z1)
        for path, (w, kind) in self.widgets.items():
            try:
                v = self._get(path)
            except (KeyError, IndexError, TypeError):
                if path[:2] == ("evaluation", "spacing_m"):
                    v = self.doc.data["evaluation"]["spacing_m"]
                else:
                    continue
            w.blockSignals(True)
            if kind == "text":
                w.setText(str(v))
            elif kind == "int":
                w.setValue(int(v))
            elif kind == "float":
                w.setValue(float(v))
            elif kind == "preset":
                w.setCurrentText(v if isinstance(v, str) else "tight")
            else:
                w.setText(fmt_floats(v))
            w.blockSignals(False)

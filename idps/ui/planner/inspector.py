"""Right-hand panel: properties of the selected item, and venue/requirement settings.

Every edit goes through SiteDocument.edit, so it is undoable (Cmd+Z)."""
from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout,
                               QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLayout, QLineEdit, QPushButton,
                               QScrollArea, QSizePolicy, QSpinBox, QTableWidget, QTableWidgetItem,
                               QTabWidget, QVBoxLayout, QWidget)

from idps.core.noise import PRESETS
from idps.ui.planner.document import frame_of


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
    """Editable vertex list in North / East from the origin, with edge lengths."""

    def __init__(self, on_edit, parent=None):
        super().__init__(0, 3, parent)
        self.on_edit = on_edit
        self.setHorizontalHeaderLabels(["N (m)", "E (m)", "edge to next"])
        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.setAlternatingRowColors(True)
        self.setMinimumHeight(170)
        self._loading = False
        self.itemChanged.connect(self._changed)

    def load(self, pts, frame, prefix="V"):
        self._loading = True
        self.setRowCount(len(pts))
        self.setVerticalHeaderLabels([f"{prefix}{k}" for k in range(len(pts))])
        for k, (x, y) in enumerate(pts):
            n, e = frame.ne((x, y))
            self.setItem(k, 0, QTableWidgetItem(f"{n:.3f}"))
            self.setItem(k, 1, QTableWidgetItem(f"{e:.3f}"))
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
        # the page is at least as tall as its contents, so the scroll area scrolls instead of squashing rows
        self.sel_layout.setSizeConstraint(QLayout.SizeConstraint.SetMinimumSize)
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
        if kind == "origin" and d.get("origin"):
            self.sel_layout.addWidget(self._origin_box())
        elif kind in (None, "volume", "vertex", "origin"):
            self.sel_layout.addWidget(self._volume_box(sel[1] if kind == "vertex" else None))
            self.sel_layout.addWidget(self._add_box())
        elif kind == "truss" and sel[1] < len(d["trusses"]):
            self.sel_layout.addWidget(self._truss_box(sel[1]))
        elif kind == "zone" and sel[1] < len(d["tripod_zones"]):
            self.sel_layout.addWidget(self._zone_box(sel[1]))
        elif kind == "nogo" and sel[1] < len(d["no_go_zones"]):
            self.sel_layout.addWidget(self._nogo_box(sel[1]))
        elif kind == "pad" and sel[1] < len((d.get("pads") or {}).get("centres") or []):
            self.sel_layout.addWidget(self._pad_box(sel[1]))
            self.sel_layout.addWidget(self._pads_settings_box())
        elif kind == "camera":
            self.sel_layout.addWidget(self._camera_box(sel[1]))
        self.sel_layout.addWidget(self._mounts_box())
        self.sel_layout.addStretch(1)
        self.settings.refresh()

    # ---------------------------------------------------------------- volume
    def _volume_box(self, vertex):
        d = self.doc.data
        vol = d["volume"]
        box = QGroupBox("Performance area (flight volume)")
        form = QFormLayout(box)
        z0, z1 = volume_z_editors(self.doc)
        form.addRow("Lowest flight height", z0)
        form.addRow("Ceiling (top) height", z1)
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
        table.load(pts, frame_of(d))
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
        hint = QLabel("Draw with the Performance area tool (P), drag corners on the plan, or type exact "
                      "coordinates here. Coordinates are North / East from the origin (the calibration "
                      "square); heights are above the floor (Down = −height).")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: palette(placeholder-text);")
        form.addRow(hint)
        return box

    def _edit_vertex(self, row, col, value):
        f = frame_of(self.doc.data)

        def edit(d):
            ne = f.ne(d["volume"]["polygon"][row])
            ne[col] = value
            d["volume"]["polygon"][row] = [round(float(v), 4) for v in f.xy(ne)]
        self.doc.edit("Edit vertex", edit)

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
        if len(self.doc.data["volume"]["polygon"]) >= 3 and not self.doc.data.get("origin"):
            place = QPushButton("Place the origin (O)…")
            place.setToolTip("Where the calibration square's corner goes; its N arrow points North")
            place.clicked.connect(lambda: self.canvas.set_tool("origin"))
            lay.addWidget(place)
        if self.doc.data["trusses"] or self.doc.data["tripod_zones"]:
            pads = QPushButton("Place the charging pads (C)…")
            pads.setToolTip("Where the drones take off and land, 4 per pad; after the trusses, before planning")
            pads.clicked.connect(lambda: self.canvas.set_tool("pad"))
            lay.addWidget(pads)
        tips = QLabel("Trusses: T tool · Tripod areas: A tool · Charging pads: C tool · No-go areas: N tool · "
                      "Origin: O tool")
        tips.setWordWrap(True)
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
        rows += [("pad", i, f"Charging pad {i + 1}", f"drones {4 * i + 1}–{4 * i + 4}")
                 for i in range(len((d.get("pads") or {}).get("centres") or []))]
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
        bulk.addWidget(QLabel("Remove all:"))
        for label, key, what in (("Trusses", "trusses", "trusses"), ("Tripod zones", "tripod_zones", "tripod zones"),
                                 ("Pads", None, "charging pads")):
            b = QPushButton(label)
            b.setToolTip(f"Remove all {what} (⌘Z to undo)")
            if key is None:
                b.setEnabled(bool((d.get("pads") or {}).get("centres")))
                b.clicked.connect(lambda: self.doc.edit("Remove all charging pads",
                                                        lambda dd: dd["pads"]["centres"].clear()))
            else:
                b.setEnabled(bool(d[key]))
                b.clicked.connect(lambda _=False, k=key, w=what: self.doc.edit(f"Remove all {w}",
                                                                                lambda dd: dd[k].clear()))
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
        f = frame_of(self.doc.data)
        for key, label in (("start", "Start"), ("end", "End")):
            row = QHBoxLayout()
            ne = f.ne(t[key])
            for c in range(2):
                s = dspin(-10000, 10000)
                s.setPrefix("NE"[c] + " ")
                s.setValue(float(ne[c]))
                s.editingFinished.connect(lambda s=s, key=key, c=c: self._set_ne(("trusses", i, key), c, s.value()))
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
            warn = QLabel(f"⚠︎ Cameras would hang below the flight ceiling ({top:g} m), so drones could fly above "
                          f"them. This truss is not used; raise it.")
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
        ceiling = self.doc.data["volume"]["z"][1] + self.doc.data["candidates"].get("min_height_above_volume_m", 0.0)
        if z["height_m"][1] < ceiling - 1e-9:
            warn = QLabel(f"⚠︎ Masts must reach the flight ceiling ({ceiling:g} m): cameras are always above the "
                          f"drones. This zone is not used.")
            warn.setStyleSheet("color: #d03b3b;")
            warn.setWordWrap(True)
            form.addRow(warn)
        elif z["height_m"][0] < ceiling:
            note = QLabel(f"Cameras go at {ceiling:g} m or higher (the flight ceiling).")
            note.setStyleSheet("color: palette(placeholder-text);")
            note.setWordWrap(True)
            form.addRow(note)
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
            table = VertexTable(lambda r, c, v: self._set_ne(("tripod_zones", i, "polygon", r), c, v))
            table.load(z["polygon"], frame_of(self.doc.data), prefix="P")
            form.addRow(table)
        form.addRow(self._delete_button("zone", i))
        return box

    def _nogo_box(self, i):
        box = QGroupBox("No-go area")
        form = QFormLayout(box)
        table = VertexTable(lambda r, c, v: self._set_ne(("no_go_zones", i, r), c, v))
        table.load(self.doc.data["no_go_zones"][i], frame_of(self.doc.data), prefix="P")
        form.addRow(table)
        form.addRow(QLabel("No cameras or tripods are placed here (doors, walkways, exits)."))
        form.addRow(self._delete_button("nogo", i))
        return box

    # ---------------------------------------------------------------- pads
    def _pad_box(self, i):
        from idps.show.pads import MIN_PAD_GAP_M, too_close
        d = self.doc.data
        pads = d["pads"]
        c = pads["centres"][i]
        box = QGroupBox(f"Charging pad {i + 1}")
        form = QFormLayout(box)
        f = frame_of(d)
        row = QHBoxLayout()
        ne = f.ne(c)
        for k in range(2):
            sp = dspin(-10000, 10000)
            sp.setPrefix("NE"[k] + " ")
            sp.setValue(float(ne[k]))
            sp.editingFinished.connect(lambda sp=sp, k=k: self._set_ne(("pads", "centres", i), k, sp.value()))
            row.addWidget(sp)
        form.addRow("Centre", row)
        layers = pads.get("layers_m", [1, 2, 3, 4])
        names = ("front-left", "front-right", "back-left", "back-right")
        lines = [f"Drone {4 * i + s + 1}: {names[s]}, batch {s + 1}"
                 + (f", grid at {layers[s]:g} m" if s < len(layers) else "") for s in range(4)]
        who = QLabel("\n".join(lines))
        who.setToolTip("Front = where the pads face (towards the performance area unless set below). Batch k "
                       "takes off k-th (slot k of every pad together) and lands (n - k + 1)-th: last up, first down")
        form.addRow("Drones", who)
        for a, b, dist in too_close(pads["centres"]):
            if i in (a, b):
                warn = QLabel(f"⚠︎ {dist:.2f} m from pad {(b if a == i else a) + 1}: keep pads at least "
                              f"{MIN_PAD_GAP_M:g} m apart.")
                warn.setStyleSheet("color: #d03b3b;")
                warn.setWordWrap(True)
                form.addRow(warn)
        form.addRow(self._delete_button("pad", i))
        return box

    def _pads_settings_box(self):
        """The take-off and landing sequence, shared by every pad (docs/SHOW.md, Charging pads)."""
        p = self.doc.data["pads"]
        box = QGroupBox("Charging pads: take-off and landing")
        form = QFormLayout(box)
        for key, label, lo, hi, dec, suffix, step in (
                ("slot_spacing_m", "Drones apart on a pad", 0.1, 1.0, 2, " m", 0.05),
                ("batch_gap_s", "Between batches", 1.0, 60.0, 1, " s", 0.5),
                ("climb_speed_mps", "Climb and descent speed", 0.2, 5.0, 2, " m/s", 0.1),
                ("transfer_speed_mps", "Speed to and from the grid", 0.2, 5.0, 2, " m/s", 0.1),
                ("hold_timeout_s", "No start for this long: land", 5.0, 3600.0, 0, " s", 5.0)):
            sp = dspin(lo, hi, dec, suffix, step)
            sp.setValue(float(p.get(key, 0.0)))
            sp.editingFinished.connect(lambda sp=sp, key=key: self._set(("pads", key), sp.value()))
            form.addRow(label, sp)
        facing = Combo()
        choices = [("auto", "Towards the performance area"), ("north", "North"), ("east", "East"),
                   ("south", "South"), ("west", "West")]
        for key, label in choices:
            facing.addItem(label, key)
        cur = str(p.get("facing", "auto")).lower()
        facing.setCurrentIndex(next((k for k, (key, _) in enumerate(choices) if key == cur), 0))
        facing.setToolTip("The pads' front (slots 1 and 2, the first batches up): where the drones fly off to. "
                          "After climbing, each drone first steps forward over the pad's empty slots.")
        facing.currentIndexChanged.connect(lambda i: self._set(("pads", "facing"), facing.itemData(i)))
        form.addRow("Pads face", facing)
        lay_edit = QLineEdit(fmt_floats(p.get("layers_m", [1, 2, 3, 4])))
        lay_edit.setToolTip("Batch 1, 2, 3, 4's height in the initial and final grid (the show's first and last frame)")

        def set_layers():
            try:
                v = parse_floats(lay_edit.text())
            except ValueError:
                v = []
            if len(v) != 4 or any(x <= 0 for x in v):
                lay_edit.setStyleSheet("color: #d03b3b;")
                return
            self._set(("pads", "layers_m"), v)
        lay_edit.editingFinished.connect(set_layers)
        form.addRow("Grid heights, batch 1–4", lay_edit)
        hint = QLabel("The cameras are planned to see every drone on its pad and on its climb to the grid. The "
                      "show tools read the pads from this venue file: idps-show check / export --pads venue.yaml.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: palette(placeholder-text);")
        form.addRow(hint)
        return box

    def _camera_box(self, k):
        box = QGroupBox("Planned camera")
        form = QFormLayout(box)
        if self.canvas.result is None or k >= len(self.canvas.result[0].cameras):
            return box
        plan, ev = self.canvas.result
        c = plan.cameras[k]
        from idps.core.frames import pan_display
        f = frame_of(self.doc.data)
        rows = [("ID", c.id), ("Mount", f"{c.mount_type} {c.mount_id}"),
                ("Position", f.fmt_ned(c.position_m) + f" m  ({c.position_m[2]:.2f} m high)"),
                ("Gimbal base heading", f"{f.heading(c.base_heading_deg):.0f}° (from North, clockwise)"),
                ("Pan / tilt", f"{pan_display(c.pan_deg):+.1f}° / {c.tilt_deg:+.1f}°  (pan + = clockwise)"),
                ("Lens", f"{c.lens_mm:g} mm"),
                ("Nearest neighbour", "" if c.neighbour_spacing_m is None else f"{c.neighbour_spacing_m:.2f} m"),
                ("Critical points", f"{int(ev.critical[k])} would fail without this camera")]
        for a, b in rows:
            form.addRow(a, QLabel(b))
        note = QLabel(c.install_note)
        note.setWordWrap(True)
        note.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        form.addRow("Install", note)
        return box

    # ---------------------------------------------------------------- origin
    def _origin_box(self):
        d = self.doc.data
        f = frame_of(d)
        box = QGroupBox("Origin (calibration square) and North")
        form = QFormLayout(box)
        bearing = dspin(0, 360)
        bearing.setDecimals(1)
        bearing.setSuffix("°")
        bearing.setValue(round((90.0 - f.north_deg) % 360.0, 3))
        bearing.setToolTip("Direction of North on the plan, clockwise from the top of the plan")
        bearing.editingFinished.connect(
            lambda: self._set(("origin", "north_deg"), round((90.0 - bearing.value()) % 360.0, 4)))
        form.addRow("North, clockwise from plan up", bearing)
        row = QHBoxLayout()
        up = QPushButton("North = plan up")
        up.clicked.connect(lambda: self._set(("origin", "north_deg"), 90.0))
        row.addWidget(up)
        pts = d["volume"]["polygon"]
        if len(pts) >= 2:
            along = QPushButton("North along edge E0")
            along.setToolTip("Point North along the performance area's first edge, V0 → V1")

            def align():
                a, b = np.array(pts[0], float), np.array(pts[1], float)
                self._set(("origin", "north_deg"), round(float(np.degrees(np.arctan2(*(b - a)[::-1]))) % 360, 4))
            along.clicked.connect(align)
            row.addWidget(along)
        form.addRow(row)
        if len(pts) >= 1:
            # exact placement from a tape measurement: the origin's offset from vertex V0
            v0 = np.array(pts[0], float)
            off = -f.ne(v0)
            row2 = QHBoxLayout()
            spins = []
            for c, lab in enumerate("NE"):
                sp = dspin(-10000, 10000)
                sp.setPrefix(lab + " ")
                sp.setValue(float(off[c]))
                spins.append(sp)
                row2.addWidget(sp)

            def set_from_v0():
                xy = f.xy(f.ne(v0) + np.array([spins[0].value(), spins[1].value()]))
                self._set(("origin", "xy"), [round(float(v), 4) for v in xy])
            for sp in spins:
                sp.editingFinished.connect(set_from_v0)
            form.addRow("Origin from V0", row2)
        hint = QLabel("Drag the origin on the plan to move it, and its red N arrow to turn it. Everything is "
                      "measured North-East-Down from here, the frame the drones fly in: on site, lay the "
                      "calibration square's corner on this spot, its long arm pointing North and its short arm "
                      "East (to the right when facing North). Drone yaw 0 = North.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: palette(placeholder-text);")
        form.addRow(hint)
        return box

    # ---------------------------------------------------------------- helpers
    def _set_ne(self, path, col, value):
        """Set one N/E coordinate of a drawing-frame XY point."""
        f = frame_of(self.doc.data)

        def edit(d):
            o = d
            for p in path:
                o = o[p]
            ne = f.ne(o)
            ne[col] = value
            o[:] = [round(float(v), 4) for v in f.xy(ne)]
        self.doc.edit("Edit", edit)

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
        ("Cameras", "Min height above flight ceiling", ("candidates", "min_height_above_volume_m"), "float",
         (0, 20, " m")),
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
        vol_form.addRow("Lowest flight height", self.z0)
        vol_form.addRow("Ceiling (top) height", self.z1)
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

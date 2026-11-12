"""Designer tab: build a show from scenes and see it fly in the venue.

Scenes (a grid, a circle, a heart, text, the shape in an image) are listed on the right; the
selected one's settings are underneath. Every change rebuilds the show (idps.show.designer):
the drones' places, the flights between the scenes and the checks. The 3D view plays it with
the LED colours; what is wrong is listed, and a click on it jumps there. The finished show goes
to the Simulate tab or the PX4 flight tab as a show file.
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QPen
from PySide6.QtWidgets import (QCheckBox, QColorDialog, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QHBoxLayout,
                               QLabel, QLineEdit, QListWidget, QMenu, QPushButton, QScrollArea, QSlider, QSpinBox, QSplitter,
                               QToolButton, QVBoxLayout, QWidget)

from idps.show.designer import DEPTHS, RATE_HZ, Compiled, Design, DesignError, Scene, compile_design
from idps.ui import theme
from idps.ui.flow import flow_row
from idps.ui.planner.view3d import View3D
from idps.ui.sim.view import led_colours

ROOT = Path(__file__).resolve().parents[3]
PLAY_MS = 40
TRAIL_S = 1.5
KIND_NAMES = {"grid": "Grid", "circle": "Circle", "heart": "Heart", "text": "Text…", "image": "Image…"}
DEPTH_NAMES = {"flat": "Flat", "tilt": "Tilt (top to the audience)", "bulge": "Bulge (middle to the audience)",
               "brightness": "By the image's brightness"}


class DesignView3D(View3D):
    """The show at one moment: each drone a dot in its LED colour, with a short trail."""

    def __init__(self, doc, canvas, parent=None):
        super().__init__(doc, canvas, parent)
        self.coverage_mode = "off"
        self.show_cones = "none"
        self.world = None                # (N, T, 3) drawing frame
        self.rgb = None                  # (N, T, 3) uint8
        self.frame = 0
        self.close = set()               # drones too close at this moment

    def set_show(self, world, rgb):
        self.world, self.rgb = world, rgb
        self.frame = 0 if world is None else min(self.frame, world.shape[1] - 1)
        self.update()

    def set_frame(self, k: int, close=()):
        self.frame, self.close = k, set(close)
        self.update()

    def _extra(self, prims, tok):
        if self.world is None:
            return
        k = min(self.frame, self.world.shape[1] - 1)
        n_trail = int(TRAIL_S * RATE_HZ)
        for i in range(len(self.world)):
            fill, _, _ = led_colours(self.rgb, i, k, QColor(tok["text2"]), tok["surface"])
            faint = QColor(fill)
            faint.setAlphaF(0.4)
            self._polyline(prims, self.world[i, max(0, k - n_trail):k + 1:2], QPen(faint, 1.2))
            P, z = self.project([self.world[i, k]])
            if z[0] > 0.05:
                edge = QColor(theme.STATUS[0]) if i in self.close else QColor(tok["text2"])
                prims.append((float(z[0]) - 0.03, "dot", (P[0], 6.5, fill, edge)))
            self._text(prims, self.world[i, k], str(i + 1), tok["text2"], False, 8)


def _spin(lo, hi, step, decimals=1, suffix="", special=None):
    b = QDoubleSpinBox()
    b.setRange(lo, hi)
    b.setSingleStep(step)
    b.setDecimals(decimals)
    b.setSuffix(suffix)
    b.setKeyboardTracking(False)
    if special:
        b.setSpecialValueText(special)
    return b


class DesignTab(QWidget):
    def __init__(self, window, parent=None):
        super().__init__(parent)
        self.win = window
        self.design = Design(n=20, scenes=[])
        self.compiled: Compiled | None = None
        self.path: Path | None = None
        self._cache: dict = {}
        self._loading = False
        self._site_key = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        bar_box, bar = flow_row()
        bar.addWidget(QLabel("Drones:"))
        self.n_drones = QSpinBox()
        self.n_drones.setRange(1, 400)
        self.n_drones.setValue(20)
        self.n_drones.setKeyboardTracking(False)
        self.n_drones.valueChanged.connect(self._design_changed)
        bar.addWidget(self.n_drones)
        self.add_btn = QToolButton()
        self.add_btn.setText("Add scene")
        self.add_btn.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        menu = QMenu(self.add_btn)
        for kind, name in KIND_NAMES.items():
            menu.addAction(name, lambda kind=kind: self._add_from_menu(kind))
        self.add_btn.setMenu(menu)
        bar.addWidget(self.add_btn)
        for text, tip, fn in (("Duplicate", "A copy of the selected scene after it", self.duplicate_scene),
                              ("Delete", "Remove the selected scene", self.delete_scene),
                              ("↑", "The selected scene earlier in the show", lambda: self.move_scene(-1)),
                              ("↓", "The selected scene later in the show", lambda: self.move_scene(1))):
            b = QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(fn)
            bar.addWidget(b)
        bar.addSpacing(10)
        for text, fn in (("Open…", self.choose_open), ("Save…", self.choose_save)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            bar.addWidget(b)
        bar.addSpacing(10)
        self.sim_btn = QPushButton("Send to Simulate")
        self.sim_btn.setToolTip("Film this show with the planned cameras (Simulate tab)")
        self.sim_btn.clicked.connect(self.send_to_simulate)
        self.px4_btn = QPushButton("Send to PX4 flight")
        self.px4_btn.setToolTip("Fly this show with simulated PX4 drones (PX4 flight tab)")
        self.px4_btn.clicked.connect(self.send_to_px4)
        self.export_btn = QPushButton("Export show file…")
        self.export_btn.clicked.connect(self.choose_export)
        for b in (self.sim_btn, self.px4_btn, self.export_btn):
            bar.addWidget(b)
        lay.addWidget(bar_box)
        self.status = QLabel("Add a scene to start: a grid, a circle, text or an image. The drones take off from the "
                             "charging pads to the venue's grid, fly your scenes, and return to it.")
        self.status.setWordWrap(True)
        lay.addWidget(self.status)

        split = QSplitter(Qt.Orientation.Horizontal)
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        self.view = DesignView3D(window.doc, window.canvas)
        ll.addWidget(self.view, 1)
        tl = QHBoxLayout()
        self.play_btn = QPushButton("Play")
        self.play_btn.setEnabled(False)
        self.play_btn.clicked.connect(self.toggle_play)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setEnabled(False)
        self.slider.valueChanged.connect(self.set_frame)
        self.clock = QLabel("")
        self.clock.setMinimumWidth(190)
        tl.addWidget(self.play_btn)
        tl.addWidget(self.slider, 1)
        tl.addWidget(self.clock)
        ll.addLayout(tl)
        self.findings = QListWidget()
        self.findings.setMaximumHeight(96)
        self.findings.setToolTip("What is wrong with the show. Click one to see that moment.")
        self.findings.itemClicked.connect(self._finding_clicked)
        ll.addWidget(self.findings)
        split.addWidget(left)

        side = QWidget()
        sl = QVBoxLayout(side)
        sl.setContentsMargins(6, 0, 0, 0)
        sl.addWidget(QLabel("Scenes (in show order):"))
        self.scenes = QListWidget()
        self.scenes.setMaximumHeight(150)
        self.scenes.currentRowChanged.connect(self._scene_selected)
        sl.addWidget(self.scenes)
        self.form_box = QWidget()
        form = QFormLayout(self.form_box)
        form.setContentsMargins(0, 4, 0, 0)
        self.f_text = QLineEdit()
        self.f_text.editingFinished.connect(self._form_changed)
        self.f_image = QPushButton("Choose image…")
        self.f_image.clicked.connect(self._choose_image)
        self.f_width = _spin(0.0, 200.0, 0.5, 1, " m", "as small as the spacing allows")
        self.f_centre = QCheckBox("In the middle of the area")
        self.f_east = _spin(-200.0, 200.0, 0.5, 1, " m")
        self.f_north = _spin(-200.0, 200.0, 0.5, 1, " m")
        self.f_height = _spin(0.0, 100.0, 0.25, 2, " m", "automatic")
        self.f_facing = _spin(-1.0, 359.0, 15.0, 0, "°", "automatic")
        self.f_depth = QComboBox()
        for key in DEPTHS:
            self.f_depth.addItem(DEPTH_NAMES[key], key)
        self.f_depth_m = _spin(-20.0, 20.0, 0.25, 2, " m")
        self.f_hold = _spin(1.0, 600.0, 1.0, 1, " s")
        self.f_move = _spin(0.0, 120.0, 1.0, 1, " s", "automatic")
        self.f_colour = QPushButton()
        self.f_colour.clicked.connect(self._choose_colour)
        self.f_image_colours = QCheckBox("Colours from the image")
        self.f_wave = _spin(0.0, 5.0, 0.05, 2, " m", "off")
        self.f_wave_len = _spin(0.5, 100.0, 0.5, 1, " m")
        self.f_wave_period = _spin(0.5, 60.0, 0.5, 1, " s")
        self.f_spin = _spin(-1440.0, 1440.0, 15.0, 0, "°")
        rows = (("Text", self.f_text), ("Image", self.f_image), ("Width", self.f_width), ("Place", self.f_centre),
                ("East of the origin", self.f_east), ("North of the origin", self.f_north),
                ("Height of its middle", self.f_height), ("Faces (from North)", self.f_facing), ("Depth", self.f_depth),
                ("Depth by", self.f_depth_m), ("Holds for", self.f_hold), ("Flight to it takes", self.f_move),
                ("Colour", self.f_colour), ("", self.f_image_colours), ("Wave height", self.f_wave),
                ("Wave length", self.f_wave_len), ("Wave period", self.f_wave_period), ("Turns while it holds", self.f_spin))
        self._rows = {}
        for label, w in rows:
            form.addRow(label, w)
            self._rows[w] = form.labelForField(w)
        for w in (self.f_width, self.f_east, self.f_north, self.f_height, self.f_facing, self.f_depth_m, self.f_hold,
                  self.f_move, self.f_wave, self.f_wave_len, self.f_wave_period, self.f_spin):
            w.valueChanged.connect(self._form_changed)
        self.f_depth.currentIndexChanged.connect(self._form_changed)
        self.f_centre.toggled.connect(self._form_changed)
        self.f_image_colours.toggled.connect(self._form_changed)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.form_box)
        sl.addWidget(scroll, 1)
        show_box = QWidget()
        sf = QFormLayout(show_box)
        sf.setContentsMargins(0, 4, 0, 0)
        self.s_sep = _spin(0.3, 5.0, 0.05, 2, " m")
        self.s_sep.setValue(self.design.min_sep_m)
        self.s_sep.setToolTip("The closest two drones may come, centre to centre, anywhere in the show")
        self.s_speed = _spin(0.2, 3.0, 0.1, 1, " m/s")
        self.s_speed.setValue(self.design.speed_mps)
        self.s_speed.setToolTip("How fast the drones fly from one scene to the next")
        self.s_facing = _spin(-1.0, 359.0, 15.0, 0, "°", "automatic")
        self.s_facing.setValue(-1.0)
        self.s_facing.setToolTip("Which way the scenes face: where the audience is, as a compass direction from the "
                                 "scene. Automatic: across the long side of the area, away from the charging pads.")
        sf.addRow("Drones at least", self.s_sep)
        sf.addRow("Speed between scenes", self.s_speed)
        sf.addRow("Audience (from North)", self.s_facing)
        for w in (self.s_sep, self.s_speed, self.s_facing):
            w.valueChanged.connect(self._design_changed)
        sl.addWidget(show_box)
        split.addWidget(side)
        split.setStretchFactor(0, 1)
        split.setSizes([900, 380])
        lay.addWidget(split, 1)

        self.timer = QTimer(self)
        self.timer.setInterval(PLAY_MS)
        self.timer.timeout.connect(self._tick)
        self._last_tick, self._acc = None, 0.0
        self._rebuild = QTimer(self)                      # one rebuild after a burst of changes
        self._rebuild.setSingleShot(True)
        self._rebuild.setInterval(150)
        self._rebuild.timeout.connect(self.rebuild)
        window.doc.changed.connect(self._venue_changed)
        self._fill_form()

    # ------------------------------------------------------------ scenes
    def _add_from_menu(self, kind: str):
        if kind == "image":
            path, _ = QFileDialog.getOpenFileName(self, "Choose an image", "", "Images (*.png *.jpg *.jpeg *.bmp *.gif *.webp)")
            if path:
                self.add_scene("image", image=path, colour="image")
        elif kind == "text":
            self.add_scene("text", text="HI")
            self.f_text.setFocus()
            self.f_text.selectAll()
        else:
            self.add_scene(kind)

    def add_scene(self, kind: str, **kw) -> Scene:
        sc = Scene(kind, **kw)
        at = self.scenes.currentRow() + 1 if self.scenes.currentRow() >= 0 else len(self.design.scenes)
        self.design.scenes.insert(at, sc)
        self._refill_list(at)
        self.rebuild()
        return sc

    def duplicate_scene(self):
        k = self.scenes.currentRow()
        if k >= 0:
            import dataclasses
            self.design.scenes.insert(k + 1, dataclasses.replace(self.design.scenes[k]))
            self._refill_list(k + 1)
            self.rebuild()

    def delete_scene(self):
        k = self.scenes.currentRow()
        if k >= 0:
            del self.design.scenes[k]
            self._refill_list(min(k, len(self.design.scenes) - 1))
            self.rebuild()

    def move_scene(self, by: int):
        k = self.scenes.currentRow()
        j = k + by
        if k >= 0 and 0 <= j < len(self.design.scenes):
            s = self.design.scenes
            s[k], s[j] = s[j], s[k]
            self._refill_list(j)
            self.rebuild()

    def _refill_list(self, select: int = -1):
        self._loading = True
        self.scenes.clear()
        for k, sc in enumerate(self.design.scenes):
            self.scenes.addItem(sc.label(k))
        self._loading = False
        self.scenes.setCurrentRow(select if 0 <= select < len(self.design.scenes) else -1)
        self._fill_form()

    def current(self) -> Scene | None:
        k = self.scenes.currentRow()
        return self.design.scenes[k] if 0 <= k < len(self.design.scenes) else None

    def _scene_selected(self, k: int):
        if self._loading:
            return
        self._fill_form()
        if self.compiled is not None and k >= 0:
            span = self.compiled.scene_span(k)
            if span is not None:
                self.slider.setValue(int(round(span[0] * RATE_HZ)))

    # ------------------------------------------------------------ the form
    def _fill_form(self):
        sc = self.current()
        self.form_box.setEnabled(sc is not None)
        if sc is None:
            return
        self._loading = True
        self.f_text.setText(sc.text)
        self.f_image.setText(Path(sc.image).name if sc.image else "Choose image…")
        self.f_width.setValue(sc.width_m)
        self.f_centre.setChecked(sc.east_m is None and sc.north_m is None)
        self.f_east.setValue(sc.east_m or 0.0)
        self.f_north.setValue(sc.north_m or 0.0)
        self.f_height.setValue(sc.height_m or 0.0)
        self.f_facing.setValue(-1.0 if sc.facing_deg is None else sc.facing_deg)
        self.f_depth.setCurrentIndex(max(0, self.f_depth.findData(sc.depth)))
        self.f_depth_m.setValue(sc.depth_m)
        self.f_hold.setValue(sc.hold_s)
        self.f_move.setValue(sc.move_s)
        self.f_image_colours.setChecked(sc.colour == "image")
        self.f_wave.setValue(sc.wave_m)
        self.f_wave_len.setValue(sc.wave_length_m)
        self.f_wave_period.setValue(sc.wave_period_s)
        self.f_spin.setValue(sc.spin_deg)
        self._paint_colour(sc)
        self._loading = False
        self._show_rows(sc)

    def _show_rows(self, sc: Scene):
        show = {self.f_text: sc.kind == "text", self.f_image: sc.kind == "image", self.f_image_colours: sc.kind == "image",
                self.f_east: not self.f_centre.isChecked(), self.f_north: not self.f_centre.isChecked(),
                self.f_depth_m: sc.depth != "flat", self.f_wave_len: sc.wave_m > 0, self.f_wave_period: sc.wave_m > 0,
                self.f_colour: not (sc.kind == "image" and sc.colour == "image")}
        for w, on in show.items():
            w.setVisible(on)
            if self._rows.get(w) is not None:
                self._rows[w].setVisible(on)

    def _paint_colour(self, sc: Scene):
        r, g, b = sc.rgb
        dark = (r * 299 + g * 587 + b * 114) / 1000 < 140
        self.f_colour.setText(f"#{r:02x}{g:02x}{b:02x}")
        self.f_colour.setStyleSheet(f"background-color: rgb({r},{g},{b}); color: {'white' if dark else 'black'};")

    def _choose_colour(self):
        sc = self.current()
        if sc is None:
            return
        c = QColorDialog.getColor(QColor(*sc.rgb), self, "The scene's LED colour")
        if c.isValid():
            self.set_colour((c.red(), c.green(), c.blue()))

    def set_colour(self, rgb):
        sc = self.current()
        if sc is not None:
            sc.rgb = tuple(int(x) for x in rgb)
            self._paint_colour(sc)
            self._rebuild.start()

    def _choose_image(self):
        sc = self.current()
        path, _ = QFileDialog.getOpenFileName(self, "Choose an image", "", "Images (*.png *.jpg *.jpeg *.bmp *.gif *.webp)")
        if path and sc is not None:
            sc.image = path
            self._refill_list(self.scenes.currentRow())
            self._rebuild.start()

    def _form_changed(self, *_):
        sc = self.current()
        if self._loading or sc is None:
            return
        sc.text = self.f_text.text()
        sc.width_m = self.f_width.value()
        if self.f_centre.isChecked():
            sc.east_m = sc.north_m = None
        else:
            sc.east_m, sc.north_m = self.f_east.value(), self.f_north.value()
        sc.height_m = self.f_height.value() or None
        sc.facing_deg = None if self.f_facing.value() < 0 else self.f_facing.value()
        sc.depth = self.f_depth.currentData()
        sc.depth_m = self.f_depth_m.value()
        if sc.depth != "flat" and sc.depth_m == 0:        # a depth chosen: start with something to see
            sc.depth_m = 1.0
            self._loading = True
            self.f_depth_m.setValue(1.0)
            self._loading = False
        sc.hold_s = self.f_hold.value()
        sc.move_s = self.f_move.value()
        sc.colour = "image" if self.f_image_colours.isChecked() and sc.kind == "image" else "single"
        sc.wave_m = self.f_wave.value()
        sc.wave_length_m = self.f_wave_len.value()
        sc.wave_period_s = self.f_wave_period.value()
        sc.spin_deg = self.f_spin.value()
        self._show_rows(sc)
        k = self.scenes.currentRow()
        if self.scenes.item(k) is not None:
            self.scenes.item(k).setText(sc.label(k))
        self._rebuild.start()

    def _design_changed(self, *_):
        if self._loading:
            return
        self.design.n = self.n_drones.value()
        self.design.min_sep_m = self.s_sep.value()
        self.design.speed_mps = self.s_speed.value()
        self.design.facing_deg = None if self.s_facing.value() < 0 else self.s_facing.value()
        self._rebuild.start()

    def _venue_changed(self, *_):
        """The venue was edited in the Planner: the show is rebuilt when this tab is next looked at."""
        if self.isVisible() and self.design.scenes:
            self._rebuild.start()

    def showEvent(self, e):
        super().showEvent(e)
        if self.design.scenes:
            self._rebuild.start()

    # ------------------------------------------------------------ building
    def rebuild(self):
        self._rebuild.stop()
        if not self.design.scenes:
            self.compiled = None
            self.view.set_show(None, None)
            self.findings.clear()
            self.slider.setEnabled(False)
            self.play_btn.setEnabled(False)
            self.clock.setText("")
            self.status.setText("Add a scene to start: a grid, a circle, text or an image. The drones take off from "
                                "the charging pads to the venue's grid, fly your scenes, and return to it.")
            return
        site = self.win.doc.to_site_dict()
        try:
            c = compile_design(self.design, site, self._cache)
        except DesignError as e:
            self.compiled = None
            self.view.set_show(None, None)
            self.findings.clear()
            self.status.setText(f"The show cannot be built yet: {e}")
            return
        from idps.core.frames import Frame
        frame = Frame.from_dict(site.get("origin"))
        ned = np.stack([c.P[..., 1], c.P[..., 0], -c.P[..., 2]], -1)
        self.compiled = c
        self._pairs = None
        self.view.set_show(frame.from_ned(ned), np.clip(np.rint(c.C), 0, 255).astype(np.uint8))
        self.slider.blockSignals(True)
        self.slider.setRange(0, len(c.t) - 1)
        self.slider.blockSignals(False)
        self.slider.setEnabled(True)
        self.play_btn.setEnabled(True)
        self.findings.clear()
        for sev, text, t in c.findings:
            self.findings.addItem(("✖ " if sev == "error" else "⚠ ") + text)
            self.findings.item(self.findings.count() - 1).setData(Qt.ItemDataRole.UserRole, float(t))
        errors = sum(1 for f in c.findings if f[0] == "error")
        warnings = len(c.findings) - errors
        s = c.stats
        self.status.setText(
            f"{self.design.n} drones, {len(self.design.scenes)} scene(s), {s['duration_s']:.0f} s in the air "
            f"(plus take-off and landing). Closest two drones {s['min_separation_m']:.2f} m, fastest {s['max_speed_mps']:.1f} m/s. "
            + ("Ready to simulate or fly." if not errors and not warnings else
               f"{errors} thing(s) to fix" + (f", {warnings} to know about" if warnings else "") + ": see the list under the view."
               if errors else f"Can fly; {warnings} thing(s) to know about: see the list under the view."))
        self.set_frame(min(self.slider.value(), len(c.t) - 1))

    def set_frame(self, k: int):
        c = self.compiled
        if c is None:
            return
        k = int(np.clip(k, 0, len(c.t) - 1))
        d = np.linalg.norm(c.P[:, None, k] - c.P[None, :, k], axis=2)
        np.fill_diagonal(d, np.inf)
        close = np.nonzero((d < self.design.min_sep_m - 1e-3).any(1))[0]
        self.view.set_frame(k, close)
        step = next((lab for lab, _, t0, t1, _ in c.steps if t0 <= c.t[k] <= t1), "")
        self.clock.setText(f"{step}  {c.t[k]:.1f} s")

    def _finding_clicked(self, item):
        t = item.data(Qt.ItemDataRole.UserRole)
        if self.compiled is not None and t is not None:
            self.slider.setValue(int(round(float(t) * RATE_HZ)))

    def toggle_play(self):
        if self.timer.isActive():
            self.timer.stop()
            self.play_btn.setText("Play")
            return
        if self.compiled is None:
            return
        if self.slider.value() >= self.slider.maximum():
            self.slider.setValue(0)
        self._last_tick, self._acc = None, 0.0
        self.timer.start()
        self.play_btn.setText("Pause")

    def _tick(self):
        now = time.perf_counter()
        dt = min(now - self._last_tick, 0.25) if self._last_tick is not None else PLAY_MS / 1000
        self._last_tick = now
        self._acc += dt * RATE_HZ
        step = int(self._acc)
        if step:
            self._acc -= step
            k = min(self.slider.value() + step, self.slider.maximum())
            self.slider.setValue(k)
            if k >= self.slider.maximum():
                self.toggle_play()

    # ------------------------------------------------------------ files
    def choose_open(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open a show design", str(ROOT / "out"), "Show designs (*.yaml *.yml)")
        if path:
            self.open_design(path)

    def open_design(self, path) -> bool:
        try:
            d = Design.load(path)
        except Exception as e:
            self.status.setText(f"Could not open {Path(path).name}: {e}")
            return False
        self.design, self.path = d, Path(path)
        self._loading = True
        self.n_drones.setValue(d.n)
        self.s_sep.setValue(d.min_sep_m)
        self.s_speed.setValue(d.speed_mps)
        self.s_facing.setValue(-1.0 if d.facing_deg is None else d.facing_deg)
        self._loading = False
        self._refill_list(0)
        self.rebuild()
        return True

    def choose_save(self):
        start = str(self.path or ROOT / "out" / "show_design.yaml")
        path, _ = QFileDialog.getSaveFileName(self, "Save the show design", start, "Show designs (*.yaml)")
        if path:
            self.save_design(path)

    def save_design(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.design.save(path)
        self.path = path
        self.status.setText(f"Saved {path.name}.")

    def export_show(self, path=None) -> Path | None:
        """The show file (CSV ZIP) of the design as built; None (and why, in the status) if it cannot fly."""
        self.rebuild()
        c = self.compiled
        if c is None:
            return None
        if not c.ok:
            self.status.setText("Fix what is marked ✖ under the view first: " + next(f[1] for f in c.findings if f[0] == "error"))
            return None
        if path is None:
            folder = ROOT / "out" / "designs"
            folder.mkdir(parents=True, exist_ok=True)
            path = folder / f"{(self.path.stem if self.path else 'design')}-{time.strftime('%Y%m%d-%H%M%S')}.zip"
        c.write_csv_zip(path)
        return Path(path)

    def choose_export(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export the show file", str(ROOT / "out" / "show.zip"), "Show files (*.zip)")
        if path and self.export_show(path) is not None:
            self.status.setText(f"Wrote {Path(path).name}: a show file for Simulate, PX4 flight and idps-show.")

    def send_to_simulate(self):
        path = self.export_show()
        if path is not None:
            self.win.sim.set_show(str(path))
            self.win.tabs.setCurrentWidget(self.win.sim)
            self.win.sim.status.setText(f"Show from the Designer: {path.name}. Press Run simulation.")

    def send_to_px4(self):
        path = self.export_show()
        if path is not None:
            self.win.px4.set_show_file(str(path))
            self.win.tabs.setCurrentWidget(self.win.px4)
            self.win.px4.status.setText(f"Show from the Designer: {path.name}. Press Fly the show.")

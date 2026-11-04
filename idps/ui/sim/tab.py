"""Simulate tab: run the full simulation on the layout being planned.

The planner's cameras film simulated drones (a formation, random paths, or your show file);
the camera stations' packets go through the real positioning engine; the result is scored
against the truth and replayed in 3D, with each camera's image alongside. Use it after
placing the cameras, before buying or installing anything.
"""
from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QAbstractItemView, QComboBox, QDoubleSpinBox, QFileDialog, QHBoxLayout, QHeaderView,
                               QLabel, QPushButton, QSlider, QSpinBox, QSplitter, QTableWidget, QTableWidgetItem,
                               QVBoxLayout, QWidget)

from idps.sim.fromplan import Cancelled, SimSettings, simulate_plan
from idps.ui.flow import flow_row
from idps.ui.planner.results import Tile
from idps.ui.planner.worker import start_fn_worker
from idps.ui.sim.view import CameraImage, SimView3D

PLAY_MS = 33
CALIBRATION = [("Calibration: target (0.05 mrad, 1 mm)", 0.05, 1.0), ("Calibration: looser (0.1 mrad, 2 mm)", 0.1, 2.0),
               ("Calibration: rough (0.3 mrad, 5 mm)", 0.3, 5.0)]


def _mm(v: float) -> str:
    return "–" if not np.isfinite(v) else f"{v:.1f} mm"


class SimTab(QWidget):
    def __init__(self, window, parent=None):
        super().__init__(parent)
        self.win = window
        self.res = None
        self.site = None
        self._running = []                      # (thread, worker) kept until finished
        self._worker = None
        self.show_path: str | None = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        bar_box, bar = flow_row()
        self.run_btn = QPushButton("Run simulation")
        self.run_btn.setToolTip("Film simulated drones with the planned cameras and run the positioning engine")
        self.run_btn.clicked.connect(self.run)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.setEnabled(False)
        self.cancel_btn.clicked.connect(self.cancel)
        bar.addWidget(self.run_btn)
        bar.addWidget(self.cancel_btn)
        bar.addSpacing(10)
        bar.addWidget(QLabel("Drones:"))
        self.mode = QComboBox()
        self.mode.addItem("Formation", "formation")
        self.mode.addItem("Random paths", "random")
        self.mode.addItem("Show file", "show")
        self.mode.currentIndexChanged.connect(self._mode_changed)
        bar.addWidget(self.mode)
        self.n_drones = QSpinBox()
        self.n_drones.setRange(1, 100)
        self.n_drones.setValue(12)
        self.n_drones.setSuffix(" drones")
        bar.addWidget(self.n_drones)
        self.duration = QDoubleSpinBox()
        self.duration.setRange(5, 600)
        self.duration.setValue(30)
        self.duration.setDecimals(0)
        self.duration.setSuffix(" s")
        bar.addWidget(self.duration)
        self.show_btn = QPushButton("Choose show…")
        self.show_btn.setToolTip("A Skybrush .skyc or a CSV zip from Blender, placed at the venue origin (ENU)")
        self.show_btn.clicked.connect(self.choose_show)
        bar.addWidget(self.show_btn)
        bar.addSpacing(10)
        self.calib = QComboBox()
        for label, *_ in CALIBRATION:
            self.calib.addItem(label)
        self.calib.setCurrentIndex(0)
        self.calib.setToolTip("How well the cameras' poses are known after calibration")
        bar.addWidget(self.calib)
        lay.addWidget(bar_box)
        self.status = QLabel("Place the cameras in the Planner tab, then run a simulation: simulated cameras film "
                             "the drones, the positioning engine finds them, and you see how accurate it is.")
        self.status.setWordWrap(True)
        lay.addWidget(self.status)

        split = QSplitter(Qt.Orientation.Horizontal)
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        self.view = SimView3D(window.doc, window.canvas)
        ll.addWidget(self.view, 1)
        tl = QHBoxLayout()
        self.play_btn = QPushButton("Play")
        self.play_btn.setEnabled(False)
        self.play_btn.clicked.connect(self.toggle_play)
        self.speed = QComboBox()
        for label, v in (("1×", 1), ("0.25×", 0.25), ("4×", 4)):
            self.speed.addItem(label, v)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setEnabled(False)
        self.slider.valueChanged.connect(self.set_frame)
        self.clock = QLabel("")
        self.clock.setMinimumWidth(90)
        for w in (self.play_btn, self.speed):
            tl.addWidget(w)
        tl.addWidget(self.slider, 1)
        tl.addWidget(self.clock)
        ll.addLayout(tl)
        split.addWidget(left)

        side = QWidget()
        sl = QVBoxLayout(side)
        sl.setContentsMargins(6, 0, 0, 0)
        tiles_box, tiles = flow_row()
        self.t_acc = Tile("Worst drone, p95")
        self.t_ids = Tile("Identified")
        self.t_avail = Tile("Tracked")
        self.t_speed = Tile("Engine speed")
        for t in (self.t_acc, self.t_ids, self.t_avail, self.t_speed):
            t.setMinimumWidth(120)
            tiles.addWidget(t)
        sl.addWidget(tiles_box)
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(["Drone", "Median", "p95", "Worst", "Tracked", "Named after"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.itemSelectionChanged.connect(self._drone_selected)
        sl.addWidget(self.table, 1)
        cam_row = QHBoxLayout()
        cam_row.addWidget(QLabel("Camera image:"))
        self.cam_pick = QComboBox()
        self.cam_pick.currentIndexChanged.connect(lambda _: self._refresh_image())
        cam_row.addWidget(self.cam_pick, 1)
        sl.addLayout(cam_row)
        self.image = CameraImage()
        sl.addWidget(self.image, 1)
        self.image_note = QLabel("")
        self.image_note.setStyleSheet("color: palette(placeholder-text); font-size: 11px;")
        sl.addWidget(self.image_note)
        split.addWidget(side)
        split.setStretchFactor(0, 1)
        split.setSizes([880, 460])
        lay.addWidget(split, 1)

        self.timer = QTimer(self)
        self.timer.setInterval(PLAY_MS)
        self.timer.timeout.connect(self._tick)
        self._acc = 0.0
        self._mode_changed()

    # ------------------------------------------------------------ settings
    def _mode_changed(self):
        show = self.mode.currentData() == "show"
        self.n_drones.setVisible(not show)
        self.duration.setVisible(not show)
        self.show_btn.setVisible(show)

    def choose_show(self):
        path, _ = QFileDialog.getOpenFileName(self, "Choose a show", "", "Shows (*.skyc *.zip *.csv);;All files (*)")
        if path:
            self.set_show(path)

    def set_show(self, path: str):
        from pathlib import Path
        self.show_path = path
        self.show_btn.setText(Path(path).name)
        self.mode.setCurrentIndex(self.mode.findData("show"))

    def settings(self) -> SimSettings:
        _, rot, pos = CALIBRATION[self.calib.currentIndex()]
        return SimSettings(drones=self.mode.currentData(), n_drones=self.n_drones.value(),
                           duration_s=self.duration.value(), show_path=self.show_path, calib_rot_mrad=rot,
                           calib_pos_mm=pos)

    def _context(self):
        if self.win.canvas.result is None:
            self.status.setText("Plan the cameras first (Planner tab → Plan, ⌘R). The simulation films with the "
                                "planned layout.")
            return None
        from idps.plan.site import site_from_dict
        try:
            site = site_from_dict(self.win.doc.to_site_dict(), str(self.win.doc.path) if self.win.doc.path else None)
        except Exception as exc:
            self.status.setText(f"The venue has a problem: {exc}")
            return None
        return site, list(self.win.canvas.result[0].cameras)

    # ------------------------------------------------------------ run
    def run(self) -> bool:
        if self._worker is not None:
            return False
        ctx = self._context()
        if ctx is None:
            return False
        s = self.settings()
        if s.drones == "show" and not s.show_path:
            self.status.setText("Choose a show file first.")
            return False
        site, cams = ctx
        self.site = site

        def fn(progress, cancel):
            try:
                return simulate_plan(site, cams, s, progress, cancel)
            except Cancelled:
                return None
        thread, worker = start_fn_worker(self, fn)
        self._worker = worker
        self._running.append((thread, worker))
        worker.progress.connect(self.status.setText)
        worker.finished.connect(self._done)
        worker.failed.connect(self._failed)
        thread.finished.connect(self._release_finished)
        self.run_btn.setEnabled(False)
        self.cancel_btn.setEnabled(True)
        self.status.setText(f"Simulating {len(cams)} cameras…")
        return True

    def cancel(self):
        if self._worker is not None:
            self._worker.stop()
            self.status.setText("Cancelling…")

    def _finish_run(self):
        self._worker = None
        self.run_btn.setEnabled(True)
        self.cancel_btn.setEnabled(False)

    def _done(self, res):
        self._finish_run()
        if res is None:
            self.status.setText("Cancelled.")
            return
        self.show_result(res)

    def _failed(self, msg):
        self._finish_run()
        self.status.setText(f"The simulation failed: {msg.splitlines()[0]}")

    def _release_finished(self):
        for keep in [k for k in self._running if k[0].isFinished()]:
            self._running.remove(keep)
            keep[0].deleteLater()

    def wait(self, timeout_s: float = 600.0):
        """Block until a running simulation is done (tests, headless)."""
        import time

        from PySide6.QtWidgets import QApplication
        t0 = time.time()
        while self._worker is not None and time.time() - t0 < timeout_s:
            QApplication.processEvents()
            time.sleep(0.01)
        QApplication.processEvents()

    # ------------------------------------------------------------ results
    def show_result(self, res):
        self.res = res
        req = self.site.requirements.p95_mm if self.site is not None else 10.0
        self.view.set_result(res, req)
        worst = res.worst_p95_mm
        n = len(res.drones)
        self.t_acc.set(_mm(worst), f"requirement {req:g} mm", bool(np.isfinite(worst) and worst <= req))
        ids = res.identified
        wrong = sum(d.wrong for d in res.drones) + res.unknown_ids
        self.t_ids.set(f"{ids}/{n}", f"{wrong} wrong" if wrong else "no wrong IDs", ids == n and wrong == 0)
        avail = min((d.available for d in res.drones), default=0.0)
        self.t_avail.set(f"{100 * avail:.1f} %", "worst drone, once named", avail >= 0.99)
        flown = res.t[-1] if len(res.t) else 0.0
        self.t_speed.set(f"{flown / max(res.engine_s, 1e-9):.0f}× real time",
                         f"filming took {res.sim_s:.0f} s")
        self.table.setRowCount(n)
        for row, d in enumerate(res.drones):
            vals = [str(d.drone_id), _mm(d.median_mm), _mm(d.p95_mm), _mm(d.max_mm), f"{100 * d.available:.1f} %",
                    "never" if not np.isfinite(d.identified_s) else f"{d.identified_s:.1f} s"]
            for c, v in enumerate(vals):
                it = QTableWidgetItem(v)
                it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter if c else
                                    Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
                self.table.setItem(row, c, it)
        self.cam_pick.blockSignals(True)
        self.cam_pick.clear()
        for c in res.cameras:
            self.cam_pick.addItem(c.id)
        self.cam_pick.blockSignals(False)
        self.slider.blockSignals(True)
        self.slider.setRange(0, len(res.t) - 1)
        self.slider.blockSignals(False)
        self.slider.setEnabled(True)
        self.play_btn.setEnabled(True)
        start = res.show_start_s or 0.0
        k0 = int(min(len(res.t) - 1, max(0.0, start) * len(res.t) / max(res.t[-1], 1e-9)))
        self.set_frame(k0)
        self.slider.setValue(k0)
        verdict = ("meets" if np.isfinite(worst) and worst <= req and ids == n and wrong == 0 else "does not meet")
        self.status.setText(f"This layout {verdict} the requirement in simulation: worst drone p95 {_mm(worst)} "
                            f"(requirement {req:g} mm), {ids} of {n} drones identified, {wrong} wrong IDs, "
                            f"{len(res.cameras)} cameras, {flown:.0f} s flown. Press Play to watch it, pick a drone "
                            "to see which cameras have it.")
        if not self.timer.isActive():
            self.toggle_play()

    def toggle_play(self):
        if self.res is None:
            return
        if self.timer.isActive():
            self.timer.stop()
            self.play_btn.setText("Play")
        else:
            if self.slider.value() >= self.slider.maximum():
                self.slider.setValue(0)
            self.timer.start()
            self.play_btn.setText("Pause")

    def _tick(self):
        hz = (len(self.res.t) - 1) / max(self.res.t[-1], 1e-9)
        self._acc += PLAY_MS / 1000 * hz * float(self.speed.currentData())
        step = int(self._acc)
        if step:
            self._acc -= step
            k = min(self.slider.value() + step, self.slider.maximum())
            self.slider.setValue(k)
            if k >= self.slider.maximum():
                self.toggle_play()

    def set_frame(self, k: int):
        if self.res is None:
            return
        self.view.set_frame(k)
        t = self.res.t[k]
        if self.res.show_start_s is not None:
            ts = t - self.res.show_start_s
            self.clock.setText(("show T−" if ts < 0 else "show T+") + f"{abs(ts):.1f} s")
        else:
            self.clock.setText(f"{t:.1f} s")
        self._refresh_image()

    def _drone_selected(self):
        rows = self.table.selectionModel().selectedRows()
        sel = rows[0].row() if rows else None
        if self.res is not None and sel is not None:
            did = self.res.drones[sel].drone_id
            sel = self.res.drone_ids.index(did)
            self._pick_camera_for(sel)
        self.view.selected = sel
        self.view.update()
        self._refresh_image()

    def _pick_camera_for(self, i: int):
        """Show the image of the nearest camera that has the drone in frame."""
        from idps.sim.fromplan import marker_in_view
        k = self.view.frame
        m = self.res.marker[i, k]
        best, dist = None, math.inf
        for j, st in enumerate(self.res.stations):
            if np.isfinite(marker_in_view(st, m)[0, 0]):
                d = np.linalg.norm(st.pos - m)
                if d < dist:
                    best, dist = j, d
        if best is not None:
            self.cam_pick.setCurrentIndex(best)

    def _refresh_image(self):
        if self.res is None or self.cam_pick.currentIndex() < 0:
            return
        j, k = self.cam_pick.currentIndex(), self.view.frame
        self.image.set_state(self.res, j, k, self.view.selected)
        from idps.sim.fromplan import marker_in_view
        uv = marker_in_view(self.res.stations[j], self.res.marker[:, k])
        n = int(np.isfinite(uv[:, 0]).sum())
        self.image_note.setText(f"{self.res.cameras[j].id}: {n} of {len(self.res.drone_ids)} markers in frame "
                                f"(1280 × 800 sensor; the dots blink each drone's ID)")

    def shutdown(self):
        self.timer.stop()
        for thread, worker in list(self._running):
            worker.stop()
            thread.quit()
            thread.wait(10000)

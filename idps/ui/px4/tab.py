"""PX4 flight tab: fly the show with PX4 simulator drones and watch how closely they follow it.

Run starts px4/sitl/run_loop.sh (60 g SIH quads flying the show files on IDPS positions) and
follows the flight live; Open run replays a finished one. What is shown is the idps_show
module's own measure: how far each drone was from its file at its own show time.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import numpy as np
from PySide6.QtCore import QProcess, QProcessEnvironment, QSettings, Qt, QTimer
from PySide6.QtWidgets import (QAbstractItemView, QComboBox, QFileDialog, QHBoxLayout, QHeaderView, QLabel,
                               QPlainTextEdit, QPushButton, QSlider, QSpinBox, QSplitter, QTableWidget,
                               QTableWidgetItem, QVBoxLayout, QWidget)

from idps.sim.px4run import REPLAY_HZ, load_run
from idps.ui.flow import flow_row
from idps.ui.planner.results import Tile
from idps.ui.px4.view import ErrorGraph, FlightView3D

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "px4" / "sitl" / "run_loop.sh"
VENUE = ROOT / "examples" / "sites" / "show_8x8.yaml"   # where run_loop.sh flies the show
LIMIT_MM = 30.0
PLAY_MS = 40
MODES = [("Positions inside PX4 (quick check)", "vision", "px4_sitl_default"),
         ("Simulated cameras + engine (full loop)", "cameras", "px4_sitl_nolockstep")]


def _mm(v: float) -> str:
    return "–" if not np.isfinite(v) else f"{v:.1f} mm"


class Px4Tab(QWidget):
    def __init__(self, window, parent=None):
        super().__init__(parent)
        self.win = window
        self.run_data = None
        self.proc: QProcess | None = None
        self.out_dir: Path | None = None
        self.settings = QSettings("IDPS", "dashboard")

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        bar_box, bar = flow_row()
        self.run_btn = QPushButton("Fly the show")
        self.run_btn.setToolTip("Start PX4 simulator drones (60 g), upload the show and fly it (px4/sitl/run_loop.sh)")
        self.run_btn.clicked.connect(self.start)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop)
        self.open_btn = QPushButton("Open run…")
        self.open_btn.setToolTip("Replay a finished flight (a run_loop.sh output folder)")
        self.open_btn.clicked.connect(self.choose_run)
        for w in (self.run_btn, self.stop_btn, self.open_btn):
            bar.addWidget(w)
        bar.addSpacing(10)
        self.mode = QComboBox()
        for label, *_ in MODES:
            self.mode.addItem(label)
        self.mode.setToolTip("Quick check: IDPS-like positions (2 mm noise, 120 Hz, 28 ms late) go straight into "
                             "PX4. Full loop: 11 simulated cameras and the positioning engine (needs a fast machine)")
        bar.addWidget(self.mode)
        self.n_drones = QSpinBox()
        self.n_drones.setRange(1, 5)
        self.n_drones.setValue(5)
        self.n_drones.setSuffix(" drones")
        bar.addWidget(self.n_drones)
        self.px4_btn = QPushButton()
        self.px4_btn.setToolTip("Your PX4-Autopilot folder, built with the IDPS module (px4/README.md)")
        self.px4_btn.clicked.connect(self.choose_px4)
        bar.addWidget(self.px4_btn)
        self._set_px4_dir(self.settings.value("px4/dir", str(Path.home() / "PX4-Autopilot")))
        lay.addWidget(bar_box)
        self.status = QLabel("Fly the 5-drone show (examples/shows) with PX4 simulator drones, or open a finished "
                             "run. Each drone is coloured by how far it is from its show file.")
        self.status.setWordWrap(True)
        lay.addWidget(self.status)

        split = QSplitter(Qt.Orientation.Horizontal)
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        self.view = FlightView3D(window.doc, window.canvas)
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
        tl.addWidget(self.play_btn)
        tl.addWidget(self.speed)
        tl.addWidget(self.slider, 1)
        tl.addWidget(self.clock)
        ll.addLayout(tl)
        self.graph = ErrorGraph()
        ll.addWidget(self.graph)
        split.addWidget(left)

        side = QWidget()
        sl = QVBoxLayout(side)
        sl.setContentsMargins(6, 0, 0, 0)
        tiles_box, tiles = flow_row()
        self.t_worst = Tile("Worst moment, all drones")
        self.t_median = Tile("Median, all drones")
        self.t_within = Tile(f"Drones within {LIMIT_MM:g} mm")
        for t in (self.t_worst, self.t_median, self.t_within):
            t.setMinimumWidth(130)
            tiles.addWidget(t)
        sl.addWidget(tiles_box)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Drone", "Median", "p95", "Worst", "Worst at"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.itemSelectionChanged.connect(self._drone_selected)
        self.table.cellDoubleClicked.connect(self._jump_to_worst)
        sl.addWidget(self.table, 1)
        sl.addWidget(QLabel("Log:"))
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        f = self.log.font()
        f.setFamily("Menlo")
        f.setPointSizeF(f.pointSizeF() * 0.9)
        self.log.setFont(f)
        sl.addWidget(self.log, 1)
        split.addWidget(side)
        split.setStretchFactor(0, 1)
        split.setSizes([880, 460])
        lay.addWidget(split, 1)

        self.timer = QTimer(self)
        self.timer.setInterval(PLAY_MS)
        self.timer.timeout.connect(self._tick)
        self._acc = 0.0
        self.poll = QTimer(self)                       # while flying: reload the tracking log
        self.poll.setInterval(1000)
        self.poll.timeout.connect(self._poll)

    # ------------------------------------------------------------ settings
    def _set_px4_dir(self, path: str):
        self.px4_dir = str(path)
        ok = (Path(path) / "build").exists()
        self.px4_btn.setText(f"PX4: {Path(path).name}" + ("" if ok else " (not built)"))
        self.settings.setValue("px4/dir", self.px4_dir)

    def choose_px4(self):
        d = QFileDialog.getExistingDirectory(self, "PX4-Autopilot folder", self.px4_dir)
        if d:
            self._set_px4_dir(d)

    def choose_run(self):
        start = str(ROOT / "out")
        d = QFileDialog.getExistingDirectory(self, "A PX4 flight folder (run_loop.sh output)", start)
        if d:
            self.open_run(d)

    # ------------------------------------------------------------ running
    def start(self) -> bool:
        if self.proc is not None:
            return False
        if not SCRIPT.exists():
            self.status.setText(f"{SCRIPT} is missing: this IDPS checkout has no PX4 flight script.")
            return False
        _, mode, build = MODES[self.mode.currentIndex()]
        if not (Path(self.px4_dir) / "build" / build / "bin" / "px4").exists():
            self.status.setText(f"PX4 is not built in {self.px4_dir} (no build/{build}): build it as px4/README.md "
                                "says, or pick your PX4-Autopilot folder.")
            return False
        self.out_dir = ROOT / "out" / "px4_runs" / time.strftime("%Y%m%d-%H%M%S")
        env = QProcessEnvironment.systemEnvironment()
        env.insert("PATH", str(Path(sys.executable).parent) + os.pathsep + env.value("PATH"))
        env.insert("PX4_DIR", self.px4_dir)
        env.insert("PX4_BUILD", build)
        env.insert("N", str(self.n_drones.value()))
        env.insert("OUT", str(self.out_dir))
        self.proc = QProcess(self)
        self.proc.setProcessEnvironment(env)
        self.proc.setWorkingDirectory(str(ROOT))
        self.proc.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._read_output)
        self.proc.finished.connect(self._finished)
        self.log.clear()
        self.log.appendPlainText(f"$ PX4_DIR={self.px4_dir} PX4_BUILD={build} N={self.n_drones.value()} "
                                 f"px4/sitl/run_loop.sh {mode}")
        self.proc.start("bash", [str(SCRIPT), mode])
        self.run_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.status.setText(f"Starting {self.n_drones.value()} PX4 drones… (about 2–3 minutes: drones, show upload, "
                            "then the 63 s flight, shown live)")
        self.poll.start()
        return True

    def stop(self):
        if self.proc is None:
            return
        self.status.setText("Stopping: the drones and simulators are shut down…")
        self.proc.terminate()
        if not self.proc.waitForFinished(8000):
            self.proc.kill()

    def _read_output(self):
        text = bytes(self.proc.readAllStandardOutput()).decode(errors="replace").rstrip()
        if text:
            self.log.appendPlainText(text)

    def _finished(self, code, _status):
        self.poll.stop()
        self._read_output()
        self.proc = None
        self.run_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        if self.out_dir is not None and (self.out_dir / "show" / "manifest.json").exists():
            self.open_run(self.out_dir, finished=True)
        if code != 0:
            self.status.setText(self.status.text() + f"  (The flight script ended with code {code}: see the log.)")

    def _poll(self):
        if self.out_dir is None or not (self.out_dir / "show" / "manifest.json").exists():
            return
        try:
            run = load_run(self.out_dir)
        except Exception:                       # files still being written
            return
        if not np.isfinite(run.last_t):
            return
        self.show_run(run, live=True)

    # ------------------------------------------------------------ results
    def open_run(self, folder, finished: bool = False) -> bool:
        try:
            run = load_run(folder)
        except Exception as e:
            self.status.setText(f"Could not open {folder}: {e}")
            return False
        self.show_run(run, live=False)
        if not np.isfinite(run.last_t):
            self.status.setText(f"{Path(folder).name}: no flight in this run (the drones did not start). See the log"
                                + ("" if finished else f" in {folder}") + ".")
        return True

    def show_run(self, run, live: bool):
        first = self.run_data is None or self.run_data.folder != run.folder
        doc = self.win.doc
        if first and doc.path is None and doc.undo.isClean() and VENUE.exists():
            self.win.open_doc(str(VENUE))              # nothing open: show the show floor these runs fly in
        self.run_data = run
        self.view.set_run(run, LIMIT_MM)
        worst = run.worst_mm
        med = [d.median_mm for d in run.drones if np.isfinite(d.median_mm)]
        within = sum(1 for d in run.drones if np.isfinite(d.worst_mm) and d.worst_mm <= LIMIT_MM)
        flown = sum(1 for d in run.drones if d.samples)
        n = len(run.drones)
        self.t_worst.set(_mm(worst), f"target {LIMIT_MM:g} mm", bool(np.isfinite(worst) and worst <= LIMIT_MM))
        self.t_median.set(_mm(float(np.median(med))) if med else "–", "typical distance from the file")
        self.t_within.set(f"{within}/{n}", f"{flown} flew", within == n)
        sel = self.view.selected
        self.table.blockSignals(True)
        self.table.setRowCount(n)
        for row, d in enumerate(run.drones):
            vals = [str(d.drone_id), _mm(d.median_mm), _mm(d.p95_mm), _mm(d.worst_mm),
                    "–" if not np.isfinite(d.worst_at_s) else f"{d.worst_at_s:.1f} s"]
            for c, v in enumerate(vals):
                it = QTableWidgetItem(v)
                it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter if c else
                                    Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
                self.table.setItem(row, c, it)
        if sel is not None and sel < n:
            self.table.selectRow(sel)
        self.table.blockSignals(False)
        self.slider.blockSignals(True)
        self.slider.setRange(0, len(run.t) - 1)
        self.slider.blockSignals(False)
        self.slider.setEnabled(True)
        self.play_btn.setEnabled(True)
        if live:                                        # follow the newest sample
            k = int(np.clip(np.searchsorted(run.t, run.last_t), 0, len(run.t) - 1))
            self.slider.setValue(k)
            self.set_frame(k)
            how = "positions inside PX4" if run.mode == "vision" else "simulated cameras and engine"
            self.status.setText(f"Flying ({how}): show time {run.last_t:.1f} s. Worst so far {_mm(worst)}, "
                                f"{within} of {n} drones within {LIMIT_MM:g} mm.")
            return
        if first:
            self.slider.setValue(0)
        self.set_frame(self.slider.value())
        verdict = (f"every drone stayed within {LIMIT_MM:g} mm of its file" if within == n else
                   f"{within} of {n} drones stayed within {LIMIT_MM:g} mm of their files")
        how = {"vision": "positions inside PX4", "cameras": "simulated cameras and engine"}.get(run.mode, "")
        self.status.setText(f"{run.folder.name} ({how}): {verdict}; worst moment {_mm(worst)}. Press Play to watch "
                            "it; pick a drone to see its path, double-click it to jump to its worst moment.")

    def toggle_play(self):
        if self.run_data is None:
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
        self._acc += PLAY_MS / 1000 * REPLAY_HZ * float(self.speed.currentData())
        step = int(self._acc)
        if step:
            self._acc -= step
            k = min(self.slider.value() + step, self.slider.maximum())
            self.slider.setValue(k)
            if k >= self.slider.maximum():
                self.toggle_play()

    def set_frame(self, k: int):
        if self.run_data is None:
            return
        self.view.set_frame(k)
        self.clock.setText(f"show {self.run_data.t[min(k, len(self.run_data.t) - 1)]:.1f} s")
        self.graph.set_state(self.run_data, self.view.selected, k, LIMIT_MM)

    def _drone_selected(self):
        rows = self.table.selectionModel().selectedRows()
        self.view.selected = rows[0].row() if rows else None
        self.view.update()
        self.graph.set_state(self.run_data, self.view.selected, self.view.frame, LIMIT_MM)

    def _jump_to_worst(self, row, _col):
        d = self.run_data.drones[row]
        if np.isfinite(d.worst_at_s):
            k = int(np.clip(np.searchsorted(self.run_data.t, d.worst_at_s), 0, len(self.run_data.t) - 1))
            self.slider.setValue(k)

    def shutdown(self):
        self.timer.stop()
        self.poll.stop()
        if self.proc is not None:
            self.proc.finished.disconnect()
            self.proc.terminate()
            if not self.proc.waitForFinished(8000):
                self.proc.kill()

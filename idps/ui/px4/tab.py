"""PX4 flight tab: fly the show with PX4 simulator drones and watch how closely they follow it.

Run starts px4/sitl/run_loop.sh (60 g SIH quads flying the show files on IDPS positions) and
follows the flight live; Open run replays a finished one. What is shown is the idps_show
module's own measure: how far each drone was from its file at its own show time.
"""
from __future__ import annotations

import os
import re
import signal
import sys
import tempfile
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
LIVE_LAG_S = 0.6               # live view: this far behind the newest sample, so every drone has data there
PLAY_MS = 40
MODES = [("Positions inside PX4 (quick check)", "vision", "px4_sitl_default"),
         ("Simulated cameras + engine (full loop)", "cameras", None)]      # None: camera_build()
SIH_SOURCE = Path("src/modules/simulation/simulator_sih/sih.cpp")


KEEP_RUNS = 20                 # the dashboard keeps this many flights in out/px4_runs (10-20 MB each)


def prune_runs(folder, keep: int = KEEP_RUNS) -> int:
    """Delete all but the newest `keep` run folders (named by start time); returns how many went."""
    import shutil
    runs = sorted(p for p in Path(folder).glob("[0-9]*-[0-9]*") if p.is_dir())
    old = runs[:-keep] if keep > 0 else runs
    for p in old:
        shutil.rmtree(p, ignore_errors=True)
    return len(old)


def camera_build(px4_dir) -> str:
    """The PX4 build for the camera loop. The cameras run on the real clock, so the drones' clocks
    must keep real time: the default (lockstep) build does when px4/sitl/sih_realtime_pacing.patch
    is applied (and is consistent through a host stall); without it, the real-clock build
    (px4_sitl_nolockstep), where a stall jolts the physics instead."""
    try:
        paced = "next_wall_time_us" in (Path(px4_dir) / SIH_SOURCE).read_text(errors="replace")
    except OSError:
        paced = False
    return "px4_sitl_default" if paced else "px4_sitl_nolockstep"


STAGES = [("== show", "building the show files"), ("PX4 drones", "starting the PX4 drones"),
          ("== positions inside", "checking every drone's link"), ("== cameras", "starting cameras and engine"),
          ("== upload", "uploading the show"), ("show starts in", "waiting for the start"),
          ("== result", "scoring")]
FAILURES = [(r"udp port\(s\) (\[.*?\]) are in use", "An earlier run is still going (ports {0} busy). Quit it, or "
             "run: pkill -f 'idps.sim px4'; pkill -f idps_stand_in; pkill -x px4"),
            (r"build first:", "PX4 is not built in the chosen folder (see px4/README.md)."),
            (r"(\S+) cannot import idps", "The flight script's Python ({0}) cannot import IDPS: start the dashboard "
             "from the IDPS venv."),
            (r"ModuleNotFoundError: No module named '([\w.]+)'", "A Python package is missing: {0} (pip install it in "
             "the IDPS venv)."),
            (r"earlier PX4 processes would not stop \(pids: ([\d ]+)\)", "PX4s from an earlier flight would not "
             "stop (pids {0}): run kill -9 {0} in a terminal, then fly again."),
            (r"PX4 server already running for instance (\d+)", "A PX4 from an earlier flight was still running "
             "(instance {0}): fly again; if it repeats, run pkill -9 -x px4 in a terminal."),
            (r"PX4 drone (\d+) \(instance \d+\): (.*?)\. The end of its log", "PX4 drone {0}: {1} (its log is "
             "in the Log pane)."),
            (r"No data from drone\(s\) ([\d ]+) on their truth links", "No data from drone(s) {0}: their links never "
             "reached the logger."),
            (r"drone (\d+): no heartbeat on (\S+)", "Drone {0} did not answer on {1}."),
            (r"Too much for this computer: the simulated cameras run (\d+) ms behind", "Too much for this computer: "
             "the simulated cameras ran {0} ms behind the clock. Fly fewer drones, or 'Positions inside PX4'."),
            (r"Not enough free disk space", "The disk is nearly full: free some space (see the Log pane) and fly "
             "again."),
            (r"No space left on device", "The disk is full: free some space (see the Log pane) and fly again."),
            (r"Upload failed: those drones have no show file", "The show upload failed for some drones (the "
             "FAILED lines in the Log pane): fly again."),
            (r"Not started: every drone was told to abort", "Not started: a drone refused the start, so every drone "
             "was told to abort.")]
DRONE_NOTES = [(r"drone (\d+): (?:denied|temporarily rejected|failed|unsupported) \(\w+\) ?(.*)", "refused: {1}"),
               (r"drone (\d+): nothing on udp (\d+)", "no data (udp {1} silent)"),
               (r"^\s+drone (\d+): \S+\s+\[(\w+)\] (Failsafe.*|IDPS show stopped.*)", "{2}")]


def diagnose(text: str):
    """The flight script's output -> (stage, failure reason or None, {drone ID: note})."""
    stage, reason, rank, notes = "", None, len(FAILURES), {}
    for line in text.splitlines():
        for key, name in STAGES:
            if key in line:
                stage = name
        for r, (pat, msg) in enumerate(FAILURES[:rank]):     # the first match; a more specific one wins
            m = re.search(pat, line)
            if m:
                reason, rank = msg.format(*[g.strip() for g in m.groups()]), r
                break
        for pat, msg in DRONE_NOTES:
            m = re.search(pat, line)
            if m:
                notes.setdefault(int(m.group(1)), msg.format(*m.groups()).strip())
    return stage, reason, notes


def _python_shim() -> str:
    """A folder whose `python` runs this interpreter: the flight script calls `python`, and a dashboard
    started from Finder or the Dock has no activated venv (macOS has no /usr/bin/python). A wrapper
    script, not a symlink: a venv's python reached through a link elsewhere loses its venv."""
    d = Path(tempfile.gettempdir()) / f"idps-python-{os.getuid()}"
    d.mkdir(exist_ok=True)
    shim = d / "python"
    text = f'#!/bin/sh\nexec "{sys.executable}" "$@"\n'
    if shim.is_symlink() or not shim.exists() or shim.read_text() != text:
        shim.unlink(missing_ok=True)
        shim.write_text(text)
        shim.chmod(0o755)
    return str(d)


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
        self.n_drones.setRange(1, 20)             # up to 5: the 5-drone show; 6-20: the 20-drone show
        self.n_drones.setValue(5)
        self.n_drones.setToolTip("Up to 5: the 5-drone sphere/square show (63 s). 6–20: the 20-drone "
                                 "rings/wall show (117 s). Each PX4 drone takes about a third of a CPU core.")
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
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(["Drone", "State", "Median", "p95", "Worst", "Worst at"])
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
        self.poll.setInterval(500)
        self.live = QTimer(self)                       # while flying: move the view on smoothly between reloads
        self.live.setInterval(PLAY_MS)
        self.live.timeout.connect(self._live_tick)
        self._live_t = None
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
        build = build or camera_build(self.px4_dir)
        if not (Path(self.px4_dir) / "build" / build / "bin" / "px4").exists():
            self.status.setText(f"PX4 is not built in {self.px4_dir} (no build/{build}): build it as px4/README.md "
                                "says, or pick your PX4-Autopilot folder.")
            return False
        prune_runs(ROOT / "out" / "px4_runs", keep=KEEP_RUNS)
        self.out_dir = ROOT / "out" / "px4_runs" / time.strftime("%Y%m%d-%H%M%S")
        env = QProcessEnvironment.systemEnvironment()
        env.insert("PATH", _python_shim() + os.pathsep + env.value("PATH"))
        env.insert("PYTHON", sys.executable)
        env.insert("PX4_DIR", self.px4_dir)
        env.insert("PX4_BUILD", build)
        env.insert("N", str(self.n_drones.value()))
        env.insert("OUT", str(self.out_dir))
        self.proc = QProcess(self)
        pp = QProcess.UnixProcessParameters()            # its own process group: Stop reaches every helper
        pp.flags = QProcess.UnixProcessFlag.CreateNewSession
        self.proc.setUnixProcessParameters(pp)
        self._out, self._notes, self._stopping = "", {}, False
        self.proc.setProcessEnvironment(env)
        self.proc.setWorkingDirectory(str(ROOT))
        self.proc.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._read_output)
        self.proc.finished.connect(self._finished)
        self.log.clear()
        self.clear_run()
        self.log.appendPlainText(f"$ PX4_DIR={self.px4_dir} PX4_BUILD={build} N={self.n_drones.value()} "
                                 f"px4/sitl/run_loop.sh {mode}")
        if sys.platform == "darwin":            # keep the Mac awake and the flight at full speed while it runs
            self._begin_activity()
            self.proc.start("/usr/bin/caffeinate", ["-i", "bash", str(SCRIPT), mode])
        else:
            self.proc.start("bash", [str(SCRIPT), mode])
        self.run_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        n = self.n_drones.value()
        self.status.setText(f"Starting {n} PX4 drones… (about {3 if n <= 5 else 4} minutes: drones, show upload, "
                            f"then the {63 if n <= 5 else 117} s flight, shown live)")
        self._live_t = None
        self.poll.start()
        self.live.start()
        return True

    def _begin_activity(self):
        """macOS: tell the system a latency-critical activity is running, so App Nap does not slow the
        dashboard (and the simulators it started) while it is in the background. Needs pyobjc; without
        it, keep the dashboard in front during a flight."""
        try:
            from Foundation import NSProcessInfo
            opts = 0x00FFFFFF | 0xFF00000000                 # NSActivityUserInitiated | NSActivityLatencyCritical
            self._activity = NSProcessInfo.processInfo().beginActivityWithOptions_reason_(opts, "PX4 flight")
        except Exception:
            self._activity = None

    def _end_activity(self):
        act = getattr(self, "_activity", None)
        if act is not None:
            from Foundation import NSProcessInfo
            NSProcessInfo.processInfo().endActivity_(act)
            self._activity = None

    def _signal_group(self, sig, proc=None):
        """Signal the flight script's process group; with proc, only if that run is still the current one."""
        if proc is not None and proc is not self.proc:
            return
        if self.proc is not None and self.proc.processId():
            try:
                os.killpg(self.proc.processId(), sig)
            except (ProcessLookupError, PermissionError):
                pass

    def stop(self):
        if self.proc is None:
            return
        self._stopping = True
        self.status.setText("Stopping: the drones land and the simulators are shut down…")
        self._signal_group(signal.SIGTERM)               # the script's cleanup runs; helpers get it too
        proc = self.proc                                 # only this run: a new one may have started by then
        QTimer.singleShot(12000, lambda: self._signal_group(signal.SIGKILL, proc))

    def _read_output(self):
        text = bytes(self.proc.readAllStandardOutput()).decode(errors="replace").rstrip()
        if not text:
            return
        self.log.appendPlainText(text)
        self._out += text + "\n"
        stage, reason, self._notes = diagnose(self._out)
        if reason:
            self.status.setText(reason)
        elif stage and (self.run_data is None or not np.isfinite(self.run_data.last_t)):
            self.status.setText(f"{stage[0].upper()}{stage[1:]}…")

    def _finished(self, code, _status):
        self.poll.stop()
        self.live.stop()
        self._end_activity()
        self._read_output()
        self.proc = None
        self.run_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        if self.out_dir is not None and (self.out_dir / "show" / "manifest.json").exists():
            self.open_run(self.out_dir, finished=True)
        _, reason, self._notes = diagnose(self._out)
        if self.run_data is not None:
            self._fill_table(self.run_data, live=False)
        if self._stopping:
            self.status.setText("Stopped." + (f" {self.status.text()}" if self.run_data is not None
                                              and np.isfinite(self.run_data.last_t) else ""))
        elif reason:
            self.status.setText(reason)
        elif code != 0:
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

    def clear_run(self):
        """Forget the run on screen (a new flight is starting: its data replaces it as it comes)."""
        self.timer.stop()
        self.play_btn.setText("Play")
        self.run_data, self._live_t = None, None
        self.view.set_run(None, LIMIT_MM)
        self.graph.set_state(None, None, 0, LIMIT_MM)
        self.t_worst.set("–", f"target {LIMIT_MM:g} mm")
        self.t_median.set("–", "typical distance from the file")
        self.t_within.set("–", "")
        self.table.setRowCount(0)
        self.slider.setEnabled(False)
        self.play_btn.setEnabled(False)
        self.clock.setText("")

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
        self._fill_table(run, live)
        self.slider.blockSignals(True)
        self.slider.setRange(0, len(run.t) - 1)
        self.slider.blockSignals(False)
        self.slider.setEnabled(True)
        self.play_btn.setEnabled(True)
        if live:                                        # the view follows the flight (_live_tick)
            if self._live_t is None:
                self._live_t = run.last_t - LIVE_LAG_S
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

    def _state(self, run, d, live: bool) -> str:
        note = getattr(self, "_notes", {}).get(d.drone_id)
        if note:
            return note
        if not d.samples:
            return "waiting" if live else "no data"
        if np.isfinite(run.last_t) and run.last_t - d.last_s > 2.0:
            return f"data stopped at {d.last_s:.0f} s"
        return "flying" if live else "flew"

    def _fill_table(self, run, live: bool):
        sel = self.view.selected
        self.table.blockSignals(True)
        self.table.setRowCount(len(run.drones))
        for row, d in enumerate(run.drones):
            vals = [str(d.drone_id), self._state(run, d, live), _mm(d.median_mm), _mm(d.p95_mm), _mm(d.worst_mm),
                    "–" if not np.isfinite(d.worst_at_s) else f"{d.worst_at_s:.1f} s"]
            for c, v in enumerate(vals):
                it = QTableWidgetItem(v)
                it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter if c > 1 else
                                    Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
                if c == 1:
                    it.setToolTip(v)
                self.table.setItem(row, c, it)
        if sel is not None and sel < len(run.drones):
            self.table.selectRow(sel)
        self.table.blockSignals(False)

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

    def _live_tick(self):
        """Advance the live view with the clock, LIVE_LAG_S behind the newest sample (it arrives in
        bursts every reload); catch up if it falls further behind."""
        run = self.run_data
        if run is None or self._live_t is None or not np.isfinite(run.last_t):
            return
        target = run.last_t - LIVE_LAG_S
        step = PLAY_MS / 1000
        if target - self._live_t > 1.0:
            step *= 1.5
        self._live_t = min(self._live_t + step, target)
        k = int(np.clip(np.searchsorted(run.t, self._live_t, "right") - 1, 0, len(run.t) - 1))
        if k != self.slider.value():
            self.slider.setValue(k)

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
        self.live.stop()
        if self.proc is not None:                         # quitting mid-flight: land and stop everything
            self.proc.finished.disconnect()
            self._signal_group(signal.SIGTERM)
            if not self.proc.waitForFinished(10000):
                self._signal_group(signal.SIGKILL)

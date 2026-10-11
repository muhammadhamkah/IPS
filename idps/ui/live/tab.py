"""Live tracking tab: the show in 3D plus the brain's alert feed.

The brain watches every tick on its own. When it finds a repeated problem it
raises an alert card with the evidence and a recommended fix; the operator only
decides: **Accept fix** or **Live with it**. Until the positioning engine
exists, a simulated show feeds it, and faults can be injected to rehearse.
"""
from __future__ import annotations

import time

import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QAction
from PySide6.QtWidgets import (QComboBox, QFrame, QHBoxLayout, QLabel, QMenu, QPushButton, QScrollArea, QSizePolicy,
                               QSpinBox, QSplitter, QToolButton, QVBoxLayout, QWidget)

from idps.monitor.brain import Brain
from idps.monitor.model import (BLIND_SPOT, CAMERA_MOVED, CAMERA_OFFLINE, FIX_ACCEPTED, LIVED_WITH, OPEN,
                                RESOLVED)
from idps.ui import theme
from idps.ui.flow import flow_row
from idps.ui.live.view import LiveView3D
from idps.ui.planner.results import Tile
from idps.ui.planner.worker import start_fn_worker

TICK_MS = 100
DT = 0.1                                  # show seconds per simulation step

SEVERITY = {"critical": theme.STATUS[0], "warning": theme.STATUS[2]}


def _fmt_t(t: float) -> str:
    m, s = divmod(int(t), 60)
    return f"{m:d}:{s:02d}"


class IssueCard(QFrame):
    acceptRequested = Signal(str)
    liveWithRequested = Signal(str)
    showRequested = Signal(str)

    def __init__(self, iss, parent=None):
        super().__init__(parent)
        self.iss_id = iss.id
        self.setObjectName(f"issue_{iss.id}")
        self.setFrameShape(QFrame.Shape.StyledPanel)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(4)
        self.head = QLabel()
        self.head.setWordWrap(True)
        self.head.setTextFormat(Qt.TextFormat.RichText)
        self.body = QLabel()
        self.body.setWordWrap(True)
        self.body.setTextFormat(Qt.TextFormat.RichText)
        self.body.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        for w in (self.head, self.body):
            w.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            lay.addWidget(w)
        row = QHBoxLayout()
        self.accept_btn = QPushButton("Accept fix")
        self.accept_btn.setObjectName("accept")
        self.live_btn = QPushButton("Live with it")
        self.live_btn.setObjectName("live_with")
        self.show_btn = QPushButton("Show")
        self.accept_btn.clicked.connect(lambda: self.acceptRequested.emit(self.iss_id))
        self.live_btn.clicked.connect(lambda: self.liveWithRequested.emit(self.iss_id))
        self.show_btn.clicked.connect(lambda: self.showRequested.emit(self.iss_id))
        for b in (self.accept_btn, self.live_btn, self.show_btn):
            row.addWidget(b)
        row.addStretch(1)
        lay.addLayout(row)
        self.update_from(iss)

    def update_from(self, iss):
        col = SEVERITY.get(iss.severity, "#888") if iss.state == OPEN else theme.tokens()["text2"].name()
        state = {OPEN: "", FIX_ACCEPTED: " · fix accepted, checking it works", LIVED_WITH: " · living with it",
                 RESOLVED: " · resolved"}[iss.state]
        self.setStyleSheet(f"QFrame#issue_{iss.id} {{ border-left: 4px solid {col}; }}")
        self.head.setText(f"<b style='color:{col}'>{iss.id} · {iss.severity.upper()}</b>"
                          f"<span style='color:gray'> {_fmt_t(iss.first_seen)}{state}</span><br><b>{iss.title}</b>")
        parts = [iss.evidence]
        if iss.state == OPEN:
            if iss.analysing:
                parts.append("<i>Working out the best fix…</i>")
            elif iss.fix_text:
                parts.append(f"<b>Recommended:</b> {iss.fix_text}")
            if len(iss.fixes) > 1 or (iss.fixes and iss.kind != CAMERA_OFFLINE):
                steps = "".join(f"<li><b>{r.title}</b>: {r.detail}</li>" for r in iss.fixes)
                parts.append(f"<ol style='margin:0'>{steps}</ol>")
            elif iss.fixes:
                parts.append(iss.fixes[0].detail)
        elif iss.state == FIX_ACCEPTED:
            parts.append("Watching the next passes through the region to confirm the fix worked.")
        elif iss.state == LIVED_WITH:
            parts.append("You chose to live with this. The brain keeps watching and alerts again only if it "
                         "gets clearly worse.")
        if iss.history:
            t, text = iss.history[-1]
            parts.append(f"<span style='color:gray'>{_fmt_t(t)} {text}</span>")
        self.body.setText("<br>".join(parts))
        can_fix = iss.state == OPEN and (bool(iss.fixes) or iss.action == "restore")
        self.accept_btn.setVisible(can_fix)
        self.accept_btn.setText("Restore camera" if iss.action == "restore" else
                                ("Accept fix" if len(iss.fixes) <= 1 else f"Accept {len(iss.fixes)} changes"))
        self.live_btn.setVisible(iss.state == OPEN)
        self.show_btn.setVisible(iss.state != RESOLVED and iss.location is not None)


class LiveTab(QWidget):
    def __init__(self, window, parent=None):
        super().__init__(parent)
        self.win = window
        self.show_sim = None
        self.brain: Brain | None = None
        self.cards: dict[str, IssueCard] = {}
        self._running = []                       # (thread, worker) kept until finished
        self._analysing: str | None = None
        self._cams_ref = None
        self._last_wall = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        bar_box, bar = flow_row()                 # wraps in a narrow window
        self.start_btn = QPushButton("Start simulated show")
        self.start_btn.setToolTip("Fly simulated drones over the planned layout; the brain monitors it live")
        self.start_btn.clicked.connect(self.toggle)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop)
        self.stop_btn.setEnabled(False)
        bar.addWidget(self.start_btn)
        bar.addWidget(self.stop_btn)
        bar.addSpacing(10)
        bar.addWidget(QLabel("Drones:"))
        self.n_drones = QSpinBox()
        self.n_drones.setRange(1, 64)
        self.n_drones.setValue(12)
        bar.addWidget(self.n_drones)
        bar.addWidget(QLabel("Speed:"))
        self.speed = QComboBox()
        for label, v in (("1×", 1), ("5×", 5), ("20×", 20)):
            self.speed.addItem(label, v)
        self.speed.setCurrentIndex(1)
        bar.addWidget(self.speed)
        bar.addSpacing(10)
        self.fault_btn = QToolButton()
        self.fault_btn.setText("Inject fault")
        self.fault_btn.setToolTip("Rehearse: make something go wrong and watch the brain find it")
        self.fault_btn.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        self.fault_menu = QMenu(self)
        self.fault_btn.setMenu(self.fault_menu)
        self.fault_menu.aboutToShow.connect(self._build_fault_menu)
        self.fault_btn.setEnabled(False)
        bar.addWidget(self.fault_btn)
        bar.addSpacing(16)
        self.clock = QLabel("")
        self.clock.setStyleSheet("color: palette(placeholder-text);")
        bar.addWidget(self.clock)
        lay.addWidget(bar_box)

        split = QSplitter(Qt.Orientation.Horizontal)
        self.view = LiveView3D(window.doc, window.canvas)
        split.addWidget(self.view)
        side = QWidget()
        sl = QVBoxLayout(side)
        sl.setContentsMargins(6, 0, 0, 0)
        tiles_box, tiles = flow_row()
        self.t_drones = Tile("Drones tracked")
        self.t_cams = Tile("Cameras online")
        self.t_issues = Tile("Open alerts")
        for t in (self.t_drones, self.t_cams, self.t_issues):
            t.setMinimumWidth(120)
            tiles.addWidget(t)
        sl.addWidget(tiles_box)
        self.status = QLabel("Plan the cameras in the Planner tab, then start the show. The brain watches "
                             "tracking and every station on its own and only asks you to decide.")
        self.status.setWordWrap(True)
        sl.addWidget(self.status)
        self.feed_box = QWidget()
        self.feed = QVBoxLayout(self.feed_box)
        self.feed.setContentsMargins(0, 0, 0, 0)
        self.feed.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidget(self.feed_box)
        sl.addWidget(scroll, 1)
        split.addWidget(side)
        split.setStretchFactor(0, 1)
        split.setSizes([900, 440])
        lay.addWidget(split, 1)

        self.timer = QTimer(self)
        self.timer.setInterval(TICK_MS)
        self.timer.timeout.connect(self.tick)

    # ------------------------------------------------------------ run
    def _context(self):
        if self.win.canvas.result is None:
            return None
        from idps.plan.site import site_from_dict
        try:
            site = site_from_dict(self.win.doc.to_site_dict(), str(self.win.doc.path) if self.win.doc.path else None)
        except Exception as exc:
            self.status.setText(f"The venue has a problem: {exc}")
            return None
        return site, list(self.win.canvas.result[0].cameras)

    def toggle(self):
        if self.timer.isActive():
            self.timer.stop()
            self.start_btn.setText("Resume")
            return
        if self.show_sim is None and not self.start():
            return
        self._last_wall = None
        self.timer.start()
        self.start_btn.setText("Pause")

    def start(self) -> bool:
        ctx = self._context()
        if ctx is None:
            self.status.setText("Plan the cameras first (Planner tab → Plan, ⌘R). The live monitor runs on the "
                                "planned layout.")
            return False
        from idps.monitor.simsource import SimShow
        site, cams = ctx
        self.site = site
        self.show_sim = SimShow(site, cams, n_drones=self.n_drones.value(), seed=int(time.time()) % 10000)
        self.brain = Brain(site, cams)
        self._cams_ref = self.win.canvas.result[0].cameras
        self.view.need = site.requirements.min_cameras
        self.view.trails.clear()
        self._clear_cards()
        self.stop_btn.setEnabled(True)
        self.fault_btn.setEnabled(True)
        self.n_drones.setEnabled(False)
        self.status.setText("Monitoring. Nothing to do unless an alert appears.")
        return True

    def stop(self):
        self.timer.stop()
        self.show_sim = self.brain = None
        self.start_btn.setText("Start simulated show")
        self.stop_btn.setEnabled(False)
        self.fault_btn.setEnabled(False)
        self.n_drones.setEnabled(True)
        self.view.clear()
        self._clear_cards()
        self.clock.setText("")
        self.status.setText("Stopped.")

    def step(self, steps: int = 1):
        """Advance the show and feed the brain (also used by tests, headless)."""
        if self.show_sim is None:
            return []
        self._sync_layout()
        changes = []
        for _ in range(steps):
            samples, statuses = self.show_sim.step(DT)
            changes += self.brain.feed(samples, statuses)
        self._samples = samples
        self._refresh(changes)
        self._kick_analysis()
        return changes

    def tick(self):
        self.step(int(self.speed.currentData()))

    def _sync_layout(self):
        """The plan changed elsewhere (Planner advisor, re-plan): the crew installed it."""
        res = self.win.canvas.result
        if res is None or res[0].cameras is self._cams_ref:
            return
        self._cams_ref = res[0].cameras
        cams = list(res[0].cameras)
        self.show_sim.set_cameras(cams)
        self.brain.set_cameras(cams)

    # ------------------------------------------------------------ display
    def _refresh(self, changes):
        sim, brain = self.show_sim, self.brain
        need = brain.min_cameras
        drones = {s.drone_id: (s.pos, s.n_cams) for s in self._samples}
        moved = {c for c, m in brain._moved.items() if m >= brain.s.moved_warn_mrad}
        self.view.set_state(drones, brain.offline, moved, list(brain.issues.values()), sim.occluders)
        ok = sum(n >= need for _, n in drones.values())
        lost = sum(n < 2 for _, n in drones.values())
        self.t_drones.set(f"{ok}/{len(drones)}", f"{lost} lost now" if lost else "all tracked", lost == 0)
        n_cams = len(sim.stations)
        off = len(brain.offline)
        self.t_cams.set(f"{n_cams - off}/{n_cams}", f"{len(moved)} knocked" if moved else "all on calibration",
                        off == 0 and not moved)
        open_ = [i for i in brain.issues.values() if i.state == OPEN]
        self.t_issues.set(str(len(open_)), f"{sum(i.state == LIVED_WITH for i in brain.issues.values())} lived with",
                          not open_)
        self.clock.setText(f"Show time {_fmt_t(sim.t)}")
        self.status.setText(brain.status_line())
        for iss, change in changes:
            if change in ("new", "worse"):
                self.win.statusBar().showMessage(f"Alert {iss.id}: {iss.title}", 10000)
        self._sync_cards()

    def _clear_cards(self):
        for c in self.cards.values():
            c.hide()
            c.deleteLater()
        self.cards.clear()

    def _sync_cards(self):
        if self.brain is None:
            return
        # newest first; open alerts above accepted / lived-with / resolved ones
        order = {OPEN: 0, FIX_ACCEPTED: 1, LIVED_WITH: 2, RESOLVED: 3}
        issues = sorted(self.brain.issues.values(), key=lambda i: (order[i.state], -i.first_seen))
        resolved = [i for i in issues if i.state == RESOLVED][5:]          # keep the last few resolved
        issues = [i for i in issues if i not in resolved]
        for iid in [k for k in self.cards if k not in {i.id for i in issues}]:
            card = self.cards.pop(iid)
            card.hide()
            card.deleteLater()
        for pos, iss in enumerate(issues):
            card = self.cards.get(iss.id)
            if card is None:
                card = self.cards[iss.id] = IssueCard(iss)
                card.acceptRequested.connect(self.accept)
                card.liveWithRequested.connect(self.live_with)
                card.showRequested.connect(self.show_issue)
            else:
                card.update_from(iss)
            if self.feed.indexOf(card) != pos:
                self.feed.removeWidget(card)
                self.feed.insertWidget(pos, card)

    # ------------------------------------------------------------ analysis
    def _kick_analysis(self):
        if self._analysing is not None or self.brain is None:
            return
        pend = self.brain.pending()
        if not pend:
            return
        iss = sorted(pend, key=lambda i: (i.severity != "critical", i.first_seen))[0]
        iss.analysing = True
        self._analysing = iss.id
        snap = self.brain.snapshot(iss)

        def fn(progress, cancel):
            return Brain.analyse(snap, progress, cancel)
        thread, worker = start_fn_worker(self, fn)
        self._running.append((thread, worker))
        worker.finished.connect(self._analysis_done)
        worker.failed.connect(self._analysis_failed)
        thread.finished.connect(self._release_finished)
        self._sync_cards()

    def _analysis_done(self, result):
        iid, self._analysing = self._analysing, None
        if self.brain is not None and iid is not None:
            self.brain.set_analysis(iid, result)
            self._sync_cards()

    def _analysis_failed(self, msg):
        iid, self._analysing = self._analysing, None
        if self.brain is not None and iid in self.brain.issues:
            self.brain.issues[iid].analysing = False
            self.brain.issues[iid].no_fix = True
            self.brain.issues[iid].fix_text = f"The advisor failed: {msg.splitlines()[0]}"
            self._sync_cards()

    def _release_finished(self):
        for keep in [k for k in self._running if k[0].isFinished()]:
            self._running.remove(keep)
            keep[0].deleteLater()

    def wait_analysis(self, timeout_s: float = 120.0):
        """Block until a running analysis is done (tests, headless)."""
        from PySide6.QtWidgets import QApplication
        t0 = time.time()
        while self._analysing is not None and time.time() - t0 < timeout_s:
            QApplication.processEvents()
            time.sleep(0.01)

    # ------------------------------------------------------------ operator
    def accept(self, iid: str):
        iss = self.brain.issues.get(iid) if self.brain else None
        if iss is None:
            return
        if iss.action == "restore":
            self.brain.accept(iid)
            for c in iss.cameras:
                self.show_sim.restore(c)
        elif iss.fixes:
            self.brain.accept(iid)
            self.win.apply_fixes(list(iss.fixes))        # the plan, install sheet and Planner follow
            self._sync_layout()
        self.view.highlight = None
        self.win.preview_fix(None)
        self._sync_cards()

    def live_with(self, iid: str):
        if self.brain is None or iid not in self.brain.issues:
            return
        self.brain.live_with(iid)
        self._sync_cards()

    def show_issue(self, iid: str):
        iss = self.brain.issues.get(iid) if self.brain else None
        if iss is None:
            return
        self.view.highlight = iid
        self.view.preview = iss.recommendation
        if iss.location is not None:
            self.view.target = np.asarray(iss.location, float)
            self.view.dist = max(20.0, iss.radius_m * 8)
        self.view.update()

    # ------------------------------------------------------------ faults
    def _build_fault_menu(self):
        m = self.fault_menu
        m.clear()
        if self.show_sim is None:
            return
        a = QAction("Hang an obstruction where the drones fly", m)
        a.triggered.connect(self.fault_obstruction)
        m.addAction(a)
        if self.show_sim.occluders:
            a = QAction("Remove all obstructions", m)
            a.triggered.connect(self.show_sim.clear_occluders)
            m.addAction(a)
        m.addSeparator()
        sel = self.win.canvas.selection
        cams = [c.id for c in self.show_sim.cams]
        pick = cams[sel[1]] if sel and sel[0] == "camera" and sel[1] < len(cams) else None
        target = pick or (cams[len(cams) // 2] if cams else None)
        if target:
            off = target in self.show_sim.offline
            a = QAction(f"{'Bring back' if off else 'Take offline'} {target}" + ("" if pick else
                        " (select a camera to choose)"), m)
            a.triggered.connect(lambda _=False, c=target, o=off: self.fault_offline(c, not o))
            m.addAction(a)
            a = QAction(f"Knock {target} (3 mrad)", m)
            a.triggered.connect(lambda _=False, c=target: self.fault_bump(c))
            m.addAction(a)

    def fault_obstruction(self, target=None):
        tgt, blocked = self.show_sim.obstruct(target)
        self.win.statusBar().showMessage(f"Simulated obstructions in front of {', '.join(blocked)} near "
                                         f"X {tgt[0]:.0f}, Y {tgt[1]:.0f}, Z {tgt[2]:.0f} m. Watch the brain.", 10000)
        return tgt, blocked

    def fault_offline(self, cid, off=True):
        self.show_sim.set_offline(cid, off)

    def fault_bump(self, cid, mrad=3.0):
        self.show_sim.bump(cid, mrad)

    def shutdown(self):
        self.timer.stop()
        for thread, worker in list(self._running):
            worker.stop()
            thread.quit()
            thread.wait(10000)

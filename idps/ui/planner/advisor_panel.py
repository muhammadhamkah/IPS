"""Advisor tab: system health and ranked, one-click fixes (re-aim / move / add a
camera) with tape-measure dimensions. Uses idps.plan.advisor; during calibration
and shows the same panel will be fed live measurements instead of predictions."""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QFrame, QGroupBox, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout,
                               QWidget)

from idps.ui.planner.worker import start_fn_worker

KIND_SUFFIX = {"reaim": " (motorised, no physical move)", "move": "", "add": ""}


class AdvisorPanel(QWidget):
    applyRequested = Signal(object)          # list[Recommendation] to apply in order
    previewRequested = Signal(object)        # Recommendation or None

    def __init__(self, window, parent=None):
        super().__init__(parent)
        self.win = window
        self._running = []
        self.worker = None
        lay = QVBoxLayout(self)
        self.health = QGroupBox("System health")
        hl = QVBoxLayout(self.health)
        self.status = QLabel("Plan the cameras first; the advisor then checks the layout and suggests fixes.")
        self.status.setWordWrap(True)
        hl.addWidget(self.status)
        lay.addWidget(self.health)

        row = QHBoxLayout()
        self.find_btn = QPushButton("Find fixes")
        self.find_btn.setToolTip("Rank the single changes that remove the most problem points")
        self.find_btn.clicked.connect(self.find_fixes)
        self.auto_btn = QPushButton("Auto-fix plan…")
        self.auto_btn.setToolTip("Apply the best fix, re-check, repeat: an ordered to-do list for the crew")
        self.auto_btn.clicked.connect(self.auto_fix)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.clicked.connect(self.cancel)
        self.cancel_btn.hide()
        for b in (self.find_btn, self.auto_btn, self.cancel_btn):
            row.addWidget(b)
        lay.addLayout(row)
        self.progress = QLabel("")
        self.progress.setWordWrap(True)
        self.progress.setStyleSheet("color: palette(placeholder-text);")
        lay.addWidget(self.progress)
        self.list = QVBoxLayout()
        lay.addLayout(self.list)
        lay.addStretch(1)
        self.refresh_health()

    # ------------------------------------------------------------- state
    def _context(self):
        """(site, cameras) for the advisor, or None. Uses the venue as drawn now,
        so mounts added after planning (e.g. a tripod strip) can be used for fixes."""
        if self.win.canvas.result is None:
            return None
        from idps.plan.site import site_from_dict
        try:
            site = site_from_dict(self.win.doc.to_site_dict(), str(self.win.doc.path) if self.win.doc.path else None)
        except Exception as exc:                       # venue currently invalid
            self.status.setText(f"The venue has a problem: {exc}")
            return None
        return site, list(self.win.canvas.result[0].cameras)

    def refresh_health(self):
        ctx = self._context() if self.win.canvas.result is not None else None
        self._clear_list()
        enabled = ctx is not None
        self.find_btn.setEnabled(enabled and not self._busy())
        self.auto_btn.setEnabled(enabled and not self._busy())
        if not ctx:
            return
        from idps.plan.advisor import Advisor
        site, cams = ctx
        h = Advisor(site, cams).health()
        ok = h.failing == 0
        lines = [("✅ " if ok else "⚠️ ")
                 + (f"Every evaluation point meets the requirement with {h.cameras} cameras." if ok else
                    f"{h.failing} of {h.points} points fail the requirement (about {h.failing_volume_m3:,.0f} m³); "
                    f"{h.fraction_ok * 100:.1f} % OK.")]
        if h.worst_camera:
            lines.append(f"Most critical camera: {h.worst_camera}. Losing it would make {h.worst_camera_points} more "
                         f"points fail; check it first if tracking degrades.")
        if not ok:
            lines.append("Press Find fixes for ranked suggestions, or Auto-fix for a complete to-do list.")
        self.status.setText("\n".join(lines))
        self.status.setStyleSheet("" if ok else "color: #b8860b;")

    def _busy(self):
        return self.worker is not None

    def _clear_list(self):
        while self.list.count():
            w = self.list.takeAt(0).widget()
            if w is not None:
                w.hide()
                w.deleteLater()

    # ------------------------------------------------------------- actions
    def _run(self, fn, done):
        self.find_btn.setEnabled(False)
        self.auto_btn.setEnabled(False)
        self.cancel_btn.show()
        self._clear_list()
        thread, worker = start_fn_worker(self, fn)
        self.worker = worker
        self._on_done = done
        self._running.append((thread, worker))
        # Bound methods of this (UI-thread) object, so Qt queues the calls onto the UI
        # thread; lambdas would run in the worker thread and must not touch widgets.
        worker.progress.connect(self._on_progress)
        worker.finished.connect(self._on_finished)
        worker.failed.connect(self._on_failed)
        thread.finished.connect(self._release_finished)

    def _on_progress(self, text):
        self.progress.setText(text)

    def _on_finished(self, result):
        self._done(self._on_done, result)

    def _on_failed(self, msg):
        self._done(None, msg)

    def _release_finished(self):
        for keep in [k for k in self._running if k[0].isFinished()]:
            self._running.remove(keep)
            keep[0].deleteLater()

    def _done(self, done, result):
        self.worker = None
        self.cancel_btn.hide()
        self.find_btn.setEnabled(True)
        self.auto_btn.setEnabled(True)
        if done is None:
            self.progress.setText(f"The advisor failed: {str(result).splitlines()[0]}")
            return
        done(result)

    def cancel(self):
        if self.worker is not None:
            self.worker.stop()
            self.progress.setText("Cancelling…")

    def find_fixes(self):
        ctx = self._context()
        if not ctx:
            return
        site, cams = ctx
        from idps.plan.advisor import Advisor

        def fn(progress, cancel):
            return Advisor(site, cams).recommend(top=6, progress=progress)
        self.progress.setText("Searching…")
        self._run(fn, self._show_recs)

    def auto_fix(self):
        ctx = self._context()
        if not ctx:
            return
        site, cams = ctx
        from idps.plan.advisor import auto_fix

        def fn(progress, cancel):
            return auto_fix(site, cams, progress=progress, cancel=cancel)[0]
        self.progress.setText("Working out a complete fix list…")
        self._run(fn, self._show_steps)

    def _show_recs(self, recs):
        if not recs:
            self.progress.setText("No fixes needed: every point meets the requirement." if
                                  "✅" in self.status.text() else
                                  "No single change improves the layout with the current mounts. Add trusses or "
                                  "tripod zones (the advisor can only use allowed mount positions), or raise trusses.")
            return
        self.progress.setText("Ranked by problem points removed. Show highlights the change on the plan.")
        for r in recs:
            self.list.addWidget(self._card(r, [r]))

    def _show_steps(self, steps):
        if not steps:
            self._show_recs([])
            return
        last = steps[-1]
        self.progress.setText(f"{len(steps)} changes, in this order, take the failing points from "
                              f"{steps[0].failing_before} to {last.failing_after}.")
        box = QFrame()
        bl = QVBoxLayout(box)
        bl.setContentsMargins(0, 0, 0, 0)
        for i, r in enumerate(steps, 1):
            bl.addWidget(self._card(r, None, number=i))
        apply_all = QPushButton(f"Apply all {len(steps)} changes")
        apply_all.clicked.connect(lambda: self.applyRequested.emit(steps))
        bl.addWidget(apply_all)
        self.list.addWidget(box)

    def _card(self, r, apply_list, number=None):
        g = QGroupBox(("" if number is None else f"{number}. ") + r.title + KIND_SUFFIX[r.kind])
        v = QVBoxLayout(g)
        fixed = QLabel(f"Removes {r.points_fixed} problem points ({r.failing_before} → {r.failing_after}).")
        fixed.setStyleSheet("font-weight: bold;")
        detail = QLabel(r.detail)
        detail.setWordWrap(True)
        detail.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        detail.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        v.addWidget(fixed)
        v.addWidget(detail)
        row = QHBoxLayout()
        show = QPushButton("Show")
        show.clicked.connect(lambda: self.previewRequested.emit(r))
        row.addWidget(show)
        if apply_list is not None:
            ap = QPushButton("Apply")
            ap.clicked.connect(lambda: self.applyRequested.emit(apply_list))
            row.addWidget(ap)
        row.addStretch(1)
        v.addLayout(row)
        return g

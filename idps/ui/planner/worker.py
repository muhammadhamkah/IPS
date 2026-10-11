"""Runs the optimiser off the UI thread so the app stays responsive."""
from __future__ import annotations

import threading
import traceback

from PySide6.QtCore import QObject, QThread, Signal

from idps.plan.optimizer import Cancelled
from idps.plan.planner import optimise
from idps.plan.site import site_from_dict


class PlanWorker(QObject):
    progress = Signal(str)
    finished = Signal(object, object, object)     # plan, evaluation, site
    failed = Signal(str)
    cancelled = Signal()

    def __init__(self, site_dict: dict, source_path: str | None):
        super().__init__()
        self.site_dict = site_dict
        self.source_path = source_path
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            site = site_from_dict(self.site_dict, self.source_path)
            plan, ev = optimise(site, log=self.progress.emit, cancel=self._stop.is_set)
            self.finished.emit(plan, ev, site)
        except Cancelled:
            self.cancelled.emit()
        except Exception as exc:          # surface any error to the user
            self.failed.emit(f"{exc}\n\n{traceback.format_exc(limit=3)}")


def start_worker(parent, site_dict, source_path):
    thread = QThread(parent)
    worker = PlanWorker(site_dict, source_path)
    worker.moveToThread(thread)
    thread.started.connect(worker.run)
    for sig in (worker.finished, worker.failed, worker.cancelled):
        sig.connect(thread.quit)
    # The worker has no Qt parent, so Python owns it: the caller must keep a
    # reference until thread.finished, then drop it (no deleteLater, which
    # would delete it a second time).
    thread.start()
    return thread, worker

"""Main dashboard window. macOS conventions: native menu bar with ⌘ shortcuts,
unified title/toolbar, document proxy icon and edited-dot in the title bar,
sheet-style save prompts, trackpad gestures on the plan canvas."""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

from PySide6.QtCore import QSettings, Qt, QUrl
from PySide6.QtGui import QAction, QActionGroup, QDesktopServices, QKeySequence
from PySide6.QtWidgets import (QApplication, QComboBox, QFileDialog, QHBoxLayout, QLabel, QMainWindow, QMessageBox,
                               QPushButton, QSizePolicy, QSplitter, QStackedWidget, QTabWidget, QToolBar,
                               QVBoxLayout, QWidget)

from idps.ui.planner.canvas import TOOLS, PlanCanvas
from idps.ui.planner.advisor_panel import AdvisorPanel
from idps.ui.planner.view3d import View3D
from idps.ui.planner.document import SiteDocument
from idps.ui.planner.inspector import Inspector
from idps.ui.planner.results import ResultsPanel
from idps.ui.planner.worker import start_worker

EXAMPLES = Path(__file__).resolve().parents[2] / "examples" / "sites"
TOOL_KEYS = {"select": "V", "volume": "P", "origin": "O", "truss": "T", "zone": "A", "pad": "C", "nogo": "N",
             "measure": "M"}


def placeholder(text: str) -> QWidget:
    w = QWidget()
    lay = QVBoxLayout(w)
    lab = QLabel(text)
    lab.setAlignment(Qt.AlignmentFlag.AlignCenter)
    lab.setStyleSheet("color: palette(placeholder-text); font-size: 15px;")
    lab.setWordWrap(True)
    lay.addWidget(lab)
    return w


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.settings = QSettings()
        self.doc = SiteDocument(self)
        self.canvas = PlanCanvas(self.doc)
        self.inspector = Inspector(self.doc, self.canvas)
        self.results = ResultsPanel(self.canvas)
        self.thread = self.worker = None
        self._running = []          # (thread, worker) kept alive until the thread has finished
        self.last_site = None
        self.advisor = AdvisorPanel(self)
        self.inspector.addTab(Inspector._scroll(self.advisor), "Advisor")
        self.advisor.applyRequested.connect(self.apply_fixes)
        self.advisor.previewRequested.connect(self.preview_fix)

        self.setUnifiedTitleAndToolBarOnMac(True)
        self.setWindowTitle("IDPS[*]")
        self.resize(1440, 900)

        # Planner page: (2D plan | 3D view) + inspector, results underneath
        self.view3d = View3D(self.doc, self.canvas)
        self.views = QStackedWidget()
        self.views.addWidget(self.canvas)
        self.views.addWidget(self._view3d_page())
        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self.views)
        top.addWidget(self.inspector)
        top.setStretchFactor(0, 1)
        top.setSizes([1060, 380])
        page = QSplitter(Qt.Orientation.Vertical)
        page.addWidget(top)
        page.addWidget(self.results)
        page.setStretchFactor(0, 1)
        page.setSizes([620, 280])
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.addTab(page, "Planner")
        from idps.ui.sim.tab import SimTab
        self.sim = SimTab(self)
        self.tabs.addTab(self.sim, "Simulate")
        from idps.ui.px4.tab import Px4Tab
        self.px4 = Px4Tab(self)
        self.tabs.addTab(self.px4, "PX4 flight")
        from idps.ui.live.tab import LiveTab
        self.live = LiveTab(self)
        self.tabs.addTab(self.live, "Live tracking")
        self.tabs.addTab(placeholder("Calibration\n\nAim cameras to plan, wand + calibration square, "
                                     "continuous calibration and tripod restore (roadmap phase 6)."),
                         "Calibration")
        self.setCentralWidget(self.tabs)

        self._actions()
        self._toolbar()
        self.tabs.currentChanged.connect(self._tab_changed)
        self._menus()
        self.cursor_label = QLabel("")
        self.statusBar().addPermanentWidget(self.cursor_label)

        self.doc.changed.connect(self._changed)
        self.doc.pathChanged.connect(self._title)
        self.doc.undo.cleanChanged.connect(self._clean_changed)
        self.canvas.cursorMoved.connect(self._cursor_moved)
        self.canvas.message.connect(lambda m: self.statusBar().showMessage(m, 8000))
        self.canvas.selectionChanged.connect(self._selection)
        self.results.runRequested.connect(self.run_plan)
        self.results.cancelRequested.connect(self.cancel_plan)
        self.results.exportRequested.connect(self.export_plan)
        self.results.reportRequested.connect(self.open_report)
        self.results.cameraSelected.connect(lambda k: self.canvas.set_selection(("camera", k)))
        self.results.mode_group.buttonClicked.connect(lambda _: self.view3d.update())
        self.results.level.valueChanged.connect(lambda _: self.view3d.update())

        geo = self.settings.value("window/geometry")
        if geo is not None:
            self.restoreGeometry(geo)
        self.fit_to_screen()
        self._title(self.doc.title())
        self.canvas.toolChanged.connect(self._tool_changed)
        self.canvas.set_tool("select")

    # ------------------------------------------------------------ actions
    def _act(self, text, slot, shortcut=None, tip=None, checkable=False):
        a = QAction(text, self)
        if shortcut is not None:
            a.setShortcut(QKeySequence(shortcut))
        if tip:
            a.setStatusTip(tip)
            a.setToolTip(tip)
        a.setCheckable(checkable)
        a.triggered.connect(slot)
        return a

    def _actions(self):
        S = QKeySequence.StandardKey
        self.a_new = self._act("New Venue", self.new_doc, S.New)
        self.a_open = self._act("Open…", self.open_doc, S.Open)
        self.a_save = self._act("Save", self.save_doc, S.Save)
        self.a_save_as = self._act("Save As…", self.save_doc_as, S.SaveAs)
        self.a_export = self._act("Export Plan…", self.export_plan, "Ctrl+E")
        self.a_close = self._act("Close Window", self.close, S.Close)
        self.a_undo = self.doc.undo.createUndoAction(self, "Undo")
        self.a_undo.setShortcut(QKeySequence(S.Undo))
        self.a_redo = self.doc.undo.createRedoAction(self, "Redo")
        self.a_redo.setShortcut(QKeySequence(S.Redo))
        self.a_delete = self._act("Delete", self.canvas.delete_selection)
        self.a_zoom_in = self._act("Zoom In", lambda: self.canvas.zoom(1.25), S.ZoomIn)
        self.a_zoom_out = self._act("Zoom Out", lambda: self.canvas.zoom(0.8), S.ZoomOut)
        self.a_fit = self._act("Zoom to Fit", self.zoom_to_fit, "Ctrl+0")
        self.view_group = QActionGroup(self)
        self.a_view2d = self._act("2D Plan", lambda: self.show_view("2d"), "Ctrl+1",
                                  "Top-down plan for drawing the venue (⌘1)", checkable=True)
        self.a_view3d = self._act("3D View", lambda: self.show_view("3d"), "Ctrl+2",
                                  "3D view of the volume, trusses, tripods and cameras (⌘2)", checkable=True)
        for a in (self.a_view2d, self.a_view3d):
            self.view_group.addAction(a)
        self.a_view2d.setChecked(True)
        self.a_run = self._act("Plan Cameras", self.run_plan, "Ctrl+R", "Find camera count, positions and pan/tilt")
        self.a_cancel = self._act("Cancel Planning", self.cancel_plan, "Ctrl+.")
        self.a_about = self._act("About IDPS", self.about)
        self.a_about.setMenuRole(QAction.MenuRole.AboutRole)
        self.a_quit = self._act("Quit IDPS", self.close, S.Quit)
        self.a_quit.setMenuRole(QAction.MenuRole.QuitRole)
        self.tool_group = QActionGroup(self)
        self.tool_actions = {}
        for key, label in TOOLS.items():
            a = self._act(label.split(" ")[0] if key == "select" else label.replace("Draw ", "").capitalize(),
                          lambda _=False, k=key: self.set_tool(k), TOOL_KEYS[key],
                          f"{label} ({TOOL_KEYS[key]})", checkable=True)
            self.tool_group.addAction(a)
            self.tool_actions[key] = a
        self.tool_actions["select"].setChecked(True)
        self.a_snap = self._act("Snap to Grid", self._snap_toggled, None, "Snap clicks to the grid "
                                "(hold Option to place freely)", checkable=True)
        self.a_snap.setChecked(True)
        self.a_dims = self._act("Dimensions", self._dims_toggled, "Ctrl+D",
                                "Show install measurements: distances along trusses and volume edges",
                                checkable=True)
        self.a_dims.setChecked(True)

    def _toolbar(self):
        tb = QToolBar("Tools")
        self.toolbar = tb
        tb.setMovable(False)
        tb.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)
        self.addToolBar(tb)
        for a in self.tool_actions.values():
            tb.addAction(a)
        tb.addSeparator()
        tb.addAction(self.a_dims)
        tb.addAction(self.a_snap)
        self.snap_combo = QComboBox()
        for s in ("0.1", "0.25", "0.5", "1", "2", "5"):
            self.snap_combo.addItem(f"{s} m", float(s))
        self.snap_combo.setCurrentIndex(2)
        self.snap_combo.currentIndexChanged.connect(
            lambda i: setattr(self.canvas, "snap_step", self.snap_combo.itemData(i)))
        tb.addWidget(self.snap_combo)
        tb.addSeparator()
        tb.addAction(self.a_fit)
        tb.addSeparator()
        tb.addAction(self.a_view2d)
        tb.addAction(self.a_view3d)
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        tb.addWidget(spacer)
        tb.addAction(self.a_run)

    def _menus(self):
        mb = self.menuBar()                      # becomes the macOS global menu bar
        f = mb.addMenu("File")
        for a in (self.a_new, self.a_open):
            f.addAction(a)
        ex = f.addMenu("Open Example")
        for p in sorted(EXAMPLES.glob("*.yaml")):
            ex.addAction(p.stem.replace("_", " "), lambda p=p: self.open_doc(str(p)))
        self.recent_menu = f.addMenu("Open Recent")
        self._recent_refresh()
        f.addSeparator()
        for a in (self.a_save, self.a_save_as, self.a_export):
            f.addAction(a)
        f.addSeparator()
        f.addAction(self.a_close)
        f.addAction(self.a_quit)
        e = mb.addMenu("Edit")
        e.addAction(self.a_undo)
        e.addAction(self.a_redo)
        e.addSeparator()
        e.addAction(self.a_delete)
        t = mb.addMenu("Tools")
        for a in self.tool_actions.values():
            t.addAction(a)
        t.addSeparator()
        t.addAction(self.a_snap)
        v = mb.addMenu("View")
        v.addAction(self.a_view2d)
        v.addAction(self.a_view3d)
        v.addAction(self.a_dims)
        v.addSeparator()
        for a in (self.a_zoom_in, self.a_zoom_out, self.a_fit):
            v.addAction(a)
        p = mb.addMenu("Plan")
        p.addAction(self.a_run)
        p.addAction(self.a_cancel)
        h = mb.addMenu("Help")
        h.addAction(self.a_about)
        h.addAction("Planner Workflow", lambda: QDesktopServices.openUrl(
            QUrl.fromLocalFile(str(EXAMPLES.parents[1] / "docs" / "WORKFLOW.md"))))

    # ------------------------------------------------------------ state
    def _view3d_page(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        from idps.ui.flow import flow_row
        bar_box, bar = flow_row()                 # wraps onto two lines in a narrow window
        bar.setContentsMargins(8, 4, 8, 4)
        for label, name in (("3D", "iso"), ("Top", "top"), ("Front", "front"), ("Side", "side")):
            b = QPushButton(label)
            b.setToolTip(f"{label} view")
            b.clicked.connect(lambda _=False, n=name: self.view3d.set_view(n))
            bar.addWidget(b)
        fit = QPushButton("Fit")
        fit.clicked.connect(self.view3d.fit)
        bar.addWidget(fit)
        bar.addSpacing(16)
        bar.addWidget(QLabel("Camera view cones:"))
        cones = QComboBox()
        cones.addItems(["Selected camera", "All cameras", "None"])
        cones.currentIndexChanged.connect(
            lambda i: (setattr(self.view3d, "show_cones", ["selected", "all", "none"][i]), self.view3d.update()))
        bar.addWidget(cones)
        bar.addSpacing(12)
        bar.addWidget(QLabel("Coverage:"))
        self.cov_combo = QComboBox()
        modes = [("Altitude slice", "slice"), ("Whole volume", "volume"), ("Problems only", "problems"), ("Off", "off")]
        for label, key in modes:
            self.cov_combo.addItem(label, key)
        self.cov_combo.setToolTip("Altitude slice: a coloured layer at the altitude slider's height\n"
                                  "Whole volume: every evaluation point\n"
                                  "Problems only: points with fewer than 3 cameras or over the accuracy limit")
        self.cov_combo.currentIndexChanged.connect(
            lambda i: (setattr(self.view3d, "coverage_mode", self.cov_combo.itemData(i)), self.view3d.update()))
        bar.addWidget(self.cov_combo)
        lay.addWidget(bar_box)
        lay.addWidget(self.view3d, 1)
        return page

    def fit_to_screen(self):
        """Keep the window inside the screen's usable area (menu bar and Dock excluded), e.g. on a
        laptop, or when a size saved on a bigger external display is restored."""
        screen = self.screen() or QApplication.primaryScreen()
        if screen is None:
            return
        avail = screen.availableGeometry()
        frame = self.frameGeometry().size() - self.geometry().size()      # title bar
        w = min(self.width(), avail.width() - frame.width())
        h = min(self.height(), avail.height() - frame.height())
        if (w, h) != (self.width(), self.height()):
            self.resize(w, h)
        g = self.frameGeometry()
        x = min(max(g.x(), avail.x()), avail.right() - g.width() + 1)
        y = min(max(g.y(), avail.y()), avail.bottom() - g.height() + 1)
        if (x, y) != (g.x(), g.y()):
            self.move(x, y)

    def _cursor_moved(self, x: float, y: float):
        from idps.ui.planner.document import frame_of
        n, e = frame_of(self.doc.data).ne((x, y))
        self.cursor_label.setText(f"N {n:8.2f} m   E {e:8.2f} m")

    def _tab_changed(self, i: int):
        """The drawing toolbar belongs to the Planner."""
        self.toolbar.setVisible(i == 0)

    def _tool_changed(self, tool: str):
        """Keep the toolbar's highlighted tool in step with the canvas (e.g. after
        finishing a shape the canvas returns to Select)."""
        a = self.tool_actions.get(tool)
        if a is not None and not a.isChecked():
            a.setChecked(True)

    def show_view(self, which: str):
        three = which == "3d"
        self.views.setCurrentIndex(1 if three else 0)
        (self.a_view3d if three else self.a_view2d).setChecked(True)
        if three:
            self.view3d.setFocus()
        else:
            self.canvas.setFocus()

    def zoom_to_fit(self):
        if self.views.currentIndex() == 1:
            self.view3d.fit()
        else:
            self.canvas.zoom_to_fit()

    def set_tool(self, key):
        if self.views.currentIndex() == 1 and key != "select":
            self.show_view("2d")                 # drawing happens on the 2D plan
        self.tool_actions[key].setChecked(True)
        self.canvas.set_tool(key)
        self.canvas.setFocus()

    def _dims_toggled(self, on):
        self.canvas.show_dimensions = on
        self.canvas.viewport().update()

    def _snap_toggled(self, on):
        self.canvas.snap_enabled = on

    def _title(self, name):
        self.setWindowTitle(f"{name}[*] — IDPS Planner")
        self.setWindowFilePath(str(self.doc.path) if self.doc.path else "")
        self.setWindowModified(self.doc.dirty)

    def _clean_changed(self, clean: bool):
        self.setWindowModified(not clean)

    def _changed(self):
        self.setWindowModified(self.doc.dirty)
        if self.canvas.result is not None:
            self.results.mark_stale(True)

    def _selection(self, sel):
        if sel and sel[0] == "camera":
            self.results.select_camera(sel[1])

    def _recent_refresh(self):
        self.recent_menu.clear()
        recent = self.settings.value("recent", []) or []
        if isinstance(recent, str):
            recent = [recent]
        for p in recent:
            if Path(p).exists():
                self.recent_menu.addAction(Path(p).name, lambda p=p: self.open_doc(p))
        self.recent_menu.setEnabled(bool(self.recent_menu.actions()))

    def _remember(self, path):
        recent = self.settings.value("recent", []) or []
        if isinstance(recent, str):
            recent = [recent]
        recent = [str(path)] + [p for p in recent if p != str(path)]
        self.settings.setValue("recent", recent[:10])
        self.settings.setValue("lastDir", str(Path(path).parent))
        self._recent_refresh()

    # ------------------------------------------------------------ files
    def _maybe_save(self) -> bool:
        if not self.doc.dirty:
            return True
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Icon.Warning)
        box.setText(f"Do you want to save the changes to “{self.doc.title()}”?")
        box.setInformativeText("Your changes will be lost if you don’t save them.")
        box.setStandardButtons(QMessageBox.StandardButton.Save | QMessageBox.StandardButton.Discard
                               | QMessageBox.StandardButton.Cancel)
        box.setDefaultButton(QMessageBox.StandardButton.Save)
        box.setWindowModality(Qt.WindowModality.WindowModal)      # sheet on macOS
        r = box.exec()
        if r == QMessageBox.StandardButton.Save:
            return self.save_doc()
        return r == QMessageBox.StandardButton.Discard

    def new_doc(self):
        if self._maybe_save():
            self.canvas.clear_result()
            self.doc.new()
            self.canvas.set_selection(None)
            self.canvas.zoom_to_fit()
            self.set_tool("volume")

    def open_doc(self, path: str | None = None):
        if not self._maybe_save():
            return
        if not path:
            path, _ = QFileDialog.getOpenFileName(self, "Open Venue", self.settings.value("lastDir", str(EXAMPLES)),
                                                  "IDPS site (*.yaml *.yml)")
            if not path:
                return
        try:
            self.doc.load(path)
        except Exception as exc:
            QMessageBox.critical(self, "Could not open venue", str(exc))
            return
        self.canvas.clear_result()
        self.canvas.set_selection(None)
        self.canvas.zoom_to_fit()
        if not str(path).startswith(str(EXAMPLES)):
            self._remember(path)

    def save_doc(self) -> bool:
        if self.doc.path is None or str(self.doc.path).startswith(str(EXAMPLES)):
            return self.save_doc_as()
        self.doc.save()
        return True

    def save_doc_as(self) -> bool:
        name = (self.doc.data.get("name") or "venue").replace(" ", "_").lower() + ".yaml"
        path, _ = QFileDialog.getSaveFileName(self, "Save Venue",
                                              str(Path(self.settings.value("lastDir", str(Path.home()))) / name),
                                              "IDPS site (*.yaml)")
        if not path:
            return False
        if not path.endswith((".yaml", ".yml")):
            path += ".yaml"
        self.doc.save(path)
        self._remember(path)
        return True

    # ------------------------------------------------------------ planning
    def run_plan(self):
        if self.thread is not None:
            return
        if len(self.doc.data["volume"]["polygon"]) < 3:
            QMessageBox.information(self, "Draw the volume first",
                                    "Use the Volume tool (P) to click the floor outline of the flight volume.")
            return
        if not self.doc.data.get("origin"):
            self.statusBar().showMessage("Place the origin first: click where the calibration square's corner goes, "
                                         "then drag its red N arrow to point North.", 15000)
            self.set_tool("origin")
            return
        if not self.doc.data["trusses"] and not self.doc.data["tripod_zones"]:
            if not self._offer_tripod_strip():
                return
        site_dict = self.doc.to_site_dict()
        problem = self._site_problem(site_dict)
        if problem:
            box = QMessageBox(self)
            box.setIcon(QMessageBox.Icon.Warning)
            box.setText("The venue needs a fix before planning")
            box.setInformativeText(problem)
            box.setWindowModality(Qt.WindowModality.WindowModal)
            self._exec_dialog(box)
            return
        if not (self.doc.data.get("pads") or {}).get("centres") and not self._ask_pads():
            return
        self.results.running(True)
        self.a_run.setEnabled(False)
        self.thread, self.worker = start_worker(self, site_dict, str(self.doc.path) if self.doc.path else None)
        keep = (self.thread, self.worker)
        self._running.append(keep)
        self.thread.finished.connect(lambda keep=keep: self._release(keep))
        self.worker.progress.connect(self.results.log)
        self.worker.finished.connect(self._plan_done)
        self.worker.failed.connect(self._plan_failed)
        self.worker.cancelled.connect(self._plan_cancelled)

    def _release(self, keep):
        if keep in self._running:
            self._running.remove(keep)
        keep[0].deleteLater()

    def _site_problem(self, site_dict) -> str | None:
        """Validate in the UI thread and explain problems in plain words."""
        z0, z1 = site_dict["volume"]["z"]
        if z1 <= z0:
            return (f"The performance area's ceiling ({z1:g} m) must be above its lowest flight height "
                    f"({z0:g} m). Set them under Venue & requirements → Flight volume height, or click inside "
                    "the performance area and use the Inspector.")
        try:
            from idps.plan.site import site_from_dict
            site = site_from_dict(site_dict)
        except (ValueError, KeyError, TypeError) as exc:
            return f"{exc}"
        if site.volume.volume_m3 <= 0:
            return "The flight volume is empty."
        ceiling = site.volume.z_max + site.candidates.min_height_above_volume_m
        reach = [t.height_m - site.candidates.truss_drop_m for t in site.trusses] + \
                [z.height_range_m[1] for z in site.tripod_zones]
        if reach and max(reach) < ceiling - 1e-9:
            return (f"Cameras must be above the drones, but no truss or tripod reaches the flight ceiling "
                    f"({ceiling:g} m; the highest mount gives {max(reach):g} m). Raise the trusses or the tripod "
                    f"mast heights, or lower the ceiling height.")
        return None

    def _offer_tripod_strip(self) -> bool:
        """No mounts yet: offer the common default (tripods in a 1-3 m strip
        around the volume) instead of a dead-end message."""
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Icon.Question)
        box.setText("Where can cameras be mounted?")
        box.setInformativeText("There are no trusses or tripod zones yet. Add a tripod strip 1–3 m outside "
                               "the volume (masts 2–20 m) and plan? You can edit it in the inspector "
                               "afterwards, or draw trusses (T) and tripod areas (A) instead.")
        add = box.addButton("Add Tripod Strip && Plan", QMessageBox.ButtonRole.AcceptRole)
        box.addButton(QMessageBox.StandardButton.Cancel)
        box.setDefaultButton(add)
        box.setWindowModality(Qt.WindowModality.WindowModal)
        self._exec_dialog(box)
        if box.clickedButton() is not add:
            return False
        self.inspector._add_strip()
        return True

    def _ask_pads(self) -> bool:
        """No charging pads yet: they come before planning, as the cameras must also see the
        drones on their pads and climbing to the grid. True: plan anyway."""
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Icon.Question)
        box.setText("Place the charging pads first?")
        box.setInformativeText("The drones take off from charging pads, and the cameras must see them there and "
                               "on their climb to the grid, so the pads go in before the cameras are planned. "
                               "Click where each pad stands with the Charging pad tool (C).")
        place = box.addButton("Place Pads", QMessageBox.ButtonRole.AcceptRole)
        anyway = box.addButton("Plan Without Pads", QMessageBox.ButtonRole.DestructiveRole)
        box.addButton(QMessageBox.StandardButton.Cancel)
        box.setDefaultButton(place)
        box.setWindowModality(Qt.WindowModality.WindowModal)
        self._exec_dialog(box)
        if box.clickedButton() is place:
            self.set_tool("pad")
        return box.clickedButton() is anyway

    def _exec_dialog(self, box):          # separate so tests can stub it
        return box.exec()

    def cancel_plan(self):
        if self.worker is not None:
            self.worker.stop()
            self.results.log("Cancelling…")

    def _plan_finished(self):
        self.thread = self.worker = None
        self.results.running(False)
        self.a_run.setEnabled(True)

    def _plan_done(self, plan, ev, site):
        self._plan_finished()
        self.last_site = site
        self.canvas.set_result(plan, ev)
        self.results.show_result(plan, ev)
        self.canvas.zoom_to_fit()                # include every planned camera
        self.view3d.fit()
        if ev.summary["fraction_meeting_all_requirements"] < 0.9999:
            self.results.log(self.results.status.text() + "  Not every point meets the requirement: the "
                             "Advisor tab suggests fixes; 3D view → Coverage → “Problems only” shows where.")
        self.preview_fix(None)
        self.advisor.refresh_health()

    def preview_fix(self, rec):
        """Show a suggested change on the plan and 3D view before applying it."""
        self.canvas.preview = rec
        self.view3d.preview = rec
        self.canvas.viewport().update()
        self.view3d.update()
        if rec is not None:
            self.canvas.zoom_to_fit()            # includes the suggested position
            self.statusBar().showMessage(f"{rec.title}: {rec.detail}", 15000)

    def apply_fixes(self, recs):
        """Apply advisor changes in order, re-evaluate, and show the result."""
        from idps.plan.advisor import apply
        from idps.plan.model import Plan
        from idps.plan.planner import evaluate
        from idps.plan.site import site_from_dict
        if self.canvas.result is None or not recs:
            return
        site = site_from_dict(self.doc.to_site_dict(), str(self.doc.path) if self.doc.path else None)
        plan, _ = self.canvas.result
        cams = list(plan.cameras)
        for r in recs:
            cams = apply(cams, r)
        ev = evaluate(site, cams)
        from idps.plan.planner import fine_check
        ev.summary["fine_check"] = fine_check(site, cams, max_points=150_000)    # quick: runs on the UI thread
        summary = dict(ev.summary)
        summary["optimiser"] = {**plan.summary.get("optimiser", {}),
                                "stopped_because": f"edited by the advisor ({len(recs)} change(s))"}
        new_plan = Plan(plan.site_name, plan.site_file, cams, summary, site.frame.to_dict())
        self.last_site = site
        self.canvas.set_result(new_plan, ev)
        self.results.show_result(new_plan, ev)
        self.preview_fix(None)
        self.advisor.refresh_health()
        self.view3d.update()
        changed = ", ".join(r.title for r in recs)
        self.statusBar().showMessage(f"Applied: {changed}. ⌘R re-plans from scratch if you prefer.", 15000)

    def _plan_failed(self, msg):
        self._plan_finished()
        self.results.log("Planning failed.")
        summary, _, detail = msg.partition("\n\n")
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Icon.Warning)
        box.setText("Planning failed")
        box.setInformativeText(summary)
        if detail:
            box.setDetailedText(detail)          # traceback stays available, but out of the way
        box.setWindowModality(Qt.WindowModality.WindowModal)
        self._exec_dialog(box)

    def _plan_cancelled(self):
        self._plan_finished()
        self.results.log("Planning cancelled.")

    def _write_outputs(self, folder: Path):
        from idps.plan.report import write_report
        plan, ev = self.canvas.result
        folder.mkdir(parents=True, exist_ok=True)
        plan.save(folder / "plan.yaml")
        plan.write_csv(folder / "cameras.csv")
        from idps.plan.measure import chains_for_site, write_csv
        write_csv(folder / "measurements.csv", chains_for_site(self.last_site, plan.cameras))
        write_report(folder / "report.html", self.last_site, plan, ev, "IDPS camera plan")
        return folder / "report.html"

    def export_plan(self):
        if self.canvas.result is None:
            return
        folder = QFileDialog.getExistingDirectory(self, "Export Plan To Folder",
                                                  self.settings.value("lastDir", str(Path.home())))
        if folder:
            report = self._write_outputs(Path(folder))
            self.statusBar().showMessage(f"Saved plan.yaml, cameras.csv, measurements.csv and report.html to {folder}", 8000)
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(report)))

    def open_report(self):
        if self.canvas.result is None:
            return
        folder = Path(tempfile.mkdtemp(prefix="idps-plan-"))
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(self._write_outputs(folder))))

    def about(self):
        from idps import __version__
        QMessageBox.about(self, "About IDPS", f"<b>IDPS</b> {__version__}<br>Indoor Drone Positioning System"
                          "<br><br>Planner: draw the venue, plan cameras, export the install sheet."
                          "<br>Simulate: film simulated drones with the planned cameras and check the accuracy.")

    def closeEvent(self, e):
        if not self._maybe_save():
            e.ignore()
            return
        self.live.shutdown()
        self.sim.shutdown()
        self.px4.shutdown()
        for thread, worker in list(self._running):
            worker.stop()
            thread.quit()
            thread.wait(10000)
        self.settings.setValue("window/geometry", self.saveGeometry())
        e.accept()


def main_window_for_tests() -> MainWindow:     # pragma: no cover - convenience
    return MainWindow()


if __name__ == "__main__":                     # pragma: no cover
    from idps.ui.app import main
    sys.exit(main())

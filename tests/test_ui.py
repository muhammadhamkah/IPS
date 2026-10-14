"""Headless dashboard tests (QT_QPA_PLATFORM=offscreen)."""
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6.QtWidgets", exc_type=ImportError)

from PySide6.QtCore import QPoint, QPointF, Qt  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from idps.plan.site import site_from_dict  # noqa: E402


@pytest.fixture(scope="module")
def app():
    a = QApplication.instance() or QApplication([])
    a.setOrganizationName("IDPS-tests")
    a.setApplicationName("IDPS-tests")
    yield a


@pytest.fixture
def win(app):
    from idps.ui.main_window import MainWindow
    w = MainWindow()
    w.resize(1200, 800)
    w.show()
    app.processEvents()
    yield w
    w.doc.undo.setClean()
    w.close()


def click(canvas, x, y, double=False):
    p = canvas.mapFromScene(QPointF(x, y))
    if double:
        QTest.mouseDClick(canvas.viewport(), Qt.MouseButton.LeftButton, pos=p)
    else:
        QTest.mouseClick(canvas.viewport(), Qt.MouseButton.LeftButton, pos=p)


def draw_rect(win, w=20, h=10, origin=True):
    """Step 1: draw the performance area; step 2 (origin=True): click the origin at (0, 0)."""
    c = win.canvas
    win.set_tool("volume")
    c.centerOn(0, 0)
    for x, y in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)):
        click(c, x, y)
    QTest.keyClick(c.viewport(), Qt.Key.Key_Return)
    if origin and c.tool == "origin":
        c.centerOn(0, 0)
        click(c, 0, 0)


def test_draw_volume_undo_redo(win):
    c = win.canvas
    c.resetTransform()
    c.scale(20, -20)
    draw_rect(win, origin=False)
    poly = win.doc.data["volume"]["polygon"]
    assert len(poly) == 4
    assert poly[0] == pytest.approx([-10, -5]) and poly[2] == pytest.approx([10, 5])
    assert win.doc.dirty and c.tool == "origin"          # step 2: place the origin
    c.centerOn(0, 0)
    click(c, 2, 1)
    assert c.tool == "select" and win.doc.data["origin"] == {"xy": [2.0, 1.0], "north_deg": 90.0}
    win.a_undo.trigger()
    assert "origin" not in win.doc.data
    win.a_undo.trigger()
    assert win.doc.data["volume"]["polygon"] == []
    win.a_redo.trigger()
    assert len(win.doc.data["volume"]["polygon"]) == 4


def test_drag_vertex_is_one_undo_step(win):
    c = win.canvas
    c.resetTransform()
    c.scale(20, -20)
    draw_rect(win)
    c.centerOn(0, 0)
    start = c.mapFromScene(QPointF(10, 5))
    end = c.mapFromScene(QPointF(12, 7))
    vp = c.viewport()
    QTest.mousePress(vp, Qt.MouseButton.LeftButton, pos=start)
    for t in (0.3, 0.6, 1.0):
        QTest.mouseMove(vp, QPoint(int(start.x() + (end.x() - start.x()) * t),
                                   int(start.y() + (end.y() - start.y()) * t)))
    QTest.mouseRelease(vp, Qt.MouseButton.LeftButton, pos=end)
    assert win.doc.data["volume"]["polygon"][2] == pytest.approx([12, 7])
    win.a_undo.trigger()
    assert win.doc.data["volume"]["polygon"][2] == pytest.approx([10, 5])


def test_truss_tool_and_vertex_table(win):
    c = win.canvas
    c.resetTransform()
    c.scale(20, -20)
    draw_rect(win)
    win.set_tool("truss")
    c.centerOn(0, 0)
    click(c, -12, 0)
    click(c, 12, 0)
    t = win.doc.data["trusses"][0]
    assert t["id"] == "T1" and t["start"] == pytest.approx([-12, 0]) and t["end"] == pytest.approx([12, 0])
    assert t["height_m"] == pytest.approx(win.doc.data["volume"]["z"][1] + 1.5)
    # select the volume and type an exact coordinate into the vertex table: columns are N, E from
    # the origin (North = up the plan by default, so East = drawing +X)
    win.canvas.set_selection(("vertex", 1))
    from idps.ui.planner.inspector import VertexTable
    table = win.inspector.findChildren(VertexTable)[-1]
    assert float(table.item(1, 0).text()) == pytest.approx(-5) and float(table.item(1, 1).text()) == pytest.approx(10)
    table.item(1, 1).setText("11.25")
    assert win.doc.data["volume"]["polygon"][1] == pytest.approx([11.25, -5])


def test_save_load_roundtrip_and_site_validates(win, tmp_path):
    c = win.canvas
    c.resetTransform()
    c.scale(20, -20)
    draw_rect(win)
    win.inspector._add_strip()
    path = tmp_path / "venue.yaml"
    win.doc.save(path)
    assert not win.doc.dirty
    site = site_from_dict(win.doc.to_site_dict())
    assert site.volume.floor_area_m2 == pytest.approx(200)
    assert site.tripod_zones[0].offset_m == (1.0, 3.0)
    win.doc.load(path)
    assert len(win.doc.data["volume"]["polygon"]) == 4


def test_open_example_and_plan(win):
    from idps.ui.main_window import EXAMPLES
    from idps.ui.planner.worker import PlanWorker
    win.open_doc(str(EXAMPLES / "lab.yaml"))
    assert len(win.doc.data["volume"]["polygon"]) == 4
    got = {}
    worker = PlanWorker(win.doc.to_site_dict(), None)
    worker.finished.connect(lambda plan, ev, site: got.update(plan=plan, ev=ev, site=site))
    worker.run()                                 # synchronous in the test
    assert got["ev"].summary["fraction_meeting_all_requirements"] == pytest.approx(1.0)
    win._plan_done(got["plan"], got["ev"], got["site"])
    assert win.results.table.rowCount() == len(got["plan"].cameras)
    win.results.mode_group.button(2).click()
    assert win.canvas.overlay_mode == "accuracy"
    win.canvas.set_selection(("camera", 0))
    # editing the site marks the plan stale
    win.doc.edit("x", lambda d: d["volume"]["z"].__setitem__(1, 4.0))
    assert win.canvas.result_stale


def test_worker_cancel(app):
    from idps.ui.planner.worker import PlanWorker
    from idps.ui.main_window import EXAMPLES
    import yaml
    d = yaml.safe_load(open(EXAMPLES / "hall.yaml"))
    worker = PlanWorker(d, None)
    events = []
    worker.cancelled.connect(lambda: events.append("cancelled"))
    worker.stop()
    worker.run()
    assert events == ["cancelled"]


def test_plan_without_mounts_offers_tripod_strip(win, monkeypatch):
    c = win.canvas
    c.resetTransform()
    c.scale(20, -20)
    draw_rect(win)
    started = []
    monkeypatch.setattr("idps.ui.main_window.start_worker", lambda *a: started.append(a) or (_FakeThread(), _FakeWorker()))

    # Cancel: nothing added, nothing started
    monkeypatch.setattr(win, "_exec_dialog", lambda box: box.buttons()[-1].click())
    win.run_plan()
    assert not win.doc.data["tripod_zones"] and not started
    # Accept: strip added and planning starts
    monkeypatch.setattr(win, "_exec_dialog", lambda box: box.buttons()[0].click())
    win.run_plan()
    assert win.doc.data["tripod_zones"][0]["offset_m"] == [1.0, 3.0]
    assert started
    win.thread = win.worker = None
    win._running.clear()


def test_app_icon_and_name_helpers(app):
    from idps.ui.app import _macos_app_name, app_icon
    assert not app_icon().isNull()
    _macos_app_name("IDPS")        # no-op without pyobjc; must not raise


class _Sig:
    def connect(self, *a):
        pass


class _FakeWorker:
    progress = finished = failed = cancelled = _Sig()


class _FakeThread:
    finished = _Sig()

    def deleteLater(self):
        pass


def test_scrolling_over_unfocused_number_field_does_not_change_it(win, app):
    from PySide6.QtCore import QPointF as QPF
    from PySide6.QtGui import QWheelEvent
    from idps.ui.planner.inspector import DSpin
    c = win.canvas
    c.resetTransform()
    c.scale(20, -20)
    draw_rect(win)
    win.canvas.set_selection(("volume", None))
    app.processEvents()
    top = [s for s in win.inspector.findChildren(DSpin) if s.isVisible() and s.value() == 10.0][0]
    ev = QWheelEvent(QPF(5, 5), QPF(5, 5), QPoint(0, -120), QPoint(0, -120), Qt.MouseButton.NoButton,
                     Qt.KeyboardModifier.NoModifier, Qt.ScrollPhase.NoScrollPhase, False)
    QApplication.sendEvent(top, ev)
    assert top.value() == 10.0
    assert win.doc.data["volume"]["z"] == [0.0, 10.0]


def test_top_cannot_go_below_floor_and_bad_heights_explained(win, monkeypatch):
    from idps.ui.planner.inspector import volume_z_editors
    c = win.canvas
    c.resetTransform()
    c.scale(20, -20)
    draw_rect(win)
    z0, z1 = volume_z_editors(win.doc)
    assert z1.minimum() == pytest.approx(0.5) and z0.maximum() == pytest.approx(9.5)
    # a file edited by hand can still contain bad heights: planning explains instead of crashing
    win.doc.edit("bad", lambda d: d["volume"]["z"].__setitem__(1, 0.0))
    win.inspector._add_strip()
    shown = []
    monkeypatch.setattr(win, "_exec_dialog", lambda box: shown.append(box.informativeText()))
    monkeypatch.setattr("idps.ui.main_window.start_worker", lambda *a: (_ for _ in ()).throw(AssertionError("started")))
    win.run_plan()
    assert shown and "ceiling" in shown[0] and "lowest flight height" in shown[0]


def test_settings_tab_has_volume_height(win):
    from idps.ui.planner.inspector import DSpin
    page = win.inspector.settings
    vals = sorted(s.value() for s in page.vol_box.findChildren(DSpin))
    assert vals == [0.0, 10.0]


def test_edit_height_on_settings_tab(win, app):
    page = win.inspector.settings
    page.z1.setValue(14.0)
    page.z1.editingFinished.emit()
    app.processEvents()
    assert win.doc.data["volume"]["z"][1] == pytest.approx(14.0)
    win.a_undo.trigger()
    app.processEvents()
    assert page.z1.value() == pytest.approx(10.0)


def test_ctrl_c_handler_closes_window(app, monkeypatch):
    import signal

    from idps.ui.app import _quit_on_ctrl_c
    closed = []

    class W:
        def close(self):
            closed.append(True)
    old = signal.getsignal(signal.SIGINT)
    try:
        monkeypatch.setattr(app, "window", W(), raising=False)
        _quit_on_ctrl_c(app)
        signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
        assert closed == [True]
    finally:
        signal.signal(signal.SIGINT, old)
        app._sigint_timer.stop()


def test_window_close_then_undo_signals_do_not_raise(app):
    from idps.ui.main_window import MainWindow
    w = MainWindow()
    doc = w.doc
    w.close()
    w.deleteLater()
    app.processEvents()
    doc.undo.setClean()           # used to call a lambda on the deleted window


def _venue_with_mounts(win):
    c = win.canvas
    c.resetTransform()
    c.scale(20, -20)
    draw_rect(win)
    win.set_tool("truss")
    c.centerOn(0, 0)
    click(c, -9, 7)
    click(c, 9, 7)
    win.set_tool("select")
    win.inspector._add_strip()
    win.canvas.set_selection(None)


def test_mounts_list_one_click_remove(win, app):
    from PySide6.QtWidgets import QPushButton
    _venue_with_mounts(win)
    app.processEvents()
    btn = win.inspector.findChild(QPushButton, "remove_truss_0")
    assert btn is not None
    btn.click()
    app.processEvents()
    assert win.doc.data["trusses"] == [] and len(win.doc.data["tripod_zones"]) == 1
    win.inspector.findChild(QPushButton, "remove_zone_0").click()
    app.processEvents()
    assert win.doc.data["tripod_zones"] == []
    win.a_undo.trigger()
    win.a_undo.trigger()
    assert len(win.doc.data["trusses"]) == 1 and len(win.doc.data["tripod_zones"]) == 1


def test_remove_all_and_item_delete_button(win, app):
    from PySide6.QtWidgets import QPushButton
    _venue_with_mounts(win)
    app.processEvents()
    [b for b in win.inspector.findChildren(QPushButton)
     if b.text() == "Remove all tripod zones" and b.isVisible()][0].click()
    app.processEvents()
    assert win.doc.data["tripod_zones"] == []
    win.canvas.set_selection(("truss", 0))
    app.processEvents()
    [b for b in win.inspector.findChildren(QPushButton) if b.text() == "Delete Truss T1" and b.isVisible()][0].click()
    assert win.doc.data["trusses"] == []


def test_right_click_delete_truss(win, app):
    _venue_with_mounts(win)
    c = win.canvas
    c.centerOn(0, 0)
    menu = c.context_menu_at(c.mapFromScene(QPointF(0, 7)))
    assert menu is not None and menu.actions()[0].text() == "Delete Truss T1"
    menu.actions()[0].trigger()
    assert win.doc.data["trusses"] == []
    assert c.context_menu_at(c.mapFromScene(QPointF(500, 500))) is None


def test_3d_projection_presets_fit_and_pick(win, app):
    import math

    import numpy as np
    from idps.ui.main_window import EXAMPLES
    from idps.ui.planner.worker import PlanWorker
    win.open_doc(str(EXAMPLES / "lab.yaml"))
    got = {}
    w = PlanWorker(win.doc.to_site_dict(), None)
    w.finished.connect(lambda plan, ev, site: got.update(plan=plan, ev=ev, site=site))
    w.run()
    win._plan_done(got["plan"], got["ev"], got["site"])
    win.show_view("3d")
    app.processEvents()
    v = win.view3d
    assert win.views.currentIndex() == 1 and win.a_view3d.isChecked()
    # the orbit target projects to the centre of the view
    P, z = v.project([v.target])
    assert P[0] == pytest.approx([v.width() / 2, v.height() / 2]) and z[0] > 0
    # top view looks straight down with North up the screen and East to the right
    v.set_view("top")
    ax = v._ned_axes()
    P, _ = v.project([v.target, v.target + ax["E"], v.target + ax["N"]])
    assert P[1, 0] > P[0, 0] and abs(P[1, 1] - P[0, 1]) < 1e-6 * v.height() + 1e-3
    assert P[2, 1] < P[0, 1] and abs(P[2, 0] - P[0, 0]) < 1e-3
    # fit: every camera is on screen
    v.set_view("iso")
    P, z = v.project([c.position_m for c in got["plan"].cameras])
    assert (z > 0).all() and (P[:, 0] >= 0).all() and (P[:, 0] <= v.width()).all()
    assert (P[:, 1] >= 0).all() and (P[:, 1] <= v.height()).all()
    # clicking a camera selects it everywhere
    from PySide6.QtCore import QPointF as QPF
    v._pick(QPF(*P[2]))
    assert win.canvas.selection == ("camera", 2)
    assert win.results.table.currentRow() == 2
    # painting with cones for all cameras and the coverage layer must not fail
    v.show_cones = "all"
    img = v.grab()
    assert not img.isNull()
    # choosing a drawing tool returns to the 2D plan
    win.set_tool("truss")
    assert win.views.currentIndex() == 0
    assert math.isfinite(v.dist)


def test_3d_gizmo_axis_snap_buttons_and_ball_drag(win, app):
    import math

    import numpy as np
    draw_rect(win)
    win.show_view("3d")
    app.processEvents()
    v = win.view3d
    v.grab()                                            # paint once so the gizmo hit regions exist
    tips = {n: p for n, p, _ in v._axis_points()}
    QTest.mouseClick(v, Qt.MouseButton.LeftButton, pos=tips["-D"].toPoint())     # U: top view
    assert math.degrees(v.pitch) == pytest.approx(89.9)
    v.grab()
    tips = {n: p for n, p, _ in v._axis_points()}
    QTest.mouseClick(v, Qt.MouseButton.LeftButton, pos=tips["-N"].toPoint())     # S: from the South side
    assert math.degrees(v.pitch) == pytest.approx(0) and math.degrees(v.yaw) == pytest.approx(-90)
    v.grab()
    acts = {a: r for r, a in v._gizmo_hits if a != "orbit"}
    target, dist, yaw = v.target.copy(), v.dist, v.yaw
    QTest.mouseClick(v, Qt.MouseButton.LeftButton, pos=acts["pan_right"].center().toPoint())
    assert not np.allclose(v.target, target)
    QTest.mouseClick(v, Qt.MouseButton.LeftButton, pos=acts["zoom_in"].center().toPoint())
    assert v.dist < dist
    QTest.mouseClick(v, Qt.MouseButton.LeftButton, pos=acts["rot_left"].center().toPoint())
    assert v.yaw != pytest.approx(yaw)
    QTest.mouseClick(v, Qt.MouseButton.LeftButton, pos=acts["home"].center().toPoint())
    assert math.degrees(v.pitch) == pytest.approx(30)
    # dragging on the ball orbits instead of selecting
    c, r = v._gizmo_geometry()
    yaw = v.yaw
    QTest.mousePress(v, Qt.MouseButton.LeftButton, pos=c.toPoint())
    QTest.mouseMove(v, (c + QPointF(30, 0)).toPoint())
    QTest.mouseRelease(v, Qt.MouseButton.LeftButton, pos=(c + QPointF(30, 0)).toPoint())
    assert v.yaw != pytest.approx(yaw)
    assert v.gizmo_tooltip(acts["zoom_in"].center()) == "Zoom in"


def test_finishing_a_shape_returns_to_select(win):
    c = win.canvas
    c.resetTransform()
    c.scale(20, -20)
    draw_rect(win)
    win.set_tool("truss")
    c.centerOn(0, 0)
    click(c, -9, 7)
    click(c, 9, 7)
    assert c.tool == "select" and win.tool_actions["select"].isChecked()


def test_mount_counts_groups_shared_spots():
    from idps.plan.model import PlannedCamera
    from idps.ui.planner.view3d import mount_counts

    def cam(mtype, x, y, z):
        return PlannedCamera("C", mtype, "m", [x, y, z], 0.0, 0.0, 2.8)
    counts = mount_counts([cam("truss", 1, 2, 11.2), cam("truss", 1, 2, 11.2), cam("truss", 4, 2, 11.2),
                           cam("tripod", 0, -3, 3.0), cam("tripod", 0, -3, 10.0)])
    assert counts[(1, 2, 11.2)] == 2 and counts[(4, 2, 11.2)] == 1
    assert counts[(0, -3, 10.0)] == 2                    # one tripod, two heights on the mast


def test_3d_coverage_modes_render_and_count_problems(win, app):
    from idps.ui.planner.worker import PlanWorker

    def setup(d):
        d["volume"]["polygon"] = [[-10, -5], [-10, 7], [17.5, 7], [17.5, -5]]
        d["volume"]["z"] = [0.0, 10.0]
        d["trusses"] = [{"id": "T1", "start": [-9.5, 7.5], "end": [17.5, 7.5], "height_m": 11.5},
                        {"id": "T2", "start": [-11, -5.5], "end": [18, -5.5], "height_m": 11.5}]
        d["candidates"]["truss_elevations_deg"] = [-90, -60, -35]   # steep only: leaves the top layer short
        d["candidates"]["max_cameras_per_mount"] = 1
    win.doc.edit("setup", setup)
    got = {}
    w = PlanWorker(win.doc.to_site_dict(), None)
    w.finished.connect(lambda plan, ev, site: got.update(plan=plan, ev=ev, site=site))
    w.run()
    win._plan_done(got["plan"], got["ev"], got["site"])
    assert "Problems only" in win.results.status.text()          # short plan points the user to the 3D view
    win.show_view("3d")
    v = win.view3d
    n_bad, vol_bad = v.coverage_stats()
    assert n_bad == int((~got["ev"].meets).sum()) > 0 and vol_bad > 0
    for i in range(win.cov_combo.count()):
        win.cov_combo.setCurrentIndex(i)
        assert v.coverage_mode == win.cov_combo.itemData(i)
        assert not v.grab().isNull()


def test_measure_tool_and_dimensions_toggle(win, app):
    c = win.canvas
    c.resetTransform()
    c.scale(20, -20)
    draw_rect(win)                                     # 20 x 10 m
    win.set_tool("measure")
    c.centerOn(0, 0)
    msgs = []
    c.message.connect(msgs.append)
    click(c, -10, -5)                                  # snaps to V0
    click(c, 10, -5)                                   # snaps to V3
    assert c.ruler == [(-10.0, -5.0), (10.0, -5.0)]
    assert "20.000 m" in msgs[-1]
    QTest.keyClick(c.viewport(), Qt.Key.Key_Escape)
    assert c.ruler == []
    win.a_dims.trigger()
    assert c.show_dimensions is False


def test_advisor_panel_find_apply_and_auto_fix(win, app):
    import time

    from PySide6.QtWidgets import QPushButton
    from idps.ui.planner.worker import PlanWorker

    def setup(d):
        d["volume"]["polygon"] = [[-10, -5], [-10, 7], [17.5, 7], [17.5, -5]]
        d["volume"]["z"] = [0.0, 10.0]
        d["trusses"] = [{"id": "T1", "start": [-9.5, 7.5], "end": [17.5, 7.5], "height_m": 11.5},
                        {"id": "T2", "start": [-11, -5.5], "end": [18, -5.5], "height_m": 11.5}]
        d["candidates"]["truss_elevations_deg"] = [-90, -60, -35]   # steep only: leaves the top layer short
    win.doc.edit("setup", setup)
    got = {}
    w = PlanWorker(win.doc.to_site_dict(), None)
    w.finished.connect(lambda p, e, s: got.update(p=p, e=e, s=s))
    w.run()
    win._plan_done(got["p"], got["e"], got["s"])
    win.inspector._add_strip()
    adv = win.advisor
    assert "fail the requirement" in adv.status.text()

    def wait():
        t0 = time.time()
        while (adv.worker is not None or adv._running) and time.time() - t0 < 240:
            app.processEvents()
            time.sleep(0.02)
        for _ in range(3):
            app.processEvents()

    def buttons(prefix):
        return [b for b in adv.findChildren(QPushButton) if b.text().startswith(prefix) and not b.isHidden()]
    adv.find_fixes()
    wait()
    assert buttons("Show") and buttons("Apply")
    buttons("Show")[0].click()
    assert win.canvas.preview is not None and win.view3d.preview is not None
    before = got["e"].summary["fraction_meeting_all_requirements"]
    buttons("Apply")[0].click()
    after = win.canvas.result[1].summary["fraction_meeting_all_requirements"]
    assert after > before and win.canvas.preview is None
    adv.auto_fix()
    wait()
    buttons("Apply all")[0].click()
    assert win.canvas.result[1].summary["fraction_meeting_all_requirements"] == pytest.approx(1.0)
    assert "✅" in adv.status.text()


def test_live_tab_brain_alerts_and_accept(win, app):
    """Live tab: simulated show, injected faults, alert cards, Accept / Live with it."""
    from PySide6.QtWidgets import QPushButton
    from idps.plan.model import Plan
    from idps.plan.planner import evaluate
    from idps.plan.site import site_from_dict
    win.open_doc("examples/sites/lab.yaml")
    site = site_from_dict(win.doc.to_site_dict(), str(win.doc.path))
    plan = Plan.load("examples/plans/lab.yaml")
    win._plan_done(plan, evaluate(site, plan.cameras), site)
    win.tabs.setCurrentWidget(win.live)
    assert not win.toolbar.isVisible()                    # drawing tools belong to the Planner
    live = win.live
    live.n_drones.setValue(6)
    assert live.start()
    live.step(50)
    assert live.view.drones and not live.brain.issues
    cams = [c.id for c in live.show_sim.cams]
    live.fault_bump(cams[2])
    live.fault_offline(cams[5])
    live.step(20)
    kinds = {i.kind for i in live.brain.issues.values()}
    assert {"camera_moved", "camera_offline"} <= kinds
    moved = next(i for i in live.brain.issues.values() if i.kind == "camera_moved")
    card = live.cards[moved.id]
    restore = card.findChild(QPushButton, "accept")
    assert restore.text() == "Restore camera" and not restore.isHidden()
    restore.click()
    live.step(2)
    assert moved.state == "resolved"
    off = next(i for i in live.brain.issues.values() if i.kind == "camera_offline")
    live.cards[off.id].findChild(QPushButton, "live_with").click()
    assert off.state == "lived_with"
    live.wait_analysis(120)
    live.fault_offline(cams[5], False)
    live.step(2)
    assert off.state == "resolved"

    live.fault_obstruction()
    for _ in range(300):
        live.step(10)
        if any(i.kind == "blind_spot" for i in live.brain.issues.values()):
            break
    iss = next(i for i in live.brain.issues.values() if i.kind == "blind_spot")
    for _ in range(5):
        live.wait_analysis(240)
        live.step(1)
    live.wait_analysis(240)
    live.step(1)
    if iss.fixes:
        before = list(win.canvas.result[0].cameras)
        live.show_issue(iss.id)
        assert live.view.highlight == iss.id and live.view.preview is iss.fixes[0]
        live.cards[iss.id].findChild(QPushButton, "accept").click()
        assert iss.state == "fix_accepted"
        assert win.canvas.result[0].cameras != before            # the plan follows the accepted fix
        assert live.show_sim.cams == list(win.canvas.result[0].cameras)
    else:
        assert iss.fix_text                                     # explains why nothing fits
    win.grab()                                                  # renders the live layers
    live.stop()
    assert live.show_sim is None and not live.cards


def test_simulate_tab_runs_the_planned_layout(win, app, tmp_path):
    """Simulate tab: the planned cameras film drones, the engine finds them, the replay plays."""
    from idps.plan.model import Plan
    from idps.plan.planner import evaluate
    from idps.plan.site import site_from_dict
    from idps.show.demo import write_demo_skyc
    sim = win.sim
    win.tabs.setCurrentWidget(sim)
    assert not sim.run()                                        # nothing planned yet: explains instead
    assert "Plan the cameras first" in sim.status.text()
    win.open_doc("examples/sites/lab.yaml")
    site = site_from_dict(win.doc.to_site_dict(), str(win.doc.path))
    plan = Plan.load("examples/plans/lab.yaml")
    win._plan_done(plan, evaluate(site, plan.cameras), site)
    sim.n_drones.setValue(3)
    sim.duration.setValue(8)
    assert sim.run()
    sim.wait()
    res = sim.res
    assert res is not None and len(res.drones) == 3 and res.identified == 3
    assert res.worst_p95_mm < site.requirements.p95_mm
    assert sim.table.rowCount() == 3 and "meets the requirement" in sim.status.text()
    sim.table.selectRow(1)
    assert sim.view.selected == res.drone_ids.index(res.drones[1].drone_id)
    sim.slider.setValue(sim.slider.maximum() // 2)
    assert "markers in frame" in sim.image_note.text()
    win.grab()                                                  # renders the replay layers and camera image
    sim.toggle_play()
    # a show file: drones stand on their spots, then fly it
    sim.set_show(str(write_demo_skyc(tmp_path / "demo.skyc", n=3)))
    assert sim.mode.currentData() == "show" and sim.run()
    sim.wait()
    assert sim.res.show_start_s is not None and sim.res.identified == 3
    assert sim.clock.text().startswith("show T")


def test_window_fits_a_laptop_screen(win, app):
    """Control bars wrap instead of forcing the window wider than a 13-inch laptop."""
    assert win.minimumSizeHint().width() <= 900
    win.show_view("3d")
    app.processEvents()
    assert win.minimumSizeHint().width() <= 900
    win.resize(860, 700)
    app.processEvents()
    assert win.width() <= 900
    # a size saved on a big external display is pulled back inside the screen
    avail = win.screen().availableGeometry()
    win.resize(avail.width() + 600, avail.height() + 400)
    win.fit_to_screen()
    assert win.frameGeometry().width() <= avail.width() + 1
    assert win.frameGeometry().height() <= avail.height() + 1


def test_area_first_then_origin_drag_and_turn_north(win, app):
    """Mount tools wait for the performance area; the origin is placed, dragged and turned on
    the plan, and every coordinate shown follows it (North-East-Down)."""
    c = win.canvas
    c.resetTransform()
    c.scale(20, -20)
    win.set_tool("truss")
    assert c.tool == "volume"                                  # area first
    draw_rect(win, origin=False)
    assert c.tool == "origin"
    c.centerOn(0, 0)
    click(c, 0, 0)
    assert win.doc.data["origin"] == {"xy": [0.0, 0.0], "north_deg": 90.0}
    assert c.selection == ("origin", None)
    # drag the origin to (-10, -5) (vertex V0): one undo step
    vp = c.viewport()
    a, b = c.mapFromScene(QPointF(0, 0)), c.mapFromScene(QPointF(-10, -5))
    QTest.mousePress(vp, Qt.MouseButton.LeftButton, pos=a)
    QTest.mouseMove(vp, b)
    QTest.mouseRelease(vp, Qt.MouseButton.LeftButton, pos=b)
    assert win.doc.data["origin"]["xy"] == pytest.approx([-10, -5])
    # turn North to point along +X of the plan by dragging the N tip
    _, n_tip, _ = c._origin_geometry()
    a = c.mapFromScene(QPointF(*n_tip))
    b = c.mapFromScene(QPointF(-10 + 3, -5 + 0.2))
    QTest.mousePress(vp, Qt.MouseButton.LeftButton, pos=a)
    QTest.mouseMove(vp, b)
    QTest.mouseRelease(vp, Qt.MouseButton.LeftButton, pos=b)
    assert win.doc.data["origin"]["north_deg"] == pytest.approx(0.0)        # snapped to 15° steps
    # V2 (10, 5) is now 20 m North and 10 m West of the origin
    from idps.ui.planner.document import frame_of
    assert frame_of(win.doc.data).ne([10, 5]) == pytest.approx([20, -10])
    win.canvas.set_selection(("vertex", 2))
    from idps.ui.planner.inspector import VertexTable
    table = win.inspector.findChildren(VertexTable)[-1]
    assert float(table.item(2, 0).text()) == pytest.approx(20) and float(table.item(2, 1).text()) == pytest.approx(-10)
    win._cursor_moved(10, 5)
    assert "N    20.00" in win.cursor_label.text() and "E   -10.00" in win.cursor_label.text()
    win.a_undo.trigger()
    assert win.doc.data["origin"]["north_deg"] == pytest.approx(90.0)
    # the origin panel: North straight up the plan with one click
    win.canvas.set_selection(("origin", None))
    from PySide6.QtWidgets import QPushButton
    btn = next(b for b in win.inspector.findChildren(QPushButton) if b.text() == "North along edge E0" and not b.isHidden())
    btn.click()
    assert win.doc.data["origin"]["north_deg"] == pytest.approx(0.0)
    win.grab()


def test_plan_needs_origin_and_origin_from_v0(win, app):
    c = win.canvas
    c.resetTransform()
    c.scale(20, -20)
    draw_rect(win, origin=False)
    win.inspector._add_strip()
    win.run_plan()
    assert c.tool == "origin" and win.thread is None              # asked to place the origin first
    c.centerOn(0, 0)
    click(c, 0, 0)
    # type the origin's exact offset from V0 (-10, -5): 3 m North, 2 m East of it
    c.set_selection(("origin", None))
    from idps.ui.planner.inspector import DSpin
    spins = [s for s in win.inspector.findChildren(DSpin) if s.prefix() in ("N ", "E ") and not s.isHidden()]
    n_spin = next(s for s in spins if s.prefix() == "N ")
    e_spin = next(s for s in spins if s.prefix() == "E ")
    assert n_spin.value() == pytest.approx(5) and e_spin.value() == pytest.approx(10)
    n_spin.setValue(3)
    n_spin.editingFinished.emit()
    assert win.doc.data["origin"]["xy"] == pytest.approx([0, -2])           # North = up: 3 m up from V0


def test_px4_flight_tab_replays_a_run(win, app, tmp_path):
    """PX4 flight tab: a run folder (show files + idps_show's tracking log) is replayed with each
    drone's distance from its file."""
    import json

    import numpy as np

    from idps.show.trj import Trj, write_trj
    from idps.sim.px4run import load_run
    show = tmp_path / "show"
    show.mkdir()
    n = 201                                                      # 10 s at 20 Hz: drone 1 climbs, drone 2 hovers
    rows = []
    for did, east in ((1, -1.0), (2, 1.0)):
        pos = np.zeros((n, 3))
        pos[:, 1] = east
        pos[:, 2] = -np.linspace(0, 2, n) if did == 1 else -1.0
        write_trj(show / f"drone_{did:03d}.trj", Trj(did, 20.0, 0.0, np.array([0, east, 0.0]), 0.0, pos,
                                                       np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 3))))
        rows.append({"drone_id": did, "trj": f"drone_{did:03d}.trj", "home_ned": [0, east, 0]})
    (show / "manifest.json").write_text(json.dumps({"venue": {"origin": {"xy": [0, 0], "north_deg": 90}},
                                                    "drones": rows}))
    lines = ["drone_id,show_t,dn,de,dd"]
    for k in range(1000):                                        # 100 Hz; drone 2 is 40 mm off at 6 s
        t = k * 0.01
        lines.append(f"1,{t:.2f},0.005,0,0")
        lines.append(f"2,{t:.2f},{0.04 if 5.95 < t < 6.05 else 0.003},0,0")
    (tmp_path / "track.csv").write_text("\n".join(lines) + "\n")

    run = load_run(tmp_path)
    assert run.mode == "cameras" and run.drone_ids == [1, 2]
    d1, d2 = run.drones
    assert abs(d1.median_mm - 5) < 0.01 and abs(d2.worst_mm - 40) < 0.01 and abs(d2.worst_at_s - 6) < 0.06
    k = int(np.searchsorted(run.t, 5.0))
    assert np.allclose(run.actual[0, k], run.plan[0, k] + [0.005, 0, 0])   # the file + the logged distance
    assert np.isnan(run.error_mm[0, -1])                                    # no samples after 10 s

    tab = win.px4
    win.tabs.setCurrentWidget(tab)
    assert tab.open_run(tmp_path)
    assert tab.t_worst.v.text() == "40.0 mm" and tab.t_within.v.text() == "1/2"
    assert "1 of 2 drones stayed within 30 mm" in tab.status.text()
    tab.table.selectRow(1)
    tab._jump_to_worst(1, 0)
    assert abs(run.t[tab.slider.value()] - 6.0) < 0.06 and tab.view.selected == 1
    tab.toggle_play()
    assert tab.timer.isActive()
    tab.toggle_play()


def test_px4_flight_tab_explains_a_failed_run():
    """The flight script's output becomes a plain reason, a stage and a state per drone."""
    from idps.ui.px4.tab import diagnose
    out = "\n".join(["== show", "== 5 PX4 drones (micro60)", "== upload and fly (about 80 s)", "5 drone(s) uploaded",
                     "drone 2: denied (ready) no position (IDPS positions not arriving)",
                     "Not started: every drone was told to abort. Fix the refused drone(s) and start again.",
                     "   drone 5: WARN  [failsafe] Failsafe activated",
                     "drone 3: nothing on udp 14802 for 15 s. Is another program using that port?"])
    stage, reason, notes = diagnose(out)
    assert stage == "uploading the show" and reason.startswith("Not started: a drone refused")
    assert notes == {2: "refused: no position (IDPS positions not arriving)", 5: "Failsafe activated",
                     3: "no data (udp 14802 silent)"}
    _, reason, _ = diagnose("udp port(s) [14801, 14803] are in use, probably by an earlier run")
    assert "ports [14801, 14803] busy" in reason
    _, reason, _ = diagnose("PX4 drone 4 (instance 3): the idps_show module is not running (is PX4 built with "
                            "EXTERNAL_MODULES_LOCATION=.../IPS/px4?). The end of its log (x):")
    assert reason.startswith("PX4 drone 4: the idps_show module is not running")
    _, reason, _ = diagnose("PX4 drone 1 (instance 0): PX4 stopped while starting. The end of its log (x):\n"
                            "INFO  [px4] PX4 server already running for instance 0")
    assert reason.startswith("A PX4 from an earlier flight was still running (instance 0)")


def test_px4_camera_loop_uses_the_paced_build_when_patched(tmp_path):
    from idps.ui.px4.tab import SIH_SOURCE, camera_build
    assert camera_build(tmp_path) == "px4_sitl_nolockstep"             # no PX4 source: the real-clock build
    src = tmp_path / SIH_SOURCE
    src.parent.mkdir(parents=True)
    src.write_text("void Sih::lockstep_loop() {}")
    assert camera_build(tmp_path) == "px4_sitl_nolockstep"             # unpatched
    src.write_text("uint64_t next_wall_time_us = 0;")
    assert camera_build(tmp_path) == "px4_sitl_default"                # sih_realtime_pacing.patch applied


def test_px4_stop_kill_does_not_reach_a_newer_run(monkeypatch):
    import signal
    from idps.ui.px4 import tab as px4tab
    t = px4tab.Px4Tab.__new__(px4tab.Px4Tab)
    sent = []
    monkeypatch.setattr(px4tab.os, "killpg", lambda pid, sig: sent.append((pid, sig)))

    class P:
        def __init__(self, pid): self.pid = pid
        def processId(self): return self.pid
    old, new = P(11), P(22)
    t.proc = new                                           # the stopped run (old) ended, a new one started
    t._signal_group(signal.SIGKILL, old)
    assert sent == []
    t._signal_group(signal.SIGKILL, new)
    assert sent == [(22, signal.SIGKILL)]

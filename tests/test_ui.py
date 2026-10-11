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


def draw_rect(win, w=20, h=10):
    c = win.canvas
    win.set_tool("volume")
    c.centerOn(0, 0)
    for x, y in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)):
        click(c, x, y)
    QTest.keyClick(c.viewport(), Qt.Key.Key_Return)


def test_draw_volume_undo_redo(win):
    c = win.canvas
    c.resetTransform()
    c.scale(20, -20)
    draw_rect(win)
    poly = win.doc.data["volume"]["polygon"]
    assert len(poly) == 4
    assert poly[0] == pytest.approx([-10, -5]) and poly[2] == pytest.approx([10, 5])
    assert win.doc.dirty and c.tool == "select"
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
    # select the volume and type an exact coordinate into the vertex table
    win.canvas.set_selection(("vertex", 1))
    from idps.ui.planner.inspector import VertexTable
    table = win.inspector.findChildren(VertexTable)[-1]
    table.item(1, 0).setText("11.25")
    assert win.doc.data["volume"]["polygon"][1][0] == pytest.approx(11.25)


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
    assert shown and "Top Z" in shown[0] and "Floor Z" in shown[0]


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

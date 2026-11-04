"""Entry point: `idps-dashboard [site.yaml]`."""
from __future__ import annotations

import sys

from PySide6.QtCore import QEvent, Qt
from PySide6.QtWidgets import QApplication


class DashboardApp(QApplication):
    """Handles macOS Finder 'open with' / dock drops via QFileOpenEvent."""

    def __init__(self, argv):
        super().__init__(argv)
        self.window = None
        self._pending_open = None

    def event(self, e):
        if e.type() == QEvent.Type.FileOpen:
            path = e.file()
            if self.window is not None:
                self.window.open_doc(path)
            else:
                self._pending_open = path
            return True
        return super().event(e)


def _macos_app_name(name: str) -> None:
    """When launched from Terminal the process is 'Python', so the macOS menu
    bar and Dock show that. Renaming the bundle info before Qt starts fixes it
    (needs pyobjc, installed with the ui extra on macOS; silently skipped otherwise)."""
    try:
        from Foundation import NSBundle
    except ImportError:
        return
    bundle = NSBundle.mainBundle()
    info = bundle.localizedInfoDictionary() or bundle.infoDictionary()
    if info is not None:
        info["CFBundleName"] = name
        info["CFBundleDisplayName"] = name


def app_icon():
    """Simple drawn icon (replaces the Python rocket in the Dock and dialogs)."""
    from PySide6.QtCore import QPointF, QRectF
    from PySide6.QtGui import QColor, QIcon, QPainter, QPen, QPixmap
    pm = QPixmap(512, 512)
    pm.fill(Qt.GlobalColor.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)
    p.setBrush(QColor("#184f95"))
    p.setPen(Qt.PenStyle.NoPen)
    p.drawRoundedRect(QRectF(24, 24, 464, 464), 104, 104)
    p.setPen(QPen(QColor("#9ec5f4"), 14))
    for a, b in (((110, 380), (256, 150)), ((402, 380), (256, 150)), ((110, 380), (402, 380))):
        p.drawLine(QPointF(*a), QPointF(*b))                  # three camera rays to a marker
    p.setPen(Qt.PenStyle.NoPen)
    p.setBrush(QColor("#ffffff"))
    p.drawEllipse(QPointF(256, 150), 46, 46)
    p.setBrush(QColor("#eb6834"))
    for c in ((110, 380), (402, 380)):
        p.drawEllipse(QPointF(*c), 34, 34)
    p.end()
    return QIcon(pm)


def _quit_on_ctrl_c(app) -> None:
    """Ctrl+C in the launching Terminal closes the window normally (with the
    unsaved-changes prompt) instead of raising KeyboardInterrupt inside Qt."""
    import signal

    from PySide6.QtCore import QTimer

    def handler(*_):
        if app.window is not None:
            app.window.close()
        else:
            app.quit()
    signal.signal(signal.SIGINT, handler)
    timer = QTimer(app)              # let the Python interpreter run so it can see the signal
    timer.timeout.connect(lambda: None)
    timer.start(200)
    app._sigint_timer = timer


def main(argv=None) -> int:
    argv = list(sys.argv if argv is None else argv)
    if sys.platform == "darwin":
        _macos_app_name("IDPS")
    QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = DashboardApp(argv)
    app.setApplicationName("IDPS")
    app.setApplicationDisplayName("IDPS")
    app.setOrganizationName("IDPS")
    app.setOrganizationDomain("idps.local")
    app.setWindowIcon(app_icon())
    if sys.platform == "darwin":
        app.setAttribute(Qt.ApplicationAttribute.AA_DontShowIconsInMenus, True)

    from idps.ui.main_window import MainWindow
    win = MainWindow()
    app.window = win
    win.show()
    path = app._pending_open or next((a for a in argv[1:] if a.endswith((".yaml", ".yml"))), None)
    if path:
        win.open_doc(path)
    else:
        win.canvas.zoom_to_fit()
    _quit_on_ctrl_c(app)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

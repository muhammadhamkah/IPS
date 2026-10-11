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


def main(argv=None) -> int:
    argv = list(sys.argv if argv is None else argv)
    QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = DashboardApp(argv)
    app.setApplicationName("IDPS")
    app.setApplicationDisplayName("IDPS")
    app.setOrganizationName("IDPS")
    app.setOrganizationDomain("idps.local")
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
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

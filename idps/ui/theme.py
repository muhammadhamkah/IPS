"""Colour tokens for the dashboard, following the system light/dark appearance.

Data colours come from the reference data-viz palette (same as the HTML report):
status steps for camera-count coverage, a one-hue blue ramp for accuracy, and
categorical slots for mount types.
"""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QGuiApplication

STATUS = ["#d03b3b", "#ec835a", "#fab219", "#0ca30c"]           # 0, 1, 2, 3+ cameras
BLUE_RAMP = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]

LIGHT = {
    "surface": "#fcfcfb", "grid_minor": "#efeeea", "grid_major": "#dddbd5", "axis": "#8a8984",
    "text": "#0b0b0b", "text2": "#52514e", "volume": "#2a78d6", "truss": "#6b6a65",
    "truss_gap": "#c9c7c0", "zone": "#eb6834", "nogo": "#d03b3b", "select": "#6250d6",
    "cam_truss": "#2a78d6", "cam_tripod": "#eb6834", "handle_fill": "#ffffff",
}
DARK = {
    "surface": "#1a1a19", "grid_minor": "#252523", "grid_major": "#353532", "axis": "#8a8984",
    "text": "#f2f1ee", "text2": "#b4b2ab", "volume": "#3987e5", "truss": "#a3a19a",
    "truss_gap": "#4a4945", "zone": "#d95926", "nogo": "#e66767", "select": "#9085e9",
    "cam_truss": "#3987e5", "cam_tripod": "#d95926", "handle_fill": "#1a1a19",
}


def is_dark() -> bool:
    app = QGuiApplication.instance()
    if app is None:
        return False
    try:
        return app.styleHints().colorScheme() == Qt.ColorScheme.Dark
    except AttributeError:                      # Qt < 6.5
        return app.palette().window().color().lightness() < 128


def tokens() -> dict[str, QColor]:
    return {k: QColor(v) for k, v in (DARK if is_dark() else LIGHT).items()}


def ramp_color(value: float, vmax: float) -> QColor:
    """Blue ramp, 0 -> lightest, vmax -> darkest (clipped)."""
    t = max(0.0, min(1.0, value / vmax if vmax > 0 else 0.0)) * (len(BLUE_RAMP) - 1)
    i = int(t)
    if i >= len(BLUE_RAMP) - 1:
        return QColor(BLUE_RAMP[-1])
    a, b = QColor(BLUE_RAMP[i]), QColor(BLUE_RAMP[i + 1])
    f = t - i
    return QColor.fromRgbF(a.redF() + (b.redF() - a.redF()) * f, a.greenF() + (b.greenF() - a.greenF()) * f,
                           a.blueF() + (b.blueF() - a.blueF()) * f)

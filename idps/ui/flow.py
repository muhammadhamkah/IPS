"""A layout that wraps its widgets onto the next line when the window is narrow,
so control bars never force the window wider than a laptop screen."""
from __future__ import annotations

from PySide6.QtCore import QPoint, QRect, QSize, Qt
from PySide6.QtWidgets import QLayout, QSizePolicy, QWidget


class FlowLayout(QLayout):
    def __init__(self, parent=None, margins=(0, 0, 0, 0), hspacing=6, vspacing=4):
        super().__init__(parent)
        self.setContentsMargins(*margins)
        self._items = []
        self._h, self._v = hspacing, vspacing
        self._gaps: dict[int, int] = {}          # item index -> extra space before it

    # --- QLayout API
    def addItem(self, item):
        self._items.append(item)

    def count(self):
        return len(self._items)

    def itemAt(self, i):
        return self._items[i] if 0 <= i < len(self._items) else None

    def takeAt(self, i):
        return self._items.pop(i) if 0 <= i < len(self._items) else None

    def expandingDirections(self):
        return Qt.Orientation(0)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, width):
        return self._do(QRect(0, 0, width, 0), apply=False)

    def setGeometry(self, rect):
        super().setGeometry(rect)
        h = self._do(rect, apply=True)
        # Make sure the parent reserves room for the wrapped lines (box layouts in splitters
        # do not always ask for height-for-width).
        w = self.parentWidget()
        if w is not None and w.minimumHeight() != h:
            w.setMinimumHeight(h)

    def sizeHint(self):
        return self.minimumSize()

    def minimumSize(self):
        size = QSize()
        for it in self._items:
            size = size.expandedTo(it.minimumSize())
        m = self.contentsMargins()
        return size + QSize(m.left() + m.right(), m.top() + m.bottom())

    # --- helpers
    def addSpacing(self, px: int):
        """Extra space before the next widget (dropped at the start of a line)."""
        self._gaps[len(self._items)] = self._gaps.get(len(self._items), 0) + px

    def _do(self, rect, apply):
        m = self.contentsMargins()
        r = rect.adjusted(m.left(), m.top(), -m.right(), -m.bottom())
        x, y, line_h = r.x(), r.y(), 0
        for i, it in enumerate(self._items):
            w = it.widget()
            if w is not None and w.isHidden():
                continue
            hint = it.sizeHint()
            gap = self._gaps.get(i, 0)
            if x > r.x():
                x += gap
            if x + hint.width() > r.right() + 1 and x > r.x():
                x, y = r.x(), y + line_h + self._v
                line_h = 0
            if apply:
                it.setGeometry(QRect(QPoint(x, y), hint))
            x += hint.width() + self._h
            line_h = max(line_h, hint.height())
        return y + line_h - rect.y() + m.bottom()


def flow_row(*widgets, parent=None) -> tuple[QWidget, FlowLayout]:
    """A container widget whose children wrap. Returns (widget, layout)."""
    box = QWidget(parent)
    lay = FlowLayout(box)
    sp = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
    sp.setHeightForWidth(True)
    box.setSizePolicy(sp)
    for w in widgets:
        lay.addWidget(w)
    return box, lay

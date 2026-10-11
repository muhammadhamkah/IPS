"""Editable site document: the site-file dict (same schema as examples/sites/*.yaml)
plus file path, dirty state and snapshot-based undo/redo."""
from __future__ import annotations

import copy
from pathlib import Path

import yaml
from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QUndoCommand, QUndoStack

DEFAULT_SITE = {
    "name": "Untitled venue",
    "volume": {"polygon": [], "z": [0.0, 10.0]},
    "sensor": "OV9281",
    "lenses_mm": [2.8],
    "requirements": {"min_cameras": 3, "p95_mm": 10.0, "noise": "tight"},
    "design": {"min_cameras": 4, "p95_mm": 7.0},
    "visibility": {"max_range_m": 80.0},
    "evaluation": {"spacing_m": [2.0, 2.0, 2.0], "inset_m": 0.5},
    "trusses": [],
    "tripod_zones": [],
    "no_go_zones": [],
    "candidates": {
        "truss_step_m": 3.0, "truss_drop_m": 0.3, "truss_elevations_deg": [-90, -60, -35],
        "tripod_step_m": 3.0, "tripod_heights_m": [3, 10, 18], "tripod_elevations_deg": [0, -15, -30, -50],
        "azimuth_step_deg": 30, "max_cameras_per_mount": 1, "min_camera_spacing_m": 1.5,
    },
    "optimiser": {"max_cameras": 200},
}


def normalise(d: dict) -> dict:
    """Fill defaults and convert a box volume / rect zones to polygons so the
    editor only has to deal with one representation."""
    out = copy.deepcopy(DEFAULT_SITE)
    for k, v in d.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k].update(copy.deepcopy(v))
        else:
            out[k] = copy.deepcopy(v)
    vol = out["volume"]
    if "min" in vol and "max" in vol:
        (x0, y0, z0), (x1, y1, z1) = vol["min"], vol["max"]
        out["volume"] = {"polygon": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]], "z": [z0, z1]}
    vol = out["volume"]
    vol.setdefault("polygon", [])
    z = vol.get("z", vol.pop("height_m", [0.0, 10.0]))
    vol["z"] = [0.0, float(z)] if isinstance(z, (int, float)) else [float(z[0]), float(z[1])]
    for z in out["tripod_zones"]:
        if "rect" in z:
            x0, y0, x1, y1 = z.pop("rect")
            z["polygon"] = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
    nogo = []
    for r in out["no_go_zones"]:
        if len(r) == 4 and not isinstance(r[0], (list, tuple)):
            x0, y0, x1, y1 = r
            r = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
        nogo.append(r)
    out["no_go_zones"] = nogo
    return out


class _Snapshot(QUndoCommand):
    def __init__(self, doc: "SiteDocument", before: dict, after: dict, text: str):
        super().__init__(text)
        self.doc, self.before, self.after = doc, before, after

    def redo(self):
        self.doc._set(self.after)

    def undo(self):
        self.doc._set(self.before)


class SiteDocument(QObject):
    changed = Signal()             # any content change (edit, undo, load)
    pathChanged = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.data = normalise({})
        self.path: Path | None = None
        self.undo = QUndoStack(self)
        self.undo.cleanChanged.connect(self._clean_changed)

    def _clean_changed(self, _clean: bool):
        self.pathChanged.emit(self.title())

    # --- state -------------------------------------------------------
    def _set(self, d: dict):
        self.data = copy.deepcopy(d)
        self.changed.emit()

    def snapshot(self) -> dict:
        return copy.deepcopy(self.data)

    def edit(self, text: str, fn) -> None:
        """Apply fn(data) as one undoable step."""
        before = self.snapshot()
        after = self.snapshot()
        fn(after)
        if after != before:
            self.undo.push(_Snapshot(self, before, after, text))

    def commit(self, text: str, before: dict) -> None:
        """Record a change already applied live (e.g. a vertex drag)."""
        if self.data != before:
            self.undo.push(_Snapshot(self, before, self.snapshot(), text))

    def set_live(self, fn) -> None:
        """Change without an undo step (used while dragging)."""
        fn(self.data)
        self.changed.emit()

    @property
    def dirty(self) -> bool:
        return not self.undo.isClean()

    def title(self) -> str:
        name = self.path.name if self.path else self.data.get("name", "Untitled")
        return name

    # --- files -------------------------------------------------------
    def new(self):
        self.path = None
        self.undo.clear()
        self._set(normalise({}))
        self.undo.setClean()
        self.pathChanged.emit(self.title())

    def load(self, path: str | Path):
        with open(path) as f:
            d = yaml.safe_load(f) or {}
        self.path = Path(path)
        self.undo.clear()
        self._set(normalise(d))
        self.undo.setClean()
        self.pathChanged.emit(self.title())

    def save(self, path: str | Path | None = None):
        if path is not None:
            self.path = Path(path)
        if self.path is None:
            raise ValueError("no file path")
        with open(self.path, "w") as f:
            f.write("# IDPS site file - edit in the IDPS dashboard or by hand\n")
            yaml.safe_dump(_tidy(self.data), f, sort_keys=False, default_flow_style=None, width=100)
        self.undo.setClean()
        self.pathChanged.emit(self.title())

    def to_site_dict(self) -> dict:
        return _tidy(self.data)


def _tidy(d):
    """Round floats so saved files stay readable."""
    if isinstance(d, dict):
        return {k: _tidy(v) for k, v in d.items()}
    if isinstance(d, list):
        return [_tidy(v) for v in d]
    if isinstance(d, float):
        r = round(d, 4)
        return int(r) if r.is_integer() and abs(r) < 1e9 else r
    return d

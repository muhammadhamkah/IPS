"""Read shows exported from Skybrush Studio for Blender.

* ``.skyc``, the Skybrush compiled show (a ZIP). Two layouts are accepted, since the
  documentation in the wild differs:
    a. ``show.json`` with the trajectories inside: ``swarm.drones[i].settings.trajectory``
       (or ``swarm.drones[i].trajectory``);
    b. ``show.json`` plus ``drones/<name>/trajectory.json`` (and ``lights.json``).
  A trajectory is ``{"version": 1, "points": [[t, [x, y, z], [control points]], ...],
  "takeoffTime": ..., "landingTime": ...}`` (seconds, metres). Light programmes in .skyc
  are Skybrush bytecode; they are kept as raw data (use the CSV export for colours).
* **CSV (ZIP)** from Skybrush's CSV exporter: one CSV per drone, columns time (ms), x, y,
  z (m), red, green, blue (0-255).
* A folder of such CSVs, or a single CSV.
"""
from __future__ import annotations

import csv
import io
import json
import re
import zipfile
from pathlib import Path, PurePosixPath

import numpy as np

from idps.show.model import Show, track_from_points, track_from_samples


def load_show(path: str | Path) -> Show:
    path = Path(path)
    if path.is_dir():
        return _load_csv_files({p.name: p.read_text() for p in path.glob("*.csv") if not _junk(p.name)}, str(path))
    if path.suffix.lower() == ".csv":
        return _load_csv_files({path.name: path.read_text()}, str(path))
    if not zipfile.is_zipfile(path):
        raise ValueError(f"{path}: not a .skyc or ZIP file")
    with zipfile.ZipFile(path) as z:
        names = z.namelist()
        if any(n.endswith("show.json") for n in names):
            return _load_skyc(z, str(path))
        csvs = {n: z.read(n).decode("utf-8-sig") for n in names
                if n.lower().endswith(".csv") and not _junk(n)}
        if csvs:
            return _load_csv_files(csvs, str(path))
    raise ValueError(f"{path}: no show.json or CSV files inside")


# --------------------------------------------------------------------------- .skyc
def _load_skyc(z: zipfile.ZipFile, source: str) -> Show:
    show_name = next(n for n in z.namelist() if n.endswith("show.json"))
    root = PurePosixPath(show_name).parent
    spec = json.loads(z.read(show_name))
    swarm = spec.get("swarm", {}) or {}
    entries = swarm.get("drones", []) or []
    drones = []
    folder = {}                                                     # layout b: drones/<name>/...
    for n in z.namelist():
        p = PurePosixPath(n)
        if len(p.parts) >= 3 and p.parts[-3] == "drones" and p.name in ("trajectory.json", "lights.json"):
            folder.setdefault(p.parts[-2], {})[p.name] = n
    for i, entry in enumerate(entries):
        settings = entry.get("settings", entry) or {}
        name = str(settings.get("name") or entry.get("name") or f"Drone {i + 1}")
        traj = settings.get("trajectory") or entry.get("trajectory")
        lights = settings.get("lights") or entry.get("lights")
        if traj is None:
            key = name if name in folder else (sorted(folder)[i] if i < len(folder) else None)
            if key is not None and "trajectory.json" in folder[key]:
                traj = json.loads(z.read(folder[key]["trajectory.json"]))
                if "lights.json" in folder[key]:
                    lights = json.loads(z.read(folder[key]["lights.json"]))
        if traj is None:
            raise ValueError(f"{source}: no trajectory for drone {name}")
        drones.append(_track_from_traj(name, traj, settings, lights))
    if not entries and folder:                                      # folder layout without a drone list
        for key in sorted(folder):
            traj = json.loads(z.read(folder[key]["trajectory.json"]))
            drones.append(_track_from_traj(key, traj, {}, None))
    meta = {k: spec[k] for k in ("environment", "meta", "settings", "version") if k in spec}
    return Show(drones, source, "skyc", meta)


def _track_from_traj(name, traj: dict, settings: dict, lights):
    version = traj.get("version", 1)
    if version != 1:
        raise ValueError(f"{name}: trajectory version {version} is not supported (only version 1)")
    points = traj.get("points") or []
    scale = float(traj.get("scale", 1.0) or 1.0)
    if scale != 1.0:
        points = [[t, [c * scale for c in p], [[c * scale for c in q] for q in (cps or [])]]
                  for t, p, cps in points]
    home = settings.get("home") or traj.get("home")
    tr = track_from_points(name, points, home=None if home is None else np.asarray(home, float)[:3],
                           takeoff_time=float(traj.get("takeoffTime", 0.0) or 0.0),
                           landing_time=traj.get("landingTime"), lights=lights,
                           meta={k: v for k, v in settings.items() if k not in ("trajectory", "lights")})
    return tr


# --------------------------------------------------------------------------- CSV
def _load_csv_files(files: dict, source: str) -> Show:
    drones = []
    for name, text in sorted(files.items(), key=lambda kv: _natural(kv[0])):
        rows = [r for r in csv.reader(io.StringIO(text)) if r and not r[0].lstrip().startswith("#")]
        if rows and not _is_number(rows[0][0]):
            rows = rows[1:]                                         # header
        if not rows:
            continue
        a = np.array([[float(x) for x in r[:7]] + [255.0] * max(0, 7 - len(r)) for r in rows])
        t = a[:, 0] / 1000.0                                        # milliseconds
        stem = PurePosixPath(name).stem
        drones.append(track_from_samples(stem, t, a[:, 1:4], lights=(t, a[:, 4:7]), takeoff_time=float(t[0])))
    if not drones:
        raise ValueError(f"{source}: no drone CSV files")
    return Show(drones, source, "csv")


def _junk(name: str) -> bool:
    """macOS zip debris (__MACOSX/, ._Drone 1.csv) and other hidden files."""
    p = PurePosixPath(name)
    return "__MACOSX" in p.parts or p.name.startswith(".")


def _natural(name: str):
    """Sort key so 'Drone 2' comes before 'Drone 10': Drone N keeps drone ID N."""
    return [int(x) if x.isdigit() else x.lower() for x in re.split(r"(\d+)", name)]


def _is_number(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False

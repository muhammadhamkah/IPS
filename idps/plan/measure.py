"""Installer measurements: chained distances along each truss, and along each
volume edge for tripods (plus how far out from the edge each tripod stands).

Shared by the dashboard's Dimensions overlay, the HTML report and the CSV, so
everyone measures from the same reference points.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from idps.coverage.volume import distance_to_polygon


@dataclass
class Chain:
    kind: str                    # "truss" | "edge"
    ref: str                     # truss id, or edge name "E2 (V2->V3)"
    origin: tuple                # XY of the zero point (truss start end, or the edge's first vertex)
    origin_label: str            # e.g. "T1 start end" or "V2"
    direction: tuple             # unit XY direction of the chain
    stations: list = field(default_factory=list)   # [(distance from origin m, [camera ids], offset m)]
    length: float = 0.0          # truss / edge length (stations may lie beyond it for corner tripods)
    end_label: str = ""          # e.g. "V3"

    def segments(self) -> list[float]:
        d = [0.0] + [s[0] for s in self.stations]
        return [b - a for a, b in zip(d, d[1:])]


def chains(trusses, polygon, cameras) -> list[Chain]:
    """trusses: iterable of dicts with id/start/end; polygon: (K, 2) volume outline
    (may be empty); cameras: PlannedCamera list."""
    out = []
    by_id = {t["id"]: t for t in trusses}
    # --- trusses: distance from the truss's start end to each camera mount point
    groups: dict = {}
    for c in cameras:
        if c.mount_type == "truss" and c.mount_id in by_id and c.along_truss_m is not None:
            groups.setdefault(c.mount_id, {}).setdefault(round(float(c.along_truss_m), 3), []).append(c.id)
    for tid, mounts in groups.items():
        t = by_id[tid]
        a, b = np.array(t["start"], float), np.array(t["end"], float)
        L = float(np.linalg.norm(b - a))
        if L == 0:
            continue
        u = (b - a) / L
        ch = Chain("truss", tid, tuple(a), f"{tid} start end", tuple(u), length=L, end_label=f"{tid} far end")
        ch.stations = [(s, ids, 0.0) for s, ids in sorted(mounts.items())]
        out.append(ch)
    # --- tripods: along the nearest volume edge from its first vertex, and the offset out from it
    poly = np.asarray(polygon, float)
    if len(poly) >= 3:
        spots: dict = {}
        for c in cameras:
            if c.mount_type == "tripod":
                spots.setdefault((round(c.position_m[0], 3), round(c.position_m[1], 3)), []).append(c.id)
        if spots:
            xy = np.array(list(spots), float)
            _, edge, _ = distance_to_polygon(xy, poly)
            n = len(poly)
            per_edge: dict = {}
            for (key, ids), e, q in zip(spots.items(), edge, xy):
                # Measure along the edge's *line* (negative = before the vertex) and square
                # out from it, so corner tripods get two tape-measurable numbers.
                a, b = poly[e], poly[(e + 1) % n]
                u = (b - a) / max(np.linalg.norm(b - a), 1e-9)
                along = float(np.dot(q - a, u))
                out_dist = abs(float(u[0] * (q - a)[1] - u[1] * (q - a)[0]))
                per_edge.setdefault(int(e), []).append((round(along, 3), ids, round(out_dist, 3)))
            for e, st in sorted(per_edge.items()):
                a, b = poly[e], poly[(e + 1) % n]
                u = (b - a) / max(np.linalg.norm(b - a), 1e-9)
                ch = Chain("edge", f"E{e} (V{e}->V{(e + 1) % n})", tuple(a), f"V{e}", tuple(u),
                           length=float(np.linalg.norm(b - a)), end_label=f"V{(e + 1) % n}")
                ch.stations = sorted(st)
                out.append(ch)
    return out


def chains_for_site(site, cameras) -> list[Chain]:
    trusses = [{"id": t.id, "start": list(t.start), "end": list(t.end)} for t in site.trusses]
    return chains(trusses, site.volume.polygon, cameras)


def write_csv(path, chain_list) -> None:
    """One row per camera mount: where to measure from, how far along, how far out."""
    import csv
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["reference", "measure_from", "camera_ids", "distance_from_origin_m", "from_previous_m",
                    "out_from_edge_m", "note"])
        for ch in chain_list:
            for (s, ids, off), seg in zip(ch.stations, ch.segments()):
                note = ""
                if s < 0:
                    note = f"before {ch.origin_label}, on the extended edge line"
                elif s > ch.length + 1e-6:
                    note = f"beyond {ch.end_label}, on the extended edge line"
                w.writerow([("Truss " if ch.kind == "truss" else "Edge ") + ch.ref, ch.origin_label, " ".join(ids),
                            f"{s:.3f}", f"{seg:.3f}", f"{off:.3f}" if ch.kind == "edge" else "", note])


def describe(ch: Chain) -> str:
    """One line for a printed install sheet."""
    parts = []
    for (s, ids, off), seg in zip(ch.stations, ch.segments()):
        if s < 0:
            where = f"{-s:.2f} m before {ch.origin_label} on the edge line"
        elif s > ch.length + 1e-6:
            where = f"{s:.2f} m (+{seg:.2f}; {s - ch.length:.2f} m beyond {ch.end_label} on the edge line)"
        else:
            where = f"{s:.2f} m (+{seg:.2f})"
        if ch.kind == "edge":
            where += f", {off:.2f} m out"
        parts.append(f"{'/'.join(ids)} at {where}")
    head = (f"Truss {ch.ref}, measured from its start end" if ch.kind == "truss"
            else f"Tripods along edge {ch.ref}, measured from {ch.origin_label}")
    return head + ": " + "; ".join(parts)

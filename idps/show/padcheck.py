"""Can drones take off from these charging pads and land back on them? (the Planner asks, as
pads are placed and moved)

The answer needs a show: the layered grid in front of the pads that `Fly the show` flies
(examples/shows/layers_from_pads.py), with every pad full (4 drones each). Its take-off and landing
are checked as for a real flight (idps.show.pads.check_sequence)."""
from __future__ import annotations

import importlib.util
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
_BUILDER = None


@dataclass
class PadLayoutReport:
    pads_at_fault: set = field(default_factory=set)   # 0-based pads whose drones would fly too close to others
    warning: str = ""                                 # why the layout cannot fly as placed ("" if it can)
    note: str = ""                                    # how it flies, when that is not the plain way (rows)
    drones: int = 0


def _builder():
    global _BUILDER
    if _BUILDER is None:
        spec = importlib.util.spec_from_file_location("idps_layers_from_pads", ROOT / "examples" / "shows" / "layers_from_pads.py")
        _BUILDER = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(_BUILDER)
    return _BUILDER


def _and(nums) -> str:
    nums = [str(n) for n in nums]
    return nums[0] if len(nums) == 1 else ", ".join(nums[:-1]) + " and " + nums[-1]


def pad_layout_report(site: dict) -> PadLayoutReport:
    """How the venue's pads fly with every pad full. site: the venue dict (pads, volume, origin)."""
    from idps.show.pads import SLOTS, check_sequence, pad_ranks, pads_from_site, sequence_plan
    lay = pads_from_site(site)
    if lay is None or len(lay.pads_ne) < 2:
        return PadLayoutReport()
    n = SLOTS * len(lay.pads_ne)
    try:
        ch = _builder().build(site, n)
        _, P, _ = ch.arrays()
    except (SystemExit, Exception):                    # no area or origin yet, or no room for the grid
        return PadLayoutReport(drones=n)
    ned = np.stack([P[:, :, 1], P[:, :, 0], -P[:, :, 2]], -1)
    ids = list(range(1, n + 1))
    problems, _ = check_sequence(lay, ids, ned[:, 0], ned[:, -1])
    plan = sequence_plan(lay, ids, ned[:, 0], ned[:, -1])
    ranks = pad_ranks(lay)
    rear = sorted(p for p, r in ranks.items() if r > 0)
    if problems:
        bad = sorted({lay.pad_slot(d)[0] for _, who, *_ in problems for d in who if len(who) == 2})
        closest = [float(m.split("come within ")[1].split(" m")[0]) for *_, m in problems if "come within" in m]
        how = (f"their drones would come within {min(closest):.2f} m of each other (0.5 m at least)" if closest
               else "one would fly in another's downwash") + " on the way to the grid or back"
        fix = ("Move them further apart, or line the rows up squarely one behind the other."
               if rear else "Move them further apart (0.5 m centre to centre is enough side by side).")
        return PadLayoutReport({p - 1 for p in bad},
                               f"Pads {_and(bad)} cannot fly as placed ({n} drones): {how}. {fix}", "", n)
    note = ""
    if plan.max_delay > 0 and rear:
        note = (f"Pad{'s' if len(rear) > 1 else ''} {_and(rear)} stand{'' if len(rear) > 1 else 's'} behind another pad: "
                f"in each of the 4 batches {'their' if len(rear) > 1 else 'its'} drones take off {plan.row_gap_s:g} s "
                "after the front pads' and land before them.")
    return PadLayoutReport(set(), "", note, n)

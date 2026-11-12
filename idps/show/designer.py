"""Show designer: a show as a list of scenes, with the flights between them worked out.

A scene is a formation on a vertical wall facing the audience (a grid, a circle, a heart, text,
or the shape in an image, with the drones spread evenly over it), placed in the venue, given
depth, held for a while and optionally moved while it holds (a wave, a spin). Between scenes
the designer chooses which drone flies to which place (the shortest moves, which do not cross)
and, where two drones would still come too close, lets some leave a little later.

The show starts and ends in the air in the venue's layered grid above the charging pads
(examples/shows/layers_from_pads.py), as the take-off from the pads needs: grid -> scene 1 ->
... -> last scene -> grid.

    d = Design(n=20, scenes=[Scene("grid"), Scene("text", text="HI"), Scene("circle", wave_m=0.3)])
    c = d.compile(site_dict)             # Compiled: t, P (N, T, 3), C (N, T, 3), steps, findings
    c.write_csv_zip("show.zip")          # read by Simulate, the PX4 flight tab and idps-show

Show frame: x East, y North, z up (m), origin = the venue origin.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from idps.show.choreo import Choreography, min_jerk

RATE_HZ = 20.0
KINDS = ("grid", "circle", "heart", "text", "image")
DEPTHS = ("flat", "tilt", "bulge", "brightness")
MIN_HOLD_S = 1.0
MIN_MOVE_S = 2.0
DELAY_STEP_S = 0.5
DELAYS_S = (2.0, 4.0, 6.0)           # the longest a drone may wait at the start of a move, tried in turn
FADE_S = 1.5                         # a wave or spin eases in and out over this long (less in a short hold)
DOWNWASH_RADIUS_M, DOWNWASH_DEPTH_M = 0.4, 3.0
MAX_ACCEL = 3.0                      # a move is given more time until no drone needs more than this (m/s²)
AUTO_SPACING = 1.25                  # a scene sized automatically: its closest drones this many min_sep apart


class DesignError(ValueError):
    """The design cannot be built (no pads, too few places in an image, ...): the message says why."""


# --------------------------------------------------------------------------- places on a shape
def hungarian(cost: np.ndarray) -> np.ndarray:
    """For each row the column it is assigned (smallest total cost). cost: (n, n)."""
    n = len(cost)
    u, v = np.zeros(n + 1), np.zeros(n + 1)
    p, way = np.zeros(n + 1, int), np.zeros(n + 1, int)
    for i in range(1, n + 1):
        p[0], j0 = i, 0
        minv, used = np.full(n + 1, np.inf), np.zeros(n + 1, bool)
        while True:
            used[j0] = True
            i0, free = p[j0], ~used
            free[0] = False
            cur = cost[i0 - 1] - u[i0] - v[1:]
            better = free[1:] & (cur < minv[1:])
            minv[1:][better] = cur[better]
            way[1:][better] = j0
            j1 = 1 + int(np.argmin(np.where(free[1:], minv[1:], np.inf)))
            delta = minv[j1]
            u[p[used]] += delta
            v[used] -= delta
            minv[~used] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
    out = np.zeros(n, int)
    out[p[1:] - 1] = np.arange(n)
    return out


def sample_mask(mask: np.ndarray, n: int, seed: int = 0) -> np.ndarray:
    """n places spread evenly over the True pixels of mask (H, W): (n, 2) pixel coordinates, x to
    the right and y up. Farthest-point start, then each place moves to the middle of the pixels
    nearest to it (Lloyd), and back onto the shape."""
    rows, cols = np.nonzero(mask)
    if len(rows) < n:
        raise DesignError(f"the shape has {len(rows)} pixel(s): too small for {n} drones")
    pts = np.column_stack([cols, mask.shape[0] - 1 - rows]).astype(float)
    rng = np.random.default_rng(seed)
    if len(pts) > 8000:
        pts = pts[rng.choice(len(pts), 8000, replace=False)]
    c = [pts[np.argmin(np.linalg.norm(pts - pts.mean(0), axis=1))]]
    d = np.linalg.norm(pts - c[0], axis=1)
    for _ in range(n - 1):
        c.append(pts[int(d.argmax())])
        d = np.minimum(d, np.linalg.norm(pts - c[-1], axis=1))
    c = np.array(c)
    for _ in range(40):
        near = np.linalg.norm(pts[:, None] - c[None], axis=2).argmin(1)
        new = np.array([pts[near == k].mean(0) if (near == k).any() else c[k] for k in range(n)])
        if np.abs(new - c).max() < 0.05:
            c = new
            break
        c = new
    return np.array([pts[np.linalg.norm(pts - ck, axis=1).argmin()] for ck in c])


def _font(size: int):
    from PIL import ImageFont
    for name in ("DejaVuSans-Bold.ttf", "Arial Bold.ttf", "Arial.ttf", "Helvetica.ttc"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    try:
        from matplotlib import font_manager
        return ImageFont.truetype(font_manager.findfont("DejaVu Sans:bold"), size)
    except Exception:
        return ImageFont.load_default(size)


def text_mask(text: str, size: int = 160) -> np.ndarray:
    from PIL import Image, ImageDraw
    font = _font(size)
    box = ImageDraw.Draw(Image.new("L", (8, 8))).textbbox((0, 0), text, font=font)
    img = Image.new("L", (box[2] - box[0] + 16, box[3] - box[1] + 16), 0)
    ImageDraw.Draw(img).text((8 - box[0], 8 - box[1]), text, fill=255, font=font)
    return np.asarray(img) > 127


def image_arrays(path) -> tuple[np.ndarray, np.ndarray]:
    """(mask (H, W) of the picture's shape, rgb (H, W, 3) 0..255). The shape is what is not
    transparent; in a picture without transparency, what differs from its border colour."""
    from PIL import Image
    try:
        img = Image.open(path).convert("RGBA")
    except Exception as e:
        raise DesignError(f"cannot read the image {Path(str(path)).name}: {e}") from None
    img.thumbnail((400, 400))
    a = np.asarray(img).astype(float)
    rgb, alpha = a[..., :3], a[..., 3]
    if (alpha < 128).mean() > 0.02:
        mask = alpha >= 128
    else:
        edge = np.concatenate([rgb[0], rgb[-1], rgb[:, 0], rgb[:, -1]])
        mask = np.linalg.norm(rgb - np.median(edge, axis=0), axis=2) > 60
    if not mask.any():
        raise DesignError(f"{Path(str(path)).name}: no shape found (one flat colour?)")
    return mask, rgb


def _unit(points: np.ndarray) -> np.ndarray:
    """Centred on its bounding box and scaled to width 1 (the wider of width and height)."""
    lo, hi = points.min(0), points.max(0)
    return (points - (lo + hi) / 2) / max(float((hi - lo).max()), 1e-9)


def shape_points(kind: str, n: int, text: str = "", image: str = "") -> tuple[np.ndarray, np.ndarray | None]:
    """((n, 2) places on the wall, u to the right and v up, the wider side 1 long; their colours
    from the image (n, 3), else None)."""
    if kind == "grid":
        cols = max(1, min(n, math.ceil(math.sqrt(n * 1.6))))
        rows = math.ceil(n / cols)
        pts = []
        for r in range(rows):
            m = cols if r < rows - 1 else n - cols * (rows - 1)
            pts += [((k - (m - 1) / 2), (rows - 1) / 2 - r) for k in range(m)]
        return _unit(np.array(pts, float)), None
    if kind == "circle":
        a = 2 * np.pi * np.arange(n) / n
        return _unit(np.column_stack([np.sin(a), np.cos(a)])), None
    if kind == "heart":
        a = np.linspace(0, 2 * np.pi, 2000, endpoint=False)
        xy = np.column_stack([16 * np.sin(a) ** 3, 13 * np.cos(a) - 5 * np.cos(2 * a) - 2 * np.cos(3 * a) - np.cos(4 * a)])
        s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(np.vstack([xy, xy[:1]]), axis=0), axis=1))])
        at = np.arange(n) * s[-1] / n
        return _unit(np.column_stack([np.interp(at, s[:-1], xy[:, k]) for k in (0, 1)])), None
    if kind == "text":
        if not text.strip():
            raise DesignError("type the text for this scene")
        return _unit(sample_mask(text_mask(text.strip()), n)), None
    if kind == "image":
        if not image:
            raise DesignError("choose an image for this scene")
        mask, rgb = image_arrays(image)
        px = sample_mask(mask, n)
        H = mask.shape[0]
        return _unit(px), rgb[(H - 1 - px[:, 1]).astype(int), px[:, 0].astype(int)]
    raise DesignError(f"unknown scene kind {kind!r}")


# --------------------------------------------------------------------------- the design
@dataclass
class Scene:
    kind: str = "grid"
    name: str = ""
    text: str = ""
    image: str = ""
    width_m: float = 0.0             # the wider side of the formation (0: as small as the spacing allows)
    east_m: float | None = None      # its middle (None: the middle of the performance area)
    north_m: float | None = None
    height_m: float | None = None    # ... above the floor (None: its lowest drone at the lowest flight height)
    facing_deg: float | None = None  # the way the wall faces (the audience), from North; None: the design's
    depth: str = "flat"              # flat | tilt (the top leans to the audience) | bulge | brightness (image)
    depth_m: float = 0.0
    hold_s: float = 6.0
    move_s: float = 0.0              # the flight to this scene (0: from the distance and the design's speed)
    colour: str = "single"           # single | image
    rgb: tuple = (255, 255, 255)
    wave_m: float = 0.0              # up and down by this much, travelling along the wall
    wave_length_m: float = 4.0
    wave_period_s: float = 3.0
    spin_deg: float = 0.0            # the wall turns this far about its middle while it holds

    def label(self, k: int) -> str:
        return self.name or f"{k + 1}. " + {"text": f"Text “{self.text}”", "image": f"Image {Path(self.image).name}"}.get(
            self.kind, self.kind.capitalize())


@dataclass
class Design:
    n: int = 20
    scenes: list = field(default_factory=list)
    min_sep_m: float = 0.8           # between any two drones, centre to centre
    speed_mps: float = 1.0           # the fastest a drone flies between scenes
    facing_deg: float | None = None  # the way the walls face (None: across the area's long side, away from the pads)

    def to_dict(self) -> dict:
        return {"idps_design": 1, "n": self.n, "min_sep_m": self.min_sep_m, "speed_mps": self.speed_mps,
                "facing_deg": self.facing_deg,
                "scenes": [{k: (list(v) if isinstance(v, tuple) else v) for k, v in dataclasses.asdict(s).items()}
                           for s in self.scenes]}

    @classmethod
    def from_dict(cls, d: dict) -> "Design":
        if not isinstance(d, dict) or "scenes" not in d:
            raise DesignError("not a show design file")
        names = {f.name for f in dataclasses.fields(Scene)}
        scenes = []
        for s in d.get("scenes") or []:
            s = {k: v for k, v in s.items() if k in names}
            s["rgb"] = tuple(int(x) for x in s.get("rgb", (255, 255, 255)))
            scenes.append(Scene(**s))
        return cls(int(d.get("n", 20)), scenes, float(d.get("min_sep_m", 0.8)), float(d.get("speed_mps", 1.0)),
                   d.get("facing_deg"))

    def save(self, path) -> None:
        import yaml
        Path(path).write_text(yaml.safe_dump(self.to_dict(), sort_keys=False))

    @classmethod
    def load(cls, path) -> "Design":
        import yaml
        return cls.from_dict(yaml.safe_load(Path(path).read_text()))

    def compile(self, site: dict) -> "Compiled":
        return compile_design(self, site)


@dataclass
class Compiled:
    t: np.ndarray                    # (T,) s
    P: np.ndarray                    # (N, T, 3) show frame (x East, y North, z up)
    C: np.ndarray                    # (N, T, 3) LED colours 0..255
    steps: list                      # (label, kind "scene" | "move", t0, t1, scene index or None)
    findings: list                   # (severity "error" | "warning", text, time s)
    stats: dict

    @property
    def ok(self) -> bool:
        return not any(sev == "error" for sev, *_ in self.findings)

    def scene_span(self, k: int) -> tuple[float, float] | None:
        for _, kind, t0, t1, idx in self.steps:
            if kind == "scene" and idx == k:
                return t0, t1
        return None

    def choreography(self) -> Choreography:
        ch = Choreography(self.P[:, 0])
        ch.t = [float(x) for x in self.t]
        ch.P = [self.P[:, k] for k in range(len(self.t))]
        ch.C = [self.C[:, k] for k in range(len(self.t))]
        ch.steps = [(label, t0, t1) for label, _, t0, t1, _ in self.steps]
        return ch

    def write_csv_zip(self, path):
        return self.choreography().write_csv_zip(path)


# --------------------------------------------------------------------------- building
def _layout(site: dict, n: int):
    import importlib.util
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location("idps_layers_from_pads", root / "examples" / "shows" / "layers_from_pads.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if not (site.get("volume") or {}).get("polygon") and "min" not in (site.get("volume") or {}):
        raise DesignError("draw the performance area in the Planner first")
    if not site.get("origin"):
        raise DesignError("place the origin in the Planner first")
    try:
        return mod.Layout(site, n)
    except SystemExit as e:
        raise DesignError(str(e)) from None


def _auto_facing(L) -> float:
    """Across the area's long side, turned away from the charging pads (degrees from North)."""
    ne = L.area_ne - L.area_ne.mean(0)
    _, _, vt = np.linalg.svd(ne, full_matrices=False)
    across = vt[1]                                               # (N, E): the short way
    if across @ (L.lay.pads_ne.mean(0) - L.area_ne.mean(0)) > 0:
        across = -across
    return float(np.degrees(np.arctan2(across[1], across[0])) % 360)


def scene_places(design: Design, scene: Scene, L, cache: dict | None = None):
    """(places (n, 3) in the show frame, colours (n, 3), the wall's middle (3,), notes [text])."""
    n = design.n
    key = (scene.kind, n, scene.text, scene.image, Path(scene.image).stat().st_mtime if scene.kind == "image"
           and scene.image and Path(scene.image).exists() else 0)
    if cache is not None and key in cache:
        uv, img_rgb = cache[key]
    else:
        uv, img_rgb = shape_points(scene.kind, n, scene.text, scene.image)
        if cache is not None:
            cache[key] = (uv, img_rgb)
    notes = []
    gaps = np.linalg.norm(uv[:, None] - uv[None], axis=2)
    closest = float(gaps[np.triu_indices(n, 1)].min()) if n > 1 else 1.0
    need = design.min_sep_m * 1.02 / max(closest, 1e-6)          # the width at which the closest two are min_sep apart
    width = scene.width_m if scene.width_m > 0 else need * AUTO_SPACING / 1.02
    if width < need - 1e-6:
        notes.append(f"its drones would be {closest * width:.2f} m apart: make it {need:.1f} m wide or more")
    facing = scene.facing_deg if scene.facing_deg is not None else (
        design.facing_deg if design.facing_deg is not None else _auto_facing(L))
    a = np.radians(facing)
    f = np.array([np.sin(a), np.cos(a), 0.0])                    # to the audience (x East, y North)
    r = np.array([-np.cos(a), np.sin(a), 0.0])                   # the audience's right
    up = np.array([0.0, 0.0, 1.0])
    span_v = float(np.ptp(uv[:, 1])) * width
    lowest = float(L.site_z[0]) + 0.5
    mid_h = scene.height_m if scene.height_m is not None else max(lowest + span_v / 2, min(3.0, (L.site_z[0] + L.site_z[1]) / 2))
    east = scene.east_m if scene.east_m is not None else float(L.area_ne.mean(0)[1])
    north = scene.north_m if scene.north_m is not None else float(L.area_ne.mean(0)[0])
    centre = np.array([east, north, mid_h])
    depth = np.zeros(n)
    if scene.depth_m and scene.depth != "flat":
        if scene.depth == "tilt":
            depth = scene.depth_m * uv[:, 1] / max(float(np.ptp(uv[:, 1])), 1e-9)
        elif scene.depth == "bulge":
            rad = np.linalg.norm(uv, axis=1)
            depth = scene.depth_m * (1 - (rad / max(float(rad.max()), 1e-9)) ** 2)
        elif scene.depth == "brightness" and img_rgb is not None:
            depth = scene.depth_m * (img_rgb.mean(1) / 255.0 - 0.5)
    P = centre + np.outer(uv[:, 0] * width, r) + np.outer(uv[:, 1] * width, up) + np.outer(depth, f)
    C = (img_rgb if scene.colour == "image" and img_rgb is not None else np.tile(scene.rgb, (n, 1))).astype(float)
    return P, C, centre, notes


def _hold(scene: Scene, P0: np.ndarray, centre: np.ndarray, facing_r: np.ndarray) -> np.ndarray:
    """(N, T, 3) through the scene's hold, from P0 (its places): the wave and the spin, eased in and out."""
    hold = max(scene.hold_s, MIN_HOLD_S)
    n = int(round(hold * RATE_HZ))
    tt = np.arange(1, n + 1) / RATE_HZ
    fade = min(FADE_S, hold / 3)
    env = min_jerk(tt / fade) * min_jerk((hold - tt) / fade)
    out = np.repeat(P0[:, None, :], n, axis=1).astype(float)
    if scene.wave_m:
        along = (P0 - centre) @ facing_r                                   # metres along the wall
        phase = 2 * np.pi * (along[:, None] / max(scene.wave_length_m, 0.1) - tt[None, :] / max(scene.wave_period_s, 0.2))
        out[:, :, 2] += scene.wave_m * env[None, :] * np.sin(phase)
    if scene.spin_deg:
        ang = np.radians(scene.spin_deg) * min_jerk(tt / hold)
        rel = out - centre
        c, s = np.cos(ang)[None, :], np.sin(ang)[None, :]
        x, y = rel[:, :, 0].copy(), rel[:, :, 1].copy()
        out[:, :, 0] = centre[0] + c * x + s * y                           # clockwise seen from above
        out[:, :, 1] = centre[1] - s * x + c * y
    return out


def _move(start: np.ndarray, target: np.ndarray, seconds: float, delays: np.ndarray, bows=None) -> np.ndarray:
    """(N, T, 3): each drone from its start to its target in `seconds` (minimum jerk), leaving
    `delays` late; bows (N, 3): its path bows out by this much at half way (zero at both ends)."""
    total = seconds + float(delays.max())
    n = int(round(total * RATE_HZ))
    tt = np.arange(1, n + 1) / RATE_HZ
    s = min_jerk((tt[None, :] - delays[:, None]) / seconds)
    M = start[:, None, :] + (target - start)[:, None, :] * s[:, :, None]
    if bows is not None:
        M = M + bows[:, None, :] * (np.sin(np.pi * s) ** 2)[:, :, None]
    return M


def _closest(M: np.ndarray, only: int | None = None, among=None):
    """(smallest distance, i, j, sample) between two drones over M (N, T, 3)."""
    N = len(M)
    best = (np.inf, 0, 0, 0)
    rows = [only] if only is not None else range(N)
    for i in rows:
        others = [j for j in (among if among is not None else range(N)) if j != i and (only is not None or j > i)]
        if not others:
            continue
        d = np.linalg.norm(M[others] - M[i], axis=2)
        k = np.unravel_index(int(d.argmin()), d.shape)
        if d[k] < best[0]:
            best = (float(d[k]), i, others[k[0]], int(k[1]))
    return best


def _path(a: np.ndarray, b: np.ndarray, seconds: float, delay: float, bow: np.ndarray, span: int) -> np.ndarray:
    """One drone's move, (span, 3): from a to b in `seconds`, leaving `delay` late, bowing out by `bow`."""
    tt = np.arange(1, span + 1) / RATE_HZ
    s = min_jerk((tt - delay) / seconds)
    return a + (b - a) * s[:, None] + bow * (np.sin(np.pi * s) ** 2)[:, None]


def plan_move(start: np.ndarray, target: np.ndarray, seconds: float, min_sep: float, assign: bool = True,
              axis=(0.0, 1.0, 0.0), z_range=None, waits=DELAYS_S):
    """(order: the target place of each drone, delays (N,) s, bows (N, 3) m, smallest distance
    during the move). assign: the drones take the places that make the squared distances flown
    smallest (such moves do not cross). Drones that would still come within min_sep are kept
    apart two ways, whichever costs least: the path bows out sideways (along `axis`, the way the
    wall faces, and up or down) by a step of min_sep or more, or the drone leaves a little later.
    Each drone in turn, the longest flights first, takes the cheapest way that keeps it clear of
    those planned before it; then the two that are still closest are planned again, against
    every other drone, until none is too close or nothing more helps."""
    N = len(start)
    order = hungarian(((start[:, None] - target[None]) ** 2).sum(2)) if assign else np.arange(N)
    tgt = target[order]
    delays, bows = np.zeros(N), np.zeros((N, 3))
    worst = _closest(_move(start, tgt, seconds, delays))[0]
    if worst >= min_sep - 1e-3:
        return order, delays, bows, worst
    best = (worst, delays, bows)
    ax = np.asarray(axis, float)
    ax = ax / max(float(np.linalg.norm(ax)), 1e-9)
    up = np.array([0.0, 0.0, 1.0])
    lanes = [a * min_sep * ax + b * min_sep * up for a in (0, 1, -1, 2, -2, 3, -3) for b in (0, 1, -1)]
    mid = (start + tgt) / 2
    far = np.argsort(-np.linalg.norm(tgt - start, axis=1))                 # the longest flights first
    need = min_sep + 0.02
    for most in waits:
        steps = np.arange(0.0, most + 1e-9, DELAY_STEP_S)
        ways = sorted(((w + 1.2 * np.linalg.norm(b) / max(min_sep, 1e-9), w, k) for w in steps for k, b in enumerate(lanes)),
                      key=lambda x: x[0])
        span = int(round((seconds + most) * RATE_HZ))
        d, bw = np.zeros(N), np.zeros((N, 3))
        paths = np.stack([_path(start[i], tgt[i], seconds, 0.0, bw[i], span) for i in range(N)])

        def pick(i, others):
            """The cheapest way for drone i that keeps it `need` from `others`; else the one that keeps it furthest."""
            got = (-1.0, 0.0, 0)
            if not others:
                return np.inf, 0.0, 0
            for _, w, k in ways:
                b = lanes[k]
                if z_range is not None and b[2] and not (z_range[0] <= mid[i, 2] + b[2] <= z_range[1]):
                    continue                                               # (not through the floor or the ceiling)
                p = _path(start[i], tgt[i], seconds, w, b, span)
                c = float(np.linalg.norm(paths[others] - p, axis=2).min())
                if c > got[0]:
                    got = (c, w, k)
                if c >= need:
                    break
            return got

        def take(i, others):
            c, w, k = pick(i, others)
            d[i], bw[i] = w, lanes[k]
            paths[i] = _path(start[i], tgt[i], seconds, w, lanes[k], span)
            return c

        done: list = []
        for i in far:
            take(int(i), done)
            done.append(int(i))
        seen = set()
        for _ in range(3 * N):                                             # the closest two again, against everyone
            worst, i, j, _k = _closest(paths)
            if worst >= min_sep - 1e-3 or (i, j, round(worst, 3)) in seen:
                break
            seen.add((i, j, round(worst, 3)))
            for x in (j, i):
                take(x, [y for y in range(N) if y != x])
        worst = _closest(paths)[0]
        if worst > best[0]:
            best = (worst, d.copy(), bw.copy())
        if worst >= min_sep - 1e-3:
            break
    return order, best[1], best[2], best[0]


def _inside(poly_ne: np.ndarray, ne: np.ndarray) -> np.ndarray:
    """Points (M, 2) inside the polygon (V, 2)?"""
    x, y = ne[:, 0], ne[:, 1]
    inside = np.zeros(len(ne), bool)
    a, b = poly_ne, np.roll(poly_ne, -1, 0)
    for (x1, y1), (x2, y2) in zip(a, b):
        cross = ((y1 > y) != (y2 > y)) & (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1)
        inside ^= cross
    return inside


def compile_design(design: Design, site: dict, cache: dict | None = None) -> Compiled:
    if design.n < 1:
        raise DesignError("a show needs at least one drone")
    L = _layout(site, design.n)
    vol = site["volume"]
    L.site_z = tuple(float(v) for v in (vol["z"] if "z" in vol else (vol["min"][2], vol["max"][2])))
    grid = L.grid()
    findings, steps = [], []
    white = np.tile((255.0, 255.0, 255.0), (design.n, 1))
    P_parts, C_parts = [grid[:, None, :]], [white[:, None, :]]
    t_now = 0.0
    where, colour = grid.copy(), white

    def add(M, C0, C1, label, kind, idx):
        nonlocal t_now, where
        n = M.shape[1]
        if C1 is None:
            Cs = np.repeat(C0[:, None, :], n, axis=1)
        else:
            w = (np.arange(1, n + 1) / n)[None, :, None]
            Cs = C0[:, None, :] * (1 - w) + C1[:, None, :] * w
        P_parts.append(M)
        C_parts.append(Cs)
        steps.append((label, kind, t_now, t_now + n / RATE_HZ, idx))
        t_now += n / RATE_HZ
        where = M[:, -1].copy()

    def fly(target, C1, label, idx, assign, seconds=0.0, then=None, leave=None):
        # then: where each drone must go after this (the last scene: back to its own grid place)
        nonlocal colour
        a = np.radians(design.facing_deg if design.facing_deg is not None else _auto_facing(L))
        face = (np.sin(a), np.cos(a), 0.0)
        cost = ((where[:, None] - target[None]) ** 2).sum(2)
        if then is not None:                  # ... so it takes a place that suits both flights
            cost = cost + ((then[:, None] - (target if leave is None else leave)[None]) ** 2).sum(2)
        first = hungarian(cost) if assign else np.arange(design.n)
        far = float(np.linalg.norm(target[first] - where, axis=1).max())
        T = seconds if seconds > 0 else max(MIN_MOVE_S, 1.875 * far / max(design.speed_mps, 0.1))
        zr = (L.site_z[0] + 0.3, L.site_z[1] - 0.1)
        for _ in range(6):                    # a path that bows out is longer: more time until no drone is rushed
            _, delays, bows, worst = plan_move(where, target[first], T, design.min_sep_m, assign=False, axis=face,
                                               z_range=zr, waits=DELAYS_S + (10.0, 16.0))
            M = _move(where, target[first], T, delays, bows)
            V = np.gradient(M, 1 / RATE_HZ, axis=1)
            fast = float(np.linalg.norm(V, axis=2).max())
            hard = float(np.linalg.norm(np.gradient(V, 1 / RATE_HZ, axis=1), axis=2).max())
            if seconds > 0 or (fast <= 1.9 * design.speed_mps + 0.2 and hard <= MAX_ACCEL):
                break
            T *= 1.3
        if worst < design.min_sep_m - 1e-3:
            findings.append(("error", f"{label}: two drones come within {worst:.2f} m (minimum {design.min_sep_m:g} m). "
                             "Give this move more time, or move or widen the scene.", t_now))
        C1 = C1[first] if C1 is not None else colour
        add(M, colour, C1, label, "move", idx)
        colour = C1

    add(np.repeat(grid[:, None, :], int(RATE_HZ), axis=1), white, None, "start grid", "scene", None)
    for k, sc in enumerate(design.scenes):
        name = sc.label(k)
        try:
            places, cols, centre, notes = scene_places(design, sc, L, cache)
        except DesignError as e:
            findings.append(("error", f"{name}: {e}", t_now))
            continue
        for note in notes:
            findings.append(("error", f"{name}: {note}", t_now))
        a = np.radians(sc.facing_deg if sc.facing_deg is not None else
                       (design.facing_deg if design.facing_deg is not None else _auto_facing(L)))
        right = np.array([-np.cos(a), np.sin(a), 0.0])
        last = k == len(design.scenes) - 1
        # (the last scene: each drone takes a place that also suits its flight back to its own grid
        # place, from where that place is when the scene ends: a spin moves it)
        fly(places, cols, f"to {name}", k, True, sc.move_s, then=grid if last else None,
            leave=_hold(sc, places, centre, right)[:, -1] if last else None)
        add(_hold(sc, where, centre, right), colour, None, name, "scene", k)
    fly(grid, white, "back to the grid", None, False)
    add(np.repeat(grid[:, None, :], int(RATE_HZ), axis=1), white, None, "end grid", "scene", None)

    P = np.concatenate(P_parts, axis=1)
    C = np.clip(np.concatenate(C_parts, axis=1), 0, 255)
    t = np.arange(P.shape[1]) / RATE_HZ

    # the whole show: distance between drones, the area, speed
    d, i, j, kk = _closest(P)
    if d < design.min_sep_m - 1e-3 and not any("come within" in f[1] for f in findings):
        findings.append(("error", f"drones {i + 1} and {j + 1} come within {d:.2f} m at {t[kk]:.1f} s "
                         f"(minimum {design.min_sep_m:g} m): a wave or spin too strong for this spacing?", float(t[kk])))
    ne = np.stack([P[:, ::4, 1], P[:, ::4, 0]], -1).reshape(-1, 2)
    out = ~_inside(L.area_ne, ne)
    if out.any():
        when = t[::4][np.nonzero(out.reshape(design.n, -1).any(0))[0][0]]
        findings.append(("error", f"drones leave the performance area (first at {when:.1f} s): move or narrow that scene.",
                         float(when)))
    z = P[:, :, 2]
    if z.min() < L.site_z[0] - 1e-6 or z.max() > L.site_z[1] + 1e-6:
        k_bad = int(np.nonzero((z < L.site_z[0] - 1e-6).any(0) | (z > L.site_z[1] + 1e-6).any(0))[0][0])
        findings.append(("error", f"drones fly below {L.site_z[0]:g} m or above {L.site_z[1]:g} m (first at {t[k_bad]:.1f} s): "
                         "change that scene's height.", float(t[k_bad])))
    V = np.gradient(P, 1 / RATE_HZ, axis=1)
    A = np.gradient(V, 1 / RATE_HZ, axis=1)
    speed, acc = float(np.linalg.norm(V, axis=2).max()), float(np.linalg.norm(A, axis=2).max())
    if speed > 3.0:
        kf = int(np.linalg.norm(V, axis=2).max(0).argmax())
        at = next((lab for lab, _, t0, t1, _ in steps if t0 <= t[kf] <= t1), "")
        findings.append(("error", f"{at}: a drone flies {speed:.1f} m/s (limit 3). Give a move more time; make a spin "
                         "smaller or its scene longer.", float(t[kf])))
    if acc > 4.0:
        ka = int(np.linalg.norm(A, axis=2).max(0).argmax())
        at = next((lab for lab, _, t0, t1, _ in steps if t0 <= t[ka] <= t1), "")
        findings.append(("warning", f"{at}: a drone accelerates at {acc:.1f} m/s² (IDPS allows 4). Slow that move, wave "
                         "or spin.", float(t[ka])))
    from idps.show.pads import downwash_pairs
    ned = np.stack([P[:, ::4, 1], P[:, ::4, 0], -P[:, ::4, 2]], -1)
    stacked = downwash_pairs(ned, np.ones(ned.shape[:2], bool), DOWNWASH_RADIUS_M, DOWNWASH_DEPTH_M)
    if stacked:
        findings.append(("warning", f"{len(stacked)} pair(s) of drones fly one above the other (within "
                         f"{DOWNWASH_RADIUS_M:g} m sideways): the lower one is in the upper one's downwash. A wall "
                         "stacks drones; tilt it (depth: tilt) to offset them.", float(t[::4][stacked[0][2]])))
    stats = {"duration_s": float(t[-1]), "max_speed_mps": speed, "max_accel_mps2": acc, "min_separation_m": float(d),
             "scenes": len(design.scenes)}
    return Compiled(t, P, C, steps, findings, stats)

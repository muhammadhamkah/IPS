"""Lens calibration from checkerboard pictures (``idps-calib lens``).

Each camera's lens is measured once: focal length in pixels, image centre and distortion
(Brown-Conrady k1, k2, p1, p2, k3, the model idps.engine.calib uses). Pictures come from
``idps-station snapshot`` (PGM), taken without the IR filter (or with the board lit by IR light)
and a longer exposure, the board held at many angles and reaching into the image corners.

Needs OpenCV (pip install 'idps[calib]')."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml


@dataclass
class Lens:
    f_px: float
    cx: float
    cy: float
    distortion: list            # k1, k2, p1, p2, k3
    width: int = 1280
    height: int = 800
    station_id: int | None = None
    rms_px: float | None = None                 # checkerboard corner residual
    images_used: int = 0
    coverage: float | None = None               # share of a 4 x 4 image grid that had corners in it
    notes: list = field(default_factory=list)

    def to_dict(self) -> dict:
        d = {"format": "idps-lens/1", "station_id": self.station_id, "width_px": self.width, "height_px": self.height,
             "f_px": round(float(self.f_px), 4), "cx": round(float(self.cx), 4), "cy": round(float(self.cy), 4),
             "distortion": [round(float(v), 8) for v in self.distortion]}
        if self.rms_px is not None:
            d["quality"] = {"rms_px": round(float(self.rms_px), 4), "images_used": self.images_used,
                            "coverage": None if self.coverage is None else round(float(self.coverage), 3),
                            "notes": list(self.notes)}
        return d

    def save(self, path) -> Path:
        path = Path(path)
        path.write_text(yaml.safe_dump(self.to_dict(), sort_keys=False, default_flow_style=None, width=120))
        return path


def load_lens(path) -> Lens:
    d = yaml.safe_load(Path(path).read_text())
    if d.get("format") != "idps-lens/1":
        raise ValueError(f"{path}: not an idps-lens/1 file")
    q = d.get("quality") or {}
    return Lens(float(d["f_px"]), float(d["cx"]), float(d["cy"]), [float(v) for v in d.get("distortion", [0] * 5)],
                int(d.get("width_px", 1280)), int(d.get("height_px", 800)), d.get("station_id"), q.get("rms_px"),
                int(q.get("images_used", 0)), q.get("coverage"), list(q.get("notes", [])))


def nominal_lens(hfov_deg: float, width: int = 1280, height: int = 800) -> Lens:
    """A lens known only by its horizontal field of view: no distortion, centre in the middle.
    Good enough to start the room calibration with --refine-lens."""
    f = (width / 2) / np.tan(np.radians(hfov_deg) / 2)
    return Lens(float(f), (width - 1) / 2, (height - 1) / 2, [0.0] * 5, width, height,
                notes=[f"nominal: {hfov_deg:g} deg horizontal field of view, not measured"])


def find_corners(img: np.ndarray, cols: int, rows: int):
    """Inner corners (cols-1 x rows-1) of a cols x rows-square board in a grey image, sub-pixel,
    or None."""
    import cv2
    if img.dtype != np.uint8:
        img = (img.astype(np.float64) * (255.0 / max(float(img.max()), 1.0))).astype(np.uint8)
    size = (cols - 1, rows - 1)
    ok, c = cv2.findChessboardCornersSB(img, size, flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY)
    if not ok:
        return None
    return c.reshape(-1, 2).astype(np.float64)


def calibrate_lens(images: list, cols: int = 10, rows: int = 7, square_mm: float = 25.0,
                   station_id: int | None = None, log=print) -> Lens:
    """images: paths (PGM/PNG/JPG) or grey arrays. Returns the lens with its quality figures."""
    import cv2
    obj = np.zeros(((cols - 1) * (rows - 1), 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols - 1, 0:rows - 1].T.reshape(-1, 2) * (square_mm / 1000.0)
    pts_obj, pts_img, names, size = [], [], [], None
    for k, im in enumerate(images):
        name = str(im) if not isinstance(im, np.ndarray) else f"image {k + 1}"
        img = cv2.imread(str(im), cv2.IMREAD_GRAYSCALE) if not isinstance(im, np.ndarray) else im
        if img is None:
            log(f"  {name}: cannot read")
            continue
        if size is None:
            size = (img.shape[1], img.shape[0])
        elif size != (img.shape[1], img.shape[0]):
            log(f"  {name}: different size, skipped")
            continue
        c = find_corners(img, cols, rows)
        if c is None:
            log(f"  {name}: board not found")
            continue
        pts_obj.append(obj)
        pts_img.append(c.astype(np.float32))
        names.append(name)
    if len(pts_img) < 6:
        raise ValueError(f"the board was found in {len(pts_img)} pictures; at least 6 are needed (15-25 is good)")
    flags = cv2.CALIB_FIX_ASPECT_RATIO                    # one focal length, as the engine's model has
    K0 = np.array([[size[0], 0, (size[0] - 1) / 2], [0, size[0], (size[1] - 1) / 2], [0, 0, 1]], float)
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(pts_obj, pts_img, size, K0, None,
                                                    flags=flags | cv2.CALIB_USE_INTRINSIC_GUESS)
    # per-picture residuals: drop pictures far worse than the rest (a blurred or mis-detected board), once
    per = []
    for o, i, r, t in zip(pts_obj, pts_img, rvecs, tvecs):
        p, _ = cv2.projectPoints(o, r, t, K, dist)
        per.append(float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - i) ** 2, axis=1)))))
    per = np.array(per)
    keep = per < max(3 * np.median(per), 0.3)
    notes = []
    if not keep.all() and keep.sum() >= 6:
        for n, e in zip(np.array(names)[~keep], per[~keep]):
            log(f"  {n}: left out ({e:.2f} px, the others are about {np.median(per):.2f} px)")
        pts_obj = [o for o, k in zip(pts_obj, keep) if k]
        pts_img = [i for i, k in zip(pts_img, keep) if k]
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(pts_obj, pts_img, size, K0, None,
                                                        flags=flags | cv2.CALIB_USE_INTRINSIC_GUESS)
        notes.append(f"{int((~keep).sum())} pictures left out as outliers")
    allp = np.concatenate(pts_img)
    grid = np.zeros((4, 4), bool)
    grid[np.clip((allp[:, 1] * 4 / size[1]).astype(int), 0, 3), np.clip((allp[:, 0] * 4 / size[0]).astype(int), 0, 3)] = True
    cov = float(grid.mean())
    if cov < 0.9:
        notes.append("the board never reached some parts of the image: take more pictures with it in the corners")
    if rms > 0.2:
        notes.append("residual above 0.2 px: check focus is locked, the board is flat, and the pictures are sharp")
    d = dist.ravel()[:5]
    return Lens(float(K[0, 0]), float(K[0, 2]), float(K[1, 2]), [float(v) for v in d], size[0], size[1], station_id,
                float(rms), len(pts_img), cov, notes)

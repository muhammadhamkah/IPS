"""Run the simulator through the real station code: every simulated camera renders its frame as
an image and finds the dots with the station's C++ detector (edge/src/detect.hpp), instead of
handing the engine ideal noisy dots.

    sim = Simulation(scenario)
    film_with_station_detector(sim)       # then sim.run(...) as usual

What changes against the plain simulator: the dots' centres, sums, peaks, areas and flags come
from images (blur, sensor noise, clipping, neighbouring spots running into each other) measured
by the detector the stations run, so the engine sees what a real station would send.
"""
from __future__ import annotations

import numpy as np

from idps.protocol import packets as P
from idps.station.render import SensorNoise, Spot, render_sparse

BLACK_DN = 16.0           # 10-bit black level
READ_NOISE_DN = 2.0


def film_with_station_detector(sim, threshold_above_black: int | None = None, low_above_black: int = 8,
                               noise: SensorNoise | None = None):
    """Wrap every station of `sim` (idps.sim.runner.Simulation) to render and detect. Needs the
    compiled station module (idps.station._station). Returns the list of detectors."""
    from idps.station import _station
    m = sim.sc.camera_model
    thr = int(BLACK_DN) + (m.threshold_dn if threshold_above_black is None else threshold_above_black)
    noise = noise or SensorNoise(bits=10, black=BLACK_DN, read_noise=READ_NOISE_DN, dn_per_electron=1.0)
    params = {"threshold": thr, "threshold_low": int(BLACK_DN) + low_above_black,
              "saturation": m.saturation_dn}
    dets = []
    for st in sim.stations:
        det = _station.Detector(params)
        dets.append(det)
        st.observe = _filmed(st, st.observe, det, noise, m.psf_sigma_px)
        st._merge = lambda blobs: blobs          # spots running into each other happens in the image now
    return dets


def _filmed(st, observe, det, noise, sigma):
    rng = np.random.default_rng([st.id, 777])

    def run(t, marker_pos, body_up, levels, drone_centres, marker_drone):
        markers, truth = observe(t, marker_pos, body_up, levels, drone_centres, marker_drone)
        spots, ids = [], []
        for mk, (idx, u, v) in zip(markers, truth):
            # a marker at its true image position (the image adds the noise); a spurious blob
            # (reflection, false detection) where the simulator put it
            spots.append(Spot(u if idx >= 0 else mk.x, v if idx >= 0 else mk.y, max(float(mk.sum), 1.0), sigma))
            ids.append((idx, u, v))
        img = render_sparse(st.W, st.H, spots, noise, rng)
        out, tr = [], []
        for x, y, s, peak, area, flags in det.detect(img):
            out.append(P.Marker(float(x), float(y), int(s), int(peak), int(area), int(flags)))
            # which true spot this dot is (for the simulator's statistics only)
            best = min(ids, key=lambda k: (k[1] - x) ** 2 + (k[2] - y) ** 2, default=(-1, x, y))
            tr.append(best if (best[1] - x) ** 2 + (best[2] - y) ** 2 < 4.0 else (-1, x, y))
        return out, tr

    return run

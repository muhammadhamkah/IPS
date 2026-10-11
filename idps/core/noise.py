"""Measurement error budget used to predict positioning accuracy.

Per-camera angular variance (rad^2) of an observation at range r:

    (sigma_px / f_px)^2 + sigma_rot^2 + (sigma_pos / r)^2 + (speed * sigma_t / r)^2

See docs/RISKS.md section 0 for where the presets come from.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

# For an isotropic 3D Gaussian the 95 % error radius is ~1.62 x the 3D RMS
# (sqrt of the covariance trace). Used to turn predicted RMS into p95.
P95_PER_RMS = 1.62


@dataclass(frozen=True)
class NoiseBudget:
    sigma_px: float = 0.05        # centroid noise, pixels
    sigma_rot_rad: float = 5e-5   # camera orientation, rad
    sigma_pos_m: float = 2e-3     # camera position, m
    sigma_t_s: float = 5e-5       # timestamp, s
    speed_mps: float = 4.0        # marker speed used for the timing term

    def angular_var(self, f_px, range_m):
        r = np.maximum(range_m, 1e-3)
        return ((self.sigma_px / f_px) ** 2 + self.sigma_rot_rad ** 2
                + (self.sigma_pos_m / r) ** 2 + (self.speed_mps * self.sigma_t_s / r) ** 2)

    def to_dict(self) -> dict:
        return asdict(self)


PRESETS = {
    "realistic": NoiseBudget(0.10, 1e-4, 3e-3, 1e-4),
    "tight": NoiseBudget(0.05, 5e-5, 2e-3, 5e-5),
    "very_tight": NoiseBudget(0.05, 3e-5, 1e-3, 2e-5),
}


def noise_from_config(cfg) -> NoiseBudget:
    """Accepts a preset name or a mapping of NoiseBudget fields."""
    if cfg is None:
        return PRESETS["tight"]
    if isinstance(cfg, str):
        try:
            return PRESETS[cfg]
        except KeyError:
            raise ValueError(f"unknown noise preset {cfg!r}; choose from {sorted(PRESETS)}") from None
    base = PRESETS[cfg.get("preset", "tight")].to_dict()
    base.update({k: v for k, v in cfg.items() if k != "preset"})
    return NoiseBudget(**base)

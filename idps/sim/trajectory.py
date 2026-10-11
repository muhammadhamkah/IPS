"""Drone trajectories with fixed yaw and tilt derived from acceleration.

A multirotor tilts its thrust axis along (a + g z). With yaw held fixed
(decision D7) the body attitude follows from that axis, so a marker mounted
away from the drone's reference point moves when the drone accelerates -
exactly the lever-arm effect the engine must handle (RISKS R16).
"""
from __future__ import annotations

import math

import numpy as np

G = 9.81


class Trajectory:
    def position(self, t) -> np.ndarray:            # (..., 3)
        raise NotImplementedError

    def acceleration(self, t, h: float = 1e-3) -> np.ndarray:
        t = np.asarray(t, float)
        return (self.position(t + h) - 2 * self.position(t) + self.position(t - h)) / (h * h)

    def velocity(self, t, h: float = 1e-4) -> np.ndarray:
        t = np.asarray(t, float)
        return (self.position(t + h) - self.position(t - h)) / (2 * h)


class Hover(Trajectory):
    def __init__(self, position):
        self.p = np.asarray(position, float)

    def position(self, t):
        return np.broadcast_to(self.p, np.shape(t) + (3,)).copy()


class Line(Trajectory):
    """Back and forth between a and b; smooth (cosine) speed profile with peak `speed`."""

    def __init__(self, a, b, speed):
        self.a, self.b = np.asarray(a, float), np.asarray(b, float)
        L = float(np.linalg.norm(self.b - self.a))
        self.T = L * math.pi / (2 * speed) if speed > 0 and L > 0 else math.inf

    def position(self, t):
        t = np.asarray(t, float)
        s = 0.5 * (1 - np.cos(np.pi * t / self.T)) if math.isfinite(self.T) else np.zeros_like(t)
        return self.a + s[..., None] * (self.b - self.a)


class Circle(Trajectory):
    def __init__(self, center, radius, speed, phase_deg=0.0, clockwise=False, z_amplitude=0.0, z_period_s=10.0):
        self.c = np.asarray(center, float)
        self.r = float(radius)
        self.w = (speed / radius) * (-1 if clockwise else 1) if radius > 0 else 0.0
        self.ph = math.radians(phase_deg)
        self.za, self.zp = float(z_amplitude), float(z_period_s)

    def position(self, t):
        t = np.asarray(t, float)
        a = self.ph + self.w * t
        z = self.c[2] + self.za * np.sin(2 * np.pi * t / self.zp)
        return np.stack([self.c[0] + self.r * np.cos(a), self.c[1] + self.r * np.sin(a), z], -1)


class Figure8(Trajectory):
    """Lemniscate of Gerono: x = A sin(wt), y = (B/2) sin(2wt); `speed` is the approximate mean speed."""

    def __init__(self, center, size_x, size_y, speed):
        self.c = np.asarray(center, float)
        self.A, self.B = size_x / 2, size_y / 2
        perim = 4 * math.hypot(self.A, self.B) * 1.1
        self.w = 2 * math.pi * speed / perim

    def position(self, t):
        t = np.asarray(t, float)
        x = self.A * np.sin(self.w * t)
        y = self.B * np.sin(2 * self.w * t)
        return np.stack([self.c[0] + x, self.c[1] + y, np.full_like(t, self.c[2])], -1)


class Waypoints(Trajectory):
    """Loops through waypoints; each leg uses a cosine speed profile (stops at
    every waypoint) with peak `speed`."""

    def __init__(self, points, speed, loop=True):
        self.P = np.asarray(points, float)
        if not loop:
            self.P = np.concatenate([self.P, self.P[-2:0:-1]]) if len(self.P) > 2 else np.concatenate(
                [self.P, self.P[::-1]])
        self.L = np.linalg.norm(np.roll(self.P, -1, 0) - self.P, axis=1)
        self.T = np.maximum(self.L * math.pi / (2 * speed), 1e-6)
        self.cum = np.concatenate([[0], np.cumsum(self.T)])

    def position(self, t):
        t = np.asarray(t, float)
        tt = np.mod(t, self.cum[-1])
        k = np.clip(np.searchsorted(self.cum, tt, side="right") - 1, 0, len(self.T) - 1)
        s = 0.5 * (1 - np.cos(np.pi * (tt - self.cum[k]) / self.T[k]))
        a = self.P[k]
        b = self.P[(k + 1) % len(self.P)]
        return a + s[..., None] * (b - a)


def random_waypoints(rng, lo, hi, n, speed) -> Waypoints:
    pts = rng.uniform(lo, hi, size=(n, 3))
    return Waypoints(pts, speed, loop=True)


def body_axes(acc: np.ndarray, yaw_deg: float):
    """Body x, y, z axes (each (..., 3)) for thrust along acc + g z at fixed yaw."""
    thrust = acc + np.array([0.0, 0.0, G])
    z = thrust / np.linalg.norm(thrust, axis=-1, keepdims=True)
    yaw = math.radians(yaw_deg)
    xh = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    x = xh - (z @ xh)[..., None] * z
    x /= np.linalg.norm(x, axis=-1, keepdims=True)
    y = np.cross(z, x)
    return x, y, z


def tilt_deg(acc: np.ndarray) -> np.ndarray:
    thrust = acc + np.array([0.0, 0.0, G])
    return np.degrees(np.arccos(np.clip(thrust[..., 2] / np.linalg.norm(thrust, axis=-1), -1, 1)))


def from_config(cfg: dict, rng=None, bounds=None) -> Trajectory:
    kind = cfg.get("type", "hover")
    if kind == "hover":
        return Hover(cfg["position"])
    if kind == "line":
        return Line(cfg["start"], cfg["end"], float(cfg.get("speed", 1.0)))
    if kind == "circle":
        return Circle(cfg["center"], float(cfg["radius"]), float(cfg.get("speed", 1.0)),
                      float(cfg.get("phase_deg", 0.0)), bool(cfg.get("clockwise", False)),
                      float(cfg.get("z_amplitude", 0.0)), float(cfg.get("z_period_s", 10.0)))
    if kind == "figure8":
        return Figure8(cfg["center"], float(cfg["size"][0]), float(cfg["size"][1]), float(cfg.get("speed", 1.0)))
    if kind == "waypoints":
        return Waypoints(cfg["points"], float(cfg.get("speed", 1.0)), bool(cfg.get("loop", True)))
    if kind == "random":
        if rng is None or bounds is None:
            raise ValueError("random trajectory needs rng and bounds")
        lo, hi = bounds
        return random_waypoints(rng, lo, hi, int(cfg.get("waypoints", 8)), float(cfg.get("speed", 2.0)))
    raise ValueError(f"unknown trajectory type {kind!r}")

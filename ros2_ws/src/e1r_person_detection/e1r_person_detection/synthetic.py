"""Ray-cast synthetic E1R-like scene, for testing without the sensor.

Sensor: 120 x 90 deg FOV, mounted `height` m above the floor and pitched down `pitch_deg`.
Scene: floor, back wall, side wall, a pole, a table, and optionally a walking person.
Returns points in the lidar frame (x forward, y left, z up).
"""
import numpy as np


def _rays(h_fov=120.0, v_fov=90.0, h_res=0.5, v_res=0.6):
    az = np.deg2rad(np.arange(-h_fov / 2, h_fov / 2, h_res))
    el = np.deg2rad(np.arange(-v_fov / 2, v_fov / 2, v_res))
    A, E = np.meshgrid(az, el)
    return np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], axis=-1).reshape(-1, 3)


def _hit_plane(o, d, n, c):
    den = d @ n
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((c - o) @ n) / den
    t[(np.abs(den) < 1e-9) | (t <= 0)] = np.inf
    return t


def _hit_vcyl(o, d, cx, cy, r, z0, z1):
    ox, oy = o[0] - cx, o[1] - cy
    a = d[:, 0] ** 2 + d[:, 1] ** 2
    b = 2 * (ox * d[:, 0] + oy * d[:, 1])
    c = ox ** 2 + oy ** 2 - r ** 2
    disc = b ** 2 - 4 * a * c
    t = np.full(len(d), np.inf)
    ok = (disc >= 0) & (a > 1e-12)
    tt = (-b[ok] - np.sqrt(disc[ok])) / (2 * a[ok])
    z = o[2] + tt * d[ok, 2]
    good = (tt > 0) & (z >= z0) & (z <= z1)
    idx = np.flatnonzero(ok)[good]
    t[idx] = tt[good]
    return t


def _hit_box(o, d, lo, hi):
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (lo - o) / d
        t2 = (hi - o) / d
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    t = np.where((tmax >= tmin) & (tmin > 0), tmin, np.inf)
    return t


def person_parts(x, y, phase=0.0, height=1.75):
    """Vertical cylinders approximating a walking person standing at (x, y)."""
    s = height / 1.75
    swing = 0.15 * np.sin(phase)
    return [
        (x + swing, y - 0.1, 0.07, 0.0, 0.85 * s),        # left leg
        (x - swing, y + 0.1, 0.07, 0.0, 0.85 * s),        # right leg
        (x, y, 0.19, 0.85 * s, 1.45 * s),                 # torso
        (x + 0.02, y - 0.25, 0.05, 0.9 * s, 1.4 * s),     # arms
        (x - 0.02, y + 0.25, 0.05, 0.9 * s, 1.4 * s),
        (x, y, 0.10, 1.47 * s, 1.75 * s),                 # head
    ]


def render(persons=(), height=1.2, pitch_deg=10.0, noise=0.01, rng=None, rays=None):
    """persons: iterable of (x, y, phase, height) in world coords (floor z=0)."""
    rng = rng or np.random.default_rng()
    d_l = _rays() if rays is None else rays
    p = np.deg2rad(pitch_deg)
    # lidar -> world: pitch down by p about y
    R = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    d = d_l @ R.T
    o = np.array([0.0, 0.0, height])

    ts = [
        _hit_plane(o, d, np.array([0, 0, 1.0]), np.array([0, 0, 0.0])),     # floor
        _hit_plane(o, d, np.array([1.0, 0, 0]), np.array([9.0, 0, 0])),     # back wall
        _hit_plane(o, d, np.array([0, 1.0, 0]), np.array([0, -5.0, 0])),    # right wall
        _hit_vcyl(o, d, 4.0, 2.5, 0.06, 0.0, 2.5),                          # pole
        _hit_box(o, d, np.array([3.0, -3.5, 0.0]), np.array([3.8, -2.3, 0.75])),  # table
    ]
    for (px, py, ph, ht) in persons:
        for (cx, cy, r, z0, z1) in person_parts(px, py, ph, ht):
            ts.append(_hit_vcyl(o, d, cx, cy, r, z0, z1))
    t = np.min(np.stack(ts), axis=0)
    ok = np.isfinite(t) & (t < 30.0)
    t = t[ok] + rng.normal(0, noise, ok.sum())
    pts = d_l[ok] * t[:, None]
    inten = rng.uniform(5, 100, len(pts))
    return pts.astype(np.float32), inten.astype(np.float32)


def walking_person(t, period=20.0, empty_time=4.0):
    """Person absent for `empty_time` s, then walks across the room at x≈5 m."""
    tc = t % period
    if tc < empty_time:
        return []
    u = (tc - empty_time) / (period - empty_time)
    y = -3.5 + 7.0 * u
    x = 5.0 + 1.0 * np.sin(2 * np.pi * u)
    return [(x, y, tc * 6.0, 1.75)]

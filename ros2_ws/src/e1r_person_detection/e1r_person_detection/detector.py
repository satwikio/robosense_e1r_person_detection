"""Geometric person detector for a static RoboSense E1R (pure numpy, no ROS).

Pipeline per frame:
  1. crop to ROI + voxel downsample
  2. ground plane (RANSAC) -> transform points into a ground-aligned frame (z = height above floor)
  3. remove ground and (optionally) a learned static background
  4. DBSCAN clustering of the remaining points
  5. size/shape test of each cluster against a human envelope
  6. multi-frame tracking (see tracker.py) to suppress one-frame false positives
"""
from dataclasses import dataclass, field

import numpy as np
from sklearn.cluster import DBSCAN


@dataclass
class DetectorParams:
    # ROI in the lidar frame (metres)
    min_range: float = 0.3
    max_range: float = 20.0
    voxel_size: float = 0.06

    # mounting: rotation of the lidar about its x (roll) / y (pitch) axis relative to upright (z up).
    # Only used to know which way is "up" when searching for the floor; e.g. upside down: roll 180.
    mount_roll_deg: float = 0.0
    mount_pitch_deg: float = 0.0

    # ground plane
    ground_ransac_iters: int = 120
    ground_dist_thresh: float = 0.08      # inlier distance to plane
    ground_max_tilt_deg: float = 35.0     # max angle between floor normal and the mounting "up"
    ground_seed_fraction: float = 0.15    # RANSAC samples from this lowest fraction of points (floor is lowest)
    ground_min_inliers: float = 0.03      # min floor inlier fraction (floor may be mostly occluded)
    ground_clearance: float = 0.12        # points below this height are treated as ground
    fallback_sensor_height: float = 1.0   # used if no floor is visible
    fix_ground_after_learning: bool = True

    # static background model (assumes the lidar does not move)
    use_background: bool = True
    background_voxel: float = 0.20
    background_learn_frames: int = 30     # ~3 s at 10 Hz
    background_min_ratio: float = 0.6     # voxel is background if occupied in >= this fraction of frames

    # clustering
    cluster_eps: float = 0.30
    cluster_min_samples: int = 4
    min_cluster_points: int = 12
    max_points_per_frame: int = 40000

    # human envelope (ground frame)
    min_height: float = 1.0               # top of the cluster above the floor
    max_height: float = 2.2
    min_vertical_extent: float = 0.7
    max_bottom: float = 0.7               # lowest point must be near the floor (legs visible)
    max_footprint_major: float = 1.2
    max_footprint_minor: float = 0.9
    min_footprint_major: float = 0.15


@dataclass
class Detection:
    center: np.ndarray          # (3,) box centre in lidar frame
    size: np.ndarray            # (3,) length (major), width (minor), height
    yaw_ground: float           # yaw of the box in the ground frame
    position_ground: np.ndarray  # (2,) x, y on the floor (for tracking)
    num_points: int
    range: float
    is_person: bool
    reason: str = ""


@dataclass
class FrameResult:
    detections: list = field(default_factory=list)   # all clusters (is_person flags the humans)
    foreground: np.ndarray = None                     # (N,3) lidar-frame points that were clustered
    ground_ok: bool = False
    learning: bool = False


def voxel_downsample(pts, voxel):
    if len(pts) == 0 or voxel <= 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def _pack(keys):
    # pack 3 signed ints (each within +-2^20) into one int64 for fast set membership
    k = keys.astype(np.int64) + (1 << 20)
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


class PersonDetector:
    def __init__(self, params: DetectorParams = None, rng_seed=0):
        self.p = params or DetectorParams()
        self.rng = np.random.default_rng(rng_seed)
        self.plane = None            # (n, d) with n·x + d = height above floor, n pointing up
        self.R = None                # rotation lidar -> ground frame
        self.frames_seen = 0
        self._bg_counts = {}
        self._bg_keys = None         # sorted packed keys of background voxels

    # ------------------------------------------------------------------ ground
    def _up(self):
        """World "up" in the lidar frame implied by mount_roll_deg / mount_pitch_deg."""
        r, q = np.deg2rad(self.p.mount_roll_deg), np.deg2rad(self.p.mount_pitch_deg)
        return np.array([-np.sin(q), np.sin(r) * np.cos(q), np.cos(r) * np.cos(q)])

    def _fit_ground(self, pts):
        p = self.p
        cand = pts
        if len(cand) > 6000:
            cand = cand[self.rng.choice(len(cand), 6000, replace=False)]
        if len(cand) < 50:
            return None
        cos_tilt = np.cos(np.deg2rad(p.ground_max_tilt_deg))
        up = self._up()
        # draw samples from the lowest points: the floor may be a small fraction of the whole cloud
        hu = cand @ up
        seeds = cand[hu <= np.quantile(hu, p.ground_seed_fraction)]
        if len(seeds) < 3:
            seeds = cand
        best_n, best_d, best_inl = None, None, 0
        for _ in range(p.ground_ransac_iters):
            s = seeds[self.rng.choice(len(seeds), 3, replace=False)]
            n = np.cross(s[1] - s[0], s[2] - s[0])
            norm = np.linalg.norm(n)
            if norm < 1e-6:
                continue
            n /= norm
            if n @ up < 0:
                n = -n
            if n @ up < cos_tilt:
                continue
            d = -n @ s[0]
            # floor must be the lowest large plane: almost nothing should lie far below it
            h = cand @ n + d
            inl = np.count_nonzero(np.abs(h) < p.ground_dist_thresh)
            below = np.count_nonzero(h < -0.3)
            score = inl - 3 * below
            if score > best_inl:
                best_n, best_d, best_inl = n, d, score
        if best_n is None or best_inl < p.ground_min_inliers * len(cand):
            return None
        # least-squares refine on inliers
        h = pts @ best_n + best_d
        inl = pts[np.abs(h) < p.ground_dist_thresh]
        c = inl.mean(axis=0)
        _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
        n = vt[2] if vt[2] @ up > 0 else -vt[2]
        return n, -n @ c

    def _set_plane(self, n, d):
        self.plane = (n, d)
        # ground frame: z along n, x = lidar x projected on the floor
        ex = np.array([1.0, 0.0, 0.0]) - n[0] * n
        if np.linalg.norm(ex) < 1e-3:
            ex = np.array([0.0, 1.0, 0.0]) - n[1] * n
        ex /= np.linalg.norm(ex)
        ey = np.cross(n, ex)
        self.R = np.vstack([ex, ey, n])

    def to_ground(self, pts):
        n, d = self.plane
        g = pts @ self.R.T
        g[:, 2] += d
        return g

    def from_ground(self, g):
        n, d = self.plane
        g = np.atleast_2d(g).copy()
        g[:, 2] -= d
        return g @ self.R

    # ------------------------------------------------------------------ main
    def process(self, pts) -> FrameResult:
        p = self.p
        res = FrameResult()
        pts = np.asarray(pts, dtype=np.float64)
        pts = pts[np.isfinite(pts).all(axis=1)]
        r = np.linalg.norm(pts, axis=1)
        pts = pts[(r > p.min_range) & (r < p.max_range)]
        pts = voxel_downsample(pts, p.voxel_size)
        if len(pts) > p.max_points_per_frame:
            pts = pts[self.rng.choice(len(pts), p.max_points_per_frame, replace=False)]

        learning = p.use_background and self.frames_seen < p.background_learn_frames
        res.learning = learning

        # ground plane
        if self.plane is None or not (p.fix_ground_after_learning and not learning):
            fit = self._fit_ground(pts)
            if fit is not None:
                self._set_plane(*fit)
            elif self.plane is None:
                self._set_plane(self._up(), p.fallback_sensor_height)
        res.ground_ok = self.plane is not None
        self.frames_seen += 1

        g = self.to_ground(pts)
        above = g[:, 2] > p.ground_clearance
        g, pts = g[above], pts[above]

        # background
        if p.use_background:
            keys = _pack(np.floor(g / p.background_voxel).astype(np.int64))
            if learning:
                for k in np.unique(keys):
                    self._bg_counts[k] = self._bg_counts.get(k, 0) + 1
                if self.frames_seen >= p.background_learn_frames:
                    self._finalize_background()
                res.foreground = np.empty((0, 3))
                return res
            if self._bg_keys is not None and len(self._bg_keys):
                fg = ~np.isin(keys, self._bg_keys, assume_unique=False)
                g, pts = g[fg], pts[fg]

        res.foreground = pts
        if len(g) < p.min_cluster_points:
            return res

        labels = DBSCAN(eps=p.cluster_eps, min_samples=p.cluster_min_samples).fit_predict(g)
        for lab in np.unique(labels):
            if lab < 0:
                continue
            m = labels == lab
            if np.count_nonzero(m) < p.min_cluster_points:
                continue
            res.detections.append(self._classify(g[m]))
        return res

    def _finalize_background(self):
        p = self.p
        thresh = max(1, int(np.ceil(p.background_min_ratio * p.background_learn_frames)))
        keys = np.array([k for k, c in self._bg_counts.items() if c >= thresh], dtype=np.int64)
        if len(keys) == 0:
            self._bg_keys = keys
            return
        # unpack, dilate by one voxel in x/y (handles sensor noise at object edges)
        off = 1 << 20
        k = np.stack([(keys >> 42) & 0x1FFFFF, (keys >> 21) & 0x1FFFFF, keys & 0x1FFFFF], axis=1) - off
        shifts = np.array([[dx, dy, dz] for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)])
        dil = (k[:, None, :] + shifts[None, :, :]).reshape(-1, 3)
        self._bg_keys = np.unique(_pack(dil))
        self._bg_counts = {}

    def reset_background(self):
        self.frames_seen = 0
        self._bg_counts = {}
        self._bg_keys = None
        self.plane = None

    def _classify(self, g) -> Detection:
        p = self.p
        xy = g[:, :2]
        c_xy = xy.mean(axis=0)
        cov = np.cov((xy - c_xy).T) if len(xy) > 2 else np.eye(2) * 1e-4
        evals, evecs = np.linalg.eigh(cov)
        major = evecs[:, 1]
        yaw = float(np.arctan2(major[1], major[0]))
        rot = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
        local = (xy - c_xy) @ rot.T
        lo, hi = local.min(axis=0), local.max(axis=0)
        length, width = hi - lo
        box_c_xy = c_xy + ((lo + hi) / 2) @ rot
        zmin, zmax = float(g[:, 2].min()), float(g[:, 2].max())

        checks = [
            (p.min_height <= zmax <= p.max_height, f"top {zmax:.2f}m"),
            (zmax - zmin >= p.min_vertical_extent, f"vert {zmax - zmin:.2f}m"),
            (zmin <= p.max_bottom, f"bottom {zmin:.2f}m"),
            (p.min_footprint_major <= length <= p.max_footprint_major, f"len {length:.2f}m"),
            (width <= p.max_footprint_minor, f"wid {width:.2f}m"),
        ]
        failed = [msg for ok, msg in checks if not ok]

        # box centre: from floor to top of cluster
        center_g = np.array([box_c_xy[0], box_c_xy[1], zmax / 2.0])
        center = self.from_ground(center_g)[0]
        return Detection(
            center=center,
            size=np.array([max(length, 0.05), max(width, 0.05), zmax]),
            yaw_ground=yaw,
            position_ground=box_c_xy,
            num_points=len(g),
            range=float(np.linalg.norm(center[:2])),
            is_person=not failed,
            reason="ok" if not failed else "fail: " + ", ".join(failed),
        )

    def box_quaternion(self, yaw_ground):
        """Orientation (x, y, z, w) in the lidar frame of a box with given yaw in the ground frame."""
        cy, sy = np.cos(yaw_ground), np.sin(yaw_ground)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        m = self.R.T @ rz
        return _mat_to_quat(m)


def _mat_to_quat(m):
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return np.array([x, y, z, w])

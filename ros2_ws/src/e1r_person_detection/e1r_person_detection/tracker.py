"""Simple multi-object tracker on the floor plane (constant velocity + Hungarian association)."""
import numpy as np
from scipy.optimize import linear_sum_assignment


class Track:
    def __init__(self, tid, pos, det, t):
        self.id = tid
        self.pos = np.array(pos, dtype=float)
        self.vel = np.zeros(2)
        self.det = det
        self.hits = 1
        self.misses = 0
        self.last_t = t
        self.confirmed = False


class Tracker:
    def __init__(self, gate=1.0, confirm_hits=3, max_misses=15, coast_frames=8):
        self.gate = gate
        self.confirm_hits = confirm_hits
        self.max_misses = max_misses
        self.coast_frames = coast_frames  # keep reporting a confirmed track through short occlusions
        self.tracks = []
        self._next_id = 1

    def update(self, detections, t):
        """detections: list of Detection (only persons). Returns confirmed, recently seen tracks."""
        for tr in self.tracks:
            dt = min(t - tr.last_t, 1.0)
            tr.pred = tr.pos + tr.vel * dt

        pos = np.array([d.position_ground for d in detections]).reshape(-1, 2)
        assigned_t, assigned_d = set(), set()
        if self.tracks and len(detections):
            cost = np.linalg.norm(np.array([tr.pred for tr in self.tracks])[:, None, :] - pos[None, :, :], axis=2)
            rows, cols = linear_sum_assignment(cost)
            for r, c in zip(rows, cols):
                if cost[r, c] > self.gate:
                    continue
                tr = self.tracks[r]
                dt = max(t - tr.last_t, 1e-3)
                new_vel = (pos[c] - tr.pos) / dt
                tr.vel = 0.6 * tr.vel + 0.4 * new_vel
                tr.pos = pos[c]
                tr.det = detections[c]
                tr.hits += 1
                tr.misses = 0
                tr.last_t = t
                tr.confirmed = tr.confirmed or tr.hits >= self.confirm_hits
                assigned_t.add(r)
                assigned_d.add(c)

        for i, tr in enumerate(self.tracks):
            if i not in assigned_t:
                tr.misses += 1
        self.tracks = [tr for tr in self.tracks if tr.misses <= self.max_misses]

        for j, d in enumerate(detections):
            if j not in assigned_d:
                self.tracks.append(Track(self._next_id, pos[j], d, t))
                self._next_id += 1

        return [tr for tr in self.tracks if tr.confirmed and tr.misses <= self.coast_frames]

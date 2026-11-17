"""
rrt.py — RRT* in base_link, then shortcut + cubic-spline smoothing.

Sampling   : in Frenet (s, d) on the raceline, inside the scan-derived lateral
             bounds, so samples land on the track rather than in walls.
Goal       : a point on the normal at s = horizon, at the lateral offset closest
             to the raceline that has clearance (goal-biased sampling).
Edges      : straight, collision-checked against the scan every `edge_res` m.
             A new edge may turn at most kappa_max * edge_len * turn_slack
             relative to its parent's edge (root heading = car heading), which
             keeps the tree roughly car-followable before smoothing.
Smoothing  : greedy shortcut -> CubicSpline (start tangent clamped to the car
             heading) -> resample. Re-check clearance and kappa; if the smooth
             path fails, try smoothing the un-shortcut path; if that fails too,
             return the raw (collision-free) tree path, flagged unsmoothed.
"""

import time
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline

from local_planner.corridor import (RefPath, Obstacles, lateral_bounds,
                                    heading_curvature, resample, required_clearance)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


@dataclass
class RRTResult:
    path: np.ndarray        # (N, 2) final path in base_link, None if no path found
    smoothed: bool          # False -> raw tree path (fallback)
    n_nodes: int
    iters: int
    tree_edges: np.ndarray  # (E, 2, 2) for debug markers
    goal: np.ndarray


class RRTStarPlanner:
    def __init__(self, p: dict):
        self.p = p
        self.rng = np.random.default_rng(p['seed'] if p['seed'] >= 0 else None)

    # ---------------------------------------------------------------- checks
    def edge_free(self, a, b, obstacles, threshold):
        L = float(np.linalg.norm(b - a))
        n = max(int(np.ceil(L / self.p['edge_res'])), 1)
        t = np.linspace(0.0, 1.0, n + 1)[:, None]
        return self.clear_ok(a + t * (b - a), obstacles, threshold)

    def clear_ok(self, pts, obstacles, threshold):
        need = required_clearance(pts, threshold, self.p['near_threshold'],
                                  self.p['near_radius'])
        return bool((obstacles.clearance(pts) >= need).all())

    def path_ok(self, pts, obstacles, threshold, kappa_max):
        if not self.clear_ok(pts, obstacles, threshold):
            return False
        k = np.abs(heading_curvature(pts)[1][1:-1])
        return bool(len(k) == 0 or k.max() <= kappa_max)

    # ------------------------------------------------------------------ plan
    def plan(self, ref: RefPath, obstacles: Obstacles, threshold, kappa_max) -> RRTResult:
        p = self.p
        H = min(p['horizon'], ref.length)

        # sampling bounds along s (scan-derived), slightly inside the walls
        st = np.linspace(0.0, H, 21)
        left, right = lateral_bounds(ref, obstacles, st)
        lo_d, hi_d = right + threshold, left - threshold

        # goal: on the normal at s = H, offset closest to 0 that has clearance
        goal = None
        lo_g, hi_g = lo_d[-1], hi_d[-1]
        if hi_g >= lo_g:
            cand = np.arange(lo_g, hi_g + 1e-9, 0.05)
            cand = cand[np.argsort(np.abs(cand))]
            pts = ref.to_cartesian(np.full(len(cand), H), cand)
            ok = obstacles.clearance(pts) >= threshold + 0.05
            if ok.any():
                goal = pts[int(np.argmax(ok))]
        if goal is None:
            return RRTResult(None, False, 1, 0, np.empty((0, 2, 2)), None)

        # tree (arrays grown in place)
        cap = p['max_iters'] + 1
        nodes = np.zeros((cap, 2))
        parent = np.full(cap, -1)
        cost = np.zeros(cap)
        head = np.zeros(cap)          # heading of the edge INTO the node (root: car heading 0)
        n = 1
        best_goal_node, best_goal_cost = -1, np.inf
        step, r_near = p['max_step'], p['neighbour_radius']

        def turn_lim(edge_len):
            return kappa_max * edge_len * p['turn_slack']

        t0 = time.perf_counter()
        it = 0
        for it in range(1, p['max_iters'] + 1):
            if time.perf_counter() - t0 > p['time_budget']:
                break
            # sample
            if self.rng.random() < p['goal_bias']:
                q = goal
            else:
                ss = self.rng.uniform(0.0, H)
                lo, hi = np.interp(ss, st, lo_d), np.interp(ss, st, hi_d)
                if hi < lo:
                    continue
                q = ref.to_cartesian(ss, self.rng.uniform(lo, hi))
            # nearest + steer
            dist = np.linalg.norm(nodes[:n] - q, axis=1)
            i_near = int(np.argmin(dist))
            v = q - nodes[i_near]
            L = float(np.linalg.norm(v))
            if L < 1e-6:
                continue
            new = nodes[i_near] + v * min(1.0, step / L)
            L = min(L, step)
            h_new = float(np.arctan2(*(new - nodes[i_near])[::-1]))
            if abs(wrap(h_new - head[i_near])) > turn_lim(L):
                continue
            if not self.edge_free(nodes[i_near], new, obstacles, threshold):
                continue

            # choose best parent among neighbours (RRT*)
            dn = np.linalg.norm(nodes[:n] - new, axis=1)
            near = np.flatnonzero(dn < r_near)
            best_p, best_c = i_near, cost[i_near] + L
            for j in near:
                c = cost[j] + dn[j]
                if c >= best_c or dn[j] < 1e-6:
                    continue
                h = float(np.arctan2(*(new - nodes[j])[::-1]))
                if abs(wrap(h - head[j])) > turn_lim(dn[j]):
                    continue
                if self.edge_free(nodes[j], new, obstacles, threshold):
                    best_p, best_c = j, c
            k = n
            nodes[k], parent[k], cost[k] = new, best_p, best_c
            head[k] = float(np.arctan2(*(new - nodes[best_p])[::-1]))
            n += 1

            # rewire neighbours through the new node
            anc, a = set(), k
            while a >= 0:                 # ancestors of k: never rewire them (no cycles)
                anc.add(int(a))
                a = parent[a]
            for j in near:
                if j in anc or dn[j] < 1e-6:
                    continue
                c = cost[k] + dn[j]
                if c >= cost[j]:
                    continue
                h = float(np.arctan2(*(nodes[j] - new)[::-1]))
                if abs(wrap(h - head[k])) > turn_lim(dn[j]):
                    continue
                if self.edge_free(new, nodes[j], obstacles, threshold):
                    parent[j], cost[j], head[j] = k, c, h
                    # (descendant costs are refreshed lazily when the path is extracted)

            # reached goal?
            if np.linalg.norm(new - goal) < p['goal_tol'] and cost[k] < best_goal_cost:
                best_goal_node, best_goal_cost = k, cost[k]
                if p['stop_at_first']:
                    break

        edges = np.stack((nodes[1:n], nodes[parent[1:n]]), 1) if n > 1 else np.empty((0, 2, 2))
        if best_goal_node < 0:
            return RRTResult(None, False, n, it, edges, goal)

        # extract raw path root -> goal
        raw, k = [], best_goal_node
        while k >= 0:
            raw.append(nodes[k])
            k = parent[k]
        raw = np.array(raw[::-1])
        if (np.linalg.norm(raw[-1] - goal) > 1e-6
                and self.edge_free(raw[-1], goal, obstacles, threshold)):
            raw = np.vstack((raw, goal))

        # smoothing with re-check + fallback
        for base in (self.shortcut(raw, obstacles, threshold), raw):
            sm = self.spline(base)
            if sm is not None and self.path_ok(sm, obstacles, threshold, kappa_max):
                return RRTResult(sm, True, n, it, edges, goal)
        raw_dense, _ = resample(raw, self.p['ds'])
        return RRTResult(raw_dense, False, n, it, edges, goal)

    # ----------------------------------------------------------- smoothing
    def shortcut(self, pts, obstacles, threshold):
        out, i = [pts[0]], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self.edge_free(pts[i], pts[j], obstacles, threshold):
                j -= 1
            out.append(pts[j])
            i = j
        return np.array(out)

    def spline(self, pts):
        """Chord-length cubic spline; start tangent = car heading (+x in base_link)."""
        if len(pts) < 2:
            return None
        t = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))))
        if t[-1] < 1e-3 or np.any(np.diff(t) < 1e-6):
            return None
        try:
            cs = CubicSpline(t, pts, axis=0, bc_type=((1, np.array([1.0, 0.0])), (2, np.zeros(2))))
        except ValueError:
            return None
        tt = np.arange(0.0, t[-1] + 1e-9, self.p['ds'] / 2)
        dense, _ = resample(cs(tt), self.p['ds'])
        return dense

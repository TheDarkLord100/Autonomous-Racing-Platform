"""PL-ICP scan matching core (Censi, "An ICP variant using a point-to-line
metric", ICRA 2008). No ROS imports, so it can be tested offline.

Conventions
- A pose / transform is a numpy array [x, y, theta].
- icp() returns the pose of the *current* scan's frame expressed in the
  *reference* scan's frame, i.e. it maps current-scan points into the
  reference frame: p_ref = R(theta) p_cur + t.

Speed: the two loops that cannot be vectorised (jump-table build and the
correspondence search) are compiled with numba when it is installed, ~20x
faster. Without numba the same functions run as plain Python on lists, with
identical results. NUMBA_DISABLE_JIT=1 forces plain Python (for debugging).
"""

import math
import os
import numpy as np

try:
    from numba import njit
    HAVE_NUMBA = os.environ.get('NUMBA_DISABLE_JIT', '0') != '1'
except ImportError:
    HAVE_NUMBA = False

    def njit(*args, **kwargs):              # no-op decorator
        """Return the function unchanged (stand-in for numba.njit when numba is missing)."""
        if args and callable(args[0]):
            return args[0]
        return lambda f: f


# ---------------------------------------------------------------- SE(2) ----

def wrap_angle(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def se2_compose(a, b):
    """a ⊕ b: apply b in a's frame."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1],
                     a[1] + s * b[0] + c * b[1],
                     wrap_angle(a[2] + b[2])])


def se2_inverse(a):
    """Return the inverse of an SE(2) pose [x, y, theta]."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([-c * a[0] - s * a[1],
                     s * a[0] - c * a[1],
                     -a[2]])


def transform_points(points, pose):
    """Transform (N, 2) points by an SE(2) pose."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    R = np.array([[c, -s], [s, c]])
    return points @ R.T + pose[:2]


# ----------------------------------------------------------------- Scan ----

class Scan:
    """One laser scan in its own (laser) frame, plus Censi's jump tables.

    Jump tables (absolute indices; n = none above, -1 = none below):
      up_bigger[i]    first j > i with valid r_j >  r_i (stops at invalid ray)
      up_smaller[i]   first j > i with valid r_j <  r_i (stops at invalid ray)
      down_bigger[i]  first j < i with valid r_j >  r_i (stops at invalid ray)
      down_smaller[i] first j < i with valid r_j <  r_i (stops at invalid ray)
    """

    def __init__(self, ranges, angle_min, angle_increment,
                 range_min=0.0, range_max=float('inf'),
                 normal_length=0.4, normal_max_gap=0.2):
        """Convert ranges to points, mark valid beams, precompute normals and jump tables."""
        r = np.asarray(ranges, dtype=float)
        self.n = len(r)
        self.angle_min = float(angle_min)
        self.angle_increment = float(angle_increment)
        self.theta = self.angle_min + self.angle_increment * np.arange(self.n)
        self.valid = np.isfinite(r) & (r > range_min) & (r < range_max)
        self.ranges = np.where(self.valid, r, 0.0)
        self.points = np.column_stack((self.ranges * np.cos(self.theta),
                                       self.ranges * np.sin(self.theta)))
        # Inputs for the search kernel: contiguous numpy arrays for numba,
        # plain Python lists otherwise (element access on lists is several
        # times faster than on numpy arrays in plain Python).
        conv = (lambda a: np.ascontiguousarray(a)) if HAVE_NUMBA \
            else (lambda a: a.tolist())
        self.k_r = conv(self.ranges)
        self.k_th = conv(self.theta)
        self.k_valid = conv(self.valid)
        self.k_x = conv(self.points[:, 0])
        self.k_y = conv(self.points[:, 1])
        self._build_jump_tables()
        self._compute_normals(normal_length, normal_max_gap)

    def _build_jump_tables(self):
        """Build the jump tables used by the correspondence search."""
        n = self.n
        if HAVE_NUMBA:
            ub = np.full(n, n, np.int64)
            us = np.full(n, n, np.int64)
            db = np.full(n, -1, np.int64)
            ds = np.full(n, -1, np.int64)
        else:
            ub, us, db, ds = [n] * n, [n] * n, [-1] * n, [-1] * n
        _jump_tables_kernel(self.k_r, self.k_valid, ub, us, db, ds)
        self.up_bigger, self.up_smaller = ub, us
        self.down_bigger, self.down_smaller = db, ds

    def _compute_normals(self, length, max_gap, min_points=3,
                         max_flatness=0.05):
        """Surface normal at each ray from a line fit over the neighbouring
        points within +-length/2 of path length along the same surface.

        A line through only two neighbouring rays (Censi's j1-j2) points in a
        near-random direction when the beam spacing is comparable to the
        range noise (1080 beams over 270 deg, 1 cm noise: 0.4 cm spacing at
        1 m). The window is a fixed length of wall, not a fixed number of
        rays, so near and far walls get equally good normals.
        length <= 0 disables this; the ICP loop then uses the j1-j2 line.

        Surfaces are split wherever a ray is invalid or two consecutive
        points are more than max_gap apart. A window that is not close to a
        straight line (spread across the line / spread along it >
        max_flatness, e.g. it wraps round a corner) gets no normal.
        """
        n = self.n
        self.normal_length = length
        self.normals = np.zeros((n, 2))
        self.normal_ok = np.zeros(n, dtype=bool)
        if length <= 0 or n < min_points:
            return
        P, v = self.points, self.valid
        step = np.linalg.norm(np.diff(P, axis=0), axis=1)
        good_link = v[:-1] & v[1:] & (step < max_gap)
        # Surface id per ray: increments at every broken link.
        surf = np.concatenate(([0], np.cumsum(~good_link)))
        # Path length along the scan (only meaningful within one surface).
        s = np.concatenate(([0.0], np.cumsum(np.where(good_link, step, 0.0))))
        # Composite key so the search never crosses into another surface.
        key = surf * 1e6 + s
        half = 0.5 * length
        lo = np.searchsorted(key, key - half, side='left')
        hi = np.searchsorted(key, key + half, side='right')   # exclusive
        cnt = hi - lo
        ok = v & (cnt >= min_points)

        def wsum(a):
            """Return windowed sums of ``a`` over [lo, hi) using a cumulative sum."""
            c = np.concatenate(([0.0], np.cumsum(a)))
            return c[hi] - c[lo]
        x, y = P[:, 0], P[:, 1]
        k = np.maximum(cnt, 1)
        mx, my = wsum(x) / k, wsum(y) / k
        sxx = wsum(x * x) / k - mx * mx
        syy = wsum(y * y) / k - my * my
        sxy = wsum(x * y) / k - mx * my
        # Line direction phi from the scatter matrix; normal is phi + 90 deg.
        phi = 0.5 * np.arctan2(2.0 * sxy, sxx - syy)
        # Eigenvalues of the scatter matrix: spread along / across the line.
        half_tr = 0.5 * (sxx + syy)
        rad = np.sqrt(0.25 * (sxx - syy) ** 2 + sxy ** 2)
        lam_max, lam_min = half_tr + rad, half_tr - rad
        flat = lam_min <= max_flatness * np.maximum(lam_max, 1e-12)
        self.normals = np.column_stack((-np.sin(phi), np.cos(phi)))
        self.normal_ok = ok & flat

    def index_of_angle(self, phi):
        """Return the beam index nearest to angle ``phi``, clipped to the scan."""
        i = int(round((phi - self.angle_min) / self.angle_increment))
        return min(max(i, 0), self.n - 1)


# ------------------------------------------------- correspondence search ----
#
# Kernels: written so the same source runs compiled by numba (numpy arrays)
# or as plain Python (lists). Only indexing, arithmetic and math.* inside.

@njit(cache=True)
def _jump_tables_kernel(r, v, ub, us, db, ds):
    """Censi's jump tables (outputs ub, us, db, ds pre-filled with n / -1):
      ub[i] first j > i with valid r_j >  r_i (stops at an invalid ray)
      us[i] first j > i with valid r_j <  r_i
      db[i] first j < i with valid r_j >  r_i
      ds[i] first j < i with valid r_j <  r_i
    If r_j <= r_i, nothing up to ub[j] can beat r_i either, so hop there
    instead of stepping one by one."""
    n = len(r)
    for i in range(n - 1, -1, -1):          # up tables, right to left
        j = i + 1
        while j < n and v[j] and r[j] <= r[i]:
            j = ub[j]
        ub[i] = j
        j = i + 1
        while j < n and v[j] and r[j] >= r[i]:
            j = us[j]
        us[i] = j
    for i in range(n):                      # down tables, left to right
        j = i - 1
        while j >= 0 and v[j] and r[j] <= r[i]:
            j = db[j]
        db[i] = j
        j = i - 1
        while j >= 0 and v[j] and r[j] >= r[i]:
            j = ds[j]
        ds[i] = j


@njit(cache=True)
def _jump_search_kernel(r, th, valid, rx, ry, ub, us, db, ds,
                        angle_min, angle_inc, px_all, py_all, max_dist,
                        j1_out, j2_out):
    """Censi's fast search: start near the expected index, walk up and down,
    stop a direction once the angular gap alone rules it out, and use the
    jump tables to skip runs of rays that cannot be closer.
    j1_out / j2_out (pre-filled with -1) receive the closest ray and its
    closer valid neighbour (the second point of the line)."""
    n = len(r)
    last = n - 1
    max_d2 = max_dist * max_dist
    half_pi = 0.5 * math.pi
    last_best = -1
    for i in range(len(px_all)):
        px = px_all[i]
        py = py_all[i]
        p_norm = math.hypot(px, py)
        p_phi = math.atan2(py, px)
        start_cell = int(round((p_phi - angle_min) / angle_inc))
        start_cell = min(max(start_cell, 0), last)
        # Consecutive points usually match consecutive rays: start next to
        # the previous point's match when there is one.
        start = last_best + 1 if last_best != -1 else start_cell
        start = min(max(start, 0), last)

        up = start + 1
        down = start
        last_d_up = 0.0
        last_d_down = -1.0                  # down goes first
        up_done = False
        down_done = False
        best = -1
        best_d = 1e300

        while not (up_done and down_done):
            go_up = (not up_done) and (down_done or last_d_up < last_d_down)
            if go_up:
                if up > last:
                    up_done = True
                    continue
                if not valid[up]:
                    up += 1
                    continue
                last_d_up = (rx[up] - px) ** 2 + (ry[up] - py) ** 2
                if last_d_up < best_d:
                    best = up
                    best_d = last_d_up
                if up > start_cell:
                    # Any ray further up has a bigger angular gap, so it is
                    # at least p_norm * sin(gap) away from p.
                    gap = min(th[up] - p_phi, half_pi)
                    lower = p_norm * math.sin(gap)
                    if lower * lower > best_d:
                        up_done = True
                        continue
                    up = ub[up] if r[up] < p_norm else us[up]
                else:
                    up += 1
            else:
                if down < 0:
                    down_done = True
                    continue
                if not valid[down]:
                    down -= 1
                    continue
                last_d_down = (rx[down] - px) ** 2 + (ry[down] - py) ** 2
                if last_d_down < best_d:
                    best = down
                    best_d = last_d_down
                if down < start_cell:
                    gap = min(p_phi - th[down], half_pi)
                    lower = p_norm * math.sin(gap)
                    if lower * lower > best_d:
                        down_done = True
                        continue
                    down = db[down] if r[down] < p_norm else ds[down]
                else:
                    down -= 1

        if best == -1 or best_d > max_d2:
            last_best = -1
            continue
        # Second line point: the closer valid neighbour of j1.
        j2 = -1
        j2_d = 1e300
        for j in (best - 1, best + 1):
            if 0 <= j < n and valid[j]:
                d = (rx[j] - px) ** 2 + (ry[j] - py) ** 2
                if d < j2_d:
                    j2 = j
                    j2_d = d
        if j2 == -1:
            last_best = -1
            continue
        j1_out[i] = best
        j2_out[i] = j2
        last_best = best


def find_correspondences_jump(ref, pts_w, max_dist):
    """pts_w: (M, 2) current-scan points already moved into the ref frame.
    Returns j1, j2 arrays of length M (-1 = no correspondence)."""
    M = len(pts_w)
    if HAVE_NUMBA:
        px = np.ascontiguousarray(pts_w[:, 0])
        py = np.ascontiguousarray(pts_w[:, 1])
        j1 = np.full(M, -1, np.int64)
        j2 = np.full(M, -1, np.int64)
    else:
        px, py = pts_w[:, 0].tolist(), pts_w[:, 1].tolist()
        j1, j2 = [-1] * M, [-1] * M
    _jump_search_kernel(ref.k_r, ref.k_th, ref.k_valid, ref.k_x, ref.k_y,
                        ref.up_bigger, ref.up_smaller, ref.down_bigger,
                        ref.down_smaller, ref.angle_min, ref.angle_increment,
                        px, py, float(max_dist), j1, j2)
    return np.asarray(j1), np.asarray(j2)


def find_correspondences_brute(ref, pts_w, max_dist):
    """Reference answer: compare every point against every valid ray.
    Same j2 rule and distance gate as the jump-table search."""
    ref_idx = np.flatnonzero(ref.valid)
    M = len(pts_w)
    j1_out = np.full(M, -1)
    j2_out = np.full(M, -1)
    if len(ref_idx) == 0 or M == 0:
        return j1_out, j2_out
    q = ref.points[ref_idx]
    d2 = ((pts_w[:, None, :] - q[None, :, :]) ** 2).sum(axis=2)
    k = d2.argmin(axis=1)
    best_d = d2[np.arange(M), k]
    for i in range(M):
        if best_d[i] > max_dist * max_dist:
            continue
        j1 = int(ref_idx[k[i]])
        j2, j2_d = -1, float('inf')
        for j in (j1 - 1, j1 + 1):
            if 0 <= j < ref.n and ref.valid[j]:
                d = float(((ref.points[j] - pts_w[i]) ** 2).sum())
                if d < j2_d:
                    j2, j2_d = j, d
        if j2 != -1:
            j1_out[i], j2_out[i] = j1, j2
    return j1_out, j2_out


def warmup():
    """Compile the numba kernels now (about 1-2 s the first time, well under
    1 s from the on-disk cache afterwards) instead of on the first scan.
    Returns True if numba is in use."""
    n = 16
    ang = np.linspace(-1.0, 1.0, n)
    ranges = 2.0 + 0.1 * np.sin(5 * ang)
    scan = Scan(ranges, ang[0], ang[1] - ang[0])
    find_correspondences_jump(scan, scan.points[scan.valid], 0.5)
    return HAVE_NUMBA


# ------------------------------------------------------ closed-form solve ----

def solve_point_to_line(p, q, n, prior_t=None, prior_w=0.0):
    """Exact minimiser of
        sum_i ( n_i . (R p_i + t - q_i) )^2  +  prior_w * |t - prior_t|^2
    over (t, theta).

    Unknown x = [tx, ty, c, s] with c = cos(theta), s = sin(theta), so
    R p + t = M_i x with M_i = [[1, 0, px, -py], [0, 1, py, px]].
    Cost = x^T A x - 2 b^T x + const, subject to c^2 + s^2 = 1.
    Lagrange: (A + lam W) x = b, W = diag(0, 0, 1, 1). Eliminating t gives a
    2x2 system in (c, s); the unit-norm constraint becomes a quartic in lam.

    The prior (odometry translation) only matters along directions the
    walls do not constrain (a straight / corridor): there the data term is
    ~0 and the prior fills in; elsewhere the data term is much larger.
    prior_w is in "points' worth", like eig_min.

    p, q, n: (K, 2) sensor points, ref line points, unit line normals.
    Returns (pose [tx, ty, theta], min eigenvalue of the translation block
    of the data term alone) or (None, eig) if the system is singular.
    """
    nx, ny = n[:, 0], n[:, 1]
    # Row of the scalar residual per correspondence: n^T M_i  (K, 4)
    rows = np.column_stack((nx, ny,
                            nx * p[:, 0] + ny * p[:, 1],
                            -nx * p[:, 1] + ny * p[:, 0]))
    rhs = nx * q[:, 0] + ny * q[:, 1]
    A = rows.T @ rows
    b = rows.T @ rhs
    eig_min = float(np.linalg.eigvalsh(A[:2, :2])[0])
    if prior_w > 0.0 and prior_t is not None:
        A[0, 0] += prior_w
        A[1, 1] += prior_w
        b[:2] += prior_w * np.asarray(prior_t, dtype=float)

    At, B, D = A[:2, :2], A[:2, 2:], A[2:, 2:]
    bt, br = b[:2], b[2:]
    try:
        At_inv = np.linalg.inv(At)
    except np.linalg.LinAlgError:
        return None, eig_min

    S = D - B.T @ At_inv @ B          # 2x2 symmetric
    v = br - B.T @ At_inv @ bt
    a_, b_, c_ = S[0, 0], S[0, 1], S[1, 1]
    P = np.polynomial.polynomial
    # det(S + lam I) and adj(S + lam I) v, as polynomials in lam (low->high).
    det = P.polysub(P.polymul([a_, 1.0], [c_, 1.0]), [b_ * b_])
    adj1 = P.polysub(P.polymul([c_, 1.0], [v[0]]), [b_ * v[1]])
    adj2 = P.polysub(P.polymul([a_, 1.0], [v[1]]), [b_ * v[0]])
    # |r|^2 = 1  <=>  |adj v|^2 - det^2 = 0
    quartic = P.polysub(P.polyadd(P.polymul(adj1, adj1), P.polymul(adj2, adj2)),
                        P.polymul(det, det))
    roots = P.polyroots(quartic)
    real = roots[np.abs(roots.imag) < 1e-6 * (1.0 + np.abs(roots.real))].real
    if len(real) == 0:
        return None, eig_min

    best, best_cost = None, float('inf')
    for lam in real:
        try:
            cs = np.linalg.solve(S + lam * np.eye(2), v)
        except np.linalg.LinAlgError:
            continue
        cs /= np.linalg.norm(cs)
        t = At_inv @ (bt - B @ cs)
        x = np.concatenate((t, cs))
        cost = x @ A @ x - 2.0 * b @ x
        if cost < best_cost:
            best, best_cost = x, cost
    if best is None:
        return None, eig_min
    return np.array([best[0], best[1], math.atan2(best[3], best[2])]), eig_min


# ------------------------------------------------------------- ICP loop ----

class IcpResult:
    """Result of one PL-ICP match."""

    def __init__(self):
        """Initialise an empty (invalid) result."""
        self.pose = None          # [x, y, theta] of current scan in ref frame
        self.valid = False
        self.iterations = 0
        self.n_corr = 0           # correspondences used in the last solve
        self.error = float('nan') # mean squared point-to-line error
        self.degenerate = False   # translation poorly constrained (corridor)
        self.eig_min = float('nan')  # weakest-direction constraint (last solve)
        self.reason = ''


def icp(ref, cur, guess, max_iterations=30, max_corr_dist=0.5,
        outlier_quantile=0.7, outlier_mult=2.0, outlier_min_dist=0.02,
        min_correspondences=50,
        epsilon_xy=1e-3, epsilon_theta=1e-3,
        degeneracy_min_eig=10.0, odom_prior_weight=1.0, search='jump'):
    """PL-ICP: pose of `cur` in `ref`'s frame, starting from `guess`.

    odom_prior_weight: weight of a prior pulling the translation towards
    the guess (odometry). 0 disables it.

    degeneracy_min_eig: smaller eigenvalue of sum(n n^T) over the inliers,
    i.e. roughly "how many points' worth of wall face the weakest
    translation direction". Below it, that direction is unconstrained
    (featureless corridor / long straight) and the result is flagged
    degenerate. With the odometry prior on, a degenerate result is still
    valid: the walls fix heading and the sideways position, odometry fills
    in the along-track direction. With the prior off, it is invalid and the
    caller falls back to odometry entirely.
    Absolute, not divided by the point count: a track's forward motion is
    pinned by a few end-wall points, which a ratio would hide.
    """
    prior_t = np.array(guess[:2], dtype=float)
    find = find_correspondences_jump if search == 'jump' else find_correspondences_brute
    res = IcpResult()
    cur_idx = np.flatnonzero(cur.valid)
    p_all = cur.points[cur_idx]
    x = np.array(guess, dtype=float)
    prev_j1 = None

    for it in range(1, max_iterations + 1):
        res.iterations = it
        pts_w = transform_points(p_all, x)
        j1, j2 = find(ref, pts_w, max_corr_dist)
        ok = j1 >= 0
        if ref.normal_length > 0:
            ok &= ref.normal_ok[np.maximum(j1, 0)]
        if ok.sum() < min_correspondences:
            res.reason = f'only {int(ok.sum())} correspondences'
            return res

        p = p_all[ok]
        q1 = ref.points[j1[ok]]
        if ref.normal_length > 0:
            n = ref.normals[j1[ok]]
        else:
            d = ref.points[j2[ok]] - q1
            n = np.column_stack((-d[:, 1], d[:, 0]))
            n /= np.linalg.norm(n, axis=1, keepdims=True)

        # Adaptive trimming (Censi): drop matches whose error is well above
        # the typical error. A fixed "drop worst X%" would throw away the
        # few points that constrain the motion (e.g. walls at the ends).
        # The floor starts at max_corr_dist and halves every iteration down
        # to outlier_min_dist: with a rough guess, the few far points that
        # pin one direction have large errors at first and would be trimmed
        # for good, leaving that direction unconstrained.
        err = np.abs(((pts_w[ok] - q1) * n).sum(axis=1))
        floor = max(outlier_min_dist, max_corr_dist * 0.5 ** (it - 1))
        thresh = max(outlier_mult * np.quantile(err, outlier_quantile), floor)
        keep = err <= thresh
        if keep.sum() < min_correspondences:
            res.reason = f'only {int(keep.sum())} inliers'
            return res
        p, q1, n = p[keep], q1[keep], n[keep]

        x_new, eig_min = solve_point_to_line(p, q1, n, prior_t,
                                             odom_prior_weight)
        res.n_corr = len(p)
        res.eig_min = eig_min
        if x_new is None:
            res.reason = 'singular system'
            return res

        dx = x_new - x
        dx[2] = wrap_angle(dx[2])
        x = x_new
        # PL-ICP stops in finitely many steps: once correspondences repeat,
        # the next solve gives the same pose.
        same = prev_j1 is not None and np.array_equal(j1, prev_j1)
        prev_j1 = j1
        if same or (math.hypot(dx[0], dx[1]) < epsilon_xy
                    and abs(dx[2]) < epsilon_theta):
            break

    pts_w = transform_points(p, x)
    res.error = float(((((pts_w - q1) * n).sum(axis=1)) ** 2).mean())
    res.pose = x
    # Checked on the converged match only: early iterations start from a
    # rougher guess, keep fewer inliers and would raise false alarms.
    if res.eig_min < degeneracy_min_eig:
        res.degenerate = True
        res.reason = f'degenerate (eig_min {res.eig_min:.1f})'
        if odom_prior_weight <= 0.0:
            return res
    res.valid = True
    return res
"""Monte Carlo Localization core. No ROS imports.

Cycle (driven by the node on every scan):
    predict(delta)   motion update with the odometry since the last scan
    update(scan)     sensor update -> weights -> estimate -> resample if N_eff is low
                     -> (recovery) replace some particles with fresh guesses

Recovery = augmented MCL (Thrun et al., Probabilistic Robotics, Table 8.3).
Track the average measurement likelihood with a slow and a fast filter:
    l_avg  = log mean_i exp(mean_k log p_ik)          (per-beam likelihood, log)
    w_slow <- w_slow + alpha_slow (w_avg - w_slow)    (long-term "normal" fit)
    w_fast <- w_fast + alpha_fast (w_avg - w_fast)    (recent fit)
    p_inject = max(0, 1 - w_fast / w_slow)
When the recent fit drops well below normal (the filter has lost track), a
fraction p_inject of particles is replaced: around the current estimate
(local) and/or uniformly over free space (global). The averages are kept in
log space so they never underflow. Particles injected on the previous step
are left out of l_avg: they are mostly bad guesses, and counting them would
lower the fit and trigger more injection (a feedback loop).

Odometry scale calibration: each particle also carries a scale factor k that
multiplies its odometry translation. Along a featureless straight the LiDAR
cannot fix the position along the track, so a systematic odometry scale error
(wheel radius, VESC speed gain, slip) makes the estimate creep ahead or fall
behind. At corners, particles with the wrong k end up off the walls and die,
so the population converges to the true k and the drift on the next straight
disappears. k does a slow random walk so it can follow changing grip.
The weighted mean of k (odom_scale) is a direct estimate of the odometry
calibration error.
"""

from dataclasses import dataclass

import numpy as np

from .geometry import compose, wrap_angle
from .resampling import effective_sample_size, low_variance_resample


@dataclass
class RecoveryConfig:
    enabled: bool = True
    alpha_slow: float = 0.001     # per sensor update
    alpha_fast: float = 0.1
    min_inject_ratio: float = 0.12   # ignore p_inject below this (normal fit variation)
    max_inject_ratio: float = 0.25   # never replace more than this fraction at once
    local_std_xy: float = 1.0        # m, local injection around the estimate
    local_std_theta: float = 0.4     # rad
    global_fraction: float = 0.0     # share of injected particles drawn over the whole map


@dataclass
class ScaleConfig:
    enabled: bool = True
    init_std: float = 0.05        # initial spread of k around the current estimate
    walk_std_per_m: float = 0.002 # random walk of k per metre driven
    min_scale: float = 0.7
    max_scale: float = 1.3


class ParticleFilter:

    def __init__(self, motion_model, sensor_model, num_particles=2000,
                 resample_threshold=0.5, update_min_d=0.02, update_min_a=0.02,
                 recovery=None, occ_map=None, scale=None, rng=None):
        self.motion_model = motion_model
        self.sensor_model = sensor_model
        self.n = int(num_particles)
        self.resample_threshold = resample_threshold
        self.update_min_d = update_min_d
        self.update_min_a = update_min_a
        self.recovery = recovery if recovery is not None else RecoveryConfig(enabled=False)
        self.occ_map = occ_map
        self.scale_cfg = scale if scale is not None else ScaleConfig(enabled=False)
        self.rng = rng if rng is not None else np.random.default_rng()

        self.particles = np.zeros((self.n, 3))
        self.weights = np.full(self.n, 1.0 / self.n)
        self.scale = np.ones(self.n)      # per-particle odometry scale k
        self._fresh = np.zeros(self.n, dtype=bool)   # injected on the previous update
        self.odom_scale = 1.0             # weighted mean of k (kept across re-initialisations)
        self.initialized = False

        self._moved_d = 0.0   # translation since the last sensor update
        self._moved_a = 0.0   # rotation since the last sensor update
        self.estimate = np.zeros(3)
        self.covariance = np.zeros((3, 3))

        # diagnostics
        self.last_neff = float(self.n)
        self.resampled = False
        self.last_injected = 0
        self.p_inject = 0.0
        self._log_w_slow = None
        self._log_w_fast = None
        self._avg_count = 0

    # ------------------------------------------------------------------ init
    def initialize(self, pose, std_xy, std_theta, is_free=None, max_tries=20):
        """Gaussian cloud around pose. If is_free(x, y) is given, particles that
        land outside free space are redrawn (up to max_tries rounds)."""
        self.particles = self._sample_gaussian(self.n, pose, std_xy, std_theta,
                                               is_free, max_tries)
        self.weights = np.full(self.n, 1.0 / self.n)
        self.scale = self._sample_scale(self.n)
        self._fresh[:] = False
        self._moved_d = self._moved_a = 0.0
        self._log_w_slow = self._log_w_fast = None
        self._compute_estimate()
        self.initialized = True

    def _sample_gaussian(self, n, pose, std_xy, std_theta, is_free=None, max_tries=20):
        pose = np.asarray(pose, dtype=float)
        p = np.empty((n, 3))
        todo = np.arange(n)
        for _ in range(max_tries):
            k = todo.size
            p[todo, 0] = pose[0] + self.rng.normal(0.0, std_xy, k)
            p[todo, 1] = pose[1] + self.rng.normal(0.0, std_xy, k)
            p[todo, 2] = pose[2] + self.rng.normal(0.0, std_theta, k)
            if is_free is None:
                break
            ok = is_free(p[todo, 0], p[todo, 1])
            todo = todo[~ok]
            if todo.size == 0:
                break
        p[:, 2] = wrap_angle(p[:, 2])
        return p

    def _sample_scale(self, n):
        cfg = self.scale_cfg
        if not cfg.enabled:
            return np.ones(n)
        k = self.odom_scale + self.rng.normal(0.0, cfg.init_std, n)
        return np.clip(k, cfg.min_scale, cfg.max_scale)

    def extrapolate(self, delta):
        """Estimate moved by an odometry delta, using the learned odometry scale."""
        delta = np.asarray(delta, dtype=float)
        k = self.odom_scale
        return compose(self.estimate, np.array([k * delta[0], k * delta[1], delta[2]]))

    # --------------------------------------------------------------- predict
    def predict(self, delta):
        """Motion update with a local odometry delta (dx, dy, dtheta)."""
        if not self.initialized:
            return
        delta = np.asarray(delta, dtype=float)
        trans = float(np.hypot(delta[0], delta[1]))
        cfg = self.scale_cfg
        if cfg.enabled:
            self.motion_model.apply(self.particles, delta, self.scale)
            if cfg.walk_std_per_m > 0 and trans > 0:
                self.scale += self.rng.normal(0.0, cfg.walk_std_per_m * trans, self.n)
                np.clip(self.scale, cfg.min_scale, cfg.max_scale, out=self.scale)
        else:
            self.motion_model.apply(self.particles, delta)
        # carry the estimate forward too, so it stays current when update() is gated
        self.estimate = self.extrapolate(delta)
        self._moved_d += trans
        self._moved_a += abs(float(delta[2]))

    # ---------------------------------------------------------------- update
    def update(self, ranges, range_min, range_max):
        """Sensor update. Returns True if it ran, False if gated (car hasn't moved)."""
        self.resampled = False
        self.last_injected = 0
        if not self.initialized:
            return False
        if self._moved_d < self.update_min_d and self._moved_a < self.update_min_a:
            return False
        self._moved_d = self._moved_a = 0.0

        logw = self.sensor_model.log_likelihood(self.particles, ranges, range_min, range_max)
        self.p_inject = self._update_likelihood_averages()

        # combine with prior weights (non-uniform if the last step didn't resample)
        # floor avoids log(0) for particles whose weight underflowed last step
        logw = logw + np.log(np.maximum(self.weights, 1e-300))
        logw -= logw.max()
        w = np.exp(logw)
        self.weights = w / w.sum()

        self._compute_estimate()

        self.last_neff = effective_sample_size(self.weights)
        n_inject = int(round(self.p_inject * self.n))
        if self.last_neff < self.resample_threshold * self.n or n_inject > 0:
            idx = low_variance_resample(self.weights, self.rng)
            self.particles = self.particles[idx]
            self.scale = self.scale[idx]
            self.weights = np.full(self.n, 1.0 / self.n)
            self.resampled = True
        self._fresh[:] = False
        if n_inject > 0:
            self._inject(n_inject)
        return True

    # -------------------------------------------------------------- recovery
    def _update_likelihood_averages(self):
        """Update log w_slow / log w_fast; return the injection ratio for this step."""
        cfg = self.recovery
        mean_ll = self.sensor_model.last_mean_beam_loglik
        if not cfg.enabled or mean_ll is None:
            return 0.0
        if self._fresh.any() and not self._fresh.all():
            mean_ll = mean_ll[~self._fresh]
        # log of the particle-average per-beam likelihood (log-mean-exp)
        m = mean_ll.max()
        log_avg = m + np.log(np.mean(np.exp(mean_ll - m)))

        if self._log_w_slow is None:
            self._log_w_slow = self._log_w_fast = log_avg
            self._avg_count = 1
            return 0.0
        # Warm-up: until 1/alpha updates have been seen, use a plain running
        # mean (alpha = 1/count) so the baseline isn't stuck at the first,
        # unconverged scans.
        self._avg_count += 1
        a_slow = max(cfg.alpha_slow, 1.0 / self._avg_count)
        a_fast = max(cfg.alpha_fast, 1.0 / self._avg_count)
        # w <- (1 - a) w + a w_avg, done in log space
        self._log_w_slow = np.logaddexp(np.log1p(-a_slow) + self._log_w_slow,
                                        np.log(a_slow) + log_avg)
        self._log_w_fast = np.logaddexp(np.log1p(-a_fast) + self._log_w_fast,
                                        np.log(a_fast) + log_avg)

        p = 1.0 - np.exp(self._log_w_fast - self._log_w_slow)
        if p < cfg.min_inject_ratio:
            return 0.0
        return float(min(p, cfg.max_inject_ratio))

    def _inject(self, k):
        cfg = self.recovery
        is_free = self.occ_map.is_free if self.occ_map is not None else None
        n_global = int(self.rng.binomial(k, cfg.global_fraction)) if self.occ_map is not None else 0
        n_local = k - n_global

        new = []
        if n_local > 0:
            new.append(self._sample_gaussian(n_local, self.estimate, cfg.local_std_xy,
                                             cfg.local_std_theta, is_free))
        if n_global > 0:
            xy = self.occ_map.sample_free(n_global, self.rng)
            th = self.rng.uniform(-np.pi, np.pi, n_global)
            new.append(np.column_stack([xy, th]))
        replace = self.rng.choice(self.n, k, replace=False)
        self.particles[replace] = np.vstack(new)
        self.scale[replace] = self._sample_scale(k)
        self._fresh[replace] = True
        self.last_injected = k

    # -------------------------------------------------------------- estimate
    def _compute_estimate(self):
        w = self.weights
        x = float(np.dot(w, self.particles[:, 0]))
        y = float(np.dot(w, self.particles[:, 1]))
        th = float(np.arctan2(np.dot(w, np.sin(self.particles[:, 2])),
                              np.dot(w, np.cos(self.particles[:, 2]))))
        self.estimate = np.array([x, y, th])
        self.odom_scale = float(np.dot(w, self.scale))

        err = self.particles - self.estimate
        err[:, 2] = wrap_angle(err[:, 2])
        self.covariance = (err * w[:, None]).T @ err
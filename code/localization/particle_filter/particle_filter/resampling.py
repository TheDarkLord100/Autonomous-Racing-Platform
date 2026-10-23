"""Resampling utilities."""

import numpy as np


def effective_sample_size(weights):
    """N_eff = 1 / sum(w^2) for normalised weights."""
    return 1.0 / float(np.sum(weights * weights))


def low_variance_resample(weights, rng):
    """Systematic (low-variance) resampling. Returns (N,) indices.

    One random offset u ~ U[0, 1/N); pointers u + i/N for i = 0..N-1 are
    matched against the weight CDF. O(N), and a particle with weight w is
    copied either floor(N w) or ceil(N w) times, giving lower variance
    than drawing N independent samples.
    """
    n = weights.shape[0]
    pointers = (rng.random() + np.arange(n)) / n
    cdf = np.cumsum(weights)
    cdf[-1] = 1.0  # guard against round-off
    return np.searchsorted(cdf, pointers, side='left')
"""Occupancy grid wrapper: world<->grid transforms, free-space mask, distance field.

No ROS imports. The node converts a nav_msgs/OccupancyGrid into the plain
arguments taken by OccupancyMap.__init__.
"""

import numpy as np
from scipy.ndimage import distance_transform_edt


class OccupancyMap:
    """Static 2D map used by the filter.

    data: (height, width) int array, row-major as in OccupancyGrid
          (row 0 = bottom of the map, i.e. lowest y). Values: -1 unknown,
          0..100 occupancy probability.
    resolution: metres per cell.
    origin: (x, y, yaw) of cell (0, 0)'s corner in the world frame.
    occupied_threshold: cells with value >= this are obstacles.
    free_threshold: cells with 0 <= value < this are free space.
    max_dist: distance field is clipped at this value (m).
    """

    def __init__(self, data, resolution, origin, occupied_threshold=65,
                 free_threshold=25, max_dist=2.0):
        """Store the grid, classify free cells and compute the distance field."""
        data = np.asarray(data, dtype=np.int16)
        self.height, self.width = data.shape
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        self._cos = np.cos(self.origin[2])
        self._sin = np.sin(self.origin[2])
        self.max_dist = float(max_dist)

        occupied = data >= occupied_threshold
        self.free = (data >= 0) & (data < free_threshold)

        # Euclidean distance (m) from every cell to the nearest obstacle cell.
        # edt measures distance to the nearest zero, so pass "not occupied".
        if occupied.any():
            dist = distance_transform_edt(~occupied) * self.resolution
        else:
            dist = np.full(data.shape, self.max_dist)
        self.dist = np.minimum(dist, self.max_dist).astype(np.float32)
        self._free_rows, self._free_cols = np.nonzero(self.free)

    def world_to_grid(self, x, y):
        """World coordinates (arrays) -> integer (row, col) indices and in-bounds mask."""
        dx = np.asarray(x) - self.origin[0]
        dy = np.asarray(y) - self.origin[1]
        # rotate into the map frame (handles a non-zero origin yaw)
        mx = self._cos * dx + self._sin * dy
        my = -self._sin * dx + self._cos * dy
        col = np.floor(mx / self.resolution).astype(np.int64)
        row = np.floor(my / self.resolution).astype(np.int64)
        inside = (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        return row, col, inside

    def distance_at(self, x, y):
        """Distance to the nearest obstacle at world points; max_dist outside the map."""
        row, col, inside = self.world_to_grid(x, y)
        out = np.full(row.shape, self.max_dist, dtype=np.float32)
        out[inside] = self.dist[row[inside], col[inside]]
        return out

    def sample_free(self, n, rng):
        """n world points drawn uniformly from known free space, shape (n, 2)."""
        if self._free_rows.size == 0:
            raise RuntimeError('map has no free cells')
        k = rng.integers(0, self._free_rows.size, n)
        # random position inside each chosen cell, in the (unrotated) map frame
        mx = (self._free_cols[k] + rng.random(n)) * self.resolution
        my = (self._free_rows[k] + rng.random(n)) * self.resolution
        x = self.origin[0] + self._cos * mx - self._sin * my
        y = self.origin[1] + self._sin * mx + self._cos * my
        return np.stack([x, y], axis=1)

    def is_free(self, x, y):
        """True where world points fall in known free space."""
        row, col, inside = self.world_to_grid(x, y)
        out = np.zeros(row.shape, dtype=bool)
        out[inside] = self.free[row[inside], col[inside]]
        return out
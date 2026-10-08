"""Ground/self/side filtering and voxel-connected front-obstacle clustering."""
from itertools import product
import numpy as np

NEIGHBORS = tuple(product((-1, 0, 1), repeat=3))


def front_gap(points, config, front_bumper_x):
    """Input is an Nx3 cloud in ego base_link; return bumper-to-surface gap or None."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    keep = (np.isfinite(points).all(axis=1)
            & (points[:, 0] > front_bumper_x+0.1)
            & (points[:, 0] < front_bumper_x+config.max_distance)
            & (np.abs(points[:, 1]) < config.roi_half_width)
            & (points[:, 2] > config.min_height)
            & (points[:, 2] < config.max_height))
    points = points[keep]
    if len(points) < config.min_cluster_points:
        return None
    voxels = {}
    for index, cell in enumerate(np.floor(points/config.voxel_size).astype(int)):
        voxels.setdefault(tuple(cell), []).append(index)
    pending = set(voxels)
    candidates = []
    while pending:
        stack = [pending.pop()]
        indices = []
        while stack:
            cell = stack.pop()
            indices.extend(voxels[cell])
            for delta in NEIGHBORS:
                neighbor = tuple(cell[i]+delta[i] for i in range(3))
                if neighbor in pending:
                    pending.remove(neighbor)
                    stack.append(neighbor)
        if len(indices) >= config.min_cluster_points:
            # Low percentile selects the visible rear surface without trusting one outlier.
            candidates.append(float(np.quantile(points[indices, 0], 0.1))-front_bumper_x)
    return min(candidates) if candidates else None

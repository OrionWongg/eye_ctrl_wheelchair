"""Single-frame Euclidean clusters in base_link; no temporal state."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def small_cluster_mask(points):
    """Suppress <4-point 5 cm 3D clusters only within XY radius 1 m.

    A 5 cm support margin avoids cutting clusters at the output boundary.
    Call separately for each sensor, after body/ground/range classification.
    """
    suppressed = np.zeros(len(points), dtype=bool)
    radius = np.hypot(points[:, 0], points[:, 1])
    indices = np.flatnonzero(radius <= 1.05)
    if not len(indices):
        return suppressed
    pairs = cKDTree(points[indices]).query_pairs(.05, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
                       shape=(len(indices), len(indices)))
    _, labels = connected_components(graph, directed=False)
    sizes = np.bincount(labels)
    suppressed[indices] = (sizes[labels] < 4) & (radius[indices] <= 1.)
    return suppressed

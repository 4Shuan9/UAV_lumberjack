"""Lightweight, conservative tree hypotheses from world-frame LiDAR snapshots.

Use locally visible ground as a reference and search for a narrow lower stem
beneath a horizontally wider upper crown. No model tracking, no ML weights.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


@dataclass(frozen=True)
class TreeConfig:
    voxel_m: float = 0.18
    search_range_m: float = 22.0
    max_points: int = 14000
    stem_radius_m: float = 0.60
    min_stem_points: int = 9
    min_stem_height_m: float = 0.65
    stem_max_spread_m: float = 0.95
    crown_search_m: float = 3.5
    min_crown_points: int = 12
    min_crown_radius_m: float = 0.75
    min_height_m: float = 2.8
    max_height_m: float = 13.0
    ground_reference_m: float = 2.3
    target_exclusion_m: float = 2.5


def voxel_unique(points, voxel):
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) == 0:
        return pts.reshape(0, 3)
    keys = np.floor(pts / voxel).astype(np.int32)
    _, keep = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(keep)]


def components_xy(xy, radius):
    """Dense radius-connected components; works on sparse lower-stem returns."""
    if len(xy) == 0:
        return np.empty(0, dtype=np.int32)
    pairs = cKDTree(xy).query_pairs(radius, output_type='ndarray')
    if len(pairs) == 0:
        return np.arange(len(xy), dtype=np.int32)
    idx = np.concatenate([pairs, pairs[:, ::-1]])
    graph = coo_matrix((np.ones(len(idx), dtype=np.uint8),
                        (idx[:, 0], idx[:, 1])), shape=(len(xy), len(xy))).tocsr()
    _, label = connected_components(graph, directed=False)
    return label


def detect_tree_observations(nonground, ground, center_xy, cfg=None,
                             mapped=None, target_xy=None):
    """Return sorted geometric hypotheses: xy, ground_z, height, crown_d, score.

    A hypothesis only exists if a low thin stem and higher spreading crown are
    both actually observed. No ground support -> conservative unknown.
    """
    cfg = TreeConfig() if cfg is None else cfg
    pts = np.asarray(nonground, dtype=np.float64).reshape(-1, 3)
    ground = np.asarray(ground, dtype=np.float64).reshape(-1, 3)
    if len(pts) < cfg.min_stem_points + cfg.min_crown_points or len(ground) < 5:
        return []
    ctr = np.asarray(center_xy, dtype=np.float64)
    ok = np.isfinite(pts).all(axis=1)
    ok &= np.sum((pts[:, :2]-ctr)**2, axis=1) < cfg.search_range_m**2
    if target_xy is not None:
        ok &= np.sum((pts[:, :2]-np.asarray(target_xy))**2,axis=1) > cfg.target_exclusion_m**2
    for rec in mapped or []:
        r = max(1.4, min(3.5, .70 * float(rec['crown_d'])))
        ok &= np.sum((pts[:, :2]-np.asarray(rec['xy']))**2, axis=1) > r*r
    pts = voxel_unique(pts[ok], cfg.voxel_m)
    if len(pts) > cfg.max_points:
        # Deterministic budget; do not allocate unbounded neighbor matrices.
        pts = pts[np.linspace(0, len(pts)-1, cfg.max_points, dtype=np.int64)]
    if len(pts) < cfg.min_stem_points + cfg.min_crown_points:
        return []
    gp = ground[np.isfinite(ground).all(axis=1)]
    gtree = cKDTree(gp[:, :2])
    dist, nearest = gtree.query(pts[:, :2], k=1)
    elevation = pts[:, 2] - gp[nearest, 2]
    supported = dist <= cfg.ground_reference_m
    low = supported & (elevation >= .45) & (elevation <= 2.6)
    low_pts = pts[low]
    if len(low_pts) < cfg.min_stem_points:
        return []
    labels = components_xy(low_pts[:, :2], cfg.stem_radius_m)
    outputs = []
    for label in np.unique(labels):
        stem = low_pts[labels == label]
        if len(stem) < cfg.min_stem_points:
            continue
        if np.ptp(stem[:, 2]) < cfg.min_stem_height_m:
            continue
        xy = np.median(stem[:, :2], axis=0)
        stem_d = np.linalg.norm(stem[:, :2] - xy, axis=1)
        if np.quantile(stem_d, .85) > cfg.stem_max_spread_m:
            continue
        dg, ig = gtree.query(xy, k=1)
        if dg > cfg.ground_reference_m:
            continue
        gz = float(gp[ig, 2])
        # An upper expansion must be present, otherwise poles/rocks are rejected.
        radial = np.linalg.norm(pts[:, :2] - xy, axis=1)
        upper = pts[(radial < cfg.crown_search_m) &
                    (pts[:, 2] > gz + 2.4) &
                    (pts[:, 2] < gz + cfg.max_height_m + 1.0)]
        if len(upper) < cfg.min_crown_points:
            continue
        r_upper = np.linalg.norm(upper[:, :2]-xy,axis=1)
        crown_radius = float(np.quantile(r_upper, .80))
        if crown_radius < cfg.min_crown_radius_m:
            continue
        height = float(np.quantile(upper[:, 2], .95) - gz)
        if not cfg.min_height_m <= height <= cfg.max_height_m:
            continue
        if crown_radius <= np.quantile(stem_d, .85) + .35:
            continue
        # Reject overlapping duplicate hypotheses in the same processing cycle.
        if any(np.linalg.norm(np.asarray(o['xy'])-xy) < 1.25 for o in outputs):
            continue
        outputs.append({'xy': xy.tolist(), 'ground_z': gz,
                        'height': height, 'crown_d': 2*crown_radius,
                        'stem_points': len(stem), 'crown_points': len(upper),
                        'score': min(1., .15 + len(stem)/35. + len(upper)/90.)})
    return sorted(outputs, key=lambda r: -r['score'])

"""Airborne LiDAR terrain classification (PMF-inspired, slope-connected seeds).

Operates on individual XYZ points in *world* coordinates; does not downsample
published clouds. This is an engineered lightweight classifier, NOT PCL PMF.

Safety preference: when reliable terrain support is missing, preserve unknown
returns as obstacles rather than remove them as ground.
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy.ndimage import distance_transform_edt, grey_opening
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class TerrainConfig:
    cell_size_m: float = 0.65
    height_tolerance_m: float = 0.10
    max_slope_deg: float = 35.0
    max_reference_distance_m: float = 1.15
    min_grid_cells: int = 12
    min_ground_points: int = 50
    max_grid_cells: int = 350000
    morphology_windows: tuple = (3, 5, 9)
    opening_slope: float = 0.22
    max_seed_rise_m: float = 0.36
    # The minimum-elevation envelope alone cannot distinguish canopy-only
    # cells from ground. Demand a slope-connected surface with low anchors.
    seed_link_radius_cells: float = 2.6
    seed_max_step_rise_m: float = 0.42
    anchor_quantile: float = 0.15
    anchor_extra_height_m: float = 0.12
    # Nearest accepted terrain seed is extrapolated locally with a tangent
    # slope so a moderate hillside is not sliced into staircase bands.
    gradient_neighbors: int = 8
    gradient_max_m_per_m: float = 0.9
    # Optional known site envelope in world coordinates, disabled by default.
    # Useful when an available Gazebo terrain mesh gives a firm upper bound.
    max_ground_world_z_m: float = float("inf")
    # Auto candidate ceiling is estimated from *this scan*, not a world mesh.
    # Works for a flat validation world and for the undulating farm world.
    # Absolute upper bound is optional and disabled by default.
    auto_height_quantile: float = 0.25
    auto_height_margin_m: float = 1.10


def _terrain_slopes(xy, height, max_gradient, count=8):
    """Local least-squares tangent plane slope for each trusted terrain seed."""
    n = len(xy)
    slopes = np.zeros((n, 2), dtype=np.float64)
    if n < 3:
        return slopes
    tree = cKDTree(xy)
    k = min(n, max(3, int(count)))
    _, knn = tree.query(xy, k=k)
    if knn.ndim == 1:
        knn = knn[:, None]
    delta = xy[knn] - xy[:, None, :]
    dz = height[knn] - height[:, None]
    norms2 = np.sum(delta * delta, axis=-1)
    weights = 1. / (0.06 + norms2)
    # Only nearby seeds influence the tangent estimate, not distant hillsides.
    weights[norms2 > 3.0] = 0.
    a = np.einsum('nki,nkj,nk->nij', delta, delta, weights)
    b = np.einsum('nki,nk,nk->ni', delta, dz, weights)
    a[:, 0, 0] += .02
    a[:, 1, 1] += .02
    slopes = np.linalg.solve(a, b[..., None])[..., 0]
    lengths = np.linalg.norm(slopes, axis=1)
    over = lengths > max_gradient
    slopes[over] *= (max_gradient / lengths[over])[:, None]
    return slopes


def classify_ground(points_world, platform_mask=None, config=None):
    """Return (N,) ground mask and diagnostic reason.

    Steps: low-height grid, progressive morphological candidate removal,
    slope-constrained connectivity, low-terrain anchoring, local tangent
    extrapolation and height-residual classification of original input XYZ.
    """
    cfg = TerrainConfig() if config is None else config
    pts = np.asarray(points_world, dtype=np.float64)
    n = len(pts)
    ground = np.zeros(n, dtype=bool)
    if n == 0:
        return ground, 'empty'
    protected = (np.zeros(n, dtype=bool) if platform_mask is None
                 else np.asarray(platform_mask, dtype=bool))
    if protected.shape != (n,):
        raise ValueError('platform_mask shape mismatch')
    valid = ~protected & np.isfinite(pts).all(axis=1)
    outside = np.flatnonzero(valid)
    if len(outside) < cfg.min_ground_points:
        return ground, 'terrain_too_few_points'

    # Determine a broad search envelope from lower scan returns, before PMF.
    # This is candidate selection, NOT direct deletion by height: only points
    # supported by the resulting local terrain model become ground.
    # A global fixed -1.8m threshold excluded every observed farm return.
    auto_cap = (float(np.quantile(pts[outside, 2], cfg.auto_height_quantile))
                + cfg.auto_height_margin_m)
    ceiling = min(auto_cap, cfg.max_ground_world_z_m)
    indices = outside[pts[outside, 2] <= ceiling]
    if len(indices) < cfg.min_ground_points:
        return ground, 'terrain_too_few_candidates'
    p = pts[indices]
    keys = np.floor(p[:, :2] / cfg.cell_size_m).astype(np.int64)
    origin = keys.min(axis=0)
    ij = keys - origin
    nx, ny = (ij.max(axis=0) + 1).astype(int)
    if nx * ny > cfg.max_grid_cells:
        return ground, 'terrain_extent_too_large'
    flat = ij[:, 0] * ny + ij[:, 1]
    raw = np.full(nx * ny, np.inf, dtype=np.float64)
    np.minimum.at(raw, flat, p[:, 2])
    raw = raw.reshape((nx, ny))
    occupied = np.isfinite(raw)
    if int(occupied.sum()) < cfg.min_grid_cells:
        return ground, 'terrain_too_few_cells'

    # Morphology applied to hole-filled minimum heights, NOT to emitted XYZ.
    _, near_indices = distance_transform_edt(~occupied, return_indices=True)
    filled = raw[tuple(near_indices)]
    candidate = occupied.copy()
    baseline = filled.copy()
    for window in cfg.morphology_windows:
        opened = grey_opening(baseline, size=(window, window), mode='nearest')
        allowed = min(cfg.max_seed_rise_m,
                      cfg.height_tolerance_m +
                      cfg.opening_slope * (window - 1) * cfg.cell_size_m)
        candidate &= (raw - opened <= allowed)
        baseline = np.minimum(baseline, opened)

    # Crucial V5 difference: elevated canopy-only minima must NOT independently
    # define ground. Seed cells must be connected by believable terrain slopes
    # to large low-lying regions of the measured landscape.
    coords_grid = np.argwhere(candidate)
    if len(coords_grid) < cfg.min_grid_cells:
        return ground, 'terrain_no_candidates'
    xy = (coords_grid.astype(np.float64) + origin + .5) * cfg.cell_size_m
    z = raw[candidate]
    kd = cKDTree(xy)
    pairs = kd.query_pairs(r=cfg.seed_link_radius_cells * cfg.cell_size_m,
                           output_type='ndarray')
    max_slope = math.tan(math.radians(cfg.max_slope_deg))
    if len(pairs):
        delta = xy[pairs[:, 0]] - xy[pairs[:, 1]]
        distance = np.linalg.norm(delta, axis=1)
        rise = np.abs(z[pairs[:, 0]] - z[pairs[:, 1]])
        plausible = ((rise <= cfg.seed_max_step_rise_m + .02) &
                     (rise <= max_slope * distance + cfg.height_tolerance_m))
        links = pairs[plausible]
    else:
        links = np.empty((0, 2), dtype=np.int64)
    # SciPy connected-components is compiled and avoids Python union/find loops
    # on potentially thousands of terrain seed links per LiDAR frame.
    adjacency = coo_matrix((np.ones(len(links), dtype=np.uint8),
                            (links[:, 0], links[:, 1])),
                           shape=(len(xy), len(xy))).tocsr()
    n_components, inverse = connected_components(
        adjacency, directed=False, return_labels=True)
    group_counts = np.bincount(inverse, minlength=n_components)
    # Low altitude alone is not enough: require a spatially supported patch.
    low_ref = float(np.quantile(z, cfg.anchor_quantile))
    group_min = np.full(n_components, np.inf)
    np.minimum.at(group_min, inverse, z)
    accepted_group = ((group_counts >= cfg.min_grid_cells) &
                      (group_min <= low_ref + cfg.anchor_extra_height_m))
    accepted = accepted_group[inverse]
    if int(accepted.sum()) < cfg.min_grid_cells:
        return ground, 'terrain_no_anchored_surface'

    good_xy = xy[accepted]
    good_z = z[accepted]
    good_tree = cKDTree(good_xy)
    d, ix = good_tree.query(p[:, :2], k=1)
    slopes = _terrain_slopes(good_xy, good_z, cfg.gradient_max_m_per_m,
                            cfg.gradient_neighbors)
    offsets = p[:, :2] - good_xy[ix]
    z_est = good_z[ix] + np.einsum('ij,ij->i', offsets, slopes[ix])
    residual = p[:, 2] - z_est
    # Height test is deliberately two-sided: do not classify low spurious
    # returns or raised roots and understory as ground.
    found = ((d <= cfg.max_reference_distance_m) &
             (residual >= -cfg.height_tolerance_m) &
             (residual <= cfg.height_tolerance_m))
    if int(found.sum()) < cfg.min_ground_points:
        return ground, 'terrain_ground_support_low'
    ground[indices] = found
    return ground, 'terrain_anchored_pmf'

"""Lightweight camera-confirmed tree observations from already filtered world LiDAR.

No training weights, no data files, no Gazebo truth; RGB is used as a *positive*
foliage cue, never as a standalone 3D measurement. Confirmations require
separate camera observations and stable spatial evidence. This is a heuristic
for low-poly simulated greenery, not a general-purpose tree classifier.
"""
from dataclasses import dataclass
import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


@dataclass(frozen=True)
class VisionTreeConfig:
    search_range_m: float = 26.0
    min_above_ground_m: float = 0.72
    max_ground_reference_m: float = 5.0
    max_camera_depth_m: float = 38.0
    cluster_link_m: float = 1.10
    evidence_voxel_m: float = 0.28
    min_distinct_images: int = 4
    min_evidence_span_sec: float = 2.5
    min_unique_voxels: int = 14
    min_horizontal_extent_m: float = 0.80
    min_vertical_extent_m: float = 0.50
    min_tree_height_m: float = 2.00
    max_tree_height_m: float = 13.0
    min_canopy_radius_m: float = 0.45
    max_canopy_radius_m: float = 4.50
    min_local_texture_std: float = 8.0
    min_volume_eigen_ratio: float = 0.090
    exclude_radius_m: float = 2.0


# SDF: camera_link pose relative_to base_link = 0.120 0 -0.025 0 0.436332 0.
CAMERA_OFFSET_BASE = np.array([0.120, 0., -0.025], dtype=np.float64)
_CAMERA_PITCH = 0.436332
CAMERA_R_BASE = np.array([
    [np.cos(_CAMERA_PITCH), 0., np.sin(_CAMERA_PITCH)],
    [0., 1., 0.],
    [-np.sin(_CAMERA_PITCH), 0., np.cos(_CAMERA_PITCH)]], dtype=np.float64)


def canopy_color_mask(bgr):
    """Cheap green foliage evidence. Does NOT alone constitute a tree decision."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (29, 43, 26), (96, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            np.ones((5, 5), dtype=np.uint8))
    mask = cv2.dilate(mask, np.ones((5, 5), dtype=np.uint8))
    return mask


def select_rgb_confirmed_points(cloud_world, bgr, camera_k, base_xyz, base_R,
                                ground_world, cfg, exclude_xy=()):
    """Return points on green pixels, standing above locally measured terrain.

    Camera optical convention in UAV_lumberjack: X forward, Y left, Z up.
    Uses measured Odometry (pose at IMAGE time) and model SDF camera mount.
    """
    points = np.asarray(cloud_world, np.float64).reshape(-1, 3)
    ground = np.asarray(ground_world, np.float64).reshape(-1, 3)
    if len(points) < 5 or len(ground) < 5:
        return np.empty((0, 3), dtype=np.float32)
    xyz = np.asarray(base_xyz, np.float64)
    R = np.asarray(base_R, np.float64)
    mask = np.isfinite(points).all(axis=1)
    mask &= np.sum((points[:, :2] - xyz[:2])**2, axis=1) < cfg.search_range_m**2
    for center, radius in exclude_xy:
        center = np.asarray(center, np.float64)
        mask &= np.sum((points[:, :2] - center)**2, axis=1) > float(radius)**2
    points = points[mask]
    if not len(points):
        return np.empty((0, 3), dtype=np.float32)

    # Cull low residual surface points. We deliberately do NOT crop by a
    # fixed global z (Validation flat and Agriculture hilly share this code).
    g = ground[np.isfinite(ground).all(axis=1)]
    if len(g) < 5:
        return np.empty((0, 3), dtype=np.float32)
    dist, near = cKDTree(g[:, :2]).query(points[:, :2], k=1)
    above = (dist < cfg.max_ground_reference_m) & (
        (points[:, 2] - g[near, 2]) > cfg.min_above_ground_m)
    points = points[above]
    if len(points) < 5:
        return np.empty((0, 3), dtype=np.float32)

    camera = ((points - xyz) @ R - CAMERA_OFFSET_BASE) @ CAMERA_R_BASE
    depth = camera[:, 0]
    forward = (depth > .20) & (depth < cfg.max_camera_depth_m)
    points, camera, depth = points[forward], camera[forward], depth[forward]
    if not len(points):
        return np.empty((0, 3), dtype=np.float32)
    fx, fy, cx, cy, native_w, native_h = camera_k
    h, w = bgr.shape[:2]
    u = (cx - fx * camera[:, 1] / depth) * (w / native_w)
    v = (cy - fy * camera[:, 2] / depth) * (h / native_h)
    u = np.rint(u).astype(np.int32)
    v = np.rint(v).astype(np.int32)
    inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v, points = u[inside], v[inside], points[inside]
    if not len(points):
        return np.empty((0, 3), dtype=np.float32)
    foliage = canopy_color_mask(bgr)
    # Texture test: a broadleaf canopy is locally irregular while painted green
    # flat surfaces (notably the gazebo roof by the red barn) are smooth.
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    mu = cv2.GaussianBlur(gray, (9, 9), 0)
    sigma = np.sqrt(np.maximum(0., cv2.GaussianBlur(gray*gray, (9, 9), 0) - mu*mu))
    valid = (foliage[v, u] > 0) & (sigma[v, u] >= cfg.min_local_texture_std)
    return np.ascontiguousarray(points[valid], dtype=np.float32)


def cluster_canopy_observations(frames, ground, cfg, mapped=(), target_xy=None):
    """Cluster short-lived RGB-confirmed 3D points; return proposed tree places.

    frames: [(image_stamp, Nx3 green xyz)] from distinct image acquisitions.
    The *separate* observations are counted as evidence, not overlapping
    processing windows. Registered zones are excluded before clustering.
    """
    if len(frames) < cfg.min_distinct_images:
        return []
    all_xyz, frame_id = [], []
    frame_stamps = [float(stamp) for stamp, _ in frames]
    for fid, (_, xyz) in enumerate(frames):
        p = np.asarray(xyz, np.float64).reshape(-1, 3)
        if not len(p): continue
        all_xyz.append(p)
        frame_id.extend([fid] * len(p))
    if not all_xyz:
        return []
    pts = np.concatenate(all_xyz)
    fids = np.asarray(frame_id, dtype=np.int32)
    ok = np.isfinite(pts).all(axis=1)
    for t in mapped:
        xy = np.asarray(t['xy'], np.float64)
        radius = max(cfg.exclude_radius_m, .45 * float(t.get('crown_d', 0)))
        ok &= np.sum((pts[:, :2] - xy)**2, axis=1) > radius**2
    if target_xy is not None:
        ok &= np.sum((pts[:, :2] - np.asarray(target_xy))**2,axis=1) > 2.5**2
    pts, fids = pts[ok], fids[ok]
    if len(pts) < cfg.min_unique_voxels:
        return []
    key = np.floor(pts / cfg.evidence_voxel_m).astype(np.int32)
    _, ix = np.unique(key, axis=0, return_index=True)
    unique_xyz = pts[np.sort(ix)]
    if len(unique_xyz) < cfg.min_unique_voxels:
        return []
    # Hard cap for sustained motion through dense vegetation.
    if len(unique_xyz) > 1800:
        unique_xyz = unique_xyz[np.linspace(0,len(unique_xyz)-1,1800,dtype=int)]
    pairs = cKDTree(unique_xyz[:, :2]).query_pairs(cfg.cluster_link_m, output_type='ndarray')
    n = len(unique_xyz)
    if len(pairs):
        ind = np.concatenate([pairs, pairs[:, ::-1]], axis=0)
        graph = coo_matrix((np.ones(len(ind),dtype=np.uint8), (ind[:,0], ind[:,1])),
                           shape=(n,n)).tocsr()
        _, label = connected_components(graph, directed=False)
    else:
        label = np.arange(n)
    # Assign each raw foliage observation to the nearest voxel-level component
    # in full XYZ, so neighboring canopies do not borrow each other's frames.
    _, voxel_match = cKDTree(unique_xyz).query(pts, k=1)
    raw_label = label[voxel_match]
    g = np.asarray(ground, np.float64).reshape(-1, 3)
    g = g[np.isfinite(g).all(axis=1)]
    if len(g) < 5:
        return []
    gt = cKDTree(g[:, :2])
    obs = []
    for i in np.unique(label):
        comp = unique_xyz[label == i]
        if len(comp) < cfg.min_unique_voxels: continue
        center = np.median(comp[:, :2], axis=0)
        extent = np.ptp(comp[:, :2], axis=0)
        if max(extent) < cfg.min_horizontal_extent_m: continue
        if min(extent) < .35: continue  # Reject narrow facade / fence lines.
        eig = np.linalg.eigvalsh(np.cov(comp.T))
        if eig[-1] <= 1e-5 or eig[0]/eig[-1] < cfg.min_volume_eigen_ratio:
            continue  # Flat green gazebo roofs must not masquerade as trees.
        if np.ptp(comp[:, 2]) < cfg.min_vertical_extent_m: continue
        # Connect component evidence to ALL original point arrivals, to avoid
        # counting a reused window as multiple camera frames.
        support_ids = np.unique(fids[raw_label == i])
        supporting_fids = len(support_ids)
        if supporting_fids < cfg.min_distinct_images: continue
        # Do not count several nearly simultaneous views as independent proof.
        evidence_span = max(frame_stamps[k] for k in support_ids) - min(
            frame_stamps[k] for k in support_ids)
        if evidence_span < cfg.min_evidence_span_sec: continue
        dist, idx = gt.query(center, k=1)
        if dist > cfg.max_ground_reference_m: continue
        gz = float(g[idx, 2])
        height = float(np.quantile(comp[:, 2], .95) - gz + .4)
        if not cfg.min_tree_height_m <= height <= cfg.max_tree_height_m: continue
        radial = np.linalg.norm(comp[:, :2]-center,axis=1)
        crown_r = float(np.quantile(radial, .88))
        if not cfg.min_canopy_radius_m <= crown_r <= cfg.max_canopy_radius_m: continue
        obs.append(dict(xy=center.tolist(), ground_z=gz,
                        height=height, crown_d=2*crown_r,
                        frames=supporting_fids, evidence_span=evidence_span,
                        voxels=len(comp),
                        score=min(1., .20+.14*supporting_fids+.008*len(comp))))
    return sorted(obs,key=lambda x:-x['score'])


def refine_tree_base_xy(canopy_xy, ground_z, tree_height, crown_diameter,
                        recent_clouds):
    """Conservative one-time tree-trunk XY anchor from lower non-ground points.

    Returns (XY, source). If no sufficiently tall, *narrow*, repeatedly scanned
    lower structure exists below the canopy, preserve the original canopy XY.
    No fit/tracking runs again once the marker is registered.
    """
    xy = np.asarray(canopy_xy, dtype=np.float64).reshape(2)
    low, high = 0.35, min(1.95, float(tree_height) * .62)
    if high < 1.45:
        return xy.copy(), 'canopy'
    radius = float(np.clip(.60 * crown_diameter, 1.0, 2.3))
    gathered = []
    for _, cloud in recent_clouds:
        p = np.asarray(cloud, dtype=np.float64).reshape(-1,3)
        if len(p) == 0: continue
        rel = p[:,2] - float(ground_z)
        near = (np.sum((p[:,:2]-xy)**2, axis=1) < radius**2) & (
            (rel >= low) & (rel <= high))
        if np.any(near): gathered.append(p[near])
    if not gathered:
        return xy.copy(), 'canopy'
    p = np.concatenate(gathered)
    if len(p) < 14:
        return xy.copy(), 'canopy'
    # Repeated scans are voxel-collapsed so a single dense patch cannot dominate.
    vox = np.floor(p / np.array([.16,.16,.22])).astype(np.int32)
    _, ind = np.unique(vox, axis=0, return_index=True)
    p = p[ind]
    if len(p) < 9: return xy.copy(), 'canopy'
    # Only once per registered tree; small cropped data, no extra dependencies.
    pairs = cKDTree(p[:,:2]).query_pairs(.34, output_type='ndarray')
    if len(pairs):
        ij = np.concatenate((pairs, pairs[:,::-1]))
        graph = coo_matrix((np.ones(len(ij),np.uint8), (ij[:,0],ij[:,1])),
                           shape=(len(p),len(p))).tocsr()
        _, labels = connected_components(graph, directed=False)
    else:
        labels = np.arange(len(p))
    options=[]
    for key in np.unique(labels):
        cluster = p[labels == key]
        if len(cluster) < 9: continue
        rel = cluster[:,2] - ground_z
        if np.min(rel) > 0.95 or np.max(rel) < 1.40: continue
        if np.ptp(rel) < .70: continue
        pos = np.median(cluster[:,:2],axis=0)
        radial = np.linalg.norm(cluster[:,:2]-pos,axis=1)
        if np.quantile(radial,.85) > .42: continue
        if np.linalg.norm(pos-xy) > radius: continue
        # Prefer a vertically long, narrow structure nearest the proposed crown.
        score = np.ptp(rel) + .018*len(cluster) - .30*np.linalg.norm(pos-xy)
        options.append((score,pos))
    if not options:
        return xy.copy(), 'canopy'
    return max(options, key=lambda e:e[0])[1], 'trunk'

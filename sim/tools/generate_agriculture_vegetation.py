#!/usr/bin/env python3

import csv
import copy
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


# ============================================================
# Paths
# ============================================================

HOME = Path.home()
PROJECT = HOME / "UAV_lumberjack"
BASE_WORLD = PROJECT / "sim/worlds/agriculture_base.sdf"
DAE = PROJECT / "sim/models/cpr_agriculture/meshes/agriculture_world.dae"

OUT_CSV = PROJECT / "sim/config/agriculture_vegetation.csv"
OUT_WORLD = PROJECT / "sim/worlds/agriculture_world.sdf"


# ============================================================
# Reproducible layout
# ============================================================

SEED = 20261010
rng = random.Random(SEED)


# ============================================================
# Final target
# ============================================================

TREE_COUNT = 40
ROCK_COUNT = 10


# ============================================================
# Conservative fence-interior area
# ============================================================

X_MIN = -65.0
X_MAX =  34.0

Y_MIN = -31.0
Y_MAX =  50.0


# ============================================================
# Important locations
# ============================================================

TARGET = (0.0, 0.0)
TARGET_RADIUS = 8.0

HELIPAD = (-37.5, 15.0)
HELIPAD_RADIUS = 7.0

# Existing B -> F mission corridor
MISSION_A = HELIPAD
MISSION_B = TARGET

# Barn front -> helipad access corridor.
# Barn north/front edge is around Y=-8.
BARN_DOOR = (-39.0, -8.0)

BARN_PATH_HALF_WIDTH = 5.0


# ============================================================
# Barn maintained / no-object zone
# ============================================================

BARN = {
    "xmin": -55.246,
    "xmax": -22.743,
    "ymin": -45.605,
    "ymax":  -3.219,
}


# ============================================================
# Terrain contact offsets already verified in Gazebo
# ============================================================

TREE_Z_OFFSET = -0.05
ROCK_Z_OFFSET = -0.05


# ============================================================
# Lightweight background trees
#
# island_tree_03 intentionally excluded.
# ============================================================

TREES = {
    "island_tree_01": {
        "source": "island_tree_01_lite",
        "scale": (0.90, 1.08),
        "radius": 2.40,
        "trunk_radius": 0.24,
        "trunk_height": 3.70,
        "weight": 0.40,
    },

    "island_tree_02": {
        "source": "island_tree_02_lite",
        "scale": (0.95, 1.18),
        "radius": 2.10,
        "trunk_radius": 0.20,
        "trunk_height": 2.60,
        "weight": 0.44,
    },

    "tree_small_02": {
        "source": "tree_small_02_lite",
        "scale": (0.98, 1.15),
        "radius": 1.90,
        "trunk_radius": 0.18,
        "trunk_height": 3.30,
        "weight": 0.16,
    },
}


# ============================================================
# Rocks
#
# Only ONE-piece boulder model.
# No rock_moss_set_01 / no multi-rock set.
# ============================================================

ROCK = {
    "model": "boulder_01",

    # Larger than previous version.
    "scale": (1.55, 2.30),

    # Gazebo XYZ approximation at scale 1.
    "box": (1.272, 1.830, 1.003),
}


# ============================================================
# Geometry helpers
# ============================================================

def distance(a, b):
    return math.hypot(
        a[0] - b[0],
        a[1] - b[1],
    )


def point_segment_distance(p, a, b):

    px, py = p
    ax, ay = a
    bx, by = b

    dx = bx - ax
    dy = by - ay

    denom = dx * dx + dy * dy

    if denom == 0:
        return distance(p, a)

    t = (
        (px - ax) * dx
        +
        (py - ay) * dy
    ) / denom

    t = max(0.0, min(1.0, t))

    qx = ax + t * dx
    qy = ay + t * dy

    return math.hypot(
        px - qx,
        py - qy,
    )


def inside_barn(x, y, padding=0.0):

    return (
        BARN["xmin"] - padding
        <= x <=
        BARN["xmax"] + padding
        and
        BARN["ymin"] - padding
        <= y <=
        BARN["ymax"] + padding
    )


def valid_point(x, y, category, padding=0.0):

    # ------------------------------------
    # Fence interior
    # ------------------------------------

    if x < X_MIN + padding:
        return False

    if x > X_MAX - padding:
        return False

    if y < Y_MIN + padding:
        return False

    if y > Y_MAX - padding:
        return False

    # ------------------------------------
    # Barn + maintained surroundings
    # ------------------------------------

    if inside_barn(
        x,
        y,
        padding,
    ):
        return False

    # ------------------------------------
    # Barn door -> helipad access path
    # Hard exclusion for trees AND rocks.
    # ------------------------------------

    if point_segment_distance(
        (x, y),
        BARN_DOOR,
        HELIPAD,
    ) < BARN_PATH_HALF_WIDTH + 0.25 * padding:
        return False

    # ------------------------------------
    # Helipad
    # ------------------------------------

    if distance(
        (x, y),
        HELIPAD,
    ) < HELIPAD_RADIUS + padding:
        return False

    # ------------------------------------
    # F target tree / cutting workspace
    # ------------------------------------

    if distance(
        (x, y),
        TARGET,
    ) < TARGET_RADIUS + padding:
        return False

    # ------------------------------------
    # B -> F nominal UAV corridor
    # ------------------------------------

    if category == "tree":
        mission_clear = 3.3
    else:
        mission_clear = 2.8

    if point_segment_distance(
        (x, y),
        MISSION_A,
        MISSION_B,
    ) < mission_clear:
        return False

    return True


def random_point(category, padding=0.0):

    for _ in range(30000):

        x = rng.uniform(
            X_MIN,
            X_MAX,
        )

        y = rng.uniform(
            Y_MIN,
            Y_MAX,
        )

        if valid_point(
            x,
            y,
            category,
            padding,
        ):
            return x, y

    raise RuntimeError(
        f"Could not sample {category}"
    )


# ============================================================
# Terrain Z sampler
# ============================================================

WORLD_SHIFT = np.array(
    [-37.5, -45.0, -1.431281],
    dtype=float,
)

dae_root = ET.parse(DAE).getroot()

ns_uri = (
    dae_root.tag
    .split("}")[0]
    .strip("{")
)

ns = {"c": ns_uri}

ground = dae_root.find(
    ".//c:visual_scene//c:node[@id='Ground']",
    ns,
)

if ground is None:
    raise RuntimeError(
        "Ground node not found"
    )


matrix_node = ground.find(
    "c:matrix",
    ns,
)

if matrix_node is None:

    M = np.eye(4)

else:

    M = np.array(
        [
            float(x)
            for x
            in matrix_node.text.split()
        ],
        dtype=float,
    ).reshape(4, 4)


instance = ground.find(
    "c:instance_geometry",
    ns,
)

geometry_id = (
    instance
    .get("url")
    .lstrip("#")
)

geometry = dae_root.find(
    f".//c:geometry[@id='{geometry_id}']",
    ns,
)

mesh = geometry.find(
    "c:mesh",
    ns,
)

vertices = mesh.find(
    "c:vertices",
    ns,
)


position_source_id = None

for inp in vertices.findall(
    "c:input",
    ns,
):

    if inp.get("semantic") == "POSITION":

        position_source_id = (
            inp
            .get("source")
            .lstrip("#")
        )

        break


source = mesh.find(
    f"c:source[@id='{position_source_id}']",
    ns,
)

float_array = source.find(
    "c:float_array",
    ns,
)

accessor = source.find(
    ".//c:accessor",
    ns,
)

stride = int(
    accessor.get(
        "stride",
        "3",
    )
)

values = np.array(
    [
        float(x)
        for x
        in float_array.text.split()
    ],
    dtype=float,
)

positions_local = (
    values
    .reshape(-1, stride)[:, :3]
)

positions_h = np.hstack(
    [
        positions_local,
        np.ones(
            (len(positions_local), 1)
        ),
    ]
)

positions_world = (
    (M @ positions_h.T).T[:, :3]
    +
    WORLD_SHIFT
)


triangles = mesh.find(
    "c:triangles",
    ns,
)

inputs = triangles.findall(
    "c:input",
    ns,
)

input_stride = (
    max(
        int(
            inp.get(
                "offset",
                "0",
            )
        )
        for inp in inputs
    )
    + 1
)

vertex_offset = next(
    int(
        inp.get(
            "offset",
            "0",
        )
    )
    for inp in inputs
    if inp.get("semantic") == "VERTEX"
)

raw_indices = np.array(
    [
        int(x)
        for x
        in triangles.find(
            "c:p",
            ns,
        ).text.split()
    ],
    dtype=int,
)

faces = raw_indices[
    vertex_offset::input_stride
].reshape(-1, 3)

TRI_WORLD = positions_world[
    faces
]


def terrain_z(x, y):

    candidates = []

    for triangle in TRI_WORLD:

        x0, y0, z0 = triangle[0]
        x1, y1, z1 = triangle[1]
        x2, y2, z2 = triangle[2]

        if not (
            min(x0, x1, x2) - 1e-9
            <= x <=
            max(x0, x1, x2) + 1e-9
            and
            min(y0, y1, y2) - 1e-9
            <= y <=
            max(y0, y1, y2) + 1e-9
        ):
            continue

        denom = (
            (y1 - y2) * (x0 - x2)
            +
            (x2 - x1) * (y0 - y2)
        )

        if abs(denom) < 1e-12:
            continue

        a = (
            (y1 - y2) * (x - x2)
            +
            (x2 - x1) * (y - y2)
        ) / denom

        b = (
            (y2 - y0) * (x - x2)
            +
            (x0 - x2) * (y - y2)
        ) / denom

        c = 1.0 - a - b

        if (
            a >= -1e-8
            and
            b >= -1e-8
            and
            c >= -1e-8
        ):

            candidates.append(
                a * z0
                +
                b * z1
                +
                c * z2
            )

    if not candidates:

        raise RuntimeError(
            f"No terrain Z at "
            f"({x:.3f},{y:.3f})"
        )

    return max(candidates)


# ============================================================
# Placement storage
# ============================================================

placements = []
tree_points = []
rock_points = []


def append_object(
    category,
    model,
    group,
    x,
    y,
    scale,
):

    placements.append({
        "category": category,
        "model": model,
        "group": group,
        "x": x,
        "y": y,
        "z": terrain_z(x, y),
        "scale": scale,
        "yaw_deg": rng.uniform(
            0.0,
            360.0,
        ),
    })


# ============================================================
# Tree helpers
# ============================================================

TREE_NAMES = list(TREES)

TREE_WEIGHTS = [
    TREES[name]["weight"]
    for name in TREE_NAMES
]


def choose_tree():

    return rng.choices(
        TREE_NAMES,
        weights=TREE_WEIGHTS,
        k=1,
    )[0]


def tree_required_spacing(
    name_a,
    scale_a,
    name_b,
    scale_b,
):

    ra = (
        TREES[name_a]["radius"]
        *
        scale_a
    )

    rb = (
        TREES[name_b]["radius"]
        *
        scale_b
    )

    return max(
        3.5,
        0.74 * (ra + rb),
    )


def tree_position_valid(
    x,
    y,
    model,
    scale,
):

    for old in tree_points:

        required = tree_required_spacing(
            model,
            scale,
            old["model"],
            old["scale"],
        )

        if distance(
            (x, y),
            (old["x"], old["y"]),
        ) < required:

            return False

    return True


# ============================================================
# Trees
#
# 5 loose natural groves × 6 = 30
# + 10 isolated trees = 40
# ============================================================

grove_centres = []


while len(grove_centres) < 5:

    candidate = random_point(
        "tree",
        padding=3.0,
    )

    # Keep grove cores away from B->F
    if point_segment_distance(
        candidate,
        MISSION_A,
        MISSION_B,
    ) < 7.0:
        continue

    # Keep grove cores separated
    if any(
        distance(
            candidate,
            existing,
        ) < 15.0
        for existing
        in grove_centres
    ):
        continue

    grove_centres.append(
        candidate
    )


for grove_id, centre in enumerate(
    grove_centres,
    start=1,
):

    cx, cy = centre

    made = 0

    for _ in range(10000):

        if made >= 6:
            break

        angle = rng.uniform(
            0.0,
            2.0 * math.pi,
        )

        # Loose natural group.
        radius = min(
            abs(
                rng.gauss(
                    4.8,
                    2.2,
                )
            ),
            8.5,
        )

        x = (
            cx
            +
            radius * math.cos(angle)
        )

        y = (
            cy
            +
            radius * math.sin(angle)
        )

        if not valid_point(
            x,
            y,
            "tree",
            padding=2.0,
        ):
            continue

        model = choose_tree()

        scale = rng.uniform(
            *TREES[model]["scale"]
        )

        if not tree_position_valid(
            x,
            y,
            model,
            scale,
        ):
            continue

        append_object(
            "tree",
            model,
            f"grove_{grove_id}",
            x,
            y,
            scale,
        )

        tree_points.append({
            "model": model,
            "x": x,
            "y": y,
            "scale": scale,
        })

        made += 1


    if made != 6:

        raise RuntimeError(
            f"Could not fill grove "
            f"{grove_id}: {made}/6"
        )


# 10 isolated trees
for isolated_id in range(
    1,
    11,
):

    success = False

    for _ in range(10000):

        x, y = random_point(
            "tree",
            padding=2.0,
        )

        model = choose_tree()

        scale = rng.uniform(
            *TREES[model]["scale"]
        )

        if not tree_position_valid(
            x,
            y,
            model,
            scale,
        ):
            continue

        append_object(
            "tree",
            model,
            f"isolated_{isolated_id}",
            x,
            y,
            scale,
        )

        tree_points.append({
            "model": model,
            "x": x,
            "y": y,
            "scale": scale,
        })

        success = True
        break


    if not success:

        raise RuntimeError(
            "Could not place "
            f"isolated tree "
            f"{isolated_id}"
        )


# ============================================================
# Rocks
#
# 10 large individual boulders.
# Very dispersed.
# ============================================================

for rock_id in range(
    1,
    ROCK_COUNT + 1,
):

    success = False

    for _ in range(20000):

        x, y = random_point(
            "rock",
            padding=3.0,
        )

        # Keep rocks away from tree trunks.
        if any(
            distance(
                (x, y),
                (tree["x"], tree["y"]),
            ) < 4.5
            for tree
            in tree_points
        ):
            continue

        # Strong rock-to-rock spacing.
        if any(
            distance(
                (x, y),
                rock,
            ) < 9.0
            for rock
            in rock_points
        ):
            continue

        scale = rng.uniform(
            *ROCK["scale"]
        )

        append_object(
            "rock",
            ROCK["model"],
            f"rock_{rock_id}",
            x,
            y,
            scale,
        )

        rock_points.append(
            (x, y)
        )

        success = True
        break


    if not success:

        raise RuntimeError(
            f"Could not place "
            f"rock {rock_id}"
        )



# ============================================================
# MANUAL_RELOCATION
#
# User-reviewed final-map adjustments.
#
# 002 / 013 / 015 / 016 / 017 / 020 / 022 / 024:
#   remove from their old tree positions and redistribute into
#   naturally sparse areas.
#
# 036 / 038:
#   trees too close to fence -> move inward.
#
# 047:
#   rock too close to fence -> move inward.
# ============================================================

reloc_rng = random.Random(SEED + 1701)

SPARSE_TREE_IDS = [
    2, 13, 15, 16, 17, 20, 22, 24,
]

INWARD_IDS = [
    36, 38, 47,
]


def object_xy(obj):
    return (obj["x"], obj["y"])


def candidate_clear(
    x,
    y,
    current_index,
    category,
):
    """
    Final-layout spacing check against all currently placed objects.
    current_index is zero-based.
    """

    for j, other in enumerate(placements):

        if j == current_index:
            continue

        d = distance(
            (x, y),
            object_xy(other),
        )

        if category == "tree":

            if other["category"] == "tree":
                if d < 4.8:
                    return False

            elif other["category"] == "rock":
                if d < 4.2:
                    return False

        elif category == "rock":

            if other["category"] == "rock":
                if d < 8.0:
                    return False

            elif other["category"] == "tree":
                if d < 4.5:
                    return False

    return True


def sparse_score(
    x,
    y,
    current_index,
):
    """
    Larger score = visually sparser area.
    Primary criterion is nearest existing tree.
    """

    nearest_tree = 999.0
    nearest_any = 999.0

    for j, other in enumerate(placements):

        if j == current_index:
            continue

        d = distance(
            (x, y),
            object_xy(other),
        )

        nearest_any = min(
            nearest_any,
            d,
        )

        if other["category"] == "tree":
            nearest_tree = min(
                nearest_tree,
                d,
            )

    # Prefer genuinely sparse tree regions,
    # while mildly discouraging isolated edge placement.
    return (
        1.0 * nearest_tree
        +
        0.15 * nearest_any
    )


def relocate_tree_to_sparse_area(
    object_id,
):
    idx = object_id - 1
    obj = placements[idx]

    if obj["category"] != "tree":
        raise RuntimeError(
            f"vegetation_{object_id:03d} is not a tree"
        )

    old_x = obj["x"]
    old_y = obj["y"]

    best = None
    best_score = -1e9

    # Sample many legal locations and choose one in
    # a comparatively sparse interior region.
    for _ in range(6000):

        x = reloc_rng.uniform(
            X_MIN + 4.0,
            X_MAX - 4.0,
        )

        y = reloc_rng.uniform(
            Y_MIN + 4.0,
            Y_MAX - 4.0,
        )

        if not valid_point(
            x,
            y,
            "tree",
            padding=2.5,
        ):
            continue

        if not candidate_clear(
            x,
            y,
            idx,
            "tree",
        ):
            continue

        score = sparse_score(
            x,
            y,
            idx,
        )

        if score > best_score:
            best_score = score
            best = (x, y)

    if best is None:
        raise RuntimeError(
            f"Could not relocate vegetation_{object_id:03d}"
        )

    obj["x"] = best[0]
    obj["y"] = best[1]
    obj["z"] = terrain_z(
        best[0],
        best[1],
    )
    obj["group"] = (
        f"manual_sparse_{object_id:03d}"
    )

    print(
        f"move vegetation_{object_id:03d}: "
        f"({old_x:.2f},{old_y:.2f}) "
        f"-> "
        f"({obj['x']:.2f},{obj['y']:.2f})"
    )


def relocate_inward(
    object_id,
):
    idx = object_id - 1
    obj = placements[idx]

    category = obj["category"]

    if category not in (
        "tree",
        "rock",
    ):
        raise RuntimeError(
            f"vegetation_{object_id:03d} unsupported category "
            f"{category}"
        )

    old_x = obj["x"]
    old_y = obj["y"]

    # Pull target inward from all four fence sides.
    margin = 8.0

    target_x = min(
        max(
            old_x,
            X_MIN + margin,
        ),
        X_MAX - margin,
    )

    target_y = min(
        max(
            old_y,
            Y_MIN + margin,
        ),
        Y_MAX - margin,
    )

    best = None
    best_cost = 1e9

    # Search close to the inward-projected position.
    for _ in range(5000):

        angle = reloc_rng.uniform(
            0.0,
            2.0 * math.pi,
        )

        radius = reloc_rng.uniform(
            0.0,
            6.0,
        )

        x = (
            target_x
            +
            radius * math.cos(angle)
        )

        y = (
            target_y
            +
            radius * math.sin(angle)
        )

        if not valid_point(
            x,
            y,
            category,
            padding=3.0,
        ):
            continue

        if not candidate_clear(
            x,
            y,
            idx,
            category,
        ):
            continue

        # Stay near the original local region while moving inward.
        cost = distance(
            (x, y),
            (target_x, target_y),
        )

        if cost < best_cost:
            best_cost = cost
            best = (x, y)

    if best is None:
        raise RuntimeError(
            f"Could not move vegetation_{object_id:03d} inward"
        )

    obj["x"] = best[0]
    obj["y"] = best[1]
    obj["z"] = terrain_z(
        best[0],
        best[1],
    )
    obj["group"] = (
        f"manual_inward_{object_id:03d}"
    )

    print(
        f"move vegetation_{object_id:03d} inward: "
        f"({old_x:.2f},{old_y:.2f}) "
        f"-> "
        f"({obj['x']:.2f},{obj['y']:.2f})"
    )


print()
print("===== USER MANUAL RELOCATION =====")

# First remove / redistribute the unwanted tree positions.
for object_id in SPARSE_TREE_IDS:
    relocate_tree_to_sparse_area(
        object_id
    )

# Then pull fence-near objects inward.
for object_id in INWARD_IDS:
    relocate_inward(
        object_id
    )

print("===== RELOCATION DONE =====")
print()


# ============================================================
# POSITION_FINE_TUNING
#
# 017: move farther inward from fence
# 022: move near midpoint between 023 and 029
# 033: move near 040
# ============================================================

fine_rng = random.Random(SEED + 33040)


def set_tree_position(object_id, x, y, tag):

    idx = object_id - 1
    obj = placements[idx]

    if obj["category"] != "tree":
        raise RuntimeError(
            f"vegetation_{object_id:03d} is not a tree"
        )

    old = (obj["x"], obj["y"])

    obj["x"] = x
    obj["y"] = y
    obj["z"] = terrain_z(x, y)
    obj["group"] = tag

    print(
        f"fine move vegetation_{object_id:03d}: "
        f"({old[0]:.2f},{old[1]:.2f}) "
        f"-> ({x:.2f},{y:.2f})"
    )


# ------------------------------------------------------------
# 017: move another ~4.5 m toward map interior
# ------------------------------------------------------------

idx = 17 - 1
obj = placements[idx]

map_center = (
    (X_MIN + X_MAX) / 2.0,
    (Y_MIN + Y_MAX) / 2.0,
)

vx = map_center[0] - obj["x"]
vy = map_center[1] - obj["y"]

norm = math.hypot(vx, vy)

if norm < 1e-9:
    raise RuntimeError("vegetation_017 already at map centre")

target_017 = (
    obj["x"] + 4.5 * vx / norm,
    obj["y"] + 4.5 * vy / norm,
)

best = None
best_cost = 1e9

for _ in range(5000):

    angle = fine_rng.uniform(
        0.0,
        2.0 * math.pi,
    )

    radius = fine_rng.uniform(
        0.0,
        2.0,
    )

    x = (
        target_017[0]
        +
        radius * math.cos(angle)
    )

    y = (
        target_017[1]
        +
        radius * math.sin(angle)
    )

    if not valid_point(
        x,
        y,
        "tree",
        padding=2.0,
    ):
        continue

    if not candidate_clear(
        x,
        y,
        idx,
        "tree",
    ):
        continue

    cost = distance(
        (x, y),
        target_017,
    )

    if cost < best_cost:
        best_cost = cost
        best = (x, y)

if best is None:
    raise RuntimeError(
        "Could not fine-tune vegetation_017"
    )

set_tree_position(
    17,
    best[0],
    best[1],
    "final_inward_017",
)


# ------------------------------------------------------------
# 022: approximately between 023 and 029
# ------------------------------------------------------------

p23 = placements[23 - 1]
p29 = placements[29 - 1]

midpoint = (
    (p23["x"] + p29["x"]) / 2.0,
    (p23["y"] + p29["y"]) / 2.0,
)

idx = 22 - 1

best = None
best_cost = 1e9

for _ in range(7000):

    angle = fine_rng.uniform(
        0.0,
        2.0 * math.pi,
    )

    radius = fine_rng.uniform(
        0.0,
        3.0,
    )

    x = (
        midpoint[0]
        +
        radius * math.cos(angle)
    )

    y = (
        midpoint[1]
        +
        radius * math.sin(angle)
    )

    if not valid_point(
        x,
        y,
        "tree",
        padding=2.0,
    ):
        continue

    if not candidate_clear(
        x,
        y,
        idx,
        "tree",
    ):
        continue

    cost = distance(
        (x, y),
        midpoint,
    )

    if cost < best_cost:
        best_cost = cost
        best = (x, y)

if best is None:
    raise RuntimeError(
        "Could not place vegetation_022 between 023 and 029"
    )

set_tree_position(
    22,
    best[0],
    best[1],
    "final_between_023_029",
)


# ------------------------------------------------------------
# 033: move near 040, but keep natural spacing
# ------------------------------------------------------------

p40 = placements[40 - 1]
idx = 33 - 1

best = None
best_cost = 1e9

for _ in range(7000):

    angle = fine_rng.uniform(
        0.0,
        2.0 * math.pi,
    )

    # Near 040, but not visually stacked.
    radius = fine_rng.uniform(
        4.5,
        6.0,
    )

    x = (
        p40["x"]
        +
        radius * math.cos(angle)
    )

    y = (
        p40["y"]
        +
        radius * math.sin(angle)
    )

    if not valid_point(
        x,
        y,
        "tree",
        padding=2.0,
    ):
        continue

    if not candidate_clear(
        x,
        y,
        idx,
        "tree",
    ):
        continue

    # Prefer about 5 m from tree 040.
    cost = abs(
        distance(
            (x, y),
            (p40["x"], p40["y"]),
        )
        - 5.0
    )

    if cost < best_cost:
        best_cost = cost
        best = (x, y)

if best is None:
    raise RuntimeError(
        "Could not place vegetation_033 near 040"
    )

set_tree_position(
    33,
    best[0],
    best[1],
    "final_near_040",
)


print()
print("===== FINAL FINE-TUNE REFERENCES =====")

for object_id in [
    17,
    22, 23, 29,
    33, 40,
]:
    obj = placements[object_id - 1]

    print(
        f"vegetation_{object_id:03d}: "
        f"x={obj['x']:.2f}, "
        f"y={obj['y']:.2f}"
    )

print()

# ============================================================
# Validate counts
# ============================================================

tree_count = sum(
    p["category"] == "tree"
    for p in placements
)

rock_count = sum(
    p["category"] == "rock"
    for p in placements
)

if tree_count != TREE_COUNT:
    raise RuntimeError(
        f"Tree count = {tree_count}"
    )

if rock_count != ROCK_COUNT:
    raise RuntimeError(
        f"Rock count = {rock_count}"
    )

if len(placements) != 50:
    raise RuntimeError(
        f"Total count = "
        f"{len(placements)}"
    )


# ============================================================
# Write CSV
# ============================================================

fields = [
    "category",
    "model",
    "group",
    "x",
    "y",
    "z",
    "scale",
    "yaw_deg",
]


with OUT_CSV.open(
    "w",
    newline="",
) as f:

    writer = csv.DictWriter(
        f,
        fieldnames=fields,
    )

    writer.writeheader()

    for p in placements:

        writer.writerow({
            "category":
                p["category"],

            "model":
                p["model"],

            "group":
                p["group"],

            "x":
                f'{p["x"]:.3f}',

            "y":
                f'{p["y"]:.3f}',

            "z":
                f'{p["z"]:.6f}',

            "scale":
                f'{p["scale"]:.3f}',

            "yaw_deg":
                f'{p["yaw_deg"]:.1f}',
        })


# ============================================================
# Build optimized Gazebo test world
# ============================================================

world_text = (
    BASE_WORLD.read_text()
)

model_blocks = []


def load_model(
    sdf_path,
):

    root = (
        ET.parse(sdf_path)
        .getroot()
    )

    model = root.find(
        "model"
    )

    if model is None:

        raise RuntimeError(
            f"No model in "
            f"{sdf_path}"
        )

    return copy.deepcopy(
        model
    )


for index, p in enumerate(
    placements,
    start=1,
):

    category = p["category"]
    model_name = p["model"]
    scale = p["scale"]

    # ------------------------------------
    # Source model
    # ------------------------------------

    if category == "tree":

        source_name = (
            TREES[
                model_name
            ]["source"]
        )

        sdf_path = (
            PROJECT
            / "sim/models"
            / source_name
            / "model.sdf"
        )

        z_offset = (
            TREE_Z_OFFSET
        )

    else:

        source_name = (
            ROCK["model"]
        )

        sdf_path = (
            PROJECT
            / "sim/models"
            / source_name
            / "model.sdf"
        )

        z_offset = (
            ROCK_Z_OFFSET
        )


    model = load_model(
        sdf_path
    )


    instance_name = (
        f"vegetation_{index:03d}_"
        f"{category}_"
        f"{model_name}"
    )

    model.set(
        "name",
        instance_name,
    )


    # ------------------------------------
    # World pose
    # ------------------------------------

    yaw = math.radians(
        p["yaw_deg"]
    )

    pose = ET.Element(
        "pose"
    )

    pose.text = (
        f'{p["x"]:.3f} '
        f'{p["y"]:.3f} '
        f'{p["z"] + z_offset:.6f} '
        f'0 0 {yaw:.6f}'
    )

    model.insert(
        0,
        pose,
    )


    # ------------------------------------
    # Visual mesh scale / URI
    # ------------------------------------

    for mesh_node in model.findall(
        ".//mesh"
    ):

        uri = mesh_node.find(
            "uri"
        )

        if (
            uri is not None
            and
            uri.text
            and
            "://" not in uri.text
        ):

            uri.text = (
                f"model://"
                f"{source_name}/"
                f"{uri.text.strip()}"
            )


        scale_node = mesh_node.find(
            "scale"
        )

        if scale_node is None:

            scale_node = (
                ET.SubElement(
                    mesh_node,
                    "scale",
                )
            )


        scale_node.text = (
            f"{scale:.6f} "
            f"{scale:.6f} "
            f"{scale:.6f}"
        )


    link = model.find(
        "link"
    )

    if link is None:

        raise RuntimeError(
            f"No link in "
            f"{source_name}"
        )


    # Delete any source mesh collision.
    for old_collision in list(
        link.findall(
            "collision"
        )
    ):

        link.remove(
            old_collision
        )


    # ========================================================
    # TREE:
    # lite visual + cylinder trunk
    # ========================================================

    if category == "tree":

        cfg = TREES[
            model_name
        ]

        trunk_radius = (
            cfg["trunk_radius"]
            *
            scale
        )

        trunk_height = (
            cfg["trunk_height"]
            *
            scale
        )


        collision = ET.Element(
            "collision",
            {
                "name":
                "trunk_collision_simple"
            },
        )


        cp = ET.SubElement(
            collision,
            "pose",
        )

        cp.text = (
            f"0 0 "
            f"{trunk_height / 2:.6f} "
            f"0 0 0"
        )


        geometry = ET.SubElement(
            collision,
            "geometry",
        )

        cylinder = ET.SubElement(
            geometry,
            "cylinder",
        )


        radius_node = ET.SubElement(
            cylinder,
            "radius",
        )

        radius_node.text = (
            f"{trunk_radius:.6f}"
        )


        length_node = ET.SubElement(
            cylinder,
            "length",
        )

        length_node.text = (
            f"{trunk_height:.6f}"
        )


        link.insert(
            0,
            collision,
        )


    # ========================================================
    # ROCK:
    # original visual + simple box collision
    # ========================================================

    else:

        bx, by, bz = (
            ROCK["box"]
        )

        sx = bx * scale
        sy = by * scale
        sz = bz * scale


        collision = ET.Element(
            "collision",
            {
                "name":
                "rock_collision_simple"
            },
        )


        cp = ET.SubElement(
            collision,
            "pose",
        )

        cp.text = (
            f"0 0 "
            f"{sz / 2:.6f} "
            f"0 0 0"
        )


        geometry = ET.SubElement(
            collision,
            "geometry",
        )

        box = ET.SubElement(
            geometry,
            "box",
        )

        size = ET.SubElement(
            box,
            "size",
        )

        size.text = (
            f"{sx:.6f} "
            f"{sy:.6f} "
            f"{sz:.6f}"
        )


        link.insert(
            0,
            collision,
        )


    ET.indent(
        model,
        space="  ",
    )


    model_blocks.append(
        ET.tostring(
            model,
            encoding="unicode",
        )
    )


# ============================================================
# Insert generated models into temporary world
# ============================================================

pos = world_text.rfind(
    "</world>"
)

if pos < 0:

    raise RuntimeError(
        "</world> not found"
    )


world_text = (
    world_text[:pos]
    +
    "\n\n"
    +
    "    <!-- AGRICULTURE VEGETATION: 40 TREES + 10 BOULDERS -->\n"
    +
    "\n\n".join(
        model_blocks
    )
    +
    "\n\n"
    +
    world_text[pos:]
)


OUT_WORLD.write_text(
    world_text
)


# ============================================================
# Summary
# ============================================================

print()
print(
    "========================================"
)

print(
    " AGRICULTURE VEGETATION"
)

print(
    "========================================"
)

print(
    "seed    :", SEED
)

print(
    "world   :", OUT_WORLD
)

print(
    "csv     :", OUT_CSV
)

print()

print(
    "trees   :", tree_count
)

print(
    "rocks   :", rock_count
)

print(
    "grass   : 0"
)

print(
    "total   :", len(placements)
)

print()

print(
    "fence X :", X_MIN, "..", X_MAX
)

print(
    "fence Y :", Y_MIN, "..", Y_MAX
)

print()

print(
    "Barn -> helipad corridor:"
)

print(
    "  start  :", BARN_DOOR
)

print(
    "  finish :", HELIPAD
)

print(
    "  half width:",
    BARN_PATH_HALF_WIDTH,
    "m"
)

print()

print(
    "groves:"
)

for i, p in enumerate(
    grove_centres,
    start=1,
):

    print(
        f"  grove_{i}: "
        f"({p[0]:.2f}, "
        f"{p[1]:.2f})"
    )

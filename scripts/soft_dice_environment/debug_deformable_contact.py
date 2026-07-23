"""
Standalone contact-stability test for a prepared Isaac Lab 3.0
volume-deformable USD.

The test:

1. Imports one prepared deformable USD.
2. Applies only a deformable physics material at runtime.
3. Does not apply deformable body properties.
4. Does not rewrite nodal state.
5. Lets the object fall onto a ground plane.
6. Checks tetrahedron signed volumes every simulation step.
7. Stops immediately at the first detected inversion.
8. Reports where the inverted tetrahedra were located in the rest mesh.
9. Saves first-inversion details to CSV.
"""

import argparse
import csv
from pathlib import Path

import numpy as np
import torch

from isaaclab.app import AppLauncher


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------

parser = argparse.ArgumentParser(
    description=(
        "Detect and spatially classify the first tetrahedron inversion "
        "in a prepared volume-deformable USD."
    )
)

parser.add_argument(
    "--usd_path",
    type=str,
    default=(
        "/home/chiara/git/IsaacLab_v3_test/"
        "dice_superquadric_deformable_two_meshes.usd"
    ),
)

parser.add_argument(
    "--steps",
    type=int,
    default=1200,
)

parser.add_argument(
    "--dt",
    type=float,
    default=1.0 / 120.0,
)

parser.add_argument(
    "--print_every",
    type=int,
    default=30,
)

parser.add_argument(
    "--start_z",
    type=float,
    default=0.18,
    help=(
        "Initial wrapper position in z. For a 0.31 m object, 0.16 m "
        "places the nominal bottom approximately 5 mm above z=0."
    ),
)

parser.add_argument(
    "--ground_z",
    type=float,
    default=0.0,
)

parser.add_argument(
    "--surface_band_fraction",
    type=float,
    default=0.15,
    help=(
        "Fraction of the initial extent used to classify tetrahedron "
        "centroids as close to a surface."
    ),
)

parser.add_argument(
    "--worst_quality_fraction",
    type=float,
    default=0.01,
    help=(
        "Fraction of tetrahedra treated as the worst initial-quality set."
    ),
)

parser.add_argument(
    "--max_inverted_to_print",
    type=int,
    default=40,
)

parser.add_argument(
    "--inversion_csv",
    type=str,
    default="first_tetrahedron_inversion.csv",
)

AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(visualizer=["kit"])

args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


# -----------------------------------------------------------------------------
# Isaac Lab / USD imports
# -----------------------------------------------------------------------------

import isaaclab.sim as sim_utils

from isaaclab.assets import DeformableObject
from isaaclab.assets import DeformableObjectCfg
from isaaclab.sim import SimulationContext

from isaaclab_physx.physics import PhysxCfg
from isaaclab_physx.sim.spawners.materials import (
    PhysxDeformableBodyMaterialCfg,
)

from pxr import PhysxSchema
from pxr import Usd
from pxr import UsdGeom
from pxr import UsdPhysics
from pxr import Gf


# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------

CUBE_WRAPPER_PATH = "/World/Cube"

# These must match the prepared USD.
#
# Current asset:
EXPECTED_REST_OFFSET = 0.001
EXPECTED_CONTACT_OFFSET = 0.003

# If you regenerate the asset using the smaller offsets, use:
#
# EXPECTED_REST_OFFSET = 0.0
# EXPECTED_CONTACT_OFFSET = 0.001


# -----------------------------------------------------------------------------
# USD discovery
# -----------------------------------------------------------------------------

def find_single_deformable_root(
    stage,
    search_root_path: str,
):
    """Find exactly one Omni Physics deformable-body root."""

    search_root = stage.GetPrimAtPath(
        search_root_path
    )

    if not search_root.IsValid():
        raise RuntimeError(
            f"Search root does not exist: {search_root_path}"
        )

    matches = []

    for prim in Usd.PrimRange(search_root):
        if prim.HasAPI(
            "OmniPhysicsDeformableBodyAPI"
        ):
            matches.append(prim)

    print("\n" + "=" * 100)
    print("DEFORMABLE ROOT DISCOVERY")
    print("=" * 100)
    print("search root:", search_root_path)

    for prim in matches:
        print("  path:", prim.GetPath())
        print(
            "  schemas:",
            list(prim.GetAppliedSchemas()),
        )

    if len(matches) != 1:
        raise RuntimeError(
            "Expected exactly one "
            "OmniPhysicsDeformableBodyAPI prim below "
            f"{search_root_path}, found {len(matches)}: "
            f"{[str(prim.GetPath()) for prim in matches]}"
        )

    return matches[0]


def find_single_simulation_tetmesh(
    deformable_root,
):
    """Find the TetMesh carrying the volume-simulation API."""

    matches = []

    for prim in Usd.PrimRange(
        deformable_root
    ):
        if prim.GetTypeName() != "TetMesh":
            continue

        if prim.HasAPI(
            "OmniPhysicsVolumeDeformableSimAPI"
        ):
            matches.append(prim)

    print("\n" + "=" * 100)
    print("SIMULATION TETMESH DISCOVERY")
    print("=" * 100)

    for prim in matches:
        print("  path:", prim.GetPath())
        print(
            "  schemas:",
            list(prim.GetAppliedSchemas()),
        )

    if len(matches) != 1:
        raise RuntimeError(
            "Expected exactly one simulation TetMesh below "
            f"{deformable_root.GetPath()}, found "
            f"{[str(prim.GetPath()) for prim in matches]}"
        )

    return matches[0]

def find_single_collision_tetmesh(
    deformable_root,
):
    """
    Find exactly one TetMesh carrying UsdPhysics.CollisionAPI.

    In the two-mesh hierarchy:
        sim_mesh  -> simulation only
        coll_mesh -> collision only
    """

    matches = []

    for prim in Usd.PrimRange(
        deformable_root
    ):
        if prim.GetTypeName() != "TetMesh":
            continue

        if prim.HasAPI(
            UsdPhysics.CollisionAPI
        ):
            matches.append(prim)

    print("\n" + "=" * 100)
    print("COLLISION TETMESH DISCOVERY")
    print("=" * 100)

    for prim in matches:
        print(
            "  path:",
            prim.GetPath(),
        )

        print(
            "  schemas:",
            list(
                prim.GetAppliedSchemas()
            ),
        )

    if len(matches) != 1:
        raise RuntimeError(
            "Expected exactly one TetMesh carrying "
            "UsdPhysics.CollisionAPI under "
            f"{deformable_root.GetPath()}, found "
            f"{len(matches)}: "
            f"{[str(prim.GetPath()) for prim in matches]}"
        )

    collision_prim = matches[0]

    if collision_prim.HasAPI(
        "OmniPhysicsVolumeDeformableSimAPI"
    ):
        raise RuntimeError(
            f"{collision_prim.GetPath()} incorrectly carries "
            "OmniPhysicsVolumeDeformableSimAPI. "
            "The collision TetMesh must not also be the "
            "simulation TetMesh."
        )

    if not collision_prim.HasAPI(
        PhysxSchema.PhysxCollisionAPI
    ):
        raise RuntimeError(
            f"{collision_prim.GetPath()} carries "
            "UsdPhysics.CollisionAPI but does not carry "
            "PhysxCollisionAPI."
        )

    return collision_prim

def verify_collision_tetmesh(
    tetmesh_prim,
):
    """Verify the collision APIs and expected offsets."""

    if not tetmesh_prim.HasAPI(
        UsdPhysics.CollisionAPI
    ):
        raise RuntimeError(
            f"{tetmesh_prim.GetPath()} has no "
            "UsdPhysics.CollisionAPI."
        )

    if not tetmesh_prim.HasAPI(
        PhysxSchema.PhysxCollisionAPI
    ):
        raise RuntimeError(
            f"{tetmesh_prim.GetPath()} has no "
            "PhysxCollisionAPI."
        )

    collision_api = (
        PhysxSchema.PhysxCollisionAPI(
            tetmesh_prim
        )
    )

    rest_attr = (
        collision_api.GetRestOffsetAttr()
    )

    contact_attr = (
        collision_api.GetContactOffsetAttr()
    )

    if not rest_attr:
        raise RuntimeError(
            "Missing physxCollision:restOffset."
        )

    if not contact_attr:
        raise RuntimeError(
            "Missing physxCollision:contactOffset."
        )

    rest_offset = rest_attr.Get()
    contact_offset = contact_attr.Get()

    print("\n" + "=" * 100)
    print("COLLISION TETMESH VERIFICATION")
    print("=" * 100)
    print("path:", tetmesh_prim.GetPath())
    print("rest offset:", rest_offset)
    print("contact offset:", contact_offset)
    print(
        "rest authored:",
        rest_attr.HasAuthoredValueOpinion(),
    )
    print(
        "contact authored:",
        contact_attr.HasAuthoredValueOpinion(),
    )

    if not rest_attr.HasAuthoredValueOpinion():
        raise RuntimeError(
            "restOffset is not explicitly authored."
        )

    if not contact_attr.HasAuthoredValueOpinion():
        raise RuntimeError(
            "contactOffset is not explicitly authored."
        )

    if (
        abs(
            float(rest_offset)
            - EXPECTED_REST_OFFSET
        )
        > 1.0e-8
    ):
        raise RuntimeError(
            "Unexpected restOffset. "
            f"Expected {EXPECTED_REST_OFFSET}, "
            f"found {rest_offset}."
        )

    if (
        abs(
            float(contact_offset)
            - EXPECTED_CONTACT_OFFSET
        )
        > 1.0e-7
    ):
        raise RuntimeError(
            "Unexpected contactOffset. "
            f"Expected {EXPECTED_CONTACT_OFFSET}, "
            f"found {contact_offset}."
        )

def compare_runtime_nodes_to_usd_points(
    tetmesh_prim,
    usd_points_np,
    runtime_nodes_w,
):
    """
    Check whether USD point index i corresponds to runtime node index i.

    Global translation is removed before comparing positions because the
    deformable may already have moved under gravity when the runtime nodal
    state is read.

    This checks ordering and relative geometry, not absolute world position.
    """

    xformable = UsdGeom.Xformable(
        tetmesh_prim
    )

    local_to_world = (
        xformable.ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
    )

    usd_points_w = np.asarray(
        [
            local_to_world.Transform(
                Gf.Vec3d(
                    float(point[0]),
                    float(point[1]),
                    float(point[2]),
                )
            )
            for point in usd_points_np
        ],
        dtype=np.float64,
    )

    runtime_nodes_np = (
        runtime_nodes_w
        .detach()
        .cpu()
        .numpy()
        .astype(np.float64)
    )

    print("\n" + "=" * 100)
    print("USD POINT / RUNTIME NODE CORRESPONDENCE")
    print("=" * 100)

    print(
        "USD point count:",
        len(usd_points_w),
    )

    print(
        "runtime node count:",
        len(runtime_nodes_np),
    )

    if usd_points_w.shape != runtime_nodes_np.shape:
        raise RuntimeError(
            "USD points and runtime nodes have different shapes: "
            f"{usd_points_w.shape} versus "
            f"{runtime_nodes_np.shape}."
        )

    # ------------------------------------------------------------------
    # Absolute comparison
    # ------------------------------------------------------------------

    direct_difference = (
        runtime_nodes_np - usd_points_w
    )

    direct_error = np.linalg.norm(
        direct_difference,
        axis=1,
    )

    mean_translation = (
        direct_difference.mean(axis=0)
    )

    translation_deviation = (
        direct_difference
        - mean_translation[None, :]
    )

    translation_deviation_norm = (
        np.linalg.norm(
            translation_deviation,
            axis=1,
        )
    )

    print("\n[ABSOLUTE POSITION DIFFERENCE]")

    print(
        "direct error min/median/max:",
        float(direct_error.min()),
        float(np.median(direct_error)),
        float(direct_error.max()),
    )

    print(
        "mean runtime - USD translation:",
        mean_translation,
    )

    print(
        "translation deviation min/median/max:",
        float(
            translation_deviation_norm.min()
        ),
        float(
            np.median(
                translation_deviation_norm
            )
        ),
        float(
            translation_deviation_norm.max()
        ),
    )

    # ------------------------------------------------------------------
    # Translation-invariant comparison
    # ------------------------------------------------------------------

    usd_center = usd_points_w.mean(
        axis=0
    )

    runtime_center = runtime_nodes_np.mean(
        axis=0
    )

    usd_centered = (
        usd_points_w - usd_center
    )

    runtime_centered = (
        runtime_nodes_np
        - runtime_center
    )

    centered_difference = (
        runtime_centered - usd_centered
    )

    centered_error = np.linalg.norm(
        centered_difference,
        axis=1,
    )

    print("\n[TRANSLATION-INVARIANT SHAPE COMPARISON]")

    print(
        "USD center:",
        usd_center,
    )

    print(
        "runtime center:",
        runtime_center,
    )

    print(
        "runtime center - USD center:",
        runtime_center - usd_center,
    )

    print(
        "centered index error min/median/max:",
        float(centered_error.min()),
        float(np.median(centered_error)),
        float(centered_error.max()),
    )

    worst_ids = np.argsort(
        centered_error
    )[-10:][::-1]

    print(
        "10 largest centered-error node IDs:",
        worst_ids.tolist(),
    )

    for node_id in worst_ids:
        node_id = int(node_id)

        print(
            f"  node={node_id}, "
            f"centered_error="
            f"{centered_error[node_id]:.9e}, "
            f"usd_centered="
            f"{usd_centered[node_id]}, "
            f"runtime_centered="
            f"{runtime_centered[node_id]}"
        )

    # This tolerance tests relative geometry after removing translation.
    tolerance = 1.0e-5

    ordering_matches = bool(
        centered_error.max()
        <= tolerance
    )

    print(
        "\ncentered ordering matches within "
        f"{tolerance}:",
        ordering_matches,
    )

    if ordering_matches:
        print(
            "[PASS] USD point index i matches runtime node index i "
            "after removing global translation."
        )

    else:
        print(
            "[FAIL] Runtime relative node geometry does not match "
            "USD relative point geometry within the tolerance."
        )

    return {
        "ordering_matches":
            ordering_matches,

        "mean_translation":
            mean_translation,

        "direct_error":
            direct_error,

        "translation_deviation":
            translation_deviation_norm,

        "centered_error":
            centered_error,
    }
# -----------------------------------------------------------------------------
# Physics-material verification
# -----------------------------------------------------------------------------

def dump_physics_material_binding(
    deformable_root,
):
    """Print the material bound to the nested deformable root."""

    stage = sim_utils.get_current_stage()

    print("\n" + "=" * 100)
    print("PHYSICS MATERIAL BINDING")
    print("=" * 100)
    print("deformable root:", deformable_root.GetPath())

    relationship = deformable_root.GetRelationship(
        "material:binding:physics"
    )

    if not relationship:
        raise RuntimeError(
            f"{deformable_root.GetPath()} has no "
            "material:binding:physics relationship."
        )

    targets = relationship.GetTargets()

    print(
        "targets:",
        [str(path) for path in targets],
    )

    if len(targets) == 0:
        raise RuntimeError(
            "Physics-material binding has no target."
        )

    for target_path in targets:
        material_prim = stage.GetPrimAtPath(
            target_path
        )

        if not material_prim.IsValid():
            raise RuntimeError(
                f"Bound material does not exist: {target_path}"
            )

        print("\nmaterial:", target_path)
        print(
            "schemas:",
            list(material_prim.GetAppliedSchemas()),
        )

        for attribute in material_prim.GetAttributes():
            name = attribute.GetName()
            lower_name = name.lower()

            if (
                "density" in lower_name
                or "friction" in lower_name
                or "young" in lower_name
                or "poisson" in lower_name
                or "damping" in lower_name
                or "model" in lower_name
            ):
                print(
                    f"  {name} = {attribute.Get()}, "
                    f"authored="
                    f"{attribute.HasAuthoredValueOpinion()}"
                )


# -----------------------------------------------------------------------------
# TetMesh topology and initial quality
# -----------------------------------------------------------------------------

def read_tetmesh_topology(
    tetmesh_prim,
):
    """Read TetMesh points and tetrahedron indices."""

    tetmesh = UsdGeom.TetMesh(
        tetmesh_prim
    )

    points = (
        tetmesh.GetPointsAttr().Get()
    )

    tet_indices = (
        tetmesh.GetTetVertexIndicesAttr().Get()
    )

    if points is None or len(points) == 0:
        raise RuntimeError(
            f"No points on {tetmesh_prim.GetPath()}."
        )

    if (
        tet_indices is None
        or len(tet_indices) == 0
    ):
        raise RuntimeError(
            f"No tetrahedra on {tetmesh_prim.GetPath()}."
        )

    points_np = np.asarray(
        points,
        dtype=np.float64,
    )

    tets_np = np.asarray(
        tet_indices,
        dtype=np.int64,
    ).reshape(-1, 4)

    if np.min(tets_np) < 0:
        raise RuntimeError(
            "TetMesh contains a negative vertex index."
        )

    if np.max(tets_np) >= len(points_np):
        raise RuntimeError(
            "TetMesh references a vertex outside "
            "the point array."
        )

    duplicate_index_mask = np.asarray(
        [
            len(set(tet.tolist())) != 4
            for tet in tets_np
        ],
        dtype=bool,
    )

    print("\n" + "=" * 100)
    print("AUTHORED TETMESH TOPOLOGY")
    print("=" * 100)
    print("vertices:", len(points_np))
    print("tetrahedra:", len(tets_np))
    print(
        "duplicate-index tetrahedra:",
        int(duplicate_index_mask.sum()),
    )

    if np.any(duplicate_index_mask):
        bad_ids = np.flatnonzero(
            duplicate_index_mask
        )[:20]

        raise RuntimeError(
            "TetMesh contains tetrahedra with repeated "
            "vertex indices. First IDs: "
            f"{bad_ids.tolist()}"
        )

    return points_np, tets_np


def analyze_initial_tet_quality(
    points_np,
    tets_np,
):
    """
    Calculate initial tetrahedron quality.

    normalized_volume is a sliver proxy:
        absolute_volume / rms_edge^3

    mean_ratio_quality approaches:
        1 for a regular tetrahedron;
        0 for a flat/collapsed tetrahedron.
    """

    edge_pairs = (
        (0, 1),
        (0, 2),
        (0, 3),
        (1, 2),
        (1, 3),
        (2, 3),
    )

    edge_lengths = np.stack(
        [
            np.linalg.norm(
                points_np[tets_np[:, first]]
                - points_np[tets_np[:, second]],
                axis=1,
            )
            for first, second in edge_pairs
        ],
        axis=1,
    )

    p0 = points_np[tets_np[:, 0]]
    p1 = points_np[tets_np[:, 1]]
    p2 = points_np[tets_np[:, 2]]
    p3 = points_np[tets_np[:, 3]]

    signed_volume = (
        np.einsum(
            "ij,ij->i",
            p1 - p0,
            np.cross(
                p2 - p0,
                p3 - p0,
            ),
        )
        / 6.0
    )

    absolute_volume = np.abs(
        signed_volume
    )

    rms_edge = np.sqrt(
        np.mean(
            edge_lengths**2,
            axis=1,
        )
    )

    normalized_volume = (
        absolute_volume
        / np.maximum(
            rms_edge**3,
            1.0e-30,
        )
    )

    min_edge = edge_lengths.min(
        axis=1
    )

    max_edge = edge_lengths.max(
        axis=1
    )

    edge_ratio = (
        max_edge
        / np.maximum(
            min_edge,
            1.0e-15,
        )
    )

    edge_squared_sum = np.sum(
        edge_lengths**2,
        axis=1,
    )

    mean_ratio_quality = (
        12.0
        * np.power(
            3.0 * absolute_volume,
            2.0 / 3.0,
        )
        / np.maximum(
            edge_squared_sum,
            1.0e-30,
        )
    )

    rest_centroids = (
        points_np[tets_np]
        .mean(axis=1)
    )

    worst_quality_ids = np.argsort(
        mean_ratio_quality
    )

    print("\n" + "=" * 100)
    print("INITIAL TETRAHEDRON QUALITY")
    print("=" * 100)

    print("tetrahedra:", len(tets_np))

    print(
        "absolute volume min/median/max:",
        float(absolute_volume.min()),
        float(np.median(absolute_volume)),
        float(absolute_volume.max()),
    )

    print(
        "normalized volume min:",
        float(normalized_volume.min()),
    )

    print(
        "normalized volume 1st percentile:",
        float(
            np.percentile(
                normalized_volume,
                1,
            )
        ),
    )

    print(
        "normalized volume median:",
        float(np.median(normalized_volume)),
    )

    print(
        "edge ratio median:",
        float(np.median(edge_ratio)),
    )

    print(
        "edge ratio 99th percentile:",
        float(
            np.percentile(
                edge_ratio,
                99,
            )
        ),
    )

    print(
        "edge ratio maximum:",
        float(edge_ratio.max()),
    )

    print(
        "mean-ratio quality min:",
        float(mean_ratio_quality.min()),
    )

    print(
        "mean-ratio quality 1st percentile:",
        float(
            np.percentile(
                mean_ratio_quality,
                1,
            )
        ),
    )

    print(
        "mean-ratio quality median:",
        float(
            np.median(
                mean_ratio_quality
            )
        ),
    )

    print(
        "50 worst mean-ratio tet IDs:",
        worst_quality_ids[:50].tolist(),
    )

    return {
        "signed_volume": signed_volume,
        "absolute_volume": absolute_volume,
        "normalized_volume":
            normalized_volume,
        "edge_ratio": edge_ratio,
        "mean_ratio_quality":
            mean_ratio_quality,
        "rest_centroids": rest_centroids,
        "worst_quality_ids":
            worst_quality_ids,
    }


# -----------------------------------------------------------------------------
# Runtime volume calculations
# -----------------------------------------------------------------------------

def signed_tet_volumes(
    points,
    tets,
):
    """Calculate signed volume for every tetrahedron."""

    p0 = points[tets[:, 0]]
    p1 = points[tets[:, 1]]
    p2 = points[tets[:, 2]]
    p3 = points[tets[:, 3]]

    return (
        torch.sum(
            (p1 - p0)
            * torch.cross(
                p2 - p0,
                p3 - p0,
                dim=-1,
            ),
            dim=-1,
        )
        / 6.0
    )


def capture_rest_state(
    cube,
    tets,
):
    """Capture the post-reset nodal configuration."""

    nodal_pos = (
        cube.data.nodal_pos_w.torch[0]
        .detach()
        .clone()
    )

    if not torch.isfinite(
        nodal_pos
    ).all():
        raise RuntimeError(
            "Initial nodal positions contain NaN or Inf."
        )

    rest_volumes = signed_tet_volumes(
        nodal_pos,
        tets,
    )

    if not torch.isfinite(
        rest_volumes
    ).all():
        raise RuntimeError(
            "Initial tetrahedron volumes contain NaN or Inf."
        )

    bbox_min = nodal_pos.amin(dim=0)
    bbox_max = nodal_pos.amax(dim=0)
    extent = bbox_max - bbox_min

    rest_tet_centroids = (
        nodal_pos[tets]
        .mean(dim=1)
    )

    print("\n" + "=" * 100)
    print("POST-RESET REST STATE")
    print("=" * 100)

    print("node count:", nodal_pos.shape[0])
    print(
        "tetrahedron count:",
        tets.shape[0],
    )

    print(
        "bbox min:",
        bbox_min.cpu().numpy(),
    )

    print(
        "bbox max:",
        bbox_max.cpu().numpy(),
    )

    print(
        "extent:",
        extent.cpu().numpy(),
    )

    print(
        "ground clearance:",
        float(
            bbox_min[2].cpu().item()
            - args_cli.ground_z
        ),
    )

    print(
        "minimum absolute tet volume:",
        float(
            rest_volumes.abs().min().cpu()
        ),
    )

    print(
        "median absolute tet volume:",
        float(
            rest_volumes.abs().median().cpu()
        ),
    )

    print(
        "positive signed volumes:",
        int(
            (rest_volumes > 0.0)
            .sum()
            .cpu()
        ),
    )

    print(
        "negative signed volumes:",
        int(
            (rest_volumes < 0.0)
            .sum()
            .cpu()
        ),
    )

    print(
        "near-zero volumes:",
        int(
            (
                rest_volumes.abs()
                < 1.0e-12
            )
            .sum()
            .cpu()
        ),
    )

    return {
        "nodal_pos": nodal_pos,
        "volumes": rest_volumes,
        "bbox_min": bbox_min,
        "bbox_max": bbox_max,
        "extent": extent,
        "tet_centroids":
            rest_tet_centroids,
    }


# -----------------------------------------------------------------------------
# Spatial classification
# -----------------------------------------------------------------------------

def classify_tet_centroid(
    centroid,
    bbox_min,
    bbox_max,
    ground_z,
    surface_band_fraction,
):
    """
    Classify a tetrahedron centroid relative to the initial cube bounds.

    Multiple classifications can apply, for example:
        bottom+x_min+y_max

    If no surface band is matched, the classification is:
        interior
    """

    centroid = np.asarray(
        centroid,
        dtype=np.float64,
    )

    bbox_min = np.asarray(
        bbox_min,
        dtype=np.float64,
    )

    bbox_max = np.asarray(
        bbox_max,
        dtype=np.float64,
    )

    extent = bbox_max - bbox_min

    safe_extent = np.maximum(
        extent,
        1.0e-12,
    )

    normalized = (
        centroid - bbox_min
    ) / safe_extent

    labels = []

    fraction = float(
        surface_band_fraction
    )

    if normalized[2] <= fraction:
        labels.append("bottom")

    if normalized[2] >= 1.0 - fraction:
        labels.append("top")

    if normalized[0] <= fraction:
        labels.append("x_min")

    if normalized[0] >= 1.0 - fraction:
        labels.append("x_max")

    if normalized[1] <= fraction:
        labels.append("y_min")

    if normalized[1] >= 1.0 - fraction:
        labels.append("y_max")

    if not labels:
        labels.append("interior")

    distance_to_ground = float(
        centroid[2] - ground_z
    )

    distance_to_bottom = float(
        centroid[2] - bbox_min[2]
    )

    distance_to_top = float(
        bbox_max[2] - centroid[2]
    )

    distance_to_nearest_side = float(
        min(
            centroid[0] - bbox_min[0],
            bbox_max[0] - centroid[0],
            centroid[1] - bbox_min[1],
            bbox_max[1] - centroid[1],
        )
    )

    return {
        "region": "+".join(labels),
        "normalized": normalized,
        "distance_to_ground":
            distance_to_ground,
        "distance_to_bottom":
            distance_to_bottom,
        "distance_to_top":
            distance_to_top,
        "distance_to_nearest_side":
            distance_to_nearest_side,
    }


def summarize_inverted_regions(
    rows,
):
    """Print region counts for the first inversion."""

    counts = {}

    bottom_count = 0
    top_count = 0
    side_count = 0
    interior_count = 0

    for row in rows:
        region = row["rest_region"]

        counts[region] = (
            counts.get(region, 0)
            + 1
        )

        region_tokens = set(
            region.split("+")
        )

        if "bottom" in region_tokens:
            bottom_count += 1

        if "top" in region_tokens:
            top_count += 1

        if region_tokens.intersection(
            {
                "x_min",
                "x_max",
                "y_min",
                "y_max",
            }
        ):
            side_count += 1

        if "interior" in region_tokens:
            interior_count += 1

    print("\n[INVERTED REGION SUMMARY]")
    print("total inverted:", len(rows))
    print("near bottom:", bottom_count)
    print("near top:", top_count)
    print("near any side:", side_count)
    print("interior:", interior_count)

    print("\n[Exact region combinations]")

    for region, count in sorted(
        counts.items(),
        key=lambda item: (
            -item[1],
            item[0],
        ),
    ):
        print(
            f"  {region}: {count}"
        )


def write_inversion_csv(
    rows,
    output_path,
):
    """Write first-inversion tetrahedron information."""

    if not rows:
        return

    output_path = Path(
        output_path
    )

    with output_path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=list(
                rows[0].keys()
            ),
        )

        writer.writeheader()
        writer.writerows(rows)

    print(
        "\nFirst-inversion CSV:",
        output_path.resolve(),
    )


# -----------------------------------------------------------------------------
# First-inversion report
# -----------------------------------------------------------------------------

def build_first_inversion_report(
    step,
    sim_dt,
    current_pos,
    current_volumes,
    sign_changes,
    tets,
    tets_np,
    rest_state,
    initial_quality,
):
    """
    Build and print a spatial report for the first detected inversion.
    """

    inverted_ids_t = torch.nonzero(
        sign_changes,
        as_tuple=False,
    ).flatten()

    inverted_ids = (
        inverted_ids_t
        .cpu()
        .numpy()
        .astype(np.int64)
    )

    current_abs = (
        current_volumes.abs()
        .cpu()
        .numpy()
    )

    rest_abs = (
        rest_state["volumes"]
        .abs()
        .cpu()
        .numpy()
    )

    volume_ratio = (
        current_abs
        / np.maximum(
            rest_abs,
            1.0e-30,
        )
    )

    current_tet_centroids = (
        current_pos[tets]
        .mean(dim=1)
        .cpu()
        .numpy()
    )

    rest_tet_centroids = (
        rest_state["tet_centroids"]
        .cpu()
        .numpy()
    )

    rest_bbox_min = (
        rest_state["bbox_min"]
        .cpu()
        .numpy()
    )

    rest_bbox_max = (
        rest_state["bbox_max"]
        .cpu()
        .numpy()
    )

    current_bbox_min = (
        current_pos.amin(dim=0)
        .cpu()
        .numpy()
    )

    current_bbox_max = (
        current_pos.amax(dim=0)
        .cpu()
        .numpy()
    )

    current_extent = (
        current_bbox_max
        - current_bbox_min
    )

    rest_extent = (
        rest_bbox_max
        - rest_bbox_min
    )

    extent_ratio = (
        current_extent
        / np.maximum(
            rest_extent,
            1.0e-12,
        )
    )

    worst_count = max(
        50,
        int(
            args_cli.worst_quality_fraction
            * len(tets_np)
        ),
    )

    worst_count = min(
        worst_count,
        len(tets_np),
    )

    worst_initial_set = set(
        initial_quality[
            "worst_quality_ids"
        ][:worst_count].tolist()
    )

    overlap_ids = [
        int(tet_id)
        for tet_id in inverted_ids
        if int(tet_id)
        in worst_initial_set
    ]

    rows = []

    for tet_id in inverted_ids:
        tet_id = int(tet_id)

        rest_centroid = (
            rest_tet_centroids[tet_id]
        )

        current_centroid = (
            current_tet_centroids[tet_id]
        )

        rest_location = (
            classify_tet_centroid(
                centroid=rest_centroid,
                bbox_min=rest_bbox_min,
                bbox_max=rest_bbox_max,
                ground_z=args_cli.ground_z,
                surface_band_fraction=(
                    args_cli.surface_band_fraction
                ),
            )
        )

        current_location = (
            classify_tet_centroid(
                centroid=current_centroid,
                bbox_min=current_bbox_min,
                bbox_max=current_bbox_max,
                ground_z=args_cli.ground_z,
                surface_band_fraction=(
                    args_cli.surface_band_fraction
                ),
            )
        )

        tet_vertex_ids = (
            tets_np[tet_id]
        )

        row = {
            "step": int(step),
            "time_s": float(
                step * sim_dt
            ),
            "tet_id": tet_id,

            "vertex_0":
                int(tet_vertex_ids[0]),
            "vertex_1":
                int(tet_vertex_ids[1]),
            "vertex_2":
                int(tet_vertex_ids[2]),
            "vertex_3":
                int(tet_vertex_ids[3]),

            "rest_centroid_x":
                float(rest_centroid[0]),
            "rest_centroid_y":
                float(rest_centroid[1]),
            "rest_centroid_z":
                float(rest_centroid[2]),

            "current_centroid_x":
                float(current_centroid[0]),
            "current_centroid_y":
                float(current_centroid[1]),
            "current_centroid_z":
                float(current_centroid[2]),

            "rest_normalized_x":
                float(
                    rest_location[
                        "normalized"
                    ][0]
                ),
            "rest_normalized_y":
                float(
                    rest_location[
                        "normalized"
                    ][1]
                ),
            "rest_normalized_z":
                float(
                    rest_location[
                        "normalized"
                    ][2]
                ),

            "rest_region":
                rest_location["region"],

            "rest_distance_to_ground":
                rest_location[
                    "distance_to_ground"
                ],

            "rest_distance_to_bottom":
                rest_location[
                    "distance_to_bottom"
                ],

            "rest_distance_to_top":
                rest_location[
                    "distance_to_top"
                ],

            "rest_distance_to_nearest_side":
                rest_location[
                    "distance_to_nearest_side"
                ],

            "current_region":
                current_location["region"],

            "initial_signed_volume":
                float(
                    initial_quality[
                        "signed_volume"
                    ][tet_id]
                ),

            "initial_absolute_volume":
                float(
                    initial_quality[
                        "absolute_volume"
                    ][tet_id]
                ),

            "initial_normalized_volume":
                float(
                    initial_quality[
                        "normalized_volume"
                    ][tet_id]
                ),

            "initial_edge_ratio":
                float(
                    initial_quality[
                        "edge_ratio"
                    ][tet_id]
                ),

            "initial_mean_ratio_quality":
                float(
                    initial_quality[
                        "mean_ratio_quality"
                    ][tet_id]
                ),

            "current_signed_volume":
                float(
                    current_volumes[
                        tet_id
                    ]
                    .cpu()
                    .item()
                ),

            "current_volume_ratio":
                float(
                    volume_ratio[
                        tet_id
                    ]
                ),

            "in_worst_initial_quality_set":
                bool(
                    tet_id
                    in worst_initial_set
                ),
        }

        rows.append(row)

    rows.sort(
        key=lambda row: (
            row["rest_centroid_z"],
            row[
                "initial_mean_ratio_quality"
            ],
        )
    )

    print("\n" + "!" * 100)
    print("FIRST TETRAHEDRON INVERSION")
    print("!" * 100)

    print("step:", step)
    print(
        "time:",
        step * sim_dt,
    )

    print(
        "inverted count:",
        len(inverted_ids),
    )

    print(
        "worst-quality comparison set size:",
        worst_count,
    )

    print(
        "overlap with worst initial-quality set:",
        len(overlap_ids),
    )

    print(
        "overlap IDs:",
        overlap_ids[
            : args_cli.max_inverted_to_print
        ],
    )

    print("\n[Current object geometry]")
    print("bbox min:", current_bbox_min)
    print("bbox max:", current_bbox_max)
    print("extent:", current_extent)
    print(
        "extent/rest extent:",
        extent_ratio,
    )
    print(
        "floor gap:",
        float(
            current_bbox_min[2]
            - args_cli.ground_z
        ),
    )

    summarize_inverted_regions(
        rows
    )

    rest_centroid_z = np.asarray(
        [
            row["rest_centroid_z"]
            for row in rows
        ],
        dtype=np.float64,
    )

    normalized_z = np.asarray(
        [
            row["rest_normalized_z"]
            for row in rows
        ],
        dtype=np.float64,
    )

    print("\n[REST-SPACE INVERTED CENTROID Z]")
    print(
        "world z min/median/max:",
        float(rest_centroid_z.min()),
        float(np.median(rest_centroid_z)),
        float(rest_centroid_z.max()),
    )

    print(
        "normalized z min/median/max:",
        float(normalized_z.min()),
        float(np.median(normalized_z)),
        float(normalized_z.max()),
    )

    print(
        "\n[INVERTED TETRAHEDRA, "
        "SORTED FROM LOWER TO HIGHER REST CENTROID]"
    )

    for row in rows[
        : args_cli.max_inverted_to_print
    ]:
        print(
            f"  tet={row['tet_id']:5d} "
            f"region={row['rest_region']:<24s} "
            f"rest_centroid=("
            f"{row['rest_centroid_x']:+.6f}, "
            f"{row['rest_centroid_y']:+.6f}, "
            f"{row['rest_centroid_z']:+.6f}) "
            f"normalized_z="
            f"{row['rest_normalized_z']:.4f} "
            f"ground_distance="
            f"{row['rest_distance_to_ground']:.6f} "
            f"mean_ratio="
            f"{row['initial_mean_ratio_quality']:.6e} "
            f"norm_volume="
            f"{row['initial_normalized_volume']:.6e} "
            f"volume_ratio="
            f"{row['current_volume_ratio']:.6e} "
            f"worst_set="
            f"{row['in_worst_initial_quality_set']}"
        )

    write_inversion_csv(
        rows=rows,
        output_path=args_cli.inversion_csv,
    )

    return {
        "step": int(step),
        "time": float(
            step * sim_dt
        ),
        "inverted_count":
            len(inverted_ids),
        "overlap_count":
            len(overlap_ids),
        "rows": rows,
        "extent_ratio":
            extent_ratio,
        "floor_gap": float(
            current_bbox_min[2]
            - args_cli.ground_z
        ),
    }


# -----------------------------------------------------------------------------
# Periodic healthy-state report
# -----------------------------------------------------------------------------

def report_current_state(
    cube,
    tets,
    rest_state,
    step,
):
    """
    Print a compact report before any inversion has been found.

    World translation is not treated as deformation.
    """

    current_pos = (
        cube.data.nodal_pos_w.torch[0]
        .detach()
    )

    current_volumes = (
        signed_tet_volumes(
            current_pos,
            tets,
        )
    )

    current_center = (
        current_pos.mean(dim=0)
    )

    rest_center = (
        rest_state["nodal_pos"]
        .mean(dim=0)
    )

    current_centered = (
        current_pos - current_center
    )

    rest_centered = (
        rest_state["nodal_pos"]
        - rest_center
    )

    shape_displacement = (
        current_centered
        - rest_centered
    )

    shape_displacement_norm = (
        torch.linalg.norm(
            shape_displacement,
            dim=-1,
        )
    )

    bbox_min = current_pos.amin(
        dim=0
    )

    bbox_max = current_pos.amax(
        dim=0
    )

    extent = bbox_max - bbox_min

    extent_ratio = (
        extent
        / torch.clamp(
            rest_state["extent"],
            min=1.0e-12,
        )
    )

    volume_ratio = (
        current_volumes.abs()
        / torch.clamp(
            rest_state["volumes"].abs(),
            min=1.0e-18,
        )
    )

    sign_changes = (
        (
            torch.sign(
                rest_state["volumes"]
            )
            != 0.0
        )
        & (
            torch.sign(
                current_volumes
            )
            != 0.0
        )
        & (
            torch.sign(
                rest_state["volumes"]
            )
            != torch.sign(
                current_volumes
            )
        )
    )

    print("\n" + "-" * 100)
    print(
        f"CONTACT STABILITY REPORT: step {step}"
    )
    print("-" * 100)

    print(
        "center:",
        current_center.cpu().numpy(),
    )

    print(
        "bbox min:",
        bbox_min.cpu().numpy(),
    )

    print(
        "bbox max:",
        bbox_max.cpu().numpy(),
    )

    print(
        "extent:",
        extent.cpu().numpy(),
    )

    print(
        "extent/rest extent:",
        extent_ratio.cpu().numpy(),
    )

    print(
        "floor gap:",
        float(
            bbox_min[2].cpu().item()
            - args_cli.ground_z
        ),
    )

    print(
        "mean centroid-relative displacement:",
        float(
            shape_displacement_norm
            .mean()
            .cpu()
        ),
    )

    print(
        "maximum centroid-relative displacement:",
        float(
            shape_displacement_norm
            .max()
            .cpu()
        ),
    )

    print(
        "minimum volume ratio:",
        float(
            volume_ratio.min().cpu()
        ),
    )

    print(
        "median volume ratio:",
        float(
            volume_ratio.median().cpu()
        ),
    )

    print(
        "maximum volume ratio:",
        float(
            volume_ratio.max().cpu()
        ),
    )

    print(
        "sign changes:",
        int(
            sign_changes.sum().cpu()
        ),
    )


# -----------------------------------------------------------------------------
# Scene
# -----------------------------------------------------------------------------

def design_scene():
    """Create ground, light and one custom deformable object."""

    ground_cfg = sim_utils.GroundPlaneCfg()

    ground_cfg.func(
        "/World/defaultGroundPlane",
        ground_cfg,
    )

    light_cfg = sim_utils.DomeLightCfg(
        intensity=3000.0,
        color=(0.8, 0.8, 0.8),
    )

    light_cfg.func(
        "/World/Light",
        light_cfg,
    )

    cube_cfg = DeformableObjectCfg(
        prim_path=CUBE_WRAPPER_PATH,

        spawn=sim_utils.UsdFileCfg(
            usd_path=args_cli.usd_path,
            scale=(1.0, 1.0, 1.0),

            # Material only.
            #
            # Do not add deformable_props here because the
            # imported USD already contains the nested
            # OmniPhysicsDeformableBodyAPI root.
            physics_material=(
                PhysxDeformableBodyMaterialCfg(
                    density=21.5,
                    poissons_ratio=0.25,
                    youngs_modulus=9.5e3,

                    static_friction=1.8,
                    dynamic_friction=1.2,

                    elasticity_damping=0.05,
                )
            ),

            visual_material=(
                sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(
                        0.8,
                        0.1,
                        0.1,
                    ),
                )
            ),
        ),

        init_state=(
            DeformableObjectCfg.InitialStateCfg(
                pos=(
                    0.0,
                    0.0,
                    args_cli.start_z,
                ),
            )
        ),

        debug_vis=True,
    )

    return DeformableObject(
        cfg=cube_cfg
    )


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    physics_cfg = PhysxCfg(
        solver_type=1,
    )

    sim_cfg = sim_utils.SimulationCfg(
        dt=float(args_cli.dt),
        device=args_cli.device,
        physics=physics_cfg,
    )

    sim = SimulationContext(
        sim_cfg
    )

    sim.set_camera_view(
        eye=[
            -1.1,
            -1.1,
            0.9,
        ],
        target=[
            0.0,
            0.0,
            args_cli.start_z,
        ],
    )

    cube = design_scene()

    stage = sim_utils.get_current_stage()

    deformable_root = find_single_deformable_root(
        stage,
        CUBE_WRAPPER_PATH,
    )

    # This confirms that the expected hexahedral two-mesh asset is loaded.
    if not deformable_root.HasAPI(
        "PhysxAutoDeformableHexahedralMeshAPI"
    ):
        raise RuntimeError(
            "The imported deformable root does not carry "
            "PhysxAutoDeformableHexahedralMeshAPI. "
            "This is not the expected two-mesh asset."
        )

    simulation_tetmesh_prim = (
        find_single_simulation_tetmesh(
            deformable_root
        )
    )

    collision_tetmesh_prim = (
        find_single_collision_tetmesh(
            deformable_root
        )
    )

    print("\n" + "=" * 100)
    print("TWO-MESH ROLE VERIFICATION")
    print("=" * 100)

    print(
        "simulation TetMesh:",
        simulation_tetmesh_prim.GetPath(),
    )

    print(
        "simulation schemas:",
        list(
            simulation_tetmesh_prim.GetAppliedSchemas()
        ),
    )

    print(
        "collision TetMesh:",
        collision_tetmesh_prim.GetPath(),
    )

    print(
        "collision schemas:",
        list(
            collision_tetmesh_prim.GetAppliedSchemas()
        ),
    )

    if simulation_tetmesh_prim == collision_tetmesh_prim:
        raise RuntimeError(
            "Simulation and collision roles resolve to the "
            "same TetMesh. Expected two distinct meshes."
        )

    if str(
        simulation_tetmesh_prim.GetPath()
    ) != (
        "/World/Cube/dice_superquadric/sim_mesh"
    ):
        raise RuntimeError(
            "Unexpected simulation TetMesh path: "
            f"{simulation_tetmesh_prim.GetPath()}"
        )

    if str(
        collision_tetmesh_prim.GetPath()
    ) != (
        "/World/Cube/dice_superquadric/coll_mesh"
    ):
        raise RuntimeError(
            "Unexpected collision TetMesh path: "
            f"{collision_tetmesh_prim.GetPath()}"
        )

    if simulation_tetmesh_prim.HasAPI(
        UsdPhysics.CollisionAPI
    ):
        raise RuntimeError(
            "The simulation TetMesh unexpectedly carries "
            "UsdPhysics.CollisionAPI."
        )

    if simulation_tetmesh_prim.HasAPI(
        PhysxSchema.PhysxCollisionAPI
    ):
        raise RuntimeError(
            "The simulation TetMesh unexpectedly carries "
            "PhysxCollisionAPI."
        )

    if not collision_tetmesh_prim.HasAPI(
        UsdPhysics.CollisionAPI
    ):
        raise RuntimeError(
            "The collision TetMesh does not carry "
            "UsdPhysics.CollisionAPI."
        )

    if collision_tetmesh_prim.HasAPI(
        "OmniPhysicsVolumeDeformableSimAPI"
    ):
        raise RuntimeError(
            "The collision TetMesh unexpectedly carries "
            "OmniPhysicsVolumeDeformableSimAPI."
        )

    # Collision settings belong to coll_mesh.
    verify_collision_tetmesh(
        collision_tetmesh_prim
    )

    # FEM topology and quality belong to sim_mesh.
    points_np, tets_np = (
        read_tetmesh_topology(
            simulation_tetmesh_prim
        )
    )

    if len(points_np) != 1346:
        raise RuntimeError(
            "Unexpected simulation vertex count. "
            f"Expected 1346, found {len(points_np)}."
        )

    if len(tets_np) != 6090:
        raise RuntimeError(
            "Unexpected simulation tetrahedron count. "
            f"Expected 6090, found {len(tets_np)}."
        )

    initial_quality = (
        analyze_initial_tet_quality(
            points_np,
            tets_np,
        )
    )

    pre_reset_points_np, pre_reset_tets_np = (
        read_tetmesh_topology(
            simulation_tetmesh_prim
        )
    )

    # -------------------------------------------------------------------------
    # Initialize PhysX. The auto-deformable workflow may update generated data.
    # -------------------------------------------------------------------------

    sim.reset()

    print("\n[INFO] Simulation reset completed.")

    cube.update(0.0)

    # -------------------------------------------------------------------------
    # Reacquire and reread the TetMesh after PhysX initialization.
    # -------------------------------------------------------------------------

    stage = sim_utils.get_current_stage()

    stage = sim_utils.get_current_stage()

    deformable_root = find_single_deformable_root(
        stage,
        CUBE_WRAPPER_PATH,
    )

    simulation_tetmesh_prim = (
        find_single_simulation_tetmesh(
            deformable_root
        )
    )

    collision_tetmesh_prim = (
        find_single_collision_tetmesh(
            deformable_root
        )
    )

    post_reset_points_np, post_reset_tets_np = (
        read_tetmesh_topology(
            simulation_tetmesh_prim
        )
    )
    print("\n" + "=" * 100)
    print("PRE-RESET / POST-RESET TETMESH COMPARISON")
    print("=" * 100)

   
    print(
        "post-reset vertices:",
        len(post_reset_points_np),
    )

    print(
        "pre-reset tetrahedra:",
        len(pre_reset_tets_np),
    )

    print(
        "post-reset tetrahedra:",
        len(post_reset_tets_np),
    )

    same_point_shape = (
        pre_reset_points_np.shape
        == post_reset_points_np.shape
    )

    same_tet_shape = (
        pre_reset_tets_np.shape
        == post_reset_tets_np.shape
    )

    same_tet_indices = (
        same_tet_shape
        and np.array_equal(
            pre_reset_tets_np,
            post_reset_tets_np,
        )
    )

    print(
        "same point-array shape:",
        same_point_shape,
    )

    print(
        "same tet-array shape:",
        same_tet_shape,
    )

    print(
        "exactly identical tet indices:",
        same_tet_indices,
    )

    if same_point_shape:
        point_delta = np.linalg.norm(
            pre_reset_points_np
            - post_reset_points_np,
            axis=1,
        )

        print(
            "USD point delta min/median/max:",
            float(point_delta.min()),
            float(np.median(point_delta)),
            float(point_delta.max()),
        )

    else:
        print(
            "USD point comparison skipped because "
            "the array shapes differ."
        )

    if not same_tet_indices:
        print(
            "\n[IMPORTANT] TetMesh topology changed during reset."
        )
        print(
            "The runtime diagnostic will use the post-reset indices."
        )

    # Always use the post-reset topology from here onward.
    points_np = post_reset_points_np
    tets_np = post_reset_tets_np

    initial_quality = analyze_initial_tet_quality(
        points_np,
        tets_np,
    )

    tets = torch.as_tensor(
        tets_np,
        dtype=torch.long,
        device=cube.data.nodal_pos_w.torch.device,
    )


    cube.update(0.0)

    tets = torch.as_tensor(
        tets_np,
        dtype=torch.long,
        device=(
            cube.data.nodal_pos_w
            .torch.device
        ),
    )
    node_correspondence = (
        compare_runtime_nodes_to_usd_points(
            tetmesh_prim=(
                simulation_tetmesh_prim
            ),
            usd_points_np=points_np,
            runtime_nodes_w=(
                cube.data.nodal_pos_w.torch[0]
            ),
        )
    )

    if not node_correspondence[
        "ordering_matches"
    ]:
        raise RuntimeError(
            "USD TetMesh point ordering does not match "
            "runtime nodal ordering after removing global "
            "translation."
        )
    rest_state = capture_rest_state(
        cube,
        tets,
    )

    first_inversion_report = None

    for step in range(
        1,
        args_cli.steps + 1,
    ):
        sim.step()

        cube.update(
            sim.get_physics_dt()
        )

        current_pos = (
            cube.data.nodal_pos_w
            .torch[0]
            .detach()
        )

        if not torch.isfinite(
            current_pos
        ).all():
            raise RuntimeError(
                "Non-finite nodal positions at "
                f"step {step}."
            )

        current_volumes = (
            signed_tet_volumes(
                current_pos,
                tets,
            )
        )

        if not torch.isfinite(
            current_volumes
        ).all():
            raise RuntimeError(
                "Non-finite tetrahedron volumes at "
                f"step {step}."
            )

        rest_sign = torch.sign(
            rest_state["volumes"]
        )

        current_sign = torch.sign(
            current_volumes
        )

        sign_changes = (
            (rest_sign != 0.0)
            & (current_sign != 0.0)
            & (rest_sign != current_sign)
        )

        num_inverted = int(
            sign_changes.sum().cpu()
        )

        # -------------------------------------------------------------
        # Stop immediately after the first detected inversion.
        # -------------------------------------------------------------
        if num_inverted > 0:
            first_inversion_report = (
                build_first_inversion_report(
                    step=step,
                    sim_dt=(
                        sim.get_physics_dt()
                    ),
                    current_pos=current_pos,
                    current_volumes=(
                        current_volumes
                    ),
                    sign_changes=(
                        sign_changes
                    ),
                    tets=tets,
                    tets_np=tets_np,
                    rest_state=rest_state,
                    initial_quality=(
                        initial_quality
                    ),
                )
            )

            break

        if (
            step == 1
            or step
            % args_cli.print_every
            == 0
        ):
            report_current_state(
                cube=cube,
                tets=tets,
                rest_state=rest_state,
                step=step,
            )

    print("\n" + "=" * 100)
    print("FINAL RESULT")
    print("=" * 100)

    if first_inversion_report is None:
        print(
            "[PASS] No tetrahedron inversion was "
            f"detected during {args_cli.steps} steps."
        )

    else:
        print(
            "[FAIL] First tetrahedron inversion "
            f"detected at step "
            f"{first_inversion_report['step']}."
        )

        print(
            "time:",
            first_inversion_report["time"],
        )

        print(
            "inverted tetrahedra:",
            first_inversion_report[
                "inverted_count"
            ],
        )

        print(
            "overlap with worst initial-quality set:",
            first_inversion_report[
                "overlap_count"
            ],
        )

        print(
            "extent ratio:",
            first_inversion_report[
                "extent_ratio"
            ],
        )

        print(
            "floor gap:",
            first_inversion_report[
                "floor_gap"
            ],
        )

        print(
            "\nUse the CSV rest_region, "
            "rest_normalized_z and "
            "rest_distance_to_ground columns to determine "
            "whether the first failing tetrahedra are near "
            "the floor, side walls, corners or interior."
        )


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    import traceback

    try:
        main()

    except BaseException as exc:
        print(
            "\n[ERROR]",
            repr(exc),
            flush=True,
        )

        traceback.print_exc()
        raise

    finally:
        simulation_app.close()

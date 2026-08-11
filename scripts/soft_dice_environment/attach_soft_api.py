# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""
Create and validate a simulation-ready two-TetMesh PhysX volume deformable.

Expected input hierarchy
------------------------

/root
    /dice_superquadric
        /dice_superquadric
            Mesh                    # clean source/render surface mesh

Generated output hierarchy
--------------------------

/root
    /dice_superquadric
        Xform
        OmniPhysicsDeformableBodyAPI
        PhysxAutoDeformableBodyAPI
        PhysxAutoDeformableHexahedralMeshAPI

        /dice_superquadric
            Mesh                    # source/render mesh

        /sim_mesh
            TetMesh
            OmniPhysicsVolumeDeformableSimAPI
            no PhysicsCollisionAPI

        /coll_mesh
            TetMesh
            PhysicsCollisionAPI
            PhysxCollisionAPI
            no OmniPhysicsVolumeDeformableSimAPI

Important invariant
-------------------

The volume deformable must have:

- exactly one deformable-body root;
- exactly one simulation TetMesh;
- exactly one collider;
- the simulation and collision TetMeshes must be different prims;
- only coll_mesh may carry PhysicsCollisionAPI.

Run using the Isaac Sim / Isaac Lab Python environment.
"""

from isaacsim import SimulationApp


# -----------------------------------------------------------------------------
# Launch Isaac Sim before importing Omniverse / PhysX modules
# -----------------------------------------------------------------------------

simulation_app = SimulationApp(
    {
        "headless": True,
    }
)


# -----------------------------------------------------------------------------
# Imports available after SimulationApp startup
# -----------------------------------------------------------------------------

import os

import numpy as np
import omni.usd

from omni.physx import get_physx_cooking_interface
from omni.physx.scripts import deformableUtils

from pxr import PhysxSchema
from pxr import Usd
from pxr import UsdGeom
from pxr import UsdPhysics


# -----------------------------------------------------------------------------
# Input / output configuration
# -----------------------------------------------------------------------------

INPUT_USD = (
    "/home/chiara/git/IsaacLab_v3_test/scripts/soft_dice_environment/models/dice_superquadric_clean_simplified.usd"
)

OUTPUT_USD = (
    "/home/chiara/git/IsaacLab_v3_test/"
    "dice_superquadric_reduced_resolution.usd"
)


# Existing custom-asset hierarchy.

ROOT_PATH = (
    "/root/dice_superquadric"
)

SOURCE_MESH_PATH = (
    "/root/dice_superquadric/dice_superquadric"
)


# Generated two-mesh hierarchy.

SIM_TETMESH_PATH = (
    "/root/dice_superquadric/sim_mesh"
)

COLLISION_TETMESH_PATH = (
    "/root/dice_superquadric/coll_mesh"
)


# Collision offsets are authored only on coll_mesh.

REST_OFFSET = 0.001
CONTACT_OFFSET = 0.003


# Whether PhysX should simplify/remesh the cooking source.

COOKING_SOURCE_SIMPLIFICATION_ENABLED = True


# -----------------------------------------------------------------------------
# Generic helpers
# -----------------------------------------------------------------------------

def print_section(title: str):
    print("\n" + "=" * 100)
    print(title)
    print("=" * 100)


def require_valid_prim(
    stage,
    prim_path: str,
):
    """Return a valid USD prim or raise a clear error."""

    prim = stage.GetPrimAtPath(
        prim_path
    )

    if not prim.IsValid():
        raise RuntimeError(
            f"Required prim was not found: {prim_path}"
        )

    return prim


def has_named_api(
    prim,
    schema_name: str,
):
    """
    Check a codeless API by schema name.

    This is used for Omni Physics / PhysX codeless schemas whose generated
    Python schema class may not be exposed consistently.
    """

    return prim.HasAPI(
        schema_name
    )


def dump_attributes(
    prim,
    name_filters=(),
):
    """Print selected attributes from a prim."""

    print(
        "  attributes matching:",
        list(name_filters),
    )

    for attribute in prim.GetAttributes():
        name = attribute.GetName()
        lower_name = name.lower()

        if name_filters and not any(
            token.lower() in lower_name
            for token in name_filters
        ):
            continue

        try:
            value = attribute.Get()
        except Exception:
            value = "<unreadable>"

        if (
            hasattr(value, "__len__")
            and not isinstance(
                value,
                (str, bytes),
            )
        ):
            try:
                value = f"<array size={len(value)}>"
            except Exception:
                pass

        print(
            f"    {name} = {value}, "
            f"authored="
            f"{attribute.HasAuthoredValueOpinion()}"
        )


# -----------------------------------------------------------------------------
# Source-mesh validation
# -----------------------------------------------------------------------------

def require_source_mesh(stage):
    """Validate the source/render surface mesh."""

    source_prim = require_valid_prim(
        stage,
        SOURCE_MESH_PATH,
    )

    if source_prim.GetTypeName() != "Mesh":
        raise RuntimeError(
            f"Expected a Mesh at {SOURCE_MESH_PATH}, "
            f"but found {source_prim.GetTypeName()!r}."
        )

    source_mesh = UsdGeom.Mesh(
        source_prim
    )

    points = (
        source_mesh
        .GetPointsAttr()
        .Get()
    )

    face_counts = (
        source_mesh
        .GetFaceVertexCountsAttr()
        .Get()
    )

    face_indices = (
        source_mesh
        .GetFaceVertexIndicesAttr()
        .Get()
    )

    if points is None or len(points) == 0:
        raise RuntimeError(
            f"The source mesh has no points: "
            f"{SOURCE_MESH_PATH}"
        )

    if (
        face_counts is None
        or len(face_counts) == 0
    ):
        raise RuntimeError(
            f"The source mesh has no faces: "
            f"{SOURCE_MESH_PATH}"
        )

    if (
        face_indices is None
        or len(face_indices) == 0
    ):
        raise RuntimeError(
            f"The source mesh has no face indices: "
            f"{SOURCE_MESH_PATH}"
        )

    non_triangle_faces = [
        index
        for index, count in enumerate(
            face_counts
        )
        if int(count) != 3
    ]

    print_section(
        "SOURCE MESH"
    )

    print(
        "  path:",
        SOURCE_MESH_PATH,
    )

    print(
        "  vertices:",
        len(points),
    )

    print(
        "  faces:",
        len(face_counts),
    )

    print(
        "  face-vertex indices:",
        len(face_indices),
    )

    print(
        "  non-triangle faces:",
        len(non_triangle_faces),
    )

    if non_triangle_faces:
        print(
            "  first non-triangle face IDs:",
            non_triangle_faces[:20],
        )

        raise RuntimeError(
            "The source mesh contains non-triangle faces. "
            "Triangulate it before deformable cooking."
        )

    return source_prim


# -----------------------------------------------------------------------------
# Generated TetMesh accessors
# -----------------------------------------------------------------------------

def get_simulation_tetmesh(stage):
    """Get and validate the generated simulation TetMesh."""

    prim = require_valid_prim(
        stage,
        SIM_TETMESH_PATH,
    )

    if prim.GetTypeName() != "TetMesh":
        raise RuntimeError(
            f"Expected a TetMesh at {SIM_TETMESH_PATH}, "
            f"but found {prim.GetTypeName()!r}."
        )

    return prim


def get_collision_tetmesh(stage):
    """Get and validate the generated collision TetMesh."""

    prim = require_valid_prim(
        stage,
        COLLISION_TETMESH_PATH,
    )

    if prim.GetTypeName() != "TetMesh":
        raise RuntimeError(
            f"Expected a TetMesh at "
            f"{COLLISION_TETMESH_PATH}, "
            f"but found {prim.GetTypeName()!r}."
        )

    return prim


# -----------------------------------------------------------------------------
# Mesh-role validation
# -----------------------------------------------------------------------------

def inspect_deformable_mesh_roles(
    stage,
    label: str,
    require_populated_meshes: bool,
):
    """
    Verify the deformable hierarchy roles.

    Required:
        exactly one OmniPhysicsDeformableBodyAPI prim;
        exactly one OmniPhysicsVolumeDeformableSimAPI prim;
        exactly one PhysicsCollisionAPI prim.

    For this two-mesh asset:
        the simulation prim and collision prim must be different.
    """

    root_prim = require_valid_prim(
        stage,
        ROOT_PATH,
    )

    deformable_roots = []
    simulation_prims = []
    collision_prims = []

    print_section(label)

    for prim in Usd.PrimRange(
        root_prim
    ):
        path = str(
            prim.GetPath()
        )

        is_deformable_root = (
            has_named_api(
                prim,
                "OmniPhysicsDeformableBodyAPI",
            )
        )

        is_simulation = (
            has_named_api(
                prim,
                "OmniPhysicsVolumeDeformableSimAPI",
            )
        )

        is_collision = (
            prim.HasAPI(
                UsdPhysics.CollisionAPI
            )
        )

        has_physx_collision = (
            prim.HasAPI(
                PhysxSchema.PhysxCollisionAPI
            )
        )

        if is_deformable_root:
            deformable_roots.append(
                prim
            )

        if is_simulation:
            simulation_prims.append(
                prim
            )

        if is_collision:
            collision_prims.append(
                prim
            )

        print(
            path,
            f"type={prim.GetTypeName()!r}",
        )

        print(
            "  deformable root:",
            is_deformable_root,
        )

        print(
            "  volume simulation:",
            is_simulation,
        )

        print(
            "  PhysicsCollisionAPI:",
            is_collision,
        )

        print(
            "  PhysxCollisionAPI:",
            has_physx_collision,
        )

        print(
            "  schemas:",
            list(
                prim.GetAppliedSchemas()
            ),
        )

    print("\n[ROLE COUNTS]")

    print(
        "  deformable-body roots:",
        len(deformable_roots),
    )

    print(
        "  simulation meshes:",
        len(simulation_prims),
    )

    print(
        "  collision prims:",
        len(collision_prims),
    )

    print(
        "  deformable paths:",
        [
            str(prim.GetPath())
            for prim in deformable_roots
        ],
    )

    print(
        "  simulation paths:",
        [
            str(prim.GetPath())
            for prim in simulation_prims
        ],
    )

    print(
        "  collision paths:",
        [
            str(prim.GetPath())
            for prim in collision_prims
        ],
    )

    if len(deformable_roots) != 1:
        raise RuntimeError(
            "Expected exactly one "
            "OmniPhysicsDeformableBodyAPI prim, "
            f"found {len(deformable_roots)}."
        )

    if len(simulation_prims) != 1:
        raise RuntimeError(
            "Expected exactly one "
            "OmniPhysicsVolumeDeformableSimAPI prim, "
            f"found {len(simulation_prims)}."
        )

    if len(collision_prims) != 1:
        raise RuntimeError(
            "The volume deformable must have exactly one "
            "UsdPhysics.CollisionAPI collider, but found "
            f"{len(collision_prims)}."
        )

    simulation_prim = (
        simulation_prims[0]
    )

    collision_prim = (
        collision_prims[0]
    )

    if str(
        simulation_prim.GetPath()
    ) != SIM_TETMESH_PATH:
        raise RuntimeError(
            "The simulation API is on the wrong prim. "
            f"Expected {SIM_TETMESH_PATH}, found "
            f"{simulation_prim.GetPath()}."
        )

    if str(
        collision_prim.GetPath()
    ) != COLLISION_TETMESH_PATH:
        raise RuntimeError(
            "The collision API is on the wrong prim. "
            f"Expected {COLLISION_TETMESH_PATH}, found "
            f"{collision_prim.GetPath()}."
        )

    if simulation_prim == collision_prim:
        raise RuntimeError(
            "The hierarchy is still using one shared "
            "simulation/collision TetMesh. This script "
            "expects two distinct TetMesh prims."
        )

    if simulation_prim.HasAPI(
        UsdPhysics.CollisionAPI
    ):
        raise RuntimeError(
            f"{SIM_TETMESH_PATH} incorrectly carries "
            "UsdPhysics.CollisionAPI. This would create "
            "an additional collider."
        )

    if simulation_prim.HasAPI(
        PhysxSchema.PhysxCollisionAPI
    ):
        raise RuntimeError(
            f"{SIM_TETMESH_PATH} incorrectly carries "
            "PhysxCollisionAPI."
        )

    if has_named_api(
        collision_prim,
        "OmniPhysicsVolumeDeformableSimAPI",
    ):
        raise RuntimeError(
            f"{COLLISION_TETMESH_PATH} incorrectly carries "
            "OmniPhysicsVolumeDeformableSimAPI."
        )

    if not collision_prim.HasAPI(
        UsdPhysics.CollisionAPI
    ):
        raise RuntimeError(
            f"{COLLISION_TETMESH_PATH} is not marked "
            "as the collider."
        )

    if require_populated_meshes:
        for prim, role in (
            (
                simulation_prim,
                "simulation",
            ),
            (
                collision_prim,
                "collision",
            ),
        ):
            tetmesh = UsdGeom.TetMesh(
                prim
            )

            points = (
                tetmesh
                .GetPointsAttr()
                .Get()
            )

            tets = (
                tetmesh
                .GetTetVertexIndicesAttr()
                .Get()
            )

            if (
                points is None
                or len(points) == 0
            ):
                raise RuntimeError(
                    f"The {role} TetMesh has no points: "
                    f"{prim.GetPath()}."
                )

            if (
                tets is None
                or len(tets) == 0
            ):
                raise RuntimeError(
                    f"The {role} TetMesh has no tetrahedra: "
                    f"{prim.GetPath()}."
                )

    print(
        "\n[PASS] Separate simulation/collision "
        "TetMesh roles are structurally valid."
    )

    return {
        "root": deformable_roots[0],
        "simulation": simulation_prim,
        "collision": collision_prim,
    }


# -----------------------------------------------------------------------------
# TetMesh topology validation
# -----------------------------------------------------------------------------

def read_tetmesh_arrays(
    tetmesh_prim,
):
    """Read and validate one TetMesh's points and tetrahedra."""

    tetmesh = UsdGeom.TetMesh(
        tetmesh_prim
    )

    points = (
        tetmesh
        .GetPointsAttr()
        .Get()
    )

    tet_indices = (
        tetmesh
        .GetTetVertexIndicesAttr()
        .Get()
    )

    if (
        points is None
        or len(points) == 0
    ):
        raise RuntimeError(
            f"No points on {tetmesh_prim.GetPath()}."
        )

    if (
        tet_indices is None
        or len(tet_indices) == 0
    ):
        raise RuntimeError(
            f"No tetrahedra on "
            f"{tetmesh_prim.GetPath()}."
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
            f"{tetmesh_prim.GetPath()} contains "
            "negative vertex indices."
        )

    if np.max(tets_np) >= len(
        points_np
    ):
        raise RuntimeError(
            f"{tetmesh_prim.GetPath()} references a "
            "vertex outside its point array."
        )

    duplicate_vertex_mask = (
        np.asarray(
            [
                len(set(tet.tolist()))
                != 4
                for tet in tets_np
            ],
            dtype=bool,
        )
    )

    if np.any(
        duplicate_vertex_mask
    ):
        bad_ids = np.flatnonzero(
            duplicate_vertex_mask
        )[:20]

        raise RuntimeError(
            f"{tetmesh_prim.GetPath()} contains "
            "tetrahedra with repeated vertex indices. "
            f"First bad IDs: {bad_ids.tolist()}."
        )

    return points_np, tets_np


def compute_tetmesh_quality(
    points_np,
    tets_np,
):
    """
    Compute static rest-state tetrahedron quality metrics.

    mean_ratio_quality:
        1 for a regular tetrahedron;
        approaches 0 for a flat or degenerate tetrahedron.

    normalized_volume:
        absolute volume divided by RMS edge length cubed.
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
                points_np[
                    tets_np[:, first]
                ]
                - points_np[
                    tets_np[:, second]
                ],
                axis=1,
            )
            for first, second
            in edge_pairs
        ],
        axis=1,
    )

    p0 = points_np[
        tets_np[:, 0]
    ]

    p1 = points_np[
        tets_np[:, 1]
    ]

    p2 = points_np[
        tets_np[:, 2]
    ]

    p3 = points_np[
        tets_np[:, 3]
    ]

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

    return {
        "signed_volume":
            signed_volume,

        "absolute_volume":
            absolute_volume,

        "edge_ratio":
            edge_ratio,

        "normalized_volume":
            normalized_volume,

        "mean_ratio_quality":
            mean_ratio_quality,
    }


def dump_tetmesh_stats(
    label: str,
    tetmesh_prim,
    include_quality: bool,
):
    """Print schema, topology and optional quality information."""

    print_section(label)

    print(
        "  path:",
        tetmesh_prim.GetPath(),
    )

    print(
        "  type:",
        tetmesh_prim.GetTypeName(),
    )

    print(
        "  applied schemas:",
        list(
            tetmesh_prim.GetAppliedSchemas()
        ),
    )

    print(
        "  volume simulation API:",
        has_named_api(
            tetmesh_prim,
            "OmniPhysicsVolumeDeformableSimAPI",
        ),
    )

    print(
        "  PhysicsCollisionAPI:",
        tetmesh_prim.HasAPI(
            UsdPhysics.CollisionAPI
        ),
    )

    print(
        "  PhysxCollisionAPI:",
        tetmesh_prim.HasAPI(
            PhysxSchema.PhysxCollisionAPI
        ),
    )

    points_np, tets_np = (
        read_tetmesh_arrays(
            tetmesh_prim
        )
    )

    print(
        "  vertices:",
        len(points_np),
    )

    print(
        "  tetrahedra:",
        len(tets_np),
    )

    print(
        "  bbox min:",
        points_np.min(axis=0),
    )

    print(
        "  bbox max:",
        points_np.max(axis=0),
    )

    print(
        "  extent:",
        points_np.max(axis=0)
        - points_np.min(axis=0),
    )

    if not include_quality:
        return

    quality = compute_tetmesh_quality(
        points_np,
        tets_np,
    )

    signed_volume = quality[
        "signed_volume"
    ]

    absolute_volume = quality[
        "absolute_volume"
    ]

    edge_ratio = quality[
        "edge_ratio"
    ]

    normalized_volume = quality[
        "normalized_volume"
    ]

    mean_ratio_quality = quality[
        "mean_ratio_quality"
    ]

    worst_ids = np.argsort(
        mean_ratio_quality
    )

    print("\n  [VOLUME]")

    print(
        "    min / median / max:",
        float(
            absolute_volume.min()
        ),
        float(
            np.median(
                absolute_volume
            )
        ),
        float(
            absolute_volume.max()
        ),
    )

    print(
        "    positive signed:",
        int(
            np.sum(
                signed_volume > 0.0
            )
        ),
    )

    print(
        "    negative signed:",
        int(
            np.sum(
                signed_volume < 0.0
            )
        ),
    )

    print(
        "    near zero:",
        int(
            np.sum(
                absolute_volume
                < 1.0e-12
            )
        ),
    )

    print("\n  [EDGE RATIO]")

    print(
        "    median:",
        float(
            np.median(
                edge_ratio
            )
        ),
    )

    print(
        "    99th percentile:",
        float(
            np.percentile(
                edge_ratio,
                99,
            )
        ),
    )

    print(
        "    maximum:",
        float(
            edge_ratio.max()
        ),
    )

    print(
        "\n  [NORMALIZED VOLUME]"
    )

    print(
        "    minimum:",
        float(
            normalized_volume.min()
        ),
    )

    print(
        "    1st percentile:",
        float(
            np.percentile(
                normalized_volume,
                1,
            )
        ),
    )

    print(
        "    median:",
        float(
            np.median(
                normalized_volume
            )
        ),
    )

    print(
        "\n  [MEAN-RATIO QUALITY]"
    )

    print(
        "    minimum:",
        float(
            mean_ratio_quality.min()
        ),
    )

    print(
        "    1st percentile:",
        float(
            np.percentile(
                mean_ratio_quality,
                1,
            )
        ),
    )

    print(
        "    5th percentile:",
        float(
            np.percentile(
                mean_ratio_quality,
                5,
            )
        ),
    )

    print(
        "    median:",
        float(
            np.median(
                mean_ratio_quality
            )
        ),
    )

    print(
        "    below 0.05:",
        int(
            np.sum(
                mean_ratio_quality
                < 0.05
            )
        ),
    )

    print(
        "    below 0.10:",
        int(
            np.sum(
                mean_ratio_quality
                < 0.10
            )
        ),
    )

    print(
        "    50 worst tet IDs:",
        worst_ids[:50].tolist(),
    )


# -----------------------------------------------------------------------------
# Collision-offset authoring
# -----------------------------------------------------------------------------

def set_collision_offsets(
    collision_tetmesh_prim,
    rest_offset: float,
    contact_offset: float,
):
    """
    Apply PhysxCollisionAPI and author offsets only on coll_mesh.
    """

    if (
        contact_offset
        <= rest_offset
    ):
        raise ValueError(
            f"contact_offset ({contact_offset}) must "
            f"be greater than rest_offset "
            f"({rest_offset})."
        )

    expected_path = str(
        collision_tetmesh_prim.GetPath()
    )

    if (
        expected_path
        != COLLISION_TETMESH_PATH
    ):
        raise RuntimeError(
            "Refusing to author collision offsets on "
            f"{expected_path}. Expected "
            f"{COLLISION_TETMESH_PATH}."
        )

    if (
        collision_tetmesh_prim
        .GetTypeName()
        != "TetMesh"
    ):
        raise RuntimeError(
            f"Expected a TetMesh at "
            f"{COLLISION_TETMESH_PATH}."
        )

    if not collision_tetmesh_prim.HasAPI(
        UsdPhysics.CollisionAPI
    ):
        raise RuntimeError(
            f"{COLLISION_TETMESH_PATH} does not have "
            "UsdPhysics.CollisionAPI."
        )

    if has_named_api(
        collision_tetmesh_prim,
        "OmniPhysicsVolumeDeformableSimAPI",
    ):
        raise RuntimeError(
            "The collision TetMesh incorrectly carries "
            "the simulation API."
        )

    collision_api = (
        PhysxSchema
        .PhysxCollisionAPI
        .Apply(
            collision_tetmesh_prim
        )
    )

    if not collision_api:
        raise RuntimeError(
            "Could not apply PhysxCollisionAPI to "
            f"{COLLISION_TETMESH_PATH}."
        )

    rest_attr = (
        collision_api
        .CreateRestOffsetAttr()
    )

    contact_attr = (
        collision_api
        .CreateContactOffsetAttr()
    )

    if not rest_attr.Set(
        float(rest_offset)
    ):
        raise RuntimeError(
            "Could not set restOffset."
        )

    if not contact_attr.Set(
        float(contact_offset)
    ):
        raise RuntimeError(
            "Could not set contactOffset."
        )

    print_section(
        "COLLISION SETTINGS AUTHORED"
    )

    print(
        "  prim:",
        collision_tetmesh_prim.GetPath(),
    )

    print(
        "  rest offset:",
        rest_attr.Get(),
    )

    print(
        "  rest authored:",
        rest_attr.HasAuthoredValueOpinion(),
    )

    print(
        "  contact offset:",
        contact_attr.Get(),
    )

    print(
        "  contact authored:",
        contact_attr.HasAuthoredValueOpinion(),
    )


def verify_collision_offsets(
    collision_tetmesh_prim,
):
    """Verify offsets on the sole collision TetMesh."""

    if not collision_tetmesh_prim.HasAPI(
        UsdPhysics.CollisionAPI
    ):
        raise RuntimeError(
            f"{COLLISION_TETMESH_PATH} lost "
            "UsdPhysics.CollisionAPI."
        )

    if not collision_tetmesh_prim.HasAPI(
        PhysxSchema.PhysxCollisionAPI
    ):
        raise RuntimeError(
            f"{COLLISION_TETMESH_PATH} lost "
            "PhysxCollisionAPI."
        )

    collision_api = (
        PhysxSchema.PhysxCollisionAPI(
            collision_tetmesh_prim
        )
    )

    rest_attr = (
        collision_api
        .GetRestOffsetAttr()
    )

    contact_attr = (
        collision_api
        .GetContactOffsetAttr()
    )

    if not rest_attr:
        raise RuntimeError(
            "Collision TetMesh has no restOffset."
        )

    if not contact_attr:
        raise RuntimeError(
            "Collision TetMesh has no contactOffset."
        )

    actual_rest = float(
        rest_attr.Get()
    )

    actual_contact = float(
        contact_attr.Get()
    )

    print_section(
        "COLLISION OFFSET VERIFICATION"
    )

    print(
        "  prim:",
        collision_tetmesh_prim.GetPath(),
    )

    print(
        "  rest offset:",
        actual_rest,
    )

    print(
        "  contact offset:",
        actual_contact,
    )

    if not rest_attr.HasAuthoredValueOpinion():
        raise RuntimeError(
            "restOffset is not explicitly authored."
        )

    if not contact_attr.HasAuthoredValueOpinion():
        raise RuntimeError(
            "contactOffset is not explicitly authored."
        )

    if abs(
        actual_rest
        - REST_OFFSET
    ) > 1.0e-8:
        raise RuntimeError(
            "Unexpected restOffset: "
            f"expected {REST_OFFSET}, "
            f"found {actual_rest}."
        )

    if abs(
        actual_contact
        - CONTACT_OFFSET
    ) > 1.0e-7:
        raise RuntimeError(
            "Unexpected contactOffset: "
            f"expected {CONTACT_OFFSET}, "
            f"found {actual_contact}."
        )


# -----------------------------------------------------------------------------
# Root schema inspection
# -----------------------------------------------------------------------------

def verify_auto_generation_root(
    root_prim,
):
    """Verify the auto-generation schema setup on the root."""

    print_section(
        "AUTO-GENERATION ROOT"
    )

    print(
        "  path:",
        root_prim.GetPath(),
    )

    print(
        "  schemas:",
        list(
            root_prim.GetAppliedSchemas()
        ),
    )

    required_schemas = (
        "OmniPhysicsDeformableBodyAPI",
        "PhysxAutoDeformableBodyAPI",
        "PhysxAutoDeformableHexahedralMeshAPI",
    )

    for schema_name in required_schemas:
        present = has_named_api(
            root_prim,
            schema_name,
        )

        print(
            f"  {schema_name}:",
            present,
        )

        if not present:
            raise RuntimeError(
                f"{ROOT_PATH} does not have "
                f"{schema_name} after requesting "
                "hexahedral simulation-mesh generation."
            )

    dump_attributes(
        root_prim,
        name_filters=(
            "deformable",
            "hexa",
            "resolution",
            "simplification",
            "remesh",
            "target",
            "cooking",
        ),
    )


# -----------------------------------------------------------------------------
# Final validation
# -----------------------------------------------------------------------------

def validate_complete_asset(
    stage,
    label: str,
):
    """Run every structural and topology validation."""

    roles = inspect_deformable_mesh_roles(
        stage=stage,
        label=label,
        require_populated_meshes=True,
    )

    root_prim = roles["root"]
    simulation_prim = roles[
        "simulation"
    ]
    collision_prim = roles[
        "collision"
    ]

    verify_auto_generation_root(
        root_prim
    )

    verify_collision_offsets(
        collision_prim
    )

    dump_tetmesh_stats(
        label=(
            f"{label}: SIMULATION TETMESH"
        ),
        tetmesh_prim=simulation_prim,
        include_quality=True,
    )

    dump_tetmesh_stats(
        label=(
            f"{label}: COLLISION TETMESH"
        ),
        tetmesh_prim=collision_prim,
        include_quality=True,
    )

    return roles


# -----------------------------------------------------------------------------
# Main conversion pipeline
# -----------------------------------------------------------------------------

def main():
    context = omni.usd.get_context()

    print_section(
        "OPEN INPUT USD"
    )

    print(
        "Input:",
        INPUT_USD,
    )

    opened = context.open_stage(
        INPUT_USD
    )

    if opened is False:
        raise RuntimeError(
            f"Could not open input USD: "
            f"{INPUT_USD}"
        )

    stage = context.get_stage()

    if stage is None:
        raise RuntimeError(
            f"No USD stage was created for "
            f"{INPUT_USD}."
        )

    root_prim = require_valid_prim(
        stage,
        ROOT_PATH,
    )

    require_source_mesh(
        stage
    )

    print_section(
        "INPUT ROOT"
    )

    print(
        "  path:",
        root_prim.GetPath(),
    )

    print(
        "  type:",
        root_prim.GetTypeName(),
    )

    print(
        "  schemas:",
        list(
            root_prim.GetAppliedSchemas()
        ),
    )

    # ------------------------------------------------------------------
    # Ensure the clean input has no stale generated prims.
    # ------------------------------------------------------------------

    for generated_path in (
        SIM_TETMESH_PATH,
        COLLISION_TETMESH_PATH,
    ):
        prim = stage.GetPrimAtPath(
            generated_path
        )

        if prim.IsValid():
            raise RuntimeError(
                f"The input USD already contains "
                f"{generated_path}. Use the clean source "
                "USD or remove the stale generated prim."
            )

    # ------------------------------------------------------------------
    # Generate separate simulation and collision TetMeshes.
    # ------------------------------------------------------------------

    print_section(
        "CREATE TWO-MESH AUTO VOLUME DEFORMABLE HIERARCHY"
    )

    print(
        "  root:",
        ROOT_PATH,
    )

    print(
        "  source mesh:",
        SOURCE_MESH_PATH,
    )

    print(
        "  simulation TetMesh:",
        SIM_TETMESH_PATH,
    )

    print(
        "  collision TetMesh:",
        COLLISION_TETMESH_PATH,
    )

    print(
        "  simulation hex mesh enabled:",
        True,
    )

    print(
        "  source simplification enabled:",
        COOKING_SOURCE_SIMPLIFICATION_ENABLED,
    )

    created = (
        deformableUtils
        .create_auto_volume_deformable_hierarchy(
            stage,
            root_prim_path=ROOT_PATH,
            simulation_tetmesh_path=(
                SIM_TETMESH_PATH
            ),
            collision_tetmesh_path=(
                COLLISION_TETMESH_PATH
            ),
            cooking_src_mesh_path=(
                SOURCE_MESH_PATH
            ),
            simulation_hex_mesh_enabled=True,
            cooking_src_simplification_enabled=(
                COOKING_SOURCE_SIMPLIFICATION_ENABLED
            ),
        )
    )

    if not created:
        raise RuntimeError(
            "create_auto_volume_deformable_hierarchy() "
            "returned False."
        )

    SIMULATION_HEX_RESOLUTION = 8
    root_prim = require_valid_prim(
        stage,
        ROOT_PATH,
    )

    resolution_attr = root_prim.GetAttribute(
        "physxDeformableBody:resolution"
    )

    if not resolution_attr:
        raise RuntimeError(
            "Missing physxDeformableBody:resolution."
        )

    success = resolution_attr.Set(
        SIMULATION_HEX_RESOLUTION
    )

    if not success:
        raise RuntimeError(
            "Could not author physxDeformableBody:resolution."
        )

    verify_auto_generation_root(
        root_prim
    )

    pre_cook_roles = (
        inspect_deformable_mesh_roles(
            stage=stage,
            label=(
                "PRE-COOK TWO-MESH ROLE VALIDATION"
            ),
            require_populated_meshes=False,
        )
    )

    simulation_prim = (
        pre_cook_roles[
            "simulation"
        ]
    )

    collision_prim = (
        pre_cook_roles[
            "collision"
        ]
    )

    # Explicit final protection: do not let sim_mesh be a collider.

    if simulation_prim.HasAPI(
        UsdPhysics.CollisionAPI
    ):
        raise RuntimeError(
            f"{SIM_TETMESH_PATH} carries "
            "UsdPhysics.CollisionAPI before cooking. "
            "Refusing to continue because this would "
            "produce more than one collider."
        )

    # ------------------------------------------------------------------
    # Author collision settings only on coll_mesh.
    # ------------------------------------------------------------------

    set_collision_offsets(
        collision_tetmesh_prim=(
            collision_prim
        ),
        rest_offset=REST_OFFSET,
        contact_offset=CONTACT_OFFSET,
    )

    # Recheck after applying PhysxCollisionAPI.

    inspect_deformable_mesh_roles(
        stage=stage,
        label=(
            "PRE-COOK ROLE VALIDATION "
            "AFTER OFFSET AUTHORING"
        ),
        require_populated_meshes=False,
    )

    # ------------------------------------------------------------------
    # Cook synchronously.
    # ------------------------------------------------------------------

    print_section(
        "COOK AUTO DEFORMABLE BODY"
    )

    cooking_interface = (
        get_physx_cooking_interface()
    )

    cooking_interface.cook_auto_deformable_body(
        ROOT_PATH
    )

    print(
        "Cooking command completed."
    )

    # ------------------------------------------------------------------
    # Full post-cook validation.
    # ------------------------------------------------------------------

    post_cook_roles = (
        validate_complete_asset(
            stage=stage,
            label=(
                "POST-COOK ASSET VALIDATION"
            ),
        )
    )

    # Additional explicit checks for clarity.

    if post_cook_roles[
        "simulation"
    ].HasAPI(
        UsdPhysics.CollisionAPI
    ):
        raise RuntimeError(
            "Post-cook simulation TetMesh became a "
            "collider. Two colliders would be present."
        )

    if not post_cook_roles[
        "collision"
    ].HasAPI(
        UsdPhysics.CollisionAPI
    ):
        raise RuntimeError(
            "Post-cook collision TetMesh is no longer "
            "the collider."
        )

    # ------------------------------------------------------------------
    # Export.
    # ------------------------------------------------------------------

    print_section(
        "EXPORT SIMULATION-READY USD"
    )

    print(
        "Output:",
        OUTPUT_USD,
    )

    exported = stage.Export(
        OUTPUT_USD
    )

    if not exported:
        raise RuntimeError(
            f"Failed to export USD: "
            f"{OUTPUT_USD}"
        )

    if not os.path.isfile(
        OUTPUT_USD
    ):
        raise RuntimeError(
            "Stage.Export returned success, but the "
            f"output file does not exist: {OUTPUT_USD}"
        )

    print(
        "Export completed."
    )

    print(
        "Output file size:",
        os.path.getsize(
            OUTPUT_USD
        ),
        "bytes",
    )

    # ------------------------------------------------------------------
    # Reopen the exported file independently and validate it again.
    # ------------------------------------------------------------------

    print_section(
        "REOPEN EXPORTED USD FOR INDEPENDENT VALIDATION"
    )

    exported_stage = Usd.Stage.Open(
        OUTPUT_USD
    )

    if exported_stage is None:
        raise RuntimeError(
            f"Could not reopen exported USD: "
            f"{OUTPUT_USD}"
        )

    exported_roles = (
        validate_complete_asset(
            stage=exported_stage,
            label=(
                "REOPENED EXPORTED ASSET VALIDATION"
            ),
        )
    )

    # ------------------------------------------------------------------
    # Final hierarchy dump.
    # ------------------------------------------------------------------

    print_section(
        "FINAL EXPORTED USD HIERARCHY"
    )

    for prim in exported_stage.Traverse():
        path = str(
            prim.GetPath()
        )

        if not path.startswith(
            ROOT_PATH
        ):
            continue

        print(
            path,
            f"type={prim.GetTypeName()!r}",
        )

        print(
            "  schemas:",
            list(
                prim.GetAppliedSchemas()
            ),
        )

        print(
            "  volume simulation:",
            has_named_api(
                prim,
                (
                    "OmniPhysics"
                    "VolumeDeformableSimAPI"
                ),
            ),
        )

        print(
            "  collision:",
            prim.HasAPI(
                UsdPhysics.CollisionAPI
            ),
        )

        print(
            "  PhysX collision:",
            prim.HasAPI(
                PhysxSchema
                .PhysxCollisionAPI
            ),
        )

    # ------------------------------------------------------------------
    # Final explicit summary.
    # ------------------------------------------------------------------

    simulation_points, simulation_tets = (
        read_tetmesh_arrays(
            exported_roles[
                "simulation"
            ]
        )
    )

    collision_points, collision_tets = (
        read_tetmesh_arrays(
            exported_roles[
                "collision"
            ]
        )
    )

    print_section(
        "SUCCESS"
    )

    print(
        "Created:",
        OUTPUT_USD,
    )

    print(
        "Deformable root:",
        ROOT_PATH,
    )

    print(
        "Visual/source mesh:",
        SOURCE_MESH_PATH,
    )

    print(
        "Simulation TetMesh:",
        SIM_TETMESH_PATH,
    )

    print(
        "Simulation vertices:",
        len(simulation_points),
    )

    print(
        "Simulation tetrahedra:",
        len(simulation_tets),
    )

    print(
        "Simulation mesh is collider:",
        exported_roles[
            "simulation"
        ].HasAPI(
            UsdPhysics.CollisionAPI
        ),
    )

    print(
        "Collision TetMesh:",
        COLLISION_TETMESH_PATH,
    )

    print(
        "Collision vertices:",
        len(collision_points),
    )

    print(
        "Collision tetrahedra:",
        len(collision_tets),
    )

    print(
        "Collision mesh is collider:",
        exported_roles[
            "collision"
        ].HasAPI(
            UsdPhysics.CollisionAPI
        ),
    )

    print(
        "Number of collision prims:",
        1,
    )

    print(
        "Rest offset:",
        REST_OFFSET,
    )

    print(
        "Contact offset:",
        CONTACT_OFFSET,
    )

    print(
        "\n[VERIFIED STRUCTURE]"
    )

    print(
        "  one deformable root: True"
    )

    print(
        "  one simulation TetMesh: True"
    )

    print(
        "  one collision TetMesh: True"
    )

    print(
        "  one collider total: True"
    )

    print(
        "  simulation and collision meshes "
        "are distinct: True"
    )


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

try:
    main()

except Exception:
    import traceback

    print_section(
        "TWO-MESH DEFORMABLE CREATION FAILED"
    )

    traceback.print_exc()

    raise

finally:
    simulation_app.close()

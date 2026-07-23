#!/usr/bin/env python3

import argparse
from pathlib import Path

from pxr import Usd, UsdPhysics, UsdShade, Sdf


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Attach structural USD physics APIs to a custom mesh asset. "
            "This writes a new rigid-ready USD file."
        )
    )

    parser.add_argument(
        "--input_usd",
        type=str,
        default=  "/home/chiara/git/holosoma/src/holosoma_retargeting/holosoma_retargeting/models/dice/dice_supercube_rigid2.usd",
        help="Path to the original mesh USD, e.g. dice_supercube.usd.",
    )

    parser.add_argument(
        "--output_usd",
        type=str,
        default="/home/chiara/git/holosoma/src/holosoma_retargeting/holosoma_retargeting/models/dice/dice_supercube_rigid2.usd",
        help="Path to save the rigid-ready USD, e.g. dice_supercube_rigid.usd.",
    )

    parser.add_argument(
        "--root_prim",
        type=str,
        default=None,
        help=(
            "Root prim of the asset inside the USD. "
            "If omitted, the script uses the default prim, or the first top-level prim."
        ),
    )

    parser.add_argument(
        "--mass",
        type=float,
        default=0.75,
        help="Mass in kg to author on the rigid body root.",
    )

    parser.add_argument(
        "--collision_approximation",
        type=str,
        default="convexHull",
        choices=["convexHull", "convexDecomposition", "boundingCube", "boundingSphere"],
        help=(
            "Mesh collision approximation. For your dynamic rigid superquadric dice, "
            "start with convexHull."
        ),
    )

    return parser.parse_args()


def find_asset_root_prim(stage: Usd.Stage, root_prim_path: str | None) -> Usd.Prim:
    """
    Find the prim that should become the single rigid body actor.

    In your Isaac Lab scene, this asset will be instanced under:
        /World/envs/env_0/Cube

    But inside the USD file itself, the root might be:
        /Cube
        /dice_supercube
        /World/Cube
        etc.

    This function tries:
        1. user-provided --root_prim
        2. USD default prim
        3. first top-level prim
    """

    if root_prim_path is not None:
        root_prim = stage.GetPrimAtPath(root_prim_path)
        if not root_prim.IsValid():
            raise RuntimeError(f"Requested root prim does not exist: {root_prim_path}")
        return root_prim

    default_prim = stage.GetDefaultPrim()
    if default_prim and default_prim.IsValid():
        print(f"[Root] Using USD default prim: {default_prim.GetPath()}")
        return default_prim

    top_level_prims = [p for p in stage.GetPseudoRoot().GetChildren() if p.IsValid()]

    if len(top_level_prims) == 0:
        raise RuntimeError("No top-level prims found in USD.")

    if len(top_level_prims) > 1:
        print("[WARN] Multiple top-level prims found:")
        for prim in top_level_prims:
            print(f"  {prim.GetPath()}")
        print(f"[WARN] Using first top-level prim: {top_level_prims[0].GetPath()}")

    return top_level_prims[0]


def remove_nested_rigid_bodies(root_prim: Usd.Prim):
    """
    Ensure there is only one RigidBodyAPI: on the asset root.

    Isaac Lab RigidObjectCfg expects the object to correspond to one rigid actor.
    If mesh children also have RigidBodyAPI, that can create an unexpected hierarchy.
    """

    stage = root_prim.GetStage()
    root_path = root_prim.GetPath().pathString

    removed = 0

    for prim in stage.Traverse():
        prim_path = prim.GetPath().pathString

        if not prim_path.startswith(root_path):
            continue

        if prim == root_prim:
            continue

        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            try:
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                print(f"[Clean] Removed nested RigidBodyAPI from: {prim_path}")
                removed += 1
            except Exception as exc:
                print(f"[WARN] Could not remove nested RigidBodyAPI from {prim_path}: {exc}")

    if removed == 0:
        print("[Clean] No nested RigidBodyAPI found.")


def get_mesh_prims_under(root_prim: Usd.Prim) -> list[Usd.Prim]:
    """Collect all Mesh prims under the asset root."""

    stage = root_prim.GetStage()
    root_path = root_prim.GetPath().pathString

    mesh_prims = []

    for prim in stage.Traverse():
        prim_path = prim.GetPath().pathString

        if not prim_path.startswith(root_path):
            continue

        if prim.GetTypeName() == "Mesh":
            mesh_prims.append(prim)

    return mesh_prims


def create_default_physics_material(stage: Usd.Stage, root_path: str) -> UsdShade.Material:
    """
    Create a simple USD physics material.

    These are intentionally neutral placeholder values.
    You can override friction/restitution at runtime in your Isaac Lab script.
    """

    material_path = f"{root_path}/PhysicsMaterial"
    material = UsdShade.Material.Define(stage, Sdf.Path(material_path))
    material_prim = stage.GetPrimAtPath(material_path)

    material_api = UsdPhysics.MaterialAPI.Apply(material_prim)

    # Neutral defaults; tune at runtime later.
    material_api.CreateStaticFrictionAttr().Set(0.5)
    material_api.CreateDynamicFrictionAttr().Set(0.5)
    material_api.CreateRestitutionAttr().Set(0.0)

    print(f"[Material] Created physics material: {material_path}")

    return material


def main():
    args = parse_args()

    input_usd = Path(args.input_usd).expanduser().resolve()
    output_usd = Path(args.output_usd).expanduser().resolve()

    if not input_usd.exists():
        raise FileNotFoundError(f"Input USD does not exist: {input_usd}")

    print(f"[Open] {input_usd}")
    stage = Usd.Stage.Open(str(input_usd))

    if stage is None:
        raise RuntimeError(f"Could not open USD stage: {input_usd}")

    root_prim = find_asset_root_prim(stage, args.root_prim)
    root_path = root_prim.GetPath().pathString

    print(f"[Root] Asset root prim: {root_path}")

    # Make this root the default prim for cleaner referencing.
    stage.SetDefaultPrim(root_prim)

    # Avoid nested rigid actors.
    remove_nested_rigid_bodies(root_prim)

    # -------------------------------------------------------------------------
    # Root rigid body API
    # -------------------------------------------------------------------------
    print(f"[RigidBody] Applying RigidBodyAPI to root: {root_path}")

    rigid_api = UsdPhysics.RigidBodyAPI.Apply(root_prim)
    rigid_api.CreateRigidBodyEnabledAttr().Set(True)
    rigid_api.CreateKinematicEnabledAttr().Set(False)

    # -------------------------------------------------------------------------
    # Mass API
    # -------------------------------------------------------------------------
    print(f"[Mass] Applying MassAPI mass={args.mass} kg to root: {root_path}")

    mass_api = UsdPhysics.MassAPI.Apply(root_prim)
    mass_api.CreateMassAttr().Set(float(args.mass))

    # -------------------------------------------------------------------------
    # Physics material
    # -------------------------------------------------------------------------
    material = create_default_physics_material(stage, root_path)

    # -------------------------------------------------------------------------
    # Mesh colliders
    # -------------------------------------------------------------------------
    mesh_prims = get_mesh_prims_under(root_prim)

    if len(mesh_prims) == 0:
        raise RuntimeError(f"No Mesh prims found under root: {root_path}")

    print(f"[Collision] Found {len(mesh_prims)} Mesh prim(s).")

    for mesh_prim in mesh_prims:
        mesh_path = mesh_prim.GetPath().pathString
        print(f"  [Collider] {mesh_path}")

        # Enable mesh as collider.
        UsdPhysics.CollisionAPI.Apply(mesh_prim)

        # Author mesh collision approximation.
        mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
        mesh_collision_api.CreateApproximationAttr().Set(args.collision_approximation)

        print(
            f"    approximation = {args.collision_approximation}"
        )

        # Bind physics material to collider.
        binding_api = UsdShade.MaterialBindingAPI.Apply(mesh_prim)
        binding_api.Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )

    # -------------------------------------------------------------------------
    # Save new USD
    # -------------------------------------------------------------------------
    output_usd.parent.mkdir(parents=True, exist_ok=True)

    print(f"[Save] Exporting rigid-ready USD to: {output_usd}")
    stage.GetRootLayer().Export(str(output_usd))

    print("\nDone.")
    print("Use this in your Isaac Lab scene:")
    print(f'CUSTOM_DICE_USD = "{output_usd}"')


if __name__ == "__main__":
    main()
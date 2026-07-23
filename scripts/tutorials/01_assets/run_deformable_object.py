# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates how to work with the deformable object and interact with it.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p scripts/tutorials/01_assets/run_deformable_object.py

"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on interacting with a deformable object.")
parser.add_argument("--backend", type=str, default="physx", choices=["physx", "newton"], help="Physics backend.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# demos should open Kit visualizer by default
parser.set_defaults(visualizer=["kit"])
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.assets import DeformableObject, DeformableObjectCfg
import omni.usd

from pxr import PhysxSchema, UsdPhysics, UsdGeom, Usd 

def get_visual_mesh_world_bounds(stage, env_id: int):
    """Return world-space bounds of the rendered deformable surface mesh."""

    mesh_path = f"/World/env_{env_id}/Cube/geometry/mesh"
    mesh_prim = stage.GetPrimAtPath(mesh_path)

    if not mesh_prim.IsValid():
        raise RuntimeError(f"Visual mesh not found: {mesh_path}")

    mesh = UsdGeom.Mesh(mesh_prim)
    points = mesh.GetPointsAttr().Get()

    if points is None or len(points) == 0:
        raise RuntimeError(f"No visual points found on: {mesh_path}")

    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    local_to_world = xform_cache.GetLocalToWorldTransform(mesh_prim)

    points_w = [
        local_to_world.Transform(point)
        for point in points
    ]

    min_x = min(point[0] for point in points_w)
    min_y = min(point[1] for point in points_w)
    min_z = min(point[2] for point in points_w)

    max_x = max(point[0] for point in points_w)
    max_y = max(point[1] for point in points_w)
    max_z = max(point[2] for point in points_w)

    return (
        (min_x, min_y, min_z),
        (max_x, max_y, max_z),
    )

def set_deformable_tetmesh_collision_offsets(
    rest_offset: float = 0.0,
    contact_offset: float = 0.001,
):
    """Author collision offsets on the actual collision-bearing TetMesh."""

    if contact_offset <= rest_offset:
        raise ValueError(
            f"contact_offset ({contact_offset}) must be greater than "
            f"rest_offset ({rest_offset})."
        )

    stage = omni.usd.get_context().get_stage()

    for env_id in range(4):
        tetmesh_path = f"/World/env_{env_id}/Cube/sim_mesh"
        tetmesh_prim = stage.GetPrimAtPath(tetmesh_path)

        if not tetmesh_prim.IsValid():
            raise RuntimeError(
                f"Expected generated TetMesh was not found: {tetmesh_path}"
            )

        if tetmesh_prim.GetTypeName() != "TetMesh":
            raise RuntimeError(
                f"Expected TetMesh at {tetmesh_path}, "
                f"found {tetmesh_prim.GetTypeName()!r}"
            )

        # Apply PhysX collision settings to the prim that actually carries
        # PhysicsCollisionAPI.
        collision_api = PhysxSchema.PhysxCollisionAPI.Apply(tetmesh_prim)

        if not collision_api:
            raise RuntimeError(
                f"Could not apply PhysxCollisionAPI to {tetmesh_path}"
            )

        collision_api.CreateRestOffsetAttr().Set(rest_offset)
        collision_api.CreateContactOffsetAttr().Set(contact_offset)

        actual_rest_offset = collision_api.GetRestOffsetAttr().Get()
        actual_contact_offset = collision_api.GetContactOffsetAttr().Get()

        print(f"\n[TETMESH COLLISION SETTINGS] {tetmesh_path}")
        print(f"  schemas: {list(tetmesh_prim.GetAppliedSchemas())}")
        print(f"  rest offset: {actual_rest_offset}")
        print(f"  contact offset: {actual_contact_offset}")

def _value_to_string(value):
    """Make large USD values readable without dumping entire mesh arrays."""
    if value is None:
        return "None"

    try:
        length = len(value)
    except TypeError:
        return repr(value)

    if length > 16:
        return f"<{type(value).__name__}, length={length}>"

    return repr(value)


def dump_deformable_usd(stage, label):
    """Read-only dump of all prims below every procedural Cube."""

    print("\n" + "=" * 100)
    print(f"[USD DEFORMABLE DUMP] {label}")
    print("=" * 100)

    for prim in stage.Traverse():
        path = str(prim.GetPath())

        if not any(path.startswith(f"/World/env_{i}/Cube") for i in range(4)):
            continue

        print(f"\nPRIM: {path}")
        print(f"  type name: {prim.GetTypeName()!r}")
        print(f"  active: {prim.IsActive()}")
        print(f"  valid: {prim.IsValid()}")
        print(f"  instance: {prim.IsInstance()}")
        print(f"  instance proxy: {prim.IsInstanceProxy()}")
        print(f"  prototype: {prim.IsPrototype()}")
        print(f"  applied schemas: {list(prim.GetAppliedSchemas())}")

        # Print every relationship because simulation/collision meshes may be
        # connected to the deformable body through relationships.
        relationships = prim.GetRelationships()

        if relationships:
            print("  relationships:")

            for relationship in relationships:
                targets = relationship.GetTargets()
                print(
                    f"    {relationship.GetName()}: "
                    f"{[str(target) for target in targets]}"
                )

        # Inspect PhysxCollisionAPI specifically.
        has_physx_collision_api = prim.HasAPI(
            PhysxSchema.PhysxCollisionAPI
        )

        has_usd_collision_api = prim.HasAPI(
            UsdPhysics.CollisionAPI
        )

        print(f"  has UsdPhysics.CollisionAPI: {has_usd_collision_api}")
        print(f"  has PhysxCollisionAPI: {has_physx_collision_api}")

        if has_physx_collision_api:
            collision_api = PhysxSchema.PhysxCollisionAPI(prim)

            rest_attr = collision_api.GetRestOffsetAttr()
            contact_attr = collision_api.GetContactOffsetAttr()

            print("  PhysxCollisionAPI values:")

            for name, attr in (
                ("restOffset", rest_attr),
                ("contactOffset", contact_attr),
            ):
                if attr:
                    print(
                        f"    {name}:"
                        f" value={attr.Get()},"
                        f" authored={attr.HasAuthoredValueOpinion()},"
                        f" type={attr.GetTypeName()}"
                    )
                else:
                    print(f"    {name}: <attribute unavailable>")

        # Print attributes related to deformable topology, collision, cooking
        # and offsets without dumping the point/tet arrays themselves.
        interesting_tokens = (
            "collision",
            "contact",
            "restoffset",
            "simulationmesh",
            "collisionmesh",
            "tet",
            "deformable",
            "cooking",
        )

        interesting_attributes = []

        for attr in prim.GetAttributes():
            attr_name = attr.GetName()
            attr_name_lower = attr_name.lower()

            if any(token in attr_name_lower for token in interesting_tokens):
                interesting_attributes.append(attr)

        if interesting_attributes:
            print("  relevant attributes:")

            for attr in interesting_attributes:
                try:
                    value = attr.Get()
                except Exception as exc:
                    value = f"<Get() failed: {exc}>"

                print(
                    f"    {attr.GetName()}:"
                    f" value={_value_to_string(value)},"
                    f" authored={attr.HasAuthoredValueOpinion()},"
                    f" type={attr.GetTypeName()}"
                )

    print("\n" + "=" * 100)
    print(f"[END USD DEFORMABLE DUMP] {label}")
    print("=" * 100 + "\n")

def design_scene():
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
    cfg.func("/World/Light", cfg)

    # Create a dictionary for the scene entities
    scene_entities = {}

    # Create separate groups called "env_0", "env_1", ...
    # Newton's scene loader requires the "env_\d+" naming convention to
    # detect per-environment Xforms and replicate them as separate worlds.
    origins = [[0.25, 0.25, 0.0], [-0.25, 0.25, 0.0], [0.25, -0.25, 0.0], [-0.25, -0.25, 0.0]]
    for i, origin in enumerate(origins):
        sim_utils.create_prim(f"/World/env_{i}", "Xform", translation=origin)

    youngs_modulus = 1e5
    poissons_ratio = 0.4
    density = 500.0
    if args_cli.backend == "newton":
        from isaaclab_newton.sim.schemas import NewtonDeformableBodyPropertiesCfg
        from isaaclab_newton.sim.spawners.materials import NewtonDeformableBodyMaterialCfg

        deformable_props = NewtonDeformableBodyPropertiesCfg()
        physics_material = NewtonDeformableBodyMaterialCfg(
            k_mu=youngs_modulus / (2.0 * (1.0 + poissons_ratio)),
            k_lambda=youngs_modulus * poissons_ratio / ((1.0 + poissons_ratio) * (1.0 - 2.0 * poissons_ratio)),
            density=density,
        )
    else:
        from isaaclab_physx.sim.schemas import PhysxDeformableBodyPropertiesCfg
        from isaaclab_physx.sim.spawners.materials import PhysxDeformableBodyMaterialCfg

        deformable_props = PhysxDeformableBodyPropertiesCfg(rest_offset=0.0, contact_offset=0.001)
        physics_material = PhysxDeformableBodyMaterialCfg(
            poissons_ratio=poissons_ratio, youngs_modulus=youngs_modulus, density=density
        )

    # 3D Deformable Object
    cfg = DeformableObjectCfg(
        prim_path="/World/env_.*/Cube",
        spawn=sim_utils.MeshCuboidCfg(
            size=(0.2, 0.2, 0.2),
            deformable_props=deformable_props,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.1, 0.0)),
            physics_material=physics_material,
        ),
        init_state=DeformableObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 1.0)),
        debug_vis=True,
    )

    cube_object = DeformableObject(cfg=cfg)
    scene_entities["cube_object"] = cube_object

    # return the scene information
    return scene_entities, origins


def run_simulator(sim: sim_utils.SimulationContext, entities: dict, origins: torch.Tensor, output_dir: str):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability. In general, it is better to access the entities directly from
    #   the dictionary. This dictionary is replaced by the InteractiveScene class in the next tutorial.
    cube_object: DeformableObject = entities["cube_object"]

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    # Nodal kinematic targets of the deformable bodies
    nodal_kinematic_target = cube_object.data.nodal_kinematic_target.torch.clone()

    # Simulate physics
    while simulation_app.is_running():
        # reset at start and after 3 seconds
        if count % int(3.0 / sim_dt) == 0:
            # reset counters
            count = 0

            # reset the nodal state of the object
            nodal_state = cube_object.data.default_nodal_state_w.torch.clone()
            # apply random pose to the object
            pos_w = torch.rand(cube_object.num_instances, 3, device=sim.device) * 0.1 + origins
            quat_w = math_utils.random_orientation(cube_object.num_instances, device=sim.device)
            nodal_state[..., :3] = cube_object.transform_nodal_pos(nodal_state[..., :3], pos_w, quat_w)

            # write nodal state to simulation
            cube_object.write_nodal_state_to_sim_index(nodal_state)

            # Write the nodal state to the kinematic target and free all vertices
            nodal_kinematic_target[..., :3] = nodal_state[..., :3]
            nodal_kinematic_target[..., 3] = 1.0
            cube_object.write_nodal_kinematic_target_to_sim_index(nodal_kinematic_target)

            # reset buffers
            cube_object.reset()

            print("----------------------------------------")
            print("[INFO]: Resetting object state...")

        # update the kinematic target for cubes at index 0 and 3
        kinematic_cubes = [0, 3]
        # we slightly move the cube in the z-direction by picking the vertex at index 0
        nodal_kinematic_target[kinematic_cubes, 0, 2] += 0.2 * sim_dt
        # set vertex at index 0 to be kinematically constrained
        # 0: constrained, 1: free
        nodal_kinematic_target[kinematic_cubes, 0, 3] = 0.0
        # write kinematic target to simulation
        cube_object.write_nodal_kinematic_target_to_sim_index(nodal_kinematic_target)

        # write internal data to simulation
        cube_object.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        cube_object.update(sim_dt)

        if count % int(1.0 / sim_dt) == 0:
            stage = omni.usd.get_context().get_stage()

            nodal_pos_w = cube_object.data.nodal_pos_w.torch
            nodal_min_w = torch.min(nodal_pos_w, dim=1).values
            nodal_max_w = torch.max(nodal_pos_w, dim=1).values

            print("\n[DEFORMABLE FLOOR GAP DIAGNOSTIC]")

            for env_id in range(cube_object.num_instances):
                visual_min_w, visual_max_w = (
                    get_visual_mesh_world_bounds(stage, env_id)
                )

                print(
                    f"  env_{env_id}: "
                    f"sim_min_z={nodal_min_w[env_id, 2].item():.6f}, "
                    f"sim_max_z={nodal_max_w[env_id, 2].item():.6f}, "
                    f"visual_min_z={visual_min_w[2]:.6f}, "
                    f"visual_max_z={visual_max_w[2]:.6f}"
                )

        # print the root positions every second
        if count % int(1.0 / sim_dt) == 0:
            print(f"Time {sim_time:.2f}s: \tRoot position (in world): {cube_object.data.root_pos_w.torch[:, :3]}")


def main():
    """Main function."""
    # Load kit helper
    if args_cli.backend == "newton":
        from isaaclab_newton.physics import NewtonCfg

        from isaaclab_contrib.deformable.newton_manager_cfg import VBDSolverCfg

        physics_cfg = NewtonCfg(solver_cfg=VBDSolverCfg(iterations=10), num_substeps=4)
    else:
        from isaaclab_physx.physics import PhysxCfg

        physics_cfg = PhysxCfg()
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device, physics=physics_cfg)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view(eye=[2.0, 2.0, 2.0], target=[0.0, 0.0, 0.75])
    
        # Design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)

    # MeshCuboidCfg has already generated /Cube/sim_mesh at this point.
    # Author the offsets on the actual collision-bearing TetMesh before
    # PhysX starts and before Isaac Lab creates its tensor views.
    set_deformable_tetmesh_collision_offsets(
        rest_offset=0.0,
        contact_offset=0.001,
    )

    # Optional read-only verification
    stage = omni.usd.get_context().get_stage()
    dump_deformable_usd(
        stage,
        "AFTER TETMESH OFFSET PATCH, BEFORE sim.reset()",
    )

    # Initialize PhysX and tensor views only after the final USD structure
    # and properties are complete.
    sim.reset()
    # Run the simulator
    camera_output = os.path.join(os.path.dirname(os.path.realpath(__file__)), "output", "camera")
    run_simulator(sim, scene_entities, scene_origins, camera_output)
    print("[INFO]: Simulation complete...")


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()

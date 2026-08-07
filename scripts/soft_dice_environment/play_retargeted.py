
import argparse
import sys
import traceback

from isaaclab.app import AppLauncher


# -----------------------------------------------------------------------------
# CLI and application launch
# -----------------------------------------------------------------------------
parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=6000)
parser.add_argument("--dt", type=float, default=1.0 / 60.0)
parser.add_argument("--motion_file", type=str, required=True)
parser.add_argument("--motion_start_frame", type=int, default=0)
parser.add_argument("--motion_loop", action="store_true")
parser.add_argument("--playback_speed", type=float, default=1.0)
parser.add_argument(
    "--replay_mode",
    type=str,
    default="target",
    choices=["state", "target"],
    help="state = write joint state directly; target = send joint position targets.",
)
parser.add_argument("--cube_size", type=float, default=0.31)
parser.add_argument("--table_length", type=float, default=0.8)
parser.add_argument("--table_width", type=float, default=1.20)
parser.add_argument("--ground_z", type=float, default=0.0)
parser.add_argument(
    "--table_xy_mode",
    type=str,
    default="under_cube",
    choices=["under_cube", "manual"],
)
parser.add_argument(
    "--debug",
    action="store_true",
    help="Enable all diagnostic prints, data collection, geometry checks, and plots.",
)

AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(visualizer=["kit"])
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


# Isaac Lab and local imports must be after AppLauncher.
import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, DeformableObjectCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.sim.schemas import MassPropertiesCfg
from isaaclab.utils import configclass
from isaaclab_assets.robots.unitree_h1_aist import H1_FIXED_CFG
from isaaclab_physx.physics import PhysxCfg
from isaaclab_physx.sim import PhysxDeformableBodyMaterialCfg
from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg
from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg

from scripts.soft_dice_environment.replay_debug_diagnostics import ReplayDiagnostics
from replay_runner import replay_motion
from replay_utils import (
    CUSTOM_DICE_DEFORMABLE_USD,
    CUSTOM_DICE_RIGID_REFERENCE_USD,
    CUSTOM_DICE_SCALE,
    REFERENCE_ROBOT_WORLD_OFFSET,
    desired_cube_pose_from_holosoma,
    disable_collisions_under_prim,
    force_enable_dome_light,
    initial_robot_reset,
    load_motion_file,
    quat_xyzw_to_rpy_np,
    validate_asset_paths,
)

DEBUG_PRINT_EVERY = 60


# -----------------------------------------------------------------------------
# Physics and scene configuration
# -----------------------------------------------------------------------------
physx_cfg = PhysxCfg(
    solver_type=1,
    min_position_iteration_count=32,
    max_position_iteration_count=96,
    min_velocity_iteration_count=4,
    max_velocity_iteration_count=16,
    bounce_threshold_velocity=0.2,
)


def make_rigid_cube_cfg(
    prim_path: str,
    color=(0.9, 0.45, 0.0),
    kinematic=False,
    collision_enabled=True,
):
    return RigidObjectCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=CUSTOM_DICE_RIGID_REFERENCE_USD,
            scale=CUSTOM_DICE_SCALE,
            rigid_props=PhysxRigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                kinematic_enabled=kinematic,
                disable_gravity=kinematic,
                max_depenetration_velocity=20.0,
                solver_position_iteration_count=32,
                solver_velocity_iteration_count=4,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=100.0,
                max_angular_velocity=100.0,
            ),
            mass_props=MassPropertiesCfg(mass=0.8),
            collision_props=PhysxCollisionPropertiesCfg(
                collision_enabled=collision_enabled,
                contact_offset=0.001,
                rest_offset=0.0,
            ),
            physics_material=PhysxRigidBodyMaterialCfg(
                static_friction=1.2,
                dynamic_friction=1.0,
                restitution=0.0,
                friction_combine_mode="average",
                restitution_combine_mode="average",
                compliant_contact_stiffness=None,
                compliant_contact_damping=None,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-0.15, 0.0, 1.01),
            rot=(0.0, 0.0, 0.0, 1.0),
        ),
    )


def make_deformable_cube_cfg(prim_path: str):
    return DeformableObjectCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=CUSTOM_DICE_DEFORMABLE_USD,
            scale=CUSTOM_DICE_SCALE,
            physics_material=PhysxDeformableBodyMaterialCfg(
                density=21.5,
                poissons_ratio=0.37,
                youngs_modulus=1.5e4,
                static_friction=1.2,
                dynamic_friction=0.8,
                elasticity_damping=0.02,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.8, 0.1, 0.1),
            ),
        ),
        init_state=DeformableObjectCfg.InitialStateCfg(pos=(-0.15, 0.0, 1.01)),
        debug_vis=True,
    )


@configclass
class ReplaySceneCfg(InteractiveSceneCfg):
    plane = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(
            intensity=3000.0,
            color=(0.75, 0.75, 0.75),
        ),
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.0, 0.0, 0.40],
            rot=[0.0, 0.0, 0.0, 1.0],
        ),
        spawn=sim_utils.CuboidCfg(
            size=(args_cli.table_length, args_cli.table_width, 0.80),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.008,
                rest_offset=0.002,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.35, 0.35, 0.35),
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.1,
                dynamic_friction=0.03,
                restitution=0.0,
                friction_combine_mode="average",
                restitution_combine_mode="average",
            ),
        ),
    )

    cube: DeformableObjectCfg = make_deformable_cube_cfg(
        "{ENV_REGEX_NS}/Cube"
    )

    reference_cube: RigidObjectCfg = make_rigid_cube_cfg(
        "{ENV_REGEX_NS}/ReferenceCube",
        color=(1.0, 0.0, 1.0),
        kinematic=True,
        collision_enabled=False,
    )

    robot: ArticulationCfg = H1_FIXED_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            rot=(0.0, 0.0, 1.0, 0.0),
            pos=(0.0, 0.0, 1.06),
        ),
    )

    reference_robot: ArticulationCfg = H1_FIXED_CFG.replace(
        prim_path="{ENV_REGEX_NS}/ReferenceRobot",
        init_state=ArticulationCfg.InitialStateCfg(
            rot=(0.0, 0.0, 1.0, 0.0),
            pos=(
                float(REFERENCE_ROBOT_WORLD_OFFSET[0]),
                float(REFERENCE_ROBOT_WORLD_OFFSET[1]),
                1.06 + float(REFERENCE_ROBOT_WORLD_OFFSET[2]),
            ),
        ),
    )


# -----------------------------------------------------------------------------
# Initial layout
# -----------------------------------------------------------------------------
def apply_initial_layout(scene_cfg, root_qpos, object_qpos, frame=0):
    if root_qpos is None or object_qpos is None:
        print("[Layout] No root/object qpos found; using configured scene positions.")
        return

    robot_pos = np.asarray(scene_cfg.robot.init_state.pos, dtype=np.float32)
    robot_quat = np.asarray(scene_cfg.robot.init_state.rot, dtype=np.float32)

    cube_pos, cube_quat, dice_rel, root_pos, dice_pos = desired_cube_pose_from_holosoma(
        root_qpos,
        object_qpos,
        frame,
        robot_pos,
        robot_quat,
        reference_offset=None,
        apply_z_lift=False,
    )

    scene_cfg.cube.init_state.pos = cube_pos.tolist()
    scene_cfg.reference_cube.init_state.pos = (
        cube_pos + REFERENCE_ROBOT_WORLD_OFFSET
    ).tolist()
    scene_cfg.reference_cube.init_state.rot = cube_quat.tolist()

    ground_z = float(args_cli.ground_z)
    cube_height = args_cli.cube_size * CUSTOM_DICE_SCALE[2]
    cube_bottom_z = float(cube_pos[2] - cube_height / 2.0)
    table_thickness = cube_bottom_z - ground_z
    if table_thickness <= 0.0:
        raise ValueError(f"Computed table thickness is not positive: {table_thickness}")

    table_pos = np.asarray(scene_cfg.table.init_state.pos, dtype=np.float32)
    if args_cli.table_xy_mode == "under_cube":
        table_pos[0] = cube_pos[0] - 0.25
        table_pos[1] = cube_pos[1]
    table_pos[2] = ground_z + table_thickness / 2.0

    scene_cfg.table.init_state.pos = table_pos.tolist()
    scene_cfg.table.spawn.size = (
        float(args_cli.table_length),
        float(args_cli.table_width),
        float(table_thickness),
    )

    if args_cli.debug:
        print("\n[Initial layout]")
        print(f"  frame:                    {frame}")
        print(f"  source root position:     {root_pos}")
        print(f"  source dice position:     {dice_pos}")
        print(f"  dice relative to root:    {dice_rel}")
        print(f"  Isaac cube position:      {cube_pos}")
        print(f"  Isaac cube quaternion:    {cube_quat}")
        print(f"  Isaac cube RPY [deg]:     {np.rad2deg(quat_xyzw_to_rpy_np(cube_quat))}")
        print(f"  table position:           {table_pos}")
        print(f"  table size:               {scene_cfg.table.spawn.size}")


# -----------------------------------------------------------------------------
# Main orchestration
# -----------------------------------------------------------------------------
def main():
    validate_asset_paths()

    robot_joint_qpos, motion_fps, root_qpos, object_qpos = load_motion_file(
        args_cli.motion_file
    )

    scene_cfg = ReplaySceneCfg(
        num_envs=args_cli.num_envs,
        env_spacing=2.0,
    )
    apply_initial_layout(
        scene_cfg,
        root_qpos,
        object_qpos,
        frame=args_cli.motion_start_frame,
    )

    sim = SimulationContext(
        sim_utils.SimulationCfg(
            dt=float(args_cli.dt),
            device=args_cli.device,
            physics=physx_cfg,
        )
    )
    sim.set_camera_view(
        eye=[-2.6, 0.0, 1.5],
        target=[0.0, 0.0, 1.1],
    )

    scene = InteractiveScene(scene_cfg)

    # Complete structural USD edits before sim.reset().
    for env_id in range(args_cli.num_envs):
        disable_collisions_under_prim(
            f"/World/envs/env_{env_id}/ReferenceRobot"
        )
        disable_collisions_under_prim(
            f"/World/envs/env_{env_id}/ReferenceCube"
        )
    force_enable_dome_light()

    sim.reset()
    initial_robot_reset(scene)
    force_enable_dome_light()

    simulation_app.update()
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim.get_physics_dt())

    diagnostics = ReplayDiagnostics(
        enabled=args_cli.debug,
        print_every=DEBUG_PRINT_EVERY,
    )

    replay_motion(
        simulation_app=simulation_app,
        sim=sim,
        scene=scene,
        robot_joint_qpos_np=robot_joint_qpos,
        motion_fps=motion_fps,
        root_qpos_np=root_qpos,
        object_qpos_np=object_qpos,
        args=args_cli,
        diagnostics=diagnostics,
    )


if __name__ == "__main__":

    try:
        main()
    except BaseException as exc:
        print("\n[ERROR] Python exception caught:", repr(exc), flush=True)
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        raise
    finally:
        print("[INFO] Closing simulation_app", flush=True)
        simulation_app.close()
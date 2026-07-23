import argparse
import torch

from isaaclab.app import AppLauncher
import warp as wp

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=20000)
parser.add_argument("--dt", type=float, default=1.0 / 60.0)

# Timing
parser.add_argument("--approach_duration", type=float, default=0.5)
parser.add_argument("--roll_duration", type=float, default=8.0)

# Motion strength
parser.add_argument("--roll_speed", type=float, default=0.01, help="m/s along X for roll couple")

# Geometry / placement
parser.add_argument("--cube_size", type=float, default=0.30)
parser.add_argument("--push_y_offset", type=float, default=0.30, help="start distance from cube center along Y")
parser.add_argument("--ee_z_offset", type=float, default=0.03, help="base Z offset relative to cube root z (m)")
parser.add_argument("--roll_z_delta", type=float, default=0.20, help="vertical separation between contacts (m)")
parser.add_argument("--clearance", type=float, default=+0.037, help="how far outside the cube face along Y (m)")

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# -----------------------------------------------------------------------------
# Isaac Lab imports
# -----------------------------------------------------------------------------
import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.assets import AssetBaseCfg, DeformableObjectCfg, ArticulationCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.utils.math import subtract_frame_transforms
# from isaaclab.sim.simulation_cfg import PhysxCfg
#from isaaclab_assets.robots.unitree_h1_aist import H1_FIXED_CFG
from isaaclab_assets import H1_CFG  # default Isaac Lab H1

from isaaclab_physx.sim import (
    PhysxDeformableBodyPropertiesCfg,
    PhysxDeformableBodyMaterialCfg,
)


from isaaclab.sim.utils import get_current_stage, create_prim
from pxr import Usd, UsdGeom  # for stage traversal

# -----------------------------------------------------------------------------
# PhysX settings
# -----------------------------------------------------------------------------
# physx_cfg = PhysxCfg(
#     bounce_threshold_velocity=0.2,
#     gpu_max_rigid_contact_count=2**20,
#     gpu_max_rigid_patch_count=2**18,
#     gpu_temp_buffer_capacity=2**20,
#     gpu_max_soft_body_contacts=2**24,
#     gpu_collision_stack_size=2**30,
#     min_position_iteration_count= 64,

# )

# -----------------------------------------------------------------------------
# Sphere end-effector offsets (elbow link frame -> marker), meters
# -----------------------------------------------------------------------------
LEFT_EE_OFFSET_B  = torch.tensor([0.27, 0.0, -0.02], dtype=torch.float32)
RIGHT_EE_OFFSET_B = torch.tensor([0.27, 0.0, -0.02], dtype=torch.float32)

# -----------------------------------------------------------------------------
# Utility
# -----------------------------------------------------------------------------
def cube_side_targets(cube_pos_w: torch.Tensor, cube_size: float, clearance: float):
    """Targets on +Y (left) and -Y (right) faces, outside by (half + clearance)."""
    half = cube_size * 0.5
    left = cube_pos_w.clone()
    right = cube_pos_w.clone()
    left[:, 1] = cube_pos_w[:, 1] + (half + clearance)
    right[:, 1] = cube_pos_w[:, 1] - (half + clearance)
    return left, right


def ee_goal_to_elbow_goal(ee_goal_pos_w: torch.Tensor, elbow_quat_w: torch.Tensor, ee_offset_b: torch.Tensor):
    """
    Desired sphere center (world) -> desired elbow link position (world),
    elbow_pos = sphere_pos - R(elbow_quat)*offset_b
    """
    offset_b = ee_offset_b.to(ee_goal_pos_w.device).unsqueeze(0).repeat(ee_goal_pos_w.shape[0], 1)
    offset_w = math_utils.quat_apply(elbow_quat_w, offset_b)
    return ee_goal_pos_w - offset_w


def compute_sphere_pos_w(elbow_pos_w: torch.Tensor, elbow_quat_w: torch.Tensor, ee_offset_b: torch.Tensor):
    """sphere_pos = elbow_pos + R(elbow_quat)*offset_b"""
    offset_b = ee_offset_b.to(elbow_pos_w.device).unsqueeze(0).repeat(elbow_pos_w.shape[0], 1)
    offset_w = math_utils.quat_apply(elbow_quat_w, offset_b)
    return elbow_pos_w + offset_w


def find_descendant_prim_path_by_name(stage: Usd.Stage, root_path: str, target_name: str) -> str | None:
    """Find first descendant prim under root_path whose prim name matches target_name."""
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == target_name:
            return prim.GetPath().pathString
    return None


def attach_debug_sphere_to_link(
    stage: Usd.Stage,
    link_prim_path: str,
    marker_name: str,
    offset_b: torch.Tensor,
    radius: float,
    color=(0.1, 0.9, 0.1),
):
    """
    Attach a visual-only sphere under the link prim:
      <link_prim_path>/<marker_name>           (Xform)
        └── Sphere                             (UsdGeomSphere)
    The sphere translation is LOCAL (in the link frame via the marker Xform).
    We use the Isaac Lab shape spawner: translation is w.r.t. parent prim. [1](https://isaac-sim.github.io/IsaacLab/v2.0.0/_modules/isaaclab/sim/spawners/shapes/shapes.html)[2](https://docs.robotsfan.com/isaaclab/_modules/isaaclab/sim/spawners/shapes/shapes.html)
    """
    if link_prim_path is None:
        return

    marker_xform_path = f"{link_prim_path}/{marker_name}"
    create_prim(marker_xform_path, "Xform", translation=(0.0, 0.0, 0.0))  # local to link [3](https://docs.robotsfan.com/isaaclab_official/main/_modules/isaaclab/sim/utils/prims.html)[4](https://isaac-sim.github.io/IsaacLab/main/_modules/isaaclab/sim/utils/prims.html)

    sphere_path = f"{marker_xform_path}/Sphere"
    if not stage.GetPrimAtPath(sphere_path).IsValid():
        cfg = sim_utils.SphereCfg(
            radius=float(radius),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
        )
        off = offset_b.detach().cpu().tolist()
        cfg.func(sphere_path, cfg, translation=(float(off[0]), float(off[1]), float(off[2])))  # local to marker/link [1](https://isaac-sim.github.io/IsaacLab/v2.0.0/_modules/isaaclab/sim/spawners/shapes/shapes.html)[2](https://docs.robotsfan.com/isaaclab/_modules/isaaclab/sim/spawners/shapes/shapes.html)


# -----------------------------------------------------------------------------
# Scene config
# -----------------------------------------------------------------------------
@configclass
class RobotDeformableSceneCfg(InteractiveSceneCfg):
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0.0]),
        spawn=GroundPlaneCfg(),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.2, 0, 1.0], rot=[0.7071, 0.0, 0.0, -0.7071]),  # 90 deg about X to be horizontal]),
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    )


    cube = DeformableObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.MeshCuboidCfg(
            size=(args_cli.cube_size, args_cli.cube_size, args_cli.cube_size),

            deformable_props=PhysxDeformableBodyPropertiesCfg(
                # old: deformable_enabled=True
                deformable_body_enabled=True,

                kinematic_enabled=False,
                self_collision=True,

                # IMPORTANT:
                # simulation_hexahedral_resolution was removed in the new API.
                # collision_simplification* options were also removed.
                # PhysX now handles deformable mesh generation internally.

                contact_offset=0.02,
                rest_offset=0.006,

                solver_position_iteration_count=255,
                settling_threshold=0.02,
            ),

            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.5, 0.1, 0.0)
            ),

            physics_material=PhysxDeformableBodyMaterialCfg(
                poissons_ratio=0.25,
                youngs_modulus=9.5e3,
                density=20.5,
                dynamic_friction=0.6 * 1.67,
            ),
        ),

        init_state=DeformableObjectCfg.InitialStateCfg(
            pos=(-0.1, 0.0, 1.15)
        ),

        debug_vis=True,
    )


    robot: ArticulationCfg = H1_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(pos=(-0.5, 0.0, -0.08)),
    )


# -----------------------------------------------------------------------------
# Main loop (ROLL only)
# -----------------------------------------------------------------------------
def run(sim: SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]
    cube = scene["cube"]

    sim_dt = sim.get_physics_dt()
    steps = int(args_cli.steps)

    # EE link names and joints
    LEFT_EE_NAME = "left_elbow_link"
    RIGHT_EE_NAME = "right_elbow_link"

    LEFT_ARM_JOINTS = ["left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow"]
    RIGHT_ARM_JOINTS = ["right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow"]

    LEFT_EE_ID = robot.data.body_names.index(LEFT_EE_NAME)
    RIGHT_EE_ID = robot.data.body_names.index(RIGHT_EE_NAME)

    left_arm_joint_ids_list, _ = robot.find_joints(LEFT_ARM_JOINTS)
    right_arm_joint_ids_list, _ = robot.find_joints(RIGHT_ARM_JOINTS)

    # For PyTorch indexing
    left_arm_joint_ids_long = torch.as_tensor(
        left_arm_joint_ids_list,
        device=robot.device,
        dtype=torch.long,
    )
    right_arm_joint_ids_long = torch.as_tensor(
        right_arm_joint_ids_list,
        device=robot.device,
        dtype=torch.long,
    )

    # For Isaac Lab / Warp APIs
    left_arm_joint_ids_i32 = torch.as_tensor(
        left_arm_joint_ids_list,
        device=robot.device,
        dtype=torch.int32,
    )
    right_arm_joint_ids_i32 = torch.as_tensor(
        right_arm_joint_ids_list,
        device=robot.device,
        dtype=torch.int32,
    )

    if robot.is_fixed_base:
        LEFT_EE_JAC_ID = LEFT_EE_ID - 1
        RIGHT_EE_JAC_ID = RIGHT_EE_ID - 1

        left_arm_jac_joint_ids = left_arm_joint_ids_long
        right_arm_jac_joint_ids = right_arm_joint_ids_long
    else:
        LEFT_EE_JAC_ID = LEFT_EE_ID
        RIGHT_EE_JAC_ID = RIGHT_EE_ID

        # Floating-base Jacobian includes 6 base velocity columns first
        left_arm_jac_joint_ids = left_arm_joint_ids_long + 6
        right_arm_jac_joint_ids = right_arm_joint_ids_long + 6

    # IK controllers
    left_ik = DifferentialIKController(
        DifferentialIKControllerCfg(command_type="position", use_relative_mode=False, ik_method="dls"),
        num_envs=scene.num_envs,
        device=robot.device,
    )
    right_ik = DifferentialIKController(
        DifferentialIKControllerCfg(command_type="position", use_relative_mode=False, ik_method="dls"),
        num_envs=scene.num_envs,
        device=robot.device,
    )

    # Reset robot
    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()

    robot.write_joint_position_to_sim_index(position=joint_pos)
    robot.write_joint_velocity_to_sim_index(velocity=joint_vel)

    robot.reset()
    scene.update(sim_dt)
    # -------------------------------------------------------------------------
    # Attach debug spheres UNDER the elbow link prims (true elbow-frame markers)
    # -------------------------------------------------------------------------
    stage = get_current_stage()
    robot_root_path = "/World/envs/env_0/Robot"  # matches your cfg "{ENV_REGEX_NS}/Robot" when num_envs=1

    left_elbow_prim_path = find_descendant_prim_path_by_name(stage, robot_root_path, LEFT_EE_NAME)
    right_elbow_prim_path = find_descendant_prim_path_by_name(stage, robot_root_path, RIGHT_EE_NAME)

    
    attach_debug_sphere_to_link(
        stage,
        left_elbow_prim_path,
        marker_name="DebugOffset",
        offset_b=LEFT_EE_OFFSET_B,
        radius=0.035,
        color=(0.1, 0.9, 0.1),
    )
    attach_debug_sphere_to_link(
        stage,
        right_elbow_prim_path,
        marker_name="DebugOffset",
        offset_b=RIGHT_EE_OFFSET_B,
        radius=0.035,
        color=(0.9, 0.1, 0.1),
    )

    if left_elbow_prim_path is None or right_elbow_prim_path is None:
        print("[WARN] Could not find elbow prim(s) in USD stage. Markers not attached.")
        print(f"       left_elbow_prim_path={left_elbow_prim_path}")
        print(f"       right_elbow_prim_path={right_elbow_prim_path}")

    approach_steps = max(1, int(args_cli.approach_duration / sim_dt))
    roll_steps = max(1, int(args_cli.roll_duration / sim_dt))
    dy_per_step = float(args_cli.roll_speed) * sim_dt

    for count in range(steps):
        if not simulation_app.is_running():
            break

        # Robot states
        root_pose_w = robot.data.root_pose_w
        ee_left_pose_w = robot.data.body_pose_w[:, LEFT_EE_ID]
       
        # Cube pose (world)
        cube_pos_w = cube.data.root_pos_w.clone()

        # Base contact points on ±Y faces
        left_pre_w, right_pre_w = cube_side_targets(
            cube_pos_w, float(args_cli.cube_size), clearance=float(args_cli.clearance)
        )

        # Z heights (split) -> necessary for roll about X
        z0 = cube_pos_w[:, 2] + float(args_cli.ee_z_offset)
        left_pre_w[:, 2]  = z0 + 0.5 * float(args_cli.roll_z_delta)
        right_pre_w[:, 2] = z0 - 0.5 * float(args_cli.roll_z_delta)

        # Start far: same X/Z, farther away in Y
        start_left_ee_w = left_pre_w.clone()
        start_right_ee_w = right_pre_w.clone()
        start_left_ee_w[:, 1] = cube_pos_w[:, 1] + float(args_cli.push_y_offset)
        start_right_ee_w[:, 1] = cube_pos_w[:, 1] - float(args_cli.push_y_offset)

        # Stage: approach then roll push
        if count < approach_steps:
            alpha = count / float(approach_steps)
            left_ee_goal_w = (1 - alpha) * start_left_ee_w + alpha * left_pre_w
            
        else:
            left_ee_goal_w = left_pre_w.clone()
            

            # Roll push: opposite X directions, Y/Z fixed
            k = min(count - approach_steps, roll_steps)
            left_ee_goal_w[:, 1]  = left_pre_w[:, 1]  - dy_per_step * k   # -Y
           

        # Offset: sphere goal -> elbow goal
        left_elbow_quat_w = ee_left_pose_w[:, 3:7]
        left_elbow_goal_pos_w = ee_goal_to_elbow_goal(left_ee_goal_w, left_elbow_quat_w, LEFT_EE_OFFSET_B)
        

        # World -> root frame for IK
        left_goal_pos_b, _ = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7],
            left_elbow_goal_pos_w, left_elbow_quat_w
        )
        

        ee_left_pos_b, ee_left_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7],
            ee_left_pose_w[:, 0:3], ee_left_pose_w[:, 3:7]
        )

        left_ik.set_command(left_goal_pos_b, ee_quat=ee_left_quat_b)
        
        jacobians = robot.root_view.get_jacobians()

        if not isinstance(jacobians, torch.Tensor):
            jacobians = wp.to_torch(jacobians)

        left_arm_jac_joint_ids_t = left_arm_jac_joint_ids.to(
            device=jacobians.device,
            dtype=torch.long,
        )

        J_left_full = jacobians[:, LEFT_EE_JAC_ID, :, :]
        J_left = torch.index_select(
            J_left_full,
            dim=-1,
            index=left_arm_jac_joint_ids_t,
        )

        q_left_des = left_ik.compute(
            ee_left_pos_b,
            ee_left_quat_b,
            J_left,
            robot.data.joint_pos[:, left_arm_joint_ids_long],
        )

        #Right arm: default + offset to get hand out of the way
        q_right_des = robot.data.default_joint_pos[:, right_arm_joint_ids_long].clone()

        robot.set_joint_position_target_index(
            target=q_left_des,
            joint_ids=left_arm_joint_ids_i32,
        )

        robot.set_joint_position_target_index(
            target=q_right_des,
            joint_ids=right_arm_joint_ids_i32,
        ) 

        if count % 60 == 0:
            elbow_left_pos_w = ee_left_pose_w[0, 0:3]
            elbow_left_quat_w = ee_left_pose_w[0, 3:7]
            sphere_left_pos_w = compute_sphere_pos_w(
                elbow_left_pos_w.unsqueeze(0),
                elbow_left_quat_w.unsqueeze(0),
                LEFT_EE_OFFSET_B,
            )[0]

            print(
                f"[step {count}] cube_z={cube_pos_w[0,2].item():.3f}  "
                f"left_goal(x,z)=({left_ee_goal_w[0,0].item():.3f},{left_ee_goal_w[0,2].item():.3f})  "
                f"left_sphere(x,z)=({sphere_left_pos_w[0].item():.3f},{sphere_left_pos_w[2].item():.3f})"
            )

        # Step sim
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)


def main():
    sim_cfg = sim_utils.SimulationCfg(
        dt=float(args_cli.dt),
        device=args_cli.device,
        #physx=physx_cfg,
    )
    sim = SimulationContext(sim_cfg)

    sim.set_camera_view([1.5, 0.0, 2.5], [0.0, 0.0, 1.1])

    scene_cfg = RobotDeformableSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.5)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO] Scene ready. Running ROLL-only test...")

    run(sim, scene)


if __name__ == "__main__":
    import traceback
    import sys

    try:
        print("[DEBUG] entering main()", flush=True)
        main()
        print("[DEBUG] main() returned normally", flush=True)
    except BaseException as e:
        print("\n[ERROR] Python exception caught:", repr(e), flush=True)
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        raise
    finally:
        print("[DEBUG] closing simulation_app", flush=True)
        simulation_app.close()
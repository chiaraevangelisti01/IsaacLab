import argparse
from dataclasses import replace
import torch

from isaaclab.app import AppLauncher

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=2000)
parser.add_argument("--dt", type=float, default=1.0 / 60.0)

parser.add_argument("--approach_duration", type=float, default=4.0)
parser.add_argument("--push_duration", type=float, default=6.0)
parser.add_argument("--push_speed", type=float, default=0.01)

parser.add_argument("--cube_size", type=float, default=0.30)
parser.add_argument("--push_y_offset", type=float, default=0.30)
parser.add_argument("--x_offcenter", type=float, default=0.095)
parser.add_argument("--ee_z_offset", type=float, default=0.0)

parser.add_argument("--yaw_phase1", type=float, default=10.0, help="seconds of yaw push before regrasp")
parser.add_argument("--regrasp_time", type=float, default=1.0, help="seconds to reposition contact points")

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
from isaaclab.sim.simulation_cfg import PhysxCfg
from isaaclab_assets.robots.unitree_h1_aist import H1_FIXED_CFG

# -----------------------------------------------------------------------------
# STAGE SELECTOR (choose ONE)
# -----------------------------------------------------------------------------
#STAGE = 1   # Stage 1: spawn + hold
#STAGE = 2 # Stage 2: approach only
STAGE = 3 # Stage 3: approach + push off-center

# -----------------------------------------------------------------------------
# PhysX settings copied from your test_push.py
# -----------------------------------------------------------------------------
physx_cfg = PhysxCfg(
    bounce_threshold_velocity=0.2,
    gpu_max_rigid_contact_count=2**20,
    gpu_max_rigid_patch_count=2**18,
    gpu_temp_buffer_capacity=2**20,
    gpu_max_soft_body_contacts=2**24,
    gpu_collision_stack_size=2**30,
    min_position_iteration_count=64,
)

# -----------------------------------------------------------------------------
# Sphere end-effector offsets (EDIT THESE if needed)
# offset is in the ELBOW LINK frame (meters)
# -----------------------------------------------------------------------------
LEFT_EE_OFFSET_B  = torch.tensor([0.27, 0.0, -0.02], dtype=torch.float32)
RIGHT_EE_OFFSET_B = torch.tensor([0.27, 0.0, -0.02], dtype=torch.float32)

# -----------------------------------------------------------------------------
# Utility
# -----------------------------------------------------------------------------
def cube_side_targets(cube_pos_w: torch.Tensor, cube_size: float, clearance: float = -0.00):
    """
    Return targets on +Y (left) and -Y (right) sides OUTSIDE the cube:
    y = center_y +/- (half + clearance)
    """
    half = cube_size * 0.5
    left = cube_pos_w.clone()
    right = cube_pos_w.clone()
    left[:, 1] = cube_pos_w[:, 1] + (half + clearance)
    right[:, 1] = cube_pos_w[:, 1] - (half + clearance)
    return left, right


def ee_goal_to_elbow_goal(ee_goal_pos_w: torch.Tensor, elbow_quat_w: torch.Tensor, ee_offset_b: torch.Tensor):
    """
    Convert desired SPHERE CENTER (world) -> desired ELBOW LINK position (world),
    using the CURRENT elbow orientation:
        elbow_pos_w = ee_pos_w - R(elbow_quat_w) * offset_b
    """
    offset_b = ee_offset_b.to(ee_goal_pos_w.device).unsqueeze(0).repeat(ee_goal_pos_w.shape[0], 1)
    offset_w = math_utils.quat_apply(elbow_quat_w, offset_b)
    return ee_goal_pos_w - offset_w


def compute_sphere_pos_w(elbow_pos_w: torch.Tensor, elbow_quat_w: torch.Tensor, ee_offset_b: torch.Tensor):
    """Forward model: sphere_pos_w = elbow_pos_w + R(elbow_quat_w)*offset_b"""
    offset_b = ee_offset_b.to(elbow_pos_w.device).unsqueeze(0).repeat(elbow_pos_w.shape[0], 1)
    offset_w = math_utils.quat_apply(elbow_quat_w, offset_b)
    return elbow_pos_w + offset_w

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
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0, 1.0], rot=[0.707, 0, 0, 0.707]),
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    )

    cube = DeformableObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.MeshCuboidCfg(
            size=(args_cli.cube_size, args_cli.cube_size, args_cli.cube_size),
            deformable_props=sim_utils.DeformableBodyPropertiesCfg(
                deformable_enabled=True,
                kinematic_enabled=False,
                self_collision=True,
                simulation_hexahedral_resolution=6,
                collision_simplification=True,
                collision_simplification_remeshing=True,
                collision_simplification_remeshing_resolution=30,
                collision_simplification_target_triangle_count=0,
                collision_simplification_force_conforming=True,
                contact_offset=0.01,
                rest_offset=0.004,
                solver_position_iteration_count=64,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.1, 0.0)),
            physics_material=sim_utils.DeformableBodyMaterialCfg(
                poissons_ratio=0.48,
                youngs_modulus=9.5e3,
                density=20.5,
            ),
        ),
        init_state=DeformableObjectCfg.InitialStateCfg(pos=(-0.1, 0.0, 1.15)),
        debug_vis=True,
    )

    robot: ArticulationCfg = H1_FIXED_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )

# -----------------------------------------------------------------------------
# Main loop
# -----------------------------------------------------------------------------
def run(sim: SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]
    cube = scene["cube"]

    sim_dt = sim.get_physics_dt()
    steps = int(args_cli.steps)

    # EE link names and joints (your setup)
    LEFT_EE_NAME = "left_elbow_link"
    RIGHT_EE_NAME = "right_elbow_link"

    LEFT_ARM_JOINTS = ["left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow"]
    RIGHT_ARM_JOINTS = ["right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow"]

    LEFT_EE_ID = robot.data.body_names.index(LEFT_EE_NAME)
    RIGHT_EE_ID = robot.data.body_names.index(RIGHT_EE_NAME)

    left_arm_joint_ids, _ = robot.find_joints(LEFT_ARM_JOINTS)
    right_arm_joint_ids, _ = robot.find_joints(RIGHT_ARM_JOINTS)
    left_arm_joint_ids = torch.as_tensor(left_arm_joint_ids, device=robot.device)
    right_arm_joint_ids = torch.as_tensor(right_arm_joint_ids, device=robot.device)

    # Jacobian indices: fixed base -> body_id - 1
    if robot.is_fixed_base:
        LEFT_EE_JAC_ID = LEFT_EE_ID - 1
        RIGHT_EE_JAC_ID = RIGHT_EE_ID - 1
    else:
        LEFT_EE_JAC_ID = LEFT_EE_ID
        RIGHT_EE_JAC_ID = RIGHT_EE_ID

    # IK controllers (pose)
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
    robot.write_joint_state_to_sim(joint_pos, joint_vel)
    robot.reset()
    scene.update(sim_dt)

    approach_steps = max(1, int(args_cli.approach_duration / sim_dt))
    push_steps = max(1, int(args_cli.push_duration / sim_dt))
    dy_per_step = float(args_cli.push_speed) * sim_dt

    for count in range(steps):
        if not simulation_app.is_running():
            break

        # Read robot states
        root_pose_w = robot.data.root_pose_w
        ee_left_pose_w = robot.data.body_pose_w[:, LEFT_EE_ID]
        ee_right_pose_w = robot.data.body_pose_w[:, RIGHT_EE_ID]

        # Cube pose
        cube_pos_w = cube.data.root_pos_w.clone()

        # ---- Sphere center goals (world)
        # Start far
        start_left_ee_w = cube_pos_w.clone()
        start_right_ee_w = cube_pos_w.clone()

        start_left_ee_w[:, 0] += float(args_cli.x_offcenter)
        start_right_ee_w[:, 0] -= float(args_cli.x_offcenter)
        start_left_ee_w[:, 1] += float(args_cli.push_y_offset)
        start_right_ee_w[:, 1] -= float(args_cli.push_y_offset)

        # Pre-contact (outside cube)
        left_pre_w, right_pre_w = cube_side_targets(cube_pos_w, float(args_cli.cube_size), clearance=0.03)
        left_pre_w[:, 0] += float(args_cli.x_offcenter)
        right_pre_w[:, 0] -= float(args_cli.x_offcenter)
     
        t = count * sim_dt
        t0 = args_cli.approach_duration
        t1 = t0 + args_cli.yaw_phase1
        t2 = t1 + args_cli.regrasp_time
        # Stage logic
        if STAGE == 1:
            left_ee_goal_w = left_pre_w
            right_ee_goal_w = right_pre_w
        else:
            if count < approach_steps:
                alpha = count / float(approach_steps)
                left_ee_goal_w = (1 - alpha) * start_left_ee_w + alpha * left_pre_w
                right_ee_goal_w = (1 - alpha) * start_right_ee_w + alpha * right_pre_w
            else:
                left_ee_goal_w = left_pre_w
                right_ee_goal_w = right_pre_w

            if STAGE == 3 and count >= approach_steps:
                # Time markers (seconds)
                t = count * sim_dt
                t0 = args_cli.approach_duration
                t1 = t0 + args_cli.yaw_phase1
                t2 = t1 + args_cli.regrasp_time

                # Symmetric inward push -> yaw couple
                # left (+Y) goes -Y, right (-Y) goes +Y
                def apply_yaw_push(k_steps, x_left, x_right):
                    left_ee_goal_w[:, 0]  = cube_pos_w[:, 0] + x_left
                    right_ee_goal_w[:, 0] = cube_pos_w[:, 0] + x_right
                    left_ee_goal_w[:, 1]  = left_pre_w[:, 1]  - dy_per_step * k_steps
                    right_ee_goal_w[:, 1] = right_pre_w[:, 1] + dy_per_step * k_steps

                # Phase A: rotate ~0 -> ~45 (use initial offsets)
                if t0 <= t < t1:
                    k_steps = min(int((t - t0) / sim_dt), push_steps)
                    apply_yaw_push(k_steps, +args_cli.x_offcenter, -args_cli.x_offcenter)

                # Phase B: regrasp (move contact points in X only, keep Y at pre-contact)
                elif t1 <= t < t2:
                    # interpolate x offsets from (+a,-a) to (-a,+a)
                    alpha = (t - t1) / max(1e-6, args_cli.regrasp_time)
                    xL = (1 - alpha) * (+args_cli.x_offcenter) + alpha * (-args_cli.x_offcenter)
                    xR = (1 - alpha) * (-args_cli.x_offcenter) + alpha * (+args_cli.x_offcenter)

                    left_ee_goal_w[:, 0]  = cube_pos_w[:, 0] + xL
                    right_ee_goal_w[:, 0] = cube_pos_w[:, 0] + xR
                    left_ee_goal_w[:, 1]  = left_pre_w[:, 1]
                    right_ee_goal_w[:, 1] = right_pre_w[:, 1]

                # Phase C: rotate ~45 -> ~90 (use swapped offsets)
                else:  # t >= t2
                    k_steps = min(int((t - t2) / sim_dt), push_steps)
                    apply_yaw_push(k_steps, -args_cli.x_offcenter, +args_cli.x_offcenter)


        # ---- Apply OFFSET correctly: sphere goal -> elbow goal
        left_elbow_quat_w = ee_left_pose_w[:, 3:7]
        right_elbow_quat_w = ee_right_pose_w[:, 3:7]

        left_elbow_goal_pos_w = ee_goal_to_elbow_goal(left_ee_goal_w, left_elbow_quat_w, LEFT_EE_OFFSET_B)
        right_elbow_goal_pos_w = ee_goal_to_elbow_goal(right_ee_goal_w, right_elbow_quat_w, RIGHT_EE_OFFSET_B)

        # Keep elbow orientation = current (stable)
        left_goal_pose_w = torch.cat([left_elbow_goal_pos_w, left_elbow_quat_w], dim=1)
        right_goal_pose_w = torch.cat([right_elbow_goal_pos_w, right_elbow_quat_w], dim=1)

        # ---- Convert to root frame (goal + current EE)
        left_goal_pos_b, left_goal_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7],
            left_goal_pose_w[:, 0:3], left_goal_pose_w[:, 3:7]
        )
        right_goal_pos_b, right_goal_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7],
            right_goal_pose_w[:, 0:3], right_goal_pose_w[:, 3:7]
        )

        ee_left_pos_b, ee_left_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7],
            ee_left_pose_w[:, 0:3], ee_left_pose_w[:, 3:7]
        )
        ee_right_pos_b, ee_right_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7],
            ee_right_pose_w[:, 0:3], ee_right_pose_w[:, 3:7]
        )

        left_ik.set_command(left_goal_pos_b, ee_quat=ee_left_quat_b)
        right_ik.set_command(right_goal_pos_b, ee_quat=ee_right_quat_b)
        # ---- Jacobians and IK compute
        jacobians = robot.root_physx_view.get_jacobians()
        J_left = jacobians[:, LEFT_EE_JAC_ID, :, left_arm_joint_ids]
        J_right = jacobians[:, RIGHT_EE_JAC_ID, :, right_arm_joint_ids]

        q_left_des = left_ik.compute(
            ee_left_pos_b, ee_left_quat_b, J_left,
            robot.data.joint_pos[:, left_arm_joint_ids]
        )
        q_right_des = right_ik.compute(
            ee_right_pos_b, ee_right_quat_b, J_right,
            robot.data.joint_pos[:, right_arm_joint_ids]
        )

        robot.set_joint_position_target(q_left_des, joint_ids=left_arm_joint_ids)
        robot.set_joint_position_target(q_right_des, joint_ids=right_arm_joint_ids)

        if count % 60 == 0:
            cube_z = cube_pos_w[0, 2].item()

            # elbow pose world
            elbow_left_pos_w = ee_left_pose_w[0, 0:3]
            elbow_left_quat_w = ee_left_pose_w[0, 3:7]

            # sphere (virtual EE) world
            sphere_left_pos_w = compute_sphere_pos_w(
                elbow_left_pos_w.unsqueeze(0),
                elbow_left_quat_w.unsqueeze(0),
                LEFT_EE_OFFSET_B,
            )[0]

            sphere_z = sphere_left_pos_w[2].item()
            goal_z = left_ee_goal_w[0, 2].item()

            print(f"[step {count}] cube_z={cube_z:.3f}  sphere_z={sphere_z:.3f}  goal_z={goal_z:.3f}  dz(sphere-goal)={sphere_z-goal_z:+.3f}")
        # Step
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

       

def main():
    sim_cfg = sim_utils.SimulationCfg(
        dt=float(args_cli.dt),
        device=args_cli.device,
        physx=physx_cfg,
    )
    sim = SimulationContext(sim_cfg)

    sim.set_camera_view([1.5, 0.0, 2.5], [0.0, 0.0, 1.1])

    scene_cfg = RobotDeformableSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.5)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO] Scene ready. Running H1 sphere push stages...")

    run(sim, scene)

if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
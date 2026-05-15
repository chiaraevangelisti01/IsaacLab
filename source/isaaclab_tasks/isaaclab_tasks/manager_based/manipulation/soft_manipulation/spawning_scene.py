import argparse
from isaaclab.app import AppLauncher

# CLI
parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)


AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Isaac imports
from isaaclab.envs.mdp.observations import joint_vel
import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.assets import ArticulationCfg, DeformableObjectCfg, AssetBaseCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.sensors.camera import CameraCfg

from isaaclab_assets.robots.unitree_h1_aist import H1_CFG, H1_FIXED_CFG
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.controllers import DifferentialIKControllerCfg, DifferentialIKController

from isaaclab.utils.math import subtract_frame_transforms


import torch

def cube_side_targets(cube_pos_w: torch.Tensor, cube_size: float, clearance: float = -0.3):
    """
    cube_pos_w: (N,3) tensor in world frame
    returns: left/right targets as (N,3)
    """
    half = cube_size * 0.5
    left = cube_pos_w.clone()
    right = cube_pos_w.clone()
    left[:, 1] += half + clearance
    right[:, 1] -= half + clearance
    return left, right


# ---------------------------
# Scene Config (FIXED)
# ---------------------------
@configclass
class SpawningSceneCfg(InteractiveSceneCfg):

    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0.0]),
        spawn=GroundPlaneCfg(),
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


    # Table (USD)
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0, 1.0], rot=[0.707, 0, 0, 0.707]),
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    )

    # Deformable Cube
    cube = DeformableObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.MeshCuboidCfg(
            size=(0.35, 0.35, 0.35),
            deformable_props=sim_utils.DeformableBodyPropertiesCfg(rest_offset=0.0, contact_offset=0.001),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.1, 0.0)),
            physics_material=sim_utils.DeformableBodyMaterialCfg(poissons_ratio=0.4, youngs_modulus=1e5),
        ),
        init_state=DeformableObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 1.15)),
        debug_vis=True,
    )
    
    # Robot 
    robot: ArticulationCfg = H1_FIXED_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(-0.5, 0.0, -0.08),   
           )
    )

    # RGB Camera (D435 RGB module pose from Unitree URDF)
    rgb_camera = CameraCfg(
        
        prim_path="{ENV_REGEX_NS}/Robot/torso_link/d435_rgb",

        update_period=0.0,  # update every physics step (set e.g. 0.1 for 10 Hz)
        width=640,
        height=480,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 1000.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(0.10848474394, 0.0325, 0.69317107367),
            rot=(0.2372241533, -0.6661252619, 0.6661277087, -0.2372250246),  # (w,x,y,z)
            convention="ros",
        ),
    )

# ---------------------------
# Simulation Loop
# ---------------------------
def run(sim, scene):

    robot = scene["robot"]
    cube = scene["cube"]
    cam = scene["rgb_camera"]  

    sim_dt = sim.get_physics_dt()
    count = 0

    LEFT_EE_NAME = "left_elbow_link"
    RIGHT_EE_NAME = "right_elbow_link"

    LEFT_EE_ID = robot.data.body_names.index(LEFT_EE_NAME)
    RIGHT_EE_ID = robot.data.body_names.index(RIGHT_EE_NAME)

    LEFT_ARM_JOINTS = [
        "left_shoulder_pitch",
        "left_shoulder_roll",
        "left_shoulder_yaw",
        "left_elbow",
    ]

    RIGHT_ARM_JOINTS = [
        "right_shoulder_pitch",
        "right_shoulder_roll",
        "right_shoulder_yaw",
        "right_elbow",
    ]

    left_ik = DifferentialIKController(
    DifferentialIKControllerCfg(
        command_type="pose",
        use_relative_mode=False,
        ik_method="dls",
    ),
    num_envs=scene.num_envs,
    device=robot.device,
    )

    right_ik = DifferentialIKController(
        DifferentialIKControllerCfg(
            command_type="pose",
            use_relative_mode=False,
            ik_method="dls",
        ),
        num_envs=scene.num_envs,
        device=robot.device,
    )

    left_arm_joint_ids, _ = robot.find_joints(LEFT_ARM_JOINTS)
    right_arm_joint_ids, _ = robot.find_joints(RIGHT_ARM_JOINTS)

    # ensure tensor
    left_arm_joint_ids = torch.as_tensor(left_arm_joint_ids, device=robot.device)
    right_arm_joint_ids = torch.as_tensor(right_arm_joint_ids, device=robot.device)


    if robot.is_fixed_base:
        LEFT_EE_JAC_ID = LEFT_EE_ID - 1
        RIGHT_EE_JAC_ID = RIGHT_EE_ID - 1
    else:
        LEFT_EE_JAC_ID = LEFT_EE_ID
        RIGHT_EE_JAC_ID = RIGHT_EE_ID

    # Joint IDs (comme tu as déjà)
    left_arm_joint_ids, _ = robot.find_joints(LEFT_ARM_JOINTS)
    right_arm_joint_ids, _ = robot.find_joints(RIGHT_ARM_JOINTS)
    left_arm_joint_ids = torch.as_tensor(left_arm_joint_ids, device=robot.device)
    right_arm_joint_ids = torch.as_tensor(right_arm_joint_ids, device=robot.device)


    cube_trajectory = torch.linspace(1.15, 1.55, steps=200)  # example trajectory for cube's z position



    while simulation_app.is_running() and count < 200:



        # --- (1) état robot à jour (après scene.update du tour précédent)
        root_pose_w = robot.data.root_pose_w                     # (N,7)
        ee_left_pose_w = robot.data.body_pose_w[:, LEFT_EE_ID]   # (N,7)
        ee_right_pose_w = robot.data.body_pose_w[:, RIGHT_EE_ID] # (N,7)

        # --- (2) cible cube en world
        cube_target_pos = cube.data.root_pos_w.clone()         # (N,3)

        # if you want a vertical trajectory, only override z:
        cube_target_pos[:, 2] = cube_trajectory[count]         # scalar broadcast to (N,)

        left_target_w, right_target_w = cube_side_targets(cube_target_pos, cube_size=0.35)

        quat_w = torch.tensor([1.0, 0.0, 0.0, 0.0], device=robot.device).repeat(scene.num_envs, 1)  # (N,4)
        left_goal_pose_w  = torch.cat([left_target_w,  quat_w], dim=1)   # (N,7)
        right_goal_pose_w = torch.cat([right_target_w, quat_w], dim=1)   # (N,7)


        # --- (3) convert world -> root frame (goal + current EE)
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

        left_cmd_b  = torch.cat([left_goal_pos_b,  left_goal_quat_b],  dim=1)  # (N,7)
        right_cmd_b = torch.cat([right_goal_pos_b, right_goal_quat_b], dim=1)

      

        left_ik.set_command(left_cmd_b)
        right_ik.set_command(right_cmd_b)

        # --- (4) jacobians: recompute EVERY STEP
        jacobians = robot.root_physx_view.get_jacobians()  # (N, 19, 6, dofs)

        J_left  = jacobians[:, LEFT_EE_JAC_ID, :, left_arm_joint_ids]   # (N,6,4)
        J_right = jacobians[:, RIGHT_EE_JAC_ID, :, right_arm_joint_ids] # (N,6,4)

       

        # --- (5) IK compute -> desired joint positions (pas des deltas)
        q_left_des = left_ik.compute(
            ee_left_pos_b, ee_left_quat_b, J_left,
            robot.data.joint_pos[:, left_arm_joint_ids]
        )
        q_right_des = right_ik.compute(
            ee_right_pos_b, ee_right_quat_b, J_right,
            robot.data.joint_pos[:, right_arm_joint_ids]
        )


        # --- (6) Apply as targets (stable)
        robot.set_joint_position_target(q_left_des, joint_ids=left_arm_joint_ids)
        robot.set_joint_position_target(q_right_des, joint_ids=right_arm_joint_ids)

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        # rgb = cam.data.output["rgb"]  # shape typically: (num_envs, H, W, 3)
        # if count % 60 == 0:
        #     print("RGB shape:", rgb.shape)

        count += 1


# ---------------------------
# Main
# ---------------------------
def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)

    sim.set_camera_view([2.5, 0.0, 2.5], [0.0, 0.0, 1.0])

    scene_cfg = SpawningSceneCfg(
        num_envs=args_cli.num_envs,
        env_spacing=2.5
    )

    scene = InteractiveScene(scene_cfg)

    sim.reset()

    print("[INFO]: Scene ready!")

    run(sim, scene)


if __name__ == "__main__":
    try:
        main()
    finally:simulation_app.close()
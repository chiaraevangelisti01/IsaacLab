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
from isaaclab.utils import configclass

from isaaclab_assets.robots.unitree_h1_aist import H1_CFG, H1_FIXED_CFG
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

import torch


# ---------------------------
# Scene Config (FIXED)
# ---------------------------
@configclass
class SpawningSceneCfg(InteractiveSceneCfg):

    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -0.05]),
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
            size=(0.2, 0.2, 0.2),
            deformable_props=sim_utils.DeformableBodyPropertiesCfg(rest_offset=0.0, contact_offset=0.001),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.1, 0.0)),
            physics_material=sim_utils.DeformableBodyMaterialCfg(poissons_ratio=0.4, youngs_modulus=1e5),
        ),
        init_state=DeformableObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 1.0)),
        debug_vis=True,
    )
    
    # Robot 
    robot: ArticulationCfg = H1_FIXED_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(-0.6, 0.0, 1.0),   
           )
    )


# ---------------------------
# Simulation Loop
# ---------------------------
def run(sim, scene):
    robot = scene["robot"]
    cube = scene["cube"]

    sim_dt = sim.get_physics_dt()
    count = 0

    while simulation_app.is_running():

        #  RESET EVERY N STEPS
        if count % 300 == 0:

            count = 0

            # --- reset everything else (cube, buffers, etc.)
            scene.reset()

            # --- restore robot EXACT spawn state
            root_state = robot.data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins

            robot.write_root_pose_to_sim(root_state[:, :7])
            robot.write_root_velocity_to_sim(root_state[:, 7:])

            # joint_pos = robot.data.default_joint_pos.clone()
            # joint_vel = robot.data.default_joint_vel.clone()
            print(robot.data.joint_names)
            print(robot.data.default_joint_pos.shape)
            #set randoma joint positions within limits
            joint_pos = torch.rand_like(robot.data.default_joint_pos) * 0.35 - 0.05
            joint_vel = torch.rand_like(robot.data.default_joint_vel) * 0.35 - 0.05
            robot.write_joint_state_to_sim(joint_pos, joint_vel)

            nodal_state = cube.data.default_nodal_state_w.clone()
            cube.write_nodal_state_to_sim(nodal_state)
            cube.reset()

        # normal step
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

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
    main()
    simulation_app.close()
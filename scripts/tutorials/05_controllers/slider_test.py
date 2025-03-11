from isaaclab.app import AppLauncher

# Start Isaac Lab
args_cli = {"headless": False}  # Set False to see the simulation
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.assets import AssetBaseCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab_assets import FROG_V0_CFG


@configclass
class TableTopSceneCfg(InteractiveSceneCfg):
    """Scene with a ground plane and a robot."""
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.87, 0.87, 0.87))
    )

    robot = FROG_V0_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    robot.spawn.articulation_props.fix_root_link = True


sim_cfg = SimulationCfg(dt=0.001, device="cuda:0", gravity=(0.0, 0.0, 0.0))  
sim = SimulationContext(sim_cfg)

scene_cfg = TableTopSceneCfg(num_envs=1, env_spacing=0)
scene = InteractiveScene(scene_cfg)

sim.reset()

robot = scene["robot"]
robot_entity_cfg = SceneEntityCfg("robot", joint_names=[".*"], body_names=["base"])
robot_entity_cfg.resolve(scene)
sim_dt = sim.get_physics_dt()


joint_names = robot.joint_names
num_joints = len(joint_names)
joint_positions = torch.zeros((1, num_joints), dtype=torch.float32, device="cuda:0")


fig, ax = plt.subplots(figsize=(7, num_joints * 0.6))  
plt.subplots_adjust(left=0.4, right=0.95, top=0.95, bottom=0.05) 
sliders = []

def update(val):
    """Update joint positions when sliders move."""
    for i, slider in enumerate(sliders):
        joint_positions[:, i] = slider.val
    robot.set_joint_position_target(joint_positions)
    scene.write_data_to_sim()

# Create sliders with labels
for i, name in enumerate(joint_names):
    y_position = 0.9 - (i * 0.07) 
    ax_name = plt.axes([0.05, y_position, 0.2, 0.03])  
    ax_name.set_xticks([])  
    ax_name.set_yticks([]) 
    ax_name.set_frame_on(False)  
    ax_name.text(0.5, 0.5, name, fontsize=10, ha='center', va='center')  

    ax_slider = plt.axes([0.3, y_position, 0.6, 0.03])  
    slider = Slider(ax_slider, "", -1.57, 1.57, valinit=0.0)  
    slider.on_changed(update)
    sliders.append(slider)


plt.show(block=False)

# Simulation loop
while simulation_app.is_running():
    sim.step()
    scene.update(sim.get_physics_dt())
    plt.pause(0.01)

simulation_app.close()

# # Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# # All rights reserved.
# #
# # SPDX-License-Identifier: BSD-3-Clause

# """Spawn default Isaac Lab H1 and inspect its grounding behavior.

# Usage:
#     ./isaaclab.sh -p scripts/tutorials/01_assets/run_h1_articulation.py
# """

# """Launch Isaac Sim Simulator first."""

# import argparse

# from isaaclab.app import AppLauncher

# # add argparse arguments
# parser = argparse.ArgumentParser(description="Tutorial on spawning and interacting with H1.")
# # append AppLauncher cli args
# AppLauncher.add_app_launcher_args(parser)
# # parse the arguments
# args_cli = parser.parse_args()

# # launch omniverse app
# app_launcher = AppLauncher(args_cli)
# simulation_app = app_launcher.app

# """Rest everything follows."""

# import torch

# import isaaclab.sim as sim_utils
# from isaaclab.assets import Articulation
# from isaaclab.sim import SimulationContext

# ##
# # Pre-defined configs
# ##
# from isaaclab_assets import H1_CFG  # isort:skip


# def design_scene() -> tuple[dict, list[list[float]]]:
#     """Designs the scene."""
#     # Ground-plane
#     cfg = sim_utils.GroundPlaneCfg()
#     cfg.func("/World/defaultGroundPlane", cfg)

#     # Lights
#     cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
#     cfg.func("/World/Light", cfg)

#     # Create separate groups called "Origin1", "Origin2"
#     # Each group will have a robot in it
#     origins = [[0.0, 0.0, 0.0], [-2.0, 0.0, 0.0]]

#     # Origin 1
#     sim_utils.create_prim("/World/Origin1", "Xform", translation=origins[0])

#     # Origin 2
#     sim_utils.create_prim("/World/Origin2", "Xform", translation=origins[1])

#     # Articulation
#     h1_cfg = H1_CFG.copy()
#     h1_cfg.prim_path = "/World/Origin.*/Robot"
#     h1_cfg.spawn.articulation_props.fix_root_link = True
#     h1 = Articulation(cfg=h1_cfg)

#     # return the scene information
#     scene_entities = {"h1": h1}
#     return scene_entities, origins


# def print_h1_heights(robot: Articulation):
#     """Print root and lowest body heights for checking grounding."""
#     root_pos = robot.data.root_pos_w.detach().cpu()
#     body_pos = robot.data.body_pos_w.detach().cpu()
#     body_names = robot.data.body_names

#     print("\n[H1 height check]")
#     for env_id in range(root_pos.shape[0]):
#         z_values = body_pos[env_id, :, 2]
#         min_idx = int(torch.argmin(z_values).item())

#         print(f"  env {env_id}")
#         print(f"    root_pos_w: {root_pos[env_id].numpy()}")
#         print(f"    min_body_z: {float(z_values[min_idx]):.4f}")
#         print(f"    min_body_name: {body_names[min_idx]}")

#         for name, pos in zip(body_names, body_pos[env_id]):
#             name_lower = name.lower()
#             if "ankle" in name_lower or "foot" in name_lower:
#                 print(f"    {name:30s} z={float(pos[2]):.4f}")


# def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
#     """Runs the simulation loop."""
#     robot = entities["h1"]

#     # Define simulation stepping
#     sim_dt = sim.get_physics_dt()
#     count = 0

#     # Simulation loop
#     while simulation_app.is_running():
#         # Reset
#         if count % 500 == 0:
#             count = 0

#             # Root state
#             # Offset root state by the origin because states are written in simulation world frame.
#             root_state = robot.data.default_root_state.clone()
#             root_state[:, :3] += origins

#             robot.write_root_pose_to_sim(root_state[:, :7])
#             robot.write_root_velocity_to_sim(root_state[:, 7:])

#             # Default H1 joint state
#             joint_pos = robot.data.default_joint_pos.clone()
#             joint_vel = robot.data.default_joint_vel.clone()

#             robot.write_joint_state_to_sim(joint_pos, joint_vel)

#             # Also set position targets to the default pose so the robot tries to hold it.
#             robot.set_joint_position_target(joint_pos)

#             # Clear internal buffers
#             robot.reset()

#             print("[INFO]: Resetting H1 state...")
#             print_h1_heights(robot)

#         # Hold default joint position target
#         robot.set_joint_position_target(robot.data.default_joint_pos)

#         # Write data to sim
#         robot.write_data_to_sim()

#         # Perform step
#         sim.step()

#         # Increment counter
#         count += 1

#         # Update buffers
#         robot.update(sim_dt)

#         # Print occasionally after physics has stepped
#         if count % 120 == 0:
#             print_h1_heights(robot)


# def main():
#     """Main function."""
#     # Load kit helper
#     sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
#     sim = SimulationContext(sim_cfg)

#     # Set main camera
#     sim.set_camera_view([3.0, -3.0, 2.5], [0.0, 0.0, 1.0])

#     # Design scene
#     scene_entities, scene_origins = design_scene()
#     scene_origins = torch.tensor(scene_origins, device=sim.device)

#     # Play the simulator
#     sim.reset()

#     print("[INFO]: Setup complete...")

#     # Run the simulator
#     run_simulator(sim, scene_entities, scene_origins)


# if __name__ == "__main__":
#     main()
#     simulation_app.close()

# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Spawn default Isaac Lab H1 using InteractiveScene and inspect grounding."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Spawn default H1 with InteractiveScene.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg
from isaaclab.utils import configclass

from isaaclab_assets import H1_CFG  # default Isaac Lab H1


@configclass
class H1SceneCfg(InteractiveSceneCfg):
    plane = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=GroundPlaneCfg(),
    )

    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(
            intensity=3000.0,
            color=(0.75, 0.75, 0.75),
        ),
    )

    robot: ArticulationCfg = H1_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.98)),
    )


def print_h1_debug(scene: InteractiveScene, label: str):
    robot = scene["robot"]

    root_pos = robot.data.root_pos_w.detach().cpu()
    default_root_state = robot.data.default_root_state.detach().cpu()
    body_pos = robot.data.body_pos_w.detach().cpu()
    body_names = robot.data.body_names

    print(f"\n[H1 DEBUG] {label}")
    print("  env_origins:")
    print(scene.env_origins.detach().cpu().numpy())

    print("  default_root_state[:, :3]:")
    print(default_root_state[:, :3].numpy())

    print("  root_pos_w:")
    print(root_pos.numpy())

    for env_id in range(root_pos.shape[0]):
        z_values = body_pos[env_id, :, 2]
        min_idx = int(torch.argmin(z_values).item())

        print(f"\n  env {env_id}")
        print(f"    min_body_z:    {float(z_values[min_idx]):.4f}")
        print(f"    min_body_name: {body_names[min_idx]}")

        for name, pos in zip(body_names, body_pos[env_id]):
            name_lower = name.lower()
            if "ankle" in name_lower or "foot" in name_lower:
                print(f"    {name:30s} z={float(pos[2]):.4f}")

def reset_robot_like_interactive_scene_tutorial(scene: InteractiveScene):
    robot = scene["robot"]

    # Root state is stored relative to env origin.
    # Write it in world frame by adding scene.env_origins.
    root_state = robot.data.default_root_state.clone()
    root_state[:, :3] += scene.env_origins

    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(root_state[:, 7:])

    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()

    robot.write_joint_state_to_sim(joint_pos, joint_vel)
    robot.set_joint_position_target(joint_pos)

    # Official InteractiveScene pattern clears internal buffers through scene.reset()
    scene.reset()

    print("[INFO] Reset robot using InteractiveScene tutorial pattern.")
    print("  scene.env_origins:", scene.env_origins.detach().cpu().numpy())
    print("  default_root_state[:, :3]:", robot.data.default_root_state[:, :3].detach().cpu().numpy())

def run(sim: SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]
    sim_dt = sim.get_physics_dt()
    count = 0

    # Important: after sim.reset(), update buffers once before inspecting.
    scene.update(sim_dt)
    print_h1_debug(scene, "after sim.reset() and scene.update()")

    while simulation_app.is_running():
        # Hold default H1 joint pose.
        robot.set_joint_position_target(robot.data.default_joint_pos)

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        if count % 120 == 0:
            print_h1_debug(scene, f"after {count} steps")

        count += 1


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)

    sim.set_camera_view([3.0, -3.0, 2.5], [0.0, 0.0, 1.0])

    scene_cfg = H1SceneCfg(
        num_envs=1,
        env_spacing=2.0,
    )

    print("\n[CONFIG DEBUG]")
    print("  H1_CFG.init_state.pos:", H1_CFG.init_state.pos)
    print("  scene_cfg.robot.init_state.pos:", scene_cfg.robot.init_state.pos)
    print("  scene_cfg.robot.prim_path:", scene_cfg.robot.prim_path)

    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO] Setup complete.")

    reset_robot_like_interactive_scene_tutorial(scene)

    # one update so buffers reflect the reset
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim.get_physics_dt())


    run(sim, scene)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
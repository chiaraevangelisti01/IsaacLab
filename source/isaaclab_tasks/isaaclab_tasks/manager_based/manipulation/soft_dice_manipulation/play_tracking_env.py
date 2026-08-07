"""Parity test: replay a Holosoma/OmniRetarget soft-dice motion through ManagerBasedRLEnv.
"""

import argparse
import sys
import traceback

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser()
parser.add_argument("--motion_file", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=6000)
parser.add_argument("--dt", type=float, default=1.0 / 60.0)
parser.add_argument("--motion_start_frame", type=int, default=0)
parser.add_argument("--motion_loop", action="store_true")
parser.add_argument("--playback_speed", type=float, default=1.0)
parser.add_argument("--cube_size", type=float, default=0.31)
parser.add_argument("--table_length", type=float, default=0.8)
parser.add_argument("--table_width", type=float, default=1.20)
parser.add_argument("--ground_z", type=float, default=0.0)

AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(visualizer=["kit"])
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Isaac Lab imports must be after AppLauncher.
import torch

from isaaclab.envs import ManagerBasedRLEnv

from isaaclab_tasks.manager_based.manipulation.soft_dice_manipulation.soft_dice_env_cfg import SoftDiceTrackingEnvCfg


def main():
    cfg = SoftDiceTrackingEnvCfg()
    cfg.scene.num_envs = int(args_cli.num_envs)
    cfg.sim.device = args_cli.device
    cfg.sim.dt = float(args_cli.dt)
    cfg.sim.render_interval = cfg.decimation

    cfg.configure_from_motion(
        args_cli.motion_file,
        start_frame=args_cli.motion_start_frame,
        playback_speed=args_cli.playback_speed,
        loop=args_cli.motion_loop,
        cube_size=args_cli.cube_size,
        table_length=args_cli.table_length,
        table_width=args_cli.table_width,
        ground_z=args_cli.ground_z,
    )

    env = ManagerBasedRLEnv(cfg=cfg)

    try:
        env.reset()
        motion = env.command_manager.get_term("motion")

        print("\n[ManagerBased parity test]")
        print(f"  num_envs:      {env.num_envs}")
        print(f"  sim dt:        {env.cfg.sim.dt}")
        print(f"  env step dt:   {env.step_dt}")
        print(f"  motion fps:    {motion.motion_fps}")
        print(f"  motion frames: {motion.num_frames}")
        print(f"  action dim:    {sum(env.action_manager.action_term_dim)}")

        count = 0
        while simulation_app.is_running() and count < args_cli.steps:
           
            frame_applied = motion.frame_idx.clone()
            actions = motion.joint_pos.clone() #reference trajectory

            env.step(actions)

            if count % 120 == 0:
                robot = env.scene["robot"]
                cube = env.scene["cube"]
                joint_err = torch.linalg.norm(robot.data.joint_pos.torch[0] - actions[0]).item()
                cube_pos_e = cube.data.root_pos_w.torch[0] - env.scene.env_origins[0]
                cube_ref_e = motion.cube_pos[0]
                cube_pos_err = torch.linalg.norm(cube_pos_e - cube_ref_e).item()
                print(
                    f"[ManagerBased replay] step={count}, frame={int(frame_applied[0].item())}, "
                    f"joint_l2={joint_err:.4f}, cube_pos_err={cube_pos_err:.4f} m"
                )

            count += 1

            # Send the final reference frame once, then stop for a non-looping replay.
            if not args_cli.motion_loop and torch.all(frame_applied >= motion.num_frames - 1):
                break

    finally:
        env.close()


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

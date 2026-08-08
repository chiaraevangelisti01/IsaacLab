"""Replay a Holosoma/OmniRetarget soft-dice trajectory through ManagerBasedRLEnv.

Purpose:
    - deterministic parity check against the original standalone replay
    - verify multiple deformable environments
    - verify deformable instances have independent nodal state
"""

import argparse
import sys
import traceback

from isaaclab.app import AppLauncher


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------

parser = argparse.ArgumentParser()

parser.add_argument("--motion_file", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=6000)
parser.add_argument("--dt", type=float, default= None)

parser.add_argument("--motion_start_frame", type=int, default=0)
parser.add_argument("--motion_loop", action="store_true")
parser.add_argument("--playback_speed", type=float, default=1.0)

parser.add_argument("--cube_size", type=float, default=0.31)
parser.add_argument("--table_length", type=float, default=0.80)
parser.add_argument("--table_width", type=float, default=1.20)
parser.add_argument("--ground_z", type=float, default=0.0)

AppLauncher.add_app_launcher_args(parser)

# GUI by default. Use "--viz none" for headless.
parser.set_defaults(visualizer=["kit"])

args_cli = parser.parse_args()


# -----------------------------------------------------------------------------
# Launch Isaac Sim BEFORE importing Isaac Lab task modules
# -----------------------------------------------------------------------------

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


# -----------------------------------------------------------------------------
# Imports requiring Isaac Sim
# -----------------------------------------------------------------------------

import torch

from isaaclab.envs import ManagerBasedRLEnv

from isaaclab_tasks.manager_based.manipulation.soft_dice_manipulation.soft_dice_env_cfg import (
    SoftDiceTrackingEnvCfg,
    H1_TRACKING_ACTION_SCALE,
    H1_TRACKING_JOINT_NAMES,
)


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

def print_cube_state(env, cube, title="Cube state"):
    """Print world and environment-local cube centers."""

    # Directly use the deformable nodal state instead of relying on a root pose.
    centers_w = cube.data.nodal_pos_w.torch.mean(dim=1)
    centers_e = centers_w - env.scene.env_origins

    print(f"\n[{title}]")
    print(f"deformable instances: {cube.num_instances}")

    print("\nworld centers:")
    for i in range(env.num_envs):
        print(f"  env_{i}: {centers_w[i].detach().cpu().numpy()}")

    print("\nenvironment-local centers:")
    for i in range(env.num_envs):
        print(f"  env_{i}: {centers_e[i].detach().cpu().numpy()}")

    if env.num_envs > 1:
        # Since every environment starts identically, local centers should
        # initially be almost identical.
        local_spread = torch.linalg.norm(
            centers_e - centers_e[0:1],
            dim=-1,
        )

        print(
            "\nmax local-center difference relative to env_0:",
            float(local_spread.max().item()),
            "m",
        )


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    cfg = SoftDiceTrackingEnvCfg()

    # Runtime settings
    cfg.scene.num_envs = int(args_cli.num_envs)
    cfg.scene.replicate_physics = False

    cfg.sim.device = args_cli.device
    if args_cli.dt is not None:
        cfg.sim.dt = float(args_cli.dt)
    cfg.sim.render_interval = cfg.decimation

    # -------------------------------------------------------------------------
    # Motion configuration
    # -------------------------------------------------------------------------

    cfg.commands.motion.motion_file = str(args_cli.motion_file)
    cfg.commands.motion.start_frame = int(args_cli.motion_start_frame)
    cfg.commands.motion.playback_speed = float(args_cli.playback_speed)
    cfg.commands.motion.loop = bool(args_cli.motion_loop)

    # -------------------------------------------------------------------------
    # Scene geometry configuration
    # -------------------------------------------------------------------------

    cfg.cube_size = float(args_cli.cube_size)
    cfg.table_length = float(args_cli.table_length)
    cfg.table_width = float(args_cli.table_width)
    cfg.ground_z = float(args_cli.ground_z)

    # Reads everything above from cfg.
    cfg.configure_from_motion()

    env = ManagerBasedRLEnv(cfg=cfg)

    print("\n[Creating ManagerBasedRLEnv]")
    print(f"  num envs:          {cfg.scene.num_envs}")
    print(f"  replicate physics: {cfg.scene.replicate_physics}")
    print(f"  sim dt:            {cfg.sim.dt}")


    try:

        # ---------------------------------------------------------------------
        # Reset
        # ---------------------------------------------------------------------

        env.reset()

        robot = env.scene["robot"]
        cube = env.scene["cube"]

        motion = env.command_manager.get_term("motion")

        # ---------------------------------------------------------------------
        # Resolve the same controlled joints and action scaling used by the task.
        # ---------------------------------------------------------------------

        controlled_joint_ids, controlled_joint_names = robot.find_joints(
            H1_TRACKING_JOINT_NAMES,
            preserve_order=True,
        )

        if controlled_joint_names != H1_TRACKING_JOINT_NAMES:
            raise RuntimeError(
                "Controlled joint order does not match H1_TRACKING_JOINT_NAMES.\n"
                f"Expected: {H1_TRACKING_JOINT_NAMES}\n"
                f"Resolved: {controlled_joint_names}"
            )

        action_scale = torch.tensor(
            [
                H1_TRACKING_ACTION_SCALE[name]
                for name in controlled_joint_names
            ],
            dtype=torch.float32,
            device=env.device,
        ).unsqueeze(0)

        # Default joint positions are the offset used internally by
        # JointPositionAction when use_default_offset=True.
        q_default_controlled = (
            robot.data.default_joint_pos.torch[:, controlled_joint_ids].clone()
        )

        print("\nControlled joints:")
        for i, name in enumerate(controlled_joint_names):
            print(
                f"  {i:2d}: {name:<28} "
                f"scale={action_scale[0, i].item():.6f} "
                f"default={q_default_controlled[0, i].item():.6f}"
            )
        # ---------------------------------------------------------------------
        # Initial diagnostics
        # ---------------------------------------------------------------------

        print("\n" + "=" * 80)
        print("MANAGER-BASED REFERENCE REPLAY")
        print("=" * 80)

        print(f"num_envs:             {env.num_envs}")
        print(f"sim dt:               {env.cfg.sim.dt}")
        print(f"environment step dt:  {env.step_dt}")
        print(f"decimation:           {env.cfg.decimation}")

        print(f"motion fps:            {motion.motion_fps}")
        print(f"motion frames:         {motion.num_frames}")
        print(f"motion start frame:    {motion.cfg.start_frame}")
        print(f"playback speed:        {motion.cfg.playback_speed}")

        print(f"cube object reference: {motion.has_object_reference}")
        print(f"deformable instances:  {cube.num_instances}")

        print(
            "action dimensions:",
            sum(env.action_manager.action_term_dim),
        )

        print_cube_state(
            env,
            cube,
            title="Cube state immediately after reset",
        )

        if cube.num_instances != env.num_envs:
            raise RuntimeError(
                f"Expected {env.num_envs} deformable instances, "
                f"but PhysX reports {cube.num_instances}."
            )

        # ---------------------------------------------------------------------
        # Reference playback
        # ---------------------------------------------------------------------

        count = 0

        while simulation_app.is_running() and count < args_cli.steps:

            # Reference used for THIS control step.
            frame_applied = motion.frame_idx.clone()

            q_ref = motion.joint_pos.clone()

            if motion.has_object_reference:
                cube_ref_e = motion.cube_pos.clone()
            else:
                cube_ref_e = None

            q_ref_controlled = q_ref[:, controlled_joint_ids]
            reference_action = (q_ref_controlled - q_default_controlled) / action_scale

            env.step(reference_action)

            # -----------------------------------------------------------------
            # Diagnostics
            # -----------------------------------------------------------------

            if count % 120 == 0:

                q_actual = robot.data.joint_pos.torch
                q_actual_controlled = q_actual[:, controlled_joint_ids]

                joint_errors = torch.linalg.norm(
                    q_actual_controlled - q_ref_controlled,
                    dim=-1,
                )

                centers_w = cube.data.nodal_pos_w.torch.mean(dim=1)
                centers_e = centers_w - env.scene.env_origins

                print("\n" + "-" * 70)

                print(
                    f"step={count} | "
                    f"reference frame={int(frame_applied[0].item())}"
                )

                for env_id in range(env.num_envs):

                    msg = (
                        f"env_{env_id}: "
                        f"controlled_joint_L2={joint_errors[env_id].item():.5f}"
                    )

                    if cube_ref_e is not None:
                        cube_pos_err = torch.linalg.norm(
                            centers_e[env_id] - cube_ref_e[env_id]
                        )

                        msg += (
                            f", cube_pos_err="
                            f"{cube_pos_err.item():.5f} m"
                        )

                    print(msg)

                # -------------------------------------------------------------
                # Check whether cloned environments remain independent
                # -------------------------------------------------------------

                if env.num_envs > 1:

                    local_difference = torch.linalg.norm(
                        centers_e - centers_e[0:1],
                        dim=-1,
                    )

                    print(
                        "max local cube difference from env_0:",
                        f"{local_difference.max().item():.8f} m",
                    )

            count += 1

            # -----------------------------------------------------------------
            # End of non-looping reference
            # -----------------------------------------------------------------

            # frame_applied is deliberately used here because it is the frame
            # whose joint target was actually sent during this step.
            if (
                not args_cli.motion_loop
                and torch.all(
                    frame_applied >= motion.num_frames - 1
                )
            ):
                print("\nReached final motion frame.")
                break

        # ---------------------------------------------------------------------
        # Final diagnostics
        # ---------------------------------------------------------------------

        print_cube_state(
            env,
            cube,
            title="Final cube state",
        )

        print("\nReference replay completed.")

    finally:
        env.close()


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

if __name__ == "__main__":

    try:
        main()

    except BaseException as exc:

        print(
            "\n[ERROR] Python exception caught:",
            repr(exc),
            flush=True,
        )

        traceback.print_exc()

        sys.stdout.flush()
        sys.stderr.flush()

        raise

    finally:

        print(
            "[INFO] Closing simulation_app",
            flush=True,
        )

        simulation_app.close()
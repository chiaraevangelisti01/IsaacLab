"""Evaluate an RSL-RL policy on the soft-dice evaluation environment."""

from __future__ import annotations

import argparse
import csv
import importlib.metadata as metadata
import json
import os
import re
import statistics
import subprocess
from datetime import datetime
from pathlib import Path

from isaaclab.app import AppLauncher


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------

parser = argparse.ArgumentParser(
    description="Evaluate a soft-dice RSL-RL checkpoint."
)

parser.add_argument(
    "--task",
    type=str,
    default="Isaac-Soft-Dice-Tracking-H1-Eval-v0",
)
parser.add_argument("--motion_file", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--num_episodes", type=int, default=8)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument(
    "--progress_interval",
    type=int,
    default=0,
    help="Print evaluation progress every N control steps. Use 0 to disable.",
)

# Checkpoint location.
parser.add_argument(
    "--experiment_name",
    type=str,
    default=None,
    help="RSL-RL experiment directory. Defaults to the registered agent config.",
)
parser.add_argument("--load_run", type=str, required=True)
parser.add_argument(
    "--checkpoint",
    type=str,
    default=None,
    help="Checkpoint filename. If omitted, the latest model_*.pt is used.",
)

# Evaluation outputs.
parser.add_argument("--output_dir", type=str, default=None)

# W&B.
parser.add_argument("--wandb_project", type=str, default="soft_dice_thesis")
parser.add_argument("--wandb_entity", type=str, default=None)
parser.add_argument(
    "--wandb_mode",
    type=str,
    choices=["online", "offline", "disabled"],
    default="online",
)

AppLauncher.add_app_launcher_args(parser)

args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


# -----------------------------------------------------------------------------
# Imports requiring Isaac Sim
# -----------------------------------------------------------------------------

import gymnasium as gym
import torch
import wandb
from packaging import version
from rsl_rl.runners import DistillationRunner, OnPolicyRunner

from isaaclab.utils.seed import configure_seed
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg

import isaaclab_tasks  # noqa: F401

from isaaclab_tasks.utils.parse_cfg import get_checkpoint_path, load_cfg_from_registry

from isaaclab_tasks.manager_based.manipulation.soft_dice_manipulation.evaluation import (
    compute_cartesian_trajectory_metrics,
    compute_control_quality_metrics,
    compute_terminal_cube_pose_metrics,
    compute_deformation_metrics,
)
from isaaclab_tasks.manager_based.manipulation.soft_dice_manipulation.evaluation.visualization import (
    compute_main_metric_summary,
    log_comparison_timeseries,
    log_evaluation_visualizations,
)


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

def get_git_commit() -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            text=True,
        ).strip()
    except Exception:
        return None


def resolve_checkpoint(experiment_name: str) -> str:
    log_root = os.path.abspath(os.path.join("logs", "rsl_rl", experiment_name))
    run_dir = os.path.join(log_root, args_cli.load_run)

    if not os.path.isdir(run_dir):
        raise FileNotFoundError(f"Run directory does not exist: {run_dir}")

    if args_cli.checkpoint is not None:
        checkpoint = args_cli.checkpoint
        path = (
            checkpoint
            if os.path.isabs(checkpoint)
            else os.path.join(run_dir, checkpoint)
        )

        if not os.path.isfile(path):
            raise FileNotFoundError(f"Checkpoint does not exist: {path}")

        return path

    return get_checkpoint_path(
        log_path=log_root,
        run_dir=re.escape(args_cli.load_run) + "$",
        checkpoint=r"model_.*\.pt",
    )


def write_results(
    output_dir: str,
    config: dict,
    records: list[dict],
    summary: dict,
) -> tuple[str, str]:
    os.makedirs(output_dir, exist_ok=True)

    csv_path = os.path.join(output_dir, "episodes.csv")
    json_path = os.path.join(output_dir, "evaluation.json")

    if records:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(records[0].keys()))
            writer.writeheader()
            writer.writerows(records)

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "config": config,
                "summary": summary,
                "episodes": records,
            },
            f,
            indent=2,
        )

    return csv_path, json_path

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    if args_cli.num_episodes <= 0:
        raise ValueError("--num_episodes must be > 0.")

    if args_cli.num_envs <= 0:
        raise ValueError("--num_envs must be > 0.")

    # -------------------------------------------------------------------------
    # Load registered configurations.
    # -------------------------------------------------------------------------

    env_cfg = load_cfg_from_registry(args_cli.task, "env_cfg_entry_point")
    agent_cfg = load_cfg_from_registry(args_cli.task, "rsl_rl_cfg_entry_point")

    installed_version = metadata.version("rsl-rl-lib")
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)

    experiment_name = (
        args_cli.experiment_name
        if args_cli.experiment_name is not None
        else agent_cfg.experiment_name
    )

    resume_path = resolve_checkpoint(experiment_name)
    checkpoint_name = Path(resume_path).stem

    # -------------------------------------------------------------------------
    # Evaluation environment configuration.
    # -------------------------------------------------------------------------

    env_cfg.scene.num_envs = min(
        int(args_cli.num_envs),
        int(args_cli.num_episodes),
    )
    env_cfg.seed = int(args_cli.seed)

    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
        agent_cfg.device = args_cli.device

    env_cfg.commands.motion.motion_file = str(args_cli.motion_file)
    env_cfg.commands.motion.loop = False

    # Resolve trajectory-dependent scene geometry.
    env_cfg.validate_config()

    # -------------------------------------------------------------------------
    # Output directory.
    # -------------------------------------------------------------------------

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    if args_cli.output_dir is None:
        output_dir = os.path.join(
            "logs",
            "evaluation",
            experiment_name,
            args_cli.load_run,
            checkpoint_name,
            timestamp,
        )
    else:
        output_dir = os.path.abspath(args_cli.output_dir)

    os.makedirs(output_dir, exist_ok=True)

    # -------------------------------------------------------------------------
    # Evaluation metadata.
    # -------------------------------------------------------------------------

    eval_config = {
        "task": args_cli.task,
        "evaluation_suite": "nominal",
        "training_experiment": experiment_name,
        "training_run": args_cli.load_run,
        "checkpoint": os.path.abspath(resume_path),
        "checkpoint_name": checkpoint_name,
        "motion_file": os.path.abspath(args_cli.motion_file),
        "num_envs": env_cfg.scene.num_envs,
        "num_episodes": int(args_cli.num_episodes),
        "seed": int(args_cli.seed),
        "deterministic": bool(args_cli.deterministic),
        "git_commit": get_git_commit(),
    }

    # -------------------------------------------------------------------------
    # W&B evaluation run.
    # -------------------------------------------------------------------------

    wandb_run_name = (
        f"eval_{args_cli.load_run}_{checkpoint_name}_seed{args_cli.seed}"
    )

    wb_run = wandb.init(
        project=args_cli.wandb_project,
        entity=args_cli.wandb_entity,
        name=wandb_run_name,
        group=f"{args_cli.load_run}_{checkpoint_name}",
        job_type="evaluation",
        mode=args_cli.wandb_mode,
        dir=output_dir,
        config=eval_config,
        tags=["evaluation", "soft-dice", "nominal"],
    )

    # -------------------------------------------------------------------------
    # Environment.
    # -------------------------------------------------------------------------

    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    print(f"[INFO] Loading checkpoint: {resume_path}")

    # -------------------------------------------------------------------------
    # Policy.
    # -------------------------------------------------------------------------

    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(
            env,
            agent_cfg.to_dict(),
            log_dir=None,
            device=agent_cfg.device,
        )
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(
            env,
            agent_cfg.to_dict(),
            log_dir=None,
            device=agent_cfg.device,
        )
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")

    # Follow the ordering used by Isaac Lab's play script.
    if args_cli.deterministic:
        configure_seed(env_cfg.seed, True)

    runner.load(resume_path)

    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # Older RSL-RL releases reset the network rather than
    # the inference-policy object.
    if version.parse(installed_version) < version.parse("4.0.0"):
        if hasattr(runner.alg.policy, "reset"):
            policy_reset_owner = runner.alg.policy
        else:
            policy_reset_owner = runner.alg.actor_critic
    else:
        policy_reset_owner = policy

    # -------------------------------------------------------------------------
    # Evaluation loop.
    # -------------------------------------------------------------------------
    motion = env.unwrapped.command_manager.get_term("motion")
    evaluation_step = 0

    obs = env.get_observations()

    action_cfg = env.unwrapped.cfg.actions.joint_pos
    _, controlled_joint_names = env.unwrapped.scene["robot"].find_joints(
        action_cfg.joint_names,
        preserve_order=action_cfg.preserve_order,
    )

    episode_steps = torch.zeros(
        env.num_envs,
        dtype=torch.long,
        device=env.device,
    )

    records: list[dict] = []
    trajectory_records: list[dict] = []

    try:
        while simulation_app.is_running() and len(records) < args_cli.num_episodes:
            with torch.inference_mode():
                actions = policy(obs)
                obs, _, dones, extras = env.step(actions)

            episode_steps += 1
            evaluation_step += 1

            if (
                args_cli.progress_interval > 0
                and evaluation_step % args_cli.progress_interval == 0
            ):
                print(
                    f"[EVAL PROGRESS] "
                    f"step={evaluation_step} "
                    f"motion_frame={int(motion.frame_idx[0].item())}"
                )

            done_ids = (dones > 0).nonzero(as_tuple=False).flatten()
            terminal = extras["evaluation"]

            if done_ids.numel() > 0:
                termination_manager = env.unwrapped.termination_manager


                for env_id_tensor in done_ids:
                    if len(records) >= args_cli.num_episodes:
                        break

                    env_id = int(env_id_tensor.item())
                    finished = bool(terminal["motion_finished"][env_id].item())
                    timed_out = bool(terminal["time_out"][env_id].item())  

                    if finished:
                        termination = "motion_finished"
                    elif timed_out:
                        termination = "time_out"
                    else:
                        termination = "unknown"

                    steps = int(episode_steps[env_id].item())

                    record = {
                        "episode_id": len(records),
                        "env_id": env_id,
                        "seed": int(args_cli.seed),
                        "episode_steps": steps,
                        "duration_s": steps * float(env.unwrapped.step_dt),
                        "termination": termination,
                        "motion_finished": finished,
                        "hard_timeout": timed_out,
                    }

                    records.append(record)

                    print(
                        f"[EVAL] episode={record['episode_id']} "
                        f"env={env_id} steps={steps} "
                        f"termination={termination}"
                    )

                    if not bool(terminal["terminal_valid"][env_id].item()):
                        raise RuntimeError(
                            f"Missing terminal snapshot for env {env_id}."
                        )

                    terminal_steps = int(
                        terminal["episode_steps"][env_id].item()
                    )
                    terminal_frame = int(
                        terminal["motion_frame"][env_id].item()
                    )

                    cube_pos = terminal["cube_pos_e"][env_id].detach().cpu()
                    cube_quat = terminal["cube_quat_xyzw"][env_id].detach().cpu()
                    reference_cube_pos = (
                        terminal["reference_cube_pos_e"][env_id].detach().cpu()
                    )
                    reference_cube_quat = (
                        terminal["reference_cube_quat_xyzw"][env_id].detach().cpu()
                    )

                    if terminal_steps != steps:
                        raise RuntimeError(
                            f"Episode-step mismatch for env {env_id}: "
                            f"runner={steps}, recorder={terminal_steps}."
                        )

                    record.update(
                        {
                            "terminal_motion_frame": terminal_frame,

                            "cube_x_m": float(cube_pos[0]),
                            "cube_y_m": float(cube_pos[1]),
                            "cube_z_m": float(cube_pos[2]),

                            "cube_qx": float(cube_quat[0]),
                            "cube_qy": float(cube_quat[1]),
                            "cube_qz": float(cube_quat[2]),
                            "cube_qw": float(cube_quat[3]),

                            "reference_cube_x_m": float(reference_cube_pos[0]),
                            "reference_cube_y_m": float(reference_cube_pos[1]),
                            "reference_cube_z_m": float(reference_cube_pos[2]),

                            "reference_cube_qx": float(reference_cube_quat[0]),
                            "reference_cube_qy": float(reference_cube_quat[1]),
                            "reference_cube_qz": float(reference_cube_quat[2]),
                            "reference_cube_qw": float(reference_cube_quat[3]),
                        }
                    )

                    # ---------------------------------------------------------
                    # Trajectory buffers.
                    # ---------------------------------------------------------

                    trajectory = terminal["trajectory"]

                    motion_frames = (
                        trajectory["motion_frame"][env_id, :terminal_steps]
                        .detach()
                        .cpu()
                    )
                    cube_position_delta_m = (
                        trajectory["cube_position_delta_m"][env_id, :terminal_steps]
                        .detach()
                        .cpu()
                    )
                    body_position_error_m = (
                        trajectory["body_position_error_m"][env_id, :terminal_steps]
                        .detach()
                        .cpu()
                    )
                    body_orientation_error_rad = (
                        trajectory["body_orientation_error_rad"][
                            env_id, :terminal_steps
                        ]
                        .detach()
                        .cpu()
                    )
                    hand_position_error_m = (
                        trajectory["hand_position_error_m"][env_id, :terminal_steps]
                        .detach()
                        .cpu()
                    )
                    cube_position_error_m = (
                        trajectory["cube_position_error_m"][env_id, :terminal_steps]
                        .detach()
                        .cpu()
                    )
                    cube_xy_position_error_m = (
                        trajectory["cube_xy_position_error_m"][
                            env_id, :terminal_steps
                        ]
                        .detach()
                        .cpu()
                    )
                    cube_orientation_error_rad = (
                        trajectory["cube_orientation_error_rad"][
                            env_id, :terminal_steps
                        ]
                        .detach()
                        .cpu()
                    )
                    joint_torque_nm = (
                        trajectory["joint_torque_nm"][env_id, :terminal_steps]
                        .detach()
                        .cpu()
                    )
                    action_delta = (
                        trajectory["action_delta"][env_id, :terminal_steps]
                        .detach()
                        .cpu()
                    )

                    #---------------------------------------------
                    # Deformation buffers 
                    #-----------------------------------------------

                    deformation_rms_m = (
                        trajectory["deformation_rms_m"][env_id, :terminal_steps]
                        .detach()
                        .cpu()
                    )

                    deformation_p95_m = (
                        trajectory["deformation_p95_m"][env_id, :terminal_steps]
                        .detach()
                        .cpu()
                    )

                    deformation_max_m = (
                        trajectory["deformation_max_m"][env_id, :terminal_steps]
                        .detach()
                        .cpu()
                    )

                    relative_extent_change = (
                        trajectory["relative_extent_change"][env_id, :terminal_steps]
                        .detach()
                        .cpu()
                    )
                    
                    # ---------------------------------------------------------
                    # Cartesian trajectory metrics.
                    # ---------------------------------------------------------

                    trajectory_metrics = compute_cartesian_trajectory_metrics(
                        body_position_error_m=body_position_error_m,
                        body_orientation_error_rad=body_orientation_error_rad,
                        hand_position_error_m=hand_position_error_m,
                        cube_position_error_m=cube_position_error_m,
                        cube_xy_position_error_m=cube_xy_position_error_m,
                        cube_orientation_error_rad=cube_orientation_error_rad,
                    )

                    record.update(
                        {
                            key: float(value.item())
                            for key, value in trajectory_metrics.items()
                        }
                    )

                    # ---------------------------------------------------------
                    # Terminal cube pose metrics.
                    # ---------------------------------------------------------

                    pose_metrics = compute_terminal_cube_pose_metrics(
                        reference_pos=reference_cube_pos,
                        current_pos=cube_pos,
                        reference_quat=reference_cube_quat,
                        current_quat=cube_quat,
                    )

                    record.update(
                        {
                            key: float(value.item())
                            for key, value in pose_metrics.items()
                        }
                    )

                    # ---------------------------------------------------------
                    # Control-quality metrics.
                    # ---------------------------------------------------------

                    control_metrics = compute_control_quality_metrics(
                        joint_torque_nm=joint_torque_nm,
                        action_delta=action_delta,
                        joint_names=controlled_joint_names,
                    )

                    record.update(
                        {
                            key: float(value.item())
                            for key, value in control_metrics.items()
                        }
                    )

                    # ---------------------------------------------------------
                    # Deformation metrics.
                    # ---------------------------------------------------------

                    deformation_metrics = compute_deformation_metrics(
                        deformation_rms_m=deformation_rms_m,
                        deformation_p95_m=deformation_p95_m,
                        deformation_max_m=deformation_max_m,
                        relative_extent_change=relative_extent_change,
                    )

                    record.update(
                        {
                            key: float(value.item())
                            for key, value in deformation_metrics.items()
                        }
                    )

                    # ---------------------------------------------------------
                    # Raw per-frame data for visualization.
                    # ---------------------------------------------------------

                    trajectory_records.append(
                        {
                            "episode_id": record["episode_id"],
                            "motion_frame": motion_frames.numpy().copy(),
                            "body_position_error_m": body_position_error_m.numpy().copy(),
                            "body_orientation_error_rad": body_orientation_error_rad.numpy().copy(),
                            "hand_position_error_m": hand_position_error_m.numpy().copy(),
                            "cube_position_delta_m": cube_position_delta_m.numpy().copy(),
                            "cube_position_error_m": cube_position_error_m.numpy().copy(),
                            "cube_xy_position_error_m": cube_xy_position_error_m.numpy().copy(),
                            "cube_orientation_error_rad": cube_orientation_error_rad.numpy().copy(),
                            "joint_torque_nm": joint_torque_nm.numpy().copy(),
                            "action_delta": action_delta.numpy().copy(),
                            "deformation_rms_m": deformation_rms_m.numpy().copy(),
                            "deformation_p95_m": deformation_p95_m.numpy().copy(),
                            "deformation_max_m": deformation_max_m.numpy().copy(),
                            "relative_extent_change": relative_extent_change.numpy().copy(),
                        }
                    )

                episode_steps[done_ids] = 0

            policy_reset_owner.reset(dones)

    finally:
        env.close()

    # -------------------------------------------------------------------------
    # Aggregate results.
    # -------------------------------------------------------------------------

    durations = [row["duration_s"] for row in records]
    completed = sum(row["motion_finished"] for row in records)

    summary = {
        "num_episodes": len(records),
        "motion_completion_rate": completed / len(records) if records else 0.0,
        "mean_duration_s": statistics.mean(durations) if durations else 0.0,
        "std_duration_s": (
            statistics.pstdev(durations)
            if len(durations) > 1
            else 0.0
        ),
    }

    summary.update(compute_main_metric_summary(records))

    # -------------------------------------------------------------------------
    # Local outputs.
    # -------------------------------------------------------------------------

    csv_path, json_path = write_results(
        output_dir=output_dir,
        config=eval_config,
        records=records,
        summary=summary,
    )

    print(f"\n[INFO] Evaluation results: {output_dir}")

    # -------------------------------------------------------------------------
    # W&B outputs.
    # -------------------------------------------------------------------------

    for key, value in summary.items():
        wb_run.summary[f"evaluation_summary/{key}"] = value

    log_evaluation_visualizations(
        run=wb_run,
        records=records,
        trajectory_records=trajectory_records,
        joint_names=controlled_joint_names,
    )

    log_comparison_timeseries(
        run=wb_run,
        trajectory_records=trajectory_records,
        joint_names=controlled_joint_names,
    )


    artifact = wandb.Artifact(
        name=f"evaluation-results-{wb_run.id}",
        type="evaluation-results",
        metadata=eval_config,
    )

    artifact.add_file(csv_path)
    artifact.add_file(json_path)

    wb_run.log_artifact(artifact)
    wb_run.finish()


if __name__ == "__main__":
    try:
        main()

    except BaseException as exc:
        import traceback

        print("[FATAL DEBUG]", type(exc).__name__, repr(exc))
        traceback.print_exc()
        raise

    finally:
        simulation_app.close()
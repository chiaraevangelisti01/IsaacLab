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
motion_group = parser.add_mutually_exclusive_group(required=True)

motion_group.add_argument(
    "--motion_file",
    type=str,
    default=None,
    help="Evaluate one reference trajectory.",
)

motion_group.add_argument(
    "--motion_dir",
    type=str,
    default=None,
    help="Evaluate all matching trajectories in a directory.",
)

parser.add_argument(
    "--motion_pattern",
    type=str,
    default="Traj*_fps50.npz",
    help="Glob pattern used with --motion_dir.",
)

parser.add_argument(
    "--episodes_per_motion",
    type=int,
    default=None,
    help=(
        "Number of accepted episodes for each trajectory "
        "when using --motion_dir."
    ),
)
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

#Evaluatios parameters
parser.add_argument(
    "--high_deformation_threshold_mm",
    type=float,
    default=10.0,
    help="RMS deformation threshold used to classify high-deformation frames.",
)
parser.add_argument(
    "--position_improvement_threshold_mm",
    type=float,
    default=5.0,
    help="Minimum landing-position improvement required to classify an episode as meaningfully better or worse.",
)
parser.add_argument(
    "--orientation_improvement_threshold_deg",
    type=float,
    default=2.0,
    help="Minimum landing-orientation improvement required to classify an episode as meaningfully better or worse.",
)
parser.add_argument(
    "--sustained_high_deformation_fraction",
    type=float,
    default=0.20,
    help="Episode is flagged as sustained deformation if this fraction of frames exceeds the high-deformation RMS threshold.",
)
parser.add_argument(
    "--extreme_local_deformation_threshold_mm",
    type=float,
    default=40.0,
    help="Episode is flagged if maximum local nodal deformation exceeds this value.",
)
parser.add_argument(
    "--torque_utilization_threshold",
    type=float,
    default=0.90,
    help="Episode is flagged if any controlled joint reaches this fraction of its effort limit.",
)

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
    compute_terminal_cube_landing_metrics,
    compute_terminal_task_success_metrics,
    compute_deformation_metrics,
)
from isaaclab_tasks.manager_based.manipulation.soft_dice_manipulation.evaluation.visualization import (
    compute_main_metric_summary,
    compute_per_motion_metric_summary,
    log_evaluation_visualizations,
)

from isaaclab_tasks.manager_based.manipulation.soft_dice_manipulation.evaluation.robustness import (
    ROBUSTNESS_CONDITIONS,
    robustness_record_from_terminal,
    select_condition_records,
)

from isaaclab_tasks.manager_based.manipulation.soft_dice_manipulation.evaluation.robustness_visualization import (
    log_robustness_visualizations,
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
    per_motion_summary: dict,
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
                "per_motion_summary": per_motion_summary,
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
    if args_cli.num_envs <= 0:
        raise ValueError(
            "--num_envs must be > 0."
        )
    if args_cli.high_deformation_threshold_mm <= 0.0:
        raise ValueError("--high_deformation_threshold_mm must be > 0.")
    if args_cli.position_improvement_threshold_mm < 0.0:
        raise ValueError("--position_improvement_threshold_mm must be >= 0.")
    if args_cli.orientation_improvement_threshold_deg < 0.0:
        raise ValueError("--orientation_improvement_threshold_deg must be >= 0.")
    if not 0.0 <= args_cli.sustained_high_deformation_fraction <= 1.0:
        raise ValueError("--sustained_high_deformation_fraction must be between 0 and 1.")
    if args_cli.extreme_local_deformation_threshold_mm <= 0.0:
        raise ValueError("--extreme_local_deformation_threshold_mm must be > 0.")
    if not 0.0 < args_cli.torque_utilization_threshold <= 1.0:
        raise ValueError("--torque_utilization_threshold must be in (0, 1].")
    
    position_improvement_threshold_m = 0.001 * float(args_cli.position_improvement_threshold_mm)
    orientation_improvement_threshold_deg = float(args_cli.orientation_improvement_threshold_deg)
    severe_deformation_peak_threshold_mm = 2.0 * float(args_cli.high_deformation_threshold_mm)

    num_conditions = len(ROBUSTNESS_CONDITIONS)
    
    if args_cli.motion_file is not None:
        if args_cli.num_episodes <= 0:
            raise ValueError("--num_episodes must be > 0.")
 
        if args_cli.num_episodes % num_conditions != 0:
            raise ValueError(
                f"--num_episodes must be a multiple of {num_conditions} "
                "to balance robustness conditions equally."
            )
        motion_paths = [Path(args_cli.motion_file).expanduser().resolve()]

        target_episodes_per_motion = {motion_paths[0].stem: int(args_cli.num_episodes)}

    else:

        if (
            args_cli.episodes_per_motion is None
            or args_cli.episodes_per_motion <= 0
        ):
            raise ValueError(
                "--episodes_per_motion must be > 0 "
                "when using --motion_dir."
            )

        if args_cli.episodes_per_motion % num_conditions != 0:
            raise ValueError(
                f"--episodes_per_motion must be a multiple of {num_conditions} "
                f"to balance the robustness conditions equally. "
                f"Got {args_cli.episodes_per_motion}."
            )
        
        motion_dir = (Path(args_cli.motion_dir).expanduser().resolve())

        motion_paths = sorted(
            path
            for path in motion_dir.glob(
                args_cli.motion_pattern
            )
            if path.is_file()
        )

        if not motion_paths:
            raise FileNotFoundError(
                "No trajectories matching "
                f"{args_cli.motion_pattern!r} "
                f"in {motion_dir}"
            )

        target_episodes_per_motion = {
            path.stem: int(
                args_cli.episodes_per_motion
            )
            for path in motion_paths
        }


    total_target_episodes = sum(target_episodes_per_motion.values())

    print(
        f"[INFO] Evaluating {len(motion_paths)} motions, "
        f"{total_target_episodes} total accepted episodes."
    )

    for path in motion_paths:
        print(f"  - {path.name}")

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
        int(total_target_episodes),
    )
    env_cfg.seed = int(args_cli.seed)

    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
        agent_cfg.device = args_cli.device

    if args_cli.motion_file is not None:

        env_cfg.commands.motion.motion_file = str(
            motion_paths[0]
        )

        env_cfg.commands.motion.motion_dir = ""

    else:

        env_cfg.commands.motion.motion_file = ""

        env_cfg.commands.motion.motion_dir = str(
            Path(args_cli.motion_dir)
            .expanduser()
            .resolve()
        )

        env_cfg.commands.motion.motion_pattern = (args_cli.motion_pattern)

    env_cfg.commands.motion.motion_sampling = ("balanced")
    env_cfg.commands.motion.loop = False

    # Resolve trajectory-dependent scene geometry.
    env_cfg.validate_config()

    # Get landing region information from the registered task
    landing_position_cfg = getattr(env_cfg.rewards, "landing_position", None)

    if landing_position_cfg is not None:
        landing_center_xy = tuple(landing_position_cfg.params["goal_xy"])
        landing_radius = float(landing_position_cfg.params["radius"])
        orientation_success_threshold_deg = 5
    else:
        landing_center_xy = None
        landing_radius = None
        orientation_success_threshold_deg = None

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
        "evaluation_suite": "mixed_robustness",
        "training_experiment": experiment_name,
        "training_run": args_cli.load_run,
        "checkpoint": os.path.abspath(resume_path),
        "checkpoint_name": checkpoint_name,
        "num_envs": env_cfg.scene.num_envs,
        "motion_file": str(motion_paths[0]) if args_cli.motion_file is not None else None,
        "motion_dir": str(motion_dir) if args_cli.motion_dir is not None else None,
        "motion_pattern": args_cli.motion_pattern if args_cli.motion_dir is not None else None,
        "num_episodes": total_target_episodes,
        "target_episodes_per_motion": target_episodes_per_motion,
        "seed": int(args_cli.seed),
        "deterministic": bool(args_cli.deterministic),
        "git_commit": get_git_commit(),
        "high_deformation_threshold_mm": float(args_cli.high_deformation_threshold_mm),
        "severe_deformation_peak_threshold_mm": severe_deformation_peak_threshold_mm,
        "sustained_high_deformation_fraction": float(args_cli.sustained_high_deformation_fraction),
        "extreme_local_deformation_threshold_mm": float(args_cli.extreme_local_deformation_threshold_mm),
        "torque_utilization_threshold" : float(args_cli.torque_utilization_threshold),
    }

    if landing_center_xy is not None:
        eval_config.update({
            "landing_center_xy": landing_center_xy,
            "landing_radius_m": landing_radius,
            "orientation_success_threshold_deg": orientation_success_threshold_deg,
            "position_improvement_threshold_mm": float(args_cli.position_improvement_threshold_mm),
            "orientation_improvement_threshold_deg": orientation_improvement_threshold_deg,
        })

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
        tags=["evaluation", "soft-dice", "mixed_robustness"],
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
    robot = env.unwrapped.scene["robot"]

    controlled_joint_ids, controlled_joint_names = robot.find_joints(
        action_cfg.joint_names,
        preserve_order=action_cfg.preserve_order,
    )

    controlled_joint_effort_limits_nm = (robot.data.joint_effort_limits.torch[0,controlled_joint_ids].detach().cpu())

    joint_effort_limits_config = {
        joint_name: float(limit)
        for joint_name, limit in zip(
            controlled_joint_names,
            controlled_joint_effort_limits_nm.tolist(),
        )
    }

    eval_config["joint_effort_limits_nm"] = joint_effort_limits_config

    if wb_run is not None:
        wb_run.config.update(
            {"joint_effort_limits_nm": joint_effort_limits_config},
            allow_val_change=True,
        )

    episode_steps = torch.zeros(
        env.num_envs,
        dtype=torch.long,
        device=env.device,
    )

    records: list[dict] = []
    trajectory_records: list[dict] = []
    accepted_episodes_per_motion = {name: 0 for name in target_episodes_per_motion}

    try:
        while simulation_app.is_running() and len(records) < total_target_episodes:
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
                    if len(records) >= total_target_episodes:
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

                    motion_id = int(terminal["motion_id"][env_id].item())
                    motion_name = motion.motion_name(motion_id)

                    if motion_name not in target_episodes_per_motion:
                        raise RuntimeError(f"Unexpected motion in evaluation: {motion_name}")

                    if accepted_episodes_per_motion[motion_name] >= target_episodes_per_motion[motion_name]:
                        continue

                    record = {
                        "episode_id": len(records),
                        "env_id": env_id,
                        "motion_id": motion_id,
                        "motion_name": motion_name,
                        "seed": int(args_cli.seed),
                        "episode_steps": steps,
                        "duration_s": steps * float(env.unwrapped.step_dt),
                        "termination": termination,
                        "motion_finished": finished,
                        "hard_timeout": timed_out,
                    }

                    records.append(record)
                    accepted_episodes_per_motion[motion_name] += 1

                    print(
                        f"[EVAL] episode={record['episode_id']} motion={motion_name} "
                        f"count={accepted_episodes_per_motion[motion_name]}/{target_episodes_per_motion[motion_name]} "
                        f"env={env_id} steps={steps} termination={termination}"
                    )

                    if not bool(terminal["terminal_valid"][env_id].item()):
                        raise RuntimeError(
                            f"Missing terminal snapshot for env {env_id}."
                        )
                    record.update(
                        robustness_record_from_terminal(
                            terminal=terminal,
                            env_id=env_id,
                        )
                    )
                    terminal_steps = int(
                        terminal["episode_steps"][env_id].item()
                    )
                    terminal_frame = int(
                        terminal["motion_frame"][env_id].item()
                    )

                    cube_pos = terminal["cube_pos_e"][env_id].detach().cpu()
                    cube_quat = terminal["cube_quat_xyzw"][env_id].detach().cpu()
                    cube_pos_r0 = terminal["cube_pos_r0"][env_id].detach().cpu()
                    cube_quat_r0 = terminal["cube_quat_r0_xyzw"][env_id].detach().cpu()
                    final_reference_cube_pos_r0 = terminal["final_reference_cube_pos_r0"][env_id].detach().cpu()
                    final_reference_cube_quat_r0 = terminal["final_reference_cube_quat_r0_xyzw"][env_id].detach().cpu()
                    reference_cube_pos = terminal["reference_cube_pos_e"][env_id].detach().cpu()
                    reference_cube_quat = terminal["reference_cube_quat_xyzw"][env_id].detach().cpu()
                    

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

                    motion_frames = trajectory["motion_frame"][env_id, :terminal_steps].detach().cpu()
                    motion_phase = trajectory["phase"][env_id,:terminal_steps].detach().cpu()
                    body_position_error_m = trajectory["body_position_error_m"][env_id, :terminal_steps].detach().cpu()
                    body_orientation_error_rad = trajectory["body_orientation_error_rad"][env_id, :terminal_steps].detach().cpu()
                    hand_position_error_m = trajectory["hand_position_error_m"][env_id, :terminal_steps].detach().cpu()
                    cube_xy_position_error_m = trajectory["cube_xy_position_error_m"][env_id, :terminal_steps].detach().cpu()
                    cube_orientation_error_rad = trajectory["cube_orientation_error_rad"][env_id, :terminal_steps].detach().cpu()
                    joint_torque_nm = trajectory["joint_torque_nm"][env_id, :terminal_steps].detach().cpu()
                    action_delta = (trajectory["action_delta"][env_id, :terminal_steps].detach().cpu())
                    position_landing_start_phase_tensor = terminal["position_landing_start_phase"][env_id].detach().cpu()
                    orientation_landing_start_phase_tensor = terminal["orientation_landing_start_phase"][env_id].detach().cpu()

                    phase_tracking_available = (
                        torch.isfinite(position_landing_start_phase_tensor).item()
                        and torch.isfinite(orientation_landing_start_phase_tensor).item()
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
                    trajectory_metric_kwargs = {}

                    if phase_tracking_available:
                        position_landing_start_phase = float(position_landing_start_phase_tensor.item())
                        orientation_landing_start_phase = float(orientation_landing_start_phase_tensor.item())
                        robot_tracking_relaxation_start_phase = max(position_landing_start_phase, orientation_landing_start_phase)

                        record["position_landing_start_phase"] = position_landing_start_phase
                        record["orientation_landing_start_phase"] = orientation_landing_start_phase
                        record["robot_tracking_relaxation_start_phase"] = robot_tracking_relaxation_start_phase

                        trajectory_metric_kwargs = {
                            "phase": motion_phase,
                            "position_landing_start_phase": position_landing_start_phase,
                            "orientation_landing_start_phase": orientation_landing_start_phase,
                        }

                    trajectory_metrics = compute_cartesian_trajectory_metrics(
                        body_position_error_m=body_position_error_m,
                        body_orientation_error_rad=body_orientation_error_rad,
                        hand_position_error_m=hand_position_error_m,
                        cube_xy_position_error_m=cube_xy_position_error_m,
                        cube_orientation_error_rad=cube_orientation_error_rad,
                        **trajectory_metric_kwargs,
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
                    record.update({key: float(value.item())for key, value in pose_metrics.items()})

                    if landing_center_xy is not None:
                        record["final_cube_x_r0_m"] = float(cube_pos_r0[0].item())
                        record["final_cube_y_r0_m"] = float(cube_pos_r0[1].item())
                        record["reference_final_cube_x_r0_m"] = float(final_reference_cube_pos_r0[0].item())
                        record["reference_final_cube_y_r0_m"] = float(final_reference_cube_pos_r0[1].item())

                        landing_metrics = compute_terminal_cube_landing_metrics(
                            reference_pos_r0=final_reference_cube_pos_r0,
                            current_pos_r0=cube_pos_r0,
                            reference_quat_r0=final_reference_cube_quat_r0,
                            current_quat_r0=cube_quat_r0,
                            landing_center_xy=landing_center_xy,
                            landing_radius=landing_radius,
                            position_improvement_threshold_m=position_improvement_threshold_m,
                            orientation_improvement_threshold_deg=orientation_improvement_threshold_deg,
                        )
                        record.update({key: float(value.item()) for key, value in landing_metrics.items()})

                        success_metrics = compute_terminal_task_success_metrics(
                            reference_landing_xy_error_m=landing_metrics["reference_landing_xy_error_m"],
                            final_landing_xy_error_m=landing_metrics["final_landing_xy_error_m"],
                            reference_landing_orientation_error_deg=landing_metrics[
                                "reference_landing_orientation_error_deg"
                            ],
                            final_landing_orientation_error_deg=landing_metrics[
                                "final_landing_orientation_error_deg"
                            ],
                            final_top_face_correct=landing_metrics["final_top_face_correct"],
                            motion_finished=torch.tensor(finished, dtype=torch.bool),
                            orientation_threshold_deg=orientation_success_threshold_deg,
                        )

                        record.update({
                            key: float(value.item())
                            for key, value in success_metrics.items()
                        })


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
                        high_deformation_threshold_m = 0.001 * float(args_cli.high_deformation_threshold_mm),
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
                            "joint_torque_nm": joint_torque_nm.numpy().copy(),
                            "deformation_rms_m": deformation_rms_m.numpy().copy(),
                            "deformation_p95_m": deformation_p95_m.numpy().copy(),
                            "deformation_max_m": deformation_max_m.numpy().copy(),
                        }
                    )

                episode_steps[done_ids] = 0

            policy_reset_owner.reset(dones)

    finally:
        env.close()

    

    # -------------------------------------------------------------------------
    # Aggregate results.
    # -------------------------------------------------------------------------
    for motion_name, target_count in target_episodes_per_motion.items():
        expected_per_condition = target_count // num_conditions

        for condition in ROBUSTNESS_CONDITIONS:
            actual_count = sum(
                record["motion_name"] == motion_name
                and record["robustness_condition"] == condition
                for record in records
            )

            if actual_count != expected_per_condition:
                raise RuntimeError(
                    f"Unbalanced evaluation for {motion_name}, "
                    f"condition={condition}: expected "
                    f"{expected_per_condition}, got {actual_count}."
                )
        
    nominal_records, nominal_trajectory_records = (
        select_condition_records(
            records=records,
            trajectory_records=trajectory_records,
            condition="nominal",
        )
    )

    if not nominal_records:
        raise RuntimeError(
            "Mixed evaluation produced no nominal episodes."
        )

    nominal_durations = [row["duration_s"] for row in nominal_records]
    nominal_completed = sum(row["motion_finished"] for row in nominal_records)

    summary = {
        "num_total_episodes": len(records),
        "num_nominal_episodes": len(nominal_records),
        "motion_completion_rate": nominal_completed / len(nominal_records),
        "mean_duration_s": statistics.mean(nominal_durations),
        "std_duration_s": statistics.pstdev(nominal_durations) if len(nominal_durations) > 1 else 0.0,
    }

    summary.update(compute_main_metric_summary(nominal_records))
    per_motion_summary = compute_per_motion_metric_summary(nominal_records)

    # -------------------------------------------------------------------------
    # Local outputs.
    # -------------------------------------------------------------------------

    csv_path, json_path = write_results(
        output_dir=output_dir,
        config=eval_config,
        records=records,
        summary=summary,
        per_motion_summary=per_motion_summary,
    )

    print(f"\n[INFO] Evaluation results: {output_dir}")

    # -------------------------------------------------------------------------
    # W&B outputs.
    # -------------------------------------------------------------------------

    for key, value in summary.items():
        wb_run.summary[f"evaluation_summary/{key}"] = value

    log_evaluation_visualizations(
        run=wb_run,
        records=nominal_records,
        trajectory_records=nominal_trajectory_records,
        joint_names=controlled_joint_names,
        joint_effort_limits_nm=(controlled_joint_effort_limits_nm.numpy()),
        torque_utilization_threshold=float(args_cli.torque_utilization_threshold),
        per_motion_summary=per_motion_summary,
        high_deformation_threshold_mm=float(args_cli.high_deformation_threshold_mm),
        severe_deformation_peak_threshold_mm=severe_deformation_peak_threshold_mm,
        sustained_high_deformation_fraction=float(args_cli.sustained_high_deformation_fraction),
        extreme_local_deformation_threshold_mm=float(args_cli.extreme_local_deformation_threshold_mm),
        landing_center_xy=landing_center_xy,
        landing_radius=landing_radius,
        orientation_success_threshold_deg=orientation_success_threshold_deg,
    )

    log_robustness_visualizations(
        run=wb_run,
        records=records,
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
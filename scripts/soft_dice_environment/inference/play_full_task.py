"""Execute the complete soft-dice task using symbolic planning + RL tracking.

High-level architecture:

    initial cube orientation
            |
            v
          BFS
            |
            v
       primitive
            |
            v
    reference trajectory
            |
            v
       RL tracking policy
            |
            v
    update symbolic state
            |
            v
          replan

This script is inference-only.

"""

from __future__ import annotations

import argparse
import importlib.metadata as metadata
import os
import re
from pathlib import Path

from isaaclab.app import AppLauncher


# =============================================================================
# Paths
# =============================================================================

REPO_ROOT = Path(__file__).resolve().parents[3]

DEFAULT_PRIMITIVE_DIR = (
    REPO_ROOT
    / "source"
    / "isaaclab_tasks"
    / "isaaclab_tasks"
    / "manager_based"
    / "manipulation"
    / "soft_dice_manipulation"
    / "reference_trajectories"
)


# =============================================================================
# CLI
# =============================================================================

parser = argparse.ArgumentParser(
    description=(
        "Execute the full soft-dice task using BFS planning and "
        "a reference-tracking RSL-RL policy."
    )
)

parser.add_argument(
    "--task",
    type=str,
    default="Isaac-Soft-Dice-Tracking-H1-v0",
)

parser.add_argument(
    "--experiment_name",
    type=str,
    default=None,
    help=(
        "RSL-RL experiment directory. If omitted, use the experiment "
        "name registered in the agent configuration."
    ),
)

parser.add_argument(
    "--load_run",
    type=str,
    required=True,
    help="Training run directory.",
)

parser.add_argument(
    "--checkpoint",
    type=str,
    default=None,
    help=(
        "Checkpoint filename, e.g. model_30000.pt. "
        "If omitted, use latest model_*.pt."
    ),
)

parser.add_argument(
    "--primitive_dir",
    type=str,
    default=str(DEFAULT_PRIMITIVE_DIR),
    help=(
        "Directory containing the reference trajectories that may be "
        "selected by the planner."
    ),
)

parser.add_argument(
    "--motion_pattern",
    type=str,
    default="*.npz",
    help="Glob used by MotionCommand to load reference trajectories.",
)


# -----------------------------------------------------------------------------
# Primitive -> reference mapping
#
# IMPORTANT:
# These values are filename STEMS, not full paths.
# -----------------------------------------------------------------------------

parser.add_argument(
    "--rotate_right_motion",
    type=str,
    required=True,
)

parser.add_argument(
    "--rotate_left_motion",
    type=str,
    required=True,
)

parser.add_argument(
    "--yaw_right_motion",
    type=str,
    required=True,
)

parser.add_argument(
    "--yaw_left_motion",
    type=str,
    required=True,
)

parser.add_argument("--initial_top", type=int, default=1)
parser.add_argument("--initial_bottom", type=int, default=6)
parser.add_argument("--initial_front", type=int, default=3)
parser.add_argument("--initial_back", type=int, default=5)
parser.add_argument("--initial_left", type=int, default=4)
parser.add_argument("--initial_right", type=int, default=2)

parser.add_argument(
    "--target_face",
    type=int,
    required=True,
    choices=[1, 2, 3, 4, 5, 6],
)

parser.add_argument(
    "--seed",
    type=int,
    default=42,
)

parser.add_argument(
    "--max_primitives",
    type=int,
    default=4,
    help="Safety limit on primitive executions.",
)

parser.add_argument(
    "--progress_interval",
    type=int,
    default=200,
    help=(
        "Print tracking progress every N control steps. "
        "Use 0 to disable."
    ),
)

# -----------------------------------------------------------------------------
# Isaac Sim
# -----------------------------------------------------------------------------

AppLauncher.add_app_launcher_args(parser)

# GUI by default.
parser.set_defaults(visualizer=["kit"])

args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# =============================================================================
# Imports requiring Isaac Sim
# =============================================================================

import gymnasium as gym
import torch

from packaging import version

from rsl_rl.runners import (
    DistillationRunner,
    OnPolicyRunner,
)

from isaaclab.utils.seed import configure_seed

from isaaclab_rl.rsl_rl import (
    RslRlVecEnvWrapper,
    handle_deprecated_rsl_rl_cfg,
)

import isaaclab_tasks  # noqa: F401

from isaaclab_tasks.utils.parse_cfg import (
    get_checkpoint_path,
    load_cfg_from_registry,
)

from isaaclab_tasks.manager_based.manipulation.soft_dice_manipulation.utils.primitive_graph_traversal import (
    Orientation,
    bfs_to_top_face,
    build_orientation_graph,
    validate_orientation,
)


# =============================================================================
# Helpers
# =============================================================================

def resolve_checkpoint(
    experiment_name: str,
) -> str:
    """Resolve RSL-RL checkpoint path."""

    log_root = (
        REPO_ROOT
        / "logs"
        / "rsl_rl"
        / experiment_name
    )

    run_dir = log_root / args_cli.load_run

    if not run_dir.is_dir():
        raise FileNotFoundError(
            f"Training run directory does not exist:\n{run_dir}"
        )

    if args_cli.checkpoint is not None:

        checkpoint = Path(args_cli.checkpoint)

        if checkpoint.is_absolute():
            checkpoint_path = checkpoint
        else:
            checkpoint_path = run_dir / checkpoint

        if not checkpoint_path.is_file():
            raise FileNotFoundError(
                f"Checkpoint does not exist:\n{checkpoint_path}"
            )

        return str(checkpoint_path)

    return get_checkpoint_path(
        log_path=str(log_root),
        run_dir=re.escape(args_cli.load_run) + "$",
        checkpoint=r"model_.*\.pt",
    )


def make_initial_orientation() -> Orientation:
    """Construct and validate the symbolic initial cube state."""

    orientation = Orientation(
        top=args_cli.initial_top,
        bottom=args_cli.initial_bottom,
        front=args_cli.initial_front,
        back=args_cli.initial_back,
        left=args_cli.initial_left,
        right=args_cli.initial_right,
    )

    validate_orientation(orientation)

    return orientation


def make_primitive_mapping() -> dict[str, str]:
    """Map graph actions to MotionCommand trajectory names."""

    return {
        "rotate_right": args_cli.rotate_right_motion,
        "rotate_left": args_cli.rotate_left_motion,
        "yaw_right": args_cli.yaw_right_motion,
        "yaw_left": args_cli.yaw_left_motion,
    }


def configure_planned_inference_env(
    env_cfg,
    primitive_dir: Path,
):
    """Configure the training environment for deterministic chained inference."""

    env_cfg.scene.num_envs = 1
    env_cfg.seed = int(args_cli.seed)

    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    
    # Load a pool of reference motions.
    env_cfg.commands.motion.motion_file = ""
    env_cfg.commands.motion.motion_dir = str(primitive_dir)
    env_cfg.commands.motion.motion_pattern = str(
        args_cli.motion_pattern
    )

    env_cfg.commands.motion.start_frame = 0
    env_cfg.commands.motion.playback_speed = 1.0
    env_cfg.commands.motion.loop = False

    # Disable policy observation noise.
    env_cfg.observations.policy.enable_corruption = False

    # Disable training-time domain randomization.
    env_cfg.events.physics_material = None
    env_cfg.events.add_joint_default_pos = None
    env_cfg.events.base_com = None
    env_cfg.events.randomize_cube_material = None

    # Keep reset_to_reference, but make the INITIAL reset deterministic.
    reset_params = (
        env_cfg.events.reset_to_reference.params
    )

    reset_params["joint_position_range"] = (
        0.0,
        0.0,
    )

    reset_params["cube_position_range"] = {
        "x": (0.0, 0.0),
        "y": (0.0, 0.0),
        "z": (0.0, 0.0),
    }

    reset_params["cube_orientation_range"] = {
        "roll": (0.0, 0.0),
        "pitch": (0.0, 0.0),
        "yaw": (0.0, 0.0),
    }

    # reset_to_motion_start must therefore NOT randomly call sample_motions() --> trajectory has been chosen  by the planner
    reset_params["sample_motion"] = False

    # do NOT resetting the environment after the first
    env_cfg.terminations.motion_finished = None

    # For this first chaining test, also disable the tracking failure terminations and hard timeout.
    env_cfg.terminations.ee_body_pos = None
    env_cfg.terminations.object_pose = None
    env_cfg.terminations.time_out = None

    env_cfg.validate_config()


def validate_motion_mapping(
    motion,
    primitive_to_motion: dict[str, str],
):
    """Make sure all graph primitives have a loaded reference."""

    available = set(motion.motion_names)

    print("\nAvailable loaded motions:")

    for name in motion.motion_names:
        print(f"  - {name}")

    missing = []

    for primitive, motion_name in primitive_to_motion.items():

        if motion_name not in available:
            missing.append(
                (primitive, motion_name)
            )

    if missing:

        message = "\nPrimitive reference mapping is invalid:\n"

        for primitive, motion_name in missing:
            message += (
                f"  {primitive} -> "
                f"'{motion_name}' NOT FOUND\n"
            )

        message += (
            "\nAvailable motions:\n  "
            + "\n  ".join(sorted(available))
        )

        raise ValueError(message)


def execute_active_primitive(
    env,
    policy,
    motion,
    primitive_name: str,
):
    """Track the currently selected reference until its final frame."""

    if env.num_envs != 1:
        raise RuntimeError(
            "Planned inference currently supports num_envs=1 only."
        )

    motion_id = int(motion.motion_id[0].item())

    motion_name = motion.motion_name(motion_id)

    start_frame = int(motion.frame_idx[0].item())

    final_frame = (
        int(
            motion.motion_lengths[
                motion_id
            ].item()
        )
        - 1
    )

    print("\n" + "=" * 80)
    print(f"EXECUTING PRIMITIVE: {primitive_name}")
    print(f"Reference:           {motion_name}")
    print(f"Start frame:         {start_frame}")
    print(f"Final frame:         {final_frame}")
    print("=" * 80)

    # Recompute observations AFTER switching reference.
    obs = env.get_observations()

    step_count = 0

    # Safety bound in case the command fails to progress.
    expected_steps = (
        final_frame
        - start_frame
        + 1
    )

    max_steps = expected_steps + 10

    while simulation_app.is_running():

        # --------------------------------------------------------------
        # Important:
        #
        # This is the reference frame the policy sees BEFORE env.step().
        #
        # We want the policy to act once on the final reference frame
        # before declaring the primitive finished.
        # --------------------------------------------------------------

        frame_used = int(
            motion.frame_idx[0].item()
        )

        with torch.inference_mode():

            actions = policy(obs)

            obs, _, dones, _ = env.step(
                actions
            )

        step_count += 1

        # All automatic terminations should be disabled.
        if torch.any(dones):
            raise RuntimeError(
                "Environment terminated during planned inference. "
                "Check that motion_finished/time_out/failure "
                "terminations are disabled."
            )

        if (
            args_cli.progress_interval > 0
            and step_count % args_cli.progress_interval == 0
        ):
            print(
                f"[TRACKING] "
                f"primitive={primitive_name} "
                f"frame={frame_used}/{final_frame} "
                f"step={step_count}"
            )

        # The final reference frame has now actually been given to the policy for one control step.
        if frame_used >= final_frame:
            break

        if step_count > max_steps:
            raise RuntimeError(
                f"Primitive '{primitive_name}' exceeded expected "
                f"length. frame={frame_used}, "
                f"final_frame={final_frame}."
            )

    print(
        f"[DONE] {primitive_name}: "
        f"{step_count} control steps."
    )

    return obs


# =============================================================================
# Main
# =============================================================================

def main():


    current_orientation = (make_initial_orientation())

    target_face = int(args_cli.target_face)

    graph = build_orientation_graph(current_orientation)

    if len(graph) != 24:
        raise RuntimeError(
            f"Expected 24 cube orientations, "
            f"got {len(graph)}."
        )

    primitive_to_motion = (make_primitive_mapping())

    print("\n" + "=" * 80)
    print("SOFT-DICE FULL-TASK INFERENCE")
    print("=" * 80)

    print("\nInitial symbolic orientation:")
    print(current_orientation)

    print(
        f"\nTarget top face: "
        f"{target_face}"
    )

    print("\nPrimitive mapping:")

    for primitive, motion_name in primitive_to_motion.items():
        print(
            f"  {primitive:<15} -> {motion_name}"
        )

    # Check that a plan exists before creating the simulator.

    initial_path, initial_goal = (
        bfs_to_top_face(
            graph,
            current_orientation,
            target_face,
        )
    )

    if initial_path is None:
        raise RuntimeError(
            "BFS could not find a path."
        )

    print(
        f"\nInitial BFS plan: "
        f"{initial_path}"
    )

    if len(initial_path) == 0:
        print(
            "\nTarget face is already on top. "
            "Nothing to execute."
        )
        return

    first_primitive = initial_path[0]

    first_motion_name = (
        primitive_to_motion[
            first_primitive
        ]
    )

    # Load registered environment and agent configuration.
    env_cfg = load_cfg_from_registry(
        args_cli.task,
        "env_cfg_entry_point",
    )

    agent_cfg = load_cfg_from_registry(
        args_cli.task,
        "rsl_rl_cfg_entry_point",
    )

    installed_version = metadata.version(
        "rsl-rl-lib"
    )

    agent_cfg = (
        handle_deprecated_rsl_rl_cfg(
            agent_cfg,
            installed_version,
        )
    )

    experiment_name = (
        args_cli.experiment_name
        if args_cli.experiment_name is not None
        else agent_cfg.experiment_name
    )

    resume_path = resolve_checkpoint(
        experiment_name
    )

    # Primitive trajectory directory.

    primitive_dir = Path(
        args_cli.primitive_dir
    )

    if not primitive_dir.is_absolute():
        primitive_dir = (
            REPO_ROOT
            / primitive_dir
        )

    primitive_dir = (
        primitive_dir.resolve()
    )

    if not primitive_dir.is_dir():
        raise FileNotFoundError(
            f"Primitive trajectory directory does not exist:\n"
            f"{primitive_dir}"
        )

    # Planned-inference environment configuration.

    configure_planned_inference_env(
        env_cfg,
        primitive_dir,
    )

    if args_cli.device is not None:
        agent_cfg.device = args_cli.device

    configure_seed(
        int(args_cli.seed),
        True,
    )

    # -------------------------------------------------------------------------
    # IMPORTANT:
    # RslRlVecEnvWrapper calls env.reset() in its constructor.
    #
    # We therefore need access to MotionCommand BEFORE creating the wrapper
    # so that BFS can choose the first reference.
    # -------------------------------------------------------------------------

    raw_env = gym.make(
        args_cli.task,
        cfg=env_cfg,
    )

    motion = (
        raw_env.unwrapped
        .command_manager
        .get_term("motion")
    )

    validate_motion_mapping(
        motion,
        primitive_to_motion,
    )

    # Select the FIRST planned primitive before the initial reset.

    print(
        f"\nSelecting first primitive before reset: "
        f"{first_primitive} -> {first_motion_name}"
    )

    motion.set_motion_by_name(
        first_motion_name
    )

    # -------------------------------------------------------------------------
    # RSL-RL wrapper.
    #
    # This now performs the initial reset using the selected first motion.
    # -------------------------------------------------------------------------

    env = RslRlVecEnvWrapper(
        raw_env,
        clip_actions=agent_cfg.clip_actions,
    )

    print(
        f"\n[INFO] Loading checkpoint:\n"
        f"{resume_path}"
    )

    # -------------------------------------------------------------------------
    # Create RSL-RL runner.
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
        raise ValueError(
            f"Unsupported runner class: "
            f"{agent_cfg.class_name}"
        )

    runner.load(
        resume_path
    )

    policy = runner.get_inference_policy(
        device=env.unwrapped.device
    )

    # -------------------------------------------------------------------------
    # Full task execution.
    # -------------------------------------------------------------------------

    primitive_count = 0

    try:

        while (
            simulation_app.is_running()
            and current_orientation.top
            != target_face
        ):

            if (
                primitive_count
                >= args_cli.max_primitives
            ):
                raise RuntimeError(
                    "Exceeded maximum number of primitives "
                    f"({args_cli.max_primitives})."
                )

            # --------------------------------------------------------------
            # Closed-loop planner structure.
            #
            # For NOW the state update below assumes the primitive succeeded.
            # --------------------------------------------------------------

            path, _ = bfs_to_top_face(
                graph,
                current_orientation,
                target_face,
            )

            if path is None or len(path) == 0:
                raise RuntimeError(
                    "Planner returned an invalid path."
                )

            primitive = path[0]

            motion_name = (
                primitive_to_motion[
                    primitive
                ]
            )

            print("\n" + "-" * 80)

            print(
                f"Current state: "
                f"{current_orientation}"
            )

            print(
                f"Target top:    "
                f"{target_face}"
            )

            print(
                f"Current plan:  "
                f"{path}"
            )

            print(
                f"Next primitive:"
                f" {primitive}"
            )

            print(
                f"Reference:     "
                f"{motion_name}"
            )

            # --------------------------------------------------------------
            # First primitive was selected before wrapper construction.
            #Every subsequent primitive is selected here WITHOUT resettingthe physical simulation.
            # --------------------------------------------------------------

            if primitive_count > 0:

                motion.set_motion_by_name(
                    motion_name
                )

            # --------------------------------------------------------------
            # Track active reference.
            # --------------------------------------------------------------

            execute_active_primitive(
                env=env,
                policy=policy,
                motion=motion,
                primitive_name=primitive,
            )

            primitive_count += 1

            # --------------------------------------------------------------
            # TEMPORARY symbolic update.
            #
            # Assume the robot successfully executed exactly the graph edge.
            #
            # Later replace this with:
            #
            # current_orientation = estimate_orientation_from_perception(...)
            #
            # and run BFS again from the measured state.
            # --------------------------------------------------------------

            expected_next_orientation = (
                graph[
                    current_orientation
                ][primitive]
            )

            print(
                "\nExpected orientation after primitive:"
            )

            print(
                expected_next_orientation
            )

            current_orientation = (
                expected_next_orientation
            )

        # ---------------------------------------------------------------------
        # Goal reached.
        # ---------------------------------------------------------------------

        if current_orientation.top == target_face:

            print("\n" + "=" * 80)
            print("TARGET REACHED")
            print("=" * 80)

            print(
                f"Final symbolic orientation:\n"
                f"{current_orientation}"
            )

            print(
                f"\nTarget face: "
                f"{target_face}"
            )

            print(
                f"Primitives executed: "
                f"{primitive_count}"
            )

    finally:

        env.close()


# =============================================================================
# Entry point
# =============================================================================

if __name__ == "__main__":

    try:

        main()

    finally:

        simulation_app.close()
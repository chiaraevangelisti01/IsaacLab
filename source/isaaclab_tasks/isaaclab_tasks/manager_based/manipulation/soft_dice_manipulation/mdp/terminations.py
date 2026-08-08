from __future__ import annotations

from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from .motion_command import MotionCommand


def motion_finished(
    env: ManagerBasedRLEnv,
    command_name: str = "motion",
) -> torch.Tensor:
    """End the episode at the final reference frame and log final-state errors."""

    motion: MotionCommand = env.command_manager.get_term(command_name)

    finished = motion.finished

    finished_env_ids = finished.nonzero(
        as_tuple=False
    ).squeeze(-1)

    if finished_env_ids.numel() > 0:
        motion.record_terminal_metrics(finished_env_ids)

    return finished


def excessive_joint_tracking_error(
    env: ManagerBasedRLEnv,
    command_name: str = "motion",
    threshold: float = 1.0,
) -> torch.Tensor:
    """End an episode when the joint tracking error exceeds a threshold"""
    robot = env.scene["robot"]
    motion: MotionCommand = env.command_manager.get_term(command_name)
    error = torch.max(torch.abs(robot.data.joint_pos.torch - motion.joint_pos), dim=1).values
    return error > threshold

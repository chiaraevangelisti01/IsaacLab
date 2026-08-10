from __future__ import annotations

from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from .motion_command import MotionCommand

from .motion_utils import H1_TRACKED_BODY_NAMES

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


def bad_motion_body_pos_z_only(
    env: ManagerBasedRLEnv,
    command_name: str,
    threshold: float,
    body_names: list[str] | None = None,
) -> torch.Tensor:
    """Terminate when selected tracked bodies deviate too far in z."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    body_pos_ref, _ = motion.aligned_body_reference()

    if body_names is None:
        body_indices = list(
            range(len(H1_TRACKED_BODY_NAMES))
        )
    else:
        body_indices = [
            H1_TRACKED_BODY_NAMES.index(name)
            for name in body_names
        ]

    error = torch.abs(
        body_pos_ref[
            :, body_indices, 2
        ]
        - motion.robot_body_pos[
            :, body_indices, 2
        ]
    )

    return torch.any(
        error > threshold,
        dim=-1,
    )
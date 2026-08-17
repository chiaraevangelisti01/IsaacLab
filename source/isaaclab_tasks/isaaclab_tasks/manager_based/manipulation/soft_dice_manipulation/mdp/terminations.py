from __future__ import annotations

from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from .motion_command import MotionCommand

from ..utils.motion_utils import H1_TRACKED_BODY_NAMES

from ..utils.geometry_utils import (
    orientation_error,
    vector_error,
)

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
    include_hands: bool = False,
) -> torch.Tensor:

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    if include_hands:
        if not motion.has_hand_reference:
            raise RuntimeError(
                "Hand tracking termination requested, but the motion "
                "does not contain hand references."
            )

        hand_pos_ref = motion.aligned_hand_reference()

        error = torch.abs(
            hand_pos_ref[..., 2]
            - motion.robot_hand_pos[..., 2]
        )

    else:
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
            body_pos_ref[:, body_indices, 2]
            - motion.robot_body_pos[:, body_indices, 2]
        )

    return torch.any(
        error > threshold,
        dim=-1,
    )

def bad_object_pose(
    env: ManagerBasedRLEnv,
    command_name: str,
    position_threshold: float,
    orientation_threshold: float,
) -> torch.Tensor:
    """Terminate when deformable-dice pose tracking becomes too inaccurate."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    if not motion.has_object_reference:
        return torch.zeros(
            env.num_envs,
            dtype=torch.bool,
            device=env.device,
        )

    position_error_m = vector_error(
        reference=motion.cube_pos,
        current=motion.simulator_cube_pos,
    )

    orientation_error_rad = orientation_error(
        reference_quat=motion.cube_quat,
        current_quat=motion.simulator_cube_quat,
    )

    return (
        (position_error_m > position_threshold)
        | (orientation_error_rad > orientation_threshold)
    )
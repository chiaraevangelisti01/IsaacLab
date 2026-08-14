from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_error_magnitude

from ..utils.motion_utils import H1_TRACKED_BODY_NAMES

if TYPE_CHECKING:
    from isaaclab.assets import Articulation
    from isaaclab.envs import ManagerBasedRLEnv
    from .motion_command import MotionCommand

def motion_global_anchor_orientation_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
) -> torch.Tensor:
    """Track the demonstrated torso orientation."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    torso_idx = H1_TRACKED_BODY_NAMES.index(
        "torso_link"
    )

    error = quat_error_magnitude(
        motion.body_quat[:, torso_idx],
        motion.robot_body_quat[:, torso_idx],
    ) ** 2

    return torch.exp(
        -error / (std * std)
    )


def motion_relative_body_position_error_exp(
        env: ManagerBasedRLEnv,
        command_name: str,
        std: float,
        include_hands: bool = False,
    ) -> torch.Tensor:
        """Track yaw-aligned Cartesian body positions."""

        motion: MotionCommand = env.command_manager.get_term(
            command_name
        )

        body_pos_ref, _ = motion.aligned_body_reference()

        body_error = torch.sum(
            torch.square(
                body_pos_ref
                - motion.robot_body_pos
            ),
            dim=-1,
        )

        if include_hands:
            if not motion.has_hand_reference:
                raise RuntimeError(
                    "Hand tracking requested, but the motion "
                    "does not contain hand references."
                )

            hand_pos_ref = motion.aligned_hand_reference()

            hand_error = torch.sum(
                torch.square(
                    hand_pos_ref
                    - motion.robot_hand_pos
                ),
                dim=-1,
            )

            error = torch.cat(
                (
                    body_error,
                    hand_error,
                ),
                dim=-1,
            )

        else:
            error = body_error

        return torch.exp(
            -torch.mean(error, dim=-1)
            / (std * std)
        )

def motion_relative_body_orientation_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
) -> torch.Tensor:
    """Track yaw-aligned Cartesian body orientations."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    _, body_quat_ref = motion.aligned_body_reference()

    error = quat_error_magnitude(
        body_quat_ref,
        motion.robot_body_quat,
    ) ** 2

    return torch.exp(
        -torch.mean(error, dim=-1)
        / (std * std)
    )


def motion_global_body_linear_velocity_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
) -> torch.Tensor:
    """Track fixed-root Cartesian body linear velocities."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    error = torch.sum(
        torch.square(
            motion.body_lin_vel
            - motion.robot_body_lin_vel
        ),
        dim=-1,
    )

    return torch.exp(
        -torch.mean(error, dim=-1)
        / (std * std)
    )


def motion_global_body_angular_velocity_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
) -> torch.Tensor:
    """Track fixed-root Cartesian body angular velocities."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    error = torch.sum(
        torch.square(
            motion.body_ang_vel
            - motion.robot_body_ang_vel
        ),
        dim=-1,
    )

    return torch.exp(
        -torch.mean(error, dim=-1)
        / (std * std)
    )

def object_global_ref_position_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
) -> torch.Tensor:
    """Track the demonstrated deformable-dice position."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    error = torch.sum(
        torch.square(
            motion.cube_pos
            - motion.simulator_cube_pos
        ),
        dim=-1,
    )

    return torch.exp(
        -error / (std * std)
    )


def object_global_ref_orientation_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
) -> torch.Tensor:
    """Track the demonstrated deformable-dice bulk orientation."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    error = quat_error_magnitude(
        motion.cube_quat,
        motion.simulator_cube_quat,
    ) ** 2

    return torch.exp(
        -error / (std * std)
    )



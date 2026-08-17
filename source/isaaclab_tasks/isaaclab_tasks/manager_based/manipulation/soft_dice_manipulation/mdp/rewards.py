from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg
from ..utils.geometry_utils import (
    orientation_error,
    vector_error,
)
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

    error = torch.square(
        orientation_error(
            reference_quat= motion.body_quat[:, torso_idx],
            current_quat= motion.robot_body_quat[:, torso_idx],
        )
    )

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

        body_error = torch.square(
            vector_error(
                reference=body_pos_ref,
                current=motion.robot_body_pos,
            )
        )

        if include_hands:
            if not motion.has_hand_reference:
                raise RuntimeError(
                    "Hand tracking requested, but the motion "
                    "does not contain hand references."
                )

            hand_pos_ref = motion.aligned_hand_reference()

            hand_error = torch.square(
                vector_error(
                    reference=hand_pos_ref,
                    current=motion.robot_hand_pos,
                )
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

    error = torch.square(
        orientation_error(
            reference_quat= body_quat_ref,
            current_quat=motion.robot_body_quat,
        )
    )

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

    error = torch.square(
        vector_error(
            reference=motion.body_lin_vel,
            current=motion.robot_body_lin_vel,
        )
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

    error = torch.square(
        vector_error(
            reference=motion.body_ang_vel,
            current=motion.robot_body_ang_vel,
        )
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

    error = vector_error(
        reference=motion.cube_pos,
        current=motion.simulator_cube_pos,
    )

    return torch.exp(
        -torch.square(error)
        / (std * std)
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

    error = torch.square(
        orientation_error(
            reference_quat=motion.cube_quat,
            current_quat=motion.simulator_cube_quat,
        )
    )

    return torch.exp(-error / (std * std))



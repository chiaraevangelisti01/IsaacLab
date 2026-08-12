from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import (
    matrix_from_quat,
    quat_apply_inverse,
    subtract_frame_transforms,
)

from .motion_utils import H1_TRACKED_BODY_NAMES

if TYPE_CHECKING:
    from isaaclab.assets import Articulation
    from isaaclab.envs import ManagerBasedRLEnv
    from .motion_command import MotionCommand


def reference_joint_command(
    env: ManagerBasedRLEnv,
    command_name: str = "motion",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reference q and qdot for the joints controlled by the policy."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    return torch.cat(
        (
            motion.joint_pos[:, asset_cfg.joint_ids],
            motion.joint_vel[:, asset_cfg.joint_ids],
        ),
        dim=-1,
    )


def motion_anchor_ori_b(
    env: ManagerBasedRLEnv,
    command_name: str = "motion",
) -> torch.Tensor:
    """Reference torso orientation relative to the current torso."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    torso_idx = H1_TRACKED_BODY_NAMES.index(
        "torso_link"
    )

    _, ori_b = subtract_frame_transforms(
        motion.robot_body_pos[:, torso_idx],
        motion.robot_body_quat[:, torso_idx],
        motion.body_pos[:, torso_idx],
        motion.body_quat[:, torso_idx],
    )

    mat = matrix_from_quat(ori_b)

    # BeyondMimic 6D orientation representation:
    # first two columns of the rotation matrix.
    return mat[..., :2].reshape(
        env.num_envs,
        -1,
    )


def robot_body_pos_b(
    env: ManagerBasedRLEnv,
    command_name: str = "motion",
) -> torch.Tensor:
    """Actual tracked-body positions relative to the actual torso."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    torso_idx = H1_TRACKED_BODY_NAMES.index(
        "torso_link"
    )

    num_bodies = len(H1_TRACKED_BODY_NAMES)

    torso_pos = motion.robot_body_pos[
        :, torso_idx, :
    ]
    torso_quat = motion.robot_body_quat[
        :, torso_idx, :
    ]

    pos_b, _ = subtract_frame_transforms(
        torso_pos[:, None, :].repeat(
            1, num_bodies, 1
        ),
        torso_quat[:, None, :].repeat(
            1, num_bodies, 1
        ),
        motion.robot_body_pos,
        motion.robot_body_quat,
    )

    return pos_b.reshape(
        env.num_envs,
        -1,
    )


def robot_body_ori_b(
    env: ManagerBasedRLEnv,
    command_name: str = "motion",
) -> torch.Tensor:
    """Actual tracked-body orientations relative to the actual torso."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    torso_idx = H1_TRACKED_BODY_NAMES.index(
        "torso_link"
    )

    num_bodies = len(H1_TRACKED_BODY_NAMES)

    torso_pos = motion.robot_body_pos[
        :, torso_idx, :
    ]
    torso_quat = motion.robot_body_quat[
        :, torso_idx, :
    ]

    _, ori_b = subtract_frame_transforms(
        torso_pos[:, None, :].repeat(
            1, num_bodies, 1
        ),
        torso_quat[:, None, :].repeat(
            1, num_bodies, 1
        ),
        motion.robot_body_pos,
        motion.robot_body_quat,
    )

    mat = matrix_from_quat(ori_b)

    return mat[..., :2].reshape(
        env.num_envs,
        -1,
    )

#### Holosoma-like object observatiions
def object_pos_b(
    env: ManagerBasedRLEnv,
    command_name: str = "motion",
) -> torch.Tensor:
    """Current deformable-dice position relative to the actual torso."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    torso_idx = H1_TRACKED_BODY_NAMES.index(
        "torso_link"
    )

    pos_b, _ = subtract_frame_transforms(
        motion.robot_body_pos[:, torso_idx],
        motion.robot_body_quat[:, torso_idx],
        motion.simulator_cube_pos,
        motion.simulator_cube_quat,
    )

    return pos_b

def object_ori_b(
    env: ManagerBasedRLEnv,
    command_name: str = "motion",
) -> torch.Tensor:
    """Current deformable-dice orientation relative to the actual torso."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    torso_idx = H1_TRACKED_BODY_NAMES.index(
        "torso_link"
    )

    _, ori_b = subtract_frame_transforms(
        motion.robot_body_pos[:, torso_idx],
        motion.robot_body_quat[:, torso_idx],
        motion.simulator_cube_pos,
        motion.simulator_cube_quat,
    )

    mat = matrix_from_quat(ori_b)

    # 6D orientation representation:
    # first two columns of the rotation matrix.
    return mat[..., :2].reshape(
        env.num_envs,
        -1,
    )

def object_lin_vel_b(
    env: ManagerBasedRLEnv,
    command_name: str = "motion",
) -> torch.Tensor:
    """Current deformable-dice linear velocity expressed in the actual torso frame."""

    motion: MotionCommand = env.command_manager.get_term(
        command_name
    )

    torso_idx = H1_TRACKED_BODY_NAMES.index(
        "torso_link"
    )

    return quat_apply_inverse(
        motion.robot_body_quat[:, torso_idx],
        motion.simulator_cube_lin_vel,
    )
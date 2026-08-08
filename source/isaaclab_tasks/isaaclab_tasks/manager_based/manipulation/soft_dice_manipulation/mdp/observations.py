from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.assets import Articulation
    from isaaclab.envs import ManagerBasedRLEnv
    from .motion_command import MotionCommand


def reference_joint_pos_rel(
    env: ManagerBasedRLEnv,
    command_name: str = "motion",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reference joint positions relative to the articulation default pose."""
    robot: Articulation = env.scene[asset_cfg.name]
    motion: MotionCommand = env.command_manager.get_term(command_name)

    reference = motion.joint_pos[:, asset_cfg.joint_ids]
    default = robot.data.default_joint_pos.torch[:, asset_cfg.joint_ids]
    return reference - default


def reference_joint_vel(
    env: ManagerBasedRLEnv,
    command_name: str = "motion",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Numerically differentiated reference joint velocities."""
    motion: MotionCommand = env.command_manager.get_term(command_name)
    return motion.joint_vel[:, asset_cfg.joint_ids]


def motion_phase(
    env: ManagerBasedRLEnv,
    command_name: str = "motion",
) -> torch.Tensor:
    """Normalized reference phase in [0, 1]"""
    motion: MotionCommand = env.command_manager.get_term(command_name)
    denom = max(motion.num_frames - 1, 1)
    return (motion.frame_idx.float() / float(denom)).unsqueeze(-1)

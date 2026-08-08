from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.assets import Articulation
    from isaaclab.envs import ManagerBasedRLEnv
    from .motion_command import MotionCommand


def joint_pos_tracking_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Gaussian tracking reward from mean squared joint-position error."""
    robot: Articulation = env.scene[asset_cfg.name]
    motion: MotionCommand = env.command_manager.get_term(command_name)

    error = robot.data.joint_pos.torch[:, asset_cfg.joint_ids] - motion.joint_pos[:, asset_cfg.joint_ids]
    mse = torch.mean(torch.square(error), dim=1)
    return torch.exp(-mse / (std * std))


def joint_vel_tracking_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Gaussian tracking reward from mean squared joint-velocity error."""
    robot: Articulation = env.scene[asset_cfg.name]
    motion: MotionCommand = env.command_manager.get_term(command_name)

    error = robot.data.joint_vel.torch[:, asset_cfg.joint_ids] - motion.joint_vel[:, asset_cfg.joint_ids]
    mse = torch.mean(torch.square(error), dim=1)
    return torch.exp(-mse / (std * std))


def mean_joint_pos_error(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Mean absolute joint error, useful as a logged diagnostic or termination signal."""
    robot: Articulation = env.scene[asset_cfg.name]
    motion: MotionCommand = env.command_manager.get_term(command_name)

    error = robot.data.joint_pos.torch[:, asset_cfg.joint_ids] - motion.joint_pos[:, asset_cfg.joint_ids]
    return torch.mean(torch.abs(error), dim=1)

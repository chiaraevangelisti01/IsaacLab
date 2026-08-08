from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from .motion_command import MotionCommand


def _env_ids_tensor(env, env_ids):
    if env_ids is None:
        return torch.arange(env.num_envs, dtype=torch.long, device=env.device)
    if isinstance(env_ids, torch.Tensor):
        return env_ids.to(device=env.device, dtype=torch.long)
    return torch.as_tensor(env_ids, dtype=torch.long, device=env.device)


def _as_torch(x):
    return x.torch if hasattr(x, "torch") else x


def reset_to_motion_start(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int] | torch.Tensor | None,
    command_name: str = "motion",
    robot_name: str = "robot",
    cube_name: str = "cube",
    use_reference_joint_velocity: bool = False,
):
    """Reset root, joints, and the deformable dice to the demonstrated start state."""

    env_ids = _env_ids_tensor(env, env_ids)
    if env_ids.numel() == 0:
        return

    motion: MotionCommand = env.command_manager.get_term(command_name)
    robot = env.scene[robot_name]
    cube = env.scene[cube_name]

    # Explicit root rewrite: this mirrors the manual reset that was required
    # by the validated standalone replay.
    default_root_state = _as_torch(robot.data.default_root_state)[env_ids].clone()
    default_root_state[:, :3] += env.scene.env_origins[env_ids]

    robot.write_root_pose_to_sim_index(
        root_pose=default_root_state[:, :7],
        env_ids=env_ids,
    )
    robot.write_root_velocity_to_sim_index(
        root_velocity=default_root_state[:, 7:],
        env_ids=env_ids,
    )

    joint_pos = motion.start_joint_pos(env_ids)
    joint_vel = motion.start_joint_vel(env_ids) if use_reference_joint_velocity else torch.zeros_like(joint_pos)

    robot.write_joint_position_to_sim_index(position=joint_pos, env_ids=env_ids)
    robot.write_joint_velocity_to_sim_index(velocity=joint_vel, env_ids=env_ids)
    robot.set_joint_position_target_index(target=joint_pos, env_ids=env_ids)

    if not motion.has_object_reference:
        return

    desired_pos_e = motion.start_cube_pos(env_ids)
    desired_quat = motion.start_cube_quat(env_ids)
    desired_pos_w = desired_pos_e + env.scene.env_origins[env_ids]

    nodal_state = cube.data.default_nodal_state_w.torch[env_ids].clone()
    default_pos = nodal_state[..., :3]
    default_center_w = default_pos.mean(dim=1)
    delta_pos_w = desired_pos_w - default_center_w

    nodal_state[..., :3] = cube.transform_nodal_pos(
        default_pos,
        pos=delta_pos_w,
        quat=desired_quat,
    )
    nodal_state[..., 3:] = 0.0

    cube.write_nodal_state_to_sim_index(nodal_state, env_ids=env_ids)

    nodal_targets = cube.data.nodal_kinematic_target.torch[env_ids].clone()
    nodal_targets[..., :3] = nodal_state[..., :3]
    nodal_targets[..., 3] = 1.0
    cube.write_nodal_kinematic_target_to_sim_index(nodal_targets, env_ids=env_ids)
    cube.reset(env_ids)
from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

    from .motion_command import MotionCommand


def _as_env_ids(env: ManagerBasedRLEnv, env_ids: Sequence[int] | torch.Tensor | None) -> torch.Tensor:
    if env_ids is None:
        return torch.arange(env.num_envs, dtype=torch.long, device=env.device)
    if isinstance(env_ids, torch.Tensor):
        return env_ids.to(device=env.device, dtype=torch.long)
    return torch.as_tensor(env_ids, dtype=torch.long, device=env.device)


def reset_to_motion_start(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int] | torch.Tensor | None,
    command_name: str = "motion",
    robot_name: str = "robot",
    cube_name: str = "cube",
    use_reference_joint_velocity: bool = False,
):
    """Reset H1 and the deformable dice to the first frame of the reference.

    The deformable reset is the multi-environment version of the working logic in
    ``replay_utils.reset_deformable_cube_to_pose``. It uses the Isaac Lab 3.0 / Isaac Sim 6.0
    nodal-state API and supports a selected subset of parallel environments.
    """

    env_ids = _as_env_ids(env, env_ids)
    if env_ids.numel() == 0:
        return

    motion: MotionCommand = env.command_manager.get_term(command_name)
    robot = env.scene[robot_name]
    cube = env.scene[cube_name]

    # ------------------------------------------------------------------
    # Robot
    # ------------------------------------------------------------------
    root_state = robot.data.default_root_state.torch[env_ids].clone()

    # default_root_state is environment-local; PhysX expects world position.
    root_state[:, :3] += env.scene.env_origins[env_ids]

    robot.write_root_pose_to_sim_index(
        root_pose=root_state[:, :7],
        env_ids=env_ids,
    )

    robot.write_root_velocity_to_sim_index(
        root_velocity=root_state[:, 7:],
        env_ids=env_ids,
    )
    joint_pos = motion.start_joint_pos(env_ids)
    if use_reference_joint_velocity:
        joint_vel = motion.start_joint_vel(env_ids)
    else:
        # Matches the current replay initialization.
        joint_vel = torch.zeros_like(joint_pos)

    robot.write_joint_position_to_sim_index(position=joint_pos, env_ids=env_ids)
    robot.write_joint_velocity_to_sim_index(velocity=joint_vel, env_ids=env_ids)

    # ------------------------------------------------------------------
    # Deformable cube
    # ------------------------------------------------------------------
    if not motion.has_object_reference:
        return

    desired_pos_e = motion.start_cube_pos(env_ids)
    desired_quat_xyzw = motion.start_cube_quat(env_ids)

    # Reference positions are stored in environment-local coordinates. Deformable nodal
    # states are in world coordinates, so add each cloned environment origin.
    desired_pos_w = desired_pos_e + env.scene.env_origins[env_ids]

    nodal_state = cube.data.default_nodal_state_w.torch[env_ids].clone()
    default_nodal_pos = nodal_state[..., :3]
    default_center_w = default_nodal_pos.mean(dim=1)
    delta_pos_w = desired_pos_w - default_center_w

    nodal_state[..., :3] = cube.transform_nodal_pos(
        default_nodal_pos,
        pos=delta_pos_w,
        quat=desired_quat_xyzw,
    )
    nodal_state[..., 3:] = 0.0

    cube.write_nodal_state_to_sim_index(nodal_state, env_ids=env_ids)

    # Free all volume-deformable nodes (0 = kinematic, 1 = free in this API).
    nodal_targets = cube.data.nodal_kinematic_target.torch[env_ids].clone()
    nodal_targets[..., :3] = nodal_state[..., :3]
    nodal_targets[..., 3] = 1.0
    cube.write_nodal_kinematic_target_to_sim_index(nodal_targets, env_ids=env_ids)

    cube.reset(env_ids)

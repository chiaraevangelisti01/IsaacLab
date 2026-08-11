from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING, Literal

import torch
import isaaclab.utils.math as math_utils

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.assets import Articulation
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
    use_reference_joint_velocity: bool = True,
    joint_position_range: tuple[float, float] | None = None,
    tracking_asset_cfg: SceneEntityCfg | None = None,
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

    joint_vel = (
        motion.start_joint_vel(env_ids)
        if use_reference_joint_velocity
        else torch.zeros_like(joint_pos)
    )

    # BeyondMimic-style reset perturbation.
    # Only perturb joints actually controlled by this policy.
    if (
        joint_position_range is not None
        and tracking_asset_cfg is not None
    ):
        joint_ids = tracking_asset_cfg.joint_ids

        tracked_joint_pos = joint_pos[:, joint_ids]

        tracked_joint_pos += math_utils.sample_uniform(
            joint_position_range[0],
            joint_position_range[1],
            tracked_joint_pos.shape,
            joint_pos.device,
        )

        soft_limits = (
            robot.data.soft_joint_pos_limits.torch[env_ids]
            [:, joint_ids]
        )

        tracked_joint_pos = torch.clamp(
            tracked_joint_pos,
            min=soft_limits[..., 0],
            max=soft_limits[..., 1],
        )

        joint_pos[:, joint_ids] = tracked_joint_pos

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

def randomize_joint_default_pos(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    pos_distribution_params: tuple[float, float] | None = None,
    operation: Literal["add", "scale", "abs"] = "abs",
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
):
    """Randomize joint default positions to model calibration offsets."""
    from isaaclab.envs.mdp.events import _randomize_prop_by_op

    asset: Articulation = env.scene[asset_cfg.name]

    if env_ids is None:
        env_ids = torch.arange(
            env.scene.num_envs,
            device=asset.device,
            dtype=torch.long,
        )

    if asset_cfg.joint_ids == slice(None):
        joint_ids = slice(None)
    else:
        joint_ids = torch.tensor(
            asset_cfg.joint_ids,
            dtype=torch.long,
            device=asset.device,
        )

    if pos_distribution_params is None:
        return

    default_joint_pos = asset.data.default_joint_pos.torch

    randomized = _randomize_prop_by_op(
        default_joint_pos.clone(),
        pos_distribution_params,
        env_ids,
        joint_ids,
        operation=operation,
        distribution=distribution,
    )

    if isinstance(joint_ids, slice):
        default_joint_pos[env_ids] = randomized[env_ids]
    else:
        default_joint_pos[
            env_ids[:, None],
            joint_ids,
        ] = randomized[
            env_ids[:, None],
            joint_ids,
        ]

    # JointPositionAction copied the default pose into its offset during
    # construction. Synchronize that offset with the randomized defaults.
    action_term = env.action_manager.get_term("joint_pos")

    if isinstance(action_term._offset, torch.Tensor):
        action_term._offset[env_ids] = default_joint_pos[
            env_ids
        ][:, action_term._joint_ids]

def filter_lower_body_self_collisions(
    env,
    env_ids,
    lower_body_names: list[str],
    robot_prim_name: str = "Robot",
):
    """Disable self-collision pairs involving the H1 lower body."""

    from pxr import Usd, UsdPhysics

    stage = env.sim.stage

    for env_path in env.scene.env_prim_paths:
        robot_path = f"{env_path}/{robot_prim_name}"
        robot_prim = stage.GetPrimAtPath(robot_path)

        if not robot_prim.IsValid():
            raise RuntimeError(
                f"Robot prim not found: {robot_path}"
            )

        # Find actual rigid-body link prims.
        rigid_bodies = {}

        for prim in Usd.PrimRange(robot_prim):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_bodies[prim.GetName()] = prim

        for lower_name in lower_body_names:
            if lower_name not in rigid_bodies:
                raise RuntimeError(
                    f"Rigid body not found: {lower_name}. "
                    f"Available: {list(rigid_bodies)}"
                )

            lower_prim = rigid_bodies[lower_name]

            # Important sanity check.
            if lower_prim.IsInstanceProxy():
                raise RuntimeError(
                    f"{lower_prim.GetPath()} is itself an "
                    "instance proxy."
                )

            filter_api = UsdPhysics.FilteredPairsAPI.Apply(
                lower_prim
            )

            filtered_pairs = (
                filter_api.CreateFilteredPairsRel()
            )

            # Filter this lower-body link against every other
            # rigid body belonging to the robot.
            for other_prim in rigid_bodies.values():
                if other_prim == lower_prim:
                    continue

                filtered_pairs.AddTarget(
                    other_prim.GetPath()
                )
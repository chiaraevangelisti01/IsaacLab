from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING, Literal

import torch
import warp as wp
import numpy as np
import isaaclab.utils.math as math_utils

from isaaclab.managers import SceneEntityCfg
from ..utils.event_utils import get_randomization_buffers

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

def _sample_axis_ranges(
    ranges: dict[str, tuple[float, float]] | None,
    axis_names: tuple[str, str, str],
    num_samples: int,
    device: str,
) -> torch.Tensor:
    values = torch.zeros(
        (num_samples, 3),
        dtype=torch.float32,
        device=device,
    )

    if ranges is None:
        return values

    for axis_idx, axis_name in enumerate(axis_names):
        low, high = ranges.get(
            axis_name,
            (0.0, 0.0),
        )

        values[:, axis_idx] = math_utils.sample_uniform(
            low,
            high,
            (num_samples,),
            device,
        )

    return values

def reset_to_motion_start(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int] | torch.Tensor | None,
    command_name: str = "motion",
    robot_name: str = "robot",
    cube_name: str = "cube",
    use_reference_joint_velocity: bool = True,
    joint_position_range: tuple[float, float] | None = None,
    tracking_asset_cfg: SceneEntityCfg | None = None,
    cube_position_range: dict[str, tuple[float, float]] | None = None,
    cube_orientation_range: dict[str, tuple[float, float]] | None = None,
    cube_position_offset: torch.Tensor | None = None,
    cube_orientation_offset: torch.Tensor | None = None,
):
    """Reset root, joints, and the deformable dice to the demonstrated start state."""

    env_ids = _env_ids_tensor(env, env_ids)
    if env_ids.numel() == 0:
        return

    motion: MotionCommand = env.command_manager.get_term(command_name)
    motion.sample_motions(env_ids)
    robot = env.scene[robot_name]
    cube = env.scene[cube_name]

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

    desired_pos_e = motion.start_cube_pos(env_ids).clone()
    desired_quat = motion.start_cube_quat(env_ids).clone()

    if (
        cube_position_range is not None
        and cube_position_offset is not None
    ):
        raise ValueError(
            "Specify either cube_position_range or "
            "cube_position_offset, not both."
        )

    if cube_position_offset is None:
        cube_position_offset = _sample_axis_ranges(
            ranges=cube_position_range,
            axis_names=("x", "y", "z"),
            num_samples=env_ids.numel(),
            device=env.device,
        )
    else:
        cube_position_offset = torch.as_tensor(
            cube_position_offset,
            dtype=torch.float32,
            device=env.device,
        )

        if cube_position_offset.shape != (
            env_ids.numel(),
            3,
        ):
            raise ValueError(
                "cube_position_offset must have shape "
                f"({env_ids.numel()}, 3), got "
                f"{tuple(cube_position_offset.shape)}."
            )

    desired_pos_e += cube_position_offset


    if (
        cube_orientation_range is not None
        and cube_orientation_offset is not None
    ):
        raise ValueError(
            "Specify either cube_orientation_range or "
            "cube_orientation_offset, not both."
        )

    if cube_orientation_offset is None:
        cube_orientation_offset = _sample_axis_ranges(
            ranges=cube_orientation_range,
            axis_names=("roll", "pitch", "yaw"),
            num_samples=env_ids.numel(),
            device=env.device,
        )
    else:
        cube_orientation_offset = torch.as_tensor(
            cube_orientation_offset,
            dtype=torch.float32,
            device=env.device,
        )

        if cube_orientation_offset.shape != (
            env_ids.numel(),
            3,
        ):
            raise ValueError(
                "cube_orientation_offset must have shape "
                f"({env_ids.numel()}, 3), got "
                f"{tuple(cube_orientation_offset.shape)}."
            )

    randomization = get_randomization_buffers(env)

    randomization["cube_position_offset_m"][
        env_ids
    ] = cube_position_offset

    randomization["cube_yaw_offset_rad"][
        env_ids
    ] = cube_orientation_offset[:, 2]

    if torch.any(cube_orientation_offset != 0.0):
        delta_quat = math_utils.quat_from_euler_xyz(
            cube_orientation_offset[:, 0],
            cube_orientation_offset[:, 1],
            cube_orientation_offset[:, 2],
        )

        # World-frame perturbation around the demonstrated orientation.
        desired_quat = math_utils.quat_mul(
            delta_quat,
            desired_quat,
        )
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

def randomize_deformable_material(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int] | torch.Tensor | None,
    asset_name: str = "cube",
    youngs_modulus_range: tuple[float, float] | None = None,
    poissons_ratio_range: tuple[float, float] | None = None,
):
    """Randomize deformable material properties independently per environment."""

    env_ids = _env_ids_tensor(env, env_ids)

    randomization = get_randomization_buffers(env)

    if env_ids.numel() == 0:
        return

    cube = env.scene[asset_name]
    material_view = cube.material_physx_view

    if material_view is None:
        raise RuntimeError(
            f"Deformable asset '{asset_name}' has no PhysX material view."
        )

    if material_view.count != cube.num_instances:
        raise RuntimeError(
            "Expected one deformable material per cube instance. "
            f"material_count={material_view.count}, "
            f"cube_instances={cube.num_instances}."
        )

    # PhysX deformable material views use the Warp frontend in this setup.
    # The setter therefore receives Warp arrays, following the official
    # Omni Physics tensor API examples.
    material_ids_np = (
        env_ids.detach()
        .cpu()
        .numpy()
        .astype(np.int32)
    )

    material_ids_wp = wp.from_numpy(
        material_ids_np,
        dtype=wp.int32,
        device="cpu",
    )

    # ------------------------------------------------------------------
    # Young's modulus.
    # ------------------------------------------------------------------

    if youngs_modulus_range is not None:
        low, high = youngs_modulus_range

        if low <= 0.0 or high < low:
            raise ValueError(
                "Invalid Young's modulus range: "
                f"{youngs_modulus_range}."
            )

        # Getter returns the tensor representation associated with the
        # material view. Convert it to host NumPy, modify selected rows,
        # then convert back to the Warp representation expected by PhysX.
        youngs_modulus_wp = material_view.get_youngs_modulus()
        youngs_modulus_np = (
            youngs_modulus_wp.numpy()
            .copy()
            .astype(np.float32)
        )

        sampled = math_utils.sample_uniform(
            low,
            high,
            (env_ids.numel(), 1),
            env.device,
        )

        randomization["youngs_modulus_pa"][
            env_ids
        ] = sampled[:, 0]

        youngs_modulus_np[material_ids_np] = (
            sampled.detach()
            .cpu()
            .numpy()
            .astype(np.float32)
        )

        youngs_modulus_wp = wp.from_numpy(
            youngs_modulus_np,
            dtype=wp.float32,
            device="cpu",
        )

        material_view.set_youngs_modulus(
            youngs_modulus_wp,
            material_ids_wp,
        )

    # ------------------------------------------------------------------
    # Poisson's ratio.
    # ------------------------------------------------------------------

    if poissons_ratio_range is not None:
        low, high = poissons_ratio_range

        if low < 0.0 or high >= 0.5 or high < low:
            raise ValueError(
                "Poisson's ratio must satisfy "
                f"0 <= nu < 0.5. Got {poissons_ratio_range}."
            )

        poissons_ratio_wp = material_view.get_poissons_ratio()
        poissons_ratio_np = (
            poissons_ratio_wp.numpy()
            .copy()
            .astype(np.float32)
        )

        sampled = math_utils.sample_uniform(
            low,
            high,
            (env_ids.numel(), 1),
            env.device,
        )

        randomization["poissons_ratio"][
            env_ids
        ] = sampled[:, 0]

        poissons_ratio_np[material_ids_np] = (
            sampled.detach()
            .cpu()
            .numpy()
            .astype(np.float32)
        )

        poissons_ratio_wp = wp.from_numpy(
            poissons_ratio_np,
            dtype=wp.float32,
            device="cpu",
        )

        material_view.set_poissons_ratio(
            poissons_ratio_wp,
            material_ids_wp,
        )
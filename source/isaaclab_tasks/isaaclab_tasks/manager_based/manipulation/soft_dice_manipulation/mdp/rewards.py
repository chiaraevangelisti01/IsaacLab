from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg

from ..utils.geometry_utils import (
    canonical_cube_orientation_error,
    orientation_error,
    orientation_in_frame,
    vector_error,
)

from ..utils.reward_utils import (
    decreasing_phase_scale,
    distance_to_xy_disk,
    smooth_phase_blend,
)

from ..utils.motion_utils import (
    H1_TRACKED_BODY_NAMES,
)

from .observations import object_pos_r0
from ..utils.deformable_utils import (
    compute_deformation_rms,
)

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

def late_motion_tracking_scale(
    motion: MotionCommand,
    final_scale: float,
) -> torch.Tensor:
    """Reduce robot reference-tracking authority near task completion.

    Relaxation begins at the LATER of the position and orientation
    landing phases and reaches final_scale at release.

    """

    start_phase = torch.maximum(
        motion.position_landing_start_phase,
        motion.orientation_landing_start_phase,
    )

    return decreasing_phase_scale(
        phase=motion.phase,
        start_phase=start_phase,
        end_phase=motion.release_phase,
        final_scale=final_scale,
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


###### Phase weighted rewards
def motion_phase_weighted_body_position_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    final_tracking_scale: float,
    include_hands: bool = False,
) -> torch.Tensor:
    """Track body positions, but relax tracking near final landing."""

    motion: MotionCommand = (
        env.command_manager.get_term(
            command_name
        )
    )

    reward = motion_relative_body_position_error_exp(
        env=env,
        command_name=command_name,
        std=std,
        include_hands=include_hands,
    )

    scale = late_motion_tracking_scale(
        motion=motion,
        final_scale=final_tracking_scale,
    )

    return scale * reward


def motion_phase_weighted_body_orientation_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    final_tracking_scale: float,
) -> torch.Tensor:
    """Track body orientation, but relax tracking near final landing."""

    motion: MotionCommand = (
        env.command_manager.get_term(
            command_name
        )
    )

    reward = motion_relative_body_orientation_error_exp(
        env=env,
        command_name=command_name,
        std=std,
    )

    scale = late_motion_tracking_scale(
        motion=motion,
        final_scale=final_tracking_scale,
    )

    return scale * reward


def motion_phase_weighted_body_linear_velocity_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    final_tracking_scale: float,
) -> torch.Tensor:
    """Track body linear velocity, but relax tracking near final landing."""

    motion: MotionCommand = (
        env.command_manager.get_term(
            command_name
        )
    )

    reward = motion_global_body_linear_velocity_error_exp(
        env=env,
        command_name=command_name,
        std=std,
    )

    scale = late_motion_tracking_scale(
        motion=motion,
        final_scale=final_tracking_scale,
    )

    return scale * reward


def motion_phase_weighted_body_angular_velocity_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    final_tracking_scale: float,
) -> torch.Tensor:
    """Track body angular velocity, but relax tracking near final landing."""

    motion: MotionCommand = (
        env.command_manager.get_term(
            command_name
        )
    )

    reward = motion_global_body_angular_velocity_error_exp(
        env=env,
        command_name=command_name,
        std=std,
    )

    scale = late_motion_tracking_scale(
        motion=motion,
        final_scale=final_tracking_scale,
    )

    return scale * reward

def object_phase_weighted_ref_position_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    final_reference_scale: float,
) -> torch.Tensor:
    """Track object reference position while gradually reducing its authority."""

    motion: MotionCommand = (
        env.command_manager.get_term(
            command_name
        )
    )

    reference_reward = (
        object_global_ref_position_error_exp(
            env=env,
            command_name=command_name,
            std=std,
        )
    )

    scale = decreasing_phase_scale(
        phase=motion.phase,
        start_phase=(
            motion.position_landing_start_phase
        ),
        end_phase=motion.release_phase,
        final_scale=final_reference_scale,
    )

    return (
        scale
        * reference_reward
    )


def object_phase_weighted_ref_orientation_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    final_reference_scale: float,
) -> torch.Tensor:
    """Track object reference orientation while gradually reducing its authority."""

    motion: MotionCommand = (
        env.command_manager.get_term(
            command_name
        )
    )

    reference_reward = (
        object_global_ref_orientation_error_exp(
            env=env,
            command_name=command_name,
            std=std,
        )
    )

    scale = decreasing_phase_scale(
        phase=motion.phase,
        start_phase=(
            motion.orientation_landing_start_phase
        ),
        end_phase=motion.release_phase,
        final_scale=final_reference_scale,
    )

    return (
        scale
        * reference_reward
    )

def landing_position_region_reward_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    goal_xy: tuple[float, float],
    radius: float,
    std: float,
) -> torch.Tensor:
    """Reward placing the cube inside a canonical XY landing region.

    The landing region is expressed in the fixed nominal robot-root
    frame. All positions inside the disk receive the same spatial
    reward.

    The reward is phase gated:
        - zero before landing_start,
        - smoothly activated between landing_start and release,
        - fully active after release.
    """

    motion: MotionCommand = (
        env.command_manager.get_term(
            command_name
        )
    )

    # --------------------------------------------------------------
    # Current cube position in the fixed nominal robot frame.
    # --------------------------------------------------------------

    cube_pos_r0 = object_pos_r0(
        env=env,
        command_name=command_name,
    )

    goal_xy_tensor = torch.as_tensor(
        goal_xy,
        dtype=cube_pos_r0.dtype,
        device=cube_pos_r0.device,
    )

    # --------------------------------------------------------------
    # Distance to the valid landing region.
    #
    # Inside the disk:
    #     distance = 0
    #
    # Outside:
    #     distance = distance_from_center - radius
    # --------------------------------------------------------------

    distance = distance_to_xy_disk(
        position=cube_pos_r0,
        center_xy=goal_xy_tensor,
        radius=radius,
    )

    spatial_reward = torch.exp(
        -torch.square(distance)
        / (std * std)
    )

    # --------------------------------------------------------------
    # Activate task objective as the demonstrated manipulation
    # approaches its landing/release phase.
    # --------------------------------------------------------------

    alpha = smooth_phase_blend(
        phase=motion.phase,
        start_phase=(
            motion.position_landing_start_phase
        ),
        end_phase=motion.release_phase,
    )

    return (
        alpha
        * spatial_reward
    )

def landing_orientation_reward_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
) -> torch.Tensor:
    """Reward the intended canonical final cube orientation.

    The demonstrated final pose determines:
        - final top face;
        - nearest 90-degree yaw state.

    The reward itself targets the exact canonical orientation,
    not the imperfect demonstrated quaternion.
    """

    motion: MotionCommand = (
        env.command_manager.get_term(
            command_name
        )
    )

    robot_quat_e = torch.as_tensor(
        motion.robot.cfg.init_state.rot,
        dtype=motion.simulator_cube_quat.dtype,
        device=motion.simulator_cube_quat.device,
    ).unsqueeze(
        0
    ).expand(
        env.num_envs,
        -1,
    )

    # ----------------------------------------------------------
    # Put reference and simulated cube in the same fixed
    # nominal robot frame.
    # ----------------------------------------------------------

    final_reference_quat_r0 = (
        orientation_in_frame(
            orientation=(
                motion.final_cube_quat
            ),
            frame_orientation=(
                robot_quat_e
            ),
        )
    )

    current_cube_quat_r0 = (
        orientation_in_frame(
            orientation=(
                motion.simulator_cube_quat
            ),
            frame_orientation=(
                robot_quat_e
            ),
        )
    )

    # ----------------------------------------------------------
    # Desired orientation:
    #
    #   correct top face
    #       +
    #   nearest 0/90/180/270 yaw state
    # ----------------------------------------------------------

    error = (
        canonical_cube_orientation_error(
            reference_quat=(
                final_reference_quat_r0
            ),
            current_quat=(
                current_cube_quat_r0
            ),
        )
    )

    orientation_reward = torch.exp(
        -torch.square(
            error
        )
        / (std * std)
    )

    alpha = smooth_phase_blend(
        phase=motion.phase,
        start_phase=(
            motion.orientation_landing_start_phase
        ),
        end_phase=(
            motion.release_phase
        ),
    )

    return (
        alpha
        * orientation_reward
    )

def object_deformation_rms(
    env: ManagerBasedRLEnv,
    command_name: str,
) -> torch.Tensor:

    motion: MotionCommand = (
        env.command_manager.get_term(command_name)
    )

    reference_nodal_pos = (
        motion.cube.data.default_nodal_state_w.torch[..., :3]
    )
    current_nodal_pos = (
        motion.cube.data.nodal_pos_w.torch
    )

    return compute_deformation_rms(
        reference_nodal_pos=reference_nodal_pos,
        current_nodal_pos=current_nodal_pos,
        rotation=motion.simulator_cube_rotation,
    )
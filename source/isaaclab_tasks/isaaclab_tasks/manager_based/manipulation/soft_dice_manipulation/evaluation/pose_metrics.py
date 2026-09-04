from __future__ import annotations

import torch

from ..utils.geometry_utils import canonical_cube_orientation_error, orientation_error, relative_euler_xyz_error, same_top_face, vector_error, xy_position_error, z_position_error
from ..utils.reward_utils import distance_to_xy_disk


def compute_terminal_cube_pose_metrics(
    reference_pos: torch.Tensor,
    current_pos: torch.Tensor,
    reference_quat: torch.Tensor,
    current_quat: torch.Tensor,
) -> dict[str, torch.Tensor]:
    """Compute terminal dice pose evaluation metrics."""

    position_delta = current_pos - reference_pos

    position_error_3d_m = vector_error(reference_pos, current_pos)
    position_error_xy_m = xy_position_error(reference_pos, current_pos)
    position_error_z_m = z_position_error(reference_pos, current_pos)

    orientation_error_rad = orientation_error(reference_quat, current_quat)
    rpy_error_rad = relative_euler_xyz_error(reference_quat, current_quat)

    top_face_correct = same_top_face(
        reference_quat,
        current_quat,
    ).to(torch.float32)

    return {
        "final_position_error_m": position_error_3d_m,
        "final_xy_position_error_m": position_error_xy_m,
        "final_z_position_error_m": position_error_z_m,

        "final_x_error_m": position_delta[..., 0],
        "final_y_error_m": position_delta[..., 1],

        "final_orientation_error_rad": orientation_error_rad,
        "final_orientation_error_deg": torch.rad2deg(orientation_error_rad),

        "final_roll_error_deg": torch.rad2deg(rpy_error_rad[..., 0]),
        "final_pitch_error_deg": torch.rad2deg(rpy_error_rad[..., 1]),
        "final_yaw_error_deg": torch.rad2deg(rpy_error_rad[..., 2]),

        "final_top_face_correct": top_face_correct,
    }

def compute_terminal_cube_landing_metrics(
    reference_pos_r0: torch.Tensor,
    current_pos_r0: torch.Tensor,
    reference_quat_r0: torch.Tensor,
    current_quat_r0: torch.Tensor,
    landing_center_xy: tuple[float, float],
    landing_radius: float,
    position_improvement_threshold_m: float,
    orientation_improvement_threshold_deg: float,
) -> dict[str, torch.Tensor]:
    """Compute terminal task-space landing metrics.

    Position is evaluated against the valid XY landing disk rather than the
    demonstrated final position. Orientation is evaluated against the exact
    canonical cube orientation inferred from the final demonstration.

    Positive improvement means that the policy terminal state is closer to
    the task objective than the demonstrated final state.
    """

    center_xy = torch.as_tensor(landing_center_xy, dtype=current_pos_r0.dtype, device=current_pos_r0.device)

    reference_center_distance_m = torch.linalg.norm(reference_pos_r0[..., :2] - center_xy, dim=-1)
    current_center_distance_m = torch.linalg.norm(current_pos_r0[..., :2] - center_xy, dim=-1)

    reference_landing_error_m = distance_to_xy_disk(position=reference_pos_r0, center_xy=center_xy, radius=landing_radius)
    current_landing_error_m = distance_to_xy_disk(position=current_pos_r0, center_xy=center_xy, radius=landing_radius)

    position_improvement_m = reference_landing_error_m - current_landing_error_m

    reference_orientation_error_rad = canonical_cube_orientation_error(
        reference_quat=reference_quat_r0,
        current_quat=reference_quat_r0,
    )
    current_orientation_error_rad = canonical_cube_orientation_error(
        reference_quat=reference_quat_r0,
        current_quat=current_quat_r0,
    )

    orientation_improvement_rad = reference_orientation_error_rad - current_orientation_error_rad

    reference_inside_region = reference_landing_error_m <= 1.0e-6
    current_inside_region = current_landing_error_m <= 1.0e-6

    orientation_improvement_deg = torch.rad2deg(orientation_improvement_rad)

    position_improved = position_improvement_m > position_improvement_threshold_m
    position_worsened = position_improvement_m < -position_improvement_threshold_m

    orientation_improved = orientation_improvement_deg > orientation_improvement_threshold_deg
    orientation_worsened = orientation_improvement_deg < -orientation_improvement_threshold_deg

    top_face_correct = same_top_face(reference_quat_r0, current_quat_r0)

    return {
        "reference_landing_xy_error_m": reference_landing_error_m,
        "final_landing_xy_error_m": current_landing_error_m,
        "landing_position_improvement_m": position_improvement_m,
        "landing_position_improved": position_improved.to(torch.float32),
        "landing_position_worsened": position_worsened.to(torch.float32),

        "reference_landing_center_distance_m": reference_center_distance_m,
        "final_landing_center_distance_m": current_center_distance_m,

        "reference_inside_landing_region": reference_inside_region.to(torch.float32),
        "final_inside_landing_region": current_inside_region.to(torch.float32),

        "reference_landing_orientation_error_rad": reference_orientation_error_rad,
        "reference_landing_orientation_error_deg": torch.rad2deg(reference_orientation_error_rad),

        "final_landing_orientation_error_rad": current_orientation_error_rad,
        "final_landing_orientation_error_deg": torch.rad2deg(current_orientation_error_rad),

        "landing_orientation_improvement_rad": orientation_improvement_rad,
        "landing_orientation_improvement_deg": orientation_improvement_deg,
        "landing_orientation_improved": orientation_improved.to(torch.float32),
        "landing_orientation_worsened": orientation_worsened.to(torch.float32),

        "final_top_face_correct": top_face_correct.to(torch.float32),
    }


def compute_terminal_task_success_metrics(
    reference_landing_xy_error_m: torch.Tensor,
    final_landing_xy_error_m: torch.Tensor,
    reference_landing_orientation_error_deg: torch.Tensor,
    final_landing_orientation_error_deg: torch.Tensor,
    final_top_face_correct: torch.Tensor,
    motion_finished: torch.Tensor,
    orientation_threshold_deg: float,
    epsilon: float = 1.0e-6,
) -> dict[str, torch.Tensor]:
    """Compute binary terminal landing-success metrics."""

    reference_position_success = reference_landing_xy_error_m <= epsilon
    final_position_success = final_landing_xy_error_m <= epsilon

    reference_orientation_success = reference_landing_orientation_error_deg <= orientation_threshold_deg
    final_orientation_success = final_landing_orientation_error_deg <= orientation_threshold_deg

    top_face_success = final_top_face_correct > 0.5

    reference_landing_success = reference_position_success & reference_orientation_success
    final_landing_success = final_position_success & final_orientation_success & top_face_success

    task_success = motion_finished.bool() & final_landing_success

    landing_success_gain = final_landing_success & (~reference_landing_success)
    landing_success_loss = reference_landing_success & (~final_landing_success)

    return {
        "reference_landing_position_success": reference_position_success.to(torch.float32),
        "final_landing_position_success": final_position_success.to(torch.float32),
        "reference_landing_orientation_success": reference_orientation_success.to(torch.float32),
        "final_landing_orientation_success": final_orientation_success.to(torch.float32),
        "final_top_face_success": top_face_success.to(torch.float32),
        "reference_landing_success": reference_landing_success.to(torch.float32),
        "final_landing_success": final_landing_success.to(torch.float32),
        "landing_success_gain": landing_success_gain.to(torch.float32),
        "landing_success_loss": landing_success_loss.to(torch.float32),
        "task_success": task_success.to(torch.float32),
    }
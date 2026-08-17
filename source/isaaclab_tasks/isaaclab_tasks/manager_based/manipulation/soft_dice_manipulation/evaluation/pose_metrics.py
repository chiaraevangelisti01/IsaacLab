from __future__ import annotations

import torch

from ..utils.geometry_utils import (
    orientation_error,
    relative_euler_xyz_error,
    same_top_face,
    vector_error,
    xy_position_error,
    z_position_error,
)


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
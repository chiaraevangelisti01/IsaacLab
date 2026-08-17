from __future__ import annotations

import torch

from ..utils.geometry_utils import (
    orientation_error,
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
    """Compute terminal dice pose evaluation metrics.

    Args:
        reference_pos:
            Desired terminal dice position, shape (..., 3).
        current_pos:
            Actual terminal dice centroid, shape (..., 3).
        reference_quat:
            Desired terminal dice orientation in XYZW, shape (..., 4).
        current_quat:
            Actual terminal bulk dice orientation from Kabsch in XYZW,
            shape (..., 4).

    Returns:
        Dictionary of pose-error tensors.
    """

    position_error_3d_m = vector_error(
        reference=reference_pos,
        current=current_pos,
    )

    position_error_xy_m = xy_position_error(
        reference_pos=reference_pos,
        current_pos=current_pos,
    )

    position_error_z_m = z_position_error(
        reference_pos=reference_pos,
        current_pos=current_pos,
    )

    orientation_error_rad = orientation_error(
        reference_quat=reference_quat,
        current_quat=current_quat,
    )

    return {
        "final_position_error_m":
            position_error_3d_m,

        "final_xy_position_error_m":
            position_error_xy_m,

        "final_z_position_error_m":
            position_error_z_m,

        "final_orientation_error_rad":
            orientation_error_rad,

        "final_orientation_error_deg":
            torch.rad2deg(
                orientation_error_rad
            ),
    }
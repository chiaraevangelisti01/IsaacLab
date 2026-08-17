from __future__ import annotations

import torch

from isaaclab.utils.math import quat_error_magnitude


def vector_error(
    reference: torch.Tensor,
    current: torch.Tensor,
) -> torch.Tensor:
    """Euclidean error between vectors along the last dimension."""
    return torch.linalg.norm(
        current - reference,
        dim=-1,
    )


def xy_position_error(
    reference_pos: torch.Tensor,
    current_pos: torch.Tensor,
) -> torch.Tensor:
    """Planar XY position error.

    Args:
        reference_pos: Reference position with shape (..., 3).
        current_pos: Current position with shape (..., 3).

    Returns:
        XY Euclidean distance with shape (...).
    """
    return vector_error(
        reference_pos[..., :2],
        current_pos[..., :2],
    )

def z_position_error(
    reference_pos: torch.Tensor,
    current_pos: torch.Tensor,
) -> torch.Tensor:
    """Absolute vertical position error."""
    return torch.abs(
        current_pos[..., 2]
        - reference_pos[..., 2]
    )


def orientation_error(
    reference_quat: torch.Tensor,
    current_quat: torch.Tensor,
) -> torch.Tensor:
    """Shortest angular difference between two XYZW quaternions.

    Returns:
        Angular error in radians with shape (...).
    """
    return quat_error_magnitude(
        reference_quat,
        current_quat,
    )
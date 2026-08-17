from __future__ import annotations

import torch

from isaaclab.utils.math import (
    euler_xyz_from_quat,
    quat_apply_inverse,
    quat_error_magnitude,
    quat_inv,
    quat_mul,
)


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

def relative_euler_xyz_error(
    reference_quat: torch.Tensor,
    current_quat: torch.Tensor,
) -> torch.Tensor:
    """Signed XYZ Euler decomposition of current relative to reference."""

    relative_quat = quat_mul(current_quat, quat_inv(reference_quat))
    original_shape = relative_quat.shape[:-1]

    roll, pitch, yaw = euler_xyz_from_quat(relative_quat.reshape(-1, 4))

    return torch.stack((roll, pitch, yaw), dim=-1).reshape(*original_shape, 3)


def top_face_index(quat: torch.Tensor) -> torch.Tensor:
    """Return the local cube face currently pointing upward.
    """

    world_up = torch.zeros_like(quat[..., :3])
    world_up[..., 2] = 1.0

    local_up = quat_apply_inverse(quat, world_up)
    axis = torch.argmax(torch.abs(local_up), dim=-1)

    signed_component = torch.gather(local_up, -1, axis.unsqueeze(-1)).squeeze(-1)
    negative = (signed_component < 0.0).long()

    return 2 * axis + negative


def same_top_face(
    reference_quat: torch.Tensor,
    current_quat: torch.Tensor,
) -> torch.Tensor:
    """Whether current and reference have the same physical cube face on top."""

    return top_face_index(reference_quat) == top_face_index(current_quat)
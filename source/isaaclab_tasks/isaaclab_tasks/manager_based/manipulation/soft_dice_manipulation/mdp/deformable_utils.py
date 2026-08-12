from __future__ import annotations

import torch

from isaaclab.utils.math import quat_from_matrix, quat_unique


def estimate_deformable_orientation_kabsch(
    reference_nodal_pos: torch.Tensor,
    current_nodal_pos: torch.Tensor,
) -> torch.Tensor:
    """Estimate the bulk rigid orientation of a deformable object.

    The rotation is the least-squares rigid rotation mapping the
    reference nodal cloud onto the current nodal cloud.

    Args:
        reference_nodal_pos:
            Reference nodal positions with shape (..., num_nodes, 3).
        current_nodal_pos:
            Current nodal positions with the same shape.

    Returns:
        Estimated orientation as an XYZW quaternion with shape (..., 4).
    """

    if reference_nodal_pos.shape != current_nodal_pos.shape:
        raise ValueError(
            "Reference and current nodal positions must have the same shape. "
            f"Got {reference_nodal_pos.shape} and {current_nodal_pos.shape}."
        )

    # Remove translation.
    reference_centered = (
        reference_nodal_pos
        - reference_nodal_pos.mean(dim=-2, keepdim=True)
    )
    current_centered = (
        current_nodal_pos
        - current_nodal_pos.mean(dim=-2, keepdim=True)
    )

    # Cross-covariance between corresponding material nodes.
    covariance = (
        reference_centered.transpose(-1, -2)
        @ current_centered
    )

    u, _, vh = torch.linalg.svd(covariance)

    v = vh.transpose(-1, -2)
    u_t = u.transpose(-1, -2)

    # Kabsch reflection correction: enforce R in SO(3), det(R) = +1.
    correction = torch.ones(
        (*covariance.shape[:-2], 3),
        dtype=covariance.dtype,
        device=covariance.device,
    )

    correction[..., -1] = torch.where(
        torch.det(v @ u_t) < 0.0,
        -1.0,
        1.0,
    )

    rotation = (
        v
        @ torch.diag_embed(correction)
        @ u_t
    )

    return quat_unique(
        quat_from_matrix(rotation)
    )
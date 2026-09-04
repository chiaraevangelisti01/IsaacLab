from __future__ import annotations

import torch

from isaaclab.utils.math import quat_from_matrix, quat_unique


def _validate_nodal_positions(
    reference_nodal_pos: torch.Tensor,
    current_nodal_pos: torch.Tensor,
) -> None:
    if reference_nodal_pos.shape != current_nodal_pos.shape:
        raise ValueError(
            "Reference and current nodal positions must have the same shape. "
            f"Got {reference_nodal_pos.shape} and {current_nodal_pos.shape}."
        )

    if reference_nodal_pos.ndim < 2 or reference_nodal_pos.shape[-1] != 3:
        raise ValueError(
            "Nodal positions must have shape (..., num_nodes, 3). "
            f"Got {reference_nodal_pos.shape}."
        )


def estimate_deformable_rigid_transform_kabsch(
    reference_nodal_pos: torch.Tensor,
    current_nodal_pos: torch.Tensor,
    return_singular_values: bool = False,
):
    """Estimate the best-fit rigid transform from reference to current nodes."""

    _validate_nodal_positions(
        reference_nodal_pos,
        current_nodal_pos,
    )

    reference_center = reference_nodal_pos.mean(
        dim=-2
    )
    current_center = current_nodal_pos.mean(
        dim=-2
    )

    reference_centered = (
        reference_nodal_pos
        - reference_center.unsqueeze(-2)
    )

    current_centered = (
        current_nodal_pos
        - current_center.unsqueeze(-2)
    )

    covariance = (
        reference_centered.transpose(-1, -2)
        @ current_centered
    )

    covariance_svd = covariance.to(
        torch.float64
    )

    u, singular_values, vh = torch.linalg.svd(
        covariance_svd
    )

    v = vh.transpose(-1, -2)
    u_t = u.transpose(-1, -2)

    correction = torch.ones(
        (*covariance_svd.shape[:-2], 3),
        dtype=covariance_svd.dtype,
        device=covariance_svd.device,
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

    rotation = rotation.to(
        reference_nodal_pos.dtype
    )

    translation = current_center - (
        rotation
        @ reference_center.unsqueeze(-1)
    ).squeeze(-1)

    if return_singular_values:
        return (
            rotation,
            translation,
            singular_values,
        )

    return rotation, translation


def estimate_deformable_orientation_kabsch(
    reference_nodal_pos: torch.Tensor,
    current_nodal_pos: torch.Tensor,
    return_diagnostics: bool = False,
):
    """Estimate the bulk rigid orientation of a deformable object."""

    if return_diagnostics:
        (
            rotation,
            _,
            singular_values,
            ) = estimate_deformable_rigid_transform_kabsch(
            reference_nodal_pos,
            current_nodal_pos,
            return_singular_values=True,
        )

        quat = quat_unique(
            quat_from_matrix(rotation)
        )

        return quat, singular_values

    rotation, _ = (
        estimate_deformable_rigid_transform_kabsch(
            reference_nodal_pos,
            current_nodal_pos,
        )
    )

    return quat_unique(
        quat_from_matrix(rotation)
    )

def compute_deformable_shape_metrics(
    reference_nodal_pos: torch.Tensor,
    current_nodal_pos: torch.Tensor,
    extent_eps: float = 1.0e-8,
) -> dict[str, torch.Tensor]:
    """Compute deformation after removing the best-fit rigid motion.

    Returns:
        nodal_deformation_m:
            Per-node rigid-alignment residual, shape (..., N).
        deformation_rms_m:
            RMS nodal deformation, shape (...).
        deformation_p95_m:
            95th percentile nodal deformation, shape (...).
        deformation_max_m:
            Maximum nodal deformation, shape (...).
        reference_extent_m:
            Reference material-frame X/Y/Z extents, shape (..., 3).
        current_extent_m:
            Current material-frame X/Y/Z extents, shape (..., 3).
        relative_extent_change:
            Relative X/Y/Z extent change, shape (..., 3).
            Positive means expansion, negative means compression.
    """

    rotation, _ = estimate_deformable_rigid_transform_kabsch(
        reference_nodal_pos,
        current_nodal_pos,
    )

    reference_centered = (
        reference_nodal_pos
        - reference_nodal_pos.mean(dim=-2, keepdim=True)
    )
    current_centered = (
        current_nodal_pos
        - current_nodal_pos.mean(dim=-2, keepdim=True)
    )

    # Reference cloud after applying only the best-fit rigid rotation.
    rigid_prediction_centered = (
        reference_centered @ rotation.transpose(-1, -2)
    )

    nodal_residual = current_centered - rigid_prediction_centered
    nodal_deformation_m = torch.linalg.norm(nodal_residual, dim=-1)

    deformation_rms_m = torch.sqrt(
        torch.mean(nodal_deformation_m.square(), dim=-1)
    )
    deformation_p95_m = torch.quantile(
        nodal_deformation_m,
        0.95,
        dim=-1,
    )
    deformation_max_m = torch.amax(nodal_deformation_m, dim=-1)

    # Rotate the current cloud back into the material/reference frame.
    current_material = current_centered @ rotation

    reference_extent_m = (
        reference_centered.amax(dim=-2)
        - reference_centered.amin(dim=-2)
    )
    current_extent_m = (
        current_material.amax(dim=-2)
        - current_material.amin(dim=-2)
    )

    relative_extent_change = (
        current_extent_m - reference_extent_m
    ) / reference_extent_m.clamp_min(extent_eps)

    return {
        "nodal_deformation_m": nodal_deformation_m,
        "deformation_rms_m": deformation_rms_m,
        "deformation_p95_m": deformation_p95_m,
        "deformation_max_m": deformation_max_m,
        "reference_extent_m": reference_extent_m,
        "current_extent_m": current_extent_m,
        "relative_extent_change": relative_extent_change,
    }

def compute_deformation_rms(
    reference_nodal_pos: torch.Tensor,
    current_nodal_pos: torch.Tensor,
    rotation: torch.Tensor,
) -> torch.Tensor:
    """Compute RMS non-rigid deformation using a precomputed rigid rotation.

    Args:
        reference_nodal_pos:
            Undeformed nodal positions, shape (..., N, 3).

        current_nodal_pos:
            Current nodal positions, shape (..., N, 3).

        rotation:
            Best-fit Kabsch rotation from reference to current,
            shape (..., 3, 3).

    Returns:
        RMS nodal deformation, shape (...).
    """

    # Remove translation from both clouds.
    reference_centered = (
        reference_nodal_pos
        - reference_nodal_pos.mean(dim=-2, keepdim=True)
    )

    current_centered = (
        current_nodal_pos
        - current_nodal_pos.mean(dim=-2, keepdim=True)
    )

    # Where the undeformed cloud would be if it had undergone
    # only the best-fit rigid rotation.
    rigid_prediction_centered = (
        reference_centered
        @ rotation.transpose(-1, -2)
    )

    # Anything left is non-rigid deformation.
    nodal_residual = (
        current_centered
        - rigid_prediction_centered
    )

    # Euclidean deformation of every node.
    nodal_deformation_m = torch.linalg.norm(
        nodal_residual,
        dim=-1,
    )

    # RMS across all nodes.
    deformation_rms_m = torch.sqrt(
        torch.mean(
            nodal_deformation_m.square(),
            dim=-1,
        )
    )

    return deformation_rms_m

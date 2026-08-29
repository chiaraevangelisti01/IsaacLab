from __future__ import annotations

import torch


def smooth_phase_blend(
    phase: torch.Tensor,
    start_phase: torch.Tensor,
    end_phase: torch.Tensor,
) -> torch.Tensor:
    """Smooth transition from 0 to 1 over a trajectory phase interval.

    Returns:
        0 before start_phase,
        1 after end_phase,
        cubic smoothstep interpolation in between.
    """

    denominator = (
        end_phase - start_phase
    ).clamp_min(1.0e-6)

    x = (
        (phase - start_phase)
        / denominator
    ).clamp(
        0.0,
        1.0,
    )

    return x * x * (
        3.0 - 2.0 * x
    )


def decreasing_phase_scale(
    phase: torch.Tensor,
    start_phase: torch.Tensor,
    end_phase: torch.Tensor,
    final_scale: float,
) -> torch.Tensor:
    """Decrease a scale smoothly from 1.0 to final_scale."""

    alpha = smooth_phase_blend(
        phase=phase,
        start_phase=start_phase,
        end_phase=end_phase,
    )

    return (
        1.0
        - (1.0 - final_scale)
        * alpha
    )


def distance_to_xy_disk(
    position: torch.Tensor,
    center_xy: torch.Tensor,
    radius: float,
) -> torch.Tensor:
    """Distance from a 3D position to a valid XY disk region.

    Distance is zero anywhere inside the disk.

    Args:
        position:
            Positions with shape (..., 3).

        center_xy:
            Center of the disk with shape (2,).

        radius:
            Disk radius in meters.

    Returns:
        Distance to the disk boundary, shape (...).
    """

    center_distance = torch.linalg.norm(
        position[..., :2] - center_xy,
        dim=-1,
    )

    return torch.clamp(
        center_distance - radius,
        min=0.0,
    )
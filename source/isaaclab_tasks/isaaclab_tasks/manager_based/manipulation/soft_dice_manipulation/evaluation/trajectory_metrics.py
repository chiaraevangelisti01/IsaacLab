from __future__ import annotations

import torch

from ..utils.motion_utils import (
    H1_HAND_REFERENCE_NAMES,
    H1_TRACKED_BODY_NAMES,
)


def _rms(
    error: torch.Tensor,
    dim,
) -> torch.Tensor:
    """Root-mean-square of an error magnitude."""
    return torch.sqrt(
        torch.mean(
            torch.square(error),
            dim=dim,
        )
    )


def compute_cartesian_trajectory_metrics(
    body_position_error_m: torch.Tensor,
    body_orientation_error_rad: torch.Tensor,
    hand_position_error_m: torch.Tensor,
    cube_position_error_m: torch.Tensor,
    cube_xy_position_error_m: torch.Tensor,
    cube_orientation_error_rad: torch.Tensor,
) -> dict[str, torch.Tensor]:
    """Compute episode-level Cartesian trajectory tracking metrics.

    Args:
        body_position_error_m:
            Per-frame Cartesian position error for tracked bodies.
            Shape: (T, num_bodies).

        body_orientation_error_rad:
            Per-frame orientation error for tracked bodies.
            Shape: (T, num_bodies).

        hand_position_error_m:
            Per-frame Cartesian position error for virtual hands.
            Shape: (T, num_hands).

        cube_position_error_m:
            Per-frame 3D cube position error.
            Shape: (T,).

        cube_xy_position_error_m:
            Per-frame planar cube position error.
            Shape: (T,).

        cube_orientation_error_rad:
            Per-frame cube orientation error.
            Shape: (T,).

    Returns:
        Dictionary containing scalar episode metrics.
    """

    if body_position_error_m.shape[0] == 0:
        raise ValueError(
            "Cannot compute trajectory metrics from an empty episode."
        )

    body_position_rmse_per_body = _rms(
        body_position_error_m,
        dim=0,
    )

    body_orientation_rmse_rad_per_body = _rms(
        body_orientation_error_rad,
        dim=0,
    )

    hand_position_rmse_per_hand = _rms(
        hand_position_error_m,
        dim=0,
    )

    metrics = {
        # --------------------------------------------------------------
        # Overall tracked-body Cartesian tracking.
        # --------------------------------------------------------------
        "body_position_rmse_m": _rms(
            body_position_error_m,
            dim=(0, 1),
        ),

        "body_orientation_rmse_deg": torch.rad2deg(
            _rms(
                body_orientation_error_rad,
                dim=(0, 1),
            )
        ),

        # --------------------------------------------------------------
        # Overall hand tracking.
        # --------------------------------------------------------------
        "hand_position_rmse_m": _rms(
            hand_position_error_m,
            dim=(0, 1),
        ),

        # --------------------------------------------------------------
        # Cube trajectory tracking.
        # --------------------------------------------------------------
        "cube_position_rmse_m": _rms(
            cube_position_error_m,
            dim=0,
        ),

        "cube_xy_position_rmse_m": _rms(
            cube_xy_position_error_m,
            dim=0,
        ),

        "cube_orientation_rmse_deg": torch.rad2deg(
            _rms(
                cube_orientation_error_rad,
                dim=0,
            )
        ),
    }

    # ------------------------------------------------------------------
    # Per-body metrics.
    # ------------------------------------------------------------------

    for body_index, body_name in enumerate(
        H1_TRACKED_BODY_NAMES
    ):
        metrics[
            f"body_position_rmse_{body_name}_m"
        ] = (
            body_position_rmse_per_body[
                body_index
            ]
        )

        metrics[
            f"body_orientation_rmse_{body_name}_deg"
        ] = torch.rad2deg(
            body_orientation_rmse_rad_per_body[
                body_index
            ]
        )

    # ------------------------------------------------------------------
    # Per-hand metrics.
    # ------------------------------------------------------------------

    for hand_index, hand_name in enumerate(
        H1_HAND_REFERENCE_NAMES
    ):
        metrics[
            f"hand_position_rmse_{hand_name}_m"
        ] = (
            hand_position_rmse_per_hand[
                hand_index
            ]
        )

    return metrics
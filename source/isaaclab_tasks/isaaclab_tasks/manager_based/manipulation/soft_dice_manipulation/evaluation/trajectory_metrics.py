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

def _masked_rms(
    error: torch.Tensor,
    mask: torch.Tensor,
    dim,
) -> torch.Tensor:
    if mask.ndim != 1 or mask.shape[0] != error.shape[0]:
        raise ValueError(
            "Tracking mask must have shape (T,)."
        )

    if not torch.any(mask):
        raise ValueError(
            "Tracking mask contains no valid frames."
        )

    return _rms(
        error[mask],
        dim=dim,
    )


def compute_cartesian_trajectory_metrics(
    body_position_error_m: torch.Tensor,
    body_orientation_error_rad: torch.Tensor,
    hand_position_error_m: torch.Tensor,
    cube_xy_position_error_m: torch.Tensor,
    cube_orientation_error_rad: torch.Tensor,
    phase: torch.Tensor | None = None,
    position_landing_start_phase: float | None = None,
    orientation_landing_start_phase: float | None = None,
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

    if phase is not None:
        if position_landing_start_phase is None or orientation_landing_start_phase is None:
            raise ValueError(
                "Landing phase thresholds are required when phase-aware tracking is requested."
            )

        phase = phase.to(
            dtype=body_position_error_m.dtype,
            device=body_position_error_m.device,
        )

        position_start_phase = torch.as_tensor(
            position_landing_start_phase,
            dtype=phase.dtype,
            device=phase.device,
        )

        orientation_start_phase = torch.as_tensor(
            orientation_landing_start_phase,
            dtype=phase.dtype,
            device=phase.device,
        )

        robot_tracking_start_phase = torch.maximum(
            position_start_phase,
            orientation_start_phase,
        )

        phase_epsilon = 1.0e-6

        robot_tracking_mask = (phase<= robot_tracking_start_phase + phase_epsilon)
        cube_position_tracking_mask = (phase<= position_start_phase + phase_epsilon)
        cube_orientation_tracking_mask = (phase<= orientation_start_phase + phase_epsilon)

        pre_body_position_rmse_per_body = _masked_rms(body_position_error_m,robot_tracking_mask,dim=0)
        pre_body_orientation_rmse_rad_per_body = _masked_rms(body_orientation_error_rad,robot_tracking_mask,dim=0)
        pre_hand_position_rmse_per_hand = _masked_rms(hand_position_error_m,robot_tracking_mask,dim=0)

        metrics.update(
            {
                "pre_relaxation_body_position_rmse_m": _masked_rms(
                    body_position_error_m,
                    robot_tracking_mask,
                    dim=(0, 1),
                ),
                "pre_relaxation_body_orientation_rmse_deg": torch.rad2deg(
                    _masked_rms(
                        body_orientation_error_rad,
                        robot_tracking_mask,
                        dim=(0, 1),
                    )
                ),
                "pre_relaxation_hand_position_rmse_m": _masked_rms(
                    hand_position_error_m,
                    robot_tracking_mask,
                    dim=(0, 1),
                ),
                "pre_relaxation_cube_xy_position_rmse_m": _masked_rms(
                    cube_xy_position_error_m,
                    cube_position_tracking_mask,
                    dim=0,
                ),
                "pre_relaxation_cube_orientation_rmse_deg": torch.rad2deg(
                    _masked_rms(
                        cube_orientation_error_rad,
                        cube_orientation_tracking_mask,
                        dim=0,
                    )
                ),
                "pre_relaxation_robot_tracking_frames": robot_tracking_mask.sum().to(torch.float32),
                "pre_relaxation_cube_position_tracking_frames": cube_position_tracking_mask.sum().to(torch.float32),
                "pre_relaxation_cube_orientation_tracking_frames": cube_orientation_tracking_mask.sum().to(torch.float32),
            }
        )

        for body_index, body_name in enumerate(H1_TRACKED_BODY_NAMES):

            metrics[f"pre_relaxation_body_position_rmse_{body_name}_m"] = pre_body_position_rmse_per_body[body_index]
            metrics[f"pre_relaxation_body_orientation_rmse_{body_name}_deg"] = torch.rad2deg(pre_body_orientation_rmse_rad_per_body[body_index])

        for hand_index, hand_name in enumerate(H1_HAND_REFERENCE_NAMES):

            metrics[f"pre_relaxation_hand_position_rmse_{hand_name}_m"] = pre_hand_position_rmse_per_hand[hand_index]

    return metrics
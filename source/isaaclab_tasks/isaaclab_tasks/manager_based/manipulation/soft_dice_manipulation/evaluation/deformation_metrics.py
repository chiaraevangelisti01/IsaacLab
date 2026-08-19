from __future__ import annotations

import torch


def compute_deformation_metrics(
    deformation_rms_m: torch.Tensor,
    deformation_p95_m: torch.Tensor,
    deformation_max_m: torch.Tensor,
    relative_extent_change: torch.Tensor,
) -> dict[str, torch.Tensor]:
    """Compute episode-level deformable-object shape metrics."""

    if deformation_rms_m.shape[0] == 0:
        raise ValueError(
            "Cannot compute deformation metrics from an empty episode."
        )

    metrics = {
        "deformation_rms_mean_m": torch.mean(deformation_rms_m),
        "deformation_rms_peak_m": torch.max(deformation_rms_m),
        "deformation_p95_mean_m": torch.mean(deformation_p95_m),
        "deformation_p95_peak_m": torch.max(deformation_p95_m),
        "deformation_max_peak_m": torch.max(deformation_max_m),
    }

    axis_names = ("x", "y", "z")

    for axis, axis_name in enumerate(axis_names):
        axis_extent = relative_extent_change[:, axis]

        metrics[f"relative_extent_change_{axis_name}_min"] = torch.min(
            axis_extent
        )
        metrics[f"relative_extent_change_{axis_name}_max"] = torch.max(
            axis_extent
        )

    return metrics
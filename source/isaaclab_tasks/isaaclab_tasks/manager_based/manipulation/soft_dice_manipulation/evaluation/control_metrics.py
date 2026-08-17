from __future__ import annotations

import torch


def _rms(value: torch.Tensor, dim) -> torch.Tensor:
    return torch.sqrt(torch.mean(torch.square(value), dim=dim))


def compute_control_quality_metrics(
    joint_torque_nm: torch.Tensor,
    action_delta: torch.Tensor,
    joint_names: list[str],
) -> dict[str, torch.Tensor]:
    """Compute per-joint torque usage and action smoothness metrics."""

    torque_rms = _rms(joint_torque_nm, dim=0)
    torque_peak = torch.max(torch.abs(joint_torque_nm), dim=0).values

    action_delta_rms = _rms(action_delta, dim=0)
    action_delta_peak = torch.max(torch.abs(action_delta), dim=0).values

    metrics = {}

    for i, joint_name in enumerate(joint_names):
        metrics[f"torque_rms_{joint_name}_nm"] = torque_rms[i]
        metrics[f"torque_peak_abs_{joint_name}_nm"] = torque_peak[i]

        metrics[f"action_delta_rms_{joint_name}"] = action_delta_rms[i]
        metrics[f"action_delta_peak_abs_{joint_name}"] = action_delta_peak[i]

    return metrics
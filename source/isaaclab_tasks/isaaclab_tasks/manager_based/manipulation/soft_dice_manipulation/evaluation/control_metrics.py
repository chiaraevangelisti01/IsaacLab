from __future__ import annotations

import torch


def _rms(value: torch.Tensor, dim) -> torch.Tensor:
    return torch.sqrt(torch.mean(torch.square(value), dim=dim))


def compute_control_quality_metrics(
    joint_torque_nm: torch.Tensor,
    action_delta: torch.Tensor,
    joint_names: list[str],
) -> dict[str, torch.Tensor]:
    """Compute episode-level control effort and per-joint RMS metrics."""

    torque_rms_overall = _rms(joint_torque_nm, dim=(0, 1))
    action_delta_rms_overall = _rms(action_delta, dim=(0, 1))

    torque_rms = _rms(joint_torque_nm, dim=0)
    action_delta_rms = _rms(action_delta, dim=0)

    metrics = {
        "torque_rms_overall_nm": torque_rms_overall,
        "action_delta_rms_overall": action_delta_rms_overall,
    }

    for i, joint_name in enumerate(joint_names):
        metrics[f"torque_rms_{joint_name}_nm"] = torque_rms[i]
        metrics[f"action_delta_rms_{joint_name}"] = action_delta_rms[i]

    return metrics
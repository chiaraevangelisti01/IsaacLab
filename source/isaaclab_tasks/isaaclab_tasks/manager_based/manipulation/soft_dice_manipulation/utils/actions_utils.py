from __future__ import annotations

import re
from numbers import Number

from isaaclab.assets import ArticulationCfg


def _resolve_actuator_parameter(
    value: float | dict[str, float],
    joint_name: str,
) -> float:
    """Resolve an actuator parameter for a specific joint."""

    if isinstance(value, Number):
        return float(value)

    matches = [
        parameter
        for pattern, parameter in value.items()
        if re.fullmatch(pattern, joint_name)
    ]

    if len(matches) != 1:
        raise ValueError(
            f"Expected exactly one actuator parameter match for "
            f"joint '{joint_name}', got {len(matches)}."
        )

    return float(matches[0])


def build_joint_action_scale(
    robot_cfg: ArticulationCfg,
    joint_names: list[str],
    scale_factor: float = 0.25,
) -> dict[str, float]:
    """Build BeyondMimic-style action scales.

    scale_j = scale_factor * effort_limit_j / stiffness_j
    """

    scales = {}

    for joint_name in joint_names:
        matching_actuators = [
            actuator_cfg
            for actuator_cfg in robot_cfg.actuators.values()
            if any(
                re.fullmatch(pattern, joint_name)
                for pattern in actuator_cfg.joint_names_expr
            )
        ]

        if len(matching_actuators) != 1:
            raise ValueError(
                f"Expected exactly one actuator for '{joint_name}', "
                f"got {len(matching_actuators)}."
            )

        actuator_cfg = matching_actuators[0]

        effort = _resolve_actuator_parameter(
            actuator_cfg.effort_limit_sim,
            joint_name,
        )
        stiffness = _resolve_actuator_parameter(
            actuator_cfg.stiffness,
            joint_name,
        )

        if stiffness <= 0.0:
            raise ValueError(
                f"Stiffness for '{joint_name}' must be positive."
            )

        scales[joint_name] = (
            scale_factor * effort / stiffness
        )

    return scales
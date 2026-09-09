from __future__ import annotations

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import wandb

from .robustness import ROBUSTNESS_CONDITIONS


# -----------------------------------------------------------------------------
# Plot configuration.
# -----------------------------------------------------------------------------

_ROBUSTNESS_PLOT_SPECS = {
    "initial_x": {
        "perturbation_scale": 100.0,
        "perturbation_label": "Initial X perturbation [cm]",
        "terminal_error_key": "final_xy_position_error_m",
        "terminal_error_scale": 100.0,
        "terminal_error_label": "Final XY error [cm]",
    },
    "initial_y": {
        "perturbation_scale": 100.0,
        "perturbation_label": "Initial Y perturbation [cm]",
        "terminal_error_key": "final_xy_position_error_m",
        "terminal_error_scale": 100.0,
        "terminal_error_label": "Final XY error [cm]",
    },
    "initial_yaw": {
        "perturbation_scale": 180.0 / np.pi,
        "perturbation_label": "Initial yaw perturbation [deg]",
        "terminal_error_key": "final_orientation_error_deg",
        "terminal_error_scale": 1.0,
        "terminal_error_label": "Final orientation error [deg]",
    },
    "youngs_modulus": {
        "perturbation_scale": 1.0e-3,
        "perturbation_label": "Young's modulus [kPa]",
        "terminal_error_key": "final_xy_position_error_m",
        "terminal_error_scale": 100.0,
        "terminal_error_label": "Final XY error [cm]",
    },
    "poissons_ratio": {
        "perturbation_scale": 1.0,
        "perturbation_label": "Poisson's ratio",
        "terminal_error_key": "final_xy_position_error_m",
        "terminal_error_scale": 100.0,
        "terminal_error_label": "Final XY error [cm]",
    },
    "dynamic_friction": {
        "perturbation_scale": 1.0,
        "perturbation_label": "Cube dynamic friction coefficient",
        "terminal_error_key": "final_xy_position_error_m",
        "terminal_error_scale": 100.0,
        "terminal_error_label": "Final XY error [cm]",
    },
}


def _records_for_condition(
    records: list[dict],
    condition: str,
) -> list[dict]:
    return [
        record
        for record in records
        if record["robustness_condition"] == condition
    ]


def _perturbation_values(
    records: list[dict],
    condition: str,
) -> np.ndarray:
    spec = _ROBUSTNESS_PLOT_SPECS[condition]

    values = np.asarray(
        [
            float(record["applied_perturbation"])
            for record in records
        ],
        dtype=np.float64,
    )

    return spec["perturbation_scale"] * values


# -----------------------------------------------------------------------------
# 1. Final cube XY scatter.
# -----------------------------------------------------------------------------

def _plot_final_xy_scatter(
    condition_records: list[dict],
    nominal_records: list[dict],
    condition: str,
):
    """Plot terminal cube placement for one robustness condition."""

    spec = _ROBUSTNESS_PLOT_SPECS[condition]

    x_cm = 100.0 * np.asarray(
        [
            record["final_x_error_m"]
            for record in condition_records
        ],
        dtype=np.float64,
    )

    y_cm = 100.0 * np.asarray(
        [
            record["final_y_error_m"]
            for record in condition_records
        ],
        dtype=np.float64,
    )

    perturbation = _perturbation_values(
        condition_records,
        condition,
    )

    nominal_x_cm = 100.0 * np.asarray(
        [
            record["final_x_error_m"]
            for record in nominal_records
        ],
        dtype=np.float64,
    )

    nominal_y_cm = 100.0 * np.asarray(
        [
            record["final_y_error_m"]
            for record in nominal_records
        ],
        dtype=np.float64,
    )

    fig, ax = plt.subplots(
        figsize=(7, 6)
    )

    # Nominal reference cloud.
    if nominal_records:
        ax.scatter(
            nominal_x_cm,
            nominal_y_cm,
            alpha=0.25,
            label="Nominal",
        )

    # Robustness-condition episodes.
    scatter = ax.scatter(
        x_cm,
        y_cm,
        c=perturbation,
        label=condition,
    )

    ax.axhline(
        0.0,
        linewidth=0.8,
        linestyle="--",
    )
    ax.axvline(
        0.0,
        linewidth=0.8,
        linestyle="--",
    )

    all_x = np.concatenate(
        (
            x_cm,
            nominal_x_cm,
        )
    )
    all_y = np.concatenate(
        (
            y_cm,
            nominal_y_cm,
        )
    )

    limit = max(
        np.max(np.abs(all_x)),
        np.max(np.abs(all_y)),
        0.1,
    ) * 1.15

    ax.set_xlim(
        -limit,
        limit,
    )
    ax.set_ylim(
        -limit,
        limit,
    )
    ax.set_aspect(
        "equal",
        adjustable="box",
    )

    ax.set_xlabel(
        "Final X error [cm]"
    )
    ax.set_ylabel(
        "Final Y error [cm]"
    )
    ax.set_title(
        f"Final cube placement — {condition}"
    )

    colorbar = fig.colorbar(
        scatter,
        ax=ax,
    )
    colorbar.set_label(
        spec["perturbation_label"]
    )

    ax.legend()

    fig.tight_layout()

    return fig


# -----------------------------------------------------------------------------
# 2. Applied perturbation vs terminal error.
# -----------------------------------------------------------------------------

def _plot_perturbation_vs_terminal_error(
    condition_records: list[dict],
    condition: str,
):
    """Plot terminal task error against the actual applied perturbation."""

    spec = _ROBUSTNESS_PLOT_SPECS[condition]

    perturbation = _perturbation_values(
        condition_records,
        condition,
    )

    terminal_error = (
        spec["terminal_error_scale"]
        * np.asarray(
            [
                record[
                    spec["terminal_error_key"]
                ]
                for record in condition_records
            ],
            dtype=np.float64,
        )
    )

    fig, ax = plt.subplots(
        figsize=(7, 5)
    )

    ax.scatter(
        perturbation,
        terminal_error,
    )

    ax.set_xlabel(
        spec["perturbation_label"]
    )
    ax.set_ylabel(
        spec["terminal_error_label"]
    )

    ax.set_title(
        "Applied perturbation vs terminal error"
        f" — {condition}"
    )

    fig.tight_layout()

    return fig


# -----------------------------------------------------------------------------
# 3. Applied perturbation vs deformation.
# -----------------------------------------------------------------------------

def _plot_perturbation_vs_deformation(
    condition_records: list[dict],
    condition: str,
):
    """Plot episode deformation against the actual perturbation."""

    spec = _ROBUSTNESS_PLOT_SPECS[condition]

    perturbation = _perturbation_values(
        condition_records,
        condition,
    )

    rms_mm = 1000.0 * np.asarray(
        [
            record[
                "deformation_rms_mean_m"
            ]
            for record in condition_records
        ],
        dtype=np.float64,
    )

    p95_mm = 1000.0 * np.asarray(
        [
            record[
                "deformation_p95_mean_m"
            ]
            for record in condition_records
        ],
        dtype=np.float64,
    )

    fig, ax = plt.subplots(
        figsize=(7, 5)
    )

    ax.scatter(
        perturbation,
        rms_mm,
        label="Mean RMS",
    )

    ax.scatter(
        perturbation,
        p95_mm,
        label="Mean P95",
    )

    ax.set_xlabel(
        spec["perturbation_label"]
    )
    ax.set_ylabel(
        "Deformation [mm]"
    )

    ax.set_title(
        "Applied perturbation vs deformation"
        f" — {condition}"
    )

    ax.legend()

    fig.tight_layout()

    return fig

def _plot_dynamic_friction_sensitivity(records: list[dict]):
    """Plot task performance across the discrete friction levels."""

    friction = np.asarray([r["cube_dynamic_friction"] for r in records], dtype=np.float64)
    levels = np.unique(friction[np.isfinite(friction)])

    if len(levels) == 0:
        return None

    landing_aware = "task_success" in records[0]

    if landing_aware:
        task_success = []
        position_success = []
        orientation_success = []
        landing_error_cm = []

        for level in levels:
            group = [r for r in records if np.isclose(r["cube_dynamic_friction"], level)]

            task_success.append(100.0 * np.mean([r["task_success"] for r in group]))
            position_success.append(100.0 * np.mean([r["final_landing_position_success"] for r in group]))
            orientation_success.append(100.0 * np.mean([r["final_landing_orientation_success"] for r in group]))
            landing_error_cm.append(100.0 * np.mean([r["final_landing_xy_error_m"] for r in group]))

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))

        axes[0].plot(levels, task_success, marker="o", label="Task success")
        axes[0].plot(levels, position_success, marker="o", label="Position success")
        axes[0].plot(levels, orientation_success, marker="o", label="Orientation success")
        axes[0].set_xlabel("Cube dynamic friction coefficient")
        axes[0].set_ylabel("Success rate [%]")
        axes[0].set_ylim(0.0, 100.0)
        axes[0].set_title("Success vs dynamic friction")
        axes[0].legend()

        axes[1].plot(levels, landing_error_cm, marker="o")
        axes[1].set_xlabel("Cube dynamic friction coefficient")
        axes[1].set_ylabel("Landing XY error [cm]")
        axes[1].set_title("Landing error vs dynamic friction")

    else:
        final_xy_error_cm = []

        for level in levels:
            group = [r for r in records if np.isclose(r["cube_dynamic_friction"], level)]
            final_xy_error_cm.append(100.0 * np.mean([r["final_xy_position_error_m"] for r in group]))

        fig, ax = plt.subplots(figsize=(7, 5))
        ax.plot(levels, final_xy_error_cm, marker="o")
        ax.set_xlabel("Cube dynamic friction coefficient")
        ax.set_ylabel("Final XY error [cm]")
        ax.set_title("Final XY error vs dynamic friction")

    fig.tight_layout()
    return fig

def _plot_dynamic_friction_control(records: list[dict]):
    """Plot deformation and control effort across friction levels."""

    friction = np.asarray([r["cube_dynamic_friction"] for r in records], dtype=np.float64)
    levels = np.unique(friction[np.isfinite(friction)])

    if len(levels) == 0:
        return None

    deformation_mm = []
    torque_nm = []
    action_delta = []

    for level in levels:
        group = [r for r in records if np.isclose(r["cube_dynamic_friction"], level)]

        deformation_mm.append(1000.0 * np.mean([r["deformation_rms_mean_m"] for r in group]))
        torque_nm.append(np.mean([r["torque_rms_overall_nm"] for r in group]))
        action_delta.append(np.mean([r["action_delta_rms_overall"] for r in group]))

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    axes[0].plot(levels, deformation_mm, marker="o")
    axes[0].set_xlabel("Dynamic friction")
    axes[0].set_ylabel("Mean RMS deformation [mm]")
    axes[0].set_title("Deformation")

    axes[1].plot(levels, torque_nm, marker="o")
    axes[1].set_xlabel("Dynamic friction")
    axes[1].set_ylabel("Torque RMS [Nm]")
    axes[1].set_title("Control effort")

    axes[2].plot(levels, action_delta, marker="o")
    axes[2].set_xlabel("Dynamic friction")
    axes[2].set_ylabel("Action-delta RMS")
    axes[2].set_title("Action smoothness")

    fig.suptitle("Control sensitivity to cube dynamic friction")
    fig.tight_layout()
    return fig

# -----------------------------------------------------------------------------
# Public logging entry point.
# -----------------------------------------------------------------------------

def log_robustness_visualizations(
    run,
    records: list[dict],
) -> None:
    """Log the three robustness plots for every perturbation condition."""

    if not records:
        return

    nominal_records = _records_for_condition(
        records,
        "nominal",
    )

    for condition in ROBUSTNESS_CONDITIONS:
        if condition == "nominal":
            continue

        condition_records = (
            _records_for_condition(
                records,
                condition,
            )
        )

        if not condition_records:
            continue

        final_xy_fig = (
            _plot_final_xy_scatter(
                condition_records=condition_records,
                nominal_records=nominal_records,
                condition=condition,
            )
        )

        terminal_error_fig = (
            _plot_perturbation_vs_terminal_error(
                condition_records=condition_records,
                condition=condition,
            )
        )

        deformation_fig = (
            _plot_perturbation_vs_deformation(
                condition_records=condition_records,
                condition=condition,
            )
        )

        friction_sensitivity_fig = None
        friction_control_fig = None

        if condition == "dynamic_friction":
            friction_sensitivity_fig = _plot_dynamic_friction_sensitivity(condition_records)
            friction_control_fig = _plot_dynamic_friction_control(condition_records)

        prefix = (
            f"robustness/{condition}"
        )

        run.log(
            {
                f"{prefix}/final_xy": (
                    wandb.Image(
                        final_xy_fig
                    )
                ),
                (
                    f"{prefix}/"
                    "perturbation_vs_final_error"
                ): wandb.Image(
                    terminal_error_fig
                ),
                (
                    f"{prefix}/"
                    "perturbation_vs_deformation"
                ): wandb.Image(
                    deformation_fig
                ),
            }
        )
        if friction_sensitivity_fig is not None:
            run.log({
                f"{prefix}/sensitivity": wandb.Image(friction_sensitivity_fig),
                f"{prefix}/control_sensitivity": wandb.Image(friction_control_fig),
            })

        plt.close(
            final_xy_fig
        )
        plt.close(
            terminal_error_fig
        )
        plt.close(
            deformation_fig
        )
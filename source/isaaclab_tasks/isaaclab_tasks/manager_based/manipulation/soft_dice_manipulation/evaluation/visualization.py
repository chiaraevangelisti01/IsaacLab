from __future__ import annotations

from collections import Counter, defaultdict

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import wandb

from ..utils.motion_utils import H1_HAND_REFERENCE_NAMES, H1_TRACKED_BODY_NAMES


# -----------------------------------------------------------------------------
# Metrics shown in the compact W&B tables.
# -----------------------------------------------------------------------------

_DISPLAY_METRICS = [
    ("Final XY position error", "final_xy_position_error_m", 100.0, "cm"),
    ("Final orientation error", "final_orientation_error_deg", 1.0, "deg"),
    ("Cube trajectory XY RMSE", "cube_xy_position_rmse_m", 100.0, "cm"),
    ("Cube trajectory orientation RMSE", "cube_orientation_rmse_deg", 1.0, "deg"),
    ("Body position RMSE", "body_position_rmse_m", 100.0, "cm"),
    ("Body orientation RMSE", "body_orientation_rmse_deg", 1.0, "deg"),
    ("Hand position RMSE", "hand_position_rmse_m", 100.0, "cm"),
]


def _pretty_name(name: str) -> str:
    """Convert internal Isaac names to readable labels."""
    return name.replace("_link", "").replace("_", " ").title()


def _aggregate_by_frame(
    trajectory_records: list[dict],
    key: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Aggregate a trajectory quantity across episodes by motion frame."""

    values_by_frame: dict[int, list[np.ndarray]] = defaultdict(list)

    for episode in trajectory_records:
        frames = np.asarray(episode["motion_frame"], dtype=np.int64)
        values = np.asarray(episode[key], dtype=np.float64)

        if frames.shape[0] != values.shape[0]:
            raise ValueError(
                f"Frame/value length mismatch for {key}: "
                f"{frames.shape[0]} vs {values.shape[0]}."
            )

        for frame, value in zip(frames, values):
            if frame >= 0:
                values_by_frame[int(frame)].append(
                    np.asarray(value, dtype=np.float64)
                )

    if not values_by_frame:
        raise ValueError(f"No trajectory data available for {key}.")

    frames = np.asarray(sorted(values_by_frame.keys()), dtype=np.int64)
    means = []
    stds = []

    for frame in frames:
        stacked = np.stack(values_by_frame[int(frame)], axis=0)
        means.append(np.mean(stacked, axis=0))
        stds.append(np.std(stacked, axis=0))

    return frames, np.stack(means, axis=0), np.stack(stds, axis=0)


# -----------------------------------------------------------------------------
# Summary tables.
# -----------------------------------------------------------------------------

def compute_main_metric_summary(records: list[dict]) -> dict[str, float]:
    """Compute aggregate values for important evaluation metrics."""

    summary = {}

    for _, key, _, _ in _DISPLAY_METRICS:
        values = np.asarray([float(record[key]) for record in records], dtype=np.float64)
        summary[f"{key}_mean"] = float(np.mean(values))
        summary[f"{key}_std"] = float(np.std(values))

    if "final_top_face_correct" in records[0]:
        summary["top_face_success_rate"] = float(
            np.mean([record["final_top_face_correct"] for record in records])
        )

    return summary


def _make_episode_table(records: list[dict]) -> wandb.Table:
    """Compact one-row-per-episode table."""

    columns = [
        "episode",
        "termination",
        "duration_s",
        "top_face_correct",
        "final_xy_error_cm",
        "final_orientation_error_deg",
        "cube_xy_rmse_cm",
        "cube_orientation_rmse_deg",
        "body_position_rmse_cm",
        "body_orientation_rmse_deg",
        "hand_position_rmse_cm",
    ]

    data = [
        [
            record["episode_id"],
            record["termination"],
            record["duration_s"],
            100.0 * record["final_xy_position_error_m"],
            record["final_orientation_error_deg"],
            100.0 * record["cube_xy_position_rmse_m"],
            record["cube_orientation_rmse_deg"],
            100.0 * record["body_position_rmse_m"],
            record["body_orientation_rmse_deg"],
            100.0 * record["hand_position_rmse_m"],
            bool(record["final_top_face_correct"]),
        ]
        for record in records
    ]

    return wandb.Table(columns=columns, data=data)


def _make_aggregate_table(records: list[dict]) -> wandb.Table:
    """Mean/std/median/max table for primary metrics."""

    columns = ["metric", "mean", "std", "median", "max", "unit"]
    data = []

    for display_name, key, scale, unit in _DISPLAY_METRICS:
        values = np.asarray(
            [float(record[key]) for record in records],
            dtype=np.float64,
        ) * scale

        data.append(
            [
                display_name,
                float(np.mean(values)),
                float(np.std(values)),
                float(np.median(values)),
                float(np.max(values)),
                unit,
            ]
        )

    return wandb.Table(columns=columns, data=data)


def _make_termination_chart(records: list[dict]):
    counts = Counter(record["termination"] for record in records)

    table = wandb.Table(
        columns=["termination", "count"],
        data=[
            [termination, count]
            for termination, count in sorted(counts.items())
        ],
    )

    return wandb.plot.bar(
        table,
        "termination",
        "count",
        title="Episode termination types",
    )


# -----------------------------------------------------------------------------
# Cube trajectory.
# -----------------------------------------------------------------------------

def _plot_cube_tracking(trajectory_records: list[dict]):
    frames, position_mean, position_std = _aggregate_by_frame(
        trajectory_records,
        "cube_position_delta_m",
    )

    orientation_frames, orientation_mean, orientation_std = _aggregate_by_frame(
        trajectory_records,
        "cube_orientation_error_rad",
    )

    position_mean_cm = 100.0 * position_mean
    position_std_cm = 100.0 * position_std
    orientation_mean_deg = np.degrees(orientation_mean)
    orientation_std_deg = np.degrees(orientation_std)

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # X error.
    line = axes[0].plot(
        frames,
        position_mean_cm[:, 0],
        label="Mean",
    )[0]

    axes[0].fill_between(
        frames,
        position_mean_cm[:, 0] - position_std_cm[:, 0],
        position_mean_cm[:, 0] + position_std_cm[:, 0],
        alpha=0.2,
        color=line.get_color(),
    )

    axes[0].axhline(0.0, linewidth=0.8, linestyle="--")
    axes[0].set_ylabel("X error [cm]")
    axes[0].set_title("Cube trajectory tracking")

    # Y error.
    line = axes[1].plot(
        frames,
        position_mean_cm[:, 1],
        label="Mean",
    )[0]

    axes[1].fill_between(
        frames,
        position_mean_cm[:, 1] - position_std_cm[:, 1],
        position_mean_cm[:, 1] + position_std_cm[:, 1],
        alpha=0.2,
        color=line.get_color(),
    )

    axes[1].axhline(0.0, linewidth=0.8, linestyle="--")
    axes[1].set_ylabel("Y error [cm]")

    # Orientation error.
    line = axes[2].plot(
        orientation_frames,
        orientation_mean_deg,
        label="Mean",
    )[0]

    axes[2].fill_between(
        orientation_frames,
        orientation_mean_deg - orientation_std_deg,
        orientation_mean_deg + orientation_std_deg,
        alpha=0.2,
        color=line.get_color(),
    )

    axes[2].set_ylabel("Orientation error [deg]")
    axes[2].set_xlabel("Motion frame")

    fig.tight_layout()
    return fig


# -----------------------------------------------------------------------------
# Robot body tracking.
# -----------------------------------------------------------------------------

def _plot_body_tracking(trajectory_records: list[dict]):
    frames, position_mean, _ = _aggregate_by_frame(
        trajectory_records,
        "body_position_error_m",
    )

    orientation_frames, orientation_mean, _ = _aggregate_by_frame(
        trajectory_records,
        "body_orientation_error_rad",
    )

    fig, axes = plt.subplots(2, 1, figsize=(11, 9), sharex=True)

    for body_index, body_name in enumerate(H1_TRACKED_BODY_NAMES):
        label = _pretty_name(body_name)

        axes[0].plot(
            frames,
            100.0 * position_mean[:, body_index],
            label=label,
        )

        axes[1].plot(
            orientation_frames,
            np.degrees(orientation_mean[:, body_index]),
            label=label,
        )

    axes[0].set_title("Tracked-body Cartesian trajectory error")
    axes[0].set_ylabel("Position error [cm]")
    axes[0].legend(ncol=2, fontsize=8)

    axes[1].set_ylabel("Orientation error [deg]")
    axes[1].set_xlabel("Motion frame")
    axes[1].legend(ncol=2, fontsize=8)

    fig.tight_layout()
    return fig


# -----------------------------------------------------------------------------
# Hand tracking.
# -----------------------------------------------------------------------------

def _plot_hand_tracking(trajectory_records: list[dict]):
    frames, hand_mean, hand_std = _aggregate_by_frame(
        trajectory_records,
        "hand_position_error_m",
    )

    fig, ax = plt.subplots(figsize=(10, 5))

    for hand_index, hand_name in enumerate(H1_HAND_REFERENCE_NAMES):
        mean_cm = 100.0 * hand_mean[:, hand_index]
        std_cm = 100.0 * hand_std[:, hand_index]

        line = ax.plot(
            frames,
            mean_cm,
            label=_pretty_name(hand_name),
        )[0]

        ax.fill_between(
            frames,
            mean_cm - std_cm,
            mean_cm + std_cm,
            alpha=0.15,
            color=line.get_color(),
        )

    ax.set_title("Hand Cartesian trajectory tracking")
    ax.set_xlabel("Motion frame")
    ax.set_ylabel("Position error [cm]")
    ax.legend()

    fig.tight_layout()
    return fig


# -----------------------------------------------------------------------------
# Aggregate body/hand errors.
# -----------------------------------------------------------------------------

def _plot_per_body_rmse(records: list[dict]):
    position_labels = []
    position_values = []

    for body_name in H1_TRACKED_BODY_NAMES:
        key = f"body_position_rmse_{body_name}_m"

        position_labels.append(_pretty_name(body_name))
        position_values.append(
            100.0 * np.mean([record[key] for record in records])
        )

    for hand_name in H1_HAND_REFERENCE_NAMES:
        key = f"hand_position_rmse_{hand_name}_m"

        position_labels.append(_pretty_name(hand_name))
        position_values.append(
            100.0 * np.mean([record[key] for record in records])
        )

    orientation_labels = []
    orientation_values = []

    for body_name in H1_TRACKED_BODY_NAMES:
        key = f"body_orientation_rmse_{body_name}_deg"

        orientation_labels.append(_pretty_name(body_name))
        orientation_values.append(
            np.mean([record[key] for record in records])
        )

    fig, axes = plt.subplots(1, 2, figsize=(13, 6))

    axes[0].barh(position_labels, position_values)
    axes[0].set_title("Position tracking RMSE")
    axes[0].set_xlabel("RMSE [cm]")

    axes[1].barh(orientation_labels, orientation_values)
    axes[1].set_title("Orientation tracking RMSE")
    axes[1].set_xlabel("RMSE [deg]")

    fig.tight_layout()
    return fig

#-------------------------------------------------------------------
# Action smoothness
#---------------------------------------------------------------------

def _plot_joint_torques(
    trajectory_records: list[dict],
    joint_names: list[str],
):
    frames, torque_mean, torque_std = _aggregate_by_frame(
        trajectory_records,
        "joint_torque_nm",
    )

    fig, axes = plt.subplots(3, 3, figsize=(14, 10), sharex=True)
    axes = axes.flatten()

    for i, (ax, joint_name) in enumerate(zip(axes, joint_names)):
        mean = torque_mean[:, i]
        std = torque_std[:, i]

        line = ax.plot(frames, mean)[0]
        ax.fill_between(
            frames,
            mean - std,
            mean + std,
            alpha=0.2,
            color=line.get_color(),
        )

        ax.axhline(0.0, linewidth=0.8, linestyle="--")
        ax.set_title(_pretty_name(joint_name))
        ax.set_ylabel("Torque [Nm]")
        ax.set_xlabel("Motion frame")

    fig.suptitle("Approximate applied joint torque")
    fig.tight_layout()

    return fig

def _plot_joint_action_smoothness(
    trajectory_records: list[dict],
    joint_names: list[str],
):
    frames, action_mean, action_std = _aggregate_by_frame(
        trajectory_records,
        "action_delta",
    )

    fig, axes = plt.subplots(3, 3, figsize=(14, 10), sharex=True)
    axes = axes.flatten()

    for i, (ax, joint_name) in enumerate(zip(axes, joint_names)):
        mean = action_mean[:, i]
        std = action_std[:, i]

        line = ax.plot(frames, mean)[0]
        ax.fill_between(
            frames,
            mean - std,
            mean + std,
            alpha=0.2,
            color=line.get_color(),
        )

        ax.axhline(0.0, linewidth=0.8, linestyle="--")
        ax.set_title(_pretty_name(joint_name))
        ax.set_ylabel("Action Δ")
        ax.set_xlabel("Motion frame")

    fig.suptitle("Per-joint action change")
    fig.tight_layout()

    return fig

def _plot_per_joint_control_metrics(
    records: list[dict],
    joint_names: list[str],
):
    torque_values = []
    action_values = []

    for joint_name in joint_names:
        torque_values.append(
            np.mean([
                r[f"torque_rms_{joint_name}_nm"]
                for r in records
            ])
        )

        action_values.append(
            np.mean([
                r[f"action_delta_rms_{joint_name}"]
                for r in records
            ])
        )

    labels = [_pretty_name(name) for name in joint_names]
    y = np.arange(len(labels))

    fig, axes = plt.subplots(1, 2, figsize=(13, 6))

    axes[0].barh(y, torque_values)
    axes[0].set_yticks(y, labels=labels)
    axes[0].invert_yaxis()
    axes[0].set_xlabel("Torque RMS [Nm]")
    axes[0].set_title("Per-joint torque")

    axes[1].barh(y, action_values)
    axes[1].set_yticks(y, labels=labels)
    axes[1].invert_yaxis()
    axes[1].set_xlabel("Action Δ RMS")
    axes[1].set_title("Per-joint action smoothness")

    fig.tight_layout()
    return fig
# -----------------------------------------------------------------------------
# Final pose distribution.
# -----------------------------------------------------------------------------

def _plot_final_pose_components(records: list[dict]):
    components = [
        (
            "X position error",
            100.0 * np.asarray([r["final_x_error_m"] for r in records]),
            "X error [cm]",
        ),
        (
            "Y position error",
            100.0 * np.asarray([r["final_y_error_m"] for r in records]),
            "Y error [cm]",
        ),
        (
            "Roll error",
            np.asarray([r["final_roll_error_deg"] for r in records]),
            "Roll error [deg]",
        ),
        (
            "Pitch error",
            np.asarray([r["final_pitch_error_deg"] for r in records]),
            "Pitch error [deg]",
        ),
        (
            "Yaw error",
            np.asarray([r["final_yaw_error_deg"] for r in records]),
            "Yaw error [deg]",
        ),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    axes = axes.flatten()

    for ax, (title, values, ylabel) in zip(axes, components):
        offsets = np.linspace(0.96, 1.04, len(values))

        ax.boxplot([values], widths=0.3)
        ax.scatter(offsets, values)
        ax.axhline(0.0, linewidth=0.8, linestyle="--")

        ax.set_xticks([])
        ax.set_title(title)
        ax.set_ylabel(ylabel)

    axes[-1].axis("off")

    fig.suptitle("Final cube pose error components")
    fig.tight_layout()

    return fig

def _plot_final_xy_scatter(records: list[dict]):
    x_cm = 100.0 * np.asarray([r["final_x_error_m"] for r in records])
    y_cm = 100.0 * np.asarray([r["final_y_error_m"] for r in records])

    fig, ax = plt.subplots(figsize=(6, 6))

    ax.scatter(x_cm, y_cm)
    ax.axhline(0.0, linewidth=0.8, linestyle="--")
    ax.axvline(0.0, linewidth=0.8, linestyle="--")

    limit = max(
        np.max(np.abs(x_cm)),
        np.max(np.abs(y_cm)),
        0.1,
    ) * 1.15

    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_aspect("equal", adjustable="box")

    ax.set_xlabel("Final X error [cm]")
    ax.set_ylabel("Final Y error [cm]")
    ax.set_title("Final cube placement")

    fig.tight_layout()
    return fig

# -----------------------------------------------------------------------------
# Combined diagnostic.
# -----------------------------------------------------------------------------

def _plot_hand_cube_torque_diagnostic(
    trajectory_records: list[dict],
    joint_names: list[str],
):
    hand_frames, hand_mean, hand_std = _aggregate_by_frame(
        trajectory_records, "hand_position_error_m"
    )
    cube_frames, cube_delta_mean, cube_delta_std = _aggregate_by_frame(
        trajectory_records, "cube_position_delta_m"
    )
    ori_frames, ori_mean, ori_std = _aggregate_by_frame(
        trajectory_records, "cube_orientation_error_rad"
    )
    torque_frames, torque_mean, torque_std = _aggregate_by_frame(
        trajectory_records, "joint_torque_nm"
    )

    hand_mean_cm = 100.0 * np.mean(hand_mean, axis=-1)
    hand_std_cm = 100.0 * np.mean(hand_std, axis=-1)

    cube_delta_mean_cm = 100.0 * cube_delta_mean
    cube_delta_std_cm = 100.0 * cube_delta_std

    ori_mean_deg = np.degrees(ori_mean)
    ori_std_deg = np.degrees(ori_std)

    fig = plt.figure(figsize=(15, 18))
    grid = fig.add_gridspec(6, 3)

    # ------------------------------------------------------------------
    # Hand tracking.
    # ------------------------------------------------------------------

    ax = fig.add_subplot(grid[0, :])

    line = ax.plot(hand_frames, hand_mean_cm)[0]
    ax.fill_between(
        hand_frames,
        hand_mean_cm - hand_std_cm,
        hand_mean_cm + hand_std_cm,
        alpha=0.2,
        color=line.get_color(),
    )

    ax.set_ylabel("Hand error [cm]")
    ax.set_title("Hand / cube / torque diagnostic")

    # ------------------------------------------------------------------
    # Cube X/Y tracking.
    # ------------------------------------------------------------------

    ax = fig.add_subplot(grid[1, :])

    for i, label in enumerate(("X", "Y")):
        line = ax.plot(cube_frames, cube_delta_mean_cm[:, i], label=label)[0]
        ax.fill_between(
            cube_frames,
            cube_delta_mean_cm[:, i] - cube_delta_std_cm[:, i],
            cube_delta_mean_cm[:, i] + cube_delta_std_cm[:, i],
            alpha=0.15,
            color=line.get_color(),
        )

    ax.axhline(0.0, linewidth=0.8, linestyle="--")
    ax.set_ylabel("Cube error [cm]")
    ax.legend()

    # ------------------------------------------------------------------
    # Cube orientation tracking.
    # ------------------------------------------------------------------

    ax = fig.add_subplot(grid[2, :])

    line = ax.plot(ori_frames, ori_mean_deg)[0]
    ax.fill_between(
        ori_frames,
        ori_mean_deg - ori_std_deg,
        ori_mean_deg + ori_std_deg,
        alpha=0.2,
        color=line.get_color(),
    )

    ax.set_ylabel("Cube orientation error [deg]")

    # ------------------------------------------------------------------
    # Per-joint torque.
    # ------------------------------------------------------------------


    for i, joint_name in enumerate(joint_names):
        row = 3 + i // 3
        col = i % 3

        ax = fig.add_subplot(grid[row, col])

        mean = torque_mean[:, i]
        std = torque_std[:, i]

        line = ax.plot(torque_frames, mean)[0]
        ax.fill_between(
            torque_frames,
            mean - std,
            mean + std,
            alpha=0.2,
            color=line.get_color(),
        )

        ax.axhline(0.0, linewidth=0.8, linestyle="--")
        ax.set_title(_pretty_name(joint_name))
        ax.set_ylabel("Torque [Nm]")
        ax.set_xlabel("Motion frame")

    fig.tight_layout()
    return fig


# -----------------------------------------------------------------------------
# Public W&B logging entry point.
# -----------------------------------------------------------------------------

def log_evaluation_visualizations(
    run,
    records: list[dict],
    trajectory_records: list[dict],
    joint_names: list[str],
) -> None:
    """Log compact evaluation tables and diagnostic plots to W&B."""

    if not records:
        return

    if not trajectory_records:
        raise ValueError(
            "Trajectory records are required for evaluation visualization."
        )

    episode_table = _make_episode_table(records)
    aggregate_table = _make_aggregate_table(records)
    termination_chart = _make_termination_chart(records)

    cube_tracking_fig = _plot_cube_tracking(trajectory_records)
    final_pose_fig = _plot_final_pose_components(records)
    final_xy_fig = _plot_final_xy_scatter(records)

    body_tracking_fig = _plot_body_tracking(trajectory_records)
    hand_tracking_fig = _plot_hand_tracking(trajectory_records)
    body_rmse_fig = _plot_per_body_rmse(records)

    joint_torque_fig = _plot_joint_torques(
        trajectory_records,
        joint_names,
    )
    action_smoothness_fig = _plot_joint_action_smoothness(
        trajectory_records,
        joint_names,
    )
    control_summary_fig = _plot_per_joint_control_metrics(
        records,
        joint_names,
    )

    diagnostic_fig = _plot_hand_cube_torque_diagnostic(
        trajectory_records,
        joint_names,
    )

    run.log(
        {
            # Summary.
            "evaluation_summary/episodes": episode_table,
            "evaluation_summary/aggregate": aggregate_table,
            "evaluation_summary/terminations": termination_chart,

            # Cube/task.
            "evaluation_cube/trajectory_tracking": wandb.Image(cube_tracking_fig),
            "evaluation_cube/final_pose_components": wandb.Image(final_pose_fig),
            "evaluation_cube/final_xy_scatter": wandb.Image(final_xy_fig),

            # Robot Cartesian tracking.
            "evaluation_robot/body_tracking": wandb.Image(body_tracking_fig),
            "evaluation_robot/hand_tracking": wandb.Image(hand_tracking_fig),
            "evaluation_robot/per_body_rmse": wandb.Image(body_rmse_fig),

            # Control.
            "evaluation_control/joint_torques": wandb.Image(joint_torque_fig),
            "evaluation_control/action_smoothness": wandb.Image(
                action_smoothness_fig
            ),
            "evaluation_control/per_joint_summary": wandb.Image(
                control_summary_fig
            ),

            # Combined diagnostic.
            "evaluation_diagnostics/hand_cube_torque": wandb.Image(
                diagnostic_fig
            ),
        }
    )

    figures = [
        cube_tracking_fig,
        final_pose_fig,
        final_xy_fig,
        body_tracking_fig,
        hand_tracking_fig,
        body_rmse_fig,
        joint_torque_fig,
        action_smoothness_fig,
        control_summary_fig,
        diagnostic_fig,
    ]

    for fig in figures:
        plt.close(fig)

#-----------------------------
#Wandb native
#------------------------------
def log_comparison_timeseries(
    run,
    trajectory_records: list[dict],
    joint_names: list[str],
) -> None:
    """Log a small set of native W&B curves for cross-run comparison."""

    frames, cube_delta, _ = _aggregate_by_frame(
        trajectory_records, "cube_position_delta_m"
    )
    _, cube_orientation, _ = _aggregate_by_frame(
        trajectory_records, "cube_orientation_error_rad"
    )
    _, hand_error, _ = _aggregate_by_frame(
        trajectory_records, "hand_position_error_m"
    )
    _, body_position, _ = _aggregate_by_frame(
        trajectory_records, "body_position_error_m"
    )
    _, body_orientation, _ = _aggregate_by_frame(
        trajectory_records, "body_orientation_error_rad"
    )
    _, torque_mean, _ = _aggregate_by_frame(
        trajectory_records, "joint_torque_nm"
    )
    _, action_delta_mean, _ = _aggregate_by_frame(
        trajectory_records, "action_delta"
    )

    run.define_metric("evaluation_frame")
    run.define_metric(
        "evaluation_comparison/*",
        step_metric="evaluation_frame",
    )
    run.define_metric(
        "evaluation_torque/*",
        step_metric="evaluation_frame",
    )
    run.define_metric(
        "evaluation_action_delta/*",
        step_metric="evaluation_frame",
    )

    for index, frame in enumerate(frames):
        row = {
            "evaluation_frame": int(frame),

            "evaluation_comparison/cube_x_error_cm":
                float(100.0 * cube_delta[index, 0]),

            "evaluation_comparison/cube_y_error_cm":
                float(100.0 * cube_delta[index, 1]),

            "evaluation_comparison/cube_orientation_error_deg":
                float(np.degrees(cube_orientation[index])),

            "evaluation_comparison/hand_position_error_cm":
                float(100.0 * np.mean(hand_error[index])),

            "evaluation_comparison/body_position_error_cm":
                float(100.0 * np.mean(body_position[index])),

            "evaluation_comparison/body_orientation_error_deg":
                float(np.degrees(np.mean(body_orientation[index]))),
        }

        for joint_idx, joint_name in enumerate(joint_names):
            row[f"evaluation_torque/{joint_name}_nm"] = float(
                torque_mean[index, joint_idx]
            )
            row[f"evaluation_action_delta/{joint_name}"] = float(
                action_delta_mean[index, joint_idx]
            )

        run.log(row)
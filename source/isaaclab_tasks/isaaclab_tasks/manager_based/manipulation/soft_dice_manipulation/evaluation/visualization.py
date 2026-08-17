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
    """Compute mean/std values for important evaluation metrics."""

    summary: dict[str, float] = {}

    for _, key, _, _ in _DISPLAY_METRICS:
        values = np.asarray(
            [float(record[key]) for record in records],
            dtype=np.float64,
        )

        summary[f"{key}_mean"] = float(np.mean(values))
        summary[f"{key}_std"] = float(np.std(values))

    return summary


def _make_episode_table(records: list[dict]) -> wandb.Table:
    """Compact one-row-per-episode table."""

    columns = [
        "episode",
        "termination",
        "duration_s",
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


# -----------------------------------------------------------------------------
# Final pose distribution.
# -----------------------------------------------------------------------------

def _plot_final_pose_distribution(records: list[dict]):
    xy_error_cm = np.asarray(
        [100.0 * record["final_xy_position_error_m"] for record in records],
        dtype=np.float64,
    )

    orientation_error_deg = np.asarray(
        [record["final_orientation_error_deg"] for record in records],
        dtype=np.float64,
    )

    fig, axes = plt.subplots(1, 2, figsize=(10, 5))

    # Deterministic offsets make overlapping episode points visible.
    xy_offsets = np.linspace(0.96, 1.04, len(xy_error_cm))
    ori_offsets = np.linspace(0.96, 1.04, len(orientation_error_deg))

    axes[0].boxplot([xy_error_cm], widths=0.3)
    axes[0].scatter(xy_offsets, xy_error_cm)
    axes[0].set_xticks([])
    axes[0].set_ylabel("Final XY error [cm]")
    axes[0].set_title("Final placement error")

    axes[1].boxplot([orientation_error_deg], widths=0.3)
    axes[1].scatter(ori_offsets, orientation_error_deg)
    axes[1].set_xticks([])
    axes[1].set_ylabel("Final orientation error [deg]")
    axes[1].set_title("Final orientation error")

    fig.tight_layout()
    return fig


# -----------------------------------------------------------------------------
# Combined diagnostic.
# -----------------------------------------------------------------------------

def _plot_hand_cube_diagnostic(trajectory_records: list[dict]):
    hand_frames, hand_mean, hand_std = _aggregate_by_frame(
        trajectory_records,
        "hand_position_error_m",
    )

    cube_frames, cube_orientation_mean, cube_orientation_std = _aggregate_by_frame(
        trajectory_records,
        "cube_orientation_error_rad",
    )

    mean_hand_error_cm = 100.0 * np.mean(hand_mean, axis=-1)
    mean_hand_std_cm = 100.0 * np.mean(hand_std, axis=-1)

    cube_orientation_mean_deg = np.degrees(cube_orientation_mean)
    cube_orientation_std_deg = np.degrees(cube_orientation_std)

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    line = axes[0].plot(
        hand_frames,
        mean_hand_error_cm,
    )[0]

    axes[0].fill_between(
        hand_frames,
        mean_hand_error_cm - mean_hand_std_cm,
        mean_hand_error_cm + mean_hand_std_cm,
        alpha=0.2,
        color=line.get_color(),
    )

    axes[0].set_ylabel("Mean hand error [cm]")
    axes[0].set_title("Hand tracking vs cube response")

    line = axes[1].plot(
        cube_frames,
        cube_orientation_mean_deg,
    )[0]

    axes[1].fill_between(
        cube_frames,
        cube_orientation_mean_deg - cube_orientation_std_deg,
        cube_orientation_mean_deg + cube_orientation_std_deg,
        alpha=0.2,
        color=line.get_color(),
    )

    axes[1].set_ylabel("Cube orientation error [deg]")
    axes[1].set_xlabel("Motion frame")

    fig.tight_layout()
    return fig


# -----------------------------------------------------------------------------
# Public W&B logging entry point.
# -----------------------------------------------------------------------------

def log_evaluation_visualizations(
    run,
    records: list[dict],
    trajectory_records: list[dict],
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
    body_tracking_fig = _plot_body_tracking(trajectory_records)
    hand_tracking_fig = _plot_hand_tracking(trajectory_records)
    body_rmse_fig = _plot_per_body_rmse(records)
    final_pose_fig = _plot_final_pose_distribution(records)
    diagnostic_fig = _plot_hand_cube_diagnostic(trajectory_records)

    run.log(
        {
            # Summary.
            "evaluation_summary/episodes": episode_table,
            "evaluation_summary/aggregate": aggregate_table,
            "evaluation_summary/terminations": termination_chart,

            # Cube.
            "evaluation_cube/trajectory_tracking": wandb.Image(cube_tracking_fig),
            "evaluation_cube/final_pose_distribution": wandb.Image(final_pose_fig),

            # Robot Cartesian tracking.
            "evaluation_robot/body_tracking": wandb.Image(body_tracking_fig),
            "evaluation_robot/hand_tracking": wandb.Image(hand_tracking_fig),
            "evaluation_robot/per_body_rmse": wandb.Image(body_rmse_fig),

            # Diagnostics.
            "evaluation_diagnostics/hand_cube_tracking": wandb.Image(
                diagnostic_fig
            ),
        }
    )

    plt.close(cube_tracking_fig)
    plt.close(body_tracking_fig)
    plt.close(hand_tracking_fig)
    plt.close(body_rmse_fig)
    plt.close(final_pose_fig)
    plt.close(diagnostic_fig)
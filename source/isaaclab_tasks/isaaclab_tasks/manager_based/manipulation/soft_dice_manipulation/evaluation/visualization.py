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
def _aggregate_metric_specs(records: list[dict]):
    specs = [
        ("Cube XY tracking RMSE", _tracking_metric_key(records, "cube_xy_position_rmse_m"), 100.0, "cm"),
        ("Cube orientation tracking RMSE", _tracking_metric_key(records, "cube_orientation_rmse_deg"), 1.0, "deg"),
        ("Body position tracking RMSE", _tracking_metric_key(records, "body_position_rmse_m"), 100.0, "cm"),
        ("Body orientation tracking RMSE", _tracking_metric_key(records, "body_orientation_rmse_deg"), 1.0, "deg"),
        ("Hand position tracking RMSE", _tracking_metric_key(records, "hand_position_rmse_m"), 100.0, "cm"),
        ("Mean RMS deformation", "deformation_rms_mean_m", 1000.0, "mm"),
        ("Peak RMS deformation", "deformation_rms_peak_m", 1000.0, "mm"),
        ("High-deformation frames", "deformation_high_fraction", 100.0, "%"),
        ("Torque RMS", "torque_rms_overall_nm", 1.0, "Nm"),
        ("Action-delta RMS", "action_delta_rms_overall", 1.0, ""),
    ]

    if "task_success" in records[0]:
        specs = [
            ("Task success", "task_success", 100.0, "%"),
            ("Landing success", "final_landing_success", 100.0, "%"),
            ("Landing position success", "final_landing_position_success", 100.0, "%"),
            ("Landing orientation success", "final_landing_orientation_success", 100.0, "%"),
            ("Top-face success", "final_top_face_success", 100.0, "%"),
            ("Landing XY error", "final_landing_xy_error_m", 100.0, "cm"),
            ("Landing orientation error", "final_landing_orientation_error_deg", 1.0, "deg"),
            ("Position improvement", "landing_position_improvement_m", 100.0, "cm"),
            ("Orientation improvement", "landing_orientation_improvement_deg", 1.0, "deg"),
        ] + specs
    else:
        specs = [
            ("Top-face correct", "final_top_face_correct", 100.0, "%"),
            ("Final XY reference error", "final_xy_position_error_m", 100.0, "cm"),
            ("Final orientation reference error", "final_orientation_error_deg", 1.0, "deg"),
        ] + specs

    return specs

def _pretty_name(name: str) -> str:
    """Convert internal Isaac names to readable labels."""
    return name.replace("_link", "").replace("_", " ").title()

def _tracking_metric_key(records: list[dict], base_key: str) -> str:
    pre_key = f"pre_relaxation_{base_key}"
    return pre_key if pre_key in records[0] else base_key


# -----------------------------------------------------------------------------
# Summary tables.
# -----------------------------------------------------------------------------

def compute_main_metric_summary(records: list[dict]) -> dict[str, float]:
    """Compute aggregate values for important evaluation metrics."""

    summary = {}

    if "final_top_face_correct" in records[0]:
        summary["top_face_success_rate"] = float(
            np.mean([record["final_top_face_correct"] for record in records])
        )

    deformation_rms_mean = np.asarray([record["deformation_rms_mean_m"] for record in records], dtype=np.float64)
    deformation_rms_peak = np.asarray([record["deformation_rms_peak_m"] for record in records], dtype=np.float64)
    deformation_high_fraction = np.asarray([record["deformation_high_fraction"] for record in records],dtype=np.float64)
    torque_rms_overall = np.asarray([record["torque_rms_overall_nm"] for record in records],dtype=np.float64)
    action_delta_rms_overall = np.asarray([record["action_delta_rms_overall"] for record in records],dtype=np.float64)

    summary["num_episodes"] = len(records)
    summary["deformation_overall_mean_rms_mm"] = float(1000.0 * np.mean(deformation_rms_mean))
    summary["deformation_overall_std_rms_mm"] = float(1000.0 * np.std(deformation_rms_mean))
    summary["deformation_mean_episode_peak_rms_mm"] = float(1000.0 * np.mean(deformation_rms_peak))
    summary["deformation_worst_episode_peak_rms_mm"] = float(1000.0 * np.max(deformation_rms_peak))
    summary["deformation_high_fraction_mean_pct"] = float(100.0 * np.mean(deformation_high_fraction))
    summary["deformation_high_fraction_std_pct"] = float(100.0 * np.std(deformation_high_fraction))
    summary["torque_rms_overall_mean_nm"] = float(np.mean(torque_rms_overall))
    summary["torque_rms_overall_std_nm"] = float(np.std(torque_rms_overall))
    summary["action_delta_rms_overall_mean"] = float(np.mean(action_delta_rms_overall))
    summary["action_delta_rms_overall_std"] = float(np.std(action_delta_rms_overall))

    if "final_landing_xy_error_m" in records[0]:
        final_landing_xy_error = np.asarray([r["final_landing_xy_error_m"] for r in records], dtype=np.float64)
        reference_landing_xy_error = np.asarray([r["reference_landing_xy_error_m"] for r in records], dtype=np.float64)
        position_improvement = np.asarray([r["landing_position_improvement_m"] for r in records], dtype=np.float64)

        final_orientation_error = np.asarray([r["final_landing_orientation_error_deg"] for r in records], dtype=np.float64)
        reference_orientation_error = np.asarray([r["reference_landing_orientation_error_deg"] for r in records], dtype=np.float64)
        orientation_improvement = np.asarray([r["landing_orientation_improvement_deg"] for r in records], dtype=np.float64)

        summary["reference_landing_xy_error_mean_cm"] = float(100.0 * np.mean(reference_landing_xy_error))
        summary["final_landing_xy_error_mean_cm"] = float(100.0 * np.mean(final_landing_xy_error))
        summary["final_landing_xy_error_std_cm"] = float(100.0 * np.std(final_landing_xy_error))
        summary["landing_position_improvement_mean_cm"] = float(100.0 * np.mean(position_improvement))
        summary["landing_position_improvement_std_cm"] = float(100.0 * np.std(position_improvement))

        summary["reference_landing_orientation_error_mean_deg"] = float(np.mean(reference_orientation_error))
        summary["final_landing_orientation_error_mean_deg"] = float(np.mean(final_orientation_error))
        summary["final_landing_orientation_error_std_deg"] = float(np.std(final_orientation_error))
        summary["landing_orientation_improvement_mean_deg"] = float(np.mean(orientation_improvement))
        summary["landing_orientation_improvement_std_deg"] = float(np.std(orientation_improvement))

        summary["landing_position_improved_rate"] = float(np.mean([r["landing_position_improved"] for r in records]))
        summary["landing_orientation_improved_rate"] = float(np.mean([r["landing_orientation_improved"] for r in records]))

        summary["landing_position_success_rate"] = float(np.mean([r["final_landing_position_success"] for r in records]))
        summary["landing_orientation_success_rate"] = float(np.mean([r["final_landing_orientation_success"] for r in records]))
        summary["landing_success_rate"] = float(np.mean([r["final_landing_success"] for r in records]))
        summary["task_success_rate"] = float(np.mean([r["task_success"] for r in records]))

        summary["reference_landing_success_rate"] = float(np.mean([r["reference_landing_success"] for r in records]))
        summary["landing_success_gain_rate"] = float(np.mean([r["landing_success_gain"] for r in records]))
        summary["landing_success_loss_rate"] = float(np.mean([r["landing_success_loss"] for r in records]))

    tracking_metrics = {
        "tracking_body_position_rmse_m": "body_position_rmse_m",
        "tracking_body_orientation_rmse_deg": "body_orientation_rmse_deg",
        "tracking_hand_position_rmse_m": "hand_position_rmse_m",
        "tracking_cube_xy_position_rmse_m": "cube_xy_position_rmse_m",
        "tracking_cube_orientation_rmse_deg": "cube_orientation_rmse_deg",
    }

    for output_key, base_key in tracking_metrics.items():
        values = np.asarray([r[_tracking_metric_key(records, base_key)] for r in records], dtype=np.float64)
        summary[f"{output_key}_mean"] = float(np.mean(values))
        summary[f"{output_key}_std"] = float(np.std(values))

    return summary

def group_records_by_motion(records: list[dict]) -> dict[str, list[dict]]:
    records_by_motion = defaultdict(list)

    for record in records:
        records_by_motion[record["motion_name"]].append(record)

    return dict(sorted(records_by_motion.items()))


def compute_per_motion_metric_summary(records: list[dict]) -> dict[str, dict[str, float]]:
    records_by_motion = group_records_by_motion(records)

    return {
        motion_name: compute_main_metric_summary(motion_records)
        for motion_name, motion_records in records_by_motion.items()
    }

def _landing_improvement_labels(records: list[dict]):
    position_labels = []
    orientation_labels = []
    combined_labels = []
    success_labels = []

    for record in records:
        pos_better = bool(record["landing_position_improved"] > 0.5)
        pos_worse = bool(record["landing_position_worsened"] > 0.5)
        ori_better = bool(record["landing_orientation_improved"] > 0.5)
        ori_worse = bool(record["landing_orientation_worsened"] > 0.5)

        if pos_better:
            position_labels.append("Better")
        elif pos_worse:
            position_labels.append("Worse")
        else:
            position_labels.append("Same")

        if ori_better:
            orientation_labels.append("Better")
        elif ori_worse:
            orientation_labels.append("Worse")
        else:
            orientation_labels.append("Same")

        if (pos_better or ori_better) and not (pos_worse or ori_worse):
            combined_labels.append("Improved")
        elif (pos_worse or ori_worse) and not (pos_better or ori_better):
            combined_labels.append("Worse")
        elif pos_better or pos_worse or ori_better or ori_worse:
            combined_labels.append("Mixed")
        else:
            combined_labels.append("Same")

        reference_success = bool(record["reference_landing_success"] > 0.5)
        final_success = bool(record["final_landing_success"] > 0.5)

        if reference_success and final_success:
            success_labels.append("Kept success")
        elif not reference_success and final_success:
            success_labels.append("Failure→success")
        elif reference_success and not final_success:
            success_labels.append("Success→failure")
        else:
            success_labels.append("Kept failure")

    return position_labels, orientation_labels, combined_labels, success_labels

def _count_categories(labels: list[str], categories: list[str]) -> np.ndarray:
    counts = Counter(labels)
    return np.asarray([counts.get(category, 0) for category in categories], dtype=np.float64)


def _make_episode_table(records: list[dict]) -> wandb.Table:
    """Compact one-row-per-episode evaluation table."""

    landing_aware = "task_success" in records[0]

    columns = [
        "episode",
        "motion",
        "termination",
        "duration_s",
        "motion_finished",
        "tracking_cube_xy_rmse_cm",
        "tracking_cube_orientation_rmse_deg",
        "deformation_rms_mm",
        "torque_rms_nm",
        "action_delta_rms",
    ]

    if landing_aware:
        columns += [
            "task_success",
            "landing_success",
            "position_success",
            "orientation_success",
            "top_face_success",
            "landing_xy_error_cm",
            "landing_orientation_error_deg",
            "reference_landing_success",
            "position_improvement_cm",
            "orientation_improvement_deg",
        ]
    else:
        columns += [
            "top_face_correct",
            "final_xy_reference_error_cm",
            "final_orientation_reference_error_deg",
        ]

    cube_xy_key = _tracking_metric_key(records, "cube_xy_position_rmse_m")
    cube_orientation_key = _tracking_metric_key(records, "cube_orientation_rmse_deg")

    data = []

    for record in records:
        row = [
            record["episode_id"],
            record["motion_name"],
            record["termination"],
            record["duration_s"],
            bool(record["motion_finished"]),
            100.0 * record[cube_xy_key],
            record[cube_orientation_key],
            1000.0 * record["deformation_rms_mean_m"],
            record["torque_rms_overall_nm"],
            record["action_delta_rms_overall"],
        ]

        if landing_aware:
            row += [
                bool(record["task_success"]),
                bool(record["final_landing_success"]),
                bool(record["final_landing_position_success"]),
                bool(record["final_landing_orientation_success"]),
                bool(record["final_top_face_success"]),
                100.0 * record["final_landing_xy_error_m"],
                record["final_landing_orientation_error_deg"],
                bool(record["reference_landing_success"]),
                100.0 * record["landing_position_improvement_m"],
                record["landing_orientation_improvement_deg"],
            ]
        else:
            row += [
                bool(record["final_top_face_correct"]),
                100.0 * record["final_xy_position_error_m"],
                record["final_orientation_error_deg"],
            ]

        data.append(row)

    return wandb.Table(columns=columns, data=data)

def _make_aggregate_table(records: list[dict]) -> wandb.Table:
    """Mean/std/median/max table for primary metrics."""

    columns = ["metric", "mean", "std", "median", "max", "unit"]
    data = []

    for display_name, key, scale, unit in _aggregate_metric_specs(records):
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

def make_per_motion_summary_table(per_motion_summary: dict[str, dict[str, float]]) -> wandb.Table:
    landing_aware = "task_success_rate" in next(iter(per_motion_summary.values()))

    columns = [
        "motion",
        "episodes",
        "tracking_cube_xy_rmse_cm",
        "tracking_cube_orientation_rmse_deg",
        "tracking_body_position_rmse_cm",
        "tracking_body_orientation_rmse_deg",
        "tracking_hand_position_rmse_cm",
        "deformation_rms_mm",
        "deformation_peak_rms_mm",
        "high_deformation_pct",
        "torque_rms_nm",
        "action_delta_rms",
    ]

    if landing_aware:
        columns += [
            "task_success_pct",
            "landing_success_pct",
            "landing_xy_error_cm",
            "landing_orientation_error_deg",
            "position_improvement_cm",
            "orientation_improvement_deg",
        ]

    data = []

    for motion_name, metrics in per_motion_summary.items():
        row = [
            motion_name,
            metrics["num_episodes"],
            100.0 * metrics["tracking_cube_xy_position_rmse_m_mean"],
            metrics["tracking_cube_orientation_rmse_deg_mean"],
            100.0 * metrics["tracking_body_position_rmse_m_mean"],
            metrics["tracking_body_orientation_rmse_deg_mean"],
            100.0 * metrics["tracking_hand_position_rmse_m_mean"],
            metrics["deformation_overall_mean_rms_mm"],
            metrics["deformation_mean_episode_peak_rms_mm"],
            metrics["deformation_high_fraction_mean_pct"],
            metrics["torque_rms_overall_mean_nm"],
            metrics["action_delta_rms_overall_mean"],
        ]

        if landing_aware:
            row += [
                100.0 * metrics["task_success_rate"],
                100.0 * metrics["landing_success_rate"],
                metrics["final_landing_xy_error_mean_cm"],
                metrics["final_landing_orientation_error_mean_deg"],
                metrics["landing_position_improvement_mean_cm"],
                metrics["landing_orientation_improvement_mean_deg"],
            ]

        data.append(row)

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
# Aggregate body/hand errors.
# -----------------------------------------------------------------------------

def _plot_per_body_rmse(records: list[dict]):

    position_labels, position_values = [], []

    for body_name in H1_TRACKED_BODY_NAMES:
        key = _tracking_metric_key(records, f"body_position_rmse_{body_name}_m")
        position_labels.append(_pretty_name(body_name))
        position_values.append(100.0 * np.mean([r[key] for r in records]))

    for hand_name in H1_HAND_REFERENCE_NAMES:
        key = _tracking_metric_key(records, f"hand_position_rmse_{hand_name}_m")
        position_labels.append(_pretty_name(hand_name))
        position_values.append(100.0 * np.mean([r[key] for r in records]))

    orientation_labels, orientation_values = [], []

    for body_name in H1_TRACKED_BODY_NAMES:
        key = _tracking_metric_key(records, f"body_orientation_rmse_{body_name}_deg")
        orientation_labels.append(_pretty_name(body_name))
        orientation_values.append(np.mean([r[key] for r in records]))

    fig, axes = plt.subplots(1, 2, figsize=(13, 6))

    axes[0].barh(position_labels, position_values)
    axes[0].set_title("Position tracking RMSE")
    axes[0].set_xlabel("RMSE [cm]")

    axes[1].barh(orientation_labels, orientation_values)
    axes[1].set_title("Orientation tracking RMSE")
    axes[1].set_xlabel("RMSE [deg]")

    scope = "Pre-relaxation" if "pre_relaxation_body_position_rmse_m" in records[0] else "Full-trajectory"
    fig.suptitle(f"{scope} body and hand tracking")
    fig.tight_layout()

    return fig


# -----------------------------------------------------------------------------
# Tracking metrics
# -----------------------------------------------------------------------------

def _plot_tracking_quality_overview(records: list[dict]):

    body_pos = 100.0 * np.asarray([r[_tracking_metric_key(records, "body_position_rmse_m")] for r in records])
    hand_pos = 100.0 * np.asarray([r[_tracking_metric_key(records, "hand_position_rmse_m")] for r in records])
    cube_pos = 100.0 * np.asarray([r[_tracking_metric_key(records, "cube_xy_position_rmse_m")] for r in records])
    body_ori = np.asarray([r[_tracking_metric_key(records, "body_orientation_rmse_deg")] for r in records])
    cube_ori = np.asarray([r[_tracking_metric_key(records, "cube_orientation_rmse_deg")] for r in records])

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    position_data = [body_pos, hand_pos, cube_pos]
    for i, values in enumerate(position_data, 1):
        axes[0].scatter(np.linspace(i - 0.08, i + 0.08, len(values)), values, alpha=0.5)
    axes[0].boxplot(position_data, labels=["Body", "Hands", "Cube XY"])
    axes[0].set_ylabel("RMSE [cm]")
    axes[0].set_title("Position tracking")

    orientation_data = [body_ori, cube_ori]
    for i, values in enumerate(orientation_data, 1):
        axes[1].scatter(np.linspace(i - 0.08, i + 0.08, len(values)), values, alpha=0.5)
    axes[1].boxplot(orientation_data, labels=["Body", "Cube"])
    axes[1].set_ylabel("RMSE [deg]")
    axes[1].set_title("Orientation tracking")

    scope = "Pre-relaxation" if "pre_relaxation_body_position_rmse_m" in records[0] else "Full-trajectory"
    fig.suptitle(f"{scope} tracking quality")
    fig.tight_layout()

    return fig

def _plot_position_tracking_by_motion(records: list[dict]):

    records_by_motion = group_records_by_motion(records)
    motion_names = list(records_by_motion)
    specs = [
        ("body_position_rmse_m", "Body"),
        ("hand_position_rmse_m", "Hands"),
        ("cube_xy_position_rmse_m", "Cube XY"),
    ]

    y = np.arange(len(motion_names))
    fig, axes = plt.subplots(1, 3, figsize=(18, max(5, 0.5 * len(motion_names))), sharey=True)

    for ax, (base_key, title) in zip(axes, specs):
        means, stds = [], []
        for motion_name in motion_names:
            motion_records = records_by_motion[motion_name]
            key = _tracking_metric_key(motion_records, base_key)
            values = 100.0 * np.asarray([r[key] for r in motion_records], dtype=np.float64)
            means.append(np.mean(values))
            stds.append(np.std(values))

        ax.barh(y, means, xerr=stds)
        ax.set_xlabel("RMSE [cm]")
        ax.set_title(title)

    axes[0].set_yticks(y, labels=motion_names)
    axes[0].invert_yaxis()

    scope = "Pre-relaxation" if "pre_relaxation_body_position_rmse_m" in records[0] else "Full-trajectory"
    fig.suptitle(f"{scope} position tracking by trajectory")
    fig.tight_layout()

    return fig

def _plot_orientation_tracking_by_motion(records: list[dict]):

    records_by_motion = group_records_by_motion(records)
    motion_names = list(records_by_motion)
    specs = [
        ("body_orientation_rmse_deg", "Body"),
        ("cube_orientation_rmse_deg", "Cube"),
    ]

    y = np.arange(len(motion_names))
    fig, axes = plt.subplots(1, 2, figsize=(14, max(5, 0.5 * len(motion_names))), sharey=True)

    for ax, (base_key, title) in zip(axes, specs):
        means, stds = [], []
        for motion_name in motion_names:
            motion_records = records_by_motion[motion_name]
            key = _tracking_metric_key(motion_records, base_key)
            values = np.asarray([r[key] for r in motion_records], dtype=np.float64)
            means.append(np.mean(values))
            stds.append(np.std(values))

        ax.barh(y, means, xerr=stds)
        ax.set_xlabel("RMSE [deg]")
        ax.set_title(title)

    axes[0].set_yticks(y, labels=motion_names)
    axes[0].invert_yaxis()

    scope = "Pre-relaxation" if "pre_relaxation_body_position_rmse_m" in records[0] else "Full-trajectory"
    fig.suptitle(f"{scope} orientation tracking by trajectory")
    fig.tight_layout()

    return fig

#-------------------------------------------------------------------
# Contro metrics
#---------------------------------------------------------------------

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

def _plot_control_quality_overview(records: list[dict]):
    torque_rms_nm = np.asarray(
        [r["torque_rms_overall_nm"] for r in records],
        dtype=np.float64,
    )

    action_delta_rms = np.asarray(
        [r["action_delta_rms_overall"] for r in records],
        dtype=np.float64,
    )

    components = [
        (
            "Overall torque RMS",
            torque_rms_nm,
            "Torque RMS [Nm]",
        ),
        (
            "Overall action-delta RMS",
            action_delta_rms,
            "Action Δ RMS",
        ),
    ]

    fig, axes = plt.subplots(1, 2, figsize=(10, 5))

    for ax, (title, values, ylabel) in zip(axes, components):
        ax.boxplot([values], widths=0.35)

        x = np.linspace(
            0.94,
            1.06,
            len(values),
        )

        ax.scatter(
            x,
            values,
            alpha=0.5,
        )

        ax.set_xticks([])
        ax.set_ylabel(ylabel)
        ax.set_title(title)

    fig.suptitle("Control-quality overview")
    fig.tight_layout()

    return fig

def _plot_control_quality_by_motion(records: list[dict]):
    records_by_motion = group_records_by_motion(records)
    motion_names = list(records_by_motion.keys())

    torque_mean_nm = []
    torque_std_nm = []

    action_delta_mean = []
    action_delta_std = []

    for motion_name in motion_names:
        motion_records = records_by_motion[motion_name]

        torque_values = np.asarray(
            [
                r["torque_rms_overall_nm"]
                for r in motion_records
            ],
            dtype=np.float64,
        )

        action_delta_values = np.asarray(
            [
                r["action_delta_rms_overall"]
                for r in motion_records
            ],
            dtype=np.float64,
        )

        torque_mean_nm.append(
            np.mean(torque_values)
        )
        torque_std_nm.append(
            np.std(torque_values)
        )

        action_delta_mean.append(
            np.mean(action_delta_values)
        )
        action_delta_std.append(
            np.std(action_delta_values)
        )

    y = np.arange(len(motion_names))

    fig, axes = plt.subplots(
        1,
        2,
        figsize=(14, max(5, 0.5 * len(motion_names))),
        sharey=True,
    )

    axes[0].barh(
        y,
        torque_mean_nm,
        xerr=torque_std_nm,
    )

    axes[0].set_yticks(
        y,
        labels=motion_names,
    )
    axes[0].invert_yaxis()
    axes[0].set_xlabel("Overall torque RMS [Nm]")
    axes[0].set_title("Torque effort")

    axes[1].barh(
        y,
        action_delta_mean,
        xerr=action_delta_std,
    )

    axes[1].set_xlabel("Overall action Δ RMS")
    axes[1].set_title("Action smoothness")

    fig.suptitle("Control quality by trajectory")
    fig.tight_layout()

    return fig

def _make_extreme_torque_episode_table(
    records: list[dict],
    trajectory_records: list[dict],
    joint_names: list[str],
    joint_effort_limits_nm: list[float] | np.ndarray,
    utilization_threshold: float,
) -> wandb.Table:
    records_by_episode = {
        int(r["episode_id"]): r
        for r in records
    }

    effort_limits = np.asarray(
        joint_effort_limits_nm,
        dtype=np.float64,
    )

    if effort_limits.shape != (len(joint_names),):
        raise ValueError(
            "joint_effort_limits_nm must have one value per controlled joint."
        )

    if (
        not np.all(np.isfinite(effort_limits))
        or np.any(effort_limits <= 0.0)
    ):
        raise ValueError(
            "Joint effort limits must be finite and positive."
        )

    extreme_episodes = []

    for trajectory in trajectory_records:
        episode_id = int(
            trajectory["episode_id"]
        )

        record = records_by_episode[
            episode_id
        ]

        frames = np.asarray(
            trajectory["motion_frame"],
            dtype=np.int64,
        )

        torque_nm = np.asarray(
            trajectory["joint_torque_nm"],
            dtype=np.float64,
        )

        utilization = (
            np.abs(torque_nm)
            / effort_limits[None, :]
        )

        peak_flat_idx = int(
            np.argmax(utilization)
        )

        peak_frame_idx, peak_joint_idx = (
            np.unravel_index(
                peak_flat_idx,
                utilization.shape,
            )
        )

        peak_utilization = float(
            utilization[
                peak_frame_idx,
                peak_joint_idx,
            ]
        )

        if peak_utilization < utilization_threshold:
            continue

        high_utilization_mask = (
            utilization
            >= utilization_threshold
        )

        high_utilization_frame_fraction = float(
            np.mean(
                np.any(
                    high_utilization_mask,
                    axis=1,
                )
            )
        )

        violating_joint_ids = np.flatnonzero(
            np.any(
                high_utilization_mask,
                axis=0,
            )
        )

        violating_joints = [
            joint_names[i]
            for i in violating_joint_ids
        ]

        extreme_episodes.append(
            [
                record["motion_name"],
                episode_id,
                int(frames[peak_frame_idx]),
                joint_names[peak_joint_idx],
                float(
                    torque_nm[
                        peak_frame_idx,
                        peak_joint_idx,
                    ]
                ),
                float(
                    effort_limits[
                        peak_joint_idx
                    ]
                ),
                100.0 * peak_utilization,
                100.0 * high_utilization_frame_fraction,
                ", ".join(violating_joints),
            ]
        )

    extreme_episodes.sort(
        key=lambda row: row[6],
        reverse=True,
    )

    return wandb.Table(
        columns=[
            "motion",
            "episode",
            "peak_frame",
            "peak_joint",
            "peak_torque_nm",
            "joint_effort_limit_nm",
            "peak_utilization_pct",
            "high_utilization_frames_pct",
            "joints_above_threshold",
        ],
        data=extreme_episodes,
    )
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
# Deformation plots 
# -----------------------------------------------------------------------------

def _plot_deformation_overview(
    records: list[dict],
    high_deformation_threshold_mm: float,
):
    deformation_mean_mm = 1000.0 * np.asarray([r["deformation_rms_mean_m"] for r in records], dtype=np.float64)
    deformation_peak_mm = 1000.0 * np.asarray([r["deformation_rms_peak_m"] for r in records], dtype=np.float64)
    high_deformation_pct = 100.0 * np.asarray([r["deformation_high_fraction"] for r in records], dtype=np.float64)

    components = [
        ("Mean RMS deformation", deformation_mean_mm, "RMS deformation [mm]"),
        ("Peak RMS deformation", deformation_peak_mm, "Peak RMS deformation [mm]"),
        (f"High-deformation frames\n(RMS > {high_deformation_threshold_mm:g} mm)", high_deformation_pct, "Episode frames [%]"),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    for ax, (title, values, ylabel) in zip(axes, components):
        ax.boxplot([values], widths=0.35)
        x = np.linspace(0.94, 1.06, len(values))
        ax.scatter(x, values, alpha=0.5)

        ax.set_xticks([])
        ax.set_title(title)
        ax.set_ylabel(ylabel)

    axes[2].set_ylim(bottom=0.0)

    fig.suptitle("Deformable-object deformation overview")
    fig.tight_layout()
    return fig

def _plot_deformation_by_motion(records: list[dict]):
    records_by_motion = group_records_by_motion(records)
    motion_names = list(records_by_motion.keys())

    mean_rms_mm = []
    mean_rms_std_mm = []

    peak_rms_mm = []
    peak_rms_std_mm = []

    high_fraction_pct = []
    high_fraction_std_pct = []

    for motion_name in motion_names:
        motion_records = records_by_motion[motion_name]

        rms_values_mm = 1000.0 * np.asarray([r["deformation_rms_mean_m"] for r in motion_records], dtype=np.float64)
        peak_values_mm = 1000.0 * np.asarray([r["deformation_rms_peak_m"] for r in motion_records], dtype=np.float64)
        high_values_pct = 100.0 * np.asarray([r["deformation_high_fraction"] for r in motion_records], dtype=np.float64)

        mean_rms_mm.append(np.mean(rms_values_mm))
        mean_rms_std_mm.append(np.std(rms_values_mm))

        peak_rms_mm.append(np.mean(peak_values_mm))
        peak_rms_std_mm.append(np.std(peak_values_mm))

        high_fraction_pct.append(np.mean(high_values_pct))
        high_fraction_std_pct.append(np.std(high_values_pct))

    y = np.arange(len(motion_names))

    fig, axes = plt.subplots(
        1,
        3,
        figsize=(18, max(5, 0.5 * len(motion_names))),
        sharey=True,
    )

    axes[0].barh(y, mean_rms_mm, xerr=mean_rms_std_mm)
    axes[0].set_yticks(y, labels=motion_names)
    axes[0].invert_yaxis()
    axes[0].set_xlabel("Mean RMS deformation [mm]")
    axes[0].set_title("Mean deformation")

    axes[1].barh(y, peak_rms_mm, xerr=peak_rms_std_mm)
    axes[1].set_xlabel("Episode peak RMS [mm]")
    axes[1].set_title("Peak deformation")

    axes[2].barh(y, high_fraction_pct, xerr=high_fraction_std_pct)
    axes[2].set_xlabel("Episode frames [%]")
    axes[2].set_title("High-deformation fraction")

    fig.suptitle("Deformation by trajectory")
    fig.tight_layout()

    return fig

def _make_extreme_deformation_episode_table(
    records: list[dict],
    trajectory_records: list[dict],
    severe_peak_threshold_mm: float,
    sustained_high_fraction: float,
    extreme_local_threshold_mm: float,
) -> wandb.Table:
    records_by_episode = {int(r["episode_id"]): r for r in records}
    extreme_episodes = []

    for trajectory in trajectory_records:
        episode_id = int(trajectory["episode_id"])
        record = records_by_episode[episode_id]

        peak_rms_mm = 1000.0 * float(record["deformation_rms_peak_m"])
        high_fraction = float(record["deformation_high_fraction"])
        episode_max_mm = 1000.0 * float(record["deformation_max_peak_m"])

        severe_peak = peak_rms_mm > severe_peak_threshold_mm
        sustained = high_fraction > sustained_high_fraction
        extreme_local = episode_max_mm > extreme_local_threshold_mm

        if not (severe_peak or sustained or extreme_local):
            continue

        frames = np.asarray(trajectory["motion_frame"], dtype=np.int64)
        rms_mm = 1000.0 * np.asarray(trajectory["deformation_rms_m"], dtype=np.float64)
        p95_mm = 1000.0 * np.asarray(trajectory["deformation_p95_m"], dtype=np.float64)
        max_mm = 1000.0 * np.asarray(trajectory["deformation_max_m"], dtype=np.float64)

        peak_idx = int(np.argmax(rms_mm))

        reasons = []
        if severe_peak:
            reasons.append("severe_peak")
        if sustained:
            reasons.append("sustained")
        if extreme_local:
            reasons.append("extreme_local")

        extreme_episodes.append([
            record["motion_name"],
            episode_id,
            int(frames[peak_idx]),
            float(rms_mm[peak_idx]),
            float(p95_mm[peak_idx]),
            float(max_mm[peak_idx]),
            float(100.0 * high_fraction),
            episode_max_mm,
            ", ".join(reasons),
        ])

    extreme_episodes.sort(key=lambda row: row[3], reverse=True)

    return wandb.Table(
        columns=[
            "motion",
            "episode",
            "peak_rms_frame",
            "peak_rms_mm",
            "p95_at_peak_mm",
            "max_at_peak_mm",
            "high_deformation_pct",
            "episode_max_nodal_mm",
            "reason",
        ],
        data=extreme_episodes,
    )

#-----------------------------------------------------------------------------
# Landing aware task
#-----------------------------------------------------------------------------
def _plot_landing_xy_scatter(
    records: list[dict],
    landing_center_xy: tuple[float, float],
    landing_radius: float,
):
    records_by_motion = group_records_by_motion(records)

    center_x_cm = 100.0 * landing_center_xy[0]
    center_y_cm = 100.0 * landing_center_xy[1]
    radius_cm = 100.0 * landing_radius

    fig, ax = plt.subplots(figsize=(7, 7))

    all_x = []
    all_y = []

    for motion_name, motion_records in records_by_motion.items():
        x_cm = 100.0 * np.asarray([r["final_cube_x_r0_m"] for r in motion_records])
        y_cm = 100.0 * np.asarray([r["final_cube_y_r0_m"] for r in motion_records])

        scatter = ax.scatter(x_cm, y_cm, label=motion_name)
        color = scatter.get_facecolor()[0]

        reference_x_cm = 100.0 * motion_records[0]["reference_final_cube_x_r0_m"]
        reference_y_cm = 100.0 * motion_records[0]["reference_final_cube_y_r0_m"]

        ax.scatter(reference_x_cm, reference_y_cm, marker="x", s=100, linewidths=2.0, color=color)

        all_x.extend(x_cm.tolist())
        all_y.extend(y_cm.tolist())
        all_x.append(reference_x_cm)
        all_y.append(reference_y_cm)

    landing_circle = plt.Circle(
        (center_x_cm, center_y_cm),
        radius_cm,
        fill=False,
        linestyle="--",
        linewidth=2.0,
    )
    ax.add_patch(landing_circle)

    ax.scatter([], [], marker="x", s=100, linewidths=2.0, label="Final reference")
    ax.scatter(center_x_cm, center_y_cm, marker="+", s=100, label="Landing center")

    all_x.extend([center_x_cm - radius_cm, center_x_cm + radius_cm])
    all_y.extend([center_y_cm - radius_cm, center_y_cm + radius_cm])

    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)

    span = max(x_max - x_min, y_max - y_min, 2.0 * radius_cm)
    padding = 0.15 * span

    center_plot_x = 0.5 * (x_min + x_max)
    center_plot_y = 0.5 * (y_min + y_max)

    ax.set_xlim(center_plot_x - 0.5 * span - padding, center_plot_x + 0.5 * span + padding)
    ax.set_ylim(center_plot_y - 0.5 * span - padding, center_plot_y + 0.5 * span + padding)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Cube X in R0 [cm]")
    ax.set_ylabel("Cube Y in R0 [cm]")
    ax.set_title("Terminal cube landing positions")
    ax.legend(fontsize=8)

    fig.tight_layout()
    return fig

def _plot_landing_improvement_overview(records: list[dict]):
    position_labels, orientation_labels, combined_labels, success_labels = _landing_improvement_labels(records)

    pie_specs = [
        ("Position vs reference", position_labels, ["Better", "Same", "Worse"]),
        ("Orientation vs reference", orientation_labels, ["Better", "Same", "Worse"]),
        ("Combined improvement", combined_labels, ["Improved", "Mixed", "Same", "Worse"]),
        ("Landing success conversion", success_labels, ["Kept success", "Failure→success", "Success→failure", "Kept failure"]),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    axes = axes.flatten()

    for ax, (title, labels, categories) in zip(axes, pie_specs):
        counts = _count_categories(labels, categories)

        if np.sum(counts) == 0:
            ax.axis("off")
            continue

        ax.pie(
            counts,
            labels=categories,
            autopct=lambda pct: f"{pct:.1f}%",
            startangle=90,
        )
        ax.set_title(title)

    fig.suptitle("Landing improvement overview")
    fig.tight_layout()
    return fig

def _plot_landing_improvement_by_motion(records: list[dict]):
    
    records_by_motion = group_records_by_motion(records)
    motion_names = list(records_by_motion.keys())

    position_categories = ["Better", "Same", "Worse"]
    orientation_categories = ["Better", "Same", "Worse"]
    combined_categories = ["Improved", "Mixed", "Same", "Worse"]

    position_pct = []
    orientation_pct = []
    combined_pct = []

    for motion_name in motion_names:
        motion_records = records_by_motion[motion_name]

        position_labels, orientation_labels, combined_labels, _ = _landing_improvement_labels(
            motion_records,
        )

        position_counts = _count_categories(position_labels, position_categories)
        orientation_counts = _count_categories(orientation_labels, orientation_categories)
        combined_counts = _count_categories(combined_labels, combined_categories)

        position_pct.append(100.0 * position_counts / np.sum(position_counts))
        orientation_pct.append(100.0 * orientation_counts / np.sum(orientation_counts))
        combined_pct.append(100.0 * combined_counts / np.sum(combined_counts))

    position_pct = np.asarray(position_pct, dtype=np.float64)
    orientation_pct = np.asarray(orientation_pct, dtype=np.float64)
    combined_pct = np.asarray(combined_pct, dtype=np.float64)

    y = np.arange(len(motion_names))
    fig, axes = plt.subplots(1, 3, figsize=(18, max(5, 0.5 * len(motion_names))), sharey=True)

    left = np.zeros(len(motion_names), dtype=np.float64)
    for idx, category in enumerate(position_categories):
        values = position_pct[:, idx]
        axes[0].barh(y, values, left=left, label=category)
        left += values
    axes[0].set_yticks(y, labels=motion_names)
    axes[0].invert_yaxis()
    axes[0].set_xlim(0.0, 100.0)
    axes[0].set_xlabel("Episodes [%]")
    axes[0].set_title("Position improvement")

    left = np.zeros(len(motion_names), dtype=np.float64)
    for idx, category in enumerate(orientation_categories):
        values = orientation_pct[:, idx]
        axes[1].barh(y, values, left=left, label=category)
        left += values
    axes[1].set_xlim(0.0, 100.0)
    axes[1].set_xlabel("Episodes [%]")
    axes[1].set_title("Orientation improvement")

    left = np.zeros(len(motion_names), dtype=np.float64)
    for idx, category in enumerate(combined_categories):
        values = combined_pct[:, idx]
        axes[2].barh(y, values, left=left, label=category)
        left += values
    axes[2].set_xlim(0.0, 100.0)
    axes[2].set_xlabel("Episodes [%]")
    axes[2].set_title("Combined improvement")

    axes[0].legend(fontsize=8)
    axes[1].legend(fontsize=8)
    axes[2].legend(fontsize=8)

    fig.tight_layout()
    return fig

def _plot_landing_quality_overview(records: list[dict], orientation_success_threshold_deg: float):

    position_reference_cm = 100.0 * np.asarray([r["reference_landing_xy_error_m"] for r in records], dtype=np.float64)
    position_policy_cm = 100.0 * np.asarray([r["final_landing_xy_error_m"] for r in records], dtype=np.float64)
    orientation_reference_deg = np.asarray([r["reference_landing_orientation_error_deg"] for r in records], dtype=np.float64)
    orientation_policy_deg = np.asarray([r["final_landing_orientation_error_deg"] for r in records], dtype=np.float64)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    position_data = [position_reference_cm, position_policy_cm]
    orientation_data = [orientation_reference_deg, orientation_policy_deg]
    position_labels = ["Reference", "Policy"]
    orientation_labels = ["Reference", "Policy"]

    axes[0].boxplot(position_data, labels=position_labels, widths=0.5)
    for i, values in enumerate(position_data, start=1):
        x = np.linspace(i - 0.08, i + 0.08, len(values))
        axes[0].scatter(x, values, alpha=0.5)
    axes[0].axhline(0.0, linewidth=0.8, linestyle="--")
    axes[0].set_ylabel("Landing XY error [cm]")
    axes[0].set_title("Landing position quality")

    axes[1].boxplot(orientation_data, labels=orientation_labels, widths=0.5)
    for i, values in enumerate(orientation_data, start=1):
        x = np.linspace(i - 0.08, i + 0.08, len(values))
        axes[1].scatter(x, values, alpha=0.5)
    axes[1].axhline(orientation_success_threshold_deg, linewidth=1.0, linestyle="--", label=f"Success threshold ({orientation_success_threshold_deg:.1f}°)")
    axes[1].set_ylabel("Canonical orientation error [deg]")
    axes[1].set_title("Landing orientation quality")
    axes[1].legend(fontsize=8)

    fig.tight_layout()
    return fig

def _plot_landing_improvement_magnitude_by_motion(records: list[dict]):
    records_by_motion = group_records_by_motion(records)
    motion_names = list(records_by_motion.keys())

    position_mean_cm = []
    position_std_cm = []
    orientation_mean_deg = []
    orientation_std_deg = []

    for motion_name in motion_names:
        motion_records = records_by_motion[motion_name]
        position_values_cm = 100.0 * np.asarray([r["landing_position_improvement_m"] for r in motion_records], dtype=np.float64)
        orientation_values_deg = np.asarray([r["landing_orientation_improvement_deg"] for r in motion_records], dtype=np.float64)

        position_mean_cm.append(np.mean(position_values_cm))
        position_std_cm.append(np.std(position_values_cm))
        orientation_mean_deg.append(np.mean(orientation_values_deg))
        orientation_std_deg.append(np.std(orientation_values_deg))

    y = np.arange(len(motion_names))
    fig, axes = plt.subplots(1, 2, figsize=(14, max(5, 0.5 * len(motion_names))), sharey=True)

    axes[0].barh(y, position_mean_cm, xerr=position_std_cm)
    axes[0].axvline(0.0, linewidth=0.8, linestyle="--")
    axes[0].set_yticks(y, labels=motion_names)
    axes[0].invert_yaxis()
    axes[0].set_xlabel("Position improvement [cm]")
    axes[0].set_title("Landing position improvement by trajectory")

    axes[1].barh(y, orientation_mean_deg, xerr=orientation_std_deg)
    axes[1].axvline(0.0, linewidth=0.8, linestyle="--")
    axes[1].set_yticks(y, labels=motion_names)
    axes[1].invert_yaxis()
    axes[1].set_xlabel("Orientation improvement [deg]")
    axes[1].set_title("Landing orientation improvement by trajectory")

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
    joint_effort_limits_nm: list[float] | np.ndarray,
    torque_utilization_threshold: float,
    per_motion_summary: dict[str, dict[str, float]],
    high_deformation_threshold_mm: float,
    severe_deformation_peak_threshold_mm: float,
    sustained_high_deformation_fraction: float,
    extreme_local_deformation_threshold_mm: float,
    landing_center_xy: tuple[float, float] | None = None,
    landing_radius: float | None = None,
    orientation_success_threshold_deg: float | None = None,
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
    per_motion_table = make_per_motion_summary_table(per_motion_summary)
    termination_chart = _make_termination_chart(records)
    body_rmse_fig = _plot_per_body_rmse(records)
    tracking_overview_fig = _plot_tracking_quality_overview(records)
    position_tracking_by_motion_fig = _plot_position_tracking_by_motion(records)
    orientation_tracking_by_motion_fig = _plot_orientation_tracking_by_motion(records)
    control_summary_fig = _plot_per_joint_control_metrics(records,joint_names)
    control_overview_fig = _plot_control_quality_overview(records)
    control_by_motion_fig = _plot_control_quality_by_motion(records)
    extreme_torque_table = (
        _make_extreme_torque_episode_table(
            records=records,
            trajectory_records=trajectory_records,
            joint_names=joint_names,
            joint_effort_limits_nm=joint_effort_limits_nm,
            utilization_threshold=torque_utilization_threshold,
        )
    )
    deformation_overview_fig = _plot_deformation_overview(records,high_deformation_threshold_mm=high_deformation_threshold_mm)
    deformation_by_motion_fig = _plot_deformation_by_motion(records)
    extreme_deformation_table = _make_extreme_deformation_episode_table(
        records=records,
        trajectory_records=trajectory_records,
        severe_peak_threshold_mm=severe_deformation_peak_threshold_mm,
        sustained_high_fraction=sustained_high_deformation_fraction,
        extreme_local_threshold_mm=extreme_local_deformation_threshold_mm,
    )

    log_data = {
            # Summary.
            "evaluation_summary/episodes": episode_table,
            "evaluation_summary/aggregate": aggregate_table,
            "evaluation_summary/terminations": termination_chart,
            "evaluation_summary/per_motion": per_motion_table,

            # Cartesian tracking.
            "evaluation_tracking/per_body_rmse": wandb.Image(body_rmse_fig),
            "evaluation_tracking/overview": wandb.Image(tracking_overview_fig),
            "evaluation_tracking/position_by_motion": wandb.Image(position_tracking_by_motion_fig),
            "evaluation_tracking/orientation_by_motion": wandb.Image(orientation_tracking_by_motion_fig),

            # Control.
            "evaluation_control/per_joint_summary": wandb.Image(control_summary_fig),
            "evaluation_control/overview": wandb.Image(control_overview_fig),
            "evaluation_control/by_motion": wandb.Image(control_by_motion_fig),
            "evaluation_control/extreme_torque_episode": extreme_torque_table,
          
            # Deformation.
            "evaluation_deformation/overview": wandb.Image(deformation_overview_fig),
            "evaluation_deformation/by_motion": wandb.Image(deformation_by_motion_fig),
            "evaluation_deformation/extreme_episodes": extreme_deformation_table,
           
        }
    
    #Landing aware plots 

    landing_aware = landing_center_xy is not None
    if landing_aware and orientation_success_threshold_deg is None:
        raise ValueError("orientation_success_threshold_deg is required for LandingAware visualization.")

    if landing_aware:
        
        final_xy_fig = _plot_landing_xy_scatter(
            records=records,
            landing_center_xy=landing_center_xy,
            landing_radius=landing_radius,
        )
        landing_improvement_overview_fig = _plot_landing_improvement_overview(records)
        landing_improvement_by_motion_fig = _plot_landing_improvement_by_motion(records)
        landing_quality_fig = _plot_landing_quality_overview(records, orientation_success_threshold_deg=orientation_success_threshold_deg)
        landing_improvement_magnitude_fig = _plot_landing_improvement_magnitude_by_motion(records)

        log_data["evaluation_landing/final_xy"] = wandb.Image(final_xy_fig)
        log_data["evaluation_landing/improvement_overview"] = wandb.Image(landing_improvement_overview_fig)
        log_data["evaluation_landing/improvement_by_motion"] = wandb.Image(landing_improvement_by_motion_fig)
        log_data["evaluation_landing/quality_overview"] = wandb.Image(landing_quality_fig)
        log_data["evaluation_landing/improvement_magnitude_by_motion"] = wandb.Image(landing_improvement_magnitude_fig)
    else:
        final_xy_fig = _plot_final_xy_scatter(records)
        final_pose_fig = _plot_final_pose_components(records)
        log_data["evaluation_cube/final_xy_reference_error"] = wandb.Image(final_xy_fig)
        log_data["evaluation_cube/final_pose_components"] = wandb.Image(final_pose_fig)

    run.log(log_data)

    figures = [
        final_xy_fig,
        body_rmse_fig,
        tracking_overview_fig,
        position_tracking_by_motion_fig,
        orientation_tracking_by_motion_fig,
        control_summary_fig,
        control_overview_fig,
        control_by_motion_fig,
        deformation_overview_fig,
        deformation_by_motion_fig,
    ]

    if landing_aware:
        figures.append(landing_improvement_overview_fig)
        figures.append(landing_improvement_by_motion_fig)
        figures.append(landing_quality_fig)
        figures.append(landing_improvement_magnitude_fig)
    else:
        figures.append(final_pose_fig)

    for fig in figures:
        plt.close(fig)


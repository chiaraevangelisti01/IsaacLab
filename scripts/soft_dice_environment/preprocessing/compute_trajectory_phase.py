#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import numpy as np


HAND_NAMES = [
    "left_hand_link",
    "right_hand_link",
]


def decode_names(names):
    """Convert npz body-name array to normal Python strings."""
    return [
        x.decode() if isinstance(x, bytes) else str(x)
        for x in np.asarray(names).reshape(-1)
    ]


def quaternion_error_deg(q, q_ref):
    """Geodesic quaternion distance to one reference quaternion.

    Works as long as q and q_ref use the same convention.
    The absolute dot product handles q and -q representing
    the same physical orientation.
    """
    q = np.asarray(q, dtype=np.float64)
    q_ref = np.asarray(q_ref, dtype=np.float64)

    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    q_ref = q_ref / np.linalg.norm(q_ref)

    dots = np.abs(q @ q_ref)
    dots = np.clip(dots, 0.0, 1.0)

    return np.degrees(2.0 * np.arccos(dots))


def phase_from_frame(frame, start_frame, num_frames):
    """Normalized phase used by the motion command."""
    denominator = max(num_frames - 1 - start_frame, 1)
    return (frame - start_frame) / denominator


def analyze_trajectory(
    path: Path,
    orientation_threshold_deg: float,
    release_distance_m: float,
    start_frame: int,
):
    with np.load(path, allow_pickle=False) as data:

        required = {
            "fps",
            "body_names",
            "body_pos_w",
            "object_pos_w",
            "object_quat_w",
        }

        missing = required - set(data.files)

        if missing:
            raise ValueError(
                f"{path.name}: missing required fields: {sorted(missing)}"
            )

        fps = float(np.asarray(data["fps"]).reshape(-1)[0])

        body_names = decode_names(data["body_names"])
        body_pos = np.asarray(
            data["body_pos_w"],
            dtype=np.float64,
        )

        object_pos = np.asarray(
            data["object_pos_w"],
            dtype=np.float64,
        )

        object_quat = np.asarray(
            data["object_quat_w"],
            dtype=np.float64,
        )

    num_frames = object_pos.shape[0]

    if not 0 <= start_frame < num_frames:
        raise ValueError(
            f"{path.name}: start_frame={start_frame} outside "
            f"[0, {num_frames - 1}]"
        )

    # ==============================================================
    # 1. LANDING-START PHASE
    #
    # First frame after which the object orientation remains within
    # orientation_threshold_deg of its FINAL demonstrated orientation.
    # ==============================================================

    final_quat = object_quat[-1]

    orientation_error_deg = quaternion_error_deg(
        object_quat,
        final_quat,
    )

    # suffix_max[i] =
    # maximum orientation error occurring from frame i to the end.
    #
    # Therefore suffix_max[i] <= threshold means:
    # "from this point onwards, orientation never leaves the final
    # orientation neighborhood."
    suffix_max_orientation_error = np.maximum.accumulate(
        orientation_error_deg[::-1]
    )[::-1]

    valid_landing_frames = np.flatnonzero(
        (
            suffix_max_orientation_error
            <= orientation_threshold_deg
        )
        &
        (
            np.arange(num_frames) >= start_frame
        )
    )

    # The final frame will always satisfy the condition because its
    # error relative to itself is zero.
    landing_start_frame = int(valid_landing_frames[0])

    # ==============================================================
    # 2. RELEASE PHASE
    #
    # First frame after landing_start where the closest hand remains
    # farther than release_distance_m from the cube for the remainder
    # of the trajectory.
    # ==============================================================

    hand_indices = []

    for hand_name in HAND_NAMES:
        if hand_name not in body_names:
            raise ValueError(
                f"{path.name}: could not find {hand_name!r} "
                f"in body_names."
            )

        hand_indices.append(
            body_names.index(hand_name)
        )

    hand_pos = body_pos[:, hand_indices, :]

    # Shape: [num_frames, 2]
    hand_cube_distance = np.linalg.norm(
        hand_pos - object_pos[:, None, :],
        axis=-1,
    )

    # Distance of closest hand to cube.
    closest_hand_distance = np.min(
        hand_cube_distance,
        axis=1,
    )

    # Sanity check: if neither hand ever gets within the chosen
    # threshold, 0.25 m is not a meaningful release threshold
    # for this trajectory.
    if np.min(closest_hand_distance) > release_distance_m:
        raise ValueError(
            f"{path.name}: hands never enter the "
            f"{release_distance_m:.3f} m release threshold. "
            "Check the trajectory or choose a larger threshold."
        )

    # suffix_min[i] =
    # closest the hands ever get to the object again after frame i.
    #
    # If suffix_min[i] > release_distance_m, they remain outside
    # the release radius for the rest of the trajectory.
    suffix_min_hand_distance = np.minimum.accumulate(
        closest_hand_distance[::-1]
    )[::-1]

    valid_release_frames = np.flatnonzero(
        (
            suffix_min_hand_distance > release_distance_m
        )
        &
        (
            np.arange(num_frames) >= landing_start_frame
        )
    )

    if len(valid_release_frames) == 0:
    
        release_frame = num_frames - 1
        release_found = False
    else:
        release_frame = int(valid_release_frames[0])
        release_found = True

    # ==============================================================
    # Diagnostics
    # ==============================================================

    landing_start_phase = phase_from_frame(
        landing_start_frame,
        start_frame,
        num_frames,
    )

    release_phase = phase_from_frame(
        release_frame,
        start_frame,
        num_frames,
    )

    # How far is the final XY point from XY at release?
    remaining_xy_net = np.linalg.norm(
        object_pos[-1, :2]
        - object_pos[release_frame, :2]
    )

    # Total XY path length after release.
    # This can be larger than remaining_xy_net because mocap may
    # contain small oscillations/jitter.
    if release_frame < num_frames - 1:
        remaining_xy_path = np.sum(
            np.linalg.norm(
                np.diff(
                    object_pos[release_frame:, :2],
                    axis=0,
                ),
                axis=-1,
            )
        )
    else:
        remaining_xy_path = 0.0

    release_xy = object_pos[release_frame, :2]

    remaining_xy_excursion = np.linalg.norm(
        object_pos[release_frame:, :2] - release_xy[None, :],
        axis=-1,
    )

    max_xy_excursion_after_release = np.max(
        remaining_xy_excursion
    )

    return {
        "fps": fps,
        "num_frames": int(num_frames),
        "start_frame": int(start_frame),

        "landing_start_frame": landing_start_frame,
        "landing_start_time_s": (
            landing_start_frame - start_frame
        ) / fps,
        "landing_start_phase": float(
            landing_start_phase
        ),

        "release_frame": release_frame,
        "release_time_s": (
            release_frame - start_frame
        ) / fps,
        "release_phase": float(
            release_phase
        ),
        "release_found": release_found,

        # Convenience field for the reward blending.
        "landing_blend_phase": [
            float(landing_start_phase),
            float(release_phase),
        ],

        # Diagnostics for checking whether release makes sense.
        "orientation_error_at_landing_start_deg": float(
            orientation_error_deg[landing_start_frame]
        ),
        "closest_hand_distance_at_release_m": float(
            closest_hand_distance[release_frame]
        ),
        "minimum_hand_distance_m": float(
            np.min(closest_hand_distance)
        ),
        "remaining_xy_net_after_release_m": float(
            remaining_xy_net
        ),
        "remaining_xy_path_after_release_m": float(
            remaining_xy_path
        ),
        "max_xy_excursion_after_release_m": float(
            max_xy_excursion_after_release
        ),
    }


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--motion_dir",
        type=Path,
        required=True,
        help="Folder containing converted trajectory .npz files.",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("trajectory_phase_metadata.json"),
    )

    parser.add_argument(
        "--pattern",
        type=str,
        default="*.npz",
    )

    parser.add_argument(
        "--orientation_threshold_deg",
        type=float,
        default=15.0,
        help=(
            "Orientation is considered settled once it remains "
            "within this angle of the final orientation."
        ),
    )

    parser.add_argument(
        "--release_distance_m",
        type=float,
        default=0.22,
        help=(
            "Hands are considered released once the closest hand "
            "remains farther than this distance."
        ),
    )

    parser.add_argument(
        "--start_frame",
        type=int,
        default=0,
        help=(
            "Trajectory start frame used by the RL environment."
        ),
    )

    args = parser.parse_args()

    paths = sorted(
        args.motion_dir.glob(args.pattern)
    )

    if not paths:
        raise FileNotFoundError(
            f"No trajectories matching {args.pattern!r} "
            f"in {args.motion_dir}"
        )

    result = {
        "schema_version": 1,

        "settings": {
            "orientation_threshold_deg": (
                args.orientation_threshold_deg
            ),
            "release_distance_m": (
                args.release_distance_m
            ),
            "start_frame": args.start_frame,
        },

        "definitions": {
            "landing_start": (
                "First frame after which object orientation "
                "remains within orientation_threshold_deg "
                "of the final demonstrated orientation."
            ),
            "release": (
                "First frame after landing_start from which "
                "the closest hand remains farther than "
                "release_distance_m for the rest of the motion."
            ),
        },

        "motions": {},
    }

    print()
    print(
        f"{'trajectory':45s} "
        f"{'landing':>9s} "
        f"{'release':>9s} "
        f"{'remaining XY':>16s}"
        f"{'max_xy_excursion':>20s}"
    )
    print("-" * 100)

    for path in paths:

        metadata = analyze_trajectory(
            path=path,
            orientation_threshold_deg=(
                args.orientation_threshold_deg
            ),
            release_distance_m=(
                args.release_distance_m
            ),
            start_frame=args.start_frame,
        )

        # Use the actual file name as the lookup key. This avoids
        # ambiguity between similarly named trajectories.
        result["motions"][path.name] = metadata

        print(
            f"{path.name:45s} "
            f"{metadata['landing_start_phase']:9.3f} "
            f"{metadata['release_phase']:9.3f} "
            f"{100.0 * metadata['remaining_xy_net_after_release_m']:11.2f} cm "
            f"{100.0 * metadata['max_xy_excursion_after_release_m']:11.2f} cm"
        )

    args.output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with open(args.output, "w") as f:
        json.dump(
            result,
            f,
            indent=2,
        )

    print()
    print(f"Wrote metadata to: {args.output}")


if __name__ == "__main__":
    main()
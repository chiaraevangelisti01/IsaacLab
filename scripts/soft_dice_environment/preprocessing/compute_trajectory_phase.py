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

    q = q / np.linalg.norm(
        q,
        axis=-1,
        keepdims=True,
    )
    q_ref = q_ref / np.linalg.norm(q_ref)

    dots = np.abs(q @ q_ref)
    dots = np.clip(dots, 0.0, 1.0)

    return np.degrees(
        2.0 * np.arccos(dots)
    )


def phase_from_frame(
    frame,
    start_frame,
    num_frames,
):
    """Normalized phase used by the motion command."""
    denominator = max(
        num_frames - 1 - start_frame,
        1,
    )

    return (
        frame - start_frame
    ) / denominator


def distance_to_xy_disk(
    positions_xy,
    center_xy,
    radius,
):
    """Distance from XY positions to the valid landing disk.

    Distance is:
        0 inside the disk,
        distance_from_center - radius outside.
    """

    center_distance = np.linalg.norm(
        positions_xy - center_xy[None, :],
        axis=-1,
    )

    return np.maximum(
        center_distance - radius,
        0.0,
    )


def analyze_trajectory(
    path: Path,
    orientation_threshold_deg: float,
    landing_radius_m: float,
    position_region_tolerance_m: float,
    release_distance_m: float,
    start_frame: int,
):
    with np.load(
        path,
        allow_pickle=False,
    ) as data:

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
                f"{path.name}: missing required fields: "
                f"{sorted(missing)}"
            )

        fps = float(
            np.asarray(
                data["fps"]
            ).reshape(-1)[0]
        )

        body_names = decode_names(
            data["body_names"]
        )

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
            f"{path.name}: start_frame={start_frame} "
            f"outside [0, {num_frames - 1}]"
        )

    frame_indices = np.arange(num_frames)

    # ==============================================================
    # 1. ORIENTATION LANDING START
    #
    # First frame from which cube orientation remains within
    # orientation_threshold_deg of its final demonstrated
    # orientation.
    # ==============================================================

    final_quat = object_quat[-1]

    orientation_error_deg = (
        quaternion_error_deg(
            object_quat,
            final_quat,
        )
    )

    # suffix_max[i] =
    # maximum orientation error from frame i to the end.
    suffix_max_orientation_error = (
        np.maximum.accumulate(
            orientation_error_deg[::-1]
        )[::-1]
    )

    valid_orientation_landing_frames = (
        np.flatnonzero(
            (
                suffix_max_orientation_error
                <= orientation_threshold_deg
            )
            &
            (
                frame_indices
                >= start_frame
            )
        )
    )

    # The final frame necessarily has zero orientation error
    # relative to itself.
    orientation_landing_start_frame = int(
        valid_orientation_landing_frames[0]
    )

   
    # ==============================================================
    # 2. POSITION LANDING START
    #
    # The desired landing region is centered on the cube XY
    # position at the beginning of the reference trajectory.
    #
    # We first identify the frame at which the cube reaches its
    # maximum XY excursion from the landing center.
    #
    # We then search ONLY AFTER that frame for the first point from
    # which the cube remains sufficiently close to the landing region
    # for the rest of the trajectory.
    # ==============================================================

    landing_center_xy = (
        object_pos[start_frame, :2].copy()
    )

    center_distance = np.linalg.norm(
        object_pos[:, :2]
        - landing_center_xy[None, :],
        axis=-1,
    )

    # Find the frame at which the cube is farthest from its initial position.
    max_excursion_frame = (
        start_frame
        + int(
            np.argmax(
                center_distance[start_frame:]
            )
        )
    )

    # Distance to the VALID LANDING DISK.

    position_region_distance = (
        distance_to_xy_disk(
            positions_xy=object_pos[:, :2],
            center_xy=landing_center_xy,
            radius=landing_radius_m,
        )
    )

    # suffix_max[i] =
    # largest distance from the valid landing region occurring from
    # frame i until the end.
    #
    # Therefore:
    #
    # suffix_max_position_region_distance[i] <= tolerance
    #
    # means that from frame i onward the cube never again leaves the
    # chosen landing neighborhood.
    suffix_max_position_region_distance = (
        np.maximum.accumulate(
            position_region_distance[::-1]
        )[::-1]
    )

    # Search for landing only AFTER the maximum XY excursion.
    valid_position_landing_frames = (
        np.flatnonzero(
            (
                suffix_max_position_region_distance
                <= position_region_tolerance_m
            )
            &
            (
                frame_indices
                >= max_excursion_frame
            )
        )
    )

    if len(valid_position_landing_frames) == 0:
        final_distance = (
            position_region_distance[-1]
        )

        raise ValueError(
            f"{path.name}: the reference never remains "
            f"within {position_region_tolerance_m:.3f} m "
            f"of the valid landing disk after its maximum "
            f"XY excursion. "
            f"Final distance to region is "
            f"{final_distance:.3f} m."
        )

    position_landing_start_frame = int(
        valid_position_landing_frames[0]
    )

    # ==============================================================
    # 3. RELEASE
    #
    # First frame from which the closest hand remains farther than
    # release_distance_m for the remainder of the trajectory.

    # ==============================================================

    hand_indices = []

    for hand_name in HAND_NAMES:
        if hand_name not in body_names:
            raise ValueError(
                f"{path.name}: could not find "
                f"{hand_name!r} in body_names."
            )

        hand_indices.append(
            body_names.index(hand_name)
        )

    hand_pos = body_pos[
        :,
        hand_indices,
        :,
    ]

    # Shape: [num_frames, 2]
    hand_cube_distance = np.linalg.norm(
        hand_pos
        - object_pos[:, None, :],
        axis=-1,
    )

    closest_hand_distance = np.min(
        hand_cube_distance,
        axis=1,
    )

    if (
        np.min(closest_hand_distance)
        > release_distance_m
    ):
        raise ValueError(
            f"{path.name}: hands never enter the "
            f"{release_distance_m:.3f} m release "
            f"threshold. Check trajectory or threshold."
        )

    # suffix_min[i] =
    # closest either hand ever gets again after frame i.
    suffix_min_hand_distance = (
        np.minimum.accumulate(
            closest_hand_distance[::-1]
        )[::-1]
    )

    valid_release_frames = np.flatnonzero(
        (
            suffix_min_hand_distance
            > release_distance_m
        )
        &
        (
            frame_indices
            >= start_frame
        )
    )

    if len(valid_release_frames) == 0:
        release_frame = (
            num_frames - 1
        )
        release_found = False

    else:
        release_frame = int(
            valid_release_frames[0]
        )
        release_found = True

    # ==============================================================
    # 4. CONSISTENCY CHECKS
    # ==============================================================

    if (
        orientation_landing_start_frame
        > release_frame
    ):
        raise ValueError(
            f"{path.name}: orientation landing starts "
            f"AFTER release "
            f"({orientation_landing_start_frame} > "
            f"{release_frame})."
        )

    if (
        position_landing_start_frame
        > release_frame
    ):
        raise ValueError(
            f"{path.name}: position landing starts "
            f"AFTER release "
            f"({position_landing_start_frame} > "
            f"{release_frame})."
        )

    # ==============================================================
    # 5. NORMALIZED PHASES
    # ==============================================================

    orientation_landing_start_phase = (
        phase_from_frame(
            orientation_landing_start_frame,
            start_frame,
            num_frames,
        )
    )

    position_landing_start_phase = (
        phase_from_frame(
            position_landing_start_frame,
            start_frame,
            num_frames,
        )
    )

    release_phase = phase_from_frame(
        release_frame,
        start_frame,
        num_frames,
    )

    # ==============================================================
    # 6. DIAGNOSTICS
    # ==============================================================

    orientation_blend_duration_s = (
        release_frame
        - orientation_landing_start_frame
    ) / fps

    position_blend_duration_s = (
        release_frame
        - position_landing_start_frame
    ) / fps

    # How far final XY is from XY at release.
    remaining_xy_net = np.linalg.norm(
        object_pos[-1, :2]
        - object_pos[
            release_frame,
            :2,
        ]
    )

    # Total XY path after release.
    if release_frame < num_frames - 1:
        remaining_xy_path = np.sum(
            np.linalg.norm(
                np.diff(
                    object_pos[
                        release_frame:,
                        :2,
                    ],
                    axis=0,
                ),
                axis=-1,
            )
        )
    else:
        remaining_xy_path = 0.0

    release_xy = object_pos[
        release_frame,
        :2,
    ]

    remaining_xy_excursion = (
        np.linalg.norm(
            object_pos[
                release_frame:,
                :2,
            ]
            - release_xy[None, :],
            axis=-1,
        )
    )

    max_xy_excursion_after_release = (
        np.max(
            remaining_xy_excursion
        )
    )

    return {
        "fps": fps,
        "num_frames": int(num_frames),
        "start_frame": int(start_frame),

        # ----------------------------------------------------------
        # Landing-region definition.
        # ----------------------------------------------------------
        "landing_center_xy": [
            float(landing_center_xy[0]),
            float(landing_center_xy[1]),
        ],
        "landing_radius_m": float(
            landing_radius_m
        ),
        "position_region_tolerance_m": float(
            position_region_tolerance_m
        ),

        # ----------------------------------------------------------
        # Orientation landing.
        # ----------------------------------------------------------
        "orientation_landing_start_frame": (
            orientation_landing_start_frame
        ),
        "orientation_landing_start_time_s": (
            orientation_landing_start_frame
            - start_frame
        ) / fps,
        "orientation_landing_start_phase": float(
            orientation_landing_start_phase
        ),

        # ----------------------------------------------------------
        # Position landing.
        # ----------------------------------------------------------
        "position_landing_start_frame": (
            position_landing_start_frame
        ),
        "position_landing_start_time_s": (
            position_landing_start_frame
            - start_frame
        ) / fps,
        "position_landing_start_phase": float(
            position_landing_start_phase
        ),
        "max_xy_excursion_frame": int(
            max_excursion_frame
        ),

        "max_xy_excursion_time_s": float(
            (max_excursion_frame - start_frame)
            / fps
        ),

        "max_xy_excursion_from_landing_center_m": float(
            center_distance[max_excursion_frame]
        ),

        # ----------------------------------------------------------
        # Release.
        # ----------------------------------------------------------
        "release_frame": release_frame,
        "release_time_s": (
            release_frame
            - start_frame
        ) / fps,
        "release_phase": float(
            release_phase
        ),
        "release_found": release_found,

        # ----------------------------------------------------------
        # Blend intervals.
        # ----------------------------------------------------------
        "orientation_blend_phase": [
            float(
                orientation_landing_start_phase
            ),
            float(release_phase),
        ],

        "position_blend_phase": [
            float(
                position_landing_start_phase
            ),
            float(release_phase),
        ],

        "orientation_blend_duration_s": float(
            orientation_blend_duration_s
        ),

        "position_blend_duration_s": float(
            position_blend_duration_s
        ),

        # ----------------------------------------------------------
        # Detector diagnostics.
        # ----------------------------------------------------------
        "orientation_error_at_landing_start_deg": float(
            orientation_error_deg[
                orientation_landing_start_frame
            ]
        ),

        "position_region_distance_at_landing_start_m": float(
            position_region_distance[
                position_landing_start_frame
            ]
        ),

        "max_remaining_position_region_distance_at_landing_start_m": float(
            suffix_max_position_region_distance[
                position_landing_start_frame
            ]
        ),

        "closest_hand_distance_at_release_m": float(
            closest_hand_distance[
                release_frame
            ]
        ),

        "minimum_hand_distance_m": float(
            np.min(
                closest_hand_distance
            )
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
        help=(
            "Folder containing converted trajectory .npz files."
        ),
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path(
            "trajectory_phase_metadata.json"
        ),
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
            "Orientation is considered settled once "
            "it remains within this angular error of "
            "the final demonstrated orientation."
        ),
    )

    parser.add_argument(
        "--landing_radius_m",
        type=float,
        default=0.025,
        help=(
            "Radius of the valid XY landing disk."
        ),
    )

    parser.add_argument(
        "--position_region_tolerance_m",
        type=float,
        default=0.025,
        help=(
            "Maximum allowed distance OUTSIDE the valid "
            "landing disk for position landing detection. "
            "Use 0.0 to require the trajectory to remain "
            "strictly inside the valid disk."
        ),
    )

    parser.add_argument(
        "--release_distance_m",
        type=float,
        default=0.22,
        help=(
            "Hands are considered released once the "
            "closest hand remains farther than this "
            "distance."
        ),
    )

    parser.add_argument(
        "--start_frame",
        type=int,
        default=0,
        help=(
            "Trajectory start frame used by the RL "
            "environment."
        ),
    )

    args = parser.parse_args()

    paths = sorted(
        args.motion_dir.glob(
            args.pattern
        )
    )

    if not paths:
        raise FileNotFoundError(
            f"No trajectories matching "
            f"{args.pattern!r} in "
            f"{args.motion_dir}"
        )

    result = {
        "schema_version": 2,

        "settings": {
            "orientation_threshold_deg": (
                args.orientation_threshold_deg
            ),
            "landing_radius_m": (
                args.landing_radius_m
            ),
            "position_region_tolerance_m": (
                args.position_region_tolerance_m
            ),
            "release_distance_m": (
                args.release_distance_m
            ),
            "start_frame": (
                args.start_frame
            ),
        },

        "definitions": {
            "orientation_landing_start": (
                "First frame from which object "
                "orientation remains within the "
                "orientation threshold of the final "
                "demonstrated orientation."
            ),

            "position_landing_start": (
                "First frame from which the reference "
                "object XY position remains within the "
                "configured tolerance of the valid "
                "landing disk."
            ),

            "release": (
                "First frame from which the closest "
                "hand remains farther than the release "
                "distance for the rest of the motion."
            ),
        },

        "motions": {},
    }

    print()
    print(
        f"{'trajectory':45s} "
        f"{'ori land':>9s} "
        f"{'pos land':>9s} "
        f"{'release':>9s} "
        f"{'ori->rel':>10s} "
        f"{'pos->rel':>10s}"
    )

    print("-" * 100)

    for path in paths:

        metadata = analyze_trajectory(
            path=path,
            orientation_threshold_deg=(
                args.orientation_threshold_deg
            ),
            landing_radius_m=(
                args.landing_radius_m
            ),
            position_region_tolerance_m=(
                args.position_region_tolerance_m
            ),
            release_distance_m=(
                args.release_distance_m
            ),
            start_frame=(
                args.start_frame
            ),
        )

        result["motions"][
            path.name
        ] = metadata

        print(
            f"{path.name:45s} "
            f"{metadata['orientation_landing_start_phase']:9.3f} "
            f"{metadata['position_landing_start_phase']:9.3f} "
            f"{metadata['release_phase']:9.3f} "
            f"{metadata['orientation_blend_duration_s']:8.2f}s "
            f"{metadata['position_blend_duration_s']:8.2f}s"
        )

    args.output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with open(
        args.output,
        "w",
    ) as f:
        json.dump(
            result,
            f,
            indent=2,
        )

    print()
    print(
        f"Wrote metadata to: "
        f"{args.output}"
    )


if __name__ == "__main__":
    main()
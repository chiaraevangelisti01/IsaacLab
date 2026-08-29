from __future__ import annotations

import pickle
from pathlib import Path
import json

import numpy as np
import torch

from isaaclab_tasks import ISAACLAB_TASKS_EXT_DIR


HOLOSOMA_H1_JOINT_NAMES = [
    "left_hip_yaw_joint",
    "left_hip_roll_joint",
    "left_hip_pitch_joint",
    "left_knee_joint",
    "left_ankle_joint",
    "right_hip_yaw_joint",
    "right_hip_roll_joint",
    "right_hip_pitch_joint",
    "right_knee_joint",
    "right_ankle_joint",
    "torso_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
]

HOLOSOMA_TO_ISAAC_INDICES = [0, 5, 10, 1, 6, 11, 15, 2, 7, 12, 16, 3, 8, 13, 17, 4, 9, 14, 18]

H1_TRACKED_BODY_NAMES = [
    "torso_link",
    "left_shoulder_roll_link",
    "left_elbow_link",
    "right_shoulder_roll_link",
    "right_elbow_link",
]

H1_HAND_REFERENCE_NAMES = [
    "left_hand_link",
    "right_hand_link",
]

H1_HAND_PARENT_BODY_NAMES = [
    "left_elbow_link",
    "right_elbow_link",
]


H1_HAND_OFFSETS_B = [
    [0.28, 0.0005, -0.0185],
    [0.28, -0.0005, -0.0185],
]

_REPO_ROOT = Path(ISAACLAB_TASKS_EXT_DIR).resolve().parents[1]
_MODELS_PATH = _REPO_ROOT / "scripts" / "soft_dice_environment" / "models"
CUSTOM_DICE_DEFORMABLE_USD = str(_MODELS_PATH / "dice_superquadric_further_reduced7.usd")
CUSTOM_DICE_SCALE = (1.0, 1.0, 1.0)


class NumpyCompatUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module.startswith("numpy._core"):
            module = module.replace("numpy._core", "numpy.core")
        return super().find_class(module, name)


def validate_asset_paths() -> None:
    path = Path(CUSTOM_DICE_DEFORMABLE_USD)
    if not path.is_file():
        raise FileNotFoundError(f"Required deformable dice asset does not exist: {path}")

def load_body_reference(data: dict, num_frames: int):
    """Load Cartesian body reference from converted Holosoma data."""

    required_keys = {
        "body_names",
        "body_pos_w",
        "body_quat_w",
    }

    present_keys = required_keys.intersection(data.keys())

    if not present_keys:
        return None, None, None

    if present_keys != required_keys:
        missing = required_keys - present_keys
        raise ValueError(
            f"Incomplete Cartesian body reference. Missing: {missing}"
        )

    body_names = [
        name.decode() if isinstance(name, bytes) else str(name)
        for name in np.asarray(data["body_names"]).reshape(-1)
    ]

    body_pos_w = np.asarray(
        data["body_pos_w"],
        dtype=np.float32,
    )

    body_quat_w = np.asarray(
        data["body_quat_w"],
        dtype=np.float32,
    )

    expected_pos_shape = (
        num_frames,
        len(body_names),
        3,
    )

    expected_quat_shape = (
        num_frames,
        len(body_names),
        4,
    )

    if body_pos_w.shape != expected_pos_shape:
        raise ValueError(
            f"Expected body_pos_w shape {expected_pos_shape}, "
            f"got {body_pos_w.shape}"
        )

    if body_quat_w.shape != expected_quat_shape:
        raise ValueError(
            f"Expected body_quat_w shape {expected_quat_shape}, "
            f"got {body_quat_w.shape}"
        )

    if not np.isfinite(body_pos_w).all():
        raise ValueError("body_pos_w contains non-finite values.")

    if not np.isfinite(body_quat_w).all():
        raise ValueError("body_quat_w contains non-finite values.")

    return body_names, body_pos_w, body_quat_w

def load_motion_file(path: str):
    """Load either raw Holosoma qpos or converted Holosoma RL motion."""

    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(f"Motion file does not exist: {path}")

    if path.suffix == ".npz":
        raw = np.load(path, allow_pickle=False)
        data = {k: raw[k] for k in raw.files}

    elif path.suffix == ".pkl":
        with open(path, "rb") as f:
            data = NumpyCompatUnpickler(f).load()

    else:
        raise ValueError(
            f"Unsupported motion extension: {path.suffix}"
        )

    fps = float(
        np.asarray(data.get("fps", 30.0)).reshape(-1)[0]
    )

    if fps <= 0.0:
        raise ValueError(f"Invalid motion fps: {fps}")

    body_names = None
    body_pos_w = None
    body_quat_w = None
    robot_joint_qvel = None
    # ------------------------------------------------------------------
    # Converted Holosoma RL format
    # ------------------------------------------------------------------
    if "joint_pos" in data:

        joint_pos = np.asarray(
            data["joint_pos"],
            dtype=np.float32,
        )

        if (
            joint_pos.ndim != 2
            or not np.isfinite(joint_pos).all()
        ):
            raise ValueError(
                f"Bad joint_pos shape or values: {joint_pos.shape}"
            )

        # H1 converted format:
        # joint_pos =
        # [root position (3),
        #  root quaternion WXYZ (4),
        #  H1 joints (19)]
      
        if joint_pos.shape[1] != 26:
            raise ValueError(
                "Expected converted H1 joint_pos with "
                f"26 columns, got {joint_pos.shape[1]}"
            )

        root_qpos = joint_pos[:, 0:7]
        robot_joint_qpos = joint_pos[:, 7:26]

        (
            body_names,
            body_pos_w,
            body_quat_w,
        ) = load_body_reference(
            data=data,
            num_frames=joint_pos.shape[0],
        )

        if "joint_vel" in data:
            joint_vel = np.asarray(
                data["joint_vel"],
                dtype=np.float32,
            )

            if joint_vel.shape != (joint_pos.shape[0], 25):
                raise ValueError(
                    f"Expected converted H1 joint_vel with shape "
                    f"({joint_pos.shape[0]}, 25), got {joint_vel.shape}"
                )

            if not np.isfinite(joint_vel).all():
                raise ValueError("joint_vel contains non-finite values.")

            # Holosoma joint_vel:
            # [root linear vel (3),
            #  root angular vel (3),
            #  H1 joint velocities (19)]
            robot_joint_qvel = joint_vel[:, 6:25]

        else:
            robot_joint_qvel = None

        # Converted Holosoma stores the object separately.
        has_object_pos = "object_pos_w" in data
        has_object_quat = "object_quat_w" in data

        if has_object_pos != has_object_quat:
            raise ValueError(
                "Converted motion must contain both "
                "'object_pos_w' and 'object_quat_w', or neither."
            )

        if has_object_pos:
            object_pos = np.asarray(
                data["object_pos_w"],
                dtype=np.float32,
            )

            object_quat = np.asarray(
                data["object_quat_w"],
                dtype=np.float32,
            )

            if object_pos.shape != (joint_pos.shape[0], 3):
                raise ValueError(
                    f"Unexpected object_pos_w shape: "
                    f"{object_pos.shape}"
                )

            if object_quat.shape != (joint_pos.shape[0], 4):
                raise ValueError(
                    f"Unexpected object_quat_w shape: "
                    f"{object_quat.shape}"
                )

            if (
                not np.isfinite(object_pos).all()
                or not np.isfinite(object_quat).all()
            ):
                raise ValueError(
                    "Object trajectory contains non-finite values."
                )

            # Reconstruct the format expected by desired_cube_pose_from_holosoma: [x, y, z, qw, qx, qy, qz]
            object_qpos = np.concatenate(
                (object_pos, object_quat),
                axis=-1,
            )

        else:
            object_qpos = None

    

    # ------------------------------------------------------------------
    # Original/raw Holosoma retargeting format
    # ------------------------------------------------------------------
    elif "qpos" in data:

        qpos = np.asarray(
            data["qpos"],
            dtype=np.float32,
        )

        
        if (
            qpos.ndim != 2
            or not np.isfinite(qpos).all()
        ):
            raise ValueError(
                f"Bad qpos shape or values: {qpos.shape}"
            )

        if qpos.shape[1] == 33:
            root_qpos = qpos[:, 0:7]
            robot_joint_qpos = qpos[:, 7:26]
            object_qpos = qpos[:, 26:33]

        elif qpos.shape[1] == 26:
            root_qpos = qpos[:, 0:7]
            robot_joint_qpos = qpos[:, 7:26]
            object_qpos = None

        elif qpos.shape[1] == 19:
            root_qpos = None
            robot_joint_qpos = qpos
            object_qpos = None

        else:
            raise ValueError(
                f"Unexpected qpos dimension: {qpos.shape[1]}"
            )

    else:
        raise KeyError(
            "Expected either 'qpos' (raw Holosoma) "
            "or 'joint_pos' (converted Holosoma). "
            f"Found: {list(data.keys())}"
        )

    if robot_joint_qpos.shape[1] != 19:
        raise ValueError(
            f"Expected 19 H1 joints, "
            f"got {robot_joint_qpos.shape[1]}"
        )

    return (
        robot_joint_qpos,
        robot_joint_qvel,
        fps,
        root_qpos,
        object_qpos,
        body_names,
        body_pos_w,
        body_quat_w,
    )

def load_tracking_motion_file(path: str):
    """Load a fully converted motion suitable for RL tracking."""

    path = Path(path)

    if path.suffix != ".npz":
        raise ValueError(f"Tracking motions must be converted .npz files: {path}")

    required_keys = {
        "fps",
        "joint_pos",
        "joint_vel",
        "body_names",
        "body_pos_w",
        "body_quat_w",
        "body_lin_vel_w",
        "body_ang_vel_w",
        "object_pos_w",
        "object_quat_w",
    }

    with np.load(path, allow_pickle=False) as data:
        missing = required_keys - set(data.files)
        if missing:
            raise ValueError(
                f"Tracking motion {path} is not a complete converted trajectory. "
                f"Missing fields: {sorted(missing)}"
            )

        body_lin_vel_w = np.asarray(data["body_lin_vel_w"], dtype=np.float32)
        body_ang_vel_w = np.asarray(data["body_ang_vel_w"], dtype=np.float32)

    (
        robot_joint_qpos,
        robot_joint_qvel,
        fps,
        root_qpos,
        object_qpos,
        body_names,
        body_pos_w,
        body_quat_w,
    ) = load_motion_file(str(path))

    if robot_joint_qvel is None:
        raise ValueError(f"Tracking motion has no joint velocities: {path}")

    if root_qpos is None or object_qpos is None:
        raise ValueError(f"Tracking motion has no root/object reference: {path}")

    if body_names is None or body_pos_w is None or body_quat_w is None:
        raise ValueError(f"Tracking motion has no Cartesian body reference: {path}")

    expected_velocity_shape = (robot_joint_qpos.shape[0], len(body_names), 3)

    if body_lin_vel_w.shape != expected_velocity_shape:
        raise ValueError(
            f"Expected body_lin_vel_w shape {expected_velocity_shape}, "
            f"got {body_lin_vel_w.shape} in {path}"
        )

    if body_ang_vel_w.shape != expected_velocity_shape:
        raise ValueError(
            f"Expected body_ang_vel_w shape {expected_velocity_shape}, "
            f"got {body_ang_vel_w.shape} in {path}"
        )

    if not np.isfinite(body_lin_vel_w).all():
        raise ValueError(f"body_lin_vel_w contains non-finite values: {path}")

    if not np.isfinite(body_ang_vel_w).all():
        raise ValueError(f"body_ang_vel_w contains non-finite values: {path}")

    return (
        robot_joint_qpos,
        robot_joint_qvel,
        fps,
        root_qpos,
        object_qpos,
        body_names,
        body_pos_w,
        body_quat_w,
        body_lin_vel_w,
        body_ang_vel_w,
    )

def _quat_norm_wxyz(q):
    q = np.asarray(q, dtype=np.float64)
    return q / np.linalg.norm(q)


def _quat_wxyz_to_rotmat(q_wxyz):
    q = _quat_norm_wxyz(q_wxyz)
    w, x, y, z = q
    return np.array(
        [
            [1.0 - 2.0*y*y - 2.0*z*z, 2.0*x*y - 2.0*z*w, 2.0*x*z + 2.0*y*w],
            [2.0*x*y + 2.0*z*w, 1.0 - 2.0*x*x - 2.0*z*z, 2.0*y*z - 2.0*x*w],
            [2.0*x*z - 2.0*y*w, 2.0*y*z + 2.0*x*w, 1.0 - 2.0*x*x - 2.0*y*y],
        ],
        dtype=np.float64,
    )


def _quat_xyzw_to_rotmat(q_xyzw):
    q = np.asarray(q_xyzw, dtype=np.float64)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array(
        [
            [1.0 - 2.0*y*y - 2.0*z*z, 2.0*x*y - 2.0*z*w, 2.0*x*z + 2.0*y*w],
            [2.0*x*y + 2.0*z*w, 1.0 - 2.0*x*x - 2.0*z*z, 2.0*y*z - 2.0*x*w],
            [2.0*x*z - 2.0*y*w, 2.0*y*z + 2.0*x*w, 1.0 - 2.0*x*x - 2.0*y*y],
        ],
        dtype=np.float64,
    )


def _rotmat_to_quat_xyzw(R):
    R = np.asarray(R, dtype=np.float64)
    trace = np.trace(R)

    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.asarray([x, y, z, w], dtype=np.float32)
    q /= np.linalg.norm(q)
    return q


def desired_cube_pose_from_holosoma(
    root_qpos_np,
    object_qpos_np,
    frame: int,
    isaac_robot_pos_np,
    isaac_robot_quat_xyzw_np,
):
    """Same frame-0 root-alignment conversion validated by the replay environment."""
    frame = min(frame, root_qpos_np.shape[0] - 1, object_qpos_np.shape[0] - 1)

    root_pos = np.asarray(root_qpos_np[frame, 0:3], dtype=np.float32)
    root_quat = _quat_norm_wxyz(root_qpos_np[frame, 3:7]).astype(np.float32)
    dice_pos = np.asarray(object_qpos_np[frame, 0:3], dtype=np.float32)
    dice_quat = _quat_norm_wxyz(object_qpos_np[frame, 3:7]).astype(np.float32)

    root_pos_0 = np.asarray(root_qpos_np[0, 0:3], dtype=np.float32)
    root_quat_0 = _quat_norm_wxyz(root_qpos_np[0, 3:7]).astype(np.float32)
    dice_pos_0 = np.asarray(object_qpos_np[0, 0:3], dtype=np.float32)

    R_holo_root_0 = _quat_wxyz_to_rotmat(root_quat_0)
    R_isaac_root = _quat_xyzw_to_rotmat(isaac_robot_quat_xyzw_np)
    R_align = R_isaac_root @ R_holo_root_0.T

    dice_rel_world_0 = dice_pos_0 - root_pos_0
    initial_cube_pos = np.asarray(isaac_robot_pos_np, dtype=np.float32) + (
        R_align @ dice_rel_world_0
    ).astype(np.float32)

    dice_world_delta = dice_pos - dice_pos_0
    pos = initial_cube_pos + (R_align @ dice_world_delta).astype(np.float32)

    R_holo_dice = _quat_wxyz_to_rotmat(dice_quat)
    quat_xyzw = _rotmat_to_quat_xyzw(R_align @ R_holo_dice)

    # Kept only for compatibility/debugging with the old helper.
    R_holo_root = _quat_wxyz_to_rotmat(root_quat)
    dice_rel_root_local = R_holo_root.T @ (dice_pos - root_pos)

    return pos, quat_xyzw, dice_rel_root_local.astype(np.float32), root_pos, dice_pos

def select_body_values(
    body_names: list[str],
    values: np.ndarray,
    tracked_body_names: list[str],
) -> np.ndarray:
    """Select body-indexed values in a requested body-name order."""

    missing = [name for name in tracked_body_names if name not in body_names]
    if missing:
        raise ValueError(f"Missing required body references: {missing}")

    body_indices = [body_names.index(name) for name in tracked_body_names]
    return values[:, body_indices, ...]

def select_body_reference(
    body_names: list[str],
    body_pos_w: np.ndarray,
    body_quat_w: np.ndarray,
    tracked_body_names: list[str],
):
    """Select Cartesian trajectories for requested bodies."""

    return (
        select_body_values(body_names, body_pos_w, tracked_body_names),
        select_body_values(body_names, body_quat_w, tracked_body_names),
    )


def transform_body_reference_to_fixed_root(
    body_pos_w: np.ndarray,
    body_quat_w: np.ndarray,
    root_qpos: np.ndarray,
    fixed_root_pos: np.ndarray,
    fixed_root_quat_xyzw: np.ndarray,
):
    """Express floating-root Holosoma body poses under a fixed Isaac root."""

    num_frames = body_pos_w.shape[0]
    num_bodies = body_pos_w.shape[1]

    out_pos = np.empty_like(
        body_pos_w,
        dtype=np.float32,
    )

    out_quat = np.empty(
        (num_frames, num_bodies, 4),
        dtype=np.float32,
    )

    fixed_root_pos = np.asarray(
        fixed_root_pos,
        dtype=np.float32,
    )

    R_fixed_root = _quat_xyzw_to_rotmat(
        fixed_root_quat_xyzw
    )

    for frame in range(num_frames):

        root_pos = root_qpos[frame, :3]

        R_root = _quat_wxyz_to_rotmat(
            root_qpos[frame, 3:7]
        )

        for body in range(num_bodies):

            # Position of body expressed relative to floating root.
            body_pos_root = (
                R_root.T
                @ (
                    body_pos_w[frame, body]
                    - root_pos
                )
            )

            # Orientation of body relative to floating root.
            R_body = _quat_wxyz_to_rotmat(
                body_quat_w[frame, body]
            )

            R_body_root = (
                R_root.T
                @ R_body
            )

            # Put the root-relative pose under the fixed Isaac root.
            out_pos[frame, body] = (
                fixed_root_pos
                + R_fixed_root @ body_pos_root
            )

            out_quat[frame, body] = (
                _rotmat_to_quat_xyzw(
                    R_fixed_root @ R_body_root
                )
            )

    return out_pos, out_quat

def transform_vector_reference_to_fixed_root(
    vector_w: np.ndarray,
    root_qpos: np.ndarray,
    fixed_root_quat_xyzw: np.ndarray,
) -> np.ndarray:
    """Rotate world-frame vectors from the Holosoma root alignment to the fixed Isaac root."""

    out = np.empty_like(vector_w, dtype=np.float32)
    R_fixed_root = _quat_xyzw_to_rotmat(fixed_root_quat_xyzw)

    for frame in range(vector_w.shape[0]):
        R_root = _quat_wxyz_to_rotmat(root_qpos[frame, 3:7])
        R_align = R_fixed_root @ R_root.T
        out[frame] = vector_w[frame] @ R_align.T

    return out

def resolve_motion_paths(
    motion_file: str = "",
    motion_dir: str = "",
    motion_pattern: str = "*.npz",
) -> list[Path]:
    """Resolve either one motion file or a sorted directory-backed motion pool."""

    if bool(motion_file) == bool(motion_dir):
        raise ValueError("Configure exactly one of motion_file or motion_dir.")

    if motion_file:
        path = Path(motion_file).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Motion file does not exist: {path}")
        return [path]

    motion_dir_path = Path(motion_dir).expanduser().resolve()
    if not motion_dir_path.is_dir():
        raise FileNotFoundError(f"Motion directory does not exist: {motion_dir_path}")

    paths = sorted(path for path in motion_dir_path.glob(motion_pattern) if path.is_file())

    if not paths:
        raise FileNotFoundError(
            f"No motion files matching {motion_pattern!r} found in {motion_dir_path}"
        )

    return paths

def load_trajectory_phase_metadata(
    metadata_file: str,
    motion_paths: list[Path],
    motion_lengths: torch.Tensor,
    start_frames: torch.Tensor,
    device: str,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Load and validate landing/release phases for a motion pool."""

    metadata_path = Path(
        metadata_file
    ).expanduser()

    if not metadata_path.is_file():
        raise FileNotFoundError(
            f"Trajectory phase metadata file does not exist: "
            f"{metadata_path}"
        )

    with metadata_path.open("r") as f:
        metadata = json.load(f)

    if "motions" not in metadata:
        raise ValueError(
            f"{metadata_path} does not contain a 'motions' dictionary."
        )

    motion_metadata = metadata["motions"]

    landing_start_phases = []
    release_phases = []

    for motion_id, motion_path in enumerate(
        motion_paths
    ):
        key = motion_path.name

        if key not in motion_metadata:
            raise KeyError(
                f"No phase metadata found for "
                f"trajectory '{key}' in {metadata_path}."
            )

        entry = motion_metadata[key]

        metadata_num_frames = int(
            entry["num_frames"]
        )

        actual_num_frames = int(
            motion_lengths[motion_id].item()
        )

        if metadata_num_frames != actual_num_frames:
            raise ValueError(
                f"{key}: metadata says "
                f"{metadata_num_frames} frames, "
                f"but loaded trajectory has "
                f"{actual_num_frames}."
            )

        metadata_start_frame = int(
            entry.get("start_frame", 0)
        )

        actual_start_frame = int(
            start_frames[motion_id].item()
        )

        if metadata_start_frame != actual_start_frame:
            raise ValueError(
                f"{key}: phase metadata was computed with "
                f"start_frame={metadata_start_frame}, "
                f"but environment uses "
                f"start_frame={actual_start_frame}."
            )

        if not bool(
            entry.get("release_found", False)
        ):
            raise ValueError(
                f"{key}: preprocessing did not "
                "successfully detect a release."
            )

        landing_start = float(
            entry["landing_start_phase"]
        )

        release = float(
            entry["release_phase"]
        )

        if not (
            0.0
            <= landing_start
            <= release
            <= 1.0
        ):
            raise ValueError(
                f"{key}: invalid phase interval: "
                f"landing_start={landing_start}, "
                f"release={release}."
            )

        landing_start_phases.append(
            landing_start
        )

        release_phases.append(
            release
        )

    return (
        torch.tensor(
            landing_start_phases,
            dtype=torch.float32,
            device=device,
        ),
        torch.tensor(
            release_phases,
            dtype=torch.float32,
            device=device,
        ),
    )
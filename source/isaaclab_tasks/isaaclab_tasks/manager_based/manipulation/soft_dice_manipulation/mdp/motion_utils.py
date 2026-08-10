from __future__ import annotations

import pickle
from pathlib import Path

import numpy as np

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

_REPO_ROOT = Path(ISAACLAB_TASKS_EXT_DIR).resolve().parents[1]
_MODELS_PATH = _REPO_ROOT / "scripts" / "soft_dice_environment" / "models"
CUSTOM_DICE_DEFORMABLE_USD = str(_MODELS_PATH / "dice_superquadric_deformable_two_meshes.usd")
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

def _resample_linear(
    values: np.ndarray,
    source_fps: float,
    target_times: np.ndarray,
) -> np.ndarray:
    """Linearly resample uniformly sampled vector data."""

    source_coord = target_times * float(source_fps)

    idx0 = np.floor(source_coord).astype(np.int64)
    idx0 = np.clip(idx0, 0, values.shape[0] - 1)

    idx1 = np.minimum(idx0 + 1, values.shape[0] - 1)

    alpha = (source_coord - idx0).astype(np.float32)[:, None]

    return (
        (1.0 - alpha) * values[idx0]
        + alpha * values[idx1]
    ).astype(np.float32)


def _resample_quat_wxyz(
    quats: np.ndarray,
    source_fps: float,
    target_times: np.ndarray,
) -> np.ndarray:
    """Resample WXYZ quaternions using shortest-path SLERP."""

    q = np.asarray(quats, dtype=np.float64)

    norms = np.linalg.norm(q, axis=-1, keepdims=True)
    if np.any(norms < 1.0e-8):
        raise ValueError("Cannot interpolate zero-norm quaternion.")

    q = q / norms

    source_coord = target_times * float(source_fps)

    idx0 = np.floor(source_coord).astype(np.int64)
    idx0 = np.clip(idx0, 0, q.shape[0] - 1)

    idx1 = np.minimum(idx0 + 1, q.shape[0] - 1)

    alpha = (source_coord - idx0)[:, None]

    q0 = q[idx0]
    q1 = q[idx1].copy()

    # q and -q represent the same orientation.
    # Flip q1 when necessary so SLERP follows the shortest path.
    dot = np.sum(q0 * q1, axis=-1)

    flip = dot < 0.0
    q1[flip] *= -1.0
    dot = np.abs(dot)

    dot = np.clip(dot, -1.0, 1.0)

    result = np.empty_like(q0)

    # For very small rotations, normalized linear interpolation
    # is numerically more stable than SLERP.
    close = dot > 0.9995

    if np.any(close):
        result[close] = (
            (1.0 - alpha[close]) * q0[close]
            + alpha[close] * q1[close]
        )

    far = ~close

    if np.any(far):
        theta = np.arccos(dot[far])[:, None]
        sin_theta = np.sin(theta)

        w0 = np.sin((1.0 - alpha[far]) * theta) / sin_theta
        w1 = np.sin(alpha[far] * theta) / sin_theta

        result[far] = w0 * q0[far] + w1 * q1[far]

    result /= np.linalg.norm(result, axis=-1, keepdims=True)

    return result.astype(np.float32)


def resample_motion_to_fps(
    joint_qpos: np.ndarray,
    source_fps: float,
    target_fps: float,
    root_qpos: np.ndarray | None = None,
    object_qpos: np.ndarray | None = None,
):
    """Resample a Holosoma motion onto a fixed target time grid.

    Joint positions and Cartesian positions use linear interpolation.
    Root/object orientations use quaternion SLERP.
    """

    if source_fps <= 0.0:
        raise ValueError("source_fps must be positive.")

    if target_fps <= 0.0:
        raise ValueError("target_fps must be positive.")

    num_source_frames = joint_qpos.shape[0]

    if num_source_frames == 0:
        raise ValueError("Motion contains no frames.")

    if root_qpos is not None and root_qpos.shape[0] != num_source_frames:
        raise ValueError("root_qpos length does not match joint trajectory.")

    if object_qpos is not None and object_qpos.shape[0] != num_source_frames:
        raise ValueError("object_qpos length does not match joint trajectory.")

    if num_source_frames == 1:
        return (
            joint_qpos.copy(),
            None if root_qpos is None else root_qpos.copy(),
            None if object_qpos is None else object_qpos.copy(),
        )

    duration_s = (num_source_frames - 1) / float(source_fps)

    # Fixed target-rate grid:
    # 0, 1/target_fps, 2/target_fps, ...
    #
    # We deliberately do not append a shorter final interval.
    num_target_frames = (
        int(np.floor(duration_s * target_fps + 1.0e-9)) + 1
    )

    target_times = (
        np.arange(num_target_frames, dtype=np.float64)
        / float(target_fps)
    )

    joint_qpos_resampled = _resample_linear(
        np.asarray(joint_qpos, dtype=np.float32),
        source_fps,
        target_times,
    )

    def resample_pose(qpos):
        if qpos is None:
            return None

        pos = _resample_linear(
            qpos[:, 0:3],
            source_fps,
            target_times,
        )

        quat = _resample_quat_wxyz(
            qpos[:, 3:7],
            source_fps,
            target_times,
        )

        return np.concatenate((pos, quat), axis=-1).astype(np.float32)

    root_qpos_resampled = resample_pose(root_qpos)
    object_qpos_resampled = resample_pose(object_qpos)

    return (
        joint_qpos_resampled,
        root_qpos_resampled,
        object_qpos_resampled,
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

def select_body_reference(
    body_names: list[str],
    body_pos_w: np.ndarray,
    body_quat_w: np.ndarray,
    tracked_body_names: list[str],
):
    """Select Cartesian trajectories for requested bodies."""

    body_indices = []

    for name in tracked_body_names:
        if name not in body_names:
            raise ValueError(
                f"Body '{name}' not found in Holosoma trajectory. "
                f"Available bodies: {body_names}"
            )

        body_indices.append(
            body_names.index(name)
        )

    return (
        body_pos_w[:, body_indices, :],
        body_quat_w[:, body_indices, :],
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
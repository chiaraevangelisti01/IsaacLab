from __future__ import annotations

import pickle
from pathlib import Path

import numpy as np
import torch
import warp as wp

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from isaaclab.utils.math import quat_apply
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

HOLOSOMA_H1_JOINT_NAMES = [
    "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint", "left_knee_joint", "left_ankle_joint",
    "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint", "right_knee_joint", "right_ankle_joint",
    "torso_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
]
HOLOSOMA_TO_ISAAC_INDICES = [0, 5, 10, 1, 6, 11, 15, 2, 7, 12, 16, 3, 8, 13, 17, 4, 9, 14, 18]
FAKE_EE_OFFSET_IN_ELBOW = [0.28, 0.0, -0.0185]
REFERENCE_ROBOT_WORLD_OFFSET = np.array([0.0, 1.5, 0.0], dtype=np.float32)
CUSTOM_DICE_SCALE = (1.0, 1.0, 1.0)

# This file is expected at: <repo>/scripts/soft_dice_environment/replay_utils.py
MODELS_PATH = Path(__file__).resolve().parent/"models"
CUSTOM_DICE_DEFORMABLE_USD = str(MODELS_PATH / "dice_superquadric_deformable_two_meshes.usd")
CUSTOM_DICE_RIGID_REFERENCE_USD = str(MODELS_PATH / "dice_superquadric_rigid.usd")


def validate_asset_paths():
    for path in (CUSTOM_DICE_DEFORMABLE_USD, CUSTOM_DICE_RIGID_REFERENCE_USD):
        if not Path(path).is_file():
            raise FileNotFoundError(f"Required replay asset does not exist: {path}")


def get_upper_body_isaac_joint_ids():
    upper_holo_start = HOLOSOMA_H1_JOINT_NAMES.index("torso_joint")
    upper_isaac_ids, upper_holo_names = [], []
    for isaac_id, holo_id in enumerate(HOLOSOMA_TO_ISAAC_INDICES):
        if holo_id >= upper_holo_start:
            upper_isaac_ids.append(isaac_id)
            upper_holo_names.append(HOLOSOMA_H1_JOINT_NAMES[holo_id])
    return upper_isaac_ids, upper_holo_names


class NumpyCompatUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module.startswith("numpy._core"):
            module = module.replace("numpy._core", "numpy.core")
        return super().find_class(module, name)

def write_joint_state_to_sim_index(robot, joint_pos: torch.Tensor, joint_vel: torch.Tensor):
    robot.write_joint_position_to_sim_index(position=joint_pos)
    robot.write_joint_velocity_to_sim_index(velocity=joint_vel)


def set_joint_position_target_index(robot, target: torch.Tensor, joint_ids=None):
    if joint_ids is not None:
        joint_ids = torch.as_tensor(joint_ids, device=robot.device, dtype=torch.int32)
    robot.set_joint_position_target_index(target=target, joint_ids=joint_ids)


def write_root_state_to_sim_index(robot, root_state_xyzw: torch.Tensor):
    """Write articulation root state.

    root_state_xyzw shape: (num_envs, 13)
        xyz + qx qy qz qw + linear/angular velocity

    In IsaacLab 3.x, quaternions passed to IsaacLab APIs are XYZW.
    This function assumes root_state_xyzw already follows that convention.
    """
    root_pose_xyzw = root_state_xyzw[:, :7]
    root_vel = root_state_xyzw[:, 7:]

    if hasattr(robot, "write_root_pose_to_sim_index"):
        try:
            robot.write_root_pose_to_sim_index(root_pose=root_pose_xyzw)
        except TypeError:
            robot.write_root_pose_to_sim_index(root_pose_xyzw)

        try:
            robot.write_root_velocity_to_sim_index(root_velocity=root_vel)
        except TypeError:
            robot.write_root_velocity_to_sim_index(root_vel)
    else:
        robot.write_root_pose_to_sim(root_pose_xyzw)
        robot.write_root_velocity_to_sim(root_vel)


def write_cube_pose_to_sim(cube, pose_xyzw: torch.Tensor):
    """Write cube root pose.

    pose_xyzw shape: (num_envs, 7) = xyz + qx qy qz qw.

    IsaacLab 3.x uses XYZW quaternion order at the IsaacLab API/data boundary.
    Holosoma/MuJoCo source quaternions must be converted before calling this.
    """
    if hasattr(cube, "write_root_pose_to_sim"):
        cube.write_root_pose_to_sim(pose_xyzw)
    elif hasattr(cube, "write_root_pose_to_sim_index"):
        cube.write_root_pose_to_sim_index(root_pose=pose_xyzw)
    else:
        raise AttributeError("Could not find cube.write_root_pose_to_sim API")

    zero_vel = torch.zeros((pose_xyzw.shape[0], 6), dtype=pose_xyzw.dtype, device=pose_xyzw.device)

    if hasattr(cube, "write_root_velocity_to_sim"):
        cube.write_root_velocity_to_sim(zero_vel)
    elif hasattr(cube, "write_root_velocity_to_sim_index"):
        cube.write_root_velocity_to_sim_index(root_velocity=zero_vel)


def get_body_link_pos_quat_world_xyzw(robot):
    """Return body positions and quaternions in world frame.

    Important:
        The suffix '_w' in IsaacLab data names means world frame, not WXYZ.
        In IsaacLab 3.x, returned quaternions should be treated as XYZW.
    """
    if hasattr(robot.data, "body_link_pos_w") and hasattr(robot.data, "body_link_quat_w"):
        return robot.data.body_link_pos_w, robot.data.body_link_quat_w

    if hasattr(robot.data, "body_pos_w") and hasattr(robot.data, "body_quat_w"):
        return robot.data.body_pos_w, robot.data.body_quat_w

    if hasattr(robot.data, "body_link_pose_w"):
        pose = robot.data.body_link_pose_w
        return pose[..., 0:3], pose[..., 3:7]

    if hasattr(robot.data, "body_state_w"):
        state = robot.data.body_state_w
        return state[..., 0:3], state[..., 3:7]

    raise AttributeError("Could not find body pose tensors on robot.data")


def get_cube_root_pos_quat_world_xyzw(cube):
    """Return cube root position and quaternion in world frame.

    In IsaacLab 3.x, returned quaternions should be treated as XYZW.
    """
    data = cube.data

    if hasattr(data, "root_pos_w") and hasattr(data, "root_quat_w"):
        return data.root_pos_w[0].detach().clone(), data.root_quat_w[0].detach().clone()

    if hasattr(data, "root_state_w"):
        return data.root_state_w[0, 0:3].detach().clone(), data.root_state_w[0, 3:7].detach().clone()

    raise AttributeError("Could not find cube root pose fields on cube.data")


def to_numpy_array(x):
    """Convert torch / warp / ProxyArray / numpy-like array to numpy."""
    if isinstance(x, torch.Tensor):
        return x.detach().cpu().numpy()

    if hasattr(x, "torch"):
        return x.torch.detach().cpu().numpy()

    if isinstance(x, wp.array):
        return wp.to_torch(x).detach().cpu().numpy()

    return np.asarray(x)


def reset_deformable_cube_to_pose(cube, pos_w_np, quat_xyzw_np):
    """Reset deformable cube nodal state to a desired pose.

    Uses the IsaacLab deformable API you pasted:
        - data.default_nodal_state_w.torch
        - data.nodal_kinematic_target.torch
        - transform_nodal_pos(...)
        - write_nodal_state_to_sim_index(...)
        - write_nodal_kinematic_target_to_sim_index(...)

    pos_w_np:
        Desired cube center in world frame, shape (3,).

    quat_xyzw_np:
        Desired cube orientation in IsaacLab XYZW, shape (4,).
    """
    device = cube.data.default_nodal_state_w.torch.device
    num_instances = cube.num_instances

    desired_pos_w = torch.as_tensor(
        pos_w_np,
        dtype=torch.float32,
        device=device,
    ).view(1, 3).repeat(num_instances, 1)

    desired_quat_xyzw = torch.as_tensor(
        quat_xyzw_np,
        dtype=torch.float32,
        device=device,
    ).view(1, 4).repeat(num_instances, 1)

    # Nodal state shape:
    #   (num_instances, max_sim_vertices_per_body, 6)
    #   [:, :, 0:3] = nodal positions
    #   [:, :, 3:6] = nodal velocities
    nodal_state = cube.data.default_nodal_state_w.torch.clone()

    default_nodal_pos = nodal_state[..., :3]
    default_center_w = default_nodal_pos.mean(dim=1)

    # transform_nodal_pos applies:
    #   rotated centered nodes + original mean + pos
    # Therefore, to obtain final center = desired_pos_w,
    # pass delta_pos = desired_pos_w - default_center_w.
    delta_pos_w = desired_pos_w - default_center_w

    nodal_state[..., :3] = cube.transform_nodal_pos(
        default_nodal_pos,
        pos=delta_pos_w,
        quat=desired_quat_xyzw,
    )

    # Zero nodal velocities.
    nodal_state[..., 3:] = 0.0

    cube.write_nodal_state_to_sim_index(nodal_state)

    # Free all nodes from kinematic targets.
    # API flag:
    #   0.0 = kinematic
    #   1.0 = free
    nodal_targets = cube.data.nodal_kinematic_target.torch.clone()
    nodal_targets[..., :3] = nodal_state[..., :3]
    nodal_targets[..., 3] = 1.0

    cube.write_nodal_kinematic_target_to_sim_index(nodal_targets)

    cube.reset()


def get_deformable_cube_center_world(cube):
    """Return deformable cube center/root position.

    According to the API you pasted, root_pos_w is computed as the mean of nodal positions.
    """
    return cube.data.root_pos_w.torch[0].detach().clone()


def get_robot_root_pos_quat_world_xyzw(robot):
    """Return robot root position/quaternion in world frame.

    In IsaacLab 3.x, quaternion is XYZW.
    """
    data = robot.data

    if hasattr(data, "root_pos_w") and hasattr(data, "root_quat_w"):
        root_pos = to_numpy_array(data.root_pos_w)[0].copy()
        root_quat_xyzw = to_numpy_array(data.root_quat_w)[0].copy()
        return root_pos, root_quat_xyzw

    if hasattr(data, "root_state_w"):
        root_state = to_numpy_array(data.root_state_w)[0]
        return root_state[0:3].copy(), root_state[3:7].copy()

    raise AttributeError("Could not find robot root pose fields on robot.data")


def quat_norm_wxyz(q):
    q = np.asarray(q, dtype=np.float64)
    return q / np.linalg.norm(q)


def quat_wxyz_to_xyzw(q_wxyz):
    q_wxyz = np.asarray(q_wxyz, dtype=np.float32)
    return np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=np.float32)


def quat_xyzw_to_rpy_np(q_xyzw):
    x, y, z, w = q_xyzw

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=np.float64)


def quat_wxyz_to_rotmat_np(q_wxyz):
    """Convert Holosoma/MuJoCo WXYZ quaternion to rotation matrix."""
    q = np.asarray(q_wxyz, dtype=np.float64)
    q = q / np.linalg.norm(q)

    w, x, y, z = q

    return np.array(
        [
            [1.0 - 2.0 * y * y - 2.0 * z * z, 2.0 * x * y - 2.0 * z * w, 2.0 * x * z + 2.0 * y * w],
            [2.0 * x * y + 2.0 * z * w, 1.0 - 2.0 * x * x - 2.0 * z * z, 2.0 * y * z - 2.0 * x * w],
            [2.0 * x * z - 2.0 * y * w, 2.0 * y * z + 2.0 * x * w, 1.0 - 2.0 * x * x - 2.0 * y * y],
        ],
        dtype=np.float64,
    )


def quat_xyzw_to_rotmat_np(q_xyzw):
    """Convert IsaacLab 3.x XYZW quaternion to rotation matrix."""
    q = np.asarray(q_xyzw, dtype=np.float64)
    q = q / np.linalg.norm(q)

    x, y, z, w = q

    return np.array(
        [
            [1.0 - 2.0 * y * y - 2.0 * z * z, 2.0 * x * y - 2.0 * z * w, 2.0 * x * z + 2.0 * y * w],
            [2.0 * x * y + 2.0 * z * w, 1.0 - 2.0 * x * x - 2.0 * z * z, 2.0 * y * z - 2.0 * x * w],
            [2.0 * x * z - 2.0 * y * w, 2.0 * y * z + 2.0 * x * w, 1.0 - 2.0 * x * x - 2.0 * y * y],
        ],
        dtype=np.float64,
    )


def find_elbow_body_id(robot, side: str):
    side = side.lower()
    matches = []

    for i, name in enumerate(robot.data.body_names):
        lname = name.lower()
        if side in lname and "elbow" in lname:
            matches.append((i, name))


    if len(matches) == 0:
        raise RuntimeError(f"Could not find body containing both {side!r} and 'elbow'.")


    body_id, body_name = matches[0]
    print(f"[Fake EE] Using {side} elbow body: id={body_id}, name={body_name}")

    return body_id, body_name


def fake_ee_point_from_elbow(robot, elbow_body_id: int, offset_local):
    device = robot.device

    body_pos_world, body_quat_world_xyzw = get_body_link_pos_quat_world_xyzw(robot)

    offset_local = torch.as_tensor(offset_local, dtype=torch.float32, device=device).unsqueeze(0)
    offset_local = offset_local.repeat(body_pos_world.shape[0], 1)

    elbow_pos_world = body_pos_world[:, elbow_body_id, :]
    elbow_quat_world_xyzw = body_quat_world_xyzw[:, elbow_body_id, :]

    # IsaacLab 3.x quat_apply expects IsaacLab-side XYZW quaternions.
    return elbow_pos_world + quat_apply(elbow_quat_world_xyzw, offset_local)


def disable_collisions_under_prim(prim_path_prefix: str):
    stage = sim_utils.get_current_stage()
    count = 0

    print(f"\n[Collision] Disabling collisions under: {prim_path_prefix}")

    for prim in stage.Traverse():
        prim_path = prim.GetPath().pathString

        if not prim_path.startswith(prim_path_prefix):
            continue

        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
            count += 1

    print(f"[Collision] Disabled {count} collision prims under {prim_path_prefix}.")


def force_enable_dome_light():
    stage = sim_utils.get_current_stage()

    lights_root = "/World/Lights"
    if not stage.GetPrimAtPath(lights_root).IsValid():
        UsdGeom.Xform.Define(stage, Sdf.Path(lights_root))

    dome_path = Sdf.Path("/World/Lights/ForcedDomeLight")
    dome = UsdLux.DomeLight.Define(stage, dome_path)
    dome.CreateIntensityAttr().Set(3000.0)
    dome.CreateExposureAttr().Set(2.0)
    dome.CreateColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    UsdGeom.Imageable(stage.GetPrimAtPath(dome_path)).MakeVisible()

    print("[Lights] Forced DomeLight enabled.")


# def desired_cube_pose_from_holosoma(
#     root_qpos_np,
#     object_qpos_np,
#     frame: int,
#     isaac_robot_pos_np,
#     isaac_robot_quat_xyzw_np,
#     reference_offset=None,
#     apply_z_lift=True,
# ):
#     """Return desired cube pose in Isaac coordinates.

#     This version preserves the cube pose relative to the robot root frame.

#     Holosoma/MuJoCo:
#         root_qpos quaternion: WXYZ
#         object_qpos quaternion: WXYZ

#     IsaacLab 3.x:
#         API/data quaternion: XYZW
#     """
#     frame = min(frame, root_qpos_np.shape[0] - 1, object_qpos_np.shape[0] - 1)

#     retarget_root_pos = np.asarray(root_qpos_np[frame, 0:3], dtype=np.float32)
#     retarget_root_quat_wxyz = np.asarray(root_qpos_np[frame, 3:7], dtype=np.float32)

#     retarget_dice_pos = np.asarray(object_qpos_np[frame, 0:3], dtype=np.float32)

#     source_dice_quat_wxyz = np.asarray(object_qpos_np[frame, 3:7], dtype=np.float32)
#     source_dice_quat_wxyz = quat_norm_wxyz(source_dice_quat_wxyz).astype(np.float32)

#     R_holo_dice = quat_wxyz_to_rotmat_np(source_dice_quat_wxyz)

#     # Holosoma root orientation.
#     R_holo_root = quat_wxyz_to_rotmat_np(retarget_root_quat_wxyz)

#     # Fixed Isaac root orientation.
#     R_isaac_root = quat_xyzw_to_rotmat_np(isaac_robot_quat_xyzw_np)

#     R_root_dice = R_holo_root.T @ R_holo_dice
#     R_isaac_dice = R_isaac_root @ R_root_dice

#     corrected_dice_quat_xyzw = rotmat_to_quat_xyzw_np(R_isaac_dice)

#     # Old version used this world vector directly.
#     dice_rel_world_holo = retarget_dice_pos - retarget_root_pos

#     # New version expresses the object in the Holosoma robot root frame.
#     dice_rel_root_local = R_holo_root.T @ dice_rel_world_holo

#     # Then places that same root-local vector around the fixed Isaac root.
#     pos = np.asarray(isaac_robot_pos_np, dtype=np.float32) + (
#         R_isaac_root @ dice_rel_root_local
#     ).astype(np.float32)

#     # For exact reference matching, keep this False.
#     # If True, the cube will intentionally be lifted by +1 cm.
#     if apply_z_lift:
#         pos[2] += 0.01

#     if reference_offset is not None:
#         pos = pos + np.asarray(reference_offset, dtype=np.float32)
#     if frame in [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360, 390, 420, 450, 480, 510]:
#         print(
#             "[DesiredCubePoseDebug] "
#             f"frame={frame} | "
#             f"root_pos={retarget_root_pos} | "
#             f"dice_world={retarget_dice_pos} | "
#             f"dice_minus_root_world={dice_rel_world_holo} | "
#             f"dice_rel_root_local={dice_rel_root_local} | "
#             f"isaac_pos={pos}"
#         )
#     return (
#         pos,
#         corrected_dice_quat_xyzw,
#         dice_rel_root_local.astype(np.float32),
#         retarget_root_pos,
#         retarget_dice_pos,
#     )
def desired_cube_pose_from_holosoma(
    root_qpos_np,
    object_qpos_np,
    frame: int,
    isaac_robot_pos_np,
    isaac_robot_quat_xyzw_np,
    reference_offset=None,
    apply_z_lift=True,
):
    """Return desired cube pose in Isaac coordinates.

    Object pose is replayed as Holosoma world delta from frame 0,
    mapped into Isaac coordinates using the frame-0 root alignment.

    This prevents robot-root bending from artificially moving the dice.
    """
    frame = min(frame, root_qpos_np.shape[0] - 1, object_qpos_np.shape[0] - 1)

    retarget_root_pos = np.asarray(root_qpos_np[frame, 0:3], dtype=np.float32)
    retarget_root_quat_wxyz = np.asarray(root_qpos_np[frame, 3:7], dtype=np.float32)
    retarget_root_quat_wxyz = quat_norm_wxyz(retarget_root_quat_wxyz).astype(np.float32)

    retarget_dice_pos = np.asarray(object_qpos_np[frame, 0:3], dtype=np.float32)

    source_dice_quat_wxyz = np.asarray(object_qpos_np[frame, 3:7], dtype=np.float32)
    source_dice_quat_wxyz = quat_norm_wxyz(source_dice_quat_wxyz).astype(np.float32)

    retarget_root_pos_0 = np.asarray(root_qpos_np[0, 0:3], dtype=np.float32)

    retarget_root_quat_wxyz_0 = np.asarray(root_qpos_np[0, 3:7], dtype=np.float32)
    retarget_root_quat_wxyz_0 = quat_norm_wxyz(retarget_root_quat_wxyz_0).astype(np.float32)

    retarget_dice_pos_0 = np.asarray(object_qpos_np[0, 0:3], dtype=np.float32)

    R_holo_root_0 = quat_wxyz_to_rotmat_np(retarget_root_quat_wxyz_0)
    R_isaac_root = quat_xyzw_to_rotmat_np(isaac_robot_quat_xyzw_np)

    R_align = R_isaac_root @ R_holo_root_0.T

    dice_rel_world_holo_0 = retarget_dice_pos_0 - retarget_root_pos_0

    initial_cube_pos = np.asarray(isaac_robot_pos_np, dtype=np.float32) + (
        R_align @ dice_rel_world_holo_0
    ).astype(np.float32)

    dice_world_delta = retarget_dice_pos - retarget_dice_pos_0

    pos = initial_cube_pos + (
        R_align @ dice_world_delta
    ).astype(np.float32)

    R_holo_dice = quat_wxyz_to_rotmat_np(source_dice_quat_wxyz)

    R_isaac_dice = R_align @ R_holo_dice

    corrected_dice_quat_xyzw = rotmat_to_quat_xyzw_np(R_isaac_dice)

    dice_rel_world_holo = retarget_dice_pos - retarget_root_pos
    R_holo_root = quat_wxyz_to_rotmat_np(retarget_root_quat_wxyz)
    dice_rel_root_local = R_holo_root.T @ dice_rel_world_holo

    if apply_z_lift:
        pos[2] += 0.01

    if reference_offset is not None:
        pos = pos + np.asarray(reference_offset, dtype=np.float32)

    return (
        pos,
        corrected_dice_quat_xyzw,
        dice_rel_root_local.astype(np.float32),
        retarget_root_pos,
        retarget_dice_pos,
    )


def load_motion_file(path: str):
    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(f"Motion file does not exist: {path}")

    print(f"\n[Motion] Loading: {path}")

    if path.suffix == ".npz":
        raw = np.load(path, allow_pickle=False)
        print("[Motion] npz keys:", raw.files)
        data = {k: raw[k] for k in raw.files}

    elif path.suffix == ".pkl":
        with open(path, "rb") as f:
            data = NumpyCompatUnpickler(f).load()
        print("[Motion] pkl keys:", list(data.keys()))

    else:
        raise ValueError(f"Unsupported file extension: {path.suffix}")

    if "qpos" not in data:
        raise KeyError(f"Expected key 'qpos'. Found keys: {list(data.keys())}")

    qpos = np.asarray(data["qpos"], dtype=np.float32)

    if qpos.ndim != 2 or not np.isfinite(qpos).all():
        raise ValueError(f"Bad qpos shape or values: {qpos.shape}")

    fps = float(np.asarray(data.get("fps", 30.0)).reshape(-1)[0])

    if qpos.shape[1] == 33:
        root_qpos = qpos[:, 0:7]
        robot_joint_qpos = qpos[:, 7:26]
        object_qpos = qpos[:, 26:33]

        print("\n[Motion parse] H1 + dice qpos")
        print("  root_qpos:        ", root_qpos.shape)
        print("  robot_joint_qpos: ", robot_joint_qpos.shape)
        print("  object_qpos:      ", object_qpos.shape)
        print("  Source object_qpos quaternion convention: WXYZ from Holosoma/MuJoCo")
        print("  IsaacLab 3.x write convention: XYZW")

        quat_delta = np.max(np.abs(object_qpos[:, 3:7] - object_qpos[0:1, 3:7]), axis=0)
        print("  object_qpos quaternion max abs delta from first frame [w,x,y,z]:", quat_delta)

    elif qpos.shape[1] == 26:
        root_qpos = qpos[:, 0:7]
        robot_joint_qpos = qpos[:, 7:26]
        object_qpos = None

    elif qpos.shape[1] == 19:
        root_qpos = None
        robot_joint_qpos = qpos
        object_qpos = None

    else:
        raise ValueError(f"Unexpected qpos dimension: {qpos.shape[1]}")

    if robot_joint_qpos.shape[1] != 19:
        raise ValueError(f"Expected 19 H1 joints, got {robot_joint_qpos.shape[1]}")

    return robot_joint_qpos, fps, root_qpos, object_qpos


def reset_articulation_to_default(articulation, scene: InteractiveScene, label: str):
    root_state = articulation.data.default_root_state.clone()
    root_state[:, :3] += scene.env_origins

    write_root_state_to_sim_index(articulation, root_state)

    joint_pos = articulation.data.default_joint_pos.clone()
    joint_vel = articulation.data.default_joint_vel.clone()

    write_joint_state_to_sim_index(articulation, joint_pos, joint_vel)
    set_joint_position_target_index(articulation, joint_pos)

    print(f"[INFO] Reset {label} to default state.")


def initial_robot_reset(scene: InteractiveScene):
    reset_articulation_to_default(scene["robot"], scene, "robot")
    reset_articulation_to_default(scene["reference_robot"], scene, "reference_robot")
    scene.reset()
    print("[INFO] Reset robot and reference_robot using InteractiveScene pattern.")

def rotmat_to_quat_xyzw_np(R):
    """Convert a 3x3 rotation matrix to an IsaacLab XYZW quaternion."""
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

    q_xyzw = np.array([x, y, z, w], dtype=np.float32)
    q_xyzw /= np.linalg.norm(q_xyzw)
    return q_xyzw
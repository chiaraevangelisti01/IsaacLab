import argparse
import pickle
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import torch
import warp as wp  # required by Isaac Lab 3 / PhysX-Warp backend in some paths

from isaaclab.app import AppLauncher

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=6000)
parser.add_argument("--dt", type=float, default=1.0 / 120.0)
parser.add_argument("--motion_file", type=str, required=True)
parser.add_argument("--motion_start_frame", type=int, default=0)
parser.add_argument("--motion_loop", action="store_true")
parser.add_argument("--playback_speed", type=float, default=1.0)
parser.add_argument(
    "--replay_mode",
    type=str,
    default="state",
    choices=["state", "target"],
    help="state = directly write joint state; target = send joint position targets.",
)
parser.add_argument("--cube_size", type=float, default=0.31)
parser.add_argument("--table_length", type=float, default=0.8)
parser.add_argument("--table_width", type=float, default=1.20)
parser.add_argument("--ground_z", type=float, default=0.0)
parser.add_argument(
    "--table_xy_mode",
    type=str,
    default="under_cube",
    choices=["under_cube", "manual"],
    help="under_cube = table centered under cube; manual = keep table x/y from scene config.",
)

# Diagnostics. The reference cube is ALWAYS replayed kinematically from object_qpos.
parser.add_argument("--cube_debug_log_csv", type=str, default="cube_replay_debug_log.csv")
parser.add_argument("--cube_debug_print_every", type=int, default=60)
parser.add_argument("--cube_debug_plot", action="store_true")
parser.add_argument(
    "--cube_debug_remove_z_lift",
    action="store_true",
    help="Compare/replay the Holosoma cube without the manual +0.01 m z lift.",
)
parser.add_argument(
    "--root_debug_frame",
    type=int,
    default=-1,
    help="If >= 0, dump IsaacLab reference root-local body/cube information at this motion frame.",
)
parser.add_argument(
    "--root_debug_body_filters",
    type=str,
    default="pelvis,torso,left_shoulder,right_shoulder,left_elbow,right_elbow",
    help=(
        "Comma-separated body name filters for IsaacLab root debug. "
        "Each filter is matched against robot.data.body_names."
    ),
)
parser.add_argument(
    "--root_debug_csv",
    type=str,
    default="isaac_reference_root_debug.csv",
    help="CSV path for IsaacLab reference root-local debug output.",
)
parser.add_argument(
    "--root_debug_print_bodies",
    action="store_true",
    help="Print all IsaacLab robot body names before resolving root debug filters.",
)


AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(visualizer=["kit"])
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# -----------------------------------------------------------------------------
# Isaac Lab imports
# -----------------------------------------------------------------------------
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.assets import AssetBaseCfg, ArticulationCfg, RigidObjectCfg, DeformableObjectCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply
import isaaclab.sim as sim_utils
from pxr import Usd, UsdLux, UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema

from isaaclab_physx.physics import PhysxCfg
from isaaclab.sim.schemas import MassPropertiesCfg
from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg, PhysxCollisionPropertiesCfg
from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg
from isaaclab_physx.sim import PhysxDeformableBodyPropertiesCfg, PhysxDeformableBodyMaterialCfg
from isaaclab_assets.robots.unitree_h1_aist import H1_FIXED_CFG

# -----------------------------------------------------------------------------
# Joint order mapping
# -----------------------------------------------------------------------------
HOLOSOMA_H1_JOINT_NAMES = [
    "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint", "left_knee_joint", "left_ankle_joint",
    "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint", "right_knee_joint", "right_ankle_joint",
    "torso_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
]

HOLOSOMA_TO_ISAAC_INDICES = [0, 5, 10, 1, 6, 11, 15, 2, 7, 12, 16, 3, 8, 13, 17, 4, 9, 14, 18]


def get_upper_body_isaac_joint_ids():
    upper_holo_start = HOLOSOMA_H1_JOINT_NAMES.index("torso_joint")
    upper_isaac_ids, upper_holo_names = [], []
    for isaac_id, holo_id in enumerate(HOLOSOMA_TO_ISAAC_INDICES):
        if holo_id >= upper_holo_start:
            upper_isaac_ids.append(isaac_id)
            upper_holo_names.append(HOLOSOMA_H1_JOINT_NAMES[holo_id])
    return upper_isaac_ids, upper_holo_names


# -----------------------------------------------------------------------------
# Constants
# -----------------------------------------------------------------------------
FAKE_EE_OFFSET_IN_ELBOW = [0.28, 0.0, -0.0185]
REFERENCE_ROBOT_WORLD_OFFSET = np.array([0.0, 3.0, 0.0], dtype=np.float32)
CUSTOM_DICE_DEFORMABLE_USD = "/home/chiara/git/IsaacLab_v3_test/dice_superquadric_deformable_two_meshes.usd"

CUSTOM_DICE_RIGID_REFERENCE_USD = (
    "/home/chiara/git/IsaacLab_v3_test/dice_superquadric.usd"
)
CUSTOM_DICE_SCALE = (1.0, 1.0, 1.0)

physx_cfg = PhysxCfg(
    solver_type=1,
    min_position_iteration_count=32,
    max_position_iteration_count=96,
    min_velocity_iteration_count=4,
    max_velocity_iteration_count=16,
    bounce_threshold_velocity=0.2,
)

# -----------------------------------------------------------------------------
# Compatibility helpers
# -----------------------------------------------------------------------------
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


def resolve_body_filters(robot, body_filters_csv: str):
    """Resolve comma-separated body filters to IsaacLab body ids.

    The function first tries exact match.
    If exact match fails, it checks whether all underscore-separated tokens
    appear in the body name.
    """
    filters = [x.strip() for x in body_filters_csv.split(",") if x.strip()]
    body_names = list(robot.data.body_names)

    resolved = []

    for filt in filters:
        filt_l = filt.lower()

        exact_matches = [(i, name) for i, name in enumerate(body_names) if name.lower() == filt_l]

        if exact_matches:
            resolved.append((filt, exact_matches[0][0], exact_matches[0][1]))
            continue

        tokens = [t for t in filt_l.replace("-", "_").split("_") if t]

        token_matches = []
        for i, name in enumerate(body_names):
            lname = name.lower()
            if all(t in lname for t in tokens):
                token_matches.append((i, name))

        if len(token_matches) == 0:
            print(f"[RootDebug][WARN] No IsaacLab body matched filter {filt!r}")
            continue

        if len(token_matches) > 1:
            print(f"[RootDebug][WARN] Multiple matches for filter {filt!r}: {token_matches}; using first")

        resolved.append((filt, token_matches[0][0], token_matches[0][1]))

    return resolved


def dump_isaac_reference_root_debug(
    reference_robot,
    reference_cube,
    frame: int,
    body_filters_csv: str,
    out_csv: str,
    print_bodies: bool = False,
):
    """Dump IsaacLab reference robot/cube root-local quantities.

    This is the IsaacLab-side equivalent of the Holosoma/Viser debug:
        link/body pos_root_local
        reference cube pos_root_local
        cube - body vectors in root-local coordinates
    """
    body_pos_world_t, body_quat_world_xyzw_t = get_body_link_pos_quat_world_xyzw(reference_robot)
    body_pos_world = to_numpy_array(body_pos_world_t)

    root_pos_world, root_quat_world_xyzw = get_robot_root_pos_quat_world_xyzw(reference_robot)
    root_R_world = quat_xyzw_to_rotmat_np(root_quat_world_xyzw)

    ref_cube_pos_t, ref_cube_quat_xyzw_t = get_cube_root_pos_quat_world_xyzw(reference_cube)
    ref_cube_pos_world = to_numpy_array(ref_cube_pos_t).copy()
    ref_cube_quat_xyzw = to_numpy_array(ref_cube_quat_xyzw_t).copy()


    resolved_bodies = resolve_body_filters(reference_robot, body_filters_csv)

    rows = []

    print("\n[IsaacLab reference root-origin debug]")
    print(f"frame: {frame}")
    print(f"reference root_pos_world:      {root_pos_world}")
    print(f"reference root_quat_xyzw:      {root_quat_world_xyzw}")
    print(f"reference cube_pos_world:      {ref_cube_pos_world}")
    print(f"reference cube_quat_xyzw:      {ref_cube_quat_xyzw}")

    for requested_filter, body_id, body_name in resolved_bodies:
        body_pos = body_pos_world[0, body_id, :].copy()

        rel_world = body_pos - root_pos_world
        rel_root = root_R_world.T @ rel_world

        row = {
            "frame": int(frame),
            "type": "body",
            "requested_filter": requested_filter,
            "name": body_name,
            "body_id": int(body_id),
            "world_x": float(body_pos[0]),
            "world_y": float(body_pos[1]),
            "world_z": float(body_pos[2]),
            "root_local_x": float(rel_root[0]),
            "root_local_y": float(rel_root[1]),
            "root_local_z": float(rel_root[2]),
        }
        rows.append(row)

        print(f"\nBODY {body_name}  requested_filter={requested_filter!r}")
        print(f"  pos_world:      {body_pos}")
        print(f"  pos_root_local: {rel_root}")

    cube_rel_world = ref_cube_pos_world - root_pos_world
    cube_rel_root = root_R_world.T @ cube_rel_world

    cube_row = {
        "frame": int(frame),
        "type": "reference_cube",
        "requested_filter": "reference_cube",
        "name": "reference_cube",
        "body_id": -1,
        "world_x": float(ref_cube_pos_world[0]),
        "world_y": float(ref_cube_pos_world[1]),
        "world_z": float(ref_cube_pos_world[2]),
        "root_local_x": float(cube_rel_root[0]),
        "root_local_y": float(cube_rel_root[1]),
        "root_local_z": float(cube_rel_root[2]),
    }
    rows.append(cube_row)

    print("\nREFERENCE_CUBE")
    print(f"  cube_pos_world:      {ref_cube_pos_world}")
    print(f"  cube_pos_root_local: {cube_rel_root}")

    print("\n[Reference cube relative to selected bodies, root-local]")
    for r in rows:
        if r["type"] != "body":
            continue

        body_rel_root = np.array(
            [r["root_local_x"], r["root_local_y"], r["root_local_z"]],
            dtype=np.float64,
        )
        cube_minus_body_root = cube_rel_root - body_rel_root

        print(f"  reference_cube - {r['name']}: {cube_minus_body_root}")

    if rows:
        with Path(out_csv).open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

        print(f"\n[IsaacLab RootDebug] wrote CSV: {out_csv}")

    return rows

def dump_deformable_cube_geometry_debug(cube, label: str, desired_center_np=None, table_top_z=None):
    """Inspect the actual deformable simulation nodes.

    This does not rely on collision visualization.
    It checks where the deformable nodal mesh really is.
    """
    nodal_pos = cube.data.nodal_pos_w.torch[0].detach()
    root_pos = cube.data.root_pos_w.torch[0].detach()

    bbox_min = nodal_pos.amin(dim=0)
    bbox_max = nodal_pos.amax(dim=0)
    bbox_center = 0.5 * (bbox_min + bbox_max)
    bbox_extent = bbox_max - bbox_min

    print(f"\n[DeformableCubeGeomDebug] {label}")
    print(f"  root_pos_w mean nodes: {root_pos.cpu().numpy()}")
    print(f"  bbox_min_w:            {bbox_min.cpu().numpy()}")
    print(f"  bbox_max_w:            {bbox_max.cpu().numpy()}")
    print(f"  bbox_center_w:         {bbox_center.cpu().numpy()}")
    print(f"  bbox_extent_xyz:       {bbox_extent.cpu().numpy()}")

    if desired_center_np is not None:
        desired_center = torch.as_tensor(desired_center_np, dtype=torch.float32, device=nodal_pos.device)
        root_minus_desired = root_pos - desired_center
        bbox_center_minus_desired = bbox_center - desired_center

        print(f"  desired_center_w:      {desired_center.cpu().numpy()}")
        print(f"  root_minus_desired:    {root_minus_desired.cpu().numpy()}")
        print(f"  bbox_center-desired:   {bbox_center_minus_desired.cpu().numpy()}")

    if table_top_z is not None:
        gap = float(bbox_min[2].cpu().item() - table_top_z)
        print(f"  table_top_z:           {table_top_z:.6f}")
        print(f" expected_cube_bottom_z: {root_pos[2] - args_cli.cube_size * CUSTOM_DICE_SCALE[2] / 2.0}")
        print(f"  bbox_min_z-table_top:  {gap:.6f} m")

def dump_usd_physics_attrs_under(prim_path_prefix: str):
    stage = sim_utils.get_current_stage()

    print(f"\n[USDPhysicsAttrDebug] {prim_path_prefix}")

    for prim in stage.Traverse():
        prim_path = prim.GetPath().pathString
        if not prim_path.startswith(prim_path_prefix):
            continue

        applied = list(prim.GetAppliedSchemas())
        attrs_to_print = []

        # for attr in prim.GetAttributes():
        #     name = attr.GetName()
        #     lname = name.lower()

        #     if (
        #         "physx" in lname
        #         or "physics" in lname
        #         or "collision" in lname
        #         or "deform" in lname
        #         or "contact" in lname
        #         or "rest" in lname
        #         or "offset" in lname
        #         or "omni" in lname
        #     ):
        #         try:
        #             value = attr.Get()
        #         except Exception:
        #             value = "<could not read>"
        #         attrs_to_print.append((name, value))
        
        # if applied or attrs_to_print or prim.GetTypeName() in ["Mesh", "TetMesh"]:
        #     print(f"\nPrim: {prim_path}")
        #     print(f"  type: {prim.GetTypeName()}")
        #     print(f"  applied schemas: {applied}")

        #     for name, value in attrs_to_print:

        #         if isinstance(value, (list, tuple)):
        #             print(f"  {name}: <array length={len(value)}>")
        #             continue

        #         if hasattr(value, "__len__") and not isinstance(value, (str, bytes)):
        #             try:
        #                 n = len(value)
        #                 if n > 20:
        #                     print(f"  {name}: <array length={n}>")
        #                     continue
        #             except Exception:
        #                 pass

        #         print(f"  {name}: {value}")
        for attr in prim.GetAttributes():
            try:
                value = attr.Get()
            except Exception:
                value = "<unreadable>"

            if hasattr(value, "__len__") and not isinstance(value, (str, bytes)):
                try:
                    value = f"<array length={len(value)}>"
                except Exception:
                    pass

            print(f"  {attr.GetName()}: {value}")
# -----------------------------------------------------------------------------
# Quaternion helpers
# -----------------------------------------------------------------------------
# Holosoma/MuJoCo source quaternions are WXYZ.
# IsaacLab 3.x API/data quaternions are XYZW.
# Therefore:
#   - WXYZ helpers are only for source-side/manual correction math.
#   - Everything written to IsaacLab must be XYZW.
# -----------------------------------------------------------------------------
def quat_norm_wxyz(q):
    q = np.asarray(q, dtype=np.float64)
    return q / np.linalg.norm(q)


def quat_conj_wxyz(q):
    q = quat_norm_wxyz(q)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quat_mul_wxyz(q1, q2):
    """Hamilton product in WXYZ convention."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def quat_from_axis_angle_wxyz(axis, angle_rad):
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)

    half = 0.5 * angle_rad
    return np.array(
        [
            np.cos(half),
            axis[0] * np.sin(half),
            axis[1] * np.sin(half),
            axis[2] * np.sin(half),
        ],
        dtype=np.float64,
    )


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


def quat_abs_dot_np(q1, q2):
    q1 = np.asarray(q1, dtype=np.float64).reshape(4)
    q2 = np.asarray(q2, dtype=np.float64).reshape(4)

    n1 = np.linalg.norm(q1)
    n2 = np.linalg.norm(q2)

    if n1 < 1e-12 or n2 < 1e-12:
        return np.nan

    return float(abs(np.dot(q1 / n1, q2 / n2)))

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


# -----------------------------------------------------------------------------
# Fake EE helpers
# -----------------------------------------------------------------------------
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


# -----------------------------------------------------------------------------
# USD helpers
# -----------------------------------------------------------------------------
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

def set_collision_enabled_for_prim(prim_path: str, enabled: bool):
    stage = sim_utils.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)

    if not prim.IsValid():
        print(f"[CollisionDebug][WARN] Prim not found: {prim_path}")
        return

    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        print(f"[CollisionDebug][WARN] Prim has no UsdPhysics.CollisionAPI: {prim_path}")
        print(f"  applied schemas: {list(prim.GetAppliedSchemas())}")
        return

    collision_api = UsdPhysics.CollisionAPI(prim)
    collision_api.CreateCollisionEnabledAttr().Set(bool(enabled))

    print(f"[CollisionDebug] Set collisionEnabled={enabled} on {prim_path}")

# -----------------------------------------------------------------------------
# Cube pose math
# -----------------------------------------------------------------------------
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

    This version preserves the cube pose relative to the robot root frame.

    Holosoma/MuJoCo:
        root_qpos quaternion: WXYZ
        object_qpos quaternion: WXYZ

    IsaacLab 3.x:
        API/data quaternion: XYZW
    """
    frame = min(frame, root_qpos_np.shape[0] - 1, object_qpos_np.shape[0] - 1)

    retarget_root_pos = np.asarray(root_qpos_np[frame, 0:3], dtype=np.float32)
    retarget_root_quat_wxyz = np.asarray(root_qpos_np[frame, 3:7], dtype=np.float32)

    retarget_dice_pos = np.asarray(object_qpos_np[frame, 0:3], dtype=np.float32)

    source_dice_quat_wxyz = np.asarray(object_qpos_np[frame, 3:7], dtype=np.float32)
    source_dice_quat_wxyz = quat_norm_wxyz(source_dice_quat_wxyz).astype(np.float32)
    corrected_dice_quat_xyzw = quat_wxyz_to_xyzw(source_dice_quat_wxyz)

    # Holosoma root orientation.
    R_holo_root = quat_wxyz_to_rotmat_np(retarget_root_quat_wxyz)

    # Fixed Isaac root orientation.
    R_isaac_root = quat_xyzw_to_rotmat_np(isaac_robot_quat_xyzw_np)

    # Old version used this world vector directly.
    dice_rel_world_holo = retarget_dice_pos - retarget_root_pos

    # New version expresses the object in the Holosoma robot root frame.
    dice_rel_root_local = R_holo_root.T @ dice_rel_world_holo

    # Then places that same root-local vector around the fixed Isaac root.
    pos = np.asarray(isaac_robot_pos_np, dtype=np.float32) + (
        R_isaac_root @ dice_rel_root_local
    ).astype(np.float32)

    # For exact reference matching, keep this False.
    # If True, the cube will intentionally be lifted by +1 cm.
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

# -----------------------------------------------------------------------------
# Scene config
# -----------------------------------------------------------------------------
def make_rigid_cube_cfg(prim_path: str, color=(0.9, 0.45, 0.0), kinematic=False, collision_enabled=True):
    return RigidObjectCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=CUSTOM_DICE_RIGID_REFERENCE_USD,
            scale=CUSTOM_DICE_SCALE,
            rigid_props=PhysxRigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                kinematic_enabled=kinematic,
                disable_gravity=kinematic,
                max_depenetration_velocity=20.0,
                solver_position_iteration_count=96,
                solver_velocity_iteration_count=16,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=100.0,
                max_angular_velocity=100.0,
            ),
            mass_props=MassPropertiesCfg(mass=0.8),
            collision_props=PhysxCollisionPropertiesCfg(
                collision_enabled=collision_enabled,
                contact_offset=0.001,
                rest_offset=0.000,
            ),
            physics_material=PhysxRigidBodyMaterialCfg(
                static_friction=1.2,
                dynamic_friction=1.0,
                restitution=0.0,
                friction_combine_mode="average",
                restitution_combine_mode="average",
                compliant_contact_stiffness=None,
                compliant_contact_damping=None,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
        ),
        # IsaacLab 3.x XYZW identity quaternion.
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-0.15, 0.0, 1.01), rot=(0.0, 0.0, 0.0, 1.0)),
    )

def make_deformable_cube_cfg(
    prim_path: str,
    color=(0.5, 0.1, 0.0),
):
    return DeformableObjectCfg(
        prim_path=prim_path,

        spawn=sim_utils.UsdFileCfg(
            usd_path=CUSTOM_DICE_DEFORMABLE_USD,
            scale=CUSTOM_DICE_SCALE,

            # These are body/solver properties.
            #
            # They should be applied at scene creation, before sim.reset().
            # Collision rest/contact offsets remain authored directly on
            # the collision-bearing sim_mesh inside the prepared USD.
            # deformable_props=PhysxDeformableBodyPropertiesCfg(
            #     # Increase from the default 16.
            #     solver_position_iteration_count=64,

            #     # Prevent different parts of the deformable surface from
            #     # passing through each other.
            #     self_collision=True,

            #     # Leave automatic initially. The PhysX-selected value is
            #     # safer than guessing until self-collision is confirmed.
            #     self_collision_filter_distance=None,

            #     # Dissipate global vertex motion and oscillation.
            #     linear_damping=1.0,

            #     # Keep the existing low-speed settling behavior explicit.
            #     settling_damping=10.0,
            #     settling_threshold=0.1,
            #     sleep_threshold=0.05,

            #     # Prevent very large corrective velocities when contacts
            #     # or initial overlap are resolved.
            #     max_depenetration_velocity=1.0,

            #     # Bound runaway nodal velocities during diagnosis.
            #     max_linear_velocity=5.0,

            #     # Useful for moving robot contacts and faster vertices.
            #     enable_speculative_c_c_d=True,

            #     disable_gravity=False,
            # ),

            # Start from the same material range as the working Isaac Lab
            # tutorial rather than the very soft 9 kPa configuration.
            physics_material=(
                PhysxDeformableBodyMaterialCfg(
                    density=21.5,
                    poissons_ratio=0.35,
                    youngs_modulus=1e4,

                    static_friction=1.2,
                    dynamic_friction=0.8,

                    elasticity_damping=0.02,
                )
            ),

            visual_material=(
                sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(
                        0.8,
                        0.1,
                        0.1,
                    ),
                )
            ),
        ),

        init_state=DeformableObjectCfg.InitialStateCfg(
            pos=(-0.15, 0.0, 1.01),
        ),

        debug_vis=True,
    )

@configclass
class ReplaySceneCfg(InteractiveSceneCfg):
    plane = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.0, 0.0, 0.40],
            rot=[0.0, 0.0, 0.0, 1.0],  # IsaacLab 3.x XYZW identity.
        ),
        spawn=sim_utils.CuboidCfg(
            size=(args_cli.table_length, args_cli.table_width, 0.80),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.001),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.35, 0.35)),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.2,
                dynamic_friction=0.8,
                restitution=0.0,
                friction_combine_mode="average",
                restitution_combine_mode="average",
            ),
        ),
    )

    # Actual cube: deformable physics cube.
    cube: DeformableObjectCfg = make_deformable_cube_cfg(
        "{ENV_REGEX_NS}/Cube",
        color=(0.5, 0.1, 0.0),
    )

    # Reference cube: keep rigid/kinematic because it is replayed by root pose every frame.
    reference_cube: RigidObjectCfg = make_rigid_cube_cfg(
        "{ENV_REGEX_NS}/ReferenceCube",
        color=(1.0, 0.0, 1.0),
        kinematic=True,
        collision_enabled=False,
    )

    robot: ArticulationCfg = H1_FIXED_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        # IsaacLab 3.x XYZW. You said this was manually tuned and visually correct.
        init_state=ArticulationCfg.InitialStateCfg(rot=(0.0, 0.0, 1.0, 0.0), pos=(0.0, 0.0, 1.02)),
    )

    reference_robot: ArticulationCfg = H1_FIXED_CFG.replace(
        prim_path="{ENV_REGEX_NS}/ReferenceRobot",
        # IsaacLab 3.x XYZW. Same tuned orientation as actual robot.
        init_state=ArticulationCfg.InitialStateCfg(
            rot=(0.0, 0.0, 1.0, 0.0),
            pos=(
                float(REFERENCE_ROBOT_WORLD_OFFSET[0]),
                float(REFERENCE_ROBOT_WORLD_OFFSET[1]),
                1.02 + float(REFERENCE_ROBOT_WORLD_OFFSET[2]),
            ),
        ),
    )


# -----------------------------------------------------------------------------
# Motion loading
# -----------------------------------------------------------------------------
class NumpyCompatUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module.startswith("numpy._core"):
            module = module.replace("numpy._core", "numpy.core")
        return super().find_class(module, name)


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


# -----------------------------------------------------------------------------
# Layout
# -----------------------------------------------------------------------------
def apply_relative_cube_and_ground_table_layout(scene_cfg, root_qpos_np, object_qpos_np, frame=0):
    if root_qpos_np is None or object_qpos_np is None:
        print("[Layout] No root/object qpos found. Using manual cube/table positions.")
        return

    isaac_robot_pos = np.asarray(scene_cfg.robot.init_state.pos, dtype=np.float32)
    isaac_robot_quat_xyzw = np.asarray(scene_cfg.robot.init_state.rot, dtype=np.float32)

    cube_pos, cube_quat_xyzw, dice_rel, root_pos, dice_pos = desired_cube_pose_from_holosoma(
        root_qpos_np,
        object_qpos_np,
        frame,
        isaac_robot_pos,
        isaac_robot_quat_xyzw,
        reference_offset=None,
        apply_z_lift=False,
    )

    rpy = quat_xyzw_to_rpy_np(cube_quat_xyzw)

    scene_cfg.cube.init_state.pos = cube_pos.tolist()
    

    ref_cube_pos = cube_pos + REFERENCE_ROBOT_WORLD_OFFSET
    scene_cfg.reference_cube.init_state.pos = ref_cube_pos.tolist()
    scene_cfg.reference_cube.init_state.rot = cube_quat_xyzw.tolist()

    ground_z = float(args_cli.ground_z)
    actual_cube_height = args_cli.cube_size * CUSTOM_DICE_SCALE[2]
    cube_bottom_z = float(cube_pos[2] - actual_cube_height / 2.0)
    table_thickness = cube_bottom_z - ground_z

    if table_thickness <= 0.0:
        raise ValueError(f"Computed table_thickness <= 0: {table_thickness}")

    table_center_z = ground_z + table_thickness / 2.0

    table_pos = np.asarray(scene_cfg.table.init_state.pos, dtype=np.float32)

    if args_cli.table_xy_mode == "under_cube":
        table_pos[0] = cube_pos[0] - 0.25
        table_pos[1] = cube_pos[1]

    table_pos[2] = table_center_z
    scene_cfg.table.init_state.pos = table_pos.tolist()
    scene_cfg.table.spawn.size = (
        float(args_cli.table_length),
        float(args_cli.table_width),
        float(table_thickness),
    )

    print("\n[Layout from Holosoma relative pose + ground-derived table]")
    print(f"  frame used:                 {frame}")
    print(f"  retarget_root_pos:          {root_pos}")
    print(f"  retarget_dice_pos:          {dice_pos}")
    print(f"  dice_rel_to_root:           {dice_rel}")
    print(f"  manual Isaac robot pos:     {isaac_robot_pos}")
    print(f"  actual cube init center:    {cube_pos}")
    print(f"  reference cube init center: {ref_cube_pos}")
    print(f"  cube quat IsaacLab XYZW:    {cube_quat_xyzw}")
    print(f"  cube rpy from XYZW {np.rad2deg(rpy)}")
    print(f"  table_pos:                  {table_pos}")
    print(f"  table_size:                 {scene_cfg.table.spawn.size}")


# -----------------------------------------------------------------------------
# Plotting
# -----------------------------------------------------------------------------
def show_upper_body_tracking_plot(times, frames, q_des_log, q_act_log, q_raw_log, robot_joint_names):
    times, frames = np.asarray(times), np.asarray(frames)
    q_des_log, q_act_log, q_raw_log = np.asarray(q_des_log), np.asarray(q_act_log), np.asarray(q_raw_log)

    upper_ids, upper_holo_names = get_upper_body_isaac_joint_ids()

    print("\n[Upper-body tracking error]")
    err = q_act_log[:, upper_ids] - q_des_log[:, upper_ids]

    for j, holo_name in enumerate(upper_holo_names):
        isaac_id = upper_ids[j]
        print(
            f"  {robot_joint_names30s} "
            f"mean_abs_error={np.abs(err[:, j]).mean():.6f} rad, "
            f"max_abs_error={np.abs(err[:, j]).max():.6f} rad"
        )

    ncols = 3
    nrows = int(np.ceil(len(upper_ids) / 3))

    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(5.5 * ncols, 3.2 * nrows), sharex=True)
    axes = np.asarray(axes).reshape(-1)

    for plot_id, isaac_id in enumerate(upper_ids):
        ax = axes[plot_id]
        ax.plot(times, q_des_log[:, isaac_id], label="desired after clamp")
        ax.plot(times, q_act_log[:, isaac_id], label="actual", linestyle="--")
        ax.plot(times, q_raw_log[:, isaac_id], label="desired before clamp", linestyle=":", alpha=0.8)
        ax.set_title(f"{robot_joint_names[isaac_id]}\nfrom {upper_holo_names[plot_id]}")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("joint position [rad]")
        ax.grid(True)

    for k in range(len(upper_ids), len(axes)):
        axes[k].axis("off")

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3)
    fig.suptitle("Upper-body joint tracking: desired vs actual", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    plt.show()


def show_fake_ee_commanded_vs_actual_xyz_plot(times, frames, left_cmd, left_act, right_cmd, right_act):
    times, frames = np.asarray(times), np.asarray(frames)
    left_cmd, left_act, right_cmd, right_act = map(np.asarray, [left_cmd, left_act, right_cmd, right_act])

    le = left_act - left_cmd
    re = right_act - right_cmd

    lenorm = np.linalg.norm(le, axis=1)
    renorm = np.linalg.norm(re, axis=1)

    li = int(np.argmax(lenorm))
    ri = int(np.argmax(renorm))

    print("\n[Fake EE commanded-vs-actual task-space error]")
    print(f"  left fake EE:  mean={lenorm.mean():.6f} m, max={lenorm.max():.6f} m at frame={frames[li]}, time={times[li]:.6f} s")
    print(f"  right fake EE: mean={renorm.mean():.6f} m, max={renorm.max():.6f} m at frame={frames[ri]}, time={times[ri]:.6f} s")

    fig, axes = plt.subplots(3, 3, figsize=(16, 9), sharex=True)
    labels = ["x", "y", "z"]

    for k, lab in enumerate(labels):
        axes[0, k].plot(frames, left_cmd[:, k], label="commanded/reference")
        axes[0, k].plot(frames, left_act[:, k], "--", label="actual/executed")
        axes[0, k].axvline(frames[li], color="red", linestyle=":")
        axes[0, k].set_title(f"left fake EE {lab}")
        axes[0, k].grid(True)
        axes[0, k].legend()

        axes[1, k].plot(frames, right_cmd[:, k], label="commanded/reference")
        axes[1, k].plot(frames, right_act[:, k], "--", label="actual/executed")
        axes[1, k].axvline(frames[ri], color="red", linestyle=":")
        axes[1, k].set_title(f"right fake EE {lab}")
        axes[1, k].grid(True)
        axes[1, k].legend()

        axes[2, k].plot(frames, le[:, k], label="left actual - commanded")
        axes[2, k].plot(frames, re[:, k], "--", label="right actual - commanded")
        axes[2, k].axvline(frames[li], color="red", linestyle=":")
        axes[2, k].axvline(frames[ri], color="orange", linestyle=":")
        axes[2, k].set_title(f"fake EE tracking error {lab}")
        axes[2, k].set_xlabel("frame number")
        axes[2, k].grid(True)
        axes[2, k].legend()

    fig.suptitle("Fake MuJoCo end-effector point tracking\ncommanded/reference FK vs actual/executed Isaac robot", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    plt.show()


def show_cube_replay_debug_plot(rows):
    if not rows:
        return

    frames = np.asarray([r["frame"] for r in rows])

    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)

    for i, lab in enumerate(["x", "y", "z"]):
        axes[i].plot(frames, [r[f"desired_cube_{lab}"] for r in rows], label=f"Holosoma desired cube {lab}")
        axes[i].plot(frames, [r[f"actual_cube_{lab}"] for r in rows], "--", label=f"Isaac actual cube {lab}")
        axes[i].set_ylabel(f"{lab} [m]")
        axes[i].grid(True)
        axes[i].legend()

    axes[3].plot(frames, [r["cube_pos_err_norm"] for r in rows], label="actual - desired norm")
    axes[3].set_ylabel("error [m]")
    axes[3].set_xlabel("frame number")
    axes[3].grid(True)
    axes[3].legend()

    fig.suptitle("Cube trajectory: Holosoma desired/reference cube vs Isaac actual cube")
    fig.tight_layout()
    plt.show()


# -----------------------------------------------------------------------------
# Reset
# -----------------------------------------------------------------------------
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


# -----------------------------------------------------------------------------
# Replay
# -----------------------------------------------------------------------------
def run(sim: SimulationContext, scene: InteractiveScene, robot_joint_qpos_np: np.ndarray, motion_fps: float, root_qpos_np=None, object_qpos_np=None):
    robot = scene["robot"]
    reference_robot = scene["reference_robot"]
    cube = scene["cube"]
    reference_cube = scene["reference_cube"]

    sim_dt = sim.get_physics_dt()
    device = args_cli.device

    reference_robot_world_offset_t = torch.as_tensor(
        REFERENCE_ROBOT_WORLD_OFFSET,
        dtype=torch.float32,
        device=device,
    ).unsqueeze(0)

    q_holo = torch.as_tensor(robot_joint_qpos_np, dtype=torch.float32, device=device)

    num_envs = robot.data.default_joint_pos.shape[0]

    if robot.data.default_joint_pos.shape[1] != 19 or q_holo.shape[1] != 19:
        raise ValueError("Expected 19 H1 joints in Isaac and Holosoma motion.")

    reorder_idx = torch.tensor(HOLOSOMA_TO_ISAAC_INDICES, dtype=torch.long, device=device)
    q_traj = q_holo[:, reorder_idx]
    num_frames = q_traj.shape[0]

    print("\n[Replay settings]")
    print(f"  num_frames: {num_frames}")
    print(f"  motion_fps: {motion_fps}")
    print(f"  sim_dt: {sim_dt}")
    print(f"  replay_mode: {args_cli.replay_mode}")

    print("\n[Cube debug]")
    if root_qpos_np is None or object_qpos_np is None:
        print("  No object_qpos: ReferenceCube cannot be kinematically replayed.")
    else:
        print("  ReferenceCube is ALWAYS kinematically replayed from Holosoma object_qpos.")
        print("  Holosoma/MuJoCo source quaternion: WXYZ")
        print("  IsaacLab 3.x write/data quaternion: XYZW")

    left_elbow_body_id, _ = find_elbow_body_id(robot, "left")
    right_elbow_body_id, _ = find_elbow_body_id(robot, "right")
    ref_left_elbow_body_id, _ = find_elbow_body_id(reference_robot, "left")
    ref_right_elbow_body_id, _ = find_elbow_body_id(reference_robot, "right")

    tracking_times, tracking_frames = [], []
    tracking_q_raw, tracking_q_des, tracking_q_actual = [], [], []
    tracking_left_fake_ee_commanded, tracking_left_fake_ee_actual = [], []
    tracking_right_fake_ee_commanded, tracking_right_fake_ee_actual = [], []
    cube_rows = []

    isaac_robot_init_pos_np = np.asarray(scene["robot"].cfg.init_state.pos, dtype=np.float32)
    isaac_robot_init_quat_xyzw_np = np.asarray(scene["robot"].cfg.init_state.rot, dtype=np.float32)


    # Initialize both robots to first frame.
    first_frame = min(args_cli.motion_start_frame, num_frames - 1)

    q0 = q_traj[first_frame].unsqueeze(0).repeat(num_envs, 1)

    if hasattr(robot.data, "soft_joint_pos_limits"):
        q0 = torch.clamp(q0, robot.data.soft_joint_pos_limits[..., 0], robot.data.soft_joint_pos_limits[..., 1])

    q0_vel = torch.zeros_like(q0)

    write_joint_state_to_sim_index(robot, q0, q0_vel)
    set_joint_position_target_index(robot, q0)

    write_joint_state_to_sim_index(reference_robot, q0, q0_vel)
    set_joint_position_target_index(reference_robot, q0)


    # Initialize actual deformable cube to the same corrected Holosoma pose.
    if root_qpos_np is not None and object_qpos_np is not None:
        actual_cube_pos, actual_cube_quat_xyzw, *_ = desired_cube_pose_from_holosoma(
            root_qpos_np,
            object_qpos_np,
            first_frame,
            isaac_robot_init_pos_np,
            isaac_robot_init_quat_xyzw_np,
            reference_offset=None,
            apply_z_lift=False,
        )

        reset_deformable_cube_to_pose(
            cube,
            pos_w_np=actual_cube_pos,
            quat_xyzw_np=actual_cube_quat_xyzw,
        )

        table_top_z = float(actual_cube_pos[2] - args_cli.cube_size * CUSTOM_DICE_SCALE[2] / 2.0)
        


        dump_deformable_cube_geometry_debug(
            cube,
            label="after reset_deformable_cube_to_pose, before first sim step",
            desired_center_np=actual_cube_pos,
            table_top_z=table_top_z,
        )
                

    # Initialize reference cube to first frame too.
    if root_qpos_np is not None and object_qpos_np is not None:
        ref_pos, quat_xyzw, *_ = desired_cube_pose_from_holosoma(
            root_qpos_np,
            object_qpos_np,
            first_frame,
            isaac_robot_init_pos_np,
            isaac_robot_init_quat_xyzw_np,
            reference_offset=REFERENCE_ROBOT_WORLD_OFFSET,
            apply_z_lift=False,
        )

        ref_pose_xyzw = torch.tensor(
            [[ref_pos[0], ref_pos[1], ref_pos[2], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]]],
            dtype=torch.float32,
            device=device,
        ).repeat(num_envs, 1)

        write_cube_pose_to_sim(reference_cube, ref_pose_xyzw)

    scene.write_data_to_sim()
    sim.step()
    scene.update(sim_dt)

    dump_deformable_cube_geometry_debug(
        cube,
        label="after first sim.step/update",
        desired_center_np=actual_cube_pos,
        table_top_z=table_top_z,
    )

    root_debug_dumped = False
    count = 0

    while simulation_app.is_running() and count < args_cli.steps:
        sim_time = count * sim_dt

        frame = args_cli.motion_start_frame + int(sim_time * motion_fps * args_cli.playback_speed)
        frame = frame % num_frames if args_cli.motion_loop else min(frame, num_frames - 1)

        q_raw = q_traj[frame].unsqueeze(0).repeat(num_envs, 1)
        q_des = q_raw.clone()

        if hasattr(robot.data, "soft_joint_pos_limits"):
            limits = robot.data.soft_joint_pos_limits
            q_des = torch.clamp(q_des, limits[..., 0], limits[..., 1])

        if args_cli.replay_mode == "state":
            q_vel = torch.zeros_like(q_des)
            write_joint_state_to_sim_index(robot, q_des, q_vel)
            set_joint_position_target_index(robot, q_des)
        else:
            set_joint_position_target_index(robot, q_des)

        ref_q_vel = torch.zeros_like(q_des)
        write_joint_state_to_sim_index(reference_robot, q_des, ref_q_vel)
        set_joint_position_target_index(reference_robot, q_des)

        desired_cube_pos_real = None
        desired_cube_quat_xyzw = None
        dice_rel = None
        root_pos = None
        dice_pos = None

        if root_qpos_np is not None and object_qpos_np is not None:
            desired_cube_pos_real, desired_cube_quat_xyzw, dice_rel, root_pos, dice_pos = desired_cube_pose_from_holosoma(
                root_qpos_np,
                object_qpos_np,
                frame,
                isaac_robot_init_pos_np,
                isaac_robot_init_quat_xyzw_np,
                reference_offset=None,
                apply_z_lift=False,
            )

            desired_cube_pos_ref = desired_cube_pos_real + REFERENCE_ROBOT_WORLD_OFFSET

            ref_pose_xyzw = torch.tensor(
                [[
                    desired_cube_pos_ref[0],
                    desired_cube_pos_ref[1],
                    desired_cube_pos_ref[2],
                    desired_cube_quat_xyzw[0],
                    desired_cube_quat_xyzw[1],
                    desired_cube_quat_xyzw[2],
                    desired_cube_quat_xyzw[3],
                ]],
                dtype=torch.float32,
                device=device,
            ).repeat(num_envs, 1)

            write_cube_pose_to_sim(reference_cube, ref_pose_xyzw)

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        if (
            args_cli.root_debug_frame >= 0
            and not root_debug_dumped
            and int(frame) == int(args_cli.root_debug_frame)
        ):
            dump_isaac_reference_root_debug(
                reference_robot=reference_robot,
                reference_cube=reference_cube,
                frame=frame,
                body_filters_csv=args_cli.root_debug_body_filters,
                out_csv=args_cli.root_debug_csv,
                print_bodies=args_cli.root_debug_print_bodies,
            )
            root_debug_dumped = True

        q_actual = robot.data.joint_pos.detach().clone()

        left_fake_ee_commanded_world = fake_ee_point_from_elbow(reference_robot, ref_left_elbow_body_id, FAKE_EE_OFFSET_IN_ELBOW)
        right_fake_ee_commanded_world = fake_ee_point_from_elbow(reference_robot, ref_right_elbow_body_id, FAKE_EE_OFFSET_IN_ELBOW)

        left_fake_ee_commanded = left_fake_ee_commanded_world - reference_robot_world_offset_t
        right_fake_ee_commanded = right_fake_ee_commanded_world - reference_robot_world_offset_t

        left_fake_ee_actual = fake_ee_point_from_elbow(robot, left_elbow_body_id, FAKE_EE_OFFSET_IN_ELBOW)
        right_fake_ee_actual = fake_ee_point_from_elbow(robot, right_elbow_body_id, FAKE_EE_OFFSET_IN_ELBOW)

        tracking_times.append(sim_time)
        tracking_frames.append(frame)
        tracking_q_raw.append(q_raw[0].detach().cpu().numpy())
        tracking_q_des.append(q_des[0].detach().cpu().numpy())
        tracking_q_actual.append(q_actual[0].detach().cpu().numpy())
        tracking_left_fake_ee_commanded.append(left_fake_ee_commanded[0].detach().cpu().numpy())
        tracking_left_fake_ee_actual.append(left_fake_ee_actual[0].detach().cpu().numpy())
        tracking_right_fake_ee_commanded.append(right_fake_ee_commanded[0].detach().cpu().numpy())
        tracking_right_fake_ee_actual.append(right_fake_ee_actual[0].detach().cpu().numpy())

        if desired_cube_pos_real is not None:
            actual_cube_pos_t = get_deformable_cube_center_world(cube)
            ref_cube_pos_t, ref_cube_quat_xyzw_t = get_cube_root_pos_quat_world_xyzw(reference_cube)

            actual_cube_pos = actual_cube_pos_t.detach().cpu().numpy().astype(float)

            # Deformable cube does not expose a rigid root orientation in the API you pasted.
            actual_cube_quat_xyzw = np.array([np.nan, np.nan, np.nan, np.nan], dtype=float)

            ref_cube_pos_ref = ref_cube_pos_t.detach().cpu().numpy().astype(float)
            ref_cube_pos_real = ref_cube_pos_ref - REFERENCE_ROBOT_WORLD_OFFSET.astype(float)

            ref_cube_quat_xyzw = ref_cube_quat_xyzw_t.detach().cpu().numpy().astype(float)

            cube_pos_err = actual_cube_pos - desired_cube_pos_real
            cube_pos_err_norm = float(np.linalg.norm(cube_pos_err))

            left_actual_rel_actual_cube = left_fake_ee_actual[0].detach().cpu().numpy().astype(float) - actual_cube_pos
            right_actual_rel_actual_cube = right_fake_ee_actual[0].detach().cpu().numpy().astype(float) - actual_cube_pos

            left_cmd_rel_desired_cube = left_fake_ee_commanded[0].detach().cpu().numpy().astype(float) - desired_cube_pos_real
            right_cmd_rel_desired_cube = right_fake_ee_commanded[0].detach().cpu().numpy().astype(float) - desired_cube_pos_real

            cube_rows.append({
                "step": int(count),
                "frame": int(frame),
                "time": float(sim_time),

                "retarget_root_x": float(root_pos[0]),
                "retarget_root_y": float(root_pos[1]),
                "retarget_root_z": float(root_pos[2]),

                "retarget_dice_x": float(dice_pos[0]),
                "retarget_dice_y": float(dice_pos[1]),
                "retarget_dice_z": float(dice_pos[2]),

                "dice_rel_to_root_x": float(dice_rel[0]),
                "dice_rel_to_root_y": float(dice_rel[1]),
                "dice_rel_to_root_z": float(dice_rel[2]),

                "desired_cube_x": float(desired_cube_pos_real[0]),
                "desired_cube_y": float(desired_cube_pos_real[1]),
                "desired_cube_z": float(desired_cube_pos_real[2]),

                "actual_cube_x": float(actual_cube_pos[0]),
                "actual_cube_y": float(actual_cube_pos[1]),
                "actual_cube_z": float(actual_cube_pos[2]),

                "reference_cube_real_x": float(ref_cube_pos_real[0]),
                "reference_cube_real_y": float(ref_cube_pos_real[1]),
                "reference_cube_real_z": float(ref_cube_pos_real[2]),

                "desired_quat_xyzw_x": float(desired_cube_quat_xyzw[0]),
                "desired_quat_xyzw_y": float(desired_cube_quat_xyzw[1]),
                "desired_quat_xyzw_z": float(desired_cube_quat_xyzw[2]),
                "desired_quat_xyzw_w": float(desired_cube_quat_xyzw[3]),

                "actual_quat_xyzw_x": float(actual_cube_quat_xyzw[0]),
                "actual_quat_xyzw_y": float(actual_cube_quat_xyzw[1]),
                "actual_quat_xyzw_z": float(actual_cube_quat_xyzw[2]),
                "actual_quat_xyzw_w": float(actual_cube_quat_xyzw[3]),

                "reference_quat_xyzw_x": float(ref_cube_quat_xyzw[0]),
                "reference_quat_xyzw_y": float(ref_cube_quat_xyzw[1]),
                "reference_quat_xyzw_z": float(ref_cube_quat_xyzw[2]),
                "reference_quat_xyzw_w": float(ref_cube_quat_xyzw[3]),

                "reference_desired_quat_abs_dot_xyzw": float(
                    quat_abs_dot_np(ref_cube_quat_xyzw, desired_cube_quat_xyzw)
                ),

                "cube_pos_err_x": float(cube_pos_err[0]),
                "cube_pos_err_y": float(cube_pos_err[1]),
                "cube_pos_err_z": float(cube_pos_err[2]),
                "cube_pos_err_norm": cube_pos_err_norm,

                "left_actual_rel_actual_cube_x": float(left_actual_rel_actual_cube[0]),
                "left_actual_rel_actual_cube_y": float(left_actual_rel_actual_cube[1]),
                "left_actual_rel_actual_cube_z": float(left_actual_rel_actual_cube[2]),

                "right_actual_rel_actual_cube_x": float(right_actual_rel_actual_cube[0]),
                "right_actual_rel_actual_cube_y": float(right_actual_rel_actual_cube[1]),
                "right_actual_rel_actual_cube_z": float(right_actual_rel_actual_cube[2]),

                "left_commanded_rel_desired_cube_x": float(left_cmd_rel_desired_cube[0]),
                "left_commanded_rel_desired_cube_y": float(left_cmd_rel_desired_cube[1]),
                "left_commanded_rel_desired_cube_z": float(left_cmd_rel_desired_cube[2]),

                "right_commanded_rel_desired_cube_x": float(right_cmd_rel_desired_cube[0]),
                "right_commanded_rel_desired_cube_y": float(right_cmd_rel_desired_cube[1]),
                "right_commanded_rel_desired_cube_z": float(right_cmd_rel_desired_cube[2]),
            })

            if args_cli.cube_debug_print_every > 0 and count % args_cli.cube_debug_print_every == 0:
                print(
                    f"[CubeReplay debug] step={count}, frame={frame} | "
                    f"desired(real)={desired_cube_pos_real} actual={actual_cube_pos} "
                    f"err={cube_pos_err} |norm|={cube_pos_err_norm:.4f} m | "
                    f"quat_dot_ref_des_xyzw={quat_abs_dot_np(ref_cube_quat_xyzw, desired_cube_quat_xyzw):.6f}"
                )

        if count % 120 == 0:
            print(f"[Replay] step={count}, frame={frame}")

        count += 1

    if cube_rows and args_cli.cube_debug_log_csv:
        with Path(args_cli.cube_debug_log_csv).open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(cube_rows[0].keys()))
            writer.writeheader()
            writer.writerows(cube_rows)

        print(f"[CubeReplay debug] wrote CSV: {args_cli.cube_debug_log_csv}")

        if args_cli.cube_debug_plot:
            show_cube_replay_debug_plot(cube_rows)

    if tracking_times:
        show_upper_body_tracking_plot(
            tracking_times,
            tracking_frames,
            tracking_q_des,
            tracking_q_actual,
            tracking_q_raw,
            robot.data.joint_names,
        )

        show_fake_ee_commanded_vs_actual_xyz_plot(
            tracking_times,
            tracking_frames,
            tracking_left_fake_ee_commanded,
            tracking_left_fake_ee_actual,
            tracking_right_fake_ee_commanded,
            tracking_right_fake_ee_actual,
        )


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    # -------------------------------------------------------------------------
    # Load motion data
    # -------------------------------------------------------------------------
    (
        robot_joint_qpos_np,
        motion_fps,
        root_qpos_np,
        object_qpos_np,
    ) = load_motion_file(args_cli.motion_file)

    # -------------------------------------------------------------------------
    # Build scene configuration
    # -------------------------------------------------------------------------
    scene_cfg = ReplaySceneCfg(
        num_envs=args_cli.num_envs,
        env_spacing=2.0,
    )

    apply_relative_cube_and_ground_table_layout(
        scene_cfg,
        root_qpos_np,
        object_qpos_np,
        frame=args_cli.motion_start_frame,
    )

    # -------------------------------------------------------------------------
    # Create simulation context
    # -------------------------------------------------------------------------
    sim_cfg = sim_utils.SimulationCfg(
        dt=float(args_cli.dt),
        device=args_cli.device,
        physics=physx_cfg,
    )

    sim = SimulationContext(sim_cfg)

    sim.set_camera_view(
        eye=[-2.6, 0.0, 1.5],
        target=[0.0, 0.0, 1.1],
    )

    # -------------------------------------------------------------------------
    # Import all scene assets
    #
    # InteractiveScene imports the preprocessed deformable USD here.
    # PhysX tensor views have not yet been initialized because sim.reset()
    # has not been called.
    # -------------------------------------------------------------------------
    scene = InteractiveScene(scene_cfg)

    # -------------------------------------------------------------------------
    # Finish all intended USD changes before initializing PhysX
    #
    # ReferenceRobot and ReferenceCube are visual/kinematic references and
    # should not participate in collision response.
    # -------------------------------------------------------------------------
    for env_id in range(args_cli.num_envs):
        disable_collisions_under_prim(
            f"/World/envs/env_{env_id}/ReferenceRobot"
        )

        disable_collisions_under_prim(
            f"/World/envs/env_{env_id}/ReferenceCube"
        )

    force_enable_dome_light()

    # -------------------------------------------------------------------------
    # Initialize PhysX and Isaac Lab tensor views
    #
    # Do this exactly once, after all structural USD work is complete.
    # Do not apply collision schemas, recook TetMeshes, or replace mesh prims
    # after this point.
    # -------------------------------------------------------------------------
    sim.reset()

    # -------------------------------------------------------------------------
    # Initialize robot states
    #
    # initial_robot_reset() also calls scene.reset(), which resets asset state
    # through Isaac Lab. It does not reconstruct the imported TetMesh.
    # -------------------------------------------------------------------------
    initial_robot_reset(scene)
    force_enable_dome_light()
    
    # Optional read-only diagnostic. Remove this once the imported asset has
    # been verified consistently.
    dump_usd_physics_attrs_under(
        "/World/envs/env_0/Cube"
    )

    # -------------------------------------------------------------------------
    # Pump the application once and perform one warm-up simulation step
    # -------------------------------------------------------------------------
    simulation_app.update()

    scene.write_data_to_sim()
    sim.step()
    scene.update(sim.get_physics_dt())

    # -------------------------------------------------------------------------
    # Start replay
    # -------------------------------------------------------------------------
    run(
        sim=sim,
        scene=scene,
        robot_joint_qpos_np=robot_joint_qpos_np,
        motion_fps=motion_fps,
        root_qpos_np=root_qpos_np,
        object_qpos_np=object_qpos_np,
    )


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    import sys
    import traceback

    try:
        main()

    except BaseException as exc:
        print(
            "\n[ERROR] Python exception caught:",
            repr(exc),
            flush=True,
        )

        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()

        raise

    finally:
        print(
            "[DEBUG] Closing simulation_app",
            flush=True,
        )

        simulation_app.close()
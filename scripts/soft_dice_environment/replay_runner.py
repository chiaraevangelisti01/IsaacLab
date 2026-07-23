from __future__ import annotations

import numpy as np
import torch

from replay_diagnostics import ReplayDiagnostics
from replay_utils import (
    CUSTOM_DICE_SCALE,
    FAKE_EE_OFFSET_IN_ELBOW,
    HOLOSOMA_TO_ISAAC_INDICES,
    REFERENCE_ROBOT_WORLD_OFFSET,
    desired_cube_pose_from_holosoma,
    fake_ee_point_from_elbow,
    find_elbow_body_id,
    get_cube_root_pos_quat_world_xyzw,
    get_deformable_cube_center_world,
    reset_deformable_cube_to_pose,
    set_joint_position_target_index,
    write_cube_pose_to_sim,
    write_joint_state_to_sim_index,
)


def replay_motion(
    simulation_app,
    sim,
    scene,
    robot_joint_qpos_np: np.ndarray,
    motion_fps: float,
    args,
    diagnostics: ReplayDiagnostics,
    root_qpos_np=None,
    object_qpos_np=None,
):
    robot = scene["robot"]
    reference_robot = scene["reference_robot"]
    cube = scene["cube"]
    reference_cube = scene["reference_cube"]

    sim_dt = sim.get_physics_dt()
    device = args.device
    num_envs = robot.data.default_joint_pos.shape[0]

    q_holo = torch.as_tensor(robot_joint_qpos_np, dtype=torch.float32, device=device)
    if robot.data.default_joint_pos.shape[1] != 19 or q_holo.shape[1] != 19:
        raise ValueError("Expected 19 H1 joints in Isaac and Holosoma motion.")

    reorder_idx = torch.tensor(HOLOSOMA_TO_ISAAC_INDICES, dtype=torch.long, device=device)
    q_traj = q_holo[:, reorder_idx]
    num_frames = q_traj.shape[0]

    print("\n[Replay settings]")
    print(f"  num_frames: {num_frames}")
    print(f"  motion_fps: {motion_fps}")
    print(f"  sim_dt: {sim_dt}")
    print(f"  replay_mode: {args.replay_mode}")
    print(f"  diagnostics: {'enabled' if diagnostics.enabled else 'disabled'}")

    if diagnostics.enabled:
        left_elbow_id, _ = find_elbow_body_id(robot, "left")
        right_elbow_id, _ = find_elbow_body_id(robot, "right")
        ref_left_elbow_id, _ = find_elbow_body_id(reference_robot, "left")
        ref_right_elbow_id, _ = find_elbow_body_id(reference_robot, "right")
    else:
        left_elbow_id = right_elbow_id = None
        ref_left_elbow_id = ref_right_elbow_id = None

    reference_offset_t = torch.as_tensor(
        REFERENCE_ROBOT_WORLD_OFFSET, dtype=torch.float32, device=device
    ).unsqueeze(0)
    robot_init_pos = np.asarray(scene["robot"].cfg.init_state.pos, dtype=np.float32)
    robot_init_quat = np.asarray(scene["robot"].cfg.init_state.rot, dtype=np.float32)

    first_frame = min(args.motion_start_frame, num_frames - 1)
    q0 = q_traj[first_frame].unsqueeze(0).repeat(num_envs, 1)
    if hasattr(robot.data, "soft_joint_pos_limits"):
        q0 = torch.clamp(q0, robot.data.soft_joint_pos_limits[..., 0], robot.data.soft_joint_pos_limits[..., 1])

    q0_vel = torch.zeros_like(q0)
    for articulation in (robot, reference_robot):
        write_joint_state_to_sim_index(articulation, q0, q0_vel)
        set_joint_position_target_index(articulation, q0)

    initial_cube_pos = None
    table_top_z = None
    if root_qpos_np is not None and object_qpos_np is not None:
        initial_cube_pos, initial_cube_quat, *_ = desired_cube_pose_from_holosoma(
            root_qpos_np,
            object_qpos_np,
            first_frame,
            robot_init_pos,
            robot_init_quat,
            reference_offset=None,
            apply_z_lift=False,
        )
        reset_deformable_cube_to_pose(cube, initial_cube_pos, initial_cube_quat)
        table_top_z = float(initial_cube_pos[2] - args.cube_size * CUSTOM_DICE_SCALE[2] / 2.0)
        diagnostics.print_deformable_geometry(
            cube,
            label="after deformable reset, before first simulation step",
            desired_center_np=initial_cube_pos,
            table_top_z=table_top_z,
            cube_size=args.cube_size,
            cube_scale_z=CUSTOM_DICE_SCALE[2],
        )

        ref_pos, ref_quat, *_ = desired_cube_pose_from_holosoma(
            root_qpos_np,
            object_qpos_np,
            first_frame,
            robot_init_pos,
            robot_init_quat,
            reference_offset=REFERENCE_ROBOT_WORLD_OFFSET,
            apply_z_lift=False,
        )
        write_cube_pose_to_sim(reference_cube, _pose_tensor(ref_pos, ref_quat, num_envs, device))

    scene.write_data_to_sim()
    sim.step()
    scene.update(sim_dt)

    if initial_cube_pos is not None:
        diagnostics.print_deformable_geometry(
            cube,
            label="after first simulation step",
            desired_center_np=initial_cube_pos,
            table_top_z=table_top_z,
            cube_size=args.cube_size,
            cube_scale_z=CUSTOM_DICE_SCALE[2],
        )

    count = 0
    while simulation_app.is_running() and count < args.steps:
        sim_time = count * sim_dt
        frame = args.motion_start_frame + int(sim_time * motion_fps * args.playback_speed)
        frame = frame % num_frames if args.motion_loop else min(frame, num_frames - 1)

        q_raw = q_traj[frame].unsqueeze(0).repeat(num_envs, 1)
        q_des = q_raw.clone()
        if hasattr(robot.data, "soft_joint_pos_limits"):
            limits = robot.data.soft_joint_pos_limits
            q_des = torch.clamp(q_des, limits[..., 0], limits[..., 1])

        if args.replay_mode == "state":
            q_vel = torch.zeros_like(q_des)
            write_joint_state_to_sim_index(robot, q_des, q_vel)
            set_joint_position_target_index(robot, q_des)
        else:
            set_joint_position_target_index(robot, q_des)

        ref_vel = torch.zeros_like(q_des)
        write_joint_state_to_sim_index(reference_robot, q_des, ref_vel)
        set_joint_position_target_index(reference_robot, q_des)

        desired_cube_pos = None
        desired_cube_quat = None
        if root_qpos_np is not None and object_qpos_np is not None:
            desired_cube_pos, desired_cube_quat, *_ = desired_cube_pose_from_holosoma(
                root_qpos_np,
                object_qpos_np,
                frame,
                robot_init_pos,
                robot_init_quat,
                reference_offset=None,
                apply_z_lift=False,
            )
            reference_pos = desired_cube_pos + REFERENCE_ROBOT_WORLD_OFFSET
            write_cube_pose_to_sim(
                reference_cube,
                _pose_tensor(reference_pos, desired_cube_quat, num_envs, device),
            )

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        if diagnostics.enabled:
            _record_diagnostics(
                diagnostics=diagnostics,
                step=count,
                frame=frame,
                sim_time=sim_time,
                robot=robot,
                reference_robot=reference_robot,
                cube=cube,
                reference_cube=reference_cube,
                q_raw=q_raw,
                q_des=q_des,
                desired_cube_pos=desired_cube_pos,
                desired_cube_quat=desired_cube_quat,
                left_elbow_id=left_elbow_id,
                right_elbow_id=right_elbow_id,
                ref_left_elbow_id=ref_left_elbow_id,
                ref_right_elbow_id=ref_right_elbow_id,
                reference_offset_t=reference_offset_t,
            )

        if count % 120 == 0:
            print(f"[Replay] step={count}, frame={frame}")
        count += 1

    diagnostics.finalize(robot.data.joint_names)


def _pose_tensor(position, quaternion_xyzw, num_envs, device):
    pose = torch.tensor(
        [[*position, *quaternion_xyzw]], dtype=torch.float32, device=device
    )
    return pose.repeat(num_envs, 1)


def _record_diagnostics(
    diagnostics,
    step,
    frame,
    sim_time,
    robot,
    reference_robot,
    cube,
    reference_cube,
    q_raw,
    q_des,
    desired_cube_pos,
    desired_cube_quat,
    left_elbow_id,
    right_elbow_id,
    ref_left_elbow_id,
    ref_right_elbow_id,
    reference_offset_t,
):
    left_cmd_world = fake_ee_point_from_elbow(reference_robot, ref_left_elbow_id, FAKE_EE_OFFSET_IN_ELBOW)
    right_cmd_world = fake_ee_point_from_elbow(reference_robot, ref_right_elbow_id, FAKE_EE_OFFSET_IN_ELBOW)
    left_cmd = left_cmd_world - reference_offset_t
    right_cmd = right_cmd_world - reference_offset_t
    left_actual = fake_ee_point_from_elbow(robot, left_elbow_id, FAKE_EE_OFFSET_IN_ELBOW)
    right_actual = fake_ee_point_from_elbow(robot, right_elbow_id, FAKE_EE_OFFSET_IN_ELBOW)

    diagnostics.record_tracking(
        sim_time=sim_time,
        frame=frame,
        q_raw=q_raw[0],
        q_des=q_des[0],
        q_actual=robot.data.joint_pos[0],
        left_ee_commanded=left_cmd[0],
        left_ee_actual=left_actual[0],
        right_ee_commanded=right_cmd[0],
        right_ee_actual=right_actual[0],
    )

    if desired_cube_pos is not None:
        actual_cube_pos = get_deformable_cube_center_world(cube)
        _, reference_cube_quat = get_cube_root_pos_quat_world_xyzw(reference_cube)
        diagnostics.record_cube(
            step=step,
            frame=frame,
            desired_cube_pos=desired_cube_pos,
            actual_cube_pos=actual_cube_pos,
            desired_cube_quat_xyzw=desired_cube_quat,
            reference_cube_quat_xyzw=reference_cube_quat,
        )

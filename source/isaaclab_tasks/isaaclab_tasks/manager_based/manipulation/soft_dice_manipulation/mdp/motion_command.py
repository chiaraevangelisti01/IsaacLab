from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

from pathlib import Path
import numpy as np
import torch


from isaaclab.managers import CommandTermCfg, CommandTerm
from isaaclab.utils.configclass import configclass
from isaaclab.utils.math import (
    quat_apply,
    quat_inv,
    quat_mul,
    yaw_quat,
)

from ..utils.motion_utils import (
    H1_HAND_OFFSETS_B,
    H1_HAND_PARENT_BODY_NAMES,
    H1_HAND_REFERENCE_NAMES,
    H1_TRACKED_BODY_NAMES,
    HOLOSOMA_TO_ISAAC_INDICES,
    desired_cube_pose_from_holosoma,
    load_tracking_motion_file,
    load_trajectory_phase_metadata,
    resolve_motion_paths,
    select_body_reference,
    select_body_values,
    transform_body_reference_to_fixed_root,
    transform_vector_reference_to_fixed_root,

)

from ..utils.deformable_utils import estimate_deformable_orientation_kabsch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from .motion_command_cfg import MotionCommandCfg


class MotionCommand(CommandTerm):
    """Time-indexed reference for H1 joints and the demonstrated dice pose."""

    cfg: MotionCommandCfg

    def __init__(self, cfg: MotionCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        self.robot = env.scene[cfg.asset_name]
        self.cube = env.scene[cfg.cube_name]

        hand_parent_ids, hand_parent_names = self.robot.find_bodies(
            H1_HAND_PARENT_BODY_NAMES,
            preserve_order=True,
        )

        if list(hand_parent_names) != H1_HAND_PARENT_BODY_NAMES:
            raise ValueError(
                "Isaac hand-parent body order mismatch. "
                f"Expected {H1_HAND_PARENT_BODY_NAMES}, got {hand_parent_names}"
            )

        self._hand_parent_body_ids = torch.as_tensor(
            hand_parent_ids,
            dtype=torch.long,
            device=self.device,
        )

        self._hand_offsets_b = torch.tensor(
            H1_HAND_OFFSETS_B,
            dtype=torch.float32,
            device=self.device,
        )

        self.motion_fps = 1.0 / float(env.step_dt)

        if cfg.playback_speed <= 0.0:
            raise ValueError("playback_speed must be > 0.")

        self.motion_paths = resolve_motion_paths(
            motion_file=cfg.motion_file,
            motion_dir=cfg.motion_dir,
            motion_pattern=cfg.motion_pattern,
        )
        self.motion_names = [path.stem for path in self.motion_paths]
        self.num_motions = len(self.motion_paths)

        motions = [self._prepare_motion(path) for path in self.motion_paths]

        self.source_motion_fps = float(motions[0]["fps"])
        self.has_body_reference = True
        self.has_hand_reference = True
        self.has_object_reference = True

        self._joint_pos_all = torch.cat([motion["joint_pos"] for motion in motions], dim=0)
        self._joint_vel_all = torch.cat([motion["joint_vel"] for motion in motions], dim=0)
        self._body_pos_all = torch.cat([motion["body_pos"] for motion in motions], dim=0)
        self._body_quat_all = torch.cat([motion["body_quat"] for motion in motions], dim=0)
        self._body_lin_vel_all = torch.cat([motion["body_lin_vel"] for motion in motions], dim=0)
        self._body_ang_vel_all = torch.cat([motion["body_ang_vel"] for motion in motions], dim=0)
        self._hand_pos_all = torch.cat([motion["hand_pos"] for motion in motions], dim=0)
        self._cube_pos_all = torch.cat([motion["cube_pos"] for motion in motions], dim=0)
        self._cube_quat_all = torch.cat([motion["cube_quat"] for motion in motions], dim=0)

        self._tracked_body_ids = motions[0]["tracked_body_ids"]

        self._motion_lengths = torch.tensor(
            [motion["num_frames"] for motion in motions],
            dtype=torch.long,
            device=self.device,
        )
        self._motion_offsets = torch.zeros(
            self.num_motions,
            dtype=torch.long,
            device=self.device,
        )

        if self.num_motions > 1:
            self._motion_offsets[1:] = torch.cumsum(self._motion_lengths[:-1], dim=0)

        self._start_frames = torch.full(
            (self.num_motions,),
            int(cfg.start_frame),
            dtype=torch.long,
            device=self.device,
        )

        self._landing_start_phases = None
        self._release_phases = None

        if cfg.phase_metadata_file:
            (
                self._landing_start_phases,
                self._release_phases,
            ) = load_trajectory_phase_metadata(
                metadata_file=cfg.phase_metadata_file,
                motion_paths=self.motion_paths,
                motion_lengths=self._motion_lengths,
                start_frames=self._start_frames,
                device=self.device,
            )

        self.num_frames = int(self._motion_lengths.max().item())
        self._start_frame = int(cfg.start_frame)

        self._motion_id = torch.zeros(
            self.num_envs,
            dtype=torch.long,
            device=self.device,
        )
        self._frame_idx = torch.full(
            (self.num_envs,),
            self._start_frame,
            dtype=torch.long,
            device=self.device,
        )
        self._motion_step = torch.zeros(
            self.num_envs,
            dtype=torch.long,
            device=self.device,
        )

        self._motion_episode_count = torch.zeros(
            self.num_motions,
            dtype=torch.long,
            device=self.device,
        )
        self._motion_transition_count = torch.zeros(
            self.num_motions,
            dtype=torch.long,
            device=self.device,
        )

        self._cached_cube_quat = None
        self._cached_cube_quat_step = -1

        # -------------------------------------------------------------------------
        # Episode tracking metrics
        # -------------------------------------------------------------------------
        self.metrics["phase"] = torch.zeros(self.num_envs, device=self.device)

        self.metrics["mean_joint_pos_rmse_rad"] = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )

        self.metrics["mean_joint_vel_rmse_rad_s"] = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )

        self.metrics["mean_cube_pos_error_m"] = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )

        self.metrics["final_joint_pos_rmse_rad"] = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )

        self.metrics["final_cube_pos_error_m"] = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )

        # Running sums used to compute episode means.
        self._joint_pos_rmse_sum = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )

        self._joint_vel_rmse_sum = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )

        self._cube_pos_error_sum = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )

        self._metric_step_count = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )

    def _prepare_motion(self, path: Path) -> dict:
        """Load and prepare one fully converted tracking motion."""

        (
            joint_qpos_np,
            joint_qvel_np,
            source_fps,
            root_qpos_np,
            object_qpos_np,
            body_names,
            body_pos_w_np,
            body_quat_w_np,
            body_lin_vel_w_np,
            body_ang_vel_w_np,
        ) = load_tracking_motion_file(str(path))

        if not np.isclose(self.motion_fps, source_fps, rtol=0.0, atol=1.0e-6):
            raise ValueError(
                f"Reference {path} is {source_fps} Hz but the environment runs at "
                f"{self.motion_fps} Hz. Convert the motion to the correct output FPS first."
            )

        num_frames = int(joint_qpos_np.shape[0])

        if not 0 <= self.cfg.start_frame < num_frames:
            raise ValueError(
                f"start_frame={self.cfg.start_frame} outside [0, {num_frames - 1}] for {path}"
            )

        reorder_idx = np.asarray(HOLOSOMA_TO_ISAAC_INDICES, dtype=np.int64)
        joint_pos = torch.as_tensor(
            joint_qpos_np[:, reorder_idx],
            dtype=torch.float32,
            device=self.device,
        )
        joint_vel = torch.as_tensor(
            joint_qvel_np[:, reorder_idx],
            dtype=torch.float32,
            device=self.device,
        )

        if joint_pos.shape[1] != self.robot.num_joints:
            raise ValueError(
                f"Reference {path} has {joint_pos.shape[1]} joints after reordering; "
                f"robot has {self.robot.num_joints}."
            )

        tracked_body_ids, body_pos, body_quat, body_lin_vel, body_ang_vel = (
            self._prepare_body_reference(
                body_names=body_names,
                body_pos_w=body_pos_w_np,
                body_quat_w=body_quat_w_np,
                body_lin_vel_w=body_lin_vel_w_np,
                body_ang_vel_w=body_ang_vel_w_np,
                root_qpos=root_qpos_np,
            )
        )

        hand_pos = self._prepare_hand_position_reference(
            body_names=body_names,
            body_pos_w=body_pos_w_np,
            body_quat_w=body_quat_w_np,
            root_qpos=root_qpos_np,
        )

        robot_pos = np.asarray(self.robot.cfg.init_state.pos, dtype=np.float32)
        robot_quat = np.asarray(self.robot.cfg.init_state.rot, dtype=np.float32)
        cube_pos = np.zeros((num_frames, 3), dtype=np.float32)
        cube_quat = np.zeros((num_frames, 4), dtype=np.float32)

        for frame in range(num_frames):
            pos, quat, *_ = desired_cube_pose_from_holosoma(
                root_qpos_np,
                object_qpos_np,
                frame,
                robot_pos,
                robot_quat,
            )
            cube_pos[frame] = pos
            cube_quat[frame] = quat

        return {
            "fps": float(source_fps),
            "num_frames": num_frames,
            "joint_pos": joint_pos,
            "joint_vel": joint_vel,
            "tracked_body_ids": tracked_body_ids,
            "body_pos": body_pos,
            "body_quat": body_quat,
            "body_lin_vel": body_lin_vel,
            "body_ang_vel": body_ang_vel,
            "hand_pos": hand_pos,
            "cube_pos": torch.as_tensor(cube_pos, dtype=torch.float32, device=self.device),
            "cube_quat": torch.as_tensor(cube_quat, dtype=torch.float32, device=self.device),
        }
    
    def _prepare_body_reference(
        self,
        body_names,
        body_pos_w,
        body_quat_w,
        body_lin_vel_w,
        body_ang_vel_w,
        root_qpos,
    ):
            """Prepare Holosoma Cartesian references for the fixed-base Isaac H1."""

            if (
                body_names is None
                or body_pos_w is None
                or body_quat_w is None
            ):
                return None, None, None

            if root_qpos is None:
                raise ValueError(
                    "Cartesian body reference requires a root trajectory."
                )

            body_pos_w, body_quat_w = select_body_reference(
                body_names=body_names,
                body_pos_w=body_pos_w,
                body_quat_w=body_quat_w,
                tracked_body_names=H1_TRACKED_BODY_NAMES,
            )

            body_lin_vel_w = select_body_values(
                body_names=body_names,
                values=body_lin_vel_w,
                tracked_body_names=H1_TRACKED_BODY_NAMES,
            )
            body_ang_vel_w = select_body_values(
                body_names=body_names,
                values=body_ang_vel_w,
                tracked_body_names=H1_TRACKED_BODY_NAMES,
            )

            isaac_body_ids, isaac_body_names = self.robot.find_bodies(
                H1_TRACKED_BODY_NAMES,
                preserve_order=True,
            )

            if list(isaac_body_names) != H1_TRACKED_BODY_NAMES:
                raise ValueError(
                    f"Isaac body order mismatch. "
                    f"Expected {H1_TRACKED_BODY_NAMES}, "
                    f"got {isaac_body_names}"
                )

            body_pos_ref, body_quat_ref = (
                transform_body_reference_to_fixed_root(
                    body_pos_w=body_pos_w,
                    body_quat_w=body_quat_w,
                    root_qpos=root_qpos,
                    fixed_root_pos=np.asarray(
                        self.robot.cfg.init_state.pos,
                        dtype=np.float32,
                    ),
                    fixed_root_quat_xyzw=np.asarray(
                        self.robot.cfg.init_state.rot,
                        dtype=np.float32,
                    ),
                )
            )

            body_lin_vel_ref = transform_vector_reference_to_fixed_root(
                vector_w=body_lin_vel_w,
                root_qpos=root_qpos,
                fixed_root_quat_xyzw=np.asarray(self.robot.cfg.init_state.rot, dtype=np.float32),
            )
            body_ang_vel_ref = transform_vector_reference_to_fixed_root(
                vector_w=body_ang_vel_w,
                root_qpos=root_qpos,
                fixed_root_quat_xyzw=np.asarray(self.robot.cfg.init_state.rot, dtype=np.float32),
            )

            body_ids = torch.as_tensor(
                isaac_body_ids,
                dtype=torch.long,
                device=self.device,
            )

            body_pos_ref = torch.as_tensor(
                body_pos_ref,
                dtype=torch.float32,
                device=self.device,
            )

            body_quat_ref = torch.as_tensor(
                body_quat_ref,
                dtype=torch.float32,
                device=self.device,
            )

            return (
                body_ids,
                torch.as_tensor(body_pos_ref, dtype=torch.float32, device=self.device),
                torch.as_tensor(body_quat_ref, dtype=torch.float32, device=self.device),
                torch.as_tensor(body_lin_vel_ref, dtype=torch.float32, device=self.device),
                torch.as_tensor(body_ang_vel_ref, dtype=torch.float32, device=self.device),
            )

    def _prepare_hand_position_reference(
        self,
        body_names,
        body_pos_w,
        body_quat_w,
        root_qpos,
    ):
        """Prepare virtual-hand position references for fixed-base Isaac H1."""

        if (
            body_names is None
            or body_pos_w is None
            or body_quat_w is None
        ):
            return None

        if root_qpos is None:
            raise ValueError(
                "Cartesian hand reference requires a root trajectory."
            )

        # Hands already exist as bodies in the converted Holosoma trajectory.
        hand_pos_w, hand_quat_w = select_body_reference(
            body_names=body_names,
            body_pos_w=body_pos_w,
            body_quat_w=body_quat_w,
            tracked_body_names=H1_HAND_REFERENCE_NAMES,
        )

        hand_pos_ref, _ = transform_body_reference_to_fixed_root(
            body_pos_w=hand_pos_w,
            body_quat_w=hand_quat_w,
            root_qpos=root_qpos,
            fixed_root_pos=np.asarray(
                self.robot.cfg.init_state.pos,
                dtype=np.float32,
            ),
            fixed_root_quat_xyzw=np.asarray(
                self.robot.cfg.init_state.rot,
                dtype=np.float32,
            ),
        )

        return torch.as_tensor(
            hand_pos_ref,
            dtype=torch.float32,
            device=self.device,
        )
    
    def aligned_body_reference(
        self,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Align body references to the current torso yaw.
        """

        body_pos_ref = self.body_pos
        body_quat_ref = self.body_quat

        torso_index = H1_TRACKED_BODY_NAMES.index(
            "torso_link"
        )

        # --------------------------------------------------------------
        # Reference torso pose.
        # --------------------------------------------------------------
        torso_pos_ref = body_pos_ref[
            :, torso_index, :
        ]
        torso_quat_ref = body_quat_ref[
            :, torso_index, :
        ]

        # --------------------------------------------------------------
        # Current actual torso orientation.
        # --------------------------------------------------------------
        torso_quat_robot = self.robot_body_quat[
            :, torso_index, :
        ]

        # --------------------------------------------------------------
        # BeyondMimic-style orientation alignment, reduced to yaw only:
        # --------------------------------------------------------------
        delta_yaw = yaw_quat(
            quat_mul(
                torso_quat_robot,
                quat_inv(torso_quat_ref),
            )
        )

        # Apply the same yaw rotation to every tracked body.
        num_bodies = body_pos_ref.shape[1]

        delta_yaw_bodies = delta_yaw[:, None, :].expand(
            -1,
            num_bodies,
            -1,
        )

        # --------------------------------------------------------------
        # Rotate body positions ABOUT THE REFERENCE TORSO.
        # --------------------------------------------------------------
        body_pos_from_torso = (
            body_pos_ref
            - torso_pos_ref[:, None, :]
        )

        body_pos_aligned = (
            torso_pos_ref[:, None, :]
            + quat_apply(
                delta_yaw_bodies,
                body_pos_from_torso,
            )
        )

        # --------------------------------------------------------------
        # Orientations receive the same yaw alignment.
        # --------------------------------------------------------------
        body_quat_aligned = quat_mul(
            delta_yaw_bodies,
            body_quat_ref,
        )

        return body_pos_aligned, body_quat_aligned

    def aligned_hand_reference(
        self,
    ) -> torch.Tensor:
        """Align virtual-hand references to the current torso yaw."""

        hand_pos_ref = self.hand_pos

        torso_index = H1_TRACKED_BODY_NAMES.index(
            "torso_link"
        )

        torso_pos_ref = self.body_pos[
            :, torso_index, :
        ]

        torso_quat_ref = self.body_quat[
            :, torso_index, :
        ]

        torso_quat_robot = self.robot_body_quat[
            :, torso_index, :
        ]

        delta_yaw = yaw_quat(
            quat_mul(
                torso_quat_robot,
                quat_inv(torso_quat_ref),
            )
        )

        delta_yaw_hands = delta_yaw[
            :, None, :
        ].expand(
            -1,
            len(H1_HAND_REFERENCE_NAMES),
            -1,
        )

        hand_pos_from_torso = (
            hand_pos_ref
            - torso_pos_ref[:, None, :]
        )

        return (
            torso_pos_ref[:, None, :]
            + quat_apply(
                delta_yaw_hands,
                hand_pos_from_torso,
            )
        )

    @property
    def command(self) -> torch.Tensor:
        return torch.cat((self.joint_pos, self.joint_vel), dim=-1)

    @property
    def frame_idx(self) -> torch.Tensor:
        return self._frame_idx

    @property
    def motion_id(self) -> torch.Tensor:
        return self._motion_id

    @property
    def motion_lengths(self) -> torch.Tensor:
        return self._motion_lengths

    @property
    def start_frame(self) -> int:
        return self._start_frame

    @property
    def joint_pos(self) -> torch.Tensor:
        return self._joint_pos_all[self._global_frame_idx()]

    @property
    def joint_vel(self) -> torch.Tensor:
        return self._joint_vel_all[self._global_frame_idx()]

    @property
    def cube_pos(self) -> torch.Tensor:
        return self._cube_pos_all[self._global_frame_idx()]

    @property
    def cube_quat(self) -> torch.Tensor:
        return self._cube_quat_all[self._global_frame_idx()]

    @property
    def final_cube_quat(
        self,
    ) -> torch.Tensor:
        """Final demonstrated cube orientation for each active motion."""

        final_global_idx = (
            self._motion_offsets[
                self._motion_id
            ]
            + self._motion_lengths[
                self._motion_id
            ]
            - 1
        )

        return self._cube_quat_all[
            final_global_idx
        ]

    @property
    def body_pos(self) -> torch.Tensor:
        return self._body_pos_all[self._global_frame_idx()]

    @property
    def body_quat(self) -> torch.Tensor:
        return self._body_quat_all[self._global_frame_idx()]

    @property
    def hand_pos(self) -> torch.Tensor:
        return self._hand_pos_all[self._global_frame_idx()]

    @property
    def body_lin_vel(self) -> torch.Tensor:
        return self._body_lin_vel_all[self._global_frame_idx()]

    @property
    def body_ang_vel(self) -> torch.Tensor:
        return self._body_ang_vel_all[self._global_frame_idx()]

    @property
    def simulator_cube_pos(self) -> torch.Tensor:
        """Current deformable-dice centroid in environment-local coordinates."""

        return (
            self.cube.data.root_pos_w.torch
            - self._env.scene.env_origins
        )

    @property
    def simulator_cube_lin_vel(self) -> torch.Tensor:
        """Current deformable-dice centroid velocity in world axes."""

        return self.cube.data.root_vel_w.torch


    @property
    def simulator_cube_quat(self) -> torch.Tensor:
        """Current bulk orientation of the deformable dice."""

        step = self._env.common_step_counter

        if self._cached_cube_quat_step != step:
            reference_nodal_pos = (
                self.cube.data.default_nodal_state_w.torch[..., :3]
            )
            current_nodal_pos = self.cube.data.nodal_pos_w.torch

            self._cached_cube_quat = estimate_deformable_orientation_kabsch(
                reference_nodal_pos=reference_nodal_pos,
                current_nodal_pos=current_nodal_pos,
            )
            self._cached_cube_quat_step = step

        return self._cached_cube_quat

    @property
    def finished(self) -> torch.Tensor:
        if self.cfg.loop:
            return torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

        return self._frame_idx >= self._motion_lengths[self._motion_id] - 1

    @property
    def robot_body_pos(self) -> torch.Tensor:
        """Current tracked-body positions in environment-local coordinates."""
        if not self.has_body_reference:
            raise RuntimeError(
                "Motion does not contain Cartesian body references."
            )

        body_pos_w = self.robot.data.body_link_pose_w.torch[
            :, self._tracked_body_ids, :3
        ]

        return (
            body_pos_w
            - self._env.scene.env_origins[:, None, :]
        )


    @property
    def robot_body_quat(self) -> torch.Tensor:
        """Current tracked-body orientations in XYZW convention."""
        if not self.has_body_reference:
            raise RuntimeError(
                "Motion does not contain Cartesian body references."
            )

        return self.robot.data.body_link_pose_w.torch[
            :, self._tracked_body_ids, 3:7
        ]

    @property
    def robot_hand_pos(self) -> torch.Tensor:
        """Current virtual-hand positions on the simulated H1."""

        parent_pose_w = self.robot.data.body_link_pose_w.torch[
            :, self._hand_parent_body_ids, :
        ]

        parent_pos_w = parent_pose_w[..., :3]
        parent_quat_w = parent_pose_w[..., 3:7]

        offsets_b = self._hand_offsets_b[
            None, :, :
        ].expand(
            parent_pos_w.shape[0],
            -1,
            -1,
        )

        offsets_w = quat_apply(
            parent_quat_w,
            offsets_b,
        )

        hand_pos_w = (
            parent_pos_w
            + offsets_w
        )

        return (
            hand_pos_w
            - self._env.scene.env_origins[:, None, :]
        )
    

    @property
    def robot_body_lin_vel(self) -> torch.Tensor:
        """Current tracked-link linear velocities in world axes."""
        if not self.has_body_reference:
            raise RuntimeError(
                "Motion does not contain Cartesian body references."
            )

        return self.robot.data.body_link_lin_vel_w.torch[
            :, self._tracked_body_ids, :
        ]


    @property
    def robot_body_ang_vel(self) -> torch.Tensor:
        """Current tracked-link angular velocities in world axes."""
        if not self.has_body_reference:
            raise RuntimeError(
                "Motion does not contain Cartesian body references."
            )

        return self.robot.data.body_link_ang_vel_w.torch[
            :, self._tracked_body_ids, :
        ]

    @property
    def phase(self) -> torch.Tensor:
        motion_ids = self._motion_id

        start_frames = self._start_frames[
            motion_ids
        ]

        phase_length = (
            self._motion_lengths[motion_ids]
            - 1
            - start_frames
        ).clamp_min(1)

        return (
            self._frame_idx - start_frames
        ).float() / phase_length.float()


    @property
    def landing_start_phase(self) -> torch.Tensor:
        if self._landing_start_phases is None:
            raise RuntimeError(
                "No trajectory phase metadata loaded."
            )

        return self._landing_start_phases[
            self._motion_id
        ]


    @property
    def release_phase(self) -> torch.Tensor:
        if self._release_phases is None:
            raise RuntimeError(
                "No trajectory phase metadata loaded."
            )

        return self._release_phases[
            self._motion_id
        ]

    def start_joint_pos(self, env_ids: torch.Tensor) -> torch.Tensor:
        return self._joint_pos_all[self._start_global_idx(env_ids)]

    def start_joint_vel(self, env_ids: torch.Tensor) -> torch.Tensor:
        return self._joint_vel_all[self._start_global_idx(env_ids)]

    def start_cube_pos(self, env_ids: torch.Tensor) -> torch.Tensor:
        return self._cube_pos_all[self._start_global_idx(env_ids)]

    def start_cube_quat(self, env_ids: torch.Tensor) -> torch.Tensor:
        return self._cube_quat_all[self._start_global_idx(env_ids)]

    def _global_frame_idx(self) -> torch.Tensor:
        return self._motion_offsets[self._motion_id] + self._frame_idx

    def _start_global_idx(self, env_ids: torch.Tensor) -> torch.Tensor:
        motion_ids = self._motion_id[env_ids]
        return self._motion_offsets[motion_ids] + self._start_frames[motion_ids]

    def sample_motions(self, env_ids: torch.Tensor) -> None:
        """Assign one reference trajectory per environment for the next episode."""

        if env_ids.numel() == 0:
            return

        if self.num_motions == 1:
            motion_ids = torch.zeros(
                env_ids.numel(),
                dtype=torch.long,
                device=self.device,
            )
        else:
            motion_ids = torch.randint(
                low=0,
                high=self.num_motions,
                size=(env_ids.numel(),),
                device=self.device,
            )

        self._motion_id[env_ids] = motion_ids
        self._frame_idx[env_ids] = self._start_frames[motion_ids]
        self._motion_step[env_ids] = 0
        self._motion_episode_count += torch.bincount(
            motion_ids,
            minlength=self.num_motions,
        )
        
    def set_motion_by_name(
        self,
        motion_name: str,
        env_ids: torch.Tensor | None = None,
    ) -> None:
        """Select a specific reference motion without resetting simulation state."""

        if motion_name not in self.motion_names:
            raise ValueError(
                f"Unknown motion '{motion_name}'. "
                f"Available motions: {self.motion_names}"
            )

        motion_id = self.motion_names.index(motion_name)

        if env_ids is None:
            env_ids = torch.arange(
                self.num_envs,
                dtype=torch.long,
                device=self.device,
            )
        else:
            env_ids = torch.as_tensor(
                env_ids,
                dtype=torch.long,
                device=self.device,
            ).reshape(-1)

        motion_ids = torch.full(
            (env_ids.numel(),),
            motion_id,
            dtype=torch.long,
            device=self.device,
        )

        self._motion_id[env_ids] = motion_ids
        self._frame_idx[env_ids] = self._start_frames[motion_ids]
        self._motion_step[env_ids] = 0
    
    def motion_name(self, motion_id: int) -> str:
        return self.motion_names[motion_id]

    def _update_metrics(self):
        # ------------------------------------------------------------------
        # Motion phase
        # ------------------------------------------------------------------

        motion_ids = self._motion_id

        self.metrics["phase"][:] = self.phase

        self._motion_transition_count += torch.bincount(
            motion_ids,
            minlength=self.num_motions,
        )

        # ------------------------------------------------------------------
        # Reference-tracking errors
        # ------------------------------------------------------------------

        (
            joint_pos_rmse,
            joint_vel_rmse,
            cube_pos_error,
        ) = self._compute_tracking_errors()

        self._joint_pos_rmse_sum += joint_pos_rmse
        self._joint_vel_rmse_sum += joint_vel_rmse
        self._cube_pos_error_sum += cube_pos_error

        self._metric_step_count += 1.0

    def _resample_command(self, env_ids):
        motion_ids = self._motion_id[env_ids]
        self._frame_idx[env_ids] = self._start_frames[motion_ids]
        self._motion_step[env_ids] = 0

    def _update_command(self):
        self._motion_step += 1

        elapsed_s = self._motion_step.float() * float(self._env.step_dt)
        offset = torch.floor(
            elapsed_s * self.motion_fps * float(self.cfg.playback_speed) + 1.0e-6
        ).long()

        motion_ids = self._motion_id
        start_frames = self._start_frames[motion_ids]
        motion_lengths = self._motion_lengths[motion_ids]

        if self.cfg.loop:
            span = motion_lengths - start_frames
            self._frame_idx[:] = start_frames + torch.remainder(offset, span)
        else:
            self._frame_idx[:] = torch.minimum(
                start_frames + offset,
                motion_lengths - 1,
            )

    def _compute_tracking_errors(self):
        """Compute per-environment reference-tracking errors."""

        # Joint position RMSE across joints.
        joint_pos_error = self.robot.data.joint_pos.torch - self.joint_pos

        joint_pos_rmse = torch.sqrt(
            torch.mean(torch.square(joint_pos_error), dim=-1)
        )

        # Joint velocity RMSE across joints.
        joint_vel_error = self.robot.data.joint_vel.torch - self.joint_vel

        joint_vel_rmse = torch.sqrt(
            torch.mean(torch.square(joint_vel_error), dim=-1)
        )

        # Deformable cube center.
        cube_center_w = self.cube.data.nodal_pos_w.torch.mean(dim=1)

        # Reference cube position is stored in environment-local coordinates.
        cube_center_e = cube_center_w - self._env.scene.env_origins

        cube_pos_error = torch.linalg.norm(
            self.simulator_cube_pos - self.cube_pos,
            dim=-1,
        )

        return joint_pos_rmse, joint_vel_rmse, cube_pos_error

    def reset(self, env_ids=None):
        """Finalize episode tracking metrics before resetting the command."""

        if env_ids is None:
            env_ids = slice(None)

        count = self._metric_step_count[env_ids].clamp_min(1.0)

        self.metrics["mean_joint_pos_rmse_rad"][env_ids] = (
            self._joint_pos_rmse_sum[env_ids] / count
        )

        self.metrics["mean_joint_vel_rmse_rad_s"][env_ids] = (
            self._joint_vel_rmse_sum[env_ids] / count
        )

        self.metrics["mean_cube_pos_error_m"][env_ids] = (
            self._cube_pos_error_sum[env_ids] / count
        )

        # super().reset() reads self.metrics and returns them to CommandManager.
        extras = super().reset(env_ids)

        episode_total = self._motion_episode_count.sum().clamp_min(1)
        transition_total = self._motion_transition_count.sum().clamp_min(1)

        for motion_id, motion_name in enumerate(self.motion_names):
            extras[f"pool/{motion_name}/episode_fraction"] = (
                self._motion_episode_count[motion_id].float() / episode_total
            ).item()
            extras[f"pool/{motion_name}/transition_fraction"] = (
                self._motion_transition_count[motion_id].float() / transition_total
            ).item()

        # Clear accumulators for the next episode.
        self._joint_pos_rmse_sum[env_ids] = 0.0
        self._joint_vel_rmse_sum[env_ids] = 0.0
        self._cube_pos_error_sum[env_ids] = 0.0
        self._metric_step_count[env_ids] = 0.0

        return extras

    def record_terminal_metrics(self, env_ids: torch.Tensor):
        """Record metrics from the physical state that actually ended the episode."""

        if env_ids.numel() == 0:
            return

        (
            joint_pos_rmse,
            _,
            cube_pos_error,
        ) = self._compute_tracking_errors()

        self.metrics["final_joint_pos_rmse_rad"][env_ids] = (
            joint_pos_rmse[env_ids]
        )

        self.metrics["final_cube_pos_error_m"][env_ids] = (
            cube_pos_error[env_ids]
        )

@configclass
class MotionCommandCfg(CommandTermCfg):
    """Configuration for a time-indexed Holosoma reference motion."""

    class_type: type | str = "{DIR}.motion_command:MotionCommand"

    resampling_time_range: tuple[float, float] = (
        1.0e9,
        1.0e9,
    )

    asset_name: str = "robot"
    cube_name: str = "cube"

    motion_file: str = ""
    motion_dir: str = ""
    motion_pattern: str = "*.npz"

    phase_metadata_file: str = ""

    start_frame: int = 0
    playback_speed: float = 1.0
    loop: bool = False
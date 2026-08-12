from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import numpy as np
import torch


from isaaclab.managers import CommandTermCfg, CommandTerm
from isaaclab.utils.configclass import configclass
from isaaclab.utils.math import (
    axis_angle_from_quat,
    quat_apply,
    quat_inv,
    quat_mul,
    quat_unique,
    yaw_quat,
)

from .motion_utils import (
    H1_HAND_OFFSETS_B,
    H1_HAND_PARENT_BODY_NAMES,
    H1_HAND_REFERENCE_NAMES,
    H1_TRACKED_BODY_NAMES,
    HOLOSOMA_TO_ISAAC_INDICES,
    desired_cube_pose_from_holosoma,
    load_motion_file,
    resample_motion_to_fps,
    select_body_reference,
    transform_body_reference_to_fixed_root,
)

from .deformable_utils import estimate_deformable_orientation_kabsch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from .motion_command_cfg import MotionCommandCfg


class MotionCommand(CommandTerm):
    """Time-indexed reference for H1 joints and the demonstrated dice pose."""

    cfg: MotionCommandCfg

    def __init__(self, cfg: MotionCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        if not cfg.motion_file:
            raise ValueError(
                "MotionCommandCfg.motion_file is empty. Pass it through Hydra, e.g. "
                "env.commands.motion.motion_file=/absolute/path/to/motion.npz"
            )

        self.robot = env.scene[cfg.asset_name]
        self.cube = env.scene[cfg.cube_name]

        (
            joint_qpos_np,
            joint_qvel_np,
            source_fps,
            root_qpos_np,
            object_qpos_np,
            body_names,
            body_pos_w_np,
            body_quat_w_np,
        ) = load_motion_file(cfg.motion_file)

        (
            self._tracked_body_ids,
            self._body_pos_all,
            self._body_quat_all,
        ) = self._prepare_body_reference(
            body_names=body_names,
            body_pos_w=body_pos_w_np,
            body_quat_w=body_quat_w_np,
            root_qpos=root_qpos_np,
        )

        self.has_body_reference = (
            self._body_pos_all is not None
        )

        self._hand_pos_all = self._prepare_hand_position_reference(
            body_names=body_names,
            body_pos_w=body_pos_w_np,
            body_quat_w=body_quat_w_np,
            root_qpos=root_qpos_np,
        )

        self.has_hand_reference = (
            self._hand_pos_all is not None
        )

        hand_parent_ids, hand_parent_names = self.robot.find_bodies(
            H1_HAND_PARENT_BODY_NAMES,
            preserve_order=True,
        )

        if list(hand_parent_names) != H1_HAND_PARENT_BODY_NAMES:
            raise ValueError(
                "Isaac hand-parent body order mismatch. "
                f"Expected {H1_HAND_PARENT_BODY_NAMES}, "
                f"got {hand_parent_names}"
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
        self.source_motion_fps = float(source_fps)

        source_num_frames = int(joint_qpos_np.shape[0])

        if not 0 <= cfg.start_frame < source_num_frames:
            raise ValueError(
                f"start_frame={cfg.start_frame} outside "
                f"[0, {source_num_frames - 1}]"
            )

        self.motion_fps = 1.0 / float(env.step_dt)

        need_resampling = not np.isclose(self.motion_fps, self.source_motion_fps, rtol = 0.0, atol = 1.0e-6)

        if body_pos_w_np is not None and need_resampling:
            raise ValueError(
                "Converted Cartesian body references must currently "
                "already match the environment control frequency."
            )

        if need_resampling:

            (
                joint_qpos_np,
                root_qpos_np,
                object_qpos_np,
            ) = resample_motion_to_fps(
                joint_qpos=joint_qpos_np,
                source_fps=self.source_motion_fps,
                target_fps=self.motion_fps,
                root_qpos=root_qpos_np,
                object_qpos=object_qpos_np,
            )

        # ---------------------------------------------------------------------
        # Fixed-root Cartesian body velocities.
        # ---------------------------------------------------------------------
        if self.has_body_reference:
            self._body_lin_vel_all = self._differentiate(
                self._body_pos_all,
                self.motion_fps,
            )

            self._body_ang_vel_all = self._differentiate_quat(
                self._body_quat_all,
                self.motion_fps,
            )
        else:
            self._body_lin_vel_all = None
            self._body_ang_vel_all = None


        reorder_idx = np.asarray(
            HOLOSOMA_TO_ISAAC_INDICES,
            dtype=np.int64,
        )

        q_np = np.asarray(
            joint_qpos_np[:, reorder_idx],
            dtype=np.float32,
        )

        if q_np.shape[1] != self.robot.num_joints:
            raise ValueError(
                f"Reference has {q_np.shape[1]} joints after reordering; robot has {self.robot.num_joints}."
            )

        if joint_qvel_np is not None and not need_resampling:
            qd_np = np.asarray(
                joint_qvel_np[:, reorder_idx],
                dtype=np.float32,
            )
        else:
            qd_np = self._differentiate(q_np, self.motion_fps)

        self._joint_pos_all = torch.as_tensor(q_np, device=self.device)
        self._joint_vel_all = torch.as_tensor(qd_np, device=self.device)
        self.num_frames = int(q_np.shape[0])

        #Convert it to the nearest frame on the resampled reference grid.
        start_time_s = (
            float(cfg.start_frame)
            / self.source_motion_fps
        )

        self._start_frame = min(
            int(round(start_time_s * self.motion_fps)),
            self.num_frames - 1,
        )

        if cfg.playback_speed <= 0.0:
            raise ValueError("playback_speed must be > 0.")

        self.has_object_reference = root_qpos_np is not None and object_qpos_np is not None
        if self.has_object_reference:
            robot_pos = np.asarray(self.robot.cfg.init_state.pos, dtype=np.float32)
            robot_quat = np.asarray(self.robot.cfg.init_state.rot, dtype=np.float32)

            cube_pos = np.zeros((self.num_frames, 3), dtype=np.float32)
            cube_quat = np.zeros((self.num_frames, 4), dtype=np.float32)

            for frame in range(self.num_frames):
                pos, quat, *_ = desired_cube_pose_from_holosoma(
                    root_qpos_np,
                    object_qpos_np,
                    frame,
                    robot_pos,
                    robot_quat,
                )
                cube_pos[frame] = pos
                cube_quat[frame] = quat

            self._cube_pos_all = torch.as_tensor(cube_pos, dtype=torch.float32, device=self.device)
            self._cube_quat_all = torch.as_tensor(cube_quat, dtype=torch.float32, device=self.device)
        else:
            self._cube_pos_all = None
            self._cube_quat_all = None

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

    def _prepare_body_reference(
            self,
            body_names,
            body_pos_w,
            body_quat_w,
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
                body_pos_ref,
                body_quat_ref,
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

    @staticmethod
    def _differentiate(
        values: np.ndarray | torch.Tensor,
        fps: float,
    ) -> np.ndarray | torch.Tensor:
        """Finite-difference values along the time dimension."""

        if values.shape[0] <= 1:
            if isinstance(values, torch.Tensor):
                return torch.zeros_like(values)
            return np.zeros_like(values, dtype=np.float32)

        dt = 1.0 / float(fps)

        if isinstance(values, torch.Tensor):
            derivative = torch.zeros_like(values)
        else:
            derivative = np.zeros_like(
                values,
                dtype=np.float32,
            )

        derivative[1:-1] = (
            values[2:] - values[:-2]
        ) / (2.0 * dt)

        derivative[0] = (
            values[1] - values[0]
        ) / dt

        derivative[-1] = (
            values[-1] - values[-2]
        ) / dt

        return derivative

    @staticmethod
    def _differentiate_quat(
        quat: torch.Tensor,
        fps: float,
    ) -> torch.Tensor:
        """Compute world-frame angular velocity from an XYZW quaternion trajectory."""

        num_frames = quat.shape[0]

        ang_vel = torch.zeros(
            (*quat.shape[:-1], 3),
            dtype=quat.dtype,
            device=quat.device,
        )

        if num_frames <= 1:
            return ang_vel

        dt = 1.0 / float(fps)

        def relative_axis_angle(
            q_next: torch.Tensor,
            q_prev: torch.Tensor,
        ) -> torch.Tensor:
            shape = q_next.shape[:-1]

            q_next_flat = q_next.reshape(-1, 4)
            q_prev_flat = q_prev.reshape(-1, 4)

            # Relative spatial/world rotation:
            # q_delta = q_next * inverse(q_prev)
            q_delta = quat_mul(
                q_next_flat,
                quat_inv(q_prev_flat),
            )

            # q and -q represent the same orientation. Choose the
            # shortest rotation before converting to axis-angle.
            q_delta = quat_unique(q_delta)

            return axis_angle_from_quat(
                q_delta
            ).reshape(*shape, 3)

        # Central difference.
        ang_vel[1:-1] = (
            relative_axis_angle(
                quat[2:],
                quat[:-2],
            )
            / (2.0 * dt)
        )

        # Forward difference at the first frame.
        ang_vel[0] = (
            relative_axis_angle(
                quat[1],
                quat[0],
            )
            / dt
        )

        # Backward difference at the last frame.
        ang_vel[-1] = (
            relative_axis_angle(
                quat[-1],
                quat[-2],
            )
            / dt
        )

        return ang_vel

    @property
    def command(self) -> torch.Tensor:
        return torch.cat((self.joint_pos, self.joint_vel), dim=-1)

    @property
    def frame_idx(self) -> torch.Tensor:
        return self._frame_idx

    @property
    def start_frame(self) -> int:
        return self._start_frame

    @property
    def joint_pos(self) -> torch.Tensor:
        return self._joint_pos_all[self._frame_idx]

    @property
    def joint_vel(self) -> torch.Tensor:
        return self._joint_vel_all[self._frame_idx]

    @property
    def cube_pos(self) -> torch.Tensor:
        if self._cube_pos_all is None:
            raise RuntimeError("Motion does not contain an object reference.")
        return self._cube_pos_all[self._frame_idx]

    @property
    def cube_quat(self) -> torch.Tensor:
        if self._cube_quat_all is None:
            raise RuntimeError("Motion does not contain an object reference.")
        return self._cube_quat_all[self._frame_idx]

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
        """Current bulk orientation of the deformable dice in XYZW convention."""

        reference_nodal_pos = (
            self.cube.data.default_nodal_state_w.torch[..., :3]
        )

        current_nodal_pos = (
            self.cube.data.nodal_pos_w.torch
        )

        return estimate_deformable_orientation_kabsch(
            reference_nodal_pos=reference_nodal_pos,
            current_nodal_pos=current_nodal_pos,
        )

    @property
    def finished(self) -> torch.Tensor:
        if self.cfg.loop:
            return torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return self._frame_idx >= self.num_frames - 1

    @property
    def body_pos(self) -> torch.Tensor:
        """Current fixed-root Cartesian body-position reference."""
        if not self.has_body_reference:
            raise RuntimeError(
                "Motion does not contain Cartesian body references."
            )

        return self._body_pos_all[self._frame_idx]


    @property
    def body_quat(self) -> torch.Tensor:
        """Current fixed-root Cartesian body-orientation reference."""
        if not self.has_body_reference:
            raise RuntimeError(
                "Motion does not contain Cartesian body references."
            )

        return self._body_quat_all[self._frame_idx]

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
    def hand_pos(self) -> torch.Tensor:
        """Current fixed-root virtual-hand position reference."""

        if self._hand_pos_all is None:
            raise RuntimeError(
                "Motion does not contain hand references."
            )

        return self._hand_pos_all[self._frame_idx]

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
    def body_lin_vel(self) -> torch.Tensor:
        """Current fixed-root Cartesian body linear-velocity reference."""
        if self._body_lin_vel_all is None:
            raise RuntimeError(
                "Motion does not contain Cartesian body references."
            )

        return self._body_lin_vel_all[self._frame_idx]


    @property
    def body_ang_vel(self) -> torch.Tensor:
        """Current fixed-root Cartesian body angular-velocity reference."""
        if self._body_ang_vel_all is None:
            raise RuntimeError(
                "Motion does not contain Cartesian body references."
            )

        return self._body_ang_vel_all[self._frame_idx]

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

    def start_joint_pos(self, env_ids: torch.Tensor) -> torch.Tensor:
        return self._joint_pos_all[self._start_frame].unsqueeze(0).repeat(env_ids.numel(), 1)

    def start_joint_vel(self, env_ids: torch.Tensor) -> torch.Tensor:
        return self._joint_vel_all[self._start_frame].unsqueeze(0).repeat(env_ids.numel(), 1)

    def start_cube_pos(self, env_ids: torch.Tensor) -> torch.Tensor:
        if self._cube_pos_all is None:
            raise RuntimeError("Motion does not contain an object reference.")
        return self._cube_pos_all[self._start_frame].unsqueeze(0).repeat(env_ids.numel(), 1)

    def start_cube_quat(self, env_ids: torch.Tensor) -> torch.Tensor:
        if self._cube_quat_all is None:
            raise RuntimeError("Motion does not contain an object reference.")
        return self._cube_quat_all[self._start_frame].unsqueeze(0).repeat(env_ids.numel(), 1)

    def _update_metrics(self):
        # ------------------------------------------------------------------
        # Motion phase
        # ------------------------------------------------------------------

        denom = max(self.num_frames - 1, 1)

        self.metrics["phase"][:] = (
            self._frame_idx.float() / float(denom)
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
        self._frame_idx[env_ids] = self._start_frame
        self._motion_step[env_ids] = 0

    def _update_command(self):
        # This method is called after one environment/control step has
        # completed. Advance the reference clock first s
        self._motion_step += 1

        elapsed_s = (
            self._motion_step.float()
            * float(self._env.step_dt)
        )

        offset = torch.floor(
            elapsed_s
            * self.motion_fps
            * float(self.cfg.playback_speed)
            + 1.0e-6
        ).long()

        if self.cfg.loop:
            span = self.num_frames - self._start_frame

            self._frame_idx[:] = (
                self._start_frame
                + torch.remainder(offset, span)
            )

        else:
            self._frame_idx[:] = torch.clamp(
                self._start_frame + offset,
                min=self._start_frame,
                max=self.num_frames - 1,
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

    # The command changes every environment step internally; it is not randomly resampled by time.
    resampling_time_range: tuple[float, float] = (1.0e9, 1.0e9)

    asset_name: str = "robot"
    cube_name: str = "cube"
    motion_file: str = ""
    start_frame: int = 0
    playback_speed: float = 1.0
    loop: bool = False
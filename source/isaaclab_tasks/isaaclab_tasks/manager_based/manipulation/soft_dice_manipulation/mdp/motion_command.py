from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import numpy as np
import torch


from isaaclab.managers import CommandTermCfg, CommandTerm
from isaaclab.utils.configclass import configclass


from .motion_utils import (
    HOLOSOMA_TO_ISAAC_INDICES,
    desired_cube_pose_from_holosoma,
    load_motion_file,
)

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

        joint_qpos_np, fps, root_qpos_np, object_qpos_np = load_motion_file(cfg.motion_file)
        self.motion_fps = float(fps)

        reorder_idx = np.asarray(HOLOSOMA_TO_ISAAC_INDICES, dtype=np.int64)
        q_np = np.asarray(joint_qpos_np[:, reorder_idx], dtype=np.float32)

        if q_np.shape[1] != self.robot.num_joints:
            raise ValueError(
                f"Reference has {q_np.shape[1]} joints after reordering; robot has {self.robot.num_joints}."
            )

        qd_np = self._differentiate(q_np, self.motion_fps)

        self._joint_pos_all = torch.as_tensor(q_np, device=self.device)
        self._joint_vel_all = torch.as_tensor(qd_np, device=self.device)
        self.num_frames = int(q_np.shape[0])

        if not 0 <= cfg.start_frame < self.num_frames:
            raise ValueError(f"start_frame={cfg.start_frame} outside [0, {self.num_frames - 1}]")
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
            int(cfg.start_frame),
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

    @staticmethod
    def _differentiate(q: np.ndarray, fps: float) -> np.ndarray:
        if q.shape[0] <= 1:
            return np.zeros_like(q, dtype=np.float32)

        dt = 1.0 / float(fps)
        qd = np.zeros_like(q, dtype=np.float32)
        qd[1:-1] = (q[2:] - q[:-2]) / (2.0 * dt)
        qd[0] = (q[1] - q[0]) / dt
        qd[-1] = (q[-1] - q[-2]) / dt
        return qd

    @property
    def command(self) -> torch.Tensor:
        return torch.cat((self.joint_pos, self.joint_vel), dim=-1)

    @property
    def frame_idx(self) -> torch.Tensor:
        return self._frame_idx

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
    def finished(self) -> torch.Tensor:
        if self.cfg.loop:
            return torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return self._frame_idx >= self.num_frames - 1

    def start_joint_pos(self, env_ids: torch.Tensor) -> torch.Tensor:
        return self._joint_pos_all[self.cfg.start_frame].unsqueeze(0).repeat(env_ids.numel(), 1)

    def start_joint_vel(self, env_ids: torch.Tensor) -> torch.Tensor:
        return self._joint_vel_all[self.cfg.start_frame].unsqueeze(0).repeat(env_ids.numel(), 1)

    def start_cube_pos(self, env_ids: torch.Tensor) -> torch.Tensor:
        if self._cube_pos_all is None:
            raise RuntimeError("Motion does not contain an object reference.")
        return self._cube_pos_all[self.cfg.start_frame].unsqueeze(0).repeat(env_ids.numel(), 1)

    def start_cube_quat(self, env_ids: torch.Tensor) -> torch.Tensor:
        if self._cube_quat_all is None:
            raise RuntimeError("Motion does not contain an object reference.")
        return self._cube_quat_all[self.cfg.start_frame].unsqueeze(0).repeat(env_ids.numel(), 1)

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
        self._frame_idx[env_ids] = int(self.cfg.start_frame)
        self._motion_step[env_ids] = 0

    def _update_command(self):
        elapsed_s = self._motion_step.float() * float(self._env.step_dt)
        offset = torch.floor(
            elapsed_s * self.motion_fps * float(self.cfg.playback_speed)
        ).long()

        if self.cfg.loop:
            span = self.num_frames - int(self.cfg.start_frame)
            self._frame_idx[:] = int(self.cfg.start_frame) + torch.remainder(offset, span)
        else:
            self._frame_idx[:] = torch.clamp(
                int(self.cfg.start_frame) + offset,
                min=int(self.cfg.start_frame),
                max=self.num_frames - 1,
            )
        self._motion_step += 1

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
            cube_center_e - self.cube_pos,
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
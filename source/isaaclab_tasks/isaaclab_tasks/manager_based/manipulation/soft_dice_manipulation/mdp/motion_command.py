from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING
from dataclasses import MISSING

import numpy as np
import torch


from isaaclab.managers import CommandTermCfg
from isaaclab.utils.configclass import configclass
from isaaclab.managers import CommandTerm

from scripts.soft_dice_environment.replay_utils import (
    HOLOSOMA_TO_ISAAC_INDICES,
    desired_cube_pose_from_holosoma,
    load_motion_file,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv



class MotionCommand(CommandTerm):
    """Time-indexed reference command for the soft-dice tracking task.

    The source motion is the same Holosoma/OmniRetarget qpos file currently used by
    ``play_retargeted.py``. The term exposes, for every parallel environment:

    - reference H1 joint position
    - numerically differentiated reference H1 joint velocity
    - reference dice position in the *environment frame*
    - reference dice orientation (Isaac Lab XYZW)

    The public ``command`` tensor is ``[q_ref, qd_ref]``. Cube references are kept as
    separate properties because they will later be consumed by object-tracking rewards.
    """

    cfg: MotionCommandCfg

    def __init__(self, cfg: MotionCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        self.robot = env.scene[cfg.asset_name]

        robot_joint_qpos_np, fps, root_qpos_np, object_qpos_np = load_motion_file(cfg.motion_file)
        self.motion_fps = float(fps)

        # Match the ordering already used in replay_runner.py.
        reorder_idx_np = np.asarray(HOLOSOMA_TO_ISAAC_INDICES, dtype=np.int64)
        q_np = np.asarray(robot_joint_qpos_np[:, reorder_idx_np], dtype=np.float32)

        if q_np.shape[1] != self.robot.num_joints:
            raise ValueError(
                f"Reference has {q_np.shape[1]} joints after reordering, but robot has {self.robot.num_joints}."
            )

        qd_np = self._differentiate(q_np, self.motion_fps)

        self._joint_pos_all = torch.as_tensor(q_np, dtype=torch.float32, device=self.device)
        self._joint_vel_all = torch.as_tensor(qd_np, dtype=torch.float32, device=self.device)
        self.num_frames = int(q_np.shape[0])

        if not 0 <= int(cfg.start_frame) < self.num_frames:
            raise ValueError(f"start_frame={cfg.start_frame} outside [0, {self.num_frames - 1}].")
        if cfg.playback_speed <= 0.0:
            raise ValueError("playback_speed must be > 0.")

        # Pre-compute the object reference .
        self.has_object_reference = root_qpos_np is not None and object_qpos_np is not None
        if self.has_object_reference:
            robot_pos_np = np.asarray(self.robot.cfg.init_state.pos, dtype=np.float32)
            robot_quat_xyzw_np = np.asarray(self.robot.cfg.init_state.rot, dtype=np.float32)

            cube_pos = np.zeros((self.num_frames, 3), dtype=np.float32)
            cube_quat = np.zeros((self.num_frames, 4), dtype=np.float32)
            for frame in range(self.num_frames):
                pos, quat, *_ = desired_cube_pose_from_holosoma(
                    root_qpos_np,
                    object_qpos_np,
                    frame,
                    robot_pos_np,
                    robot_quat_xyzw_np,
                    reference_offset=None,
                    apply_z_lift=False,
                )
                cube_pos[frame] = pos
                cube_quat[frame] = quat

            self._cube_pos_all = torch.as_tensor(cube_pos, dtype=torch.float32, device=self.device)
            self._cube_quat_all = torch.as_tensor(cube_quat, dtype=torch.float32, device=self.device)
        else:
            self._cube_pos_all = None
            self._cube_quat_all = None

        self._frame_idx = torch.full(
            (self.num_envs,), int(cfg.start_frame), dtype=torch.long, device=self.device
        )

        self.metrics["phase"] = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)

    @staticmethod
    def _differentiate(q: np.ndarray, fps: float) -> np.ndarray:
        """Numerically differentiate joint positions using central differences."""
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
        """Reference ``[joint_pos, joint_vel]`` for the current frame."""
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
        """Reference cube center in each environment's local frame."""
        if self._cube_pos_all is None:
            raise RuntimeError("This motion file does not contain an object reference.")
        return self._cube_pos_all[self._frame_idx]

    @property
    def cube_quat(self) -> torch.Tensor:
        """Reference cube orientation in Isaac Lab XYZW convention."""
        if self._cube_quat_all is None:
            raise RuntimeError("This motion file does not contain an object reference.")
        return self._cube_quat_all[self._frame_idx]

    @property
    def finished(self) -> torch.Tensor:
        if self.cfg.loop:
            return torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return self._frame_idx >= (self.num_frames - 1)

    def start_joint_pos(self, env_ids: torch.Tensor) -> torch.Tensor:
        q = self._joint_pos_all[int(self.cfg.start_frame)]
        return q.unsqueeze(0).repeat(env_ids.numel(), 1)

    def start_joint_vel(self, env_ids: torch.Tensor) -> torch.Tensor:
        qd = self._joint_vel_all[int(self.cfg.start_frame)]
        return qd.unsqueeze(0).repeat(env_ids.numel(), 1)

    def start_cube_pos(self, env_ids: torch.Tensor) -> torch.Tensor:
        if self._cube_pos_all is None:
            raise RuntimeError("This motion file does not contain an object reference.")
        p = self._cube_pos_all[int(self.cfg.start_frame)]
        return p.unsqueeze(0).repeat(env_ids.numel(), 1)

    def start_cube_quat(self, env_ids: torch.Tensor) -> torch.Tensor:
        if self._cube_quat_all is None:
            raise RuntimeError("This motion file does not contain an object reference.")
        q = self._cube_quat_all[int(self.cfg.start_frame)]
        return q.unsqueeze(0).repeat(env_ids.numel(), 1)

    def _update_metrics(self):
        denom = max(self.num_frames - 1, 1)
        self.metrics["phase"][:] = self._frame_idx.float() / float(denom)

    def _resample_command(self, env_ids: Sequence[int]):
        # For Step 1 every episode starts at the configured reference frame.
        self._frame_idx[env_ids] = int(self.cfg.start_frame)

    def _update_command(self):
        # ManagerBasedRLEnv increments episode_length_buf once per environment step.
        # This reproduces the time-based frame selection in replay_runner.py:
        # frame = start + floor(sim_time * motion_fps * playback_speed).
        elapsed_s = self._env.episode_length_buf.float() * float(self._env.step_dt)
        offset = torch.floor(elapsed_s * self.motion_fps * float(self.cfg.playback_speed)).long()

        if self.cfg.loop:
            span = self.num_frames - int(self.cfg.start_frame)
            self._frame_idx[:] = int(self.cfg.start_frame) + torch.remainder(offset, span)
        else:
            self._frame_idx[:] = torch.clamp(
                int(self.cfg.start_frame) + offset,
                min=int(self.cfg.start_frame),
                max=self.num_frames - 1,
            )


@configclass
class MotionCommandCfg(CommandTermCfg):
    """Configuration for a time-indexed Holosoma/OmniRetarget motion reference."""

    class_type: type | str = "{DIR}.motion_command:MotionCommand"

    # The reference is not randomly resampled by time. It advances with episode time.
    resampling_time_range: tuple[float, float] = (1.0e6, 1.0e6)
    debug_vis: bool = False

    asset_name: str = "robot"
    motion_file: str = MISSING
    start_frame: int = 0
    playback_speed: float = 1.0
    loop: bool = False

from __future__ import annotations

from collections.abc import Sequence

import torch

from isaaclab.managers import (
    DatasetExportMode,
    RecorderManagerBaseCfg,
    RecorderTerm,
    RecorderTermCfg,
)
from isaaclab.utils.configclass import configclass

from ..utils.deformable_utils import compute_deformable_shape_metrics
from ..utils.geometry_utils import orientation_error, orientation_in_frame, position_in_frame, vector_error, xy_position_error
from ..utils.motion_utils import H1_HAND_REFERENCE_NAMES, H1_TRACKED_BODY_NAMES
from .robustness import (
    initialize_robustness_terminal_buffers,
    snapshot_robustness_terminal,
)

class SoftDiceEvaluationRecorder(RecorderTerm):
    """Capture terminal state and per-step evaluation data."""

    def __init__(self, cfg: RecorderTermCfg, env) -> None:
        super().__init__(cfg, env)

        self._motion = env.command_manager.get_term("motion")

        action_cfg = env.cfg.actions.joint_pos
        joint_ids, joint_names = self._motion.robot.find_joints(
            action_cfg.joint_names,
            preserve_order=action_cfg.preserve_order,
        )

        self._robot_pos_e = torch.as_tensor(self._motion.robot.cfg.init_state.pos, dtype=torch.float32, device=env.device).unsqueeze(0).expand(env.num_envs, -1)
        self._robot_quat_e = torch.as_tensor(self._motion.robot.cfg.init_state.rot, dtype=torch.float32, device=env.device).unsqueeze(0).expand(env.num_envs, -1)       

        self._controlled_joint_ids = torch.as_tensor(
            joint_ids,
            dtype=torch.long,
            device=env.device,
        )
        self._controlled_joint_names = list(joint_names)
        self._num_controlled_joints = len(joint_names)

        self._max_episode_steps = env.max_episode_length
        self._env_ids = torch.arange(
            env.num_envs,
            dtype=torch.long,
            device=env.device,
        )

        # ------------------------------------------------------------------
        # Communication buffer between the environment and evaluation runner.
        #
        # This is intentionally outside RecorderManager's EpisodeData because
        # EpisodeData is cleared as part of the reset/export lifecycle.
        # ------------------------------------------------------------------

        env.extras["evaluation"] = {
            "terminal_valid": torch.zeros(
                env.num_envs,
                dtype=torch.bool,
                device=env.device,
            ),
            "episode_steps": torch.zeros(
                env.num_envs,
                dtype=torch.long,
                device=env.device,
            ),
            "motion_frame": torch.zeros(
                env.num_envs,
                dtype=torch.long,
                device=env.device,
            ),
            "cube_pos_e": torch.zeros(
                env.num_envs,
                3,
                dtype=torch.float32,
                device=env.device,
            ),
            "cube_quat_xyzw": torch.zeros(
                env.num_envs,
                4,
                dtype=torch.float32,
                device=env.device,
            ),
            "reference_cube_pos_e": torch.zeros(
                env.num_envs,
                3,
                dtype=torch.float32,
                device=env.device,
            ),
            "reference_cube_quat_xyzw": torch.zeros(
                env.num_envs,
                4,
                dtype=torch.float32,
                device=env.device,
            ),
            "motion_finished": torch.zeros(
                env.num_envs,
                dtype=torch.bool,
                device=env.device,
            ),
            "time_out": torch.zeros(
                env.num_envs,
                dtype=torch.bool,
                device=env.device,
            ),
            "motion_id": torch.zeros(
                env.num_envs,
                dtype=torch.long,
                device=env.device,
            ),
            "position_landing_start_phase": torch.full(
                (env.num_envs,),
                float("nan"),
                dtype=torch.float32,
                device=env.device,
            ),
            "orientation_landing_start_phase": torch.full(
                (env.num_envs,),
                float("nan"),
                dtype=torch.float32,
                device=env.device,
            ),
            "trajectory": {
                "motion_frame": torch.full(
                    (env.num_envs, self._max_episode_steps),
                    -1,
                    dtype=torch.long,
                    device=env.device,
                ),
                "body_position_error_m": torch.zeros(
                    (
                        env.num_envs,
                        self._max_episode_steps,
                        len(H1_TRACKED_BODY_NAMES),
                    ),
                    dtype=torch.float32,
                    device=env.device,
                ),
                "body_orientation_error_rad": torch.zeros(
                    (
                        env.num_envs,
                        self._max_episode_steps,
                        len(H1_TRACKED_BODY_NAMES),
                    ),
                    dtype=torch.float32,
                    device=env.device,
                ),
                "hand_position_error_m": torch.zeros(
                    (
                        env.num_envs,
                        self._max_episode_steps,
                        len(H1_HAND_REFERENCE_NAMES),
                    ),
                    dtype=torch.float32,
                    device=env.device,
                ),
                "cube_xy_position_error_m": torch.zeros(
                    (env.num_envs, self._max_episode_steps),
                    dtype=torch.float32,
                    device=env.device,
                ),
                "cube_orientation_error_rad": torch.zeros(
                    (env.num_envs, self._max_episode_steps),
                    dtype=torch.float32,
                    device=env.device,
                ),
                "joint_torque_nm": torch.zeros(
                    (
                        env.num_envs,
                        self._max_episode_steps,
                        self._num_controlled_joints,
                    ),
                    dtype=torch.float32,
                    device=env.device,
                ),
                "action_delta": torch.zeros(
                    (
                        env.num_envs,
                        self._max_episode_steps,
                        self._num_controlled_joints,
                    ),
                    dtype=torch.float32,
                    device=env.device,
                ),
                "deformation_rms_m": torch.zeros(
                    (env.num_envs, self._max_episode_steps),
                    dtype=torch.float32,
                    device=env.device,
                ),
                "deformation_p95_m": torch.zeros(
                    (env.num_envs, self._max_episode_steps),
                    dtype=torch.float32,
                    device=env.device,
                ),
                "deformation_max_m": torch.zeros(
                    (env.num_envs, self._max_episode_steps),
                    dtype=torch.float32,
                    device=env.device,
                ),
                "relative_extent_change": torch.zeros(
                    (env.num_envs, self._max_episode_steps, 3),
                    dtype=torch.float32,
                    device=env.device,
                ),
                "phase": torch.zeros(
                    (env.num_envs,self._max_episode_steps),
                    dtype=torch.float32,
                    device=env.device,
                ),
            },
            "cube_pos_r0": torch.zeros(env.num_envs, 3, dtype=torch.float32, device=env.device),
            "cube_quat_r0_xyzw": torch.zeros(env.num_envs, 4, dtype=torch.float32, device=env.device),
            "final_reference_cube_pos_r0": torch.zeros(env.num_envs, 3, dtype=torch.float32, device=env.device),
            "final_reference_cube_quat_r0_xyzw": torch.zeros(env.num_envs, 4, dtype=torch.float32, device=env.device),
        }
        
        initialize_robustness_terminal_buffers(
            env,
            env.extras["evaluation"],
        )

    def record_post_step(self) -> tuple[None, None]:
        """Record evaluation quantities for the current control step."""

        # episode_length_buf has already been incremented here.
        step_idx = self._env.episode_length_buf - 1

        if torch.any(step_idx < 0):
            raise RuntimeError("Negative evaluation trajectory index.")

        if torch.any(step_idx >= self._max_episode_steps):
            raise RuntimeError(
                "Evaluation trajectory exceeded the allocated episode buffer."
            )

        if not self._motion.has_body_reference:
            raise RuntimeError(
                "Cartesian trajectory evaluation requires body references."
            )

        if not self._motion.has_hand_reference:
            raise RuntimeError(
                "Cartesian trajectory evaluation requires hand references."
            )

        if not self._motion.has_object_reference:
            raise RuntimeError(
                "Cartesian trajectory evaluation requires an object reference."
            )

        trajectory = self._env.extras["evaluation"]["trajectory"]

        # --------------------------------------------------------------
        # Body Cartesian tracking.
        # Use the same yaw-aligned reference as the tracking reward.
        # --------------------------------------------------------------

        body_pos_ref, body_quat_ref = self._motion.aligned_body_reference()

        body_position_error = vector_error(
            reference=body_pos_ref,
            current=self._motion.robot_body_pos,
        )

        body_orientation_error = orientation_error(
            reference_quat=body_quat_ref,
            current_quat=self._motion.robot_body_quat,
        )

        # --------------------------------------------------------------
        # Hand Cartesian tracking.
        # --------------------------------------------------------------

        hand_pos_ref = self._motion.aligned_hand_reference()

        hand_position_error = vector_error(
            reference=hand_pos_ref,
            current=self._motion.robot_hand_pos,
        )

        # --------------------------------------------------------------
        # Cube trajectory tracking.
        # --------------------------------------------------------------

        cube_xy_position_error = xy_position_error(
            reference_pos=self._motion.cube_pos,
            current_pos=self._motion.simulator_cube_pos,
        )

        cube_orientation_error = orientation_error(
            reference_quat=self._motion.cube_quat,
            current_quat=self._motion.simulator_cube_quat,
        )

        # --------------------------------------------------------------
        # Torque and action smoothness.
        # --------------------------------------------------------------

        joint_torque_nm = self._motion.robot.data.applied_torque.torch[
            :, self._controlled_joint_ids
        ]

        action_delta = (
            self._env.action_manager.action
            - self._env.action_manager.prev_action
        )

        # --------------------------------------------------------------
        # Deformable-object shape metrics.
        # --------------------------------------------------------------

        reference_nodal_pos = self._motion.cube.data.default_nodal_state_w.torch[..., :3]
        current_nodal_pos = self._motion.cube.data.nodal_pos_w.torch

        deformation = compute_deformable_shape_metrics(
            reference_nodal_pos=reference_nodal_pos,
            current_nodal_pos=current_nodal_pos,
        )

        # --------------------------------------------------------------
        # Store current control step.
        # --------------------------------------------------------------

        trajectory["motion_frame"][self._env_ids, step_idx] = self._motion.frame_idx
        trajectory["phase"][self._env_ids,step_idx,] = self._motion.phase

        trajectory["body_position_error_m"][
            self._env_ids, step_idx, :
        ] = body_position_error

        trajectory["body_orientation_error_rad"][
            self._env_ids, step_idx, :
        ] = body_orientation_error

        trajectory["hand_position_error_m"][
            self._env_ids, step_idx, :
        ] = hand_position_error

        trajectory["cube_xy_position_error_m"][
            self._env_ids, step_idx
        ] = cube_xy_position_error

        trajectory["cube_orientation_error_rad"][
            self._env_ids, step_idx
        ] = cube_orientation_error

        trajectory["joint_torque_nm"][
            self._env_ids, step_idx, :
        ] = joint_torque_nm

        trajectory["action_delta"][
            self._env_ids, step_idx, :
        ] = action_delta

        trajectory["deformation_rms_m"][
            self._env_ids, step_idx
        ] = deformation["deformation_rms_m"]

        trajectory["deformation_p95_m"][
            self._env_ids, step_idx
        ] = deformation["deformation_p95_m"]

        trajectory["deformation_max_m"][
            self._env_ids, step_idx
        ] = deformation["deformation_max_m"]

        trajectory["relative_extent_change"][
            self._env_ids, step_idx, :
        ] = deformation["relative_extent_change"]

        return None, None

    def record_pre_reset(
        self,
        env_ids: Sequence[int] | None,
    ) -> tuple[None, None]:
        """Snapshot terminal state immediately before environment reset."""

        if env_ids is None:
            env_ids_t = torch.arange(
                self._env.num_envs,
                dtype=torch.long,
                device=self._env.device,
            )
        else:
            env_ids_t = torch.as_tensor(
                env_ids,
                dtype=torch.long,
                device=self._env.device,
            )

        if env_ids_t.numel() == 0:
            return None, None

        output = self._env.extras["evaluation"]

        # Current physical cube state.
        cube_pos = self._motion.simulator_cube_pos
        cube_quat = self._motion.simulator_cube_quat

        # Current reference state. At motion_finished this should be
        # the final reference frame.
        reference_cube_pos = self._motion.cube_pos
        reference_cube_quat = self._motion.cube_quat

        final_reference_cube_pos = self._motion.final_cube_pos
        final_reference_cube_quat = self._motion.final_cube_quat

        cube_pos_r0 = position_in_frame(position=cube_pos, frame_position=self._robot_pos_e, frame_orientation=self._robot_quat_e)
        cube_quat_r0 = orientation_in_frame(orientation=cube_quat, frame_orientation=self._robot_quat_e)

        final_reference_cube_pos_r0 = position_in_frame(position=final_reference_cube_pos, frame_position=self._robot_pos_e, frame_orientation=self._robot_quat_e)
        final_reference_cube_quat_r0 = orientation_in_frame(orientation=final_reference_cube_quat, frame_orientation=self._robot_quat_e)

        output["terminal_valid"][env_ids_t] = True
        output["episode_steps"][env_ids_t] = self._env.episode_length_buf[env_ids_t]
        output["motion_frame"][env_ids_t] = self._motion.frame_idx[env_ids_t]

        output["cube_pos_e"][env_ids_t] = cube_pos[env_ids_t]
        output["cube_quat_xyzw"][env_ids_t] = cube_quat[env_ids_t]

        output["reference_cube_pos_e"][env_ids_t] = reference_cube_pos[env_ids_t]
        output["reference_cube_quat_xyzw"][env_ids_t] = reference_cube_quat[env_ids_t]
        output["cube_pos_r0"][env_ids_t] = cube_pos_r0[env_ids_t]
        output["cube_quat_r0_xyzw"][env_ids_t] = cube_quat_r0[env_ids_t]
        output["final_reference_cube_pos_r0"][env_ids_t] = final_reference_cube_pos_r0[env_ids_t]
        output["final_reference_cube_quat_r0_xyzw"][env_ids_t] = final_reference_cube_quat_r0[env_ids_t]

        termination_manager = self._env.termination_manager

        output["motion_finished"][env_ids_t] = termination_manager.get_term(
            "motion_finished"
        )[env_ids_t]

        output["time_out"][env_ids_t] = termination_manager.get_term(
            "time_out"
        )[env_ids_t]
        output["motion_id"][env_ids_t] = self._motion.motion_id[env_ids_t]

        if self._motion.cfg.phase_metadata_file:
            output["position_landing_start_phase"][env_ids_t] = self._motion.position_landing_start_phase[env_ids_t]
            output["orientation_landing_start_phase"][env_ids_t] = self._motion.orientation_landing_start_phase[env_ids_t]

        snapshot_robustness_terminal(
            env=self._env,
            output=output,
            env_ids=env_ids_t,
        )

        return None, None


@configclass
class SoftDiceEvaluationRecorderTermCfg(RecorderTermCfg):
    class_type: type[RecorderTerm] = SoftDiceEvaluationRecorder


@configclass
class SoftDiceEvaluationRecordersCfg(RecorderManagerBaseCfg):
    """Recorder configuration used only by the evaluation environment."""

    terminal_state = SoftDiceEvaluationRecorderTermCfg()

    dataset_export_mode = DatasetExportMode.EXPORT_NONE
    export_in_record_pre_reset = False
    export_in_close = False
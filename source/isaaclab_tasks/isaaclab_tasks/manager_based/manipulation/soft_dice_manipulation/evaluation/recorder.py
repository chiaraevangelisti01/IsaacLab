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


class SoftDiceEvaluationRecorder(RecorderTerm):
    """Capture terminal physical state before Isaac Lab resets an environment."""

    def __init__(
        self,
        cfg: RecorderTermCfg,
        env,
    ) -> None:
        super().__init__(cfg, env)

        self._motion = env.command_manager.get_term(
            "motion"
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
        }

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

        # Current reference state. At motion_finished this should be the
        # final reference frame.
        reference_cube_pos = self._motion.cube_pos
        reference_cube_quat = self._motion.cube_quat

        output["terminal_valid"][env_ids_t] = True

        output["episode_steps"][env_ids_t] = (
            self._env.episode_length_buf[env_ids_t]
        )

        output["motion_frame"][env_ids_t] = (
            self._motion.frame_idx[env_ids_t]
        )

        output["cube_pos_e"][env_ids_t] = (
            cube_pos[env_ids_t]
        )

        output["cube_quat_xyzw"][env_ids_t] = (
            cube_quat[env_ids_t]
        )

        output["reference_cube_pos_e"][env_ids_t] = (
            reference_cube_pos[env_ids_t]
        )

        output["reference_cube_quat_xyzw"][env_ids_t] = (
            reference_cube_quat[env_ids_t]
        )

        termination_manager = (
            self._env.termination_manager
        )

        output["motion_finished"][env_ids_t] = (
            termination_manager
            .get_term("motion_finished")[env_ids_t]
        )

        output["time_out"][env_ids_t] = (
            termination_manager
            .get_term("time_out")[env_ids_t]
        )

        return None, None


@configclass
class SoftDiceEvaluationRecorderTermCfg(
    RecorderTermCfg
):
    class_type: type[RecorderTerm] = (
        SoftDiceEvaluationRecorder
    )


@configclass
class SoftDiceEvaluationRecordersCfg(
    RecorderManagerBaseCfg
):
    """Recorder configuration used only by the evaluation environment."""

    terminal_state = (
        SoftDiceEvaluationRecorderTermCfg()
    )

    dataset_export_mode = (
        DatasetExportMode.EXPORT_NONE
    )

    export_in_record_pre_reset = False
    export_in_close = False
from __future__ import annotations

import numpy as np

from isaaclab.utils.configclass import configclass

from .evaluation.recorder import (
    SoftDiceEvaluationRecordersCfg,
)
from .evaluation.robustness import (
    reset_to_motion_start_with_robustness,
)
from .soft_dice_landing_aware_env_cfg import (
    LandingAwareCriticCfg,
    LandingAwareObservationsCfg,
    LandingAwarePolicyCfg,
    SoftDiceLandingAwareEnvCfg,
)


# ======================================================================
# NO-PHASE OBSERVATION CONFIGURATION.
# ======================================================================


@configclass
class LandingAwareNoPhasePolicyCfg(
    LandingAwarePolicyCfg
):
    """Landing-aware actor observations without explicit phase input."""

    task_phase = None


@configclass
class LandingAwareNoPhaseCriticCfg(
    LandingAwareCriticCfg
):
    """Landing-aware critic observations without explicit phase input."""

    task_phase = None


@configclass
class LandingAwareNoPhaseObservationsCfg(
    LandingAwareObservationsCfg
):
    """Landing-aware observation groups without explicit phase input."""

    policy: LandingAwareNoPhasePolicyCfg = (
        LandingAwareNoPhasePolicyCfg()
    )

    critic: LandingAwareNoPhaseCriticCfg = (
        LandingAwareNoPhaseCriticCfg()
    )


# ======================================================================
# LANDING-AWARE EVALUATION CONFIGURATION
# ======================================================================


@configclass
class SoftDiceLandingAwareEvalEnvCfg(
    SoftDiceLandingAwareEnvCfg
):
    """Evaluation environment for landing-aware soft-dice policies.

    Evaluation:
      - removes policy observation noise,
      - disables training-time domain randomization,
      - uses the evaluation recorder,
      - uses the existing robustness reset,
      - disables tracking-error early terminations.

    Motion completion and the hard episode timeout remain active.
    """

    recorders: SoftDiceEvaluationRecordersCfg = (
        SoftDiceEvaluationRecordersCfg()
    )

    def __post_init__(self):
        super().__post_init__()

        # --------------------------------------------------------------
        # Deterministic policy observations.
        # --------------------------------------------------------------
        self.observations.policy.enable_corruption = False

        # --------------------------------------------------------------
        # Disable training-time domain randomization.
        # --------------------------------------------------------------
        self.events.physics_material = None
        self.events.add_joint_default_pos = None
        self.events.base_com = None
        self.events.randomize_cube_material = None

        # --------------------------------------------------------------
        # Evaluation reset.
        # --------------------------------------------------------------
        cube_material = (
            self.scene.cube.spawn.physics_material
        )

        self.events.reset_to_reference.func = (
            reset_to_motion_start_with_robustness
        )

        self.events.reset_to_reference.params = {
            "command_name": "motion",
            "robot_name": "robot",
            "cube_name": "cube",

            # ----------------------------------------------------------
            # Initial cube-position perturbation.
            # ----------------------------------------------------------
            "cube_xy_range_m": (
                -0.02,
                0.02,
            ),

            # ----------------------------------------------------------
            # Initial cube-yaw perturbation.
            # ----------------------------------------------------------
            "cube_yaw_range_rad": (
                -np.deg2rad(10.0),
                np.deg2rad(10.0),
            ),

            # ----------------------------------------------------------
            # Deformable-material perturbations.
            # ----------------------------------------------------------
            "youngs_modulus_range_pa": (
                1.0e4,
                1.6e4,
            ),

            "poissons_ratio_range": (
                0.30,
                0.40,
            ),

            # ----------------------------------------------------------
            # Nominal material parameters.
            # ----------------------------------------------------------
            "nominal_youngs_modulus_pa": float(
                cube_material.youngs_modulus
            ),

            "nominal_poissons_ratio": float(
                cube_material.poissons_ratio
            ),
        }

        # --------------------------------------------------------------
        # motion_finished and the hard timeout remain active.
        # --------------------------------------------------------------
        self.terminations.ee_body_pos = None
        self.terminations.object_pose = None


# ======================================================================
# LANDING-AWARE EVALUATION WITHOUT PHASE OBSERVATION.
# ======================================================================


@configclass
class SoftDiceLandingAwareNoPhaseEvalEnvCfg(
    SoftDiceLandingAwareEvalEnvCfg
):
    """Evaluation environment for landing-aware policies without phase input."""

    observations: LandingAwareNoPhaseObservationsCfg = (
        LandingAwareNoPhaseObservationsCfg()
    )
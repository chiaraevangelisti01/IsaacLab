from __future__ import annotations

import numpy as np
from isaaclab.utils.configclass import configclass

from .soft_dice_env_cfg import SoftDiceTrackingEnvCfg
from .evaluation.recorder import (
    SoftDiceEvaluationRecordersCfg,
)
from .evaluation.robustness import (
    reset_to_motion_start_with_robustness,
)


@configclass
class SoftDiceTrackingEvalEnvCfg(SoftDiceTrackingEnvCfg):
    """Deterministic evaluation configuration for soft-dice tracking.

    The physical scene, actions, observations, reference motion, rewards,
    and simulation parameters are inherited from the training environment.

    Evaluation removes training-time stochasticity and tracking-error
    early terminations so that every nominal rollout can be evaluated at
    the end of the reference trajectory.
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
        # Mixed nominal + robustness evaluation reset.
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

            "cube_xy_range_m": (
                -0.02,
                0.02,
            ),
            "cube_yaw_range_rad": (
                -np.deg2rad(10.0),
                np.deg2rad(10.0),
            ),
            "youngs_modulus_range_pa": (
                1.0e4,
                1.6e4,
            ),
            "poissons_ratio_range": (
                0.30,
                0.40,
            ),

            "nominal_youngs_modulus_pa": float(
                cube_material.youngs_modulus
            ),
            "nominal_poissons_ratio": float(
                cube_material.poissons_ratio
            ),
        }
        # --------------------------------------------------------------
        # Do not define evaluation success through training termination
        # thresholds.
        #
        # motion_finished and the hard timeout remain active.
        # --------------------------------------------------------------
        self.terminations.ee_body_pos = None
        self.terminations.object_pose = None

    
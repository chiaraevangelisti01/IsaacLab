from __future__ import annotations

from isaaclab.utils.configclass import configclass

from .soft_dice_env_cfg import SoftDiceTrackingEnvCfg


@configclass
class SoftDiceTrackingEvalEnvCfg(SoftDiceTrackingEnvCfg):
    """Deterministic evaluation configuration for soft-dice tracking.

    The physical scene, actions, observations, reference motion, rewards,
    and simulation parameters are inherited from the training environment.

    Evaluation removes training-time stochasticity and tracking-error
    early terminations so that every nominal rollout can be evaluated at
    the end of the reference trajectory.
    """

    def __post_init__(self):
        super().__post_init__()

        # --------------------------------------------------------------
        # Deterministic policy observations.
        # --------------------------------------------------------------
        self.observations.policy.enable_corruption = False

        # --------------------------------------------------------------
        # Disable training-time startup domain randomization.
        # --------------------------------------------------------------
        self.events.physics_material = None
        self.events.add_joint_default_pos = None
        self.events.base_com = None

        # --------------------------------------------------------------
        # Reset exactly to the demonstrated initial state.
        #
        # Keep reset_to_reference itself because it also initializes the
        # robot and deformable dice correctly.
        # --------------------------------------------------------------
        self.events.reset_to_reference.params[
            "joint_position_range"
        ] = None

        # --------------------------------------------------------------
        # Do not define evaluation success through training termination
        # thresholds.
        #
        # motion_finished and the hard timeout remain active.
        # --------------------------------------------------------------
        self.terminations.ee_body_pos = None
        self.terminations.object_pose = None
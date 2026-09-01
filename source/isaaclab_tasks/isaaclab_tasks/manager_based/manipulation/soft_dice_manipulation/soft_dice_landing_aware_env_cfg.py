from __future__ import annotations

from pathlib import Path

from isaaclab.managers import (
    ObservationTermCfg as ObsTerm,
)
from isaaclab.managers import (
    RewardTermCfg as RewTerm,
)
from isaaclab.utils.configclass import configclass

from . import mdp
from .soft_dice_env_cfg import (
    ObservationsCfg,
    RewardsCfg,
    SoftDiceTrackingEnvCfg,
)


# ======================================================================
# Observations
# ======================================================================


@configclass
class LandingAwarePolicyCfg(
    ObservationsCfg.PolicyCfg
):
    """Actor observations for task-aware soft-dice manipulation."""

    # Current cube centroid expressed in the fixed nominal robot frame.
    object_pos = ObsTerm(
        func=mdp.object_pos_r0,
        params={
            "command_name": "motion",
        },
    )

    # Explicit task/motion phase information:
    #
    # [global motion phase,
    #  position task blend,
    #  orientation task blend]
    task_phase = ObsTerm(
        func=mdp.task_phase_observation,
        params={
            "command_name": "motion",
        },
    )

@configclass
class LandingAwareCriticCfg(
    ObservationsCfg.PrivilegedCfg
):
    """Privileged critic observations for task-aware manipulation."""

    object_pos_r0 = ObsTerm(
        func=mdp.object_pos_r0,
        params={
            "command_name": "motion",
        },
    )

    task_phase = ObsTerm(
        func=mdp.task_phase_observation,
        params={
            "command_name": "motion",
        },
    )

@configclass
class LandingAwareObservationsCfg(
    ObservationsCfg
):
    """Observation groups for the task-aware environment."""

    policy: LandingAwarePolicyCfg = (
        LandingAwarePolicyCfg()
    )

    critic: LandingAwareCriticCfg = (
        LandingAwareCriticCfg()
    )


# ======================================================================
# Rewards
# ======================================================================


@configclass
class LandingAwareRewardsCfg(
    RewardsCfg
):
    """Task-aware object reward configuration.

    Robot motion-tracking rewards are inherited unchanged.

    Exact object-reference tracking progressively loses authority
    near landing, while the canonical landing-position objective
    progressively becomes active.
    """

    # --------------------------------------------------------------
    # Demonstration object tracking.
    # --------------------------------------------------------------

    object_global_ref_position = RewTerm(
        func=(
            mdp.object_phase_weighted_ref_position_error_exp
        ),
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 0.3,
            "final_reference_scale": 0.2,
        },
    )

    object_global_ref_orientation = RewTerm(
        func=(
            mdp.object_phase_weighted_ref_orientation_error_exp
        ),
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 0.4,
            "final_reference_scale": 0.2,
        },
    )

    # --------------------------------------------------------------
    # Canonical task objective.
    # --------------------------------------------------------------

    landing_position = RewTerm(
        func=mdp.landing_position_region_reward_exp,
        weight=1.0,
        params={
            "command_name": "motion",

            # Canonical cube location in the nominal H1 root frame.
            "goal_xy": (
                0.365,
                0.0,
            ),

            # Any point inside 2.5 cm of the center is equally valid.
            "radius": 0.025,

            # Smooth decay outside the valid region.
            "std": 0.05,
        },
    )

    landing_orientation = RewTerm(
        func=(
            mdp.landing_orientation_reward_exp
        ),
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 0.35,
        },
    )

    # --------------------------------------------------------------
    # Late robot-motion tracking relaxation.
    #
    # Keep full reference tracking before the final landing phase,
    # then progressively reduce its authority until release.
    # --------------------------------------------------------------

    motion_body_pos = RewTerm(
        func=mdp.motion_phase_weighted_body_position_error_exp,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 0.3,
            "include_hands": True,
            "final_tracking_scale": 0.3,
        },
    )

    motion_body_ori = RewTerm(
        func=mdp.motion_phase_weighted_body_orientation_error_exp,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 0.4,
            "final_tracking_scale": 0.3,
        },
    )

    motion_body_lin_vel = RewTerm(
        func=mdp.motion_phase_weighted_body_linear_velocity_error_exp,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 1.0,
            "final_tracking_scale": 0.3,
        },
    )

    motion_body_ang_vel = RewTerm(
        func=mdp.motion_phase_weighted_body_angular_velocity_error_exp,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 3.14,
            "final_tracking_scale": 0.3,
        },
    )


# ======================================================================
# Environment
# ======================================================================


@configclass
class SoftDiceLandingAwareEnvCfg(
    SoftDiceTrackingEnvCfg
):
    """Task-aware extension of the soft-dice tracking environment."""

    # Replace baseline observation configuration.
    observations: LandingAwareObservationsCfg = (
        LandingAwareObservationsCfg()
    )

    # Replace baseline reward configuration.
    rewards: LandingAwareRewardsCfg = (
        LandingAwareRewardsCfg()
    )

    def __post_init__(self):
        super().__post_init__()

        # Per-trajectory landing/release phase metadata.
        self.commands.motion.phase_metadata_file = str(
            Path(__file__).resolve().parent
            / "reference_trajectories"
            / "trajectory_phase_metadata.json"
        )
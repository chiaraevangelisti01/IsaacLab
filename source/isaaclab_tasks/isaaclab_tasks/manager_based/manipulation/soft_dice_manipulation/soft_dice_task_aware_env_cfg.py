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
class TaskAwarePolicyCfg(
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


@configclass
class TaskAwareCriticCfg(
    ObservationsCfg.PrivilegedCfg
):
    """Privileged critic observations for task-aware manipulation."""

    # Keep all baseline privileged observations and additionally provide
    # the same nominal-frame position seen by the actor.
    object_pos_r0 = ObsTerm(
        func=mdp.object_pos_r0,
        params={
            "command_name": "motion",
        },
    )


@configclass
class TaskAwareObservationsCfg(
    ObservationsCfg
):
    """Observation groups for the task-aware environment."""

    policy: TaskAwarePolicyCfg = (
        TaskAwarePolicyCfg()
    )

    critic: TaskAwareCriticCfg = (
        TaskAwareCriticCfg()
    )


# ======================================================================
# Rewards
# ======================================================================


@configclass
class TaskAwareRewardsCfg(
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


# ======================================================================
# Environment
# ======================================================================


@configclass
class SoftDiceTaskAwareEnvCfg(
    SoftDiceTrackingEnvCfg
):
    """Task-aware extension of the soft-dice tracking environment."""

    # Replace baseline observation configuration.
    observations: TaskAwareObservationsCfg = (
        TaskAwareObservationsCfg()
    )

    # Replace baseline reward configuration.
    rewards: TaskAwareRewardsCfg = (
        TaskAwareRewardsCfg()
    )

    def __post_init__(self):
        super().__post_init__()

        # Per-trajectory landing/release phase metadata.
        self.commands.motion.phase_metadata_file = str(
            Path(__file__).resolve().parent
            / "reference_trajectories"
            / "trajectory_phase_metadata.json"
        )
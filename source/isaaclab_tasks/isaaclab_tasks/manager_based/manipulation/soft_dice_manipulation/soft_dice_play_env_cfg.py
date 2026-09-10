from __future__ import annotations

from isaaclab.utils.configclass import configclass

from .soft_dice_env_cfg import SoftDiceTrackingEnvCfg
from .soft_dice_landing_aware_env_cfg import SoftDiceLandingAwareEnvCfg


def _configure_nominal_play(cfg) -> None:
    """Remove training stochasticity and tracking-failure terminations."""

    # Deterministic actor observations.
    cfg.observations.policy.enable_corruption = False

    # No training-time domain randomization.
    cfg.events.physics_material = None
    cfg.events.add_joint_default_pos = None
    cfg.events.base_com = None
    cfg.events.randomize_cube_material = None

    # Exact demonstrated initial state.
    cfg.events.reset_to_reference.params["use_reference_joint_velocity"] = True
    cfg.events.reset_to_reference.params["joint_position_range"] = None
    cfg.events.reset_to_reference.params["tracking_asset_cfg"] = None
    cfg.events.reset_to_reference.params["cube_position_range"] = None
    cfg.events.reset_to_reference.params["cube_orientation_range"] = None

    # Let the motion run to completion even if tracking becomes poor.
    cfg.terminations.ee_body_pos = None
    cfg.terminations.object_pose = None


@configclass
class SoftDiceTrackingPlayEnvCfg(SoftDiceTrackingEnvCfg):

    def __post_init__(self):
        super().__post_init__()
        _configure_nominal_play(self)


@configclass
class SoftDiceLandingAwarePlayEnvCfg(SoftDiceLandingAwareEnvCfg):

    def __post_init__(self):
        super().__post_init__()
        _configure_nominal_play(self)
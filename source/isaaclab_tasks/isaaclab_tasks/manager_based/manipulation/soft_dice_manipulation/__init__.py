"""Soft-dice reference tracking task for the fixed-base Unitree H1."""

import gymnasium as gym

from . import agents

gym.register(
    id="Isaac-Soft-Dice-Tracking-H1-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.soft_dice_env_cfg:SoftDiceTrackingEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:SoftDiceTrackingPPORunnerCfg",
    },
)
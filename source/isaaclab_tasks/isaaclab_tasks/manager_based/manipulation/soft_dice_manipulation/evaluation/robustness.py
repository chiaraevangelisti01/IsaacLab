from __future__ import annotations

from collections.abc import Sequence

import torch

import isaaclab.utils.math as math_utils

from ..mdp.events import reset_to_motion_start
from ..utils.event_utils import (
    get_randomization_buffers,
    set_deformable_material_values,
)


ROBUSTNESS_BUFFER_KEY = "soft_dice_robustness"

ROBUSTNESS_CONDITIONS = (
    "nominal",
    "initial_x",
    "initial_y",
    "initial_yaw",
    "youngs_modulus",
    "poissons_ratio",
)

ROBUSTNESS_CONDITION_TO_ID = {
    name: index
    for index, name in enumerate(ROBUSTNESS_CONDITIONS)
}


def _env_ids_tensor(
    env,
    env_ids: Sequence[int] | torch.Tensor | None,
) -> torch.Tensor:
    if env_ids is None or isinstance(env_ids, slice):
        return torch.arange(
            env.num_envs,
            dtype=torch.long,
            device=env.device,
        )

    if isinstance(env_ids, torch.Tensor):
        return env_ids.to(
            device=env.device,
            dtype=torch.long,
        )

    return torch.as_tensor(
        env_ids,
        dtype=torch.long,
        device=env.device,
    )


def get_robustness_buffers(env) -> dict:
    """Get persistent per-environment robustness bookkeeping."""

    buffers = env.extras.setdefault(
        ROBUSTNESS_BUFFER_KEY,
        {},
    )

    if "condition_id" not in buffers:
        buffers["condition_id"] = torch.full(
            (env.num_envs,),
            -1,
            dtype=torch.long,
            device=env.device,
        )

    if "applied_perturbation" not in buffers:
        buffers["applied_perturbation"] = torch.zeros(
            env.num_envs,
            dtype=torch.float32,
            device=env.device,
        )

    return buffers


def _condition_env_ids(
    env_ids: torch.Tensor,
    condition_ids: torch.Tensor,
    condition_name: str,
) -> torch.Tensor:
    condition_id = ROBUSTNESS_CONDITION_TO_ID[
        condition_name
    ]

    return env_ids[
        condition_ids == condition_id
    ]


def reset_to_motion_start_with_robustness(
    env,
    env_ids: Sequence[int] | torch.Tensor | None,
    command_name: str = "motion",
    robot_name: str = "robot",
    cube_name: str = "cube",
    cube_xy_range_m: tuple[float, float] = (
        -0.02,
        0.02,
    ),
    cube_yaw_range_rad: tuple[float, float] = (
        -0.1745329252,
        0.1745329252,
    ),
    youngs_modulus_range_pa: tuple[float, float] = (
        1.0e4,
        1.6e4,
    ),
    poissons_ratio_range: tuple[float, float] = (
        0.30,
        0.40,
    ),
    nominal_youngs_modulus_pa: float = 1.5e4,
    nominal_poissons_ratio: float = 0.37,
):
    """Reset with one balanced robustness condition per environment."""

    env_ids = _env_ids_tensor(
        env,
        env_ids,
    )

    if env_ids.numel() == 0:
        return

    motion = env.command_manager.get_term(command_name)
    robustness = get_robustness_buffers(env)
    # Select the reference motion first.
    motion.sample_motions(env_ids)
    motion_ids = motion.motion_id[env_ids]

    num_conditions = len(ROBUSTNESS_CONDITIONS)

    if "condition_counts" not in robustness:
        robustness["condition_counts"] = torch.zeros(
            (motion.num_motions, num_conditions),
            dtype=torch.long,
            device=env.device,
        )

    condition_counts = robustness["condition_counts"]

    condition_ids = torch.empty(
    env_ids.numel(),
    dtype=torch.long,
    device=env.device,
)

    for motion_id in range(motion.num_motions):
        local_indices = torch.nonzero(
            motion_ids == motion_id,
            as_tuple=False,
        ).flatten()

        if local_indices.numel() == 0:
            continue

        start_condition = torch.remainder(
            condition_counts[motion_id].sum(),
            num_conditions,
        )

        assigned_conditions = torch.remainder(
            torch.arange(
                local_indices.numel(),
                dtype=torch.long,
                device=env.device,
            )
            + start_condition,
            num_conditions,
        )

        condition_ids[local_indices] = assigned_conditions

        condition_counts[motion_id] += torch.bincount(
            assigned_conditions,
            minlength=num_conditions,
        )

    robustness["condition_id"][
        env_ids
    ] = condition_ids

    robustness["applied_perturbation"][
        env_ids
    ] = 0.0

    num_envs = env_ids.numel()

    # ------------------------------------------------------------------
    # Build one per-environment initial-pose perturbation tensor.
    # ------------------------------------------------------------------

    position_offset = torch.zeros(
        (num_envs, 3),
        dtype=torch.float32,
        device=env.device,
    )

    orientation_offset = torch.zeros(
        (num_envs, 3),
        dtype=torch.float32,
        device=env.device,
    )

    x_mask = (
        condition_ids
        == ROBUSTNESS_CONDITION_TO_ID["initial_x"]
    )
    y_mask = (
        condition_ids
        == ROBUSTNESS_CONDITION_TO_ID["initial_y"]
    )
    yaw_mask = (
        condition_ids
        == ROBUSTNESS_CONDITION_TO_ID["initial_yaw"]
    )
    youngs_mask = (
        condition_ids
        == ROBUSTNESS_CONDITION_TO_ID["youngs_modulus"]
    )
    poissons_mask = (
        condition_ids
        == ROBUSTNESS_CONDITION_TO_ID["poissons_ratio"]
    )

    if torch.any(x_mask):
        position_offset[x_mask, 0] = (
            math_utils.sample_uniform(
                cube_xy_range_m[0],
                cube_xy_range_m[1],
                (int(x_mask.sum().item()),),
                env.device,
            )
        )

    if torch.any(y_mask):
        position_offset[y_mask, 1] = (
            math_utils.sample_uniform(
                cube_xy_range_m[0],
                cube_xy_range_m[1],
                (int(y_mask.sum().item()),),
                env.device,
            )
        )

    if torch.any(yaw_mask):
        orientation_offset[yaw_mask, 2] = (
            math_utils.sample_uniform(
                cube_yaw_range_rad[0],
                cube_yaw_range_rad[1],
                (int(yaw_mask.sum().item()),),
                env.device,
            )
        )

    # ------------------------------------------------------------------
    # ONE robot/cube reset for the whole env subset.
    # ------------------------------------------------------------------

    reset_to_motion_start(
        env=env,
        env_ids=env_ids,
        command_name=command_name,
        robot_name=robot_name,
        cube_name=cube_name,
        use_reference_joint_velocity=True,
        joint_position_range=None,
        tracking_asset_cfg=None,
        cube_position_range=None,
        cube_orientation_range=None,
        cube_position_offset=position_offset,
        cube_orientation_offset=orientation_offset,
        sample_motion= False,
    )

    # ------------------------------------------------------------------
    # Build complete material vectors.
    #
    # Every episode starts nominal. Only the selected condition changes.
    # ------------------------------------------------------------------

    youngs_modulus = torch.full(
        (num_envs,),
        nominal_youngs_modulus_pa,
        dtype=torch.float32,
        device=env.device,
    )

    poissons_ratio = torch.full(
        (num_envs,),
        nominal_poissons_ratio,
        dtype=torch.float32,
        device=env.device,
    )

    if torch.any(youngs_mask):
        youngs_modulus[youngs_mask] = (
            math_utils.sample_uniform(
                youngs_modulus_range_pa[0],
                youngs_modulus_range_pa[1],
                (int(youngs_mask.sum().item()),),
                env.device,
            )
        )

    if torch.any(poissons_mask):
        poissons_ratio[poissons_mask] = (
            math_utils.sample_uniform(
                poissons_ratio_range[0],
                poissons_ratio_range[1],
                (int(poissons_mask.sum().item()),),
                env.device,
            )
        )

    # ONE material update for all affected environments.
    set_deformable_material_values(
        env=env,
        env_ids=env_ids,
        asset_name=cube_name,
        youngs_modulus=youngs_modulus,
        poissons_ratio=poissons_ratio,
    )

    # ------------------------------------------------------------------
    # Store the scalar perturbation used as the robustness x-axis.
    # ------------------------------------------------------------------

    applied = robustness[
        "applied_perturbation"
    ]

    local_applied = torch.zeros(
        num_envs,
        dtype=torch.float32,
        device=env.device,
    )

    local_applied[x_mask] = (
        position_offset[x_mask, 0]
    )
    local_applied[y_mask] = (
        position_offset[y_mask, 1]
    )
    local_applied[yaw_mask] = (
        orientation_offset[yaw_mask, 2]
    )
    local_applied[youngs_mask] = (
        youngs_modulus[youngs_mask]
    )
    local_applied[poissons_mask] = (
        poissons_ratio[poissons_mask]
    )

    applied[env_ids] = local_applied

def initialize_robustness_terminal_buffers(
    env,
    output: dict,
) -> None:
    """Add robustness fields to the evaluation terminal buffer."""

    output.update(
        {
            "robustness_condition_id": torch.full(
                (env.num_envs,),
                -1,
                dtype=torch.long,
                device=env.device,
            ),
            "applied_perturbation": torch.zeros(
                env.num_envs,
                dtype=torch.float32,
                device=env.device,
            ),
            "initial_cube_position_offset_m": torch.zeros(
                (env.num_envs, 3),
                dtype=torch.float32,
                device=env.device,
            ),
            "initial_cube_yaw_offset_rad": torch.zeros(
                env.num_envs,
                dtype=torch.float32,
                device=env.device,
            ),
            "youngs_modulus_pa": torch.full(
                (env.num_envs,),
                float("nan"),
                dtype=torch.float32,
                device=env.device,
            ),
            "poissons_ratio": torch.full(
                (env.num_envs,),
                float("nan"),
                dtype=torch.float32,
                device=env.device,
            ),
        }
    )


def snapshot_robustness_terminal(
    env,
    output: dict,
    env_ids: torch.Tensor,
) -> None:
    """Snapshot robustness metadata before SAME_STEP reset."""

    robustness = get_robustness_buffers(env)
    randomization = get_randomization_buffers(env)

    output["robustness_condition_id"][
        env_ids
    ] = robustness["condition_id"][env_ids]

    output["applied_perturbation"][
        env_ids
    ] = robustness[
        "applied_perturbation"
    ][env_ids]

    output["initial_cube_position_offset_m"][
        env_ids
    ] = randomization[
        "cube_position_offset_m"
    ][env_ids]

    output["initial_cube_yaw_offset_rad"][
        env_ids
    ] = randomization[
        "cube_yaw_offset_rad"
    ][env_ids]

    output["youngs_modulus_pa"][
        env_ids
    ] = randomization[
        "youngs_modulus_pa"
    ][env_ids]

    output["poissons_ratio"][
        env_ids
    ] = randomization[
        "poissons_ratio"
    ][env_ids]


def robustness_record_from_terminal(
    terminal: dict,
    env_id: int,
) -> dict:
    """Convert one terminal robustness snapshot into CSV/JSON fields."""

    condition_id = int(
        terminal["robustness_condition_id"][
            env_id
        ].item()
    )

    if not 0 <= condition_id < len(
        ROBUSTNESS_CONDITIONS
    ):
        raise RuntimeError(
            "Invalid robustness condition ID: "
            f"{condition_id}."
        )

    position_offset = terminal[
        "initial_cube_position_offset_m"
    ][env_id]

    return {
        "robustness_condition_id": condition_id,
        "robustness_condition": (
            ROBUSTNESS_CONDITIONS[
                condition_id
            ]
        ),
        "applied_perturbation": float(
            terminal[
                "applied_perturbation"
            ][env_id].item()
        ),
        "initial_cube_x_offset_m": float(
            position_offset[0].item()
        ),
        "initial_cube_y_offset_m": float(
            position_offset[1].item()
        ),
        "initial_cube_yaw_offset_rad": float(
            terminal[
                "initial_cube_yaw_offset_rad"
            ][env_id].item()
        ),
        "youngs_modulus_pa": float(
            terminal[
                "youngs_modulus_pa"
            ][env_id].item()
        ),
        "poissons_ratio": float(
            terminal[
                "poissons_ratio"
            ][env_id].item()
        ),
    }


def select_condition_records(
    records: list[dict],
    trajectory_records: list[dict],
    condition: str,
) -> tuple[list[dict], list[dict]]:
    """Select episode and trajectory records belonging to one condition."""

    selected_records = [
        record
        for record in records
        if record["robustness_condition"]
        == condition
    ]

    episode_ids = {
        record["episode_id"]
        for record in selected_records
    }

    selected_trajectories = [
        trajectory
        for trajectory in trajectory_records
        if trajectory["episode_id"]
        in episode_ids
    ]

    return (
        selected_records,
        selected_trajectories,
    )
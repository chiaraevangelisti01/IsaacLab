from __future__ import annotations
import numpy as np
import torch
import warp as wp

def get_randomization_buffers(env):
    """Get persistent per-environment soft-dice randomization buffers."""

    buffers = env.extras.setdefault(
        "soft_dice_randomization",
        {},
    )

    if "cube_position_offset_m" not in buffers:
        buffers["cube_position_offset_m"] = torch.zeros(
            (env.num_envs, 3),
            dtype=torch.float32,
            device=env.device,
        )

    if "cube_yaw_offset_rad" not in buffers:
        buffers["cube_yaw_offset_rad"] = torch.zeros(
            env.num_envs,
            dtype=torch.float32,
            device=env.device,
        )

    if "youngs_modulus_pa" not in buffers:
        buffers["youngs_modulus_pa"] = torch.full(
            (env.num_envs,),
            float("nan"),
            dtype=torch.float32,
            device=env.device,
        )

    if "poissons_ratio" not in buffers:
        buffers["poissons_ratio"] = torch.full(
            (env.num_envs,),
            float("nan"),
            dtype=torch.float32,
            device=env.device,
        )

    if "cube_dynamic_friction" not in buffers:
        buffers["cube_dynamic_friction"] = torch.full(
            (env.num_envs,),
            float("nan"),
            dtype=torch.float32,
            device=env.device,
        )

    return buffers

def set_deformable_material_values(
    env,
    env_ids,
    asset_name: str,
    youngs_modulus: torch.Tensor | None = None,
    poissons_ratio: torch.Tensor | None = None,
):
    """Set explicit deformable material values for selected environments."""

    env_ids = torch.as_tensor(
        env_ids,
        dtype=torch.long,
        device=env.device,
    )

    if env_ids.numel() == 0:
        return

    cube = env.scene[asset_name]
    material_view = cube.material_physx_view

    if material_view is None:
        raise RuntimeError(
            f"Deformable asset '{asset_name}' "
            "has no PhysX material view."
        )

    if material_view.count != cube.num_instances:
        raise RuntimeError(
            "Expected one deformable material per cube instance. "
            f"material_count={material_view.count}, "
            f"cube_instances={cube.num_instances}."
        )

    ids_np = (
        env_ids.detach()
        .cpu()
        .numpy()
        .astype(np.int32)
    )

    ids_wp = wp.from_numpy(
        ids_np,
        dtype=wp.int32,
        device="cpu",
    )

    randomization = get_randomization_buffers(env)

    if youngs_modulus is not None:
        values = torch.as_tensor(
            youngs_modulus,
            dtype=torch.float32,
            device=env.device,
        ).reshape(-1)

        if values.shape[0] != env_ids.numel():
            raise ValueError(
                "Young's-modulus value count does not "
                "match env_ids."
            )

        if torch.any(values <= 0.0):
            raise ValueError(
                "Young's modulus must be positive."
            )

        current = (
            material_view
            .get_youngs_modulus()
            .numpy()
            .copy()
            .astype(np.float32)
        )

        current[ids_np, 0] = (
            values.detach()
            .cpu()
            .numpy()
        )

        material_view.set_youngs_modulus(
            wp.from_numpy(
                current,
                dtype=wp.float32,
                device="cpu",
            ),
            ids_wp,
        )

        randomization["youngs_modulus_pa"][
            env_ids
        ] = values

    if poissons_ratio is not None:
        values = torch.as_tensor(
            poissons_ratio,
            dtype=torch.float32,
            device=env.device,
        ).reshape(-1)

        if values.shape[0] != env_ids.numel():
            raise ValueError(
                "Poisson-ratio value count does not "
                "match env_ids."
            )

        if torch.any(
            (values < 0.0)
            | (values >= 0.5)
        ):
            raise ValueError(
                "Poisson's ratio must satisfy "
                "0 <= nu < 0.5."
            )

        current = (
            material_view
            .get_poissons_ratio()
            .numpy()
            .copy()
            .astype(np.float32)
        )

        current[ids_np, 0] = (
            values.detach()
            .cpu()
            .numpy()
        )

        material_view.set_poissons_ratio(
            wp.from_numpy(
                current,
                dtype=wp.float32,
                device="cpu",
            ),
            ids_wp,
        )

        randomization["poissons_ratio"][
            env_ids
        ] = values
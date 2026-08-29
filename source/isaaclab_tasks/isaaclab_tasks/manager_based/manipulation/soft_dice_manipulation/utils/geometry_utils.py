from __future__ import annotations

from itertools import (
    permutations,
    product,
)

import torch

from isaaclab.utils.math import (
    euler_xyz_from_quat,
    matrix_from_quat,
    quat_apply_inverse,
    quat_error_magnitude,
    quat_inv,
    quat_mul,
)

def vector_error(
    reference: torch.Tensor,
    current: torch.Tensor,
) -> torch.Tensor:
    """Euclidean error between vectors along the last dimension."""
    return torch.linalg.norm(
        current - reference,
        dim=-1,
    )


def xy_position_error(
    reference_pos: torch.Tensor,
    current_pos: torch.Tensor,
) -> torch.Tensor:
    """Planar XY position error.

    Args:
        reference_pos: Reference position with shape (..., 3).
        current_pos: Current position with shape (..., 3).

    Returns:
        XY Euclidean distance with shape (...).
    """
    return vector_error(
        reference_pos[..., :2],
        current_pos[..., :2],
    )

def z_position_error(
    reference_pos: torch.Tensor,
    current_pos: torch.Tensor,
) -> torch.Tensor:
    """Absolute vertical position error."""
    return torch.abs(
        current_pos[..., 2]
        - reference_pos[..., 2]
    )


def orientation_error(
    reference_quat: torch.Tensor,
    current_quat: torch.Tensor,
) -> torch.Tensor:
    """Shortest angular difference between two XYZW quaternions.

    Returns:
        Angular error in radians with shape (...).
    """
    return quat_error_magnitude(
        reference_quat,
        current_quat,
    )

def relative_euler_xyz_error(
    reference_quat: torch.Tensor,
    current_quat: torch.Tensor,
) -> torch.Tensor:
    """Signed XYZ Euler decomposition of current relative to reference."""

    relative_quat = quat_mul(current_quat, quat_inv(reference_quat))
    original_shape = relative_quat.shape[:-1]

    roll, pitch, yaw = euler_xyz_from_quat(relative_quat.reshape(-1, 4))

    return torch.stack((roll, pitch, yaw), dim=-1).reshape(*original_shape, 3)


def top_face_index(quat: torch.Tensor) -> torch.Tensor:
    """Return the local cube face currently pointing upward.
    """

    world_up = torch.zeros_like(quat[..., :3])
    world_up[..., 2] = 1.0

    local_up = quat_apply_inverse(quat, world_up)
    axis = torch.argmax(torch.abs(local_up), dim=-1)

    signed_component = torch.gather(local_up, -1, axis.unsqueeze(-1)).squeeze(-1)
    negative = (signed_component < 0.0).long()

    return 2 * axis + negative


def same_top_face(
    reference_quat: torch.Tensor,
    current_quat: torch.Tensor,
) -> torch.Tensor:
    """Whether current and reference have the same physical cube face on top."""

    return top_face_index(reference_quat) == top_face_index(current_quat)

def position_in_frame(
    position: torch.Tensor,
    frame_position: torch.Tensor,
    frame_orientation: torch.Tensor,
) -> torch.Tensor:
    """Express a position in a given reference frame."""

    return quat_apply_inverse(
        frame_orientation,
        position - frame_position,
    )

def orientation_in_frame(
    orientation: torch.Tensor,
    frame_orientation: torch.Tensor,
) -> torch.Tensor:
    """Express an orientation relative to a reference frame.

    Args:
        orientation:
            Object orientation in the parent frame, XYZW.

        frame_orientation:
            Reference-frame orientation in the same parent frame,
            XYZW.

    Returns:
        Object orientation expressed in the reference frame.
    """

    return quat_mul(
        quat_inv(frame_orientation),
        orientation,
    )

def _build_cube_canonical_rotations() -> torch.Tensor:
    """Construct the 24 proper axis-aligned rotations of a cube."""

    rotations = []

    for permutation in permutations(
        range(3)
    ):
        for signs in product(
            (-1.0, 1.0),
            repeat=3,
        ):
            rotation = torch.zeros(
                (3, 3),
                dtype=torch.float32,
            )

            for column, (
                axis,
                sign,
            ) in enumerate(
                zip(
                    permutation,
                    signs,
                )
            ):
                rotation[
                    axis,
                    column,
                ] = sign

            # Keep proper rotations only:
            # det(R) = +1.
            if torch.det(rotation) > 0.0:
                rotations.append(
                    rotation
                )

    result = torch.stack(
        rotations,
        dim=0,
    )

    if result.shape != (
        24,
        3,
        3,
    ):
        raise RuntimeError(
            "Expected 24 canonical cube rotations, "
            f"got {result.shape}."
        )

    return result


def _top_face_index_from_matrix(
    rotation: torch.Tensor,
) -> torch.Tensor:
    """Return the local cube face pointing upward.

    Face convention matches top_face_index():

        0 -> +X
        1 -> -X
        2 -> +Y
        3 -> -Y
        4 -> +Z
        5 -> -Z
    """

    world_up = torch.zeros(
        rotation.shape[:-2] + (3,),
        dtype=rotation.dtype,
        device=rotation.device,
    )

    world_up[..., 2] = 1.0

    # Express world-up in the local cube frame.
    local_up = (
        rotation.transpose(
            -1,
            -2,
        )
        @ world_up.unsqueeze(-1)
    ).squeeze(-1)

    axis = torch.argmax(
        torch.abs(local_up),
        dim=-1,
    )

    signed_component = torch.gather(
        local_up,
        dim=-1,
        index=axis.unsqueeze(-1),
    ).squeeze(-1)

    negative = (
        signed_component < 0.0
    ).long()

    return (
        2 * axis
        + negative
    )

_CUBE_CANONICAL_ROTATIONS_CPU = (
    _build_cube_canonical_rotations()
)

_CUBE_CANONICAL_TOP_FACES_CPU = (
    _top_face_index_from_matrix(
        _CUBE_CANONICAL_ROTATIONS_CPU
    )
)

_CUBE_CANONICAL_CACHE = {}


def _cube_canonical_data(
    device: torch.device,
    dtype: torch.dtype,
) -> tuple[
    torch.Tensor,
    torch.Tensor,
]:
    """Return canonical rotations and top-face ids on a device."""

    key = (
        str(device),
        dtype,
    )

    if key not in _CUBE_CANONICAL_CACHE:
        rotations = (
            _CUBE_CANONICAL_ROTATIONS_CPU.to(
                device=device,
                dtype=dtype,
            )
        )

        top_faces = (
            _CUBE_CANONICAL_TOP_FACES_CPU.to(
                device=device,
            )
        )

        _CUBE_CANONICAL_CACHE[
            key
        ] = (
            rotations,
            top_faces,
        )

    return _CUBE_CANONICAL_CACHE[
        key
    ]

def canonical_cube_target_from_reference(
    reference_quat: torch.Tensor,
) -> tuple[
    torch.Tensor,
    torch.Tensor,
]:
    """Choose the desired exact canonical cube orientation.

    Selection is performed in two steps:

    1. Determine the intended top face from the final reference.
    2. Among the four canonical orientations with that top face,
       choose the one closest to the final reference.

    The four candidates differ only by 90-degree rotations around
    the vertical axis.

    Args:
        reference_quat:
            Final demonstrated orientation, XYZW, shape (..., 4).

    Returns:
        desired_matrix:
            Exact canonical target rotation, shape (..., 3, 3).

        desired_top_face:
            Desired top-face index, shape (...).
    """

    reference_matrix = (
        matrix_from_quat(
            reference_quat
        )
    )

    desired_top_face = (
        top_face_index(
            reference_quat
        )
    )

    (
        canonical_rotations,
        canonical_top_faces,
    ) = _cube_canonical_data(
        device=reference_matrix.device,
        dtype=reference_matrix.dtype,
    )

    # Similarity between reference and every canonical orientation.
    scores = torch.einsum(
        "...ij,kij->...k",
        reference_matrix,
        canonical_rotations,
    )

    # Only allow canonical orientations that preserve
    # the desired top face.
    valid_top_face = (
        canonical_top_faces.view(
            *((1,) * desired_top_face.ndim),
            24,
        )
        == desired_top_face.unsqueeze(-1)
    )

    scores = torch.where(
        valid_top_face,
        scores,
        torch.full_like(
            scores,
            -torch.inf,
        ),
    )

    target_index = torch.argmax(
        scores,
        dim=-1,
    )

    desired_matrix = (
        canonical_rotations[
            target_index
        ]
    )

    return (
        desired_matrix,
        desired_top_face,
    )

def canonical_cube_orientation_error(
    reference_quat: torch.Tensor,
    current_quat: torch.Tensor,
) -> torch.Tensor:
    """Orientation error to the canonical target defined by the reference.

    The final reference determines:
        - which face should be on top;
        - which 90-degree yaw state is intended.

    The actual target is perfectly axis aligned.
    """

    (
        desired_matrix,
        _,
    ) = canonical_cube_target_from_reference(
        reference_quat
    )

    current_matrix = matrix_from_quat(
        current_quat
    )

    relative_matrix = (
        desired_matrix.transpose(
            -1,
            -2,
        )
        @ current_matrix
    )

    trace = (
        relative_matrix[..., 0, 0]
        + relative_matrix[..., 1, 1]
        + relative_matrix[..., 2, 2]
    )

    cosine = (
        (trace - 1.0)
        / 2.0
    ).clamp(
        -1.0,
        1.0,
    )

    return torch.acos(
        cosine
    )
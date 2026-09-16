from __future__ import annotations

from itertools import (
    permutations,
    product,
)

import torch

from isaaclab.utils.math import (
    euler_xyz_from_quat,
    matrix_from_quat,
    quat_apply,
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

def position_from_frame_offset(
    frame_position_w: torch.Tensor,
    frame_orientation_w: torch.Tensor,
    offset_frame: torch.Tensor | tuple[float, float, float],
) -> torch.Tensor:
    """Transform a fixed local-frame position offset into world coordinates.

    Args:
        frame_position_w:
            Position of the parent frame in world coordinates,
            shape (..., 3).

        frame_orientation_w:
            Orientation of the parent frame in world coordinates,
            XYZW quaternion with shape (..., 4).

        offset_frame:
            Position offset expressed in the parent frame,
            shape (3,) or (..., 3).

    Returns:
        Position of the offset point in world coordinates,
        shape (..., 3).
    """

    if frame_position_w.shape[-1] != 3:
        raise ValueError(
            "frame_position_w must have shape (..., 3), "
            f"got {tuple(frame_position_w.shape)}."
        )

    if frame_orientation_w.shape[-1] != 4:
        raise ValueError(
            "frame_orientation_w must have shape (..., 4), "
            f"got {tuple(frame_orientation_w.shape)}."
        )

    offset_frame = torch.as_tensor(
        offset_frame,
        dtype=frame_position_w.dtype,
        device=frame_position_w.device,
    )

    # Expand one fixed offset across all environments if necessary.
    if offset_frame.ndim == 1:
        offset_frame = offset_frame.expand_as(
            frame_position_w
        )

    offset_w = quat_apply(
        frame_orientation_w,
        offset_frame,
    )

    return frame_position_w + offset_w

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

def compute_cube_surface_topness(
    cube_rotation: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Compute world-space cube surface normals and their topness.

    The six geometric cube surfaces follow the convention:

        0 -> +X
        1 -> -X
        2 -> +Y
        3 -> -Y
        4 -> +Z
        5 -> -Z

    where X/Y/Z refer to the cube reference/material frame.

    Args:
        cube_rotation:
            Rotation matrix from the cube reference frame to the
            current world orientation, with shape (..., 3, 3).

    Returns:
        topness:
            Dot product of each world-space surface normal with
            world up (+Z), with shape (..., 6).

        surface_normals_w:
            The six cube surface normals expressed in world axes,
            with shape (..., 6, 3).
    """

    if cube_rotation.shape[-2:] != (3, 3):
        raise ValueError(
            "cube_rotation must have shape (..., 3, 3), "
            f"got {tuple(cube_rotation.shape)}."
        )

    # Columns of R are the cube's local +X, +Y, +Z axes expressed in world coordinates.
    positive_axes_w = cube_rotation.transpose(-1, -2)

    surface_normals_w = torch.stack(
        (
            positive_axes_w[..., 0, :],   # +X
            -positive_axes_w[..., 0, :],  # -X
            positive_axes_w[..., 1, :],   # +Y
            -positive_axes_w[..., 1, :],  # -Y
            positive_axes_w[..., 2, :],   # +Z
            -positive_axes_w[..., 2, :],  # -Z
        ),
        dim=-2,
    )

    # Table/world up is +Z, so n · z_hat is simply the Z component of each world-space normal.
    topness = surface_normals_w[..., 2]

    return topness, surface_normals_w

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

def compute_cube_surface_visibility(
    surface_normals: torch.Tensor,
    topness: torch.Tensor,
    cube_position: torch.Tensor,
    camera_position: torch.Tensor,
    topness_eps: float = 1.0e-1,
    camera_eps: float = 1.0e-3,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Compute camera-facing scores and valid visible-surface candidates.

    A cube surface is considered a valid candidate only if:

        1. it points upward:
               topness > topness_eps

        2. it points toward the camera:
               camera_facing_score > camera_eps

    Args:
        surface_normals:
            Cube surface normals expressed in a common frame,
            shape (..., 6, 3).

        topness:
            Dot product of each surface normal with world/table up,
            shape (..., 6).

        cube_position:
            Cube center position expressed in the same frame as
            camera_position, shape (..., 3).

        camera_position:
            Camera position, shape (..., 3).

        topness_eps:
            Small positive tolerance used to reject horizontal or
            downward-facing surfaces.

        camera_eps:
            Small positive tolerance used to reject surfaces facing
            away from, or exactly perpendicular to, the camera.

    Returns:
        camera_facing_score:
            Cosine between each surface normal and the direction
            from the cube toward the camera, shape (..., 6).

            +1 -> directly facing the camera
             0 -> perpendicular to camera direction
            -1 -> directly facing away

        candidate_mask:
            Boolean tensor with shape (..., 6). True only for
            upward-facing and camera-facing surfaces.
    """

    if surface_normals.shape[-2:] != (6, 3):
        raise ValueError(
            "surface_normals must have shape (..., 6, 3), "
            f"got {tuple(surface_normals.shape)}."
        )

    if topness.shape[-1] != 6:
        raise ValueError(
            "topness must have shape (..., 6), "
            f"got {tuple(topness.shape)}."
        )

    if cube_position.shape[-1] != 3:
        raise ValueError(
            "cube_position must have shape (..., 3), "
            f"got {tuple(cube_position.shape)}."
        )

    if camera_position.shape[-1] != 3:
        raise ValueError(
            "camera_position must have shape (..., 3), "
            f"got {tuple(camera_position.shape)}."
        )

    # Unit vector pointing from the cube center toward the camera.
    cube_to_camera = camera_position - cube_position

    cube_to_camera = cube_to_camera / torch.linalg.vector_norm(
        cube_to_camera,
        dim=-1,
        keepdim=True,
    ).clamp_min(1.0e-8)

    # Compare every surface normal against the direction toward the camera.
    camera_facing_score = torch.sum(
        surface_normals * cube_to_camera.unsqueeze(-2),
        dim=-1,
    )

    upward_facing = topness > topness_eps
    camera_facing = camera_facing_score > camera_eps
    candidate_mask = upward_facing& camera_facing

    return (
        camera_facing_score,
        candidate_mask,
    )

def compute_cube_topness_visibility_state(
    cube_rotation: torch.Tensor,
    cube_position: torch.Tensor,
    torso_position: torch.Tensor,
    torso_orientation: torch.Tensor,
    camera_offset_torso: torch.Tensor | tuple[float, float, float],
    topness_eps: float = 1.0e-1,
    camera_eps: float = 1.0e-3,
) -> dict[str, torch.Tensor]:
    """Compute cube topness and camera-facing visibility quantities."""

    camera_position = position_from_frame_offset(
        frame_position_w=torso_position,
        frame_orientation_w=torso_orientation,
        offset_frame=camera_offset_torso,
    )

    topness, surface_normals = (
        compute_cube_surface_topness(
            cube_rotation
        )
    )

    camera_facing_score, candidate_mask = (
        compute_cube_surface_visibility(
            surface_normals=surface_normals,
            topness=topness,
            cube_position=cube_position,
            camera_position=camera_position,
            topness_eps=topness_eps,
            camera_eps=camera_eps,
        )
    )

    return {
        "topness": topness,
        "surface_normals": surface_normals,
        "camera_position": camera_position,
        "cube_position": cube_position,
        "camera_facing_score": camera_facing_score,
        "candidate_mask": candidate_mask,
    }

# -------------------------------------------------------------------------
# Semantic dice-face mappings.
#
# Geometric surface ordering:
#   0 -> +X
#   1 -> -X
#   2 -> +Y
#   3 -> -Y
#   4 -> +Z
#   5 -> -Z
#
# Canonical symbolic orientation:
#   front=3, back=5, right=2, left=4, top=1, bottom=6
# -------------------------------------------------------------------------

_CUBE_SURFACE_NORMALS_CPU = torch.tensor(
    [
        [1.0, 0.0, 0.0],    # +X
        [-1.0, 0.0, 0.0],   # -X
        [0.0, 1.0, 0.0],    # +Y
        [0.0, -1.0, 0.0],   # -Y
        [0.0, 0.0, 1.0],    # +Z
        [0.0, 0.0, -1.0],   # -Z
    ],
    dtype=torch.float32,
)

# For every geometric surface, which numbered-face slot occupies it in the canonical semantic orientation.
#
# +X -> face 3 -> slot 2
# -X -> face 5 -> slot 4
# +Y -> face 2 -> slot 1
# -Y -> face 4 -> slot 3
# +Z -> face 1 -> slot 0
# -Z -> face 6 -> slot 5
_CUBE_BASE_SURFACE_TO_FACE_SLOT_CPU = torch.tensor( [2, 4, 1, 3, 0, 5], dtype=torch.long)


def _build_cube_semantic_surface_mappings() -> torch.Tensor:
    """Build all 24 valid surface-to-face mappings of a numbered cube.

    Returns:
        Tensor with shape (24, 6).

        Each row maps geometric surface index -> semantic face slot.

        For example:
            mapping[0, 4] == 2

        means geometric +Z currently carries semantic face slot 2,
        corresponding to numbered face 3.
    """

    rotations = _CUBE_CANONICAL_ROTATIONS_CPU
    normals = _CUBE_SURFACE_NORMALS_CPU

    # Apply every proper cube rotation to every canonical surface normal.
    rotated_normals = torch.einsum(
        "kij,fj->kfi",
        rotations,
        normals,
    )

    # Identify which canonical geometric surface each rotated normal coincides with.
    similarity = torch.einsum(
        "kfi,gi->kfg",
        rotated_normals,
        normals,
    )

    target_surface = torch.argmax(similarity,dim=-1,)
    mappings = torch.empty((24, 6),dtype=torch.long)

    # The semantic face originally attached to source_surface moves to target_surface after the virtual cube rotation.
    for rotation_idx in range(24):
        mappings[
            rotation_idx,
            target_surface[rotation_idx],
        ] = _CUBE_BASE_SURFACE_TO_FACE_SLOT_CPU

    return mappings


_CUBE_SEMANTIC_SURFACE_MAPPINGS_CPU = (
    _build_cube_semantic_surface_mappings()
)

def cube_semantic_surface_mappings(
    device: torch.device | str,
) -> torch.Tensor:
    """Return all 24 valid semantic cube mappings on the requested device."""

    return _CUBE_SEMANTIC_SURFACE_MAPPINGS_CPU.to(
        device=device,
    )

def map_surface_topness_to_semantic_faces(
    topness: torch.Tensor,
    candidate_mask: torch.Tensor,
    surface_to_face_slot: torch.Tensor,
    missing_value: float = -1.0,
) -> torch.Tensor:
    """Map geometric cube-surface topness to semantic numbered-face slots.

    Geometric surface convention:
        0 -> +X
        1 -> -X
        2 -> +Y
        3 -> -Y
        4 -> +Z
        5 -> -Z

    Semantic output convention:
        slot 0 -> face 1
        slot 1 -> face 2
        ...
        slot 5 -> face 6

    Args:
        topness:
            Geometric surface topness values, shape (..., 6).

        candidate_mask:
            Boolean validity mask for geometric surfaces,
            shape (..., 6).

        surface_to_face_slot:
            Mapping from geometric surface index to semantic face slot,
            shape (..., 6).

            Example:
                [2, 4, 1, 3, 0, 5]

            means:
                +X -> face 3
                -X -> face 5
                +Y -> face 2
                -Y -> face 4
                +Z -> face 1
                -Z -> face 6

        missing_value:
            Value used for semantic faces that are not currently
            visible/useful.

    Returns:
        Semantic face-indexed topness observation with shape (..., 6):

            [face1, face2, face3, face4, face5, face6]
    """

    if topness.shape[-1] != 6:
        raise ValueError(
            "topness must have shape (..., 6), "
            f"got {tuple(topness.shape)}."
        )

    if candidate_mask.shape != topness.shape:
        raise ValueError(
            "candidate_mask must have the same shape as topness, "
            f"got {tuple(candidate_mask.shape)} and "
            f"{tuple(topness.shape)}."
        )

    if surface_to_face_slot.shape != topness.shape:
        raise ValueError(
            "surface_to_face_slot must have the same shape as topness, "
            f"got {tuple(surface_to_face_slot.shape)} and "
            f"{tuple(topness.shape)}."
        )

    semantic_topness = torch.full_like(
        topness,
        missing_value,
    )

    valid_values = torch.where(
        candidate_mask,
        topness,
        torch.full_like(
            topness,
            missing_value,
        ),
    )

    semantic_topness.scatter_(
        dim=-1,
        index=surface_to_face_slot,
        src=valid_values,
    )

    return semantic_topness
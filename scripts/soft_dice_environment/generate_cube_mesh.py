# #!/usr/bin/env python3
# """
# Generate and visualize a superquadric cuboid mesh.

# Dependencies:
#     pip install numpy matplotlib

# Optional OBJ export is built in and does not require trimesh.
# """

# import argparse
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# def signed_power(x, p):
#     """Return sign(x) * |x|**p, valid for non-integer powers."""
#     return np.sign(x) * (np.abs(x) ** p)


# def superquadric_cuboid(
#     a=0.04, b=0.04, c=0.04,
#     eps1=0.25, eps2=0.25,
#     n_eta=48, n_omega=96,
#     remove_pole_duplicates=True,
# ):
#     """
#     Create a superellipsoid / superquadric cuboid surface mesh.

#     Geometry equation, centered at origin:
#         ( |x/a|^(2/eps2) + |y/b|^(2/eps2) )^(eps2/eps1)
#         + |z/c|^(2/eps1) = 1

#     Parametric form:
#         eta   in [-pi/2, pi/2]  latitude-like variable
#         omega in [-pi, pi]      longitude-like variable

#         x = a * sgn(cos eta)|cos eta|^eps1 * sgn(cos omega)|cos omega|^eps2
#         y = b * sgn(cos eta)|cos eta|^eps1 * sgn(sin omega)|sin omega|^eps2
#         z = c * sgn(sin eta)|sin eta|^eps1

#     Parameters:
#         a, b, c      half-size along x, y, z. Full dimensions are 2a, 2b, 2c.
#         eps1         vertical squareness: z profile / top-bottom flattening.
#         eps2         horizontal squareness: xy cross-section / side-corner sharpness.
#         n_eta        number of latitude samples. More -> smoother mesh.
#         n_omega      number of longitude samples. More -> smoother mesh.

#     Typical eps values:
#         eps = 1.0    ellipsoid-like
#         eps < 1.0    boxier / flatter faces / sharper edges
#         eps ~ 0.15   very cuboid-like but may create skinny triangles near edges
#         eps > 1.0    pinched / diamond-ish shapes, usually not wanted for a cuboid
#     """
#     eta = np.linspace(-np.pi / 2, np.pi / 2, n_eta + 1)
#     omega = np.linspace(-np.pi, np.pi, n_omega, endpoint=False)
#     Eta, Omega = np.meshgrid(eta, omega, indexing="ij")

#     X = a * signed_power(np.cos(Eta), eps1) * signed_power(np.cos(Omega), eps2)
#     Y = b * signed_power(np.cos(Eta), eps1) * signed_power(np.sin(Omega), eps2)
#     Z = c * signed_power(np.sin(Eta), eps1)

#     vertices = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])

#     faces = []

#     def vid(i, j):
#         return i * n_omega + (j % n_omega)

#     # Quad grid split into two triangles per cell.
#     for i in range(n_eta):
#         for j in range(n_omega):
#             v00 = vid(i, j)
#             v01 = vid(i, j + 1)
#             v10 = vid(i + 1, j)
#             v11 = vid(i + 1, j + 1)

#             faces.append([v00, v10, v11])
#             faces.append([v00, v11, v01])

#     faces = np.asarray(faces, dtype=np.int64)

#     # Optional cleanup: merge all bottom pole points and all top pole points.
#     # This avoids many duplicated vertices where cos(eta)=0.
#     if remove_pole_duplicates:
#         vertices, faces = merge_duplicate_vertices(vertices, faces, decimals=12)

#     return vertices, faces


# def merge_duplicate_vertices(vertices, faces, decimals=12):
#     """Merge vertices that are identical up to rounding."""
#     rounded = np.round(vertices, decimals=decimals)
#     unique, inverse = np.unique(rounded, axis=0, return_inverse=True)
#     new_faces = inverse[faces]

#     # Remove degenerate triangles created by vertex merging.
#     keep = []
#     for f in new_faces:
#         if len(set(f.tolist())) == 3:
#             keep.append(f)

#     return unique, np.asarray(keep, dtype=np.int64)



# def write_obj(path, vertices, faces):
#     """Write mesh as a simple Wavefront OBJ."""
#     with open(path, "w") as f:
#         for v in vertices:
#             # v is one vertex: [x, y, z]
#             f.write(f"v {v[0]:9g} {v[1]:9g} {v[2]:9g}\n")
    
#         for tri in faces:
#             # OBJ indices are 1-based, while Python indices are 0-based
#             f.write(f"f {tri[0] + 1} {tri[1] + 1} {tri[2] + 1}\n")

# def triangle_quality(vertices, faces):
#     """Return simple mesh diagnostics: area stats and edge-length stats."""
#     tri = vertices[faces]

#     e01 = np.linalg.norm(tri[:, 1] - tri[:, 0], axis=1)
#     e12 = np.linalg.norm(tri[:, 2] - tri[:, 1], axis=1)
#     e20 = np.linalg.norm(tri[:, 0] - tri[:, 2], axis=1)

#     areas = 0.5 * np.linalg.norm(
#         np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]),
#         axis=1,
#     )

#     edges = np.concatenate([e01, e12, e20])

#     return {
#         "num_vertices": len(vertices),
#         "num_faces": len(faces),
#         "min_area": float(areas.min()),
#         "mean_area": float(areas.mean()),
#         "max_area": float(areas.max()),
#         "min_edge": float(edges.min()),
#         "mean_edge": float(edges.mean()),
#         "max_edge": float(edges.max()),
#     }


# def plot_mesh(vertices, faces, outfile=None, show_edges=True):
#     fig = plt.figure(figsize=(8, 7))
#     ax = fig.add_subplot(111, projection="3d")

#     tri_vertices = vertices[faces]

#     collection = Poly3DCollection(
#         tri_vertices,
#         alpha=0.88,
#         linewidths=0.15 if show_edges else 0.0,
#         edgecolors="k" if show_edges else "none",
#     )
#     collection.set_facecolor((0.35, 0.65, 1.0, 0.88))
#     ax.add_collection3d(collection)

#     mins = vertices.min(axis=0)
#     maxs = vertices.max(axis=0)
#     center = 0.5 * (mins + maxs)
#     radius = 0.55 * np.max(maxs - mins)

#     ax.set_xlim(center[0] - radius, center[0] + radius)
#     ax.set_ylim(center[1] - radius, center[1] + radius)
#     ax.set_zlim(center[2] - radius, center[2] + radius)

#     ax.set_box_aspect([1, 1, 1])
#     ax.set_xlabel("x [m]")
#     ax.set_ylabel("y [m]")
#     ax.set_zlabel("z [m]")
#     ax.set_title("Superquadric cuboid mesh")

#     plt.tight_layout()

#     if outfile:
#         plt.savefig(outfile, dpi=220)
#         print(f"Saved plot to: {outfile}")
#     else:
#         plt.show()


# def main():
#     parser = argparse.ArgumentParser()

#     parser.add_argument("--a", type=float, default=0.04, help="half-size along x")
#     parser.add_argument("--b", type=float, default=0.04, help="half-size along y")
#     parser.add_argument("--c", type=float, default=0.04, help="half-size along z")

#     parser.add_argument("--eps1", type=float, default=0.25, help="vertical squareness")
#     parser.add_argument("--eps2", type=float, default=0.25, help="horizontal squareness")

#     parser.add_argument("--n_eta", type=int, default=56, help="latitude resolution")
#     parser.add_argument("--n_omega", type=int, default=112, help="longitude resolution")

#     parser.add_argument("--obj", type=str, default="superquadric_cuboid.obj")
#     parser.add_argument("--png", type=str, default="superquadric_cuboid.png")
#     parser.add_argument("--no_edges", action="store_true")

#     args = parser.parse_args()

#     V, F = superquadric_cuboid(
#         a=args.a,
#         b=args.b,
#         c=args.c,
#         eps1=args.eps1,
#         eps2=args.eps2,
#         n_eta=args.n_eta,
#         n_omega=args.n_omega,
#     )

#     stats = triangle_quality(V, F)

#     print("Mesh diagnostics:")
#     for k, v in stats.items():
#         print(f"  {k}: {v}")

#     write_obj(args.obj, V, F)
#     print(f"Saved OBJ to: {args.obj}")

#     plot_mesh(V, F, outfile=args.png, show_edges=not args.no_edges)


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3

"""
Generate a watertight superquadric cuboid surface using six projected
cube-face grids.

This avoids the pole singularities and highly connected pole vertices of
latitude/longitude superquadric parameterization.
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def project_direction_to_superquadric(
    directions,
    a,
    b,
    c,
    eps1,
    eps2,
):
    """
    Project nonzero directions radially onto the superquadric:

        (
            |x/a|^(2/eps2)
            + |y/b|^(2/eps2)
        )^(eps2/eps1)
        + |z/c|^(2/eps1)
        = 1

    If p = t*d, the implicit expression scales as t^(2/eps1).
    Therefore:

        t = implicit(d)^(-eps1/2)
    """

    directions = np.asarray(
        directions,
        dtype=np.float64,
    )

    if directions.ndim != 2 or directions.shape[1] != 3:
        raise ValueError(
            "directions must have shape (N, 3)"
        )

    x = directions[:, 0]
    y = directions[:, 1]
    z = directions[:, 2]

    xy_term = (
        (np.abs(x) / a) ** (2.0 / eps2)
        + (np.abs(y) / b) ** (2.0 / eps2)
    ) ** (eps2 / eps1)

    z_term = (
        np.abs(z) / c
    ) ** (2.0 / eps1)

    implicit_value = xy_term + z_term

    if np.any(implicit_value <= 0.0):
        raise RuntimeError(
            "Projection received a zero or invalid direction."
        )

    scale = implicit_value ** (
        -0.5 * eps1
    )

    return directions * scale[:, None]


def merge_duplicate_vertices(
    vertices,
    faces,
    decimals=12,
):
    """
    Merge coincident vertices, remove degenerate triangles, and remove
    duplicated triangles.
    """

    vertices = np.asarray(
        vertices,
        dtype=np.float64,
    )

    faces = np.asarray(
        faces,
        dtype=np.int64,
    )

    rounded = np.round(
        vertices,
        decimals=decimals,
    )

    unique_vertices, inverse = np.unique(
        rounded,
        axis=0,
        return_inverse=True,
    )

    remapped_faces = inverse[faces]

    nondegenerate_faces = []

    for face in remapped_faces:
        if len(set(face.tolist())) == 3:
            nondegenerate_faces.append(face)

    if not nondegenerate_faces:
        raise RuntimeError(
            "No nondegenerate triangles remained."
        )

    nondegenerate_faces = np.asarray(
        nondegenerate_faces,
        dtype=np.int64,
    )

    # Remove triangles duplicated along cube-face boundaries.
    canonical_faces = np.sort(
        nondegenerate_faces,
        axis=1,
    )

    _, unique_face_indices = np.unique(
        canonical_faces,
        axis=0,
        return_index=True,
    )

    unique_face_indices = np.sort(
        unique_face_indices
    )

    unique_faces = nondegenerate_faces[
        unique_face_indices
    ]

    return unique_vertices, unique_faces


def orient_faces_outward(
    vertices,
    faces,
):
    """
    Ensure triangle normals point away from the origin.

    The generated superquadric is centered at the origin.
    """

    vertices = np.asarray(
        vertices,
        dtype=np.float64,
    )

    faces = np.asarray(
        faces,
        dtype=np.int64,
    ).copy()

    triangles = vertices[faces]

    edge_1 = (
        triangles[:, 1]
        - triangles[:, 0]
    )

    edge_2 = (
        triangles[:, 2]
        - triangles[:, 0]
    )

    normals = np.cross(
        edge_1,
        edge_2,
    )

    centroids = triangles.mean(
        axis=1
    )

    orientation = np.einsum(
        "ij,ij->i",
        normals,
        centroids,
    )

    inward = orientation < 0.0

    faces[inward, 1], faces[inward, 2] = (
        faces[inward, 2].copy(),
        faces[inward, 1].copy(),
    )

    # Recheck.
    triangles = vertices[faces]

    normals = np.cross(
        triangles[:, 1] - triangles[:, 0],
        triangles[:, 2] - triangles[:, 0],
    )

    centroids = triangles.mean(axis=1)

    orientation = np.einsum(
        "ij,ij->i",
        normals,
        centroids,
    )

    if np.any(orientation <= 0.0):
        bad_ids = np.flatnonzero(
            orientation <= 0.0
        )[:20]

        raise RuntimeError(
            "Could not orient all faces outward. "
            f"First bad face IDs: {bad_ids.tolist()}"
        )

    return faces


def make_cube_face(
    axis,
    sign,
    resolution,
):
    """
    Generate one regular cube-face grid.

    Returns:
        vertices: ((resolution + 1)^2, 3)
        faces:    (2 * resolution^2, 3)
    """

    coordinates = np.linspace(
        -1.0,
        1.0,
        resolution + 1,
        dtype=np.float64,
    )

    vertices = []

    for row in range(resolution + 1):
        v = coordinates[row]

        for column in range(resolution + 1):
            u = coordinates[column]

            if axis == 0:
                point = [float(sign), u, v]

            elif axis == 1:
                point = [u, float(sign), v]

            elif axis == 2:
                point = [u, v, float(sign)]

            else:
                raise ValueError(
                    f"Invalid axis: {axis}"
                )

            vertices.append(point)

    vertices = np.asarray(
        vertices,
        dtype=np.float64,
    )

    def vertex_id(row, column):
        return (
            row * (resolution + 1)
            + column
        )

    faces = []

    for row in range(resolution):
        for column in range(resolution):
            v00 = vertex_id(
                row,
                column,
            )

            v10 = vertex_id(
                row,
                column + 1,
            )

            v01 = vertex_id(
                row + 1,
                column,
            )

            v11 = vertex_id(
                row + 1,
                column + 1,
            )

            # Alternating diagonal avoids imposing one global diagonal
            # direction across every quad.
            if (row + column) % 2 == 0:
                faces.append(
                    [v00, v10, v11]
                )

                faces.append(
                    [v00, v11, v01]
                )

            else:
                faces.append(
                    [v00, v10, v01]
                )

                faces.append(
                    [v10, v11, v01]
                )

    return (
        vertices,
        np.asarray(
            faces,
            dtype=np.int64,
        ),
    )


def superquadric_from_cube_faces(
    a=0.155,
    b=0.155,
    c=0.155,
    eps1=0.25,
    eps2=0.25,
    face_resolution=10,
):
    """
    Generate a closed superquadric surface from six cube-face grids.

    Approximate output counts after shared-edge/corner merging:

        vertices = 6 * resolution^2 + 2
        triangles = 12 * resolution^2

    Examples:
        resolution=8:
            approximately 386 vertices, 768 triangles

        resolution=10:
            approximately 602 vertices, 1200 triangles

        resolution=12:
            approximately 866 vertices, 1728 triangles
    """

    if face_resolution < 2:
        raise ValueError(
            "face_resolution must be at least 2."
        )

    all_vertices = []
    all_faces = []

    vertex_offset = 0

    for axis in (0, 1, 2):
        for sign in (-1, 1):
            face_vertices, face_triangles = (
                make_cube_face(
                    axis=axis,
                    sign=sign,
                    resolution=face_resolution,
                )
            )

            all_vertices.append(
                face_vertices
            )

            all_faces.append(
                face_triangles
                + vertex_offset
            )

            vertex_offset += len(
                face_vertices
            )

    cube_vertices = np.concatenate(
        all_vertices,
        axis=0,
    )

    faces = np.concatenate(
        all_faces,
        axis=0,
    )

    projected_vertices = (
        project_direction_to_superquadric(
            directions=cube_vertices,
            a=a,
            b=b,
            c=c,
            eps1=eps1,
            eps2=eps2,
        )
    )

    vertices, faces = (
        merge_duplicate_vertices(
            projected_vertices,
            faces,
            decimals=12,
        )
    )

    faces = orient_faces_outward(
        vertices,
        faces,
    )

    return vertices, faces


def analyze_surface_mesh(
    vertices,
    faces,
):
    """
    Print triangle, manifold-edge and vertex-valence diagnostics.
    """

    triangles = vertices[faces]

    edge_01 = np.linalg.norm(
        triangles[:, 1]
        - triangles[:, 0],
        axis=1,
    )

    edge_12 = np.linalg.norm(
        triangles[:, 2]
        - triangles[:, 1],
        axis=1,
    )

    edge_20 = np.linalg.norm(
        triangles[:, 0]
        - triangles[:, 2],
        axis=1,
    )

    edge_lengths = np.stack(
        [edge_01, edge_12, edge_20],
        axis=1,
    )

    areas = (
        0.5
        * np.linalg.norm(
            np.cross(
                triangles[:, 1]
                - triangles[:, 0],
                triangles[:, 2]
                - triangles[:, 0],
            ),
            axis=1,
        )
    )

    min_edge = edge_lengths.min(
        axis=1
    )

    max_edge = edge_lengths.max(
        axis=1
    )

    edge_ratio = (
        max_edge
        / np.maximum(min_edge, 1.0e-15)
    )

    # Equilateral triangle quality = 1.
    normalized_area_quality = (
        4.0
        * np.sqrt(3.0)
        * areas
        / np.maximum(
            np.sum(
                edge_lengths**2,
                axis=1,
            ),
            1.0e-30,
        )
    )

    # Count undirected edge usage.
    edge_count = {}

    for face in faces:
        for i, j in (
            (face[0], face[1]),
            (face[1], face[2]),
            (face[2], face[0]),
        ):
            edge = tuple(
                sorted(
                    (int(i), int(j))
                )
            )

            edge_count[edge] = (
                edge_count.get(edge, 0)
                + 1
            )

    boundary_edges = [
        edge
        for edge, count in edge_count.items()
        if count == 1
    ]

    nonmanifold_edges = [
        edge
        for edge, count in edge_count.items()
        if count > 2
    ]

    valence = np.zeros(
        len(vertices),
        dtype=np.int64,
    )

    for face in faces:
        for vertex_id in face:
            valence[int(vertex_id)] += 1

    print("\n" + "=" * 80)
    print("SURFACE MESH QUALITY")
    print("=" * 80)

    print("vertices:", len(vertices))
    print("triangles:", len(faces))

    print("\n[Bounds]")
    print("min:", vertices.min(axis=0))
    print("max:", vertices.max(axis=0))
    print(
        "extent:",
        vertices.max(axis=0)
        - vertices.min(axis=0),
    )

    print("\n[Triangle area]")
    print("minimum:", float(areas.min()))
    print("median:", float(np.median(areas)))
    print("maximum:", float(areas.max()))
    print(
        "minimum / median:",
        float(
            areas.min()
            / max(
                np.median(areas),
                1.0e-30,
            )
        ),
    )

    print("\n[Triangle edge ratio]")
    print(
        "median:",
        float(np.median(edge_ratio)),
    )
    print(
        "95th percentile:",
        float(
            np.percentile(
                edge_ratio,
                95,
            )
        ),
    )
    print(
        "maximum:",
        float(edge_ratio.max()),
    )

    print("\n[Normalized area quality]")
    print(
        "minimum:",
        float(
            normalized_area_quality.min()
        ),
    )
    print(
        "5th percentile:",
        float(
            np.percentile(
                normalized_area_quality,
                5,
            )
        ),
    )
    print(
        "median:",
        float(
            np.median(
                normalized_area_quality
            )
        ),
    )

    print("\n[Topology]")
    print(
        "boundary edges:",
        len(boundary_edges),
    )
    print(
        "nonmanifold edges:",
        len(nonmanifold_edges),
    )
    print(
        "minimum triangle valence:",
        int(valence.min()),
    )
    print(
        "median triangle valence:",
        float(np.median(valence)),
    )
    print(
        "maximum triangle valence:",
        int(valence.max()),
    )

    if len(boundary_edges) != 0:
        raise RuntimeError(
            "The generated mesh is not watertight: "
            f"{len(boundary_edges)} boundary edges."
        )

    if len(nonmanifold_edges) != 0:
        raise RuntimeError(
            "The generated mesh is non-manifold: "
            f"{len(nonmanifold_edges)} edges have more than "
            "two incident triangles."
        )

    return {
        "areas": areas,
        "edge_ratio": edge_ratio,
        "normalized_area_quality":
            normalized_area_quality,
        "valence": valence,
    }


def write_obj(
    path,
    vertices,
    faces,
):
    with open(
        path,
        "w",
        encoding="utf-8",
    ) as file:
        for vertex in vertices:
            file.write(
                f"v "
                f"{vertex[0]:.9g} "
                f"{vertex[1]:.9g} "
                f"{vertex[2]:.9g}\n"
            )

        for face in faces:
            file.write(
                f"f "
                f"{face[0] + 1} "
                f"{face[1] + 1} "
                f"{face[2] + 1}\n"
            )


def plot_mesh(
    vertices,
    faces,
    output_path=None,
    show_edges=True,
):
    figure = plt.figure(
        figsize=(8, 7)
    )

    axis = figure.add_subplot(
        111,
        projection="3d",
    )

    triangles = vertices[faces]

    collection = Poly3DCollection(
        triangles,
        alpha=0.88,
        linewidths=(
            0.2
            if show_edges
            else 0.0
        ),
        edgecolors=(
            "black"
            if show_edges
            else "none"
        ),
    )

    collection.set_facecolor(
        (0.35, 0.65, 1.0, 0.88)
    )

    axis.add_collection3d(
        collection
    )

    minimum = vertices.min(axis=0)
    maximum = vertices.max(axis=0)

    center = 0.5 * (
        minimum + maximum
    )

    radius = 0.55 * np.max(
        maximum - minimum
    )

    axis.set_xlim(
        center[0] - radius,
        center[0] + radius,
    )

    axis.set_ylim(
        center[1] - radius,
        center[1] + radius,
    )

    axis.set_zlim(
        center[2] - radius,
        center[2] + radius,
    )

    axis.set_box_aspect(
        [1, 1, 1]
    )

    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_zlabel("z [m]")

    axis.set_title(
        "Cube-face projected superquadric"
    )

    plt.tight_layout()

    if output_path:
        plt.savefig(
            output_path,
            dpi=220,
        )

        print(
            "Saved plot to:",
            output_path,
        )

    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--a",
        type=float,
        default=0.155,
    )

    parser.add_argument(
        "--b",
        type=float,
        default=0.155,
    )

    parser.add_argument(
        "--c",
        type=float,
        default=0.155,
    )

    parser.add_argument(
        "--eps1",
        type=float,
        default=0.25,
    )

    parser.add_argument(
        "--eps2",
        type=float,
        default=0.25,
    )

    parser.add_argument(
        "--face_resolution",
        type=int,
        default=10,
    )

    parser.add_argument(
        "--obj",
        type=str,
        default=(
            "superquadric_cube_faces.obj"
        ),
    )

    parser.add_argument(
        "--png",
        type=str,
        default=(
            "superquadric_cube_faces.png"
        ),
    )

    parser.add_argument(
        "--no_edges",
        action="store_true",
    )

    args = parser.parse_args()

    vertices, faces = (
        superquadric_from_cube_faces(
            a=args.a,
            b=args.b,
            c=args.c,
            eps1=args.eps1,
            eps2=args.eps2,
            face_resolution=args.face_resolution,
        )
    )

    analyze_surface_mesh(
        vertices,
        faces,
    )

    write_obj(
        args.obj,
        vertices,
        faces,
    )

    print(
        "Saved OBJ to:",
        args.obj,
    )

    plot_mesh(
        vertices,
        faces,
        output_path=args.png,
        show_edges=not args.no_edges,
    )


if __name__ == "__main__":
    main()
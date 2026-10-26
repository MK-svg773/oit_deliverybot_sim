#!/usr/bin/env python3
"""Crop a SLAM cloud and mesh it with Open3D; keep the original coordinates."""

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cloud', type=Path, help='GLIM PLY/PCD in metres')
    parser.add_argument('output', type=Path, help='New OBJ mesh path')
    parser.add_argument('--bounds', nargs=6, type=float, required=True,
                        metavar=('XMIN', 'YMIN', 'ZMIN', 'XMAX', 'YMAX', 'ZMAX'))
    parser.add_argument('--voxel', type=float, default=0.1,
                        help='Downsampling spacing in metres (default: 0.1)')
    args = parser.parse_args()
    lower, upper = np.array(args.bounds).reshape(2, 3)
    if not np.all(np.isfinite(args.bounds)) or np.any(lower >= upper):
        parser.error('bounds must be finite with each minimum < maximum')
    if not np.isfinite(args.voxel) or args.voxel <= 0:
        parser.error('voxel must be positive and finite')
    if not args.cloud.is_file():
        parser.error(f'input cloud not found: {args.cloud}')
    if args.output.suffix.lower() != '.obj' or args.output.exists():
        parser.error('output must be a new .obj file; existing files are not overwritten')

    cloud = o3d.io.read_point_cloud(str(args.cloud), remove_nan_points=True,
                                    remove_infinite_points=True)
    cloud = cloud.crop(o3d.geometry.AxisAlignedBoundingBox(lower, upper))
    cloud = cloud.voxel_down_sample(args.voxel)
    if len(cloud.points) < 31:
        parser.error('too few points in the requested bounds')
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=4 * args.voxel, max_nn=30))
    cloud.orient_normals_consistent_tangent_plane(30)
    # Ball pivoting uses nearby samples; unlike Poisson it does not close the
    # entire surface. Remaining holes are unknown geometry, not verified free space.
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        cloud, o3d.utility.DoubleVector([1.5 * args.voxel, 3 * args.voxel]))
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_unreferenced_vertices()
    if not len(mesh.triangles):
        parser.error('no surface was reconstructed; inspect the cloud and point spacing')
    mesh.compute_vertex_normals()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_triangle_mesh(str(args.output), mesh, write_vertex_colors=False,
                                      write_triangle_uvs=False):
        raise RuntimeError(f'could not write {args.output}')
    print(f'{args.output.resolve()}: {len(mesh.vertices)} vertices, '
          f'{len(mesh.triangles)} triangles; coordinates unchanged')
    print('Inspect holes, frozen moving objects and floor geometry before navigation.')


if __name__ == '__main__':
    main()

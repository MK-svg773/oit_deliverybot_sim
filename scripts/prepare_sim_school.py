#!/usr/bin/env python3
"""
Extrude occupied cells of an existing ROS map into a planar simulation mesh.

This fallback preserves XY occupancy; it does not reproduce slopes or height-dependent occlusion.
Use the existing oit_map_tools grid_map_generator to create the input map from GLIM.
"""
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--bounds', type=float, nargs=4, default=[-10, -10, 10, 10])
    parser.add_argument('--floor-z', type=float, default=-.4)
    parser.add_argument('--height', type=float, default=1.5)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output exists; choose a new mesh path')
    info = yaml.safe_load(args.map.read_text())
    if abs(info['origin'][2]) > 1e-9:
        parser.error('Rotated maps are not supported')
    if info.get('negate', 0):
        parser.error('Use a standard non-negated trinary map')
    image = np.flipud(np.asarray(Image.open(args.map.parent / info['image']).convert('L')))
    resolution = info['resolution']
    ox, oy = info['origin'][:2]
    occupied = image < (1 - info['occupied_thresh']) * 255
    mesh = o3d.geometry.TriangleMesh()
    for row, flags in enumerate(occupied):
        y = oy + row * resolution
        if not args.bounds[1] <= y < args.bounds[3]:
            continue
        xs = ox + (np.arange(len(flags)) + .5) * resolution
        flags = flags & (xs >= args.bounds[0]) & (xs < args.bounds[2])
        changes = np.flatnonzero(np.diff(np.r_[False, flags, False]))
        for start, end in changes.reshape(-1, 2):
            box = o3d.geometry.TriangleMesh.create_box(
                (end - start) * resolution, resolution, args.height)
            box.translate([ox + start * resolution, y, args.floor_z])
            mesh += box
    if not len(mesh.triangles):
        parser.error('No occupied geometry in selected bounds')
    mesh.compute_vertex_normals()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_triangle_mesh(str(args.output), mesh, write_triangle_uvs=False):
        raise RuntimeError('Could not write mesh')
    print(f'{args.output}: {len(mesh.triangles)} triangles; '
          f'planar approximation, floor_z={args.floor_z}')


if __name__ == '__main__':
    main()

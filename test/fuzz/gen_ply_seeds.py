#!/usr/bin/env python3
"""Structured seed generator for ply_mesh_fuzzer.

Builds structurally-valid PLY meshes from scratch that exercise paths the
repository's cube.ply does not cover: binary_little_endian and
binary_big_endian storage, mesh faces via both vertex_indices and
vertex_index list properties, mixed-width typed properties (char/short/
int/float/double), color and confidence fields, camera elements, and
range_grid list elements. Run from test/fuzz/build.sh at OSS-Fuzz build
time and zipped into the fuzzer's seed corpus.
"""
import os
import struct
import sys

SEED_ASCII_MESH = """\
comment structured seed: ascii mesh with faces
ply
format ascii 1.0
element vertex 4
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
element face 2
property list uchar int vertex_indices
end_header
0 0 0 255 0 0
1 0 0 0 255 0
1 1 0 0 0 255
0 1 0 255 255 0
3 0 1 2
3 0 2 3
"""

SEED_ASCII_VERTEX_INDEX = """\
ply
format ascii 1.0
element vertex 3
property float x
property float y
property float z
element face 1
property list uchar int vertex_index
end_header
0 0 0
1 0 0
0.5 1 0
3 0 1 2
"""

SEED_ASCII_TYPED = """\
ply
format ascii 1.0
element vertex 2
property char a
property short b
property int c
property float d
property double e
property uchar f
element face 1
property list uchar short vertex_indices
end_header
1 2 3 4.5 5.5 6
-1 -2 -3 -4.5 -5.5 7
3 0 1 2
"""

SEED_ASCII_CAMERA_RANGE_GRID = """\
ply
format ascii 1.0
element vertex 2
property float x
property float y
property float z
element camera 1
property float view_px
property float view_py
property float view_pz
property float x_axisx
property float x_axisy
property float x_axisz
property float y_axisx
property float y_axisy
property float y_axisz
property float z_axisx
property float z_axisy
property float z_axisz
element range_grid 2
property list uchar int vertex_indices
end_header
0 0 0
1 1 0
0 0 0 1 0 0 0 1 0 0 0 1
0 0 0 1 0 0 0 1 0 0 0 1
1 0
1 1
"""

SEED_ASCII_TEXCOORD = """\
ply
format ascii 1.0
element vertex 4
property float x
property float y
property float z
property float s
property float t
element face 2
property list uchar int vertex_indices
property list uchar float texcoord
end_header
0 0 0 0 0
1 0 0 1 0
1 1 0 1 1
0 1 0 0 1
3 0 1 2 4 0 0 1 0 1 1
3 0 2 3 4 0 0 1 0 1 1
"""


def binary_mesh(little: bool) -> bytes:
    endian = "<" if little else ">"
    header = (
        "ply\n"
        "format {} 1.0\n"
        "element vertex 3\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "element face 1\n"
        "property list uchar int vertex_indices\n"
        "end_header\n"
    ).format("binary_little_endian" if little else "binary_big_endian")
    body = struct.pack(endian + "9f", 0, 0, 0, 1, 0, 0, 0.5, 1, 0)
    body += struct.pack(endian + "B3i", 3, 0, 1, 2)
    return header.encode() + body


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "ply_seed_corpus"
    os.makedirs(out_dir, exist_ok=True)
    seeds = {
        "ascii_mesh.ply": SEED_ASCII_MESH,
        "ascii_vertex_index.ply": SEED_ASCII_VERTEX_INDEX,
        "ascii_typed_properties.ply": SEED_ASCII_TYPED,
        "ascii_camera_range_grid.ply": SEED_ASCII_CAMERA_RANGE_GRID,
        "ascii_texcoord_list.ply": SEED_ASCII_TEXCOORD,
        "binary_little_endian_mesh.ply": binary_mesh(True),
        "binary_big_endian_mesh.ply": binary_mesh(False),
    }
    for name, text in seeds.items():
        mode = "wb" if isinstance(text, bytes) else "w"
        with open(os.path.join(out_dir, name), mode) as f:
            f.write(text)
    print(f"wrote {len(seeds)} seeds to {out_dir}")


if __name__ == "__main__":
    main()

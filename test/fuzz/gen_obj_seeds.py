#!/usr/bin/env python3
"""Structured seed generator for obj_reader_fuzzer.

Builds structurally-valid OBJ files from scratch covering the parser paths
in pcl::OBJReader: plain vertex/face meshes, faces with v/vt/vn index
triples, group/object/smoothing state, material references (usemtl/mtllib),
comments and blank-line noise. Run from test/fuzz/build.sh at OSS-Fuzz build
time and zipped into the fuzzer's seed corpus.
"""
import os
import sys

SEEDS = {
    "plain_mesh.obj": """\
# simple triangle mesh
v 0.0 0.0 0.0
v 1.0 0.0 0.0
v 0.5 1.0 0.0
v 0.5 0.5 1.0
f 1 2 3
f 1 2 4
""",
    "indexed_normals.obj": """\
v 0 0 0
v 1 0 0
v 0 1 0
vn 0 0 1
vt 0 0
vt 1 0
vt 0 1
f 1/1/1 2/2/1 3/3/1
""",
    "groups_materials.obj": """\
mtllib seed.mtl
o mesh_a
g red_part
v 0 0 0
v 1 0 0
v 0 1 0
usemtl red
f 1 2 3
g blue_part
v 0 0 1
v 1 0 1
v 0 1 1
usemtl blue
s off
f 4 5 6
""",
    "mixed_faces.obj": """\
v 0 0 0
v 1 0 0
v 1 1 0
v 0 1 0
f 1 2 3 4
f -4 -3 -2 -1
f 2//2 3//3 1//1
""",
}


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "obj_seed_corpus"
    os.makedirs(out_dir, exist_ok=True)
    for name, text in SEEDS.items():
        with open(os.path.join(out_dir, name), "w") as f:
            f.write(text)
    print(f"wrote {len(SEEDS)} seeds to {out_dir}")


if __name__ == "__main__":
    main()

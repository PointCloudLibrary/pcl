#!/usr/bin/env python3
"""Structured seed generator for pcd_reader_fuzzer.

Builds tiny, structurally-valid ASCII PCD files from scratch that exercise
header shapes the repository's binary test clouds do not cover: PCD v0.6
(no VIEWPOINT), multi-component COUNT fields, U/I field types with 1/2/8
byte sizes, and organized (HEIGHT > 1) layouts. Appended to the seed corpus
so the mutator starts from valid structures instead of having to synthesize
the keyword skeleton.
"""
import os
import sys

SEEDS = {
    "v06_ascii_xyz": """\
# .PCD v0.6 - Point Cloud Data file format
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 3
HEIGHT 1
POINTS 3
DATA ascii
0 0 0
1 0 0
0 1 0
""",
    "v07_multi_count": """\
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z normal_x normal_y normal_z curvature
SIZE 4 4 4 4 4 4 4
TYPE F F F F F F F
COUNT 1 1 1 1 1 1 2
WIDTH 2
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 2
DATA ascii
0 0 0 0 0 1 1 2
1 1 1 0 1 0 3 4
""",
    "v07_uint8_fields": """\
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x label
SIZE 4 1
TYPE F U
COUNT 1 1
WIDTH 3
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 3
DATA ascii
0 0 7
1.5 0 9
2.5 0 11
""",
    "v07_organized": """\
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 2
HEIGHT 2
VIEWPOINT 0 0 0 1 0 0 0
POINTS 4
DATA ascii
0 0 0
1 0 0
0 1 0
1 1 0
""",
    "v07_int16_double": """\
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS a b
SIZE 2 8
TYPE I F
COUNT 1 1
WIDTH 2
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 2
DATA ascii
1 2.5
-2 -3.5
""",
}


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "pcd_seed_corpus"
    os.makedirs(out_dir, exist_ok=True)
    for name, text in SEEDS.items():
        with open(os.path.join(out_dir, name + ".pcd"), "w") as f:
            f.write(text)
    print(f"wrote {len(SEEDS)} seeds to {out_dir}")


if __name__ == "__main__":
    main()

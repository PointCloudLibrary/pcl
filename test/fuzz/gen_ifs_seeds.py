#!/usr/bin/env python3
"""Structured seed generator for ifs_reader_fuzzer.

Builds valid binary IFS files from scratch, mirroring the layout
pcl::IFSWriter produces and pcl::IFSReader consumes: length-prefixed
(including trailing null) keyword strings, a float format version, a
length-prefixed cloud name, a VERTICES block (u32 count + xyz float32
triples) and, for meshes, a TRIANGLES block (u32 facet count + u32 index
triples). Run from test/fuzz/build.sh at OSS-Fuzz build time and zipped
into the fuzzer's seed corpus.
"""
import os
import struct
import sys


def lp(s: str) -> bytes:
    b = s.encode() + b"\0"
    return struct.pack("<I", len(b)) + b


def vertices(n: int) -> bytes:
    out = struct.pack("<I", n)
    for i in range(n):
        out += struct.pack("<fff", i * 0.5, 0.25 * i, -float(i))
    return out


def triangles(facets) -> bytes:
    out = lp("TRIANGLES") + struct.pack("<I", len(facets))
    for a, b, c in facets:
        out += struct.pack("<III", a, b, c)
    return out


def header(name="cloud") -> bytes:
    return lp("IFS") + struct.pack("<f", 1.0) + lp(name) + lp("VERTICES")


SEEDS = {
    "cloud_only.ifs": header() + vertices(4),
    "mesh.ifs": header("mesh") + vertices(4)
    + triangles([(0, 1, 2), (0, 2, 3)]),
    "mesh_multi.ifs": header("multi") + vertices(6)
    + triangles([(0, 1, 2), (3, 4, 5), (0, 2, 4)]),
    "minimal.ifs": header("m") + vertices(1) + triangles([(0, 0, 0)]),
}


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "ifs_seed_corpus"
    os.makedirs(out_dir, exist_ok=True)
    for name, blob in SEEDS.items():
        with open(os.path.join(out_dir, name), "wb") as f:
            f.write(blob)
    print(f"wrote {len(SEEDS)} seeds to {out_dir}")


if __name__ == "__main__":
    main()

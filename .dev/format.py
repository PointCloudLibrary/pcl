#!/usr/bin/env python3

import os
import sys
import subprocess
import argparse

WHITELIST = [
    "apps/3d_rec_framework",
    "apps/in_hand_scanner",
    "apps/include",
    "apps/modeler",
    "apps/src",
    "benchmarks",
    "2d",
    "geometry",
    "ml",
    "octree",
    "simulation",
    "stereo",
    "tracking",
    "registration",
    "gpu/containers",
    "gpu/segmentation"
]

EXTENSIONS = (".c", ".h", ".cpp", ".hpp", ".cxx", ".hxx", ".cu")

def is_in_whitelist(file_path):
    file_path = os.path.normpath(file_path)
    for w in WHITELIST:
        w_norm = os.path.normpath(w)
        if os.path.commonpath([file_path, w_norm]) == w_norm or file_path.startswith(w_norm + os.sep):
            return True
    return False

def find_files(path):
    if os.path.isfile(path):
        if path.endswith(EXTENSIONS):
            yield path
    elif os.path.isdir(path):
        for root, _, files in os.walk(path):
            for file in files:
                if file.endswith(EXTENSIONS):
                    yield os.path.join(root, file)

def main():
    parser = argparse.ArgumentParser(
        description="Format C/C++/CUDA files using clang-format via pipx."
    )
    parser.add_argument(
        "files",
        nargs="+",
        help="List of files to check/format, or a single '.' to check all whitelisted files."
    )
    args = parser.parse_args()

    manual_mode = len(args.files) == 1 and args.files[0] == "."

    if manual_mode:
        all_files = []
        for rel_path in WHITELIST:
            abs_path = os.path.join(os.getcwd(), rel_path)
            all_files.extend(find_files(abs_path))
    else:
        all_files = [
            f for f in args.files
            if f.endswith(EXTENSIONS) and os.path.isfile(f) and is_in_whitelist(f)
        ]

    all_files = list(set(all_files))  # Remove duplicates

    if not all_files:
        print("No files found to format.")
        sys.exit(0)

    check_cmd = ["pipx", "run", "clang-format==14.0.3", "--dry-run", "--Werror", "--style=file"] + all_files
    result = subprocess.run(check_cmd, capture_output=True, text=True)
    if result.returncode != 0:
        format_cmd = ["pipx", "run", "clang-format==14.0.3", "-i", "--style=file"] + all_files
        subprocess.run(format_cmd)
        if not manual_mode:
            sys.exit(1)  # Only fail for pre-commit
        # In manual mode, just continue and exit 0

    sys.exit(0)

if __name__ == "__main__":
    main()

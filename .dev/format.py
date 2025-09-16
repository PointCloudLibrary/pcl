#!/usr/bin/env python3

import os
import sys
import subprocess
import argparse
import tempfile
import shutil
import re
from typing import Iterable, List

EXTENSIONS = (".c", ".h", ".cpp", ".hpp", ".cxx", ".hxx", ".cu")
WHITELIST_FILE = os.path.join(".dev", "whitelist.txt")

def load_whitelist():
    if not os.path.isfile(WHITELIST_FILE):
        print(f"Could not find whitelist file at {WHITELIST_FILE}")
        sys.exit(167)
    with open(WHITELIST_FILE, "r") as f:
        return [line.strip() for line in f if line.strip()]

def is_in_whitelist(file_path, whitelist):
    file_path = os.path.normpath(file_path)
    for w in whitelist:
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

def run_clang(cmd_args: List[str]) -> subprocess.CompletedProcess:
        try:
            return subprocess.run(
                cmd_args,
                capture_output=True,
                text=True,
                encoding="utf-8"
            )
        except FileNotFoundError as e:
            # Provide actionable feedback if the executable isn't found.
            print("Error: failed to launch clang-format.")
            print(
                "'pipx' may not be available on PATH.\n"
                "Install one of the following and try again:\n"
                " - pipx (https://pypa.github.io/pipx/)\n"
            )
            raise

def write_response_file(files: Iterable[str]) -> str:
        """Write file list to a clang/LLVM-style response file and return its path.

        Each path is written on its own line; paths with spaces are quoted.
        """
        fd, path = tempfile.mkstemp(prefix="clang_format_", suffix=".rsp")
        try:
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as tmp:
                for p in files:
                    if any(ch.isspace() for ch in p):
                        tmp.write(f'"{p}"\n')
                    else:
                        tmp.write(p + "\n")
        except Exception:
            # Ensure file descriptor gets closed on error
            try:
                os.close(fd)
            except Exception:
                pass
            raise
        return path

def parse_violations(text: str) -> List[str]:
            files: List[str] = []
            if not text:
                return files
            # Patterns seen across clang-format versions
            patterns = [
                re.compile(r"^(?P<file>.+?):\d+:\d+:\s+(?:error|warning):.*?(?:code should be clang-formatted|would be reformatted|not formatted)", re.IGNORECASE),
                re.compile(r"^(?:error|warning):\s.*?would be reformatted:\s(?P<file>.+)$", re.IGNORECASE),
            ]
            for line in text.splitlines():
                line = line.strip()
                for pat in patterns:
                    m = pat.match(line)
                    if m:
                        f = m.group("file").strip()
                        files.append(f)
                        break
            # Deduplicate preserving order
            seen = set()
            uniq: List[str] = []
            for f in files:
                if f not in seen:
                    seen.add(f)
                    uniq.append(f)
            return uniq

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

    whitelist = load_whitelist()
    manual_mode = len(args.files) == 1 and args.files[0] == "."

    if manual_mode:
        all_files = []
        for rel_path in whitelist:
            abs_path = os.path.join(os.getcwd(), rel_path)
            all_files.extend(find_files(abs_path))
    else:
        all_files = [
            f for f in args.files
            if f.endswith(EXTENSIONS) and os.path.isfile(f) and is_in_whitelist(f, whitelist)
        ]

    all_files = list(set(all_files))  # Remove duplicates

    # Print number of files required formatting
    print(f"Found {len(all_files)} files to format/check.")

    if not all_files:
        print("No files found to format.")
        sys.exit(0)

    rsp_path = None
    rsp_path_fix = None
    try:
        # Prefer response file to avoid command length issues on Windows.
        rsp_path = write_response_file(all_files)
        pipx_path = shutil.which("pipx")
        check_cmd = [pipx_path, "run", "clang-format==14.0.3", "--dry-run", "--Werror", "--style=file", f"@{rsp_path}"]
        result = run_clang(check_cmd)

        offenders = parse_violations(result.stderr) or parse_violations(result.stdout)

        if offenders:
            print(f"Formatting required for {len(offenders)} files.")
            # Format only the offending files for speed
            rsp_path_fix = write_response_file(offenders)
            pipx_path = shutil.which("pipx")
            format_cmd = [pipx_path, "run", "clang-format==14.0.3", "-i", "--style=file", f"@{rsp_path_fix}"]
            fmt_result = run_clang(format_cmd)
            # If pre-commit mode, fail to indicate changes were needed
            if not manual_mode:
                sys.stdout.write(result.stdout)
                sys.stderr.write(result.stderr)
                sys.stdout.write(fmt_result.stdout)
                sys.stderr.write(fmt_result.stderr)
                sys.exit(1)
            # In manual mode, continue and exit 0
        else:
            # No formatting violations parsed. If clang-format errored for other reasons, propagate.
            if result.returncode != 0:
                sys.stdout.write(result.stdout)
                sys.stderr.write(result.stderr)
                sys.exit(result.returncode)
    finally:
        if rsp_path and os.path.exists(rsp_path):
            try:
                os.remove(rsp_path)
            except OSError:
                pass
        if rsp_path_fix and os.path.exists(rsp_path_fix):
            try:
                os.remove(rsp_path_fix)
            except OSError:
                pass

    sys.exit(0)

if __name__ == "__main__":
    main()

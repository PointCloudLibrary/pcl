import os
import json
import argparse


def get_realpath(path):
    return os.path.realpath(path)


def ensure_dir_exists(dir):
    if not os.path.exists(dir):
        os.makedirs(dir)


def get_parent_directory(file):
    return os.path.dirname(os.path.dirname(file))


def join_path(*args):
    return os.path.join(*args)


def get_json_output_path(source, output_dir):
    delimiter = "/" if "/" in source else "\\"
    x_list = source.split(f"pcl{delimiter}", 1)[-1]
    x_list = x_list.split(delimiter)
    extra = ["pcl", "include"]

    filename = x_list[-1].split(".")[0]
    relative_dir = delimiter.join(x for x in x_list[:-1] if x not in extra)
    dir = join_path(output_dir, relative_dir)

    ensure_dir_exists(dir)

    return get_realpath(join_path(dir, f"{filename}.json"))


def dump_json(filepath, info):
    with open(filepath, "w") as f:
        json.dump(info, f, indent=2)


def parse_arguments(script):
    if script == "parse":
        parser = argparse.ArgumentParser(description="C++ libclang parser")
        parser.add_argument(
            "--compilation_database_path",
            default=get_parent_directory(file=__file__),
            help="Path to compilation database (json)",
        )
        parser.add_argument(
            "--json_output_path",
            default=get_parent_directory(file=__file__),
            help="Output path for generated json",
        )
        parser.add_argument("files", nargs="+", help="The source files to parse")
        return parser.parse_args()

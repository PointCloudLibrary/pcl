import os
import json
import argparse


def get_realpath(path):
    return os.path.realpath(path)


def ensure_dir_exists(dir):
    if not os.path.exists(dir):
        os.makedirs(dir)


def get_json_output_path(source, output_dir):
    x_list = source.split("pcl/", 1)[-1]
    x_list = x_list.split("/")
    extra = ["pcl", "include"]

    filename = x_list[-1].split(".")[0]
    relative_dir = "/".join(x for x in x_list[:-1] if x not in extra)
    dir = os.path.join(output_dir, relative_dir)

    ensure_dir_exists(dir)

    return get_realpath(f"{dir}/{filename}.json")


def dump_json(filepath, info):
    with open(filepath, "w") as f:
        json.dump(info, f, indent=2)


def parse_arguments(args, script):
    if script == "parse":
        parser = argparse.ArgumentParser(description="C++ libclang parser")
        parser.add_argument("files", nargs="+", help="The source files to parse")
        return parser.parse_args(args)


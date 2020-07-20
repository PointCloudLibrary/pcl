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


def get_output_path(source, output_dir, split_from, extension):
    """
    Returns json output path after manipulation of the source file's path

    Arguments:
        - source: The source's file name
        - output_dir: The output directory to write the json output
        - split_from: The folder from which to split the path
        - extension: Output extension
    
    Returns:
        - output_path: The output's realpath
    """

    # split_path: contains the path after splitting. For split_path = pcl, contains the path as seen in the pcl directory
    _, split_path = source.split(f"{split_from}{os.sep}", 1)

    # relative_dir: contains the relative output path for the json file
    # source_filename: contains the source's file name
    relative_dir, source_filename = os.path.split(split_path)

    # filename: contains the output json's file name
    filename, _ = source_filename.split(".")
    filename = f"{filename}{extension}"

    # dir: final output path
    dir = join_path(output_dir, relative_dir)

    # make the output directory if it doesn't exist
    ensure_dir_exists(dir)

    output_path = get_realpath(join_path(dir, filename))

    return output_path


def dump_json(filepath, info):
    with open(filepath, "w") as f:
        json.dump(info, f, indent=2)


def read_json(filename):
    with open(filename, "r") as f:
        return json.load(f)


def write_to_file(filename, linelist):
    with open(filename, "w") as f:
        for line in linelist:
            f.writelines(line)
            f.writelines("\n")


def parse_arguments(script):
    """
    Returns parsed command line arguments for a given script

    Arguments:
        - script: The python script for which the custom command line arguments should be parsed

    Return:
        - args: Parsed command line arguments
    """

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

    if script == "generate":
        parser = argparse.ArgumentParser(description="JSON to pybind11 generation")
        parser.add_argument("files", nargs="+", help="JSON input")
        parser.add_argument(
            "--pybind11_output_path",
            default=get_parent_directory(file=__file__),
            help="Output path for generated cpp",
        )

    else:
        args = None

    args = parser.parse_args()
    return args

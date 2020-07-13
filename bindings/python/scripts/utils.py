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
    """
    Returns json output path after manipulation of the source file's path

    Arguments:
        - source: The source's file name
        - output_dir: The output_directory to write the json output
    
    Returns:
        - json_output_path: The json output's realpath
    """

    # pcl_path: contains the path as seen in the pcl directory
    _, pcl_path = source.split(f"pcl{os.sep}", 1)

    # relative_dir: contains the relative output path for the json file
    # source_filename: contains the source's file name
    relative_dir, source_filename = os.path.split(pcl_path)

    # filename: contains the output json's file name
    filename, _ = source_filename.split(".")
    filename = f"{filename}.json"

    # dir: final output path
    dir = join_path(output_dir, relative_dir)

    # make the output directory if it doesn't exist
    ensure_dir_exists(dir)

    json_output_path = get_realpath(join_path(dir, filename))

    return json_output_path


def dump_json(filepath, info):
    with open(filepath, "w") as f:
        json.dump(info, f, indent=2)


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
        args = parser.parse_args()

        return args

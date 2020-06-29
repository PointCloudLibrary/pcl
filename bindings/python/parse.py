import os
import argparse
import sys
import json

import clang.cindex as clang


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def print_node(cursor, lines, depth):
    file = cursor.location.file
    line, column = cursor.location.line, cursor.location.column
    result_type = (
        cursor.result_type.spelling if cursor.result_type.spelling is not "" else "None"
    )
    print(
        "-" * depth,
        file,
        f"L{line} C{column}",
        bcolors.BOLD + cursor.kind.name + bcolors.ENDC,
        bcolors.OKBLUE + cursor.spelling + bcolors.ENDC,
        bcolors.OKGREEN + cursor.access_specifier.name + bcolors.ENDC,
        bcolors.OKGREEN + "->" + bcolors.ENDC,
        bcolors.OKGREEN + result_type + bcolors.ENDC,
    )


def node_in_this_file(node, file_name):
    return node.location.file and node.location.file.name == file_name


def walk_and_print(cursor, filter, lines, this_filename, depth):
    if cursor.spelling:
        print_node(cursor, lines, depth)

    for child in cursor.get_children():
        if node_in_this_file(child, this_filename):
            walk_and_print(child, filter, lines, this_filename, depth + 1)


def dump_json(filepath, parsed_list):
    with open(filepath, "w") as f:
        json.dump(parsed_list, f, indent=2)


def generate_parsed_info(cursor, filter, lines, this_filename, depth, parsed_list):
    if cursor.spelling:
        holder = {
            "depth": depth,
            "line": cursor.location.line,
            "column": cursor.location.column,
            "kind": cursor.kind.name,
            "name": cursor.spelling,
        }
        if cursor.type.kind.spelling != "Invalid":
            holder["element_type"] = cursor.type.kind.spelling
        if cursor.access_specifier.name != "INVALID":
            holder["access_specifier"] = cursor.access_specifier.name
        if cursor.result_type.spelling != "":
            holder["result_type"] = cursor.result_type.spelling
        if cursor.brief_comment:
            holder["brief_comment"] = cursor.brief_comment
        if cursor.raw_comment:
            holder["raw_comment"] = cursor.raw_comment
        holder["members"] = []

        parsed_list.append(holder)

    for child in cursor.get_children():
        if node_in_this_file(child, this_filename):
            child_list = []
            generate_parsed_info(
                child, filter, lines, this_filename, depth + 1, child_list,
            )
            if child_list and parsed_list:
                if len(parsed_list[0]["members"]):
                    parsed_list[0]["members"].append(child_list[0])
                else:
                    parsed_list[0]["members"] = child_list


def parse_arguments(args):
    parser = argparse.ArgumentParser(description="C++ libclang parser")
    parser.add_argument("files", nargs="+", help="The source files to search")
    return parser.parse_args(args)


def get_output_path(source, output_dir):
    x_list = source.split("pcl/", 1)[-1]
    x_list = x_list.split("/")
    extra = ["pcl", "include"]

    filename = x_list[-1].split(".")[0]
    relative_dir = "/".join(x for x in x_list[:-1] if x not in extra)
    dir = os.path.join(output_dir, relative_dir)

    # ensure the new directory exists
    if not os.path.exists(dir):
        os.makedirs(dir)

    return f"{dir}/{filename}.json"


def main():
    args = parse_arguments(sys.argv[1:])
    index = clang.Index.create()

    compile_db_path = os.path.dirname("./compile_commands.json")
    compdb = clang.CompilationDatabase.fromDirectory(compile_db_path)

    for source in args.files:
        with open(source) as input_file:
            lines = input_file.readlines()

        compile_commands = compdb.getCompileCommands(source)
        # extracting argument list from the command's generator object
        compile_commands = list(compile_commands[0].arguments)[1:-1]
        tu = index.parse(source, args=compile_commands)

        # walk_and_print(
        #     tu.cursor, filter, lines, tu.spelling, depth=0
        # )

        parsed_list = []
        generate_parsed_info(
            tu.cursor, filter, lines, tu.spelling, depth=0, parsed_list=parsed_list,
        )

        output_filepath = get_output_path(
            os.path.realpath(source), output_dir=f"json/{os.path.dirname(__file__)}"
        )
        dump_json(output_filepath, parsed_list)


if __name__ == "__main__":
    main()

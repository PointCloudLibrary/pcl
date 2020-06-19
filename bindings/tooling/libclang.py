import os
import argparse
import typing
import sys

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


def print_node(cursor, lines, more_than_one_file, depth):
    file = "{0}:".format(cursor.location.file) if more_than_one_file else ""
    line, column = cursor.location.line, cursor.location.column
    print(
        "-" * depth,
        file,
        f"L{line} C{column}",
        bcolors.BOLD + cursor.kind.name + bcolors.ENDC,
        bcolors.OKBLUE + cursor.spelling + bcolors.ENDC,
        bcolors.OKGREEN + cursor.displayname + bcolors.ENDC,
    )


def node_in_this_file(node, file_name):
    if node.location.file and node.location.file.name == file_name:
        return True
    return False


def walk(cursor, filter, lines, more_than_one_file, this_filename, depth):
    if cursor.spelling:
        print_node(cursor, lines, more_than_one_file, depth)

    for child in cursor.get_children():
        if node_in_this_file(child, this_filename):
            walk(child, filter, lines, more_than_one_file, this_filename, depth + 1)



#     if args.variable:
#         allowed = [clang.CursorKind.FUNCTION_DECL]
#         if args.member:
#             allowed.append(clang.CursorKind.CXX_METHOD)
#         filter.by_kind(allowed)

#     if args.struct:
#         filter.by_kind(clang.CursorKind.STRUCT_DECL)

#     if args.parameter:
#         filter.by_kind(clang.CursorKind.PARM_DECL)

#     return filter


def parse_arguments(args):
    parser = argparse.ArgumentParser(description="C++-savy grep")
    parser.add_argument("files", nargs="+", help="The source files to search")
    return parser.parse_args(args)


def main():
    args = parse_arguments(sys.argv[1:])
    index = clang.Index.create()
    more_than_one_file = len(args.files) > 1

    # compile_db_path = os.path.dirname("./compile_commands.json")
    # compdb = clang.CompilationDatabase.fromDirectory(compile_db_path)

    for source in args.files:
        with open(source) as input_file:
            lines = input_file.readlines()

        # compile_commands = compdb.getCompileCommands(source)
        # # extracting argument list from the command's generator object
        # compile_commands = list(compile_commands[0].arguments)[1:-2]
        # tu = index.parse(source, args=compile_commands)
        tu = index.parse(source)


if __name__ == "__main__":
    main()

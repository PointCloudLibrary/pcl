import argparse
import re
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


class Filter(object):
    def __init__(self, pattern):
        self.pattern = re.compile(pattern)
        self.predicates = []

    def matches(self, cursor):
        if self.pattern.fullmatch(cursor.displayname) is None:
            return False
        if not self.predicates:
            return True
        return any(predicate(cursor) for predicate in self.predicates)

    def by_kind(self, allowed):
        self.predicates.append(lambda cursor: cursor.kind in allowed)


def print_match(cursor, lines, more_than_one_file):
    file = "{0}:".format(cursor.location.file) if more_than_one_file else ""
    line, column = cursor.location.line, cursor.location.column
    # pretty = '\033[1;91m{0}\033[0m'.format(cursor.spelling)
    # before = lines[line - 1][:column - 1]
    # after = lines[line - 1][column + len(cursor.spelling) - 1:]
    # text = before + pretty + after
    print(
        file,
        line,
        column,
        bcolors.OKBLUE + cursor.spelling + bcolors.ENDC,
        bcolors.OKGREEN + cursor.displayname + bcolors.ENDC,
    )

    # diagnostic = '\033[1m{0}{1}:{2}:\033[0m {3}'
    # print(diagnostic.format(file, line, column, text.rstrip()))


def walk(cursor, filter, lines, more_than_one_file):
    if filter.matches(cursor):
        print_match(cursor, lines, more_than_one_file)

    for child in cursor.get_children():
        walk(child, filter, lines, more_than_one_file)


def make_filter(args):
    filter = Filter(args.pattern)

    if args.function:
        allowed = [clang.CursorKind.FUNCTION_DECL]
        if args.member:
            allowed.append(clang.CursorKind.CXX_METHOD)
        filter.by_kind(allowed)

    if args.variable:
        allowed = [clang.CursorKind.FUNCTION_DECL]
        if args.member:
            allowed.append(clang.CursorKind.CXX_METHOD)
        filter.by_kind(allowed)

    if args.struct:
        filter.by_kind(clang.CursorKind.STRUCT_DECL)

    if args.parameter:
        filter.by_kind(clang.CursorKind.PARM_DECL)

    return filter


def parse_arguments(args):
    parser = argparse.ArgumentParser(description="C++-savy grep")
    parser.add_argument("pattern", help="The regular expression pattern")
    parser.add_argument("files", nargs="+", help="The source files to search")
    parser.add_argument(
        "-f",
        "--function",
        action="store_true",
        help="Whether to include functions in the search",
    )
    parser.add_argument(
        "-v",
        "--variable",
        action="store_true",
        help="Whether to include variables in the search",
    )
    parser.add_argument(
        "-p",
        "--parameter",
        action="store_true",
        help="Whether to include parameters in the search",
    )
    parser.add_argument(
        "-s",
        "--struct",
        action="store_true",
        help="Whether to include parameters in the search",
    )
    parser.add_argument(
        "-m",
        "--member",
        action="store_true",
        help="Whether to include members in the search",
    )

    return parser.parse_args(args)


def main():
    args = parse_arguments(sys.argv[1:])
    filter = make_filter(args)
    index = clang.Index.create()
    more_than_one_file = len(args.files) > 1

    for source in args.files:
        with open(source) as input_file:
            lines = input_file.readlines()

        tu = index.parse(source)
        walk(tu.cursor, filter, lines, more_than_one_file)


if __name__ == "__main__":
    main()

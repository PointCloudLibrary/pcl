import sys
import clang.cindex as clang

import utils


def print_node(cursor, depth):
    print("-" * depth, cursor.location.file, f"L{cursor.location.line} C{cursor.location.column}", cursor.kind.name, cursor.spelling)


def is_node_in_this_file(node, filename):
    return node.location.file and node.location.file.name == filename


def walk_and_print(cursor, this_filename, depth):
    if cursor.spelling:
        print_node(cursor=cursor, depth=depth)

    for child in cursor.get_children():
        if is_node_in_this_file(node=child, filename=this_filename):
            walk_and_print(cursor=child, this_filename=this_filename, depth=depth + 1)


def generate_parsed_info(cursor, this_filename, depth):
    parsed_info = dict()

    if cursor.spelling:
        parsed_info["depth"] = depth
        parsed_info["line"] = cursor.location.line
        parsed_info["column"] = cursor.location.column
        parsed_info["kind"] = cursor.kind.name
        parsed_info["name"] = cursor.spelling
        if cursor.type.kind.spelling != "Invalid":
            parsed_info["element_type"] = cursor.type.kind.spelling
        if cursor.access_specifier.name != "INVALID":
            parsed_info["access_specifier"] = cursor.access_specifier.name
        if cursor.result_type.spelling != "":
            parsed_info["result_type"] = cursor.result_type.spelling
        if cursor.brief_comment:
            parsed_info["brief_comment"] = cursor.brief_comment
        if cursor.raw_comment:
            parsed_info["raw_comment"] = cursor.raw_comment
        parsed_info["members"] = []

    for child in cursor.get_children():
        if is_node_in_this_file(node=child, filename=this_filename):
            child_parsed_info = generate_parsed_info(cursor=child, this_filename=this_filename, depth=depth + 1,)
            if child_parsed_info and parsed_info:
                parsed_info["members"].append(child_parsed_info)

    return parsed_info


def create_index():
    return clang.Index.create()


def get_compilation_commands(compilation_database_path, filename):
    compilation_database = clang.CompilationDatabase.fromDirectory(buildDir=compilation_database_path)
    compilation_commands = compilation_database.getCompileCommands(filename=filename)
    # extracting argument list from the command's generator object
    return list(compilation_commands[0].arguments)[1:-1]


def main():
    args = utils.parse_arguments(args=sys.argv[1:], script=__file__)
    for source in args.files:
        source = utils.get_realpath(path=source)

        index = create_index()

        compilation_commands = get_compilation_commands(
            compilation_database_path="/home/divyanshu/Projects/active/pcl/bindings/python", filename=source
        )

        tu = index.parse(path=source, args=compilation_commands)

        # walk_and_print(cursor=tu.cursor, this_filename=tu.spelling, depth=0)

        parsed_info = generate_parsed_info(cursor=tu.cursor, this_filename=tu.spelling, depth=0)

        output_filepath = utils.get_json_output_path(source=source, output_dir="/home/divyanshu/Projects/active/pcl/bindings/python/json",)
        utils.dump_json(filepath=output_filepath, info=parsed_info)


if __name__ == "__main__":
    main()

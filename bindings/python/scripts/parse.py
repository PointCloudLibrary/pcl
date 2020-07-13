import sys
import clang.cindex as clang

import utils


def is_node_in_this_file(cursor, filename, depth):
    """
    Checks if the node in the AST belongs to the file
    
    Arguments:
        - cursor : The cursor pointing to the node
        - filename : The file's name to check the node against
        - depth: The depth of the node (root=0)
        
    Returns:
        - True/False (bool)
    """

    if cursor.location.file and cursor.location.file.name == filename:
        return True
    else:
        return False


def print_ast(cursor, filename, depth):
    """
    Prints the AST by recursively traversing the AST

    Arguments:
        - cursor: The cursor pointing to a node
        - filename: The file's name to check if the node belongs to it
        - depth: The depth of the node (root=0)
    
    Returns:
        - None
    """

    if cursor.spelling:
        print(
            "-" * depth,
            cursor.location.file,
            f"L{cursor.location.line} C{cursor.location.column}",
            cursor.kind.name,
            cursor.spelling,
        )

    for child in cursor.get_children():
        child_node = {"cursor": child, "filename": filename, "depth": depth + 1}
        if is_node_in_this_file(**child_node):
            print_ast(**child_node)


def generate_parsed_info(cursor, filename, depth):
    """
    Generates parsed information by recursively traversing the AST

    Arguments:
        - cursor: The cursor pointing to a node
        - filename: The file's name to check if the node belongs to it
        - depth: The depth of the node (root=0)

    Returns:
        - parsed_info (dict): 
            - Contains key-value pairs of various traits of a node
            - The key 'members' contains the node's children's `parsed_info`
    """

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
        child_node = {"cursor": child, "filename": filename, "depth": depth + 1}
        if is_node_in_this_file(**child_node):
            child_parsed_info = generate_parsed_info(**child_node)
            if child_parsed_info and parsed_info:
                parsed_info["members"].append(child_parsed_info)

    return parsed_info


def get_compilation_commands(compilation_database_path, filename):
    """
    Returns the compilation commands extracted from the the compile command

    Arguments:
        - compilation_database_path: The path to `compile_commands.json`
        - filename: The file's name to get it's compilation commands

    Returns:
        - compilation commands (list): The arguments passed to the compiler
    """

    compilation_database = clang.CompilationDatabase.fromDirectory(
        buildDir=compilation_database_path
    )
    compilation_commands = compilation_database.getCompileCommands(filename=filename)
    # extracting argument list from the command's generator object
    return list(compilation_commands[0].arguments)[1:-1]


def parse_file(source, compilation_database_path=None):
    index = clang.Index.create()

    # Is this check needed?
    if compilation_database_path is None:
        compilation_commands = ["-std=c++14"]
    else:
        compilation_commands = get_compilation_commands(
            compilation_database_path=compilation_database_path, filename=source,
        )

    source_ast = index.parse(path=source, args=compilation_commands)

    root_node = {
        "cursor": source_ast.cursor,
        "filename": source_ast.spelling,
        "depth": 0,
    }

    # print_ast(**root_node)

    parsed_info = generate_parsed_info(**root_node)

    return parsed_info


def main():
    args = utils.parse_arguments(script="parse")
    for source in args.files:
        source = utils.get_realpath(path=source)

        parsed_info = parse_file(source, args.compilation_database_path)

        output_filepath = utils.get_json_output_path(
            source=source, output_dir=utils.join_path(args.json_output_path, "json"),
        )

        utils.dump_json(filepath=output_filepath, info=parsed_info)


if __name__ == "__main__":
    main()

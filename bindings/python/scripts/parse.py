import sys
import clang.cindex as clang

from context import scripts
import scripts.utils as utils


def is_node_in_this_file(node):
    """
    Checks if the node in the AST is a valid node:
        - Check 1: The cursor's location should have file attribute (cursor.location.file -> not NoneType)
        - Check 2: The cursor should belong to the file (cursor.location.file.name -> filename)
    
    Arguments:
        - node (dict):
            - The node in the AST
            - Keys:
                - cursor : The cursor pointing to the node
                - filename : The file's name to check the node against
                - depth: The depth of the node (root=0)
        
    Returns:
        - True/False (bool)
    """

    cursor = node["cursor"]
    filename = node["filename"]

    if cursor.location.file is not None:
        return cursor.location.file.name == filename
    return False


def valid_children(node):
    """
    A generator function for yielding valid children nodes

    Arguments:
        - node (dict):
            - The node in the AST
            - Keys:
                - cursor: The cursor pointing to a node
                - filename: 
                    - The file's name to check if the node belongs to it
                    - Needed to ensure that only symbols belonging to the file gets parsed, not the included files' symbols
                - depth: The depth of the node (root=0)

    Yields:
        - child_node (dict): Same structure as the argument
    """

    cursor = node["cursor"]
    filename = node["filename"]
    depth = node["depth"]

    for child in cursor.get_children():
        child_node = {"cursor": child, "filename": filename, "depth": depth + 1}
        # Check if the child belongs to the file
        if is_node_in_this_file(child_node):
            yield (child_node)


def print_ast(node):
    """
    Prints the AST by recursively traversing the AST

    Arguments:
        - node (dict):
            - The node in the AST
            - Keys:
                - cursor: The cursor pointing to a node
                - filename: 
                    - The file's name to check if the node belongs to it
                    - Needed to ensure that only symbols belonging to the file gets parsed, not the included files' symbols
                - depth: The depth of the node (root=0)
    
    Returns:
        - None
    """

    cursor = node["cursor"]
    depth = node["depth"]

    if cursor.spelling is not None:
        print(
            "-" * depth,
            cursor.location.file,
            f"L{cursor.location.line} C{cursor.location.column}",
            cursor.kind.name,
            cursor.spelling,
        )

    # Get cursor's children and recursively print
    for child_node in valid_children(node):
        print_ast(child_node)


def generate_parsed_info(node):
    """
    Generates parsed information by recursively traversing the AST

    Arguments:
        - node (dict):
            - The node in the AST
            - Keys:
                - cursor: The cursor pointing to a node
                - filename: 
                    - The file's name to check if the node belongs to it
                    - Needed to ensure that only symbols belonging to the file gets parsed, not the included files' symbols
                - depth: The depth of the node (root=0)

    Returns:
        - parsed_info (dict): 
            - Contains key-value pairs of various traits of a node
            - The key 'members' contains the node's children's `parsed_info`
    """

    parsed_info = dict()

    cursor = node["cursor"]
    depth = node["depth"]

    # @TODO: fix
    if cursor.spelling != "":
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

    # Get cursor's children and recursively add their info to a dictionary, as members of the parent
    for child_node in valid_children(node):
        child_parsed_info = generate_parsed_info(child_node)
        # If both child and parent's info is not empty (opening check for dictionary population), add child's info to parent's members
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

    # Build a compilation database found in the given directory
    compilation_database = clang.CompilationDatabase.fromDirectory(
        buildDir=compilation_database_path
    )

    # Get compiler arguments from the compilation database for the given file
    compilation_commands = compilation_database.getCompileCommands(filename=filename)

    """
    - compilation_commands:
        - An iterable object providing all the compilation commands available to build filename.
        - type: <class 'clang.cindex.CompileCommands'>
    - compilation_commands[0]:
        - Since we have only one command per filename in the compile_commands.json, extract 0th element
        - type: <class 'clang.cindex.CompileCommand'>
    - compilation_commands[0].arguments:
        - Get compiler arguments from the CompileCommand object
        - type: <class 'generator'>
    - list(compilation_commands[0].arguments)[1:-1]:
        - Convert the generator object to list, and extract compiler arguments
        - 0th element is the compiler name
        - nth element is the filename
    """

    # Extracting argument list from the command's generator object
    compilation_commands = list(compilation_commands[0].arguments)[1:-1]

    return compilation_commands


def parse_file(source, compilation_database_path=None):
    """
    Returns the parsed_info for a file

    Arguments:
        - source: Source to parse
        - compilation_database_path: The path to `compile_commands.json`

    Returns:
        - parsed_info 
    """

    # Create a new index to start parsing
    index = clang.Index.create()

    # Is this check needed?
    if compilation_database_path is None:
        # Default compiler argument
        compilation_commands = ["-std=c++14"]
    else:
        # Get compiler arguments
        compilation_commands = get_compilation_commands(
            compilation_database_path=compilation_database_path, filename=source,
        )

    # Parse the given source code file by running clang and generating the AST before loading
    source_ast = index.parse(path=source, args=compilation_commands)

    # Dictionary to hold a node's information
    root_node = {
        "cursor": source_ast.cursor,
        "filename": source_ast.spelling,
        "depth": 0,
    }

    # For testing purposes
    # print_ast(root_node)

    # Dictionary containing parsed information of the source file
    parsed_info = generate_parsed_info(root_node)

    return parsed_info


def main():
    # Get command line arguments
    args = utils.parse_arguments(script="parse")
    for source in args.files:
        source = utils.get_realpath(path=source)

        # Parse the source file
        parsed_info = parse_file(source, args.compilation_database_path)

        # Output path for dumping the parsed info into a json file
        output_filepath = utils.get_output_path(
            source=source,
            output_dir=utils.join_path(args.json_output_path, "json"),
            split_from="pcl",
            extension=".json",
        )

        # Dump the parsed info at output path
        utils.dump_json(filepath=output_filepath, info=parsed_info)


if __name__ == "__main__":
    main()

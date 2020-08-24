from context import scripts
import scripts.generate as generate
import scripts.utils as utils
import test_parse


def remove_whitespace(string):
    """
    Removes whitespace from the string.

    Parameters:
        - string (str)
    
    Returns: The stripped off string.
    """

    return "".join([x for x in string if not x.isspace()])


def generate_bindings(cpp_code_block, tmp_path):
    """
    Returns binded code for a cpp code block
    - Steps:
        1. Get parsed info for the cpp code block (via get_parsed_info in test_parse.py).
        2. Generate bindings for the parsed info (via generate in generate.py).
        3. Convert the list to a string and then return the stripped off string.

    Parameters:
        - tmp_path (pathlib.PosixPath): The tmp_path for test folder
        - cpp_code_block (str): The cpp code block to generate bindings for

    Returns:
        - binded_code (str): The generated binded code
    """

    # Get parsed info for the cpp code block
    parsed_info = test_parse.get_parsed_info(
        tmp_path=tmp_path, file_contents=cpp_code_block
    )

    # JSON dump the parsed info
    json_path = tmp_path / "file.json"
    utils.dump_json(filepath=json_path, info=parsed_info, indent=None, separators=(',', ':'))

    # Get the binded code
    binded_code = generate.generate(source=json_path)

    # List to string
    binded_code = "".join(binded_code)

    return remove_whitespace(binded_code)


def get_expected_string(expected_module_code):
    """
    Returns expected output string after combining inclusions and pybind11's initial lines.

    Parameters:
        - expected_module_code (str): Module code to be combined.

    Returns:
        - expected_output (str): The stripped off, combined code.
    """

    file_include = "#include <file.cpp>"

    # Get pybind11's intial lines in the form of a string
    initial_pybind_lines = "".join(generate.bind._initial_pybind_lines)

    expected_output = remove_whitespace(
        file_include + initial_pybind_lines + expected_module_code
    )

    return expected_output


def test_case1(tmp_path):
    cpp_code_block = "struct AStruct {};"
    output = generate_bindings(tmp_path=tmp_path, cpp_code_block=cpp_code_block)

    expected_module_code = """
    PYBIND11_MODULE(pcl, m){
        py::class_<AStruct>(m, "AStruct")
        .def(py::init<>());
    }
    """

    assert output == get_expected_string(expected_module_code=expected_module_code)


def test_case2(tmp_path):
    cpp_code_block = """
    struct AStruct {
        int aMember;
    };
    """
    output = generate_bindings(tmp_path=tmp_path, cpp_code_block=cpp_code_block)

    expected_module_code = """
    PYBIND11_MODULE(pcl, m){
        py::class_<AStruct>(m, "AStruct")
        .def(py::init<>())
        .def_readwrite("aMember", &AStruct::aMember);
    }
    """

    assert output == get_expected_string(expected_module_code=expected_module_code)

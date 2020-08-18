from context import scripts
import scripts.generate as generate
import scripts.utils as utils
import test_parse


# @TODO: make a list in generate.py and use here
initial_lines = """
#include <pcl/point_types.h>
#include <pybind11/pybind11.h>
#include<pybind11/stl.h>
#include<pybind11/stl_bind.h>
namespace py = pybind11;
using namespace py::literals;
"""


def get_character_only_string(word):
    """
    Returns extracted characters from a word by excluding whitespaces (' '), newlines ('\n') and tabs ('\t').

    Parameters:
        - word (str)
    
    Returns:
        - chars (str): The stripped off string
    """

    chars = []
    for c in word:
        if c not in (" ", "\n", "\t"):
            chars += c
    return "".join(chars)


def get_binded_code(tmp_path, cpp_code_block):
    """
    Returns binded code for a cpp code block
    - Steps:
        1. Get parsed info for the cpp code block (via get_parsed_info in test_parse.py).
        2. Generate bindings for the parsed info (via generate in generate.py).
        3. Convert the list to a string and then return the stripped off string.

    Parameters:
        - tmp_path (Path): The tmp_path for test folder
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
    utils.dump_json(filepath=json_path, info=parsed_info)

    # Get the binded code
    binded_code = generate.generate(source=json_path)

    # List to string
    binded_code = "".join(binded_code)

    return get_character_only_string(binded_code)


def test_case1(tmp_path):
    cpp_code_block = "struct AStruct {};"
    output = get_binded_code(tmp_path=tmp_path, cpp_code_block=cpp_code_block)

    expected_output = """
    PYBIND11_MODULE(pcl, m){
        py::class_<AStruct>(m, "AStruct");
    }
    """
    expected_output = get_character_only_string(initial_lines + expected_output)

    assert output == expected_output


def test_case2(tmp_path):
    cpp_code_block = """
    struct AStruct {
        int aMember;
    };
    """
    output = get_binded_code(tmp_path=tmp_path, cpp_code_block=cpp_code_block)

    expected_output = """
    PYBIND11_MODULE(pcl, m){
        py::class_<AStruct>(m, "AStruct")
        .def_readwrite("aMember", &AStruct::aMember);
    }
    """
    expected_output = get_character_only_string(initial_lines + expected_output)

    assert output == expected_output

from context import scripts
import scripts.parse as parse


def create_compilation_database(tmp_path, filepath):
    input = tmp_path / "compile_commands.json"
    x = [
        {
            "directory": f"{tmp_path}",
            "command": f"/usr/bin/clang++ -std=c++14 {filepath}",
            "file": f"{filepath}",
        }
    ]

    with open(input, "w") as f:
        f.write(str(x))

    return str(tmp_path)


def get_parsed_info(tmp_path, file_contents):
    source_path = tmp_path / "file.hpp"

    with open(source_path, "w") as f:
        f.write(str(file_contents))

    parsed_info = parse.parse_file(
        source=str(source_path),
        compilation_database_path=create_compilation_database(
            tmp_path=tmp_path, filepath=source_path
        ),
    )

    return parsed_info


def test_anonymous_decls(tmp_path):
    file_contents = """
    union {
        struct {
            enum {};
        };
    };
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    union_decl = parsed_info["members"][0]

    assert union_decl["kind"] == "UNION_DECL"
    assert union_decl["name"] == ""

    struct_decl = union_decl["members"][0]

    assert struct_decl["kind"] == "STRUCT_DECL"
    assert struct_decl["name"] == ""

    enum_decl = struct_decl["members"][0]

    assert enum_decl["kind"] == "ENUM_DECL"
    assert enum_decl["name"] == ""


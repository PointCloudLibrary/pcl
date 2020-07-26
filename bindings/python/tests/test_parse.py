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

    assert union_decl["kind"] == "ANONYMOUS_UNION_DECL"
    assert union_decl["name"] == ""

    struct_decl = union_decl["members"][0]

    assert struct_decl["kind"] == "ANONYMOUS_STRUCT_DECL"
    assert struct_decl["name"] == ""

    enum_decl = struct_decl["members"][0]

    assert enum_decl["kind"] == "ANONYMOUS_ENUM_DECL"
    assert enum_decl["name"] == ""


def test_translation_unit(tmp_path):
    file_contents = ""
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    assert parsed_info["kind"] == "TRANSLATION_UNIT"
    assert parsed_info["depth"] == 0
    assert parsed_info["name"] == str(tmp_path / "file.hpp")


def test_namespace(tmp_path):
    file_contents = "namespace a_namespace {}"
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    namespace = parsed_info["members"][0]

    assert namespace["kind"] == "NAMESPACE"
    assert namespace["name"] == "a_namespace"


def test_namespace_ref(tmp_path):
    file_contents = """
    #include <ostream>
    std::ostream anOstream;
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    var_decl = parsed_info["members"][0]
    namespace_ref = var_decl["members"][0]

    assert namespace_ref["kind"] == "NAMESPACE_REF"
    assert namespace_ref["name"] == "std"


def test_var_decl(tmp_path):
    file_contents = "int anInt = 1;"
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    var_decl = parsed_info["members"][0]

    assert var_decl["kind"] == "VAR_DECL"
    assert var_decl["element_type"] == "Int"
    assert var_decl["name"] == "anInt"


def test_field_decl(tmp_path):
    file_contents = """
    struct AStruct {
        int aClassMember;
    };
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    struct_decl = parsed_info["members"][0]
    field_decl = struct_decl["members"][0]

    assert field_decl["kind"] == "FIELD_DECL"
    assert field_decl["element_type"] == "Int"
    assert field_decl["name"] == "aClassMember"


def test_parm_decl(tmp_path):
    file_contents = """
    struct AStruct {
        AStruct(int aFunctionParameter) {}
    };
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    struct_decl = parsed_info["members"][0]
    constructor = struct_decl["members"][0]
    parm_decl = constructor["members"][0]

    assert parm_decl["kind"] == "PARM_DECL"
    assert parm_decl["element_type"] == "Int"
    assert parm_decl["name"] == "aFunctionParameter"


def test_return_type_composition(tmp_path):
    file_contents = ""
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    assert type(parsed_info) is dict
    assert type(parsed_info["members"]) is list
    assert len(parsed_info["members"]) == 0


def test_translation_unit(tmp_path):
    file_contents = ""
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    assert parsed_info["kind"] == "TRANSLATION_UNIT"
    assert parsed_info["depth"] == 0
    assert parsed_info["line"] == 0
    assert parsed_info["column"] == 0
    assert parsed_info["name"] == str(tmp_path / "file.hpp")


def test_namespace(tmp_path):
    file_contents = "namespace a_namespace {}"
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    namespace = parsed_info["members"][0]

    assert namespace["kind"] == "NAMESPACE"
    assert namespace["name"] == "a_namespace"


def test_namespace_ref(tmp_path):
    file_contents = """
    #include <ostream>
    std::ostream anOstream;
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    var_decl = parsed_info["members"][0]
    namespace_ref = var_decl["members"][0]

    assert namespace_ref["kind"] == "NAMESPACE_REF"
    assert namespace_ref["name"] == "std"


def test_var_decl(tmp_path):
    file_contents = "int anInt = 1;"
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    var_decl = parsed_info["members"][0]

    assert var_decl["kind"] == "VAR_DECL"
    assert var_decl["element_type"] == "Int"
    assert var_decl["name"] == "anInt"


def test_call_expr(tmp_path):
    file_contents = """
    int aFunction() {
        return 1;
    }
    int anInt = aFunction();
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    var_decl = parsed_info["members"][1]
    call_expr = var_decl["members"][0]

    assert call_expr["kind"] == "CALL_EXPR"
    assert call_expr["name"] == "aFunction"


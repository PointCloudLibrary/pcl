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

    inclusion_directive = parsed_info["members"][0]

    assert inclusion_directive["kind"] == "INCLUSION_DIRECTIVE"
    assert inclusion_directive["name"] == "ostream"

    var_decl = parsed_info["members"][1]
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


def test_parsed_info_structure(tmp_path):
    file_contents = ""
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    assert type(parsed_info) is dict
    assert type(parsed_info["members"]) is list
    assert len(parsed_info["members"]) == 0


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

    assert var_decl["name"] == "anInt"


def test_struct_decl(tmp_path):
    file_contents = "struct AStruct {};"
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    struct_decl = parsed_info["members"][0]

    assert struct_decl["kind"] == "STRUCT_DECL"
    assert struct_decl["name"] == "AStruct"


def test_public_inheritance(tmp_path):
    file_contents = """
    struct BaseStruct {};
    struct DerivedStruct: public BaseStruct {};
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    child_struct_decl = parsed_info["members"][1]
    cxx_base_specifier = child_struct_decl["members"][0]

    assert cxx_base_specifier["kind"] == "CXX_BASE_SPECIFIER"
    assert cxx_base_specifier["access_specifier"] == "PUBLIC"
    assert cxx_base_specifier["name"] == "struct BaseStruct"


def test_member_function(tmp_path):
    file_contents = """
    struct AStruct {
        void aMethod() {}
    };
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    struct_decl = parsed_info["members"][0]
    cxx_method = struct_decl["members"][0]

    assert cxx_method["kind"] == "CXX_METHOD"
    assert cxx_method["result_type"] == "void"
    assert cxx_method["name"] == "aMethod"


def test_type_ref(tmp_path):
    file_contents = """
    struct SomeUsefulType {};

    class AClass {
        void aMethod(SomeUsefulType aParameter) {};
    };
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    class_decl = parsed_info["members"][1]
    cxx_method = class_decl["members"][0]
    parm_decl = cxx_method["members"][0]

    assert parm_decl["name"] == "aParameter"

    type_ref = parm_decl["members"][0]

    assert type_ref["kind"] == "TYPE_REF"
    assert type_ref["name"] == "struct SomeUsefulType"


def test_simple_constructor(tmp_path):
    file_contents = """
    struct AStruct {
        AStruct() {}
    };
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    struct_decl = parsed_info["members"][0]
    constructor = struct_decl["members"][0]

    assert constructor["kind"] == "CONSTRUCTOR"
    assert constructor["access_specifier"] == "PUBLIC"
    assert constructor["name"] == "AStruct"


def test_unexposed_expr(tmp_path):
    file_contents = """
    class SimpleClassWithConstructor {
        int aClassMember;
        SimpleClassWithConstructor(int aConstructorParameter) : aClassMember(aConstructorParameter) {};
    };
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    struct_decl = parsed_info["members"][0]
    constructor = struct_decl["members"][1]
    member_ref = constructor["members"][1]

    assert member_ref["name"] == "aClassMember"

    unexposed_expr = constructor["members"][2]

    assert unexposed_expr["kind"] == "UNEXPOSED_EXPR"
    assert unexposed_expr["name"] == "aConstructorParameter"


# @TODO: Not sure how to reproduce. Maybe later.
# def test_member_ref_expr(tmp_path):


def test_decl_ref_expr(tmp_path):
    file_contents = """
    struct AStruct {
        int firstMember, secondMember;
        AStruct(int firstFunctionParameter, int secondFunctionParameter)
        : firstMember(secondFunctionParameter), secondMember(firstFunctionParameter)
        {}
    };
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    struct_decl = parsed_info["members"][0]
    constructor = struct_decl["members"][2]
    unexposed_expr_1 = constructor["members"][3]
    unexposed_expr_2 = constructor["members"][5]
    decl_ref_expr_1 = unexposed_expr_1["members"][0]
    decl_ref_expr_2 = unexposed_expr_2["members"][0]

    assert decl_ref_expr_1["kind"] == "DECL_REF_EXPR"
    assert decl_ref_expr_2["kind"] == "DECL_REF_EXPR"
    assert decl_ref_expr_1["name"] == "secondFunctionParameter"
    assert decl_ref_expr_2["name"] == "firstFunctionParameter"


def test_member_ref(tmp_path):
    file_contents = """
    struct AStruct {
        int firstMember, secondMember;
        AStruct(int firstFunctionParameter, int secondFunctionParameter)
        : firstMember(secondFunctionParameter), secondMember(firstFunctionParameter)
        {}
    };
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)
    struct_decl = parsed_info["members"][0]
    constructor = struct_decl["members"][2]
    member_ref_1 = constructor["members"][2]
    member_ref_2 = constructor["members"][4]

    assert member_ref_1["kind"] == "MEMBER_REF"
    assert member_ref_2["kind"] == "MEMBER_REF"
    assert member_ref_1["element_type"] == "Int"
    assert member_ref_2["element_type"] == "Int"
    assert member_ref_1["name"] == "firstMember"
    assert member_ref_2["name"] == "secondMember"


def test_class_template(tmp_path):
    file_contents = """
    template <typename T>
    struct AStruct {};
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    class_template = parsed_info["members"][0]

    assert class_template["kind"] == "CLASS_TEMPLATE"
    assert class_template["name"] == "AStruct"

    template_type_parameter = class_template["members"][0]

    assert template_type_parameter["kind"] == "TEMPLATE_TYPE_PARAMETER"
    assert template_type_parameter["name"] == "T"
    assert template_type_parameter["access_specifier"] == "PUBLIC"


def test_template_non_type_parameter(tmp_path):
    file_contents = """
    template <int N>
    struct AStruct {};
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    class_template = parsed_info["members"][0]

    assert class_template["kind"] == "CLASS_TEMPLATE"
    assert class_template["name"] == "AStruct"

    template_non_type_parameter = class_template["members"][0]

    assert template_non_type_parameter["kind"] == "TEMPLATE_NON_TYPE_PARAMETER"
    assert template_non_type_parameter["element_type"] == "Int"
    assert template_non_type_parameter["name"] == "N"


def test_function_template(tmp_path):
    file_contents = """
    template <typename T>
    void aFunction() {}
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    function_template = parsed_info["members"][0]

    assert function_template["kind"] == "FUNCTION_TEMPLATE"
    assert function_template["result_type"] == "void"
    assert function_template["name"] == "aFunction"

    template_type_parameter = function_template["members"][0]

    assert template_type_parameter["kind"] == "TEMPLATE_TYPE_PARAMETER"
    assert template_type_parameter["name"] == "T"
    assert template_type_parameter["access_specifier"] == "PUBLIC"


def test_template_type_parameter(tmp_path):
    file_contents = """
    template <typename T>
    struct AStruct {};

    template <typename P>
    void aFunction() {}
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    class_template = parsed_info["members"][0]
    template_type_parameter = class_template["members"][0]

    assert template_type_parameter["kind"] == "TEMPLATE_TYPE_PARAMETER"
    assert template_type_parameter["element_type"] == "Unexposed"
    assert template_type_parameter["name"] == "T"

    function_template = parsed_info["members"][1]
    template_type_parameter = function_template["members"][0]

    assert template_type_parameter["kind"] == "TEMPLATE_TYPE_PARAMETER"
    assert template_type_parameter["element_type"] == "Unexposed"
    assert template_type_parameter["name"] == "P"


def test_default_delete_constructor(tmp_path):
    file_contents = """
    class aClass {
        aClass() = default;

        // disable the copy constructor
        aClass(double) = delete;
    };
    """
    parsed_info = get_parsed_info(tmp_path=tmp_path, file_contents=file_contents)

    class_decl = parsed_info["members"][0]

    default_constructor = class_decl["members"][0]

    assert default_constructor["kind"] == "CONSTRUCTOR"
    assert default_constructor["name"] == "aClass"
    assert default_constructor["result_type"] == "void"

    delete_constructor = class_decl["members"][1]

    assert delete_constructor["kind"] == "CONSTRUCTOR"
    assert delete_constructor["name"] == "aClass"
    assert delete_constructor["result_type"] == "void"

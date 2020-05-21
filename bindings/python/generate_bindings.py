import json


def read_json(filename):
    with open(filename, "r") as f:
        return json.load(f)


def write_to_cpp(filename, linelist):
    with open(filename, "w") as f:
        for line in linelist:
            f.writelines(line)
            f.writelines("\n")


namespace = None
in_struct = False
module_linelist = []


def check_namespace(item):
    if namespace:
        item["identifier"] = f"{namespace}::{item['identifier']}"


def handle_final(filename, module_name):
    linelist = []
    linelist.append(f"#include <{filename}>")
    linelist.append("#include <pybind11/pybind11.h>")
    linelist.append("namespace py = pybind11;")
    linelist.append(f"PYBIND11_MODULE({module_name}, m)")
    linelist.append("{")
    for line in module_linelist:
        linelist.append(line)
    linelist.append("}")
    return linelist


def handle_include(item):
    pass


def handle_alias(item):
    pass


def handle_constructor(item):
    parameters = ""
    parameters = ",".join(params for params in item["parameters"])
    module_linelist.append(f".def(py::init<{parameters}>())")


def handle_operator(item):
    if in_struct:
        module_linelist.append(f'.def(py::self {item["identifier"]} py::self)')


def handle_namespace(item):
    global namespace
    namespace = item["identifier"]
    for sub_item in item["members"]:
        type_functions[sub_item["type"]](sub_item)
    namespace = None


def handle_struct(item):
    global in_struct
    in_struct = True
    name_decl = item["identifier"]
    check_namespace(item)
    name_impl = item["identifier"]
    if "parent" in item.keys():
        name_impl = f'{item["identifier"]}, {item["parent"]}'

    module_linelist.append(f'py::class_<{name_impl}>(m, "{name_decl}")')
    for sub_item in item["members"]:
        type_functions[sub_item["type"]](sub_item)
    module_linelist.append(";")
    in_struct = False


def handle_macro(item):
    pass


type_functions = {
    "include": handle_include,
    "namespace": handle_namespace,
    "alias": handle_alias,
    "struct": handle_struct,
    "constructor": handle_constructor,
    "operator": handle_operator,
    "macro": handle_macro,
}


def main():
    header_info = read_json("common/point_types.json")
    for item in header_info:
        type_functions[item["type"]](item)
    lines_to_write = handle_final(filename="pcl/point_types.h", module_name="pcl")
    write_to_cpp(filename="common/py_point_types.cpp", linelist=lines_to_write)


if __name__ == "__main__":
    main()

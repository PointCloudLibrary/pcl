import json


def read_json(filename):
    with open(filename, "r") as f:
        return json.load(f)


namespace = None
module_linelist = []


def check_namespace(item):
    if namespace:
        item["identifier"] = f"{namespace}::{item['identifier']}"


def handle_final(module):
    linelist = []
    linelist.append("#include <point_types.h>")
    linelist.append("#include <pybind11/pybind11.h>")
    linelist.append("namespace py = pybind11;")
    linelist.append(f"PYBIND11_MODULE({module}, m)")
    linelist.append("{")
    for line in module_linelist:
        linelist.append(line)
    linelist.append("}")


def handle_include(item):
    pass


def handle_alias(item):
    pass


def handle_constructor(item):
    """
    @TODO
    """



def handle_namespace(item):
    namespace = item["identifier"]
    for sub_item in item["members"]:
        type_functions[sub_item["type"]]
    namespace = None


def handle_struct(item):
    name = item["identifier"]
    check_namespace(item)
    module_linelist.append(f'py::class_<{name}>(m, "{name}")')
    """
    @TODO
    Think about parent classes, which are not implementations
    """
    for sub_item in item["members"]:
        type_functions[sub_item["type"]]


type_functions = {
    "include": handle_include(item),
    "namespace": handle_namespace(item),
    "alias": handle_alias(item),
    "struct": handle_struct(struct),
    "constructor": handle_constructor(item),
    "operator": handle_operator(item),
}


def main():
    handle_final(module="pcl")
    header_info = read_json("point_types.json")
    for item in header_info:
        for key, value in item.items():
            print(key, value)

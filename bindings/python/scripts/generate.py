import json
import argparse
import sys
import os


class bind:
    def __init__(self, root):
        self._state_stack = []
        self.linelist = []
        self._skipped = []
        self.kind_functions = {
            "TRANSLATION_UNIT": [self.skip],
            "NAMESPACE": [self.handle_namespace_0],  # self.handle_namespace_1],
            "NAMESPACE_REF": [self.skip],
            "STRUCT_DECL": [self.handle_struct_decl_0],  # self.handle_struct_decl_1],
            "CXX_BASE_SPECIFIER": [self.skip],
            "CXX_METHOD": [self.skip],  # [self.handle_cxx_method],
            "VAR_DECL": [self.skip],
            "TYPE_REF": [self.skip],
            "CONSTRUCTOR": [self.handle_constructor],
            "PARM_DECL": [self.skip],
            "CALL_EXPR": [self.skip],
            "UNEXPOSED_EXPR": [self.skip],
            "MEMBER_REF_EXPR": [self.skip],
            "DECL_REF_EXPR": [self.skip],
            "FIELD_DECL": [self.skip],#[self.handle_field_decl],
            "MEMBER_REF": [self.skip],
            "CLASS_TEMPLATE": [
                self.skip
            ],  # [self.handle_class_template_0],# self.handle_class_template_1],
            "TEMPLATE_NON_TYPE_PARAMETER": [self.skip],
            "FUNCTION_TEMPLATE": [self.skip],
        }

        self.handle_node(root)

    def get_prev_depth_node(self):
        for prev_item in reversed(self._state_stack):
            if prev_item["depth"] == self.depth - 1:
                return prev_item
        return

    def skip(self):
        self._skipped.append(
            {
                "line": self.item["line"],
                "column": self.item["line"],
                "kind": self.kind,
                "name": self.name,
            }
        )

    def close(self):
        if self._state_stack[-1]["kind"] == "NAMESPACE":
            self.linelist.append("}")
        if self._state_stack[-1]["kind"] == "STRUCT_DECL":
            self.linelist.append(";")
        if self._state_stack[-1]["kind"] == "CLASS_TEMPLATE":
            self.linelist.append(";")

    def handle_node(self, item):
        self.item = item
        self.kind = self.item["kind"]
        self.name = self.item["name"]
        self.members = self.item["members"]
        self.depth = self.item["depth"]

        self._state_stack.append(
            {"kind": self.kind, "name": self.name, "depth": self.depth}
        )

        self.kind_functions[self.kind][0]()

        if self.kind_functions[self.kind][0] is not self.skip:
            for sub_item in self.members:
                self.handle_node(sub_item)

        if len(self.kind_functions[self.kind]) > 1:
            print("adf")
            self.kind_functions[self.kind][1]()

        self.close()

        self._state_stack.pop()

    def handle_namespace_0(self):
        self.linelist.append(f"namespace {self.name}" + "{")

    # def handle_namespace_1(self):
    #     self.linelist.append("}")

    def handle_struct_decl_0(self):
        cxx_base_specifier_list = [
            sub_item["name"]
            for sub_item in self.members
            if sub_item["kind"] == "CXX_BASE_SPECIFIER"
        ]
        if cxx_base_specifier_list:
            cxx_base_specifier_list = ",".join(cxx_base_specifier_list)
            self.linelist.append(
                f'py::class_<{self.name}, {cxx_base_specifier_list}>(m, "{self.name}")'
            )
        else:
            self.linelist.append(f'py::class_<{self.name}>(m, "{self.name}")')

        for sub_item in self.members:
            if sub_item["kind"] == "FIELD_DECL":
                if sub_item["element_type"] == "ConstantArray":
                    self.linelist.append(
                        f'.def_property_readonly("{sub_item["name"]}", []({self.name}& obj) {{return obj.{sub_item["name"]}; }})'#float[ ' + f'obj.{sub_item["name"]}' + '.size()];} )'
                    )
                else:  
                    self.linelist.append(
                        f'.def_readwrite("{sub_item["name"]}", &{self.name}::{sub_item["name"]})'
                    )

            # if sub_item["kind"] == "CXX_METHOD":
            #     self.linelist.append(
            #         f'.def("{sub_item["name"]}", &{self.name}::{sub_item["name"]})'
            #     )

    # def handle_struct_decl_1(self):
    #     self.linelist.append(";")

    def handle_cxx_method(self):
        prev_depth_node = self.get_prev_depth_node()
        if prev_depth_node:
            method_of = prev_depth_node["name"]
            self.linelist.append(f'.def("{self.name}", &{method_of}::{self.name})')
        else:
            self.linelist.append(f'.def("{self.name}", &{self.name})')

    def handle_constructor(self):
        argument_type_list = []
        parameter_decl_list = []
        for sub_item in self.members:
            if sub_item["kind"] == "PARM_DECL":
                parameter_decl_list.append(sub_item["name"])
                if sub_item["element_type"] == "LValueReference":
                    for sub_sub_item in sub_item["members"]:
                        if sub_sub_item["kind"] == "TYPE_REF":
                            # @TODO
                            type_ref = (
                                sub_sub_item["name"]
                                .replace("struct ", "")
                                .replace("pcl::", "")
                            )
                            argument_type_list.append(f"{type_ref} &")
                elif sub_item["element_type"] == "Elaborated":
                    namespace_ref = ""
                    for sub_sub_item in sub_item["members"]:
                        if sub_sub_item["kind"] == "NAMESPACE_REF":
                            namespace_ref += f'{sub_sub_item["name"]}::'
                        if sub_sub_item["kind"] == "TYPE_REF":
                            argument_type_list.append(
                                f'{namespace_ref}{sub_sub_item["name"]}'
                            )
                elif sub_item["element_type"] in ["Float", "Int"]:
                    argument_type_list.append(f'{sub_item["element_type"].lower()}')
                else:
                    argument_type_list.append(f'{sub_item["element_type"]}')
        parameter_decl_list = ",".join([decl + "_a" for decl in parameter_decl_list])
        argument_type_list = ",".join(argument_type_list)
        self.linelist.append(
            f".def(py::init<{argument_type_list}>())"  # , {parameter_decl_list})"
        )

    def handle_field_decl(self):
        prev_depth_node = self.get_prev_depth_node()
        if prev_depth_node:
            field_of = prev_depth_node["name"]
            self.linelist.append(
                f'.def_readwrite("{self.name}", &{field_of}::{self.name})'
            )
        else:
            self.linelist.append(f'.def("{self.name}", &{self.name})')

    def handle_class_template_0(self):
        flag = False
        for sub_item in self.members:
            if sub_item["kind"] == "TEMPLATE_NON_TYPE_PARAMETER":
                self.linelist.append(
                    f'template< {sub_item["element_type"].lower()} {sub_item["name"]} >'
                )
                flag = True
        if not flag:
            self.linelist.append(f"template<>")
        cxx_base_specifier_list = [
            sub_item["name"]
            for sub_item in self.members
            if sub_item["kind"] == "CXX_BASE_SPECIFIER"
        ]
        if cxx_base_specifier_list:
            cxx_base_specifier_list = ",".join(cxx_base_specifier_list)
            self.linelist.append(
                f'py::class_<{self.name, cxx_base_specifier_list}>(m, "{self.name}")'
            )
        else:
            self.linelist.append(f'py::class_<{self.name}>(m, "{self.name}")')

    def handle_class_template_1(self):
        self.linelist.append(";")

    def handle_final(self, filename, module_name):
        final = []
        final.append(f"#include <{filename}>")
        final.append("#include <pybind11/pybind11.h>")
        final.append("namespace py = pybind11;")
        final.append("using namespace py::literals;")
        for i in range(len(self.linelist)):
            if self.linelist[i].startswith("namespace"):
                continue
            else:
                self.linelist[i] = "".join(
                    (f"PYBIND11_MODULE({module_name}, m)", "{", self.linelist[i])
                )
                break
        for line in self.linelist:
            final.append(line)
        final.append("}")
        return final


def read_json(filename):
    with open(filename, "r") as f:
        return json.load(f)


def write_to_cpp(filename, linelist):
    with open(filename, "w") as f:
        for line in linelist:
            f.writelines(line)
            f.writelines("\n")


def get_output_path(source, output_dir):
    x_list = source.split("json/", 1)[-1]
    x_list = x_list.split("/")

    filename = x_list[-1].split(".")[0]
    relative_dir = "/".join(x for x in x_list[:-1])
    dir = os.path.join(output_dir, relative_dir)

    # ensure the new directory exists
    if not os.path.exists(dir):
        os.makedirs(dir)

    return f"{dir}/{filename}.cpp"


def parse_arguments(args):
    parser = argparse.ArgumentParser(description="JSON to pybind11 generation")
    parser.add_argument("files", nargs="+", help="JSON input")
    return parser.parse_args(args)


def main():
    args = parse_arguments(sys.argv[1:])

    for source in args.files:
        header_info = read_json(source)
        if header_info:
            bind_object = bind(header_info[0])
            lines_to_write = bind_object.handle_final(
                filename="pcl/point_types.h", module_name="pcl"
            )
            output_filepath = get_output_path(
                os.path.realpath(source),
                output_dir=f"pybind11/{os.path.dirname(__file__)}",
            )
            write_to_cpp(filename=output_filepath, linelist=lines_to_write)

        else:
            raise Exception("Empty json")


if __name__ == "__main__":
    main()

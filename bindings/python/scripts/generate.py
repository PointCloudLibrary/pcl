from context import scripts
import scripts.utils as utils


class bind:
    def __init__(self, root):
        self._state_stack = []
        self.linelist = []
        self._skipped = []
        self.inclusion_list = []
        self.kind_functions = {
            "TRANSLATION_UNIT": self.skip,
            "NAMESPACE": self.handle_namespace,
            "NAMESPACE_REF": self.skip,
            "STRUCT_DECL": self.handle_struct_decl,
            "CXX_BASE_SPECIFIER": self.skip,
            "CXX_METHOD": self.skip,
            "VAR_DECL": self.skip,
            "TYPE_REF": self.skip,
            "CONSTRUCTOR": self.handle_constructor,
            "PARM_DECL": self.skip,
            "CALL_EXPR": self.skip,
            "UNEXPOSED_EXPR": self.skip,
            "MEMBER_REF_EXPR": self.skip,
            "DECL_REF_EXPR": self.skip,
            "FIELD_DECL": self.skip,
            "MEMBER_REF": self.skip,
            "CLASS_TEMPLATE": self.skip,  # self.handle_class_template
            "TEMPLATE_NON_TYPE_PARAMETER": self.skip,
            "FUNCTION_TEMPLATE": self.skip,
            "ANONYMOUS_UNION_DECL": self.skip,
            "ALIGNED_ATTR": self.skip,
            "INTEGER_LITERAL": self.skip,
            "ANONYMOUS_STRUCT_DECL": self.skip,
            "COMPOUND_STMT": self.skip,
            "FLOATING_LITERAL": self.skip,
            "BINARY_OPERATOR": self.skip,
            "ARRAY_SUBSCRIPT_EXPR": self.skip,
            "CXX_THROW_EXPR": self.skip,
            "FRIEND_DECL": self.skip,
            "FUNCTION_DECL": self.skip,
            "INIT_LIST_EXPR": self.skip,
            "RETURN_STMT": self.skip,
            "OVERLOADED_DECL_REF": self.skip,
            "UNARY_OPERATOR": self.skip,
            "IF_STMT": self.skip,
            "OBJ_BOOL_LITERAL_EXPR": self.skip,
            "INCLUSION_DIRECTIVE": self.handle_inclusion_directive,
            "MACRO_DEFINITION": self.skip,
            "MACRO_INSTANTIATION": self.skip,
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
        self.name = self.item["name"] if "name" in self.item else ""
        self.members = self.item["members"]
        self.depth = self.item["depth"]

        self._state_stack.append(
            {"kind": self.kind, "name": self.name, "depth": self.depth}
        )

        self.kind_functions[self.kind]()

        if self.kind_functions[self.kind] is not self.skip:
            for sub_item in self.members:
                self.handle_node(sub_item)

        self.close()

        self._state_stack.pop()

    def handle_namespace(self):
        self.linelist.append(f"namespace {self.name}" + "{")

    def get_fields_from_anonymous(self, item):
        fields = []
        for sub_item in item["members"]:
            if sub_item["kind"] in ["ANONYMOUS_UNION_DECL", "ANONYMOUS_STRUCT_DECL"]:
                for field in self.get_fields_from_anonymous(sub_item):
                    fields.append(field)
            if sub_item["kind"] == "FIELD_DECL":
                fields.append(sub_item)
        return fields

    def handle_struct_decl(self):
        cxx_base_specifier_list = [
            sub_item["name"]
            for sub_item in self.members
            if sub_item["kind"] == "CXX_BASE_SPECIFIER"
        ]
        if cxx_base_specifier_list:
            cxx_base_specifier_list = ",".join(cxx_base_specifier_list)
            self.linelist.append(
                f'py::class_<{self.name}, {cxx_base_specifier_list.replace("struct ", "").replace("pcl::", "")}>(m, "{self.name}")'
            )
        else:
            self.linelist.append(f'py::class_<{self.name}>(m, "{self.name}")')


        # handle anonymous
        for sub_item in self.members:
            fields = self.get_fields_from_anonymous(sub_item)
            for field in fields:
                if field["element_type"] == "ConstantArray":
                    self.linelist.append(
                        f'.def_property_readonly("{field["name"]}", []({self.name}& obj) {{return obj.{field["name"]}; }})'  # float[ ' + f'obj.{sub_item["name"]}' + '.size()];} )'
                    )
                else:
                    self.linelist.append(
                        f'.def_readwrite("{field["name"]}", &{self.name}::{field["name"]})'
                    )

        for sub_item in self.members:
            if sub_item["kind"] == "FIELD_DECL":
                if sub_item["element_type"] == "ConstantArray":
                    self.linelist.append(
                        f'.def_property_readonly("{sub_item["name"]}", []({self.name}& obj) {{return obj.{sub_item["name"]}; }})'  # float[ ' + f'obj.{sub_item["name"]}' + '.size()];} )'
                    )
                else:
                    self.linelist.append(
                        f'.def_readwrite("{sub_item["name"]}", &{self.name}::{sub_item["name"]})'
                    )

            if sub_item["kind"] == "CXX_METHOD":
                # TODO: Add template args, currently blank
                if sub_item["name"] not in ("PCL_DEPRECATED"):
                    self.linelist.append(
                        f'.def("{sub_item["name"]}", py::overload_cast<>(&{self.name}::{sub_item["name"]}))'
                    )

    def handle_constructor(self):
        argument_type_list = []
        for sub_item in self.members:
            if sub_item["kind"] == "PARM_DECL":
                if sub_item["element_type"] == "LValueReference":
                    for sub_sub_item in sub_item["members"]:
                        if sub_sub_item["kind"] == "TYPE_REF":
                            # @TODO: Make more robust
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
        argument_type_list = ",".join(argument_type_list)
        self.linelist.append(f".def(py::init<{argument_type_list}>())")

    def handle_class_template(self):
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

    def handle_inclusion_directive(self):
        # blacklist = ["pcl/memory.h", "pcl/pcl_macros.h",]
        if self.name.startswith("pcl"):
            self.inclusion_list.append(self.name)

    def handle_final(self, filename, module_name):
        final = []
        final.append(f"#include <{filename}>")
        final.append("#include <pybind11/pybind11.h>")
        # final.append("#include<pybind11/stl.h>")
        # final.append("#include<pybind11/stl_bind.h>")
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


def main():
    args = utils.parse_arguments(script="generate")

    for source in args.files:
        source = utils.get_realpath(path=source)

        header_info = utils.read_json(filename=source)
        if header_info:
            bind_object = bind(header_info)
            lines_to_write = bind_object.handle_final(
                filename="pcl/point_types.h", module_name="pcl"
            )
            print(bind_object._skipped)
            output_filepath = utils.get_output_path(
                source=source,
                output_dir=utils.join_path(args.pybind11_output_path, "pybind11"),
                split_from="json",
                extension=".cpp",
            )
            utils.write_to_file(filename=output_filepath, linelist=lines_to_write)

        else:
            raise Exception("Empty json")


if __name__ == "__main__":
    main()

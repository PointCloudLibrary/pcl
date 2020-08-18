from context import scripts
import scripts.utils as utils


class bind:
    def __init__(self, root):
        self._state_stack = []
        self.linelist = []
        self._skipped = []
        self.inclusion_list = []
        handled_by_pybind = self.skip  # handled by pybind11
        handled_elsewhere = self.skip  # handled in another kind's function
        no_need_to_handle = self.skip  # unnecessary kind
        unsure = self.skip  # unsure as to needed or not
        self.kind_functions = {
            "TRANSLATION_UNIT": no_need_to_handle,
            "NAMESPACE": self.handle_namespace,
            "NAMESPACE_REF": handled_elsewhere,  # in (handle_constructor)
            "STRUCT_DECL": self.handle_struct_decl,
            "CXX_BASE_SPECIFIER": handled_elsewhere,  # in (handle_struct_decl)
            "CXX_METHOD": handled_elsewhere,  # in (handle_struct_decl)
            "VAR_DECL": handled_by_pybind,
            "TYPE_REF": handled_elsewhere,  # in (handle_constructor)
            "CONSTRUCTOR": self.handle_constructor,
            "PARM_DECL": handled_elsewhere,  # in (handle_constructor)
            "CALL_EXPR": handled_by_pybind,
            "UNEXPOSED_EXPR": unsure,
            "MEMBER_REF_EXPR": unsure,
            "DECL_REF_EXPR": unsure,
            "FIELD_DECL": handled_elsewhere,  # in (handle_struct_decl)
            "MEMBER_REF": handled_by_pybind,
            "CLASS_TEMPLATE": self.skip,  # self.handle_class_template
            "TEMPLATE_NON_TYPE_PARAMETER": handled_elsewhere,  # in (handle_class_template)
            "FUNCTION_TEMPLATE": self.skip,  # to be added later
            "ANONYMOUS_UNION_DECL": handled_elsewhere,  # in (handle_struct_decl) via get_fields_from_anonymous
            "ALIGNED_ATTR": no_need_to_handle,
            "INTEGER_LITERAL": unsure,
            "ANONYMOUS_STRUCT_DECL": handled_elsewhere,  # in (handle_struct_decl) via get_fields_from_anonymous
            "COMPOUND_STMT": no_need_to_handle,
            "FLOATING_LITERAL": unsure,
            "BINARY_OPERATOR": no_need_to_handle,
            "ARRAY_SUBSCRIPT_EXPR": handled_by_pybind,
            "CXX_THROW_EXPR": handled_by_pybind,
            "FRIEND_DECL": unsure,
            "FUNCTION_DECL": unsure,
            "INIT_LIST_EXPR": no_need_to_handle,
            "RETURN_STMT": handled_by_pybind,
            "OVERLOADED_DECL_REF": unsure,
            "UNARY_OPERATOR": no_need_to_handle,
            "IF_STMT": no_need_to_handle,
            "OBJ_BOOL_LITERAL_EXPR": unsure,
            "INCLUSION_DIRECTIVE": self.handle_inclusion_directive,
            "MACRO_DEFINITION": unsure,
            "MACRO_INSTANTIATION": unsure,
        }

        self.handle_node(root)

    def skip(self):
        """
        Used to keep track of skipped elements, for debugging purposes.

        Skipped elements can be:
            - elements which are not handled in their own function, or
            - elements which are not handled at all (skipped).
        """

        self._skipped.append(
            {
                "line": self.item["line"],
                "column": self.item["line"],
                "kind": self.kind,
                "name": self.name,
            }
        )

    def end_scope(self):
        """
        Used for adding ending characters (braces, semicolons, etc.) when state's scope ends.
        """

        if self._state_stack[-1]["kind"] == "NAMESPACE":
            self.linelist.append("}")
        if self._state_stack[-1]["kind"] == "STRUCT_DECL":
            self.linelist.append(";")
        if self._state_stack[-1]["kind"] == "CLASS_TEMPLATE":
            self.linelist.append(";")

    def handle_node(self, item):
        """
        Function for handling a node (any type).

        - Not to be called explicitly, it is called when a class' object is initialised.
        - Begins with the root i.e., TRANSLATION_UNIT and then recursively works through the AST.
        - Function pipeline:
          >>>
          |  1. Initialisations of member variables like item, kind, name, etc.
          |  2. Push the item's info to the state stack.
          |  3. Call the designated function for the item.
          |  4. If the designated function was not to skip the item's handling, recursively call its members' functions.
          <<<
            5. End the scope, if applicable.
            6. Pop the item's info from the stack.
        """

        self.item = item
        self.kind = self.item["kind"]
        self.name = self.item["name"]
        self.members = self.item["members"]
        self.depth = self.item["depth"]

        self._state_stack.append(
            {"kind": self.kind, "name": self.name, "depth": self.depth}
        )

        self.kind_functions[self.kind]()

        if self.kind_functions[self.kind] is not self.skip:
            for sub_item in self.members:
                self.handle_node(sub_item)

        self.end_scope()

        self._state_stack.pop()

    def handle_namespace(self):
        """
        Handles `CursorKind.NAMESPACE`
        """

        self.linelist.append(f"namespace {self.name}" + "{")

    @staticmethod
    def get_fields_from_anonymous(item):
        """
        Helper function to extract fields from anonymous types.

        Parameters:
            - item (dict): the anonymous type item from which to extract fields
        
        Returns:
            - fields (list): A list of items of kind `CursorKind.FIELD_DECL`
        """

        fields = []
        for sub_item in item["members"]:
            # base condition
            if sub_item["kind"] == "FIELD_DECL":
                fields.append(sub_item)
            # recurse
            if sub_item["kind"] in ("ANONYMOUS_UNION_DECL", "ANONYMOUS_STRUCT_DECL"):
                fields += bind.get_fields_from_anonymous(sub_item)
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

        # TODO: Merge this and next block via a design updation
        # handle anonymous structs, etc. as field declarations
        for sub_item in self.members:
            fields = self.get_fields_from_anonymous(sub_item)
            for field in fields:
                if field["element_type"] == "ConstantArray":
                    # TODO: FIX: readwrite, not readonly
                    self.linelist.append(
                        f'.def_property_readonly("{field["name"]}", []({self.name}& obj) {{return obj.{field["name"]}; }})'  # float[ ' + f'obj.{sub_item["name"]}' + '.size()];} )'
                    )
                else:
                    self.linelist.append(
                        f'.def_readwrite("{field["name"]}", &{self.name}::{field["name"]})'
                    )

        for sub_item in self.members:

            # handle field declarations
            if sub_item["kind"] == "FIELD_DECL":
                if sub_item["element_type"] == "ConstantArray":
                    self.linelist.append(
                        f'.def_property_readonly("{sub_item["name"]}", []({self.name}& obj) {{return obj.{sub_item["name"]}; }})'  # float[ ' + f'obj.{sub_item["name"]}' + '.size()];} )'
                    )
                else:
                    self.linelist.append(
                        f'.def_readwrite("{sub_item["name"]}", &{self.name}::{sub_item["name"]})'
                    )

            # handle class methods
            if sub_item["kind"] == "CXX_METHOD":
                # TODO: Add template args, currently blank
                if sub_item["name"] not in ("PCL_DEPRECATED"):
                    self.linelist.append(
                        f'.def("{sub_item["name"]}", py::overload_cast<>(&{self.name}::{sub_item["name"]}))'
                    )

    def handle_constructor(self):
        argument_type_list = []

        # generate argument type list
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
                elif sub_item["element_type"] in ("Float", "Int"):
                    argument_type_list.append(f'{sub_item["element_type"].lower()}')
                else:
                    argument_type_list.append(f'{sub_item["element_type"]}')
        argument_type_list = ",".join(argument_type_list)

        self.linelist.append(f".def(py::init<{argument_type_list}>())")

    def handle_class_template(self):
        flag = False

        # TODO: Use list based method, like in handle_struct_decl
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
        # TODO: update blacklist
        # blacklist = ["pcl/memory.h", "pcl/pcl_macros.h",]
        if self.name.startswith("pcl"):
            self.inclusion_list.append(self.name)

    def handle_final(self, filename, module_name):
        final = []
        # TODO: Inclusion list path fix needed
        # TODO: Currently commented, to be written later
        # for inclusion in self.inclusion_list:
        #     final.append(f"#include <{inclusion}>")
        final.append(f"#include <{filename}>")
        final.append("#include <pybind11/pybind11.h>")
        final.append("#include<pybind11/stl.h>")
        final.append("#include<pybind11/stl_bind.h>")
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


def generate(source):
    header_info = utils.read_json(filename=source)
    if header_info:
        bind_object = bind(header_info)
        # TODO: Remove hardcoded filename
        lines_to_write = bind_object.handle_final(
            filename="pcl/point_types.h", module_name="pcl"
        )
        return lines_to_write
    else:
        raise Exception("Empty json")


def main():
    args = utils.parse_arguments(script="generate")

    for source in args.files:
        source = utils.get_realpath(path=source)
        lines_to_write = generate(source)
        output_filepath = utils.get_output_path(
            source=source,
            output_dir=utils.join_path(args.pybind11_output_path, "pybind11-gen"),
            split_from="json",
            extension=".cpp",
        )
        utils.write_to_file(filename=output_filepath, linelist=lines_to_write)


if __name__ == "__main__":
    main()

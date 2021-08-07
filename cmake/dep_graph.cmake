###############################################################################
# Make a dependency graph dot file
function(MAKE_DEP_GRAPH)
    set(_dot_file "${PROJECT_BINARY_DIR}/pcl.dot")
    file(WRITE ${_dot_file} "digraph pcl {\n")
    foreach(_ss ${PCL_SUBSYSTEMS})
      if(NOT _ss STREQUAL "global_tests" AND
         NOT _ss STREQUAL "apps" AND
         NOT _ss STREQUAL "tools" AND
         NOT _ss STREQUAL "test" AND
         NOT _ss STREQUAL "python" AND
         NOT _ss STREQUAL "documentation")
        PCL_GET_SUBSYS_STATUS(_status ${_ss})
        if(_status)
            file(APPEND ${_dot_file}
                "  \"${_ss}\" [style=\"filled\" fillcolor=\"#008000\" shape=\"box\"];\n ")
        else()
            file(APPEND ${_dot_file}
                "  \"${_ss}\" [style=\"filled\" fillcolor=\"#D40000\" shape=\"box\"];\n ")
        endif()
        GET_IN_MAP(_deps PCL_SUBSYS_DEPS ${_ss})
        foreach(_dep ${_deps})
            file(APPEND ${_dot_file} "  \"${_ss}\" -> \"${_dep}\";\n")
        endforeach()
      endif()
    endforeach()

    #file(APPEND ${_dot_file}
    #    "  \"test\" [style=\"filled\" fillcolor=\"#A3A27C\" shape=\"box\"];\n ")
    file(APPEND ${_dot_file} "}\n")
endfunction(MAKE_DEP_GRAPH)


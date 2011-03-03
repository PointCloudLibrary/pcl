###############################################################################
# Turn a list into a string, with each item separated by spaces.
# _list List to stringify.
# _string Name of the destination variable.
macro(LIST_TO_STRING _list _string)
    set(${_string})
    foreach(_item ${_list})
        set(${_string} "${${_string}} ${_item}")
    endforeach(_item)
endmacro(LIST_TO_STRING)


###############################################################################
# Filter a list by a pattern.
# _list List to filter.
# _pattern The regular expression to filter by. See the if(... MATCHES ...)
#   expression in the CMake help.
# _output The name of the destination variable.
macro(FILTER_LIST _list _pattern _output)
    set(${_output})
    foreach(_item ${_list})
        if("${_item}" MATCHES ${_pattern})
            set(${_output} ${${_output}} ${_item})
        endif("${_item}" MATCHES ${_pattern})
    endforeach(_item)
endmacro(FILTER_LIST)


###############################################################################
# Find a pkg-config file and use it.
# _pkg The name of the package to search for.
# _required Set to "REQUIRED" to cause an error if the package is not found.
include(FindPkgConfig)
macro(GET_PKG_CONFIG_INFO _pkg _required)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(${_pkg}_PKG ${_required} ${_pkg})
        if(${_pkg}_PKG_CFLAGS_OTHER)
            LIST_TO_STRING(${_pkg}_CFLAGS "${${_pkg}_PKG_CFLAGS_OTHER}")
        else(${_pkg}_PKG_CFLAGS_OTHER)
            set(${_pkg}_CFLAGS "")
        endif(${_pkg}_PKG_CFLAGS_OTHER)
        set(${_pkg}_INCLUDE_DIRS ${${_pkg}_PKG_INCLUDE_DIRS})
        set(${_pkg}_LINK_LIBS ${${_pkg}_PKG_LIBRARIES})
        set(${_pkg}_LIBRARY_DIRS ${${_pkg}_PKG_LIBRARY_DIRS})
        if(${_pkg}_PKG_LDFLAGS_OTHER)
            LIST_TO_STRING(${_pkg}_LINK_FLAGS ${${_pkg}_PKG_LDFLAGS_OTHER})
        else(${_pkg}_PKG_LDFLAGS_OTHER)
            set(${_pkg}_LINK_FLAGS "")
        endif(${_pkg}_PKG_LDFLAGS_OTHER)
    else(PKG_CONFIG_FOUND)
        message(STATUS "Could not find pkg-config.")
        message(STATUS
            "You will need to set the following variables manually:")
        message(STATUS "${_pkg}_INCLUDE_DIRS ${_pkg}_CFLAGS_OTHER ${_pkg}_LINK_LIBS ${_pkg}_LIBRARY_DIRS ${_pkg}_LINK_FLAGS")
    endif(PKG_CONFIG_FOUND)
endmacro(GET_PKG_CONFIG_INFO)


###############################################################################
# Apply the results of a pkg-config search to the include and link directory
# settings.
# _pkg The name of the package who's settings should be applied.
macro(APPLY_PKG_CONFIG_DIRS _pkg)
    if(${_pkg}_INCLUDE_DIRS)
        include_directories(${${_pkg}_INCLUDE_DIRS})
    endif(${_pkg}_INCLUDE_DIRS)
    if(${_pkg}_LIBRARY_DIRS)
        link_directories(${${_pkg}_LIBRARY_DIRS})
    endif(${_pkg}_LIBRARY_DIRS)
endmacro(APPLY_PKG_CONFIG_DIRS)


###############################################################################
# Apply the results of a pkg-config search to a list of targets, setting the
# link flags and libraries to link to on each target based on the information
# from the pkg-config file.
# _pkg The name of the package who's settings should be applied.
# Extra arguments: The targets to apply to.
macro(APPLY_PKG_CONFIG_TO_TGTS _pkg)
    if(${_pkg}_LINK_FLAGS)
        foreach(_tgt ${ARGN})
            set_target_properties(${_tgt} PROPERTIES
                LINK_FLAGS "${${_pkg}_LINK_FLAGS}")
        endforeach(_tgt)
    endif(${_pkg}_LINK_FLAGS)
    if(${_pkg}_LINK_LIBS)
        foreach(_tgt ${ARGN})
            target_link_libraries(${_tgt} ${${_pkg}_LINK_LIBS})
        endforeach(_tgt)
    endif(${_pkg}_LINK_LIBS)
endmacro(APPLY_PKG_CONFIG_TO_TGTS)


###############################################################################
# Apply the results of a pkg-config search to a list of source files, setting
# the compile flags on each file based on the information from the pkg-config
# file.
# _pkg The name of the package who's settings should be applied.
# Extra arguments: The files to apply to.
macro(APPLY_PKG_CONFIG_TO_SRCS _pkg)
    if(${_pkg}_CFLAGS)
        set_source_files_properties(${ARGN}
            PROPERTIES COMPILE_FLAGS "${${_pkg}_CFLAGS}")
    endif(${_pkg}_CFLAGS)
endmacro(APPLY_PKG_CONFIG_TO_SRCS)


###############################################################################
# Pull the component parts out of the version number.
macro(DISSECT_VERSION)
    # Find version components
    string(REGEX REPLACE "^([0-9]+).*" "\\1"
        PCL_MAJOR_VERSION "${PCL_VERSION}")
    string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1"
        PCL_MINOR_VERSION "${PCL_VERSION}")
    string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+)" "\\1"
        PCL_REVISION_VERSION ${PCL_VERSION})
    string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.[0-9]+(.*)" "\\1"
        PCL_CANDIDATE_VERSION ${PCL_VERSION})
endmacro(DISSECT_VERSION)

###############################################################################
# Get the operating system information. Generally, CMake does a good job of
# this. Sometimes, though, it doesn't give enough information. This macro will
# distinguish between the UNIX variants. Otherwise, use the CMake variables
# such as WIN32 and APPLE and CYGWIN.
# Sets OS_IS_64BIT if the operating system is 64-bit.
# Sets LINUX if the operating system is Linux.
macro(GET_OS_INFO)
    string(REGEX MATCH "Linux" OS_IS_LINUX ${CMAKE_SYSTEM_NAME})
    if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
        set(OS_IS_64BIT TRUE)
    else(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
        set(OS_IS_64BIT FALSE)
    endif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
endmacro(GET_OS_INFO)


###############################################################################
# Set the destination directories for installing stuff.
# Sets LIB_INSTALL_DIR. Install libraries here.
# Sets BIN_INSTALL_DIR. Install binaries here.
# Sets INCLUDE_INSTALL_DIR. Install include files here, preferably in a
# subdirectory named after the library in question (e.g.
# "registration/blorgle.h")
macro(SET_INSTALL_DIRS)
    if(OS_IS_64BIT)
        set(LIB_INSTALL_DIR "lib64")
    else(OS_IS_64BIT)
        set(LIB_INSTALL_DIR "lib")
    endif(OS_IS_64BIT)
    set(INCLUDE_INSTALL_DIR
        "include/${PROJECT_NAME_LOWER}-${PCL_MAJOR_VERSION}.${PCL_MINOR_VERSION}")
endmacro(SET_INSTALL_DIRS)


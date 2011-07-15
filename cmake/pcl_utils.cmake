# cmake_parse_arguments support for CMake 2.8.2 and older
include (cmake/CMakeParseArguments.cmake)

###############################################################################
# Turn a list into a string, with each item separated by spaces.
# _string Name of the destination variable.
# _list List to stringify.
macro(LIST_TO_STRING _string _list)
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
# Prefix every item in a list.
# _output The name of the destination variable.
# _prefix The value to prepend.
# _list List to prefix.
macro(PREFIX_LIST _output _prefix _list)
    set(${_output})
    foreach(_item ${_list})
        list(APPEND ${_output} "${_prefix}${_item}")
    endforeach(_item)
endmacro(PREFIX_LIST)


###############################################################################
# Find a pkg-config file and use it.
# _pkg The name of the package to search for.
# _required Set to "REQUIRED" to cause an error if the package is not found.
include(FindPkgConfig)
macro(GET_PKG_CONFIG_INFO _pkg _required)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(${_pkg}_PKG ${_required} ${_pkg})
        if(${_pkg}_PKG_CFLAGS_OTHER)
            LIST_TO_STRING("${${_pkg}_PKG_CFLAGS_OTHER}" ${_pkg}_CFLAGS)
        else(${_pkg}_PKG_CFLAGS_OTHER)
            set(${_pkg}_CFLAGS "")
        endif(${_pkg}_PKG_CFLAGS_OTHER)
        set(${_pkg}_INCLUDE_DIRS ${${_pkg}_PKG_INCLUDE_DIRS})
        set(${_pkg}_LINK_LIBS ${${_pkg}_PKG_LIBRARIES})
        set(${_pkg}_LIBRARY_DIRS ${${_pkg}_PKG_LIBRARY_DIRS})
        if(${_pkg}_PKG_LDFLAGS_OTHER)
            LIST_TO_STRING("${${_pkg}_PKG_LDFLAGS_OTHER}" ${_pkg}_LINK_FLAGS)
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
        PCL_REVISION_VERSION "${PCL_VERSION}")
    string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.[0-9]+(.*)" "\\1"
        PCL_CANDIDATE_VERSION "${PCL_VERSION}")
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
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(OS_IS_64BIT TRUE)
    else(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(OS_IS_64BIT FALSE)
    endif(CMAKE_SIZEOF_VOID_P EQUAL 8)
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
        if(NOT APPLE)
            set(LIB_INSTALL_DIR "lib")
        else(NOT APPLE)
            set(LIB_INSTALL_DIR "lib")
        endif(NOT APPLE)
    else(OS_IS_64BIT)
        set(LIB_INSTALL_DIR "lib")
    endif(OS_IS_64BIT)
    set(INCLUDE_INSTALL_ROOT
        "include/${PROJECT_NAME_LOWER}-${PCL_MAJOR_VERSION}.${PCL_MINOR_VERSION}")
    set(INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_ROOT}/pcl")
    set(BIN_INSTALL_DIR "bin")
    set(PKGCFG_INSTALL_DIR "${LIB_INSTALL_DIR}/pkgconfig")
    set(CMAKE_INSTALL_DIR "share/${PROJECT_NAME_LOWER}-${PCL_MAJOR_VERSION}.${PCL_MINOR_VERSION}")
endmacro(SET_INSTALL_DIRS)


###############################################################################
# This macro processes a list of arguments into separate lists based on
# keywords found in the argument stream. For example:
# BUILDBLAG (misc_arg INCLUDEDIRS /usr/include LIBDIRS /usr/local/lib
#            LINKFLAGS -lthatawesomelib CFLAGS -DUSEAWESOMELIB SOURCES blag.c)
# Any other args found at the start of the stream will go into the variable
# specified in _other_args. Typically, you would take arguments to your macro
# as normal, then pass ${ARGN} to this macro to parse the dynamic-length
# arguments (so if ${_otherArgs} comes back non-empty, you've ignored something
# or the user has passed in some arguments without a keyword).
macro(PROCESS_ARGUMENTS _sources_args _include_dirs_args _lib_dirs_args
        _link_libs_args _link_flags_args _cflags_args _idl_args _other_args)
    set(${_sources_args})
    set(${_include_dirs_args})
    set(${_lib_dirs_args})
    set(${_link_libs_args})
    set(${_link_flags_args})
    set(${_cflags_args})
    set(${_idl_args})
    set(${_other_args})
    set(_current_dest ${_other_args})
    foreach(_arg ${ARGN})
        if(_arg STREQUAL "SOURCES")
            set(_current_dest ${_sources_args})
        elseif(_arg STREQUAL "INCLUDEDIRS")
            set(_current_dest ${_include_dirs_args})
        elseif(_arg STREQUAL "LIBDIRS")
            set(_current_dest ${_lib_dirs_args})
        elseif(_arg STREQUAL "LINKLIBS")
            set(_current_dest ${_link_libs_args})
        elseif(_arg STREQUAL "LINKFLAGS")
            set(_current_dest ${_link_flags_args})
        elseif(_arg STREQUAL "CFLAGS")
            set(_current_dest ${_cflags_args})
        elseif(_arg STREQUAL "IDL")
            set(_current_dest ${_idl_args})
        else(_arg STREQUAL "SOURCES")
            list(APPEND ${_current_dest} ${_arg})
        endif(_arg STREQUAL "SOURCES")
    endforeach(_arg)
endmacro(PROCESS_ARGUMENTS)


###############################################################################
# Set a value in a map.
# _map The map name.
# _key The key name.
# _value The value.
macro(SET_IN_MAP _map _key _value)
    set("${_map}_${_key}" "${_value}")
endmacro(SET_IN_MAP)


###############################################################################
# Set a value in a global, cached map.
# _map The map name.
# _key The key name.
# _value The value.
macro(SET_IN_GLOBAL_MAP _map _key _value)
    set("${_map}_${_key}" "${_value}" CACHE INTERNAL "Map value" FORCE)
endmacro(SET_IN_GLOBAL_MAP)


###############################################################################
# Get a value from a map.
# _dest The name of the variable to store the value in.
# _map The map name.
# _key The key name.
macro(GET_IN_MAP _dest _map _key)
    set(${_dest} ${${_map}_${_key}})
endmacro(GET_IN_MAP)


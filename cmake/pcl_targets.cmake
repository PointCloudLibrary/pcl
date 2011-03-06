include(${PROJECT_SOURCE_DIR}/cmake/pcl_utils.cmake)

###############################################################################
# Add an option to build a subsystem or not.
# _var The name of the variable to store the option in.
# _name The name of the option's target subsystem.
# _desc The description of the subsystem.
# _default The default value (ON or OFF)
# ARGN Any dependencies that this subsystem has.
macro(PCL_SUBSYS_OPTION _var _name _desc _default)
    # TODO: placeholder
    if(${_default})
        set(${_var} TRUE)
    else(${_default})
        set(${_var} FALSE)
    endif(${_default})
endmacro(PCL_SUBSYS_OPTION)


###############################################################################
# Add a library target.
# _name The library name.
# _component The part of PCL that this library belongs to.
# ARGN The source files for the library.
macro(PCL_ADD_LIBRARY _name _component)
    if(PCL_SHARED_LIBS)
        set(_lib_type "SHARED")
    else(PCL_SHARED_LIBS)
        set(_lib_type "STATIC")
    endif(PCL_SHARED_LIBS)
    add_library(${_name} ${_lib_type} ${ARGN})
    install(TARGETS ${_name} LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
        COMPONENT ${_component})
endmacro(PCL_ADD_LIBRARY)


###############################################################################
# Add an executable target.
# _name The executable name.
# _component The part of PCL that this library belongs to.
# ARGN the source files for the library.
macro(PCL_ADD_EXECUTABLE _name _component)
    add_executable(${_name} ${ARGN})
    set(PCL_EXECUTABLES ${PCL_EXECUTABLES} ${_name})
    install(TARGETS ${_name} RUNTIME DESTINATION ${BIN_INSTALL_DIR}
        COMPONENT ${_component})
endmacro(PCL_ADD_EXECUTABLE)


###############################################################################
# Make a pkg-config file for a library. Do not include general PCL stuff in the
# arguments; they will be added automaticaly.
# _name The library name. "pcl_" will be preprended to this.
# _component The part of PCL that this pkg-config file belongs to.
# _desc Description of the library.
# _ext_deps External dependencies, as a space-separated string of items.
# _int_deps Internal dependencies, as a space-separated string of items.
# _cflags Compiler flags necessary to build with the library.
# _lib_flags Linker flags necessary to link to the library.
macro(PCL_MAKE_PKGCONFIG _name _component _desc _ext_deps _int_deps _cflags
        _lib_flags)
    set(PKG_NAME ${_name})
    set(PKG_DESC ${_desc})
    set(PKG_CFLAGS ${_cflags})
    set(PKG_LIBFLAGS ${_lib_flags})
    set(PKG_EXTERNAL_DEPS ${_ext_deps})
    set(PKG_INTERNAL_DEPS "")
    foreach(_dep ${_int_deps})
        set(PKG_INTERNAL_DEPS "${PKG_INTERNAL_DEPS} -l${_dep}")
    endforeach(_dep)

    set(_pc_file ${CMAKE_CURRENT_BINARY_DIR}/${_name}.pc)
    configure_file(${PROJECT_SOURCE_DIR}/cmake/pkgconfig.cmake.in ${_pc_file}
        @ONLY)
    install(FILES ${_pc_file} DESTINATION ${PKGCFG_INSTALL_DIR}
        COMPONENT ${_component})
endmacro(PCL_MAKE_PKGCONFIG)


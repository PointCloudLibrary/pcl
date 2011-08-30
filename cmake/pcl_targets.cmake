include(${PROJECT_SOURCE_DIR}/cmake/pcl_utils.cmake)

###############################################################################
# Add an option to build a subsystem or not.
# _var The name of the variable to store the option in.
# _name The name of the option's target subsystem.
# _desc The description of the subsystem.
# _default The default value (TRUE or FALSE)
# ARGV5 The reason for disabling if the default is FALSE.
macro(PCL_SUBSYS_OPTION _var _name _desc _default)
    set(_opt_name "BUILD_${_name}")
    option(${_opt_name} ${_desc} ${_default})
    if(NOT ${_default} AND NOT ${_opt_name})
        set(${_var} FALSE)
        if(${ARGC} GREATER 4)
            set(_reason ${ARGV4})
        else(${ARGC} GREATER 4)
            set(_reason "Disabled by default.")
        endif(${ARGC} GREATER 4)
        PCL_SET_SUBSYS_STATUS(${_name} FALSE ${_reason})
    elseif(NOT ${_opt_name})
        set(${_var} FALSE)
        PCL_SET_SUBSYS_STATUS(${_name} FALSE "Disabled manually.")
    else(NOT ${_default} AND NOT ${_opt_name})
        set(${_var} TRUE)
        PCL_SET_SUBSYS_STATUS(${_name} TRUE)
    endif(NOT ${_default} AND NOT ${_opt_name})
    PCL_ADD_SUBSYSTEM(${_name} ${_desc})
endmacro(PCL_SUBSYS_OPTION)


###############################################################################
# Make one subsystem depend on one or more other subsystems, and disable it if
# they are not being built.
# _var The cumulative build variable. This will be set to FALSE if the
#   dependencies are not met.
# _name The name of the subsystem.
# ARGN The subsystems to depend on.
macro(PCL_SUBSYS_DEPEND _var _name)
    if(${ARGC} GREATER 2)
        SET_IN_GLOBAL_MAP(PCL_SUBSYS_DEPS ${_name} "${ARGN}")
    endif(${ARGC} GREATER 2)
    if(${_var})
        foreach(_dep ${ARGN})
            PCL_GET_SUBSYS_STATUS(_status ${_dep})
            if(NOT _status)
                set(${_var} FALSE)
                PCL_SET_SUBSYS_STATUS(${_name} FALSE "Requires ${_dep}")
            else(NOT _status)
                PCL_GET_SUBSYS_INCLUDE_DIR(_include_dir ${_dep})
                include_directories(${PROJECT_SOURCE_DIR}/${_include_dir}/include)
            endif(NOT _status)
        endforeach(_dep)
    endif(${_var})
endmacro(PCL_SUBSYS_DEPEND)


###############################################################################
# Add a set of include files to install.
# _component The part of PCL that the install files belong to.
# _subdir The sub-directory for these include files.
# ARGN The include files.
macro(PCL_ADD_INCLUDES _component _subdir)
    install(FILES ${ARGN} DESTINATION ${INCLUDE_INSTALL_DIR}/${_subdir}
        COMPONENT ${_component})
endmacro(PCL_ADD_INCLUDES)


###############################################################################
# Add a library target.
# _name The library name.
# _component The part of PCL that this library belongs to.
# ARGN The source files for the library.
macro(PCL_ADD_LIBRARY _name _component)
    add_library(${_name} ${PCL_LIB_TYPE} ${ARGN})
    # must link explicitly against boost.
    target_link_libraries(${_name} ${Boost_LIBRARIES})
    #
    # Only link if needed
    if(WIN32 AND MSVC)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
    elseif(__COMPILER_PATHSCALE)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    else()
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
    endif()
    #
    set_target_properties(${_name} PROPERTIES
        VERSION ${PCL_VERSION}
        SOVERSION ${PCL_MAJOR_VERSION}.${PCL_MINOR_VERSION}
        DEFINE_SYMBOL "PCLAPI_EXPORTS")
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${_name} PROPERTIES FOLDER "Libraries")
    endif(USE_PROJECT_FOLDERS)

    install(TARGETS ${_name} EXPORT pcl
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ${_component}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component})
    install(EXPORT pcl DESTINATION ${LIB_INSTALL_DIR}/pcl FILE PCLDepends.cmake)
endmacro(PCL_ADD_LIBRARY)


###############################################################################
# Add a cuda library target.
# _name The library name.
# _component The part of PCL that this library belongs to.
# ARGN The source files for the library.
macro(PCL_CUDA_ADD_LIBRARY _name _component)
    if(PCL_SHARED_LIBS)
        # to overcome a limitation in cuda_add_library, we add manually PCLAPI_EXPORTS macro
        cuda_add_library(${_name} ${PCL_LIB_TYPE} ${ARGN} OPTIONS -DPCLAPI_EXPORTS)
    else(PCL_SHARED_LIBS)
        cuda_add_library(${_name} ${PCL_LIB_TYPE} ${ARGN})
    endif(PCL_SHARED_LIBS)
    
    # must link explicitly against boost.
    target_link_libraries(${_name} ${Boost_LIBRARIES})
    #
    # Only link if needed
    if(WIN32 AND MSVC)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
    else()
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
    endif()
    #
    set_target_properties(${_name} PROPERTIES
        VERSION ${PCL_VERSION}
        SOVERSION ${PCL_MAJOR_VERSION}
        DEFINE_SYMBOL "PCLAPI_EXPORTS")
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${_name} PROPERTIES FOLDER "Libraries")
    endif(USE_PROJECT_FOLDERS)

    install(TARGETS ${_name} EXPORT pcl
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ${_component}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component})
    install(EXPORT pcl DESTINATION ${LIB_INSTALL_DIR}/pcl FILE PCLDepends.cmake)
endmacro(PCL_CUDA_ADD_LIBRARY)


###############################################################################
# Add an executable target.
# _name The executable name.
# _component The part of PCL that this library belongs to.
# ARGN the source files for the library.
macro(PCL_ADD_EXECUTABLE _name _component)
    add_executable(${_name} ${ARGN})
    # must link explicitly against boost.
    if(UNIX)
      target_link_libraries(${_name} ${Boost_LIBRARIES} pthread)
    else(UNIX)
      target_link_libraries(${_name} ${Boost_LIBRARIES})
    endif(UNIX)
    #
    # Only link if needed
    if(WIN32 AND MSVC)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF DEBUG_OUTPUT_NAME ${_name}${CMAKE_DEBUG_POSTFIX})
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
    elseif(__COMPILER_PATHSCALE)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    else()
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
    endif()
    #
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${_name} PROPERTIES FOLDER "Tools and demos")
    endif(USE_PROJECT_FOLDERS)

    set(PCL_EXECUTABLES ${PCL_EXECUTABLES} ${_name})
    install(TARGETS ${_name} RUNTIME DESTINATION ${BIN_INSTALL_DIR}
        COMPONENT ${_component})
endmacro(PCL_ADD_EXECUTABLE)


###############################################################################
# Add an executable target.
# _name The executable name.
# _component The part of PCL that this library belongs to.
# ARGN the source files for the library.
macro(PCL_CUDA_ADD_EXECUTABLE _name _component)
    cuda_add_executable(${_name} ${ARGN})
    # must link explicitly against boost.
    target_link_libraries(${_name} ${Boost_LIBRARIES})
    #
    # Only link if needed
    if(WIN32 AND MSVC)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF DEBUG_OUTPUT_NAME ${_name}${CMAKE_DEBUG_POSTFIX})
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
    else()
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
    endif()
    #
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${_name} PROPERTIES FOLDER "Tools and demos")
    endif(USE_PROJECT_FOLDERS)
	
    set(PCL_EXECUTABLES ${PCL_EXECUTABLES} ${_name})
    install(TARGETS ${_name} RUNTIME DESTINATION ${BIN_INSTALL_DIR}
        COMPONENT ${_component})
endmacro(PCL_CUDA_ADD_EXECUTABLE)

###############################################################################
# Add a test target.
# _name The test name.
# _exename The exe name.
# ARGN :
#    FILES the source files for the test
#    ARGUMENTS Arguments for test executable
#    LINK_WITH link test executable with libraries
macro(PCL_ADD_TEST _name _exename)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs FILES ARGUMENTS LINK_WITH)
    cmake_parse_arguments(PCL_ADD_TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
    add_executable(${_exename} ${PCL_ADD_TEST_FILES})
    PCL_ADD_OPENMP_FLAGS(${_exename})
    target_link_libraries(${_exename} ${GTEST_BOTH_LIBRARIES} ${PCL_ADD_TEST_LINK_WITH})
    #
    # Only link if needed
    if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      set_target_properties(${_exename} PROPERTIES LINK_FLAGS -Wl)
      target_link_libraries(${_exename} pthread)
    elseif(UNIX)
      set_target_properties(${_exename} PROPERTIES LINK_FLAGS -Wl,--as-needed)
      # GTest >= 1.5 requires pthread and CMake's 2.8.4 FindGTest is broken
      target_link_libraries(${_exename} pthread)
    elseif(WIN32)
      set_target_properties(${_exename} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
    endif()
    # 
    PCL_LINK_OPENMP(${_exename})
    # must link explicitly against boost only on Windows
    target_link_libraries(${_exename} ${Boost_LIBRARIES})
    #
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${_exename} PROPERTIES FOLDER "Tests")
    endif(USE_PROJECT_FOLDERS)

    if(${CMAKE_VERSION} VERSION_LESS 2.8.4)
      add_test(${_name} ${_exename} ${PCL_ADD_TEST_ARGUMENTS})
    else(${CMAKE_VERSION} VERSION_LESS 2.8.4)
      add_test(NAME ${_name} COMMAND ${_exename} ${PCL_ADD_TEST_ARGUMENTS})
    endif(${CMAKE_VERSION} VERSION_LESS 2.8.4)
endmacro(PCL_ADD_TEST)


###############################################################################
# Add compile flags to a target (because CMake doesn't provide something so
# common itself).
# _name The target name.
# _flags The new compile flags to be added, as a string.
macro(PCL_ADD_CFLAGS _name _flags)
    get_target_property(_current_flags ${_name} COMPILE_FLAGS)
    if(NOT _current_flags)
        set_target_properties(${_name} PROPERTIES COMPILE_FLAGS ${_flags})
    else(NOT _current_flags)
        set_target_properties(${_name} PROPERTIES
            COMPILE_FLAGS "${_current_flags} ${_flags}")
    endif(NOT _current_flags)
endmacro(PCL_ADD_CFLAGS)


###############################################################################
# Add link flags to a target (because CMake doesn't provide something so
# common itself).
# _name The target name.
# _flags The new link flags to be added, as a string.
macro(PCL_ADD_LINKFLAGS _name _flags)
    get_target_property(_current_flags ${_name} LINK_FLAGS)
    if(NOT _current_flags)
        set_target_properties(${_name} PROPERTIES LINK_FLAGS ${_flags})
    else(NOT _current_flags)
        set_target_properties(${_name} PROPERTIES
            LINK_FLAGS "${_current_flags} ${_flags}")
    endif(NOT _current_flags)
endmacro(PCL_ADD_LINKFLAGS)


###############################################################################
# Make a pkg-config file for a library. Do not include general PCL stuff in the
# arguments; they will be added automaticaly.
# _name The library name. "pcl_" will be preprended to this.
# _component The part of PCL that this pkg-config file belongs to.
# _desc Description of the library.
# _pcl_deps External dependencies to pcl libs, as a list. (will get mangled to external pkg-config name)
# _ext_deps External dependencies, as a list.
# _int_deps Internal dependencies, as a list.
# _cflags Compiler flags necessary to build with the library.
# _lib_flags Linker flags necessary to link to the library.
macro(PCL_MAKE_PKGCONFIG _name _component _desc _pcl_deps _ext_deps _int_deps _cflags
        _lib_flags)
    set(PKG_NAME ${_name})
    set(PKG_DESC ${_desc})
    set(PKG_CFLAGS ${_cflags})
    set(PKG_LIBFLAGS ${_lib_flags})
    LIST_TO_STRING(_ext_deps_str "${_ext_deps}")
    set(PKG_EXTERNAL_DEPS ${_ext_deps_str})
    foreach(_dep ${_pcl_deps})
      set(PKG_EXTERNAL_DEPS "${PKG_EXTERNAL_DEPS} pcl_${_dep}-${PCL_MAJOR_VERSION}.${PCL_MINOR_VERSION}")
    endforeach(_dep)
    set(PKG_INTERNAL_DEPS "")
    foreach(_dep ${_int_deps})
        set(PKG_INTERNAL_DEPS "${PKG_INTERNAL_DEPS} -l${_dep}")
    endforeach(_dep)

    set(_pc_file ${CMAKE_CURRENT_BINARY_DIR}/${_name}-${PCL_MAJOR_VERSION}.${PCL_MINOR_VERSION}.pc)
    configure_file(${PROJECT_SOURCE_DIR}/cmake/pkgconfig.cmake.in ${_pc_file}
        @ONLY)
    install(FILES ${_pc_file} DESTINATION ${PKGCFG_INSTALL_DIR}
        COMPONENT ${_component})
endmacro(PCL_MAKE_PKGCONFIG)


###############################################################################
# PRIVATE

###############################################################################
# Reset the subsystem status map.
macro(PCL_RESET_MAPS)
    set(PCL_SUBSYS_STATUS "" CACHE INTERNAL
        "To build or not to build, that is the question." FORCE)
    set(PCL_SUBSYS_REASONS "" CACHE INTERNAL
        "But why?" FORCE)
    set(PCL_SUBSYS_DEPS "" CACHE INTERNAL "A depends on B and C." FORCE)
    set(PCL_SUBSYSTEMS "" CACHE INTERNAL "Internal list of subsystems" FORCE)
    set(PCL_SUBSYS_DESC "" CACHE INTERNAL "Subsystem descriptions" FORCE)
endmacro(PCL_RESET_MAPS)


###############################################################################
# Register a subsystem.
# _name Subsystem name.
# _desc Description of the subsystem
macro(PCL_ADD_SUBSYSTEM _name _desc)
    set(_temp ${PCL_SUBSYSTEMS})
    list(APPEND _temp ${_name})
    set(PCL_SUBSYSTEMS ${_temp} CACHE INTERNAL "Internal list of subsystems"
        FORCE)
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_DESC ${_name} ${_desc})
endmacro(PCL_ADD_SUBSYSTEM)


###############################################################################
# Set the status of a subsystem.
# _name Subsystem name.
# _status TRUE if being built, FALSE otherwise.
# ARGN[0] Reason for not building.
macro(PCL_SET_SUBSYS_STATUS _name _status)
    if(${ARGC} EQUAL 3)
        set(_reason ${ARGV2})
    else(${ARGC} EQUAL 3)
        set(_reason "No reason")
    endif(${ARGC} EQUAL 3)
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_STATUS ${_name} ${_status})
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_REASONS ${_name} ${_reason})
endmacro(PCL_SET_SUBSYS_STATUS)


###############################################################################
# Get the status of a subsystem
# _var Destination variable.
# _name Name of the subsystem.
macro(PCL_GET_SUBSYS_STATUS _var _name)
    GET_IN_MAP(${_var} PCL_SUBSYS_STATUS ${_name})
endmacro(PCL_GET_SUBSYS_STATUS)


###############################################################################
# Set the include directory name of a subsystem.
# _name Subsystem name.
# _includedir Name of subdirectory for includes 
# ARGN[0] Reason for not building.
macro(PCL_SET_SUBSYS_INCLUDE_DIR _name _includedir)
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_INCLUDE ${_name} ${_includedir})
endmacro(PCL_SET_SUBSYS_INCLUDE_DIR)


###############################################################################
# Get the include directory name of a subsystem - return _name if not set
# _var Destination variable.
# _name Name of the subsystem.
macro(PCL_GET_SUBSYS_INCLUDE_DIR _var _name)
    GET_IN_MAP(${_var} PCL_SUBSYS_INCLUDE ${_name})
    if(NOT ${_var})
      set (${_var} ${_name})
    endif(NOT ${_var})
endmacro(PCL_GET_SUBSYS_INCLUDE_DIR)


###############################################################################
# Write a report on the build/not-build status of the subsystems
macro(PCL_WRITE_STATUS_REPORT)
    message(STATUS "The following subsystems will be built:")
    foreach(_ss ${PCL_SUBSYSTEMS})
        PCL_GET_SUBSYS_STATUS(_status ${_ss})
        if(_status)
            message(STATUS "  ${_ss}")
        endif(_status)
    endforeach(_ss)

    message(STATUS "The following subsystems will not be built:")
    foreach(_ss ${PCL_SUBSYSTEMS})
        PCL_GET_SUBSYS_STATUS(_status ${_ss})
        if(NOT _status)
            GET_IN_MAP(_reason PCL_SUBSYS_REASONS ${_ss})
            message(STATUS "  ${_ss}: ${_reason}")
        endif(NOT _status)
    endforeach(_ss)
endmacro(PCL_WRITE_STATUS_REPORT)


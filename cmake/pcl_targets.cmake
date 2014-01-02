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
    PCL_GET_SUBSYS_HYPERSTATUS(subsys_status ${_name})
    if(NOT ("${subsys_status}" STREQUAL "AUTO_OFF"))
      option(${_opt_name} ${_desc} ${_default})
      if((NOT ${_default} AND NOT ${_opt_name}) OR ("${_default}" STREQUAL "AUTO_OFF"))
        set(${_var} FALSE)
        if(${ARGC} GREATER 4)
          set(_reason ${ARGV4})
        else(${ARGC} GREATER 4)
          set(_reason "Disabled by default.")
        endif(${ARGC} GREATER 4)
        PCL_SET_SUBSYS_STATUS(${_name} FALSE ${_reason})
        PCL_DISABLE_DEPENDIES(${_name})
      elseif(NOT ${_opt_name})
        set(${_var} FALSE)
        PCL_SET_SUBSYS_STATUS(${_name} FALSE "Disabled manually.")
        PCL_DISABLE_DEPENDIES(${_name})
      else(NOT ${_default} AND NOT ${_opt_name})
        set(${_var} TRUE)
        PCL_SET_SUBSYS_STATUS(${_name} TRUE)
        PCL_ENABLE_DEPENDIES(${_name})
      endif((NOT ${_default} AND NOT ${_opt_name}) OR ("${_default}" STREQUAL "AUTO_OFF"))
    endif(NOT ("${subsys_status}" STREQUAL "AUTO_OFF"))
    PCL_ADD_SUBSYSTEM(${_name} ${_desc})
endmacro(PCL_SUBSYS_OPTION)

###############################################################################
# Add an option to build a subsystem or not.
# _var The name of the variable to store the option in.
# _parent The name of the parent subsystem
# _name The name of the option's target subsubsystem.
# _desc The description of the subsubsystem.
# _default The default value (TRUE or FALSE)
# ARGV5 The reason for disabling if the default is FALSE.
macro(PCL_SUBSUBSYS_OPTION _var _parent _name _desc _default)
  set(_opt_name "BUILD_${_parent}_${_name}")
  PCL_GET_SUBSYS_HYPERSTATUS(parent_status ${_parent})
  if(NOT ("${parent_status}" STREQUAL "AUTO_OFF") AND NOT ("${parent_status}" STREQUAL "OFF"))
    PCL_GET_SUBSYS_HYPERSTATUS(subsys_status ${_parent}_${_name})
    if(NOT ("${subsys_status}" STREQUAL "AUTO_OFF"))
      option(${_opt_name} ${_desc} ${_default})
      if((NOT ${_default} AND NOT ${_opt_name}) OR ("${_default}" STREQUAL "AUTO_OFF"))
        set(${_var} FALSE)
        if(${ARGC} GREATER 5)
          set(_reason ${ARGV5})
        else(${ARGC} GREATER 5)
          set(_reason "Disabled by default.")
        endif(${ARGC} GREATER 5)
        PCL_SET_SUBSYS_STATUS(${_parent}_${_name} FALSE ${_reason})
        PCL_DISABLE_DEPENDIES(${_parent}_${_name})
      elseif(NOT ${_opt_name})
        set(${_var} FALSE)
        PCL_SET_SUBSYS_STATUS(${_parent}_${_name} FALSE "Disabled manually.")
        PCL_DISABLE_DEPENDIES(${_parent}_${_name})
      else(NOT ${_default} AND NOT ${_opt_name})
        set(${_var} TRUE)
        PCL_SET_SUBSYS_STATUS(${_parent}_${_name} TRUE)
        PCL_ENABLE_DEPENDIES(${_parent}_${_name})
      endif((NOT ${_default} AND NOT ${_opt_name}) OR ("${_default}" STREQUAL "AUTO_OFF"))
    endif(NOT ("${subsys_status}" STREQUAL "AUTO_OFF"))
  endif(NOT ("${parent_status}" STREQUAL "AUTO_OFF") AND NOT ("${parent_status}" STREQUAL "OFF"))
  PCL_ADD_SUBSUBSYSTEM(${_parent} ${_name} ${_desc})
endmacro(PCL_SUBSUBSYS_OPTION)

###############################################################################
# Make one subsystem depend on one or more other subsystems, and disable it if
# they are not being built.
# _var The cumulative build variable. This will be set to FALSE if the
#   dependencies are not met.
# _name The name of the subsystem.
# ARGN The subsystems and external libraries to depend on.
macro(PCL_SUBSYS_DEPEND _var _name)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs DEPS EXT_DEPS OPT_DEPS)
    cmake_parse_arguments(SUBSYS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
    if(SUBSYS_DEPS)
        SET_IN_GLOBAL_MAP(PCL_SUBSYS_DEPS ${_name} "${SUBSYS_DEPS}")
    endif(SUBSYS_DEPS)
    if(SUBSYS_EXT_DEPS)
        SET_IN_GLOBAL_MAP(PCL_SUBSYS_EXT_DEPS ${_name} "${SUBSYS_EXT_DEPS}")
    endif(SUBSYS_EXT_DEPS)
    if(SUBSYS_OPT_DEPS)
        SET_IN_GLOBAL_MAP(PCL_SUBSYS_OPT_DEPS ${_name} "${SUBSYS_OPT_DEPS}")
    endif(SUBSYS_OPT_DEPS)
    GET_IN_MAP(subsys_status PCL_SUBSYS_HYPERSTATUS ${_name})
    if(${_var} AND (NOT ("${subsys_status}" STREQUAL "AUTO_OFF")))
        if(SUBSYS_DEPS)
        foreach(_dep ${SUBSYS_DEPS})
            PCL_GET_SUBSYS_STATUS(_status ${_dep})
            if(NOT _status)
                set(${_var} FALSE)
                PCL_SET_SUBSYS_STATUS(${_name} FALSE "Requires ${_dep}.")
            else(NOT _status)
                PCL_GET_SUBSYS_INCLUDE_DIR(_include_dir ${_dep})
                include_directories(${PROJECT_SOURCE_DIR}/${_include_dir}/include)
            endif(NOT _status)
        endforeach(_dep)
        endif(SUBSYS_DEPS)
        if(SUBSYS_EXT_DEPS)
        foreach(_dep ${SUBSYS_EXT_DEPS})
            string(TOUPPER "${_dep}_found" EXT_DEP_FOUND)
            if(NOT ${EXT_DEP_FOUND} OR (NOT ("${EXT_DEP_FOUND}" STREQUAL "TRUE")))
                set(${_var} FALSE)
                PCL_SET_SUBSYS_STATUS(${_name} FALSE "Requires external library ${_dep}.")
            endif(NOT ${EXT_DEP_FOUND} OR (NOT ("${EXT_DEP_FOUND}" STREQUAL "TRUE")))
        endforeach(_dep)
        endif(SUBSYS_EXT_DEPS)
    endif(${_var} AND (NOT ("${subsys_status}" STREQUAL "AUTO_OFF")))
endmacro(PCL_SUBSYS_DEPEND)

###############################################################################
# Make one subsystem depend on one or more other subsystems, and disable it if
# they are not being built.
# _var The cumulative build variable. This will be set to FALSE if the
#   dependencies are not met.
# _parent The parent subsystem name.
# _name The name of the subsubsystem.
# ARGN The subsystems and external libraries to depend on.
macro(PCL_SUBSUBSYS_DEPEND _var _parent _name)
    set(options)
    set(parentArg)
    set(nameArg)
    set(multiValueArgs DEPS EXT_DEPS OPT_DEPS)
    cmake_parse_arguments(SUBSYS "${options}" "${parentArg}" "${nameArg}" "${multiValueArgs}" ${ARGN} )
    if(SUBSUBSYS_DEPS)
        SET_IN_GLOBAL_MAP(PCL_SUBSYS_DEPS ${_parent}_${_name} "${SUBSUBSYS_DEPS}")
    endif(SUBSUBSYS_DEPS)
    if(SUBSUBSYS_EXT_DEPS)
        SET_IN_GLOBAL_MAP(PCL_SUBSYS_EXT_DEPS ${_parent}_${_name} "${SUBSUBSYS_EXT_DEPS}")
    endif(SUBSUBSYS_EXT_DEPS)
    if(SUBSUBSYS_OPT_DEPS)
        SET_IN_GLOBAL_MAP(PCL_SUBSYS_OPT_DEPS ${_parent}_${_name} "${SUBSUBSYS_OPT_DEPS}")
    endif(SUBSUBSYS_OPT_DEPS)
    GET_IN_MAP(subsys_status PCL_SUBSYS_HYPERSTATUS ${_parent}_${_name})
    if(${_var} AND (NOT ("${subsys_status}" STREQUAL "AUTO_OFF")))
        if(SUBSUBSYS_DEPS)
        foreach(_dep ${SUBSUBSYS_DEPS})
            PCL_GET_SUBSYS_STATUS(_status ${_dep})
            if(NOT _status)
                set(${_var} FALSE)
                PCL_SET_SUBSYS_STATUS(${_parent}_${_name} FALSE "Requires ${_dep}.")
            else(NOT _status)
                PCL_GET_SUBSYS_INCLUDE_DIR(_include_dir ${_dep})
                include_directories(${PROJECT_SOURCE_DIR}/${_include_dir}/include)
            endif(NOT _status)
        endforeach(_dep)
        endif(SUBSUBSYS_DEPS)
        if(SUBSUBSYS_EXT_DEPS)
        foreach(_dep ${SUBSUBSYS_EXT_DEPS})
            string(TOUPPER "${_dep}_found" EXT_DEP_FOUND)
            if(NOT ${EXT_DEP_FOUND} OR (NOT ("${EXT_DEP_FOUND}" STREQUAL "TRUE")))
                set(${_var} FALSE)
                PCL_SET_SUBSYS_STATUS(${_parent}_${_name} FALSE "Requires external library ${_dep}.")
            endif(NOT ${EXT_DEP_FOUND} OR (NOT ("${EXT_DEP_FOUND}" STREQUAL "TRUE")))
        endforeach(_dep)
        endif(SUBSUBSYS_EXT_DEPS)
    endif(${_var} AND (NOT ("${subsys_status}" STREQUAL "AUTO_OFF")))
endmacro(PCL_SUBSUBSYS_DEPEND)

###############################################################################
# Add a set of include files to install.
# _component The part of PCL that the install files belong to.
# _subdir The sub-directory for these include files.
# ARGN The include files.
macro(PCL_ADD_INCLUDES _component _subdir)
    install(FILES ${ARGN} DESTINATION ${INCLUDE_INSTALL_DIR}/${_subdir}
        COMPONENT pcl_${_component})
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
    if((UNIX AND NOT ANDROID) OR MINGW)
      target_link_libraries(${_name} m)
    endif()

    if (MINGW)
      target_link_libraries(${_name} gomp)
    endif()
	
	if(MSVC90 OR MSVC10)
	  target_link_libraries(${_name} delayimp.lib)  # because delay load is enabled for openmp.dll
	endif()
    #
    # Only link if needed
    if(WIN32 AND MSVC)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
      endif()
    elseif(__COMPILER_PATHSCALE)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    elseif(CMAKE_COMPILER_IS_GNUCXX AND MINGW)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS "-Wl,--allow-multiple-definition -Wl,--as-needed")
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

    install(TARGETS ${_name}
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT pcl_${_component}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT pcl_${_component}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT pcl_${_component})

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
      if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
      endif()
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

    install(TARGETS ${_name}
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT pcl_${_component}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT pcl_${_component}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT pcl_${_component})
endmacro(PCL_CUDA_ADD_LIBRARY)


###############################################################################
# Add an executable target.
# _name The executable name.
# _component The part of PCL that this library belongs to.
# ARGN the source files for the library.
macro(PCL_ADD_EXECUTABLE _name _component)
    add_executable(${_name} ${ARGN})
    # must link explicitly against boost.
    if(UNIX AND NOT ANDROID)
      target_link_libraries(${_name} ${Boost_LIBRARIES} pthread m ${CLANG_LIBRARIES})
    else()
      target_link_libraries(${_name} ${Boost_LIBRARIES})
    endif()
    #
    # Only link if needed
    if(WIN32 AND MSVC)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF
                                                DEBUG_OUTPUT_NAME ${_name}${CMAKE_DEBUG_POSTFIX}
                                                RELEASE_OUTPUT_NAME ${_name}${CMAKE_RELEASE_POSTFIX})
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
      endif()
    elseif(__COMPILER_PATHSCALE)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    elseif(CMAKE_COMPILER_IS_GNUCXX AND MINGW)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS "-Wl,--allow-multiple-definition -Wl,--as-needed")
    else()
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
    endif()
    #
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${_name} PROPERTIES FOLDER "Tools and demos")
    endif(USE_PROJECT_FOLDERS)

    set(PCL_EXECUTABLES ${PCL_EXECUTABLES} ${_name})
    install(TARGETS ${_name} RUNTIME DESTINATION ${BIN_INSTALL_DIR}
        COMPONENT pcl_${_component})
endmacro(PCL_ADD_EXECUTABLE)

###############################################################################
# Add an executable target as a bundle when available and required
# _name The executable name.
# _component The part of PCL that this library belongs to.
# _bundle 
# ARGN the source files for the library.
macro(PCL_ADD_EXECUTABLE_OPT_BUNDLE _name _component)
if(APPLE AND VTK_USE_COCOA)
    add_executable(${_name} MACOSX_BUNDLE ${ARGN})
else(APPLE AND VTK_USE_COCOA)
    add_executable(${_name} ${ARGN})
endif(APPLE AND VTK_USE_COCOA)

    # must link explicitly against boost.
    if(UNIX AND NOT ANDROID)
      target_link_libraries(${_name} ${Boost_LIBRARIES} pthread)
    else()
      target_link_libraries(${_name} ${Boost_LIBRARIES})
    endif()
    #
    # Only link if needed
    if(WIN32 AND MSVC)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF
                                                DEBUG_OUTPUT_NAME ${_name}${CMAKE_DEBUG_POSTFIX}
                                                RELEASE_OUTPUT_NAME ${_name}${CMAKE_RELEASE_POSTFIX})
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
      endif()
    elseif(__COMPILER_PATHSCALE)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    elseif(CMAKE_COMPILER_IS_GNUCXX AND MINGW)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS "-Wl,--allow-multiple-definition -Wl,--as-needed")
    else()
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
    endif()
    #
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${_name} PROPERTIES FOLDER "Tools and demos")
    endif(USE_PROJECT_FOLDERS)

    set(PCL_EXECUTABLES ${PCL_EXECUTABLES} ${_name})
#    message(STATUS "COMMAND ${CMAKE_COMMAND} -E create_symlink \"${_name}.app/Contents/MacOS/${_name}\" \"${_name}\"")
if(APPLE AND VTK_USE_COCOA)
#     add_custom_command(TARGET ${_name}
#                         POST_BUILD
#                         COMMAND ${CMAKE_COMMAND} -E create_symlink ${PCL_OUTPUT_BIN_DIR}/${_name}.app/Contents/MacOS/${_name} ${PCL_OUTPUT_BIN_DIR}/${_name}
# #			WORKING_DIRECTORY 
#                         COMMENT "Creating an alias for ${_name}.app to ${_name}")
    install(TARGETS ${_name} BUNDLE DESTINATION ${BIN_INSTALL_DIR} COMPONENT pcl_${_component})
else(APPLE AND VTK_USE_COCOA)
    install(TARGETS ${_name} RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT pcl_${_component})
endif(APPLE AND VTK_USE_COCOA)
endmacro(PCL_ADD_EXECUTABLE_OPT_BUNDLE)


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
      set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF
                                                DEBUG_OUTPUT_NAME ${_name}${CMAKE_DEBUG_POSTFIX}
                                                RELEASE_OUTPUT_NAME ${_name}${CMAKE_RELEASE_POSTFIX})
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
      endif()
    else()
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
    endif()
    #
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${_name} PROPERTIES FOLDER "Tools and demos")
    endif(USE_PROJECT_FOLDERS)
  
    set(PCL_EXECUTABLES ${PCL_EXECUTABLES} ${_name})
    install(TARGETS ${_name} RUNTIME DESTINATION ${BIN_INSTALL_DIR}
        COMPONENT pcl_${_component})
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
    if(NOT WIN32)
      set_target_properties(${_exename} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    endif(NOT WIN32)
    #target_link_libraries(${_exename} ${GTEST_BOTH_LIBRARIES} ${PCL_ADD_TEST_LINK_WITH})
    target_link_libraries(${_exename} ${PCL_ADD_TEST_LINK_WITH} ${CLANG_LIBRARIES})
    #
    # Only link if needed
    if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set_target_properties(${_exename} PROPERTIES LINK_FLAGS -Wl)
      endif()
      target_link_libraries(${_exename} pthread)
    elseif(UNIX AND NOT ANDROID)
      set_target_properties(${_exename} PROPERTIES LINK_FLAGS -Wl,--as-needed)
      # GTest >= 1.5 requires pthread and CMake's 2.8.4 FindGTest is broken
      target_link_libraries(${_exename} pthread)
    elseif(CMAKE_COMPILER_IS_GNUCXX AND MINGW)
      set_target_properties(${_exename} PROPERTIES LINK_FLAGS "-Wl,--allow-multiple-definition -Wl,--as-needed")
    elseif(WIN32)
      set_target_properties(${_exename} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
    endif()
    # 
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

    add_dependencies(tests ${_exename})
endmacro(PCL_ADD_TEST)

###############################################################################
# Add an example target.
# _name The example name.
# ARGN :
#    FILES the source files for the example
#    LINK_WITH link example executable with libraries
macro(PCL_ADD_EXAMPLE _name)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs FILES LINK_WITH)
    cmake_parse_arguments(PCL_ADD_EXAMPLE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
    add_executable(${_name} ${PCL_ADD_EXAMPLE_FILES})
    target_link_libraries(${_name} ${PCL_ADD_EXAMPLE_LINK_WITH} ${CLANG_LIBRARIES})
    if(WIN32 AND MSVC)
      set_target_properties(${_name} PROPERTIES DEBUG_OUTPUT_NAME ${_name}${CMAKE_DEBUG_POSTFIX}
                                                RELEASE_OUTPUT_NAME ${_name}${CMAKE_RELEASE_POSTFIX})
    endif(WIN32 AND MSVC)
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${_name} PROPERTIES FOLDER "Examples")
    endif(USE_PROJECT_FOLDERS)
endmacro(PCL_ADD_EXAMPLE)

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
        COMPONENT pcl_${_component})
endmacro(PCL_MAKE_PKGCONFIG)

###############################################################################
# Make a pkg-config file for a header-only library. 
# Essentially a duplicate of PCL_MAKE_PKGCONFIG, but 
# ensures that no -L or l flags will be created
# Do not include general PCL stuff in the
# arguments; they will be added automaticaly.
# _name The library name. "pcl_" will be preprended to this.
# _component The part of PCL that this pkg-config file belongs to.
# _desc Description of the library.
# _pcl_deps External dependencies to pcl libs, as a list. (will get mangled to external pkg-config name)
# _ext_deps External dependencies, as a list.
# _int_deps Internal dependencies, as a list.
# _cflags Compiler flags necessary to build with the library.
macro(PCL_MAKE_PKGCONFIG_HEADER_ONLY _name _component _desc _pcl_deps _ext_deps _int_deps _cflags)
set(PKG_NAME ${_name})
set(PKG_DESC ${_desc})
set(PKG_CFLAGS ${_cflags})
#set(PKG_LIBFLAGS ${_lib_flags})
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
configure_file(${PROJECT_SOURCE_DIR}/cmake/pkgconfig-headeronly.cmake.in ${_pc_file} @ONLY)
install(FILES ${_pc_file} DESTINATION ${PKGCFG_INSTALL_DIR}
COMPONENT pcl_${_component})
endmacro(PCL_MAKE_PKGCONFIG_HEADER_ONLY)


###############################################################################
# PRIVATE

###############################################################################
# Reset the subsystem status map.
macro(PCL_RESET_MAPS)
    foreach(_ss ${PCL_SUBSYSTEMS})
        string(TOUPPER "PCL_${_ss}_SUBSYS" PCL_SUBSYS_SUBSYS)
	if (${PCL_SUBSYS_SUBSYS})
            string(TOUPPER "PCL_${_ss}_SUBSYS_DESC" PCL_PARENT_SUBSYS_DESC)
	    set(${PCL_SUBSYS_SUBSYS_DESC} "" CACHE INTERNAL "" FORCE)
	    set(${PCL_SUBSYS_SUBSYS} "" CACHE INTERNAL "" FORCE)
	endif (${PCL_SUBSYS_SUBSYS})
    endforeach(_ss)

    set(PCL_SUBSYS_HYPERSTATUS "" CACHE INTERNAL
        "To Build Or Not To Build, That Is The Question." FORCE)
    set(PCL_SUBSYS_STATUS "" CACHE INTERNAL
        "To build or not to build, that is the question." FORCE)
    set(PCL_SUBSYS_REASONS "" CACHE INTERNAL
        "But why?" FORCE)
    set(PCL_SUBSYS_DEPS "" CACHE INTERNAL "A depends on B and C." FORCE)
    set(PCL_SUBSYS_EXT_DEPS "" CACHE INTERNAL "A depends on B and C." FORCE)
    set(PCL_SUBSYS_OPT_DEPS "" CACHE INTERNAL "A depends on B and C." FORCE)
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
# Register a subsubsystem.
# _name Subsystem name.
# _desc Description of the subsystem
macro(PCL_ADD_SUBSUBSYSTEM _parent _name _desc)
  string(TOUPPER "PCL_${_parent}_SUBSYS" PCL_PARENT_SUBSYS)
  string(TOUPPER "PCL_${_parent}_SUBSYS_DESC" PCL_PARENT_SUBSYS_DESC)
  set(_temp ${${PCL_PARENT_SUBSYS}})
  list(APPEND _temp ${_name})
  set(${PCL_PARENT_SUBSYS} ${_temp} CACHE INTERNAL "Internal list of ${_parenr} subsystems"
    FORCE)
  set_in_global_map(${PCL_PARENT_SUBSYS_DESC} ${_name} ${_desc})
endmacro(PCL_ADD_SUBSUBSYSTEM)


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
# Set the status of a subsystem.
# _name Subsystem name.
# _status TRUE if being built, FALSE otherwise.
# ARGN[0] Reason for not building.
macro(PCL_SET_SUBSUBSYS_STATUS _parent _name _status)
    if(${ARGC} EQUAL 4)
        set(_reason ${ARGV2})
    else(${ARGC} EQUAL 4)
        set(_reason "No reason")
    endif(${ARGC} EQUAL 4)
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_STATUS ${_parent}_${_name} ${_status})
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_REASONS ${_parent}_${_name} ${_reason})
endmacro(PCL_SET_SUBSUBSYS_STATUS)


###############################################################################
# Get the status of a subsystem
# _var Destination variable.
# _name Name of the subsystem.
macro(PCL_GET_SUBSYS_STATUS _var _name)
    GET_IN_MAP(${_var} PCL_SUBSYS_STATUS ${_name})
endmacro(PCL_GET_SUBSYS_STATUS)

###############################################################################
# Get the status of a subsystem
# _var Destination variable.
# _name Name of the subsystem.
macro(PCL_GET_SUBSUBSYS_STATUS _var _parent _name)
    GET_IN_MAP(${_var} PCL_SUBSYS_STATUS ${_parent}_${_name})
endmacro(PCL_GET_SUBSUBSYS_STATUS)


###############################################################################
# Set the hyperstatus of a subsystem and its dependee
# _name Subsystem name.
# _dependee Dependant subsystem.
# _status AUTO_OFF to disable AUTO_ON to enable
# ARGN[0] Reason for not building.
macro(PCL_SET_SUBSYS_HYPERSTATUS _name _dependee _status) 
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_HYPERSTATUS ${_name}_${_dependee} ${_status})
    if(${ARGC} EQUAL 4)
        SET_IN_GLOBAL_MAP(PCL_SUBSYS_REASONS ${_dependee} ${ARGV3})
    endif(${ARGC} EQUAL 4)
endmacro(PCL_SET_SUBSYS_HYPERSTATUS)

###############################################################################
# Get the hyperstatus of a subsystem and its dependee
# _name IN subsystem name.
# _dependee IN dependant subsystem.
# _var OUT hyperstatus
# ARGN[0] Reason for not building.
macro(PCL_GET_SUBSYS_HYPERSTATUS _var _name)
    set(${_var} "AUTO_ON")
    if(${ARGC} EQUAL 3)
        GET_IN_MAP(${_var} PCL_SUBSYS_HYPERSTATUS ${_name}_${ARGV2})
    else(${ARGC} EQUAL 3)
        foreach(subsys ${PCL_SUBSYS_DEPS_${_name}})
            if("${PCL_SUBSYS_HYPERSTATUS_${subsys}_${_name}}" STREQUAL "AUTO_OFF")
                set(${_var} "AUTO_OFF")
                break()
            endif("${PCL_SUBSYS_HYPERSTATUS_${subsys}_${_name}}" STREQUAL "AUTO_OFF")
        endforeach(subsys)
    endif(${ARGC} EQUAL 3)
endmacro(PCL_GET_SUBSYS_HYPERSTATUS)

###############################################################################
# Set the hyperstatus of a subsystem and its dependee
macro(PCL_UNSET_SUBSYS_HYPERSTATUS _name _dependee)
    unset(PCL_SUBSYS_HYPERSTATUS_${_name}_${dependee})
endmacro(PCL_UNSET_SUBSYS_HYPERSTATUS)

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
	    set(message_text "  ${_ss}")
	    string(TOUPPER "PCL_${_ss}_SUBSYS" PCL_SUBSYS_SUBSYS)
	    if (${PCL_SUBSYS_SUBSYS})
	        set(will_build)
		foreach(_sub ${${PCL_SUBSYS_SUBSYS}})
		    PCL_GET_SUBSYS_STATUS(_sub_status ${_ss}_${_sub})
		    if (_sub_status)
		        set(will_build "${will_build}\n       |_ ${_sub}")
		    endif (_sub_status)
		endforeach(_sub)
		if (NOT ("${will_build}" STREQUAL ""))
		  set(message_text  "${message_text}\n       building: ${will_build}")
		endif (NOT ("${will_build}" STREQUAL ""))
		set(wont_build)
		foreach(_sub ${${PCL_SUBSYS_SUBSYS}})
		    PCL_GET_SUBSYS_STATUS(_sub_status ${_ss}_${_sub})
		    PCL_GET_SUBSYS_HYPERSTATUS(_sub_hyper_status ${_ss}_${sub})
		    if (NOT _sub_status OR ("${_sub_hyper_status}" STREQUAL "AUTO_OFF"))
		        GET_IN_MAP(_reason PCL_SUBSYS_REASONS ${_ss}_${_sub})
		        set(wont_build "${wont_build}\n       |_ ${_sub}: ${_reason}")
		    endif (NOT _sub_status OR ("${_sub_hyper_status}" STREQUAL "AUTO_OFF"))
		endforeach(_sub)
		if (NOT ("${wont_build}" STREQUAL ""))
		    set(message_text  "${message_text}\n       not building: ${wont_build}")
		endif (NOT ("${wont_build}" STREQUAL ""))
	    endif (${PCL_SUBSYS_SUBSYS})
	    message(STATUS "${message_text}")
        endif(_status)
    endforeach(_ss)

    message(STATUS "The following subsystems will not be built:")
    foreach(_ss ${PCL_SUBSYSTEMS})
        PCL_GET_SUBSYS_STATUS(_status ${_ss})
        PCL_GET_SUBSYS_HYPERSTATUS(_hyper_status ${_ss})
        if(NOT _status OR ("${_hyper_status}" STREQUAL "AUTO_OFF"))
            GET_IN_MAP(_reason PCL_SUBSYS_REASONS ${_ss})
            message(STATUS "  ${_ss}: ${_reason}")
        endif(NOT _status OR ("${_hyper_status}" STREQUAL "AUTO_OFF"))
    endforeach(_ss)
endmacro(PCL_WRITE_STATUS_REPORT)

##############################################################################
# Collect subdirectories from dirname that contains filename and store them in
#  varname.
# WARNING If extra arguments are given then they are considered as exception 
# list and varname will contain subdirectories of dirname that contains 
# fielename but doesn't belong to exception list.
# dirname IN parent directory
# filename IN file name to look for in each subdirectory of parent directory
# varname OUT list of subdirectories containing filename
# exception_list OPTIONAL and contains list of subdirectories not to account
macro(collect_subproject_directory_names dirname filename names dirs)
    file(GLOB globbed RELATIVE "${dirname}" "${dirname}/*/${filename}")
    if(${ARGC} GREATER 4)
        set(exclusion_list ${ARGN})
        foreach(file ${globbed})
            get_filename_component(dir ${file} PATH)
            list(FIND exclusion_list ${dir} excluded)
            if(excluded EQUAL -1)
                set(${dirs} ${${dirs}} ${dir})
            endif(excluded EQUAL -1)
        endforeach()
    else(${ARGC} GREATER 4)
        foreach(file ${globbed})
            get_filename_component(dir ${file} PATH)
            set(${dirs} ${${dirs}} ${dir})
        endforeach(file)      
    endif(${ARGC} GREATER 4)
    foreach(subdir ${${dirs}})
        file(STRINGS ${dirname}/${subdir}/CMakeLists.txt name REGEX "[setSET ]+\\(.*SUBSYS_NAME .*\\)$")
        string(REGEX REPLACE "[setSET ]+\\(.*SUBSYS_NAME[ ]+([A-Za-z0-9_]+)[ ]*\\)" "\\1" name "${name}")
        set(${names} ${${names}} ${name})
        file(STRINGS ${dirname}/${subdir}/CMakeLists.txt DEPENDENCIES REGEX "set.*SUBSYS_DEPS .*\\)")
        string(REGEX REPLACE "set.*SUBSYS_DEPS" "" DEPENDENCIES "${DEPENDENCIES}")
        string(REPLACE ")" "" DEPENDENCIES "${DEPENDENCIES}")
        string(STRIP "${DEPENDENCIES}" DEPENDENCIES)
        string(REPLACE " " ";" DEPENDENCIES "${DEPENDENCIES}")
        if(NOT("${DEPENDENCIES}" STREQUAL ""))
            list(REMOVE_ITEM DEPENDENCIES "#")
            string(TOUPPER "PCL_${name}_DEPENDS" SUBSYS_DEPENDS)
            set(${SUBSYS_DEPENDS} ${DEPENDENCIES})
            foreach(dependee ${DEPENDENCIES})
                string(TOUPPER "PCL_${dependee}_DEPENDIES" SUBSYS_DEPENDIES)
                set(${SUBSYS_DEPENDIES} ${${SUBSYS_DEPENDIES}} ${name})
            endforeach(dependee)
        endif(NOT("${DEPENDENCIES}" STREQUAL ""))
    endforeach(subdir)
endmacro()

########################################################################################
# Macro to disable subsystem dependies
# _subsys IN subsystem name
macro(PCL_DISABLE_DEPENDIES _subsys)
    string(TOUPPER "pcl_${_subsys}_dependies" PCL_SUBSYS_DEPENDIES)
    if(NOT ("${${PCL_SUBSYS_DEPENDIES}}" STREQUAL ""))
        foreach(dep ${${PCL_SUBSYS_DEPENDIES}})
            PCL_SET_SUBSYS_HYPERSTATUS(${_subsys} ${dep} AUTO_OFF "Disabled: ${_subsys} missing.")
            set(BUILD_${dep} OFF CACHE BOOL "Disabled: ${_subsys} missing." FORCE)
        endforeach(dep)
    endif(NOT ("${${PCL_SUBSYS_DEPENDIES}}" STREQUAL ""))
endmacro(PCL_DISABLE_DEPENDIES subsys)

########################################################################################
# Macro to enable subsystem dependies
# _subsys IN subsystem name
macro(PCL_ENABLE_DEPENDIES _subsys)
    string(TOUPPER "pcl_${_subsys}_dependies" PCL_SUBSYS_DEPENDIES)
    if(NOT ("${${PCL_SUBSYS_DEPENDIES}}" STREQUAL ""))
        foreach(dep ${${PCL_SUBSYS_DEPENDIES}})
            PCL_GET_SUBSYS_HYPERSTATUS(dependee_status ${_subsys} ${dep})
            if("${dependee_status}" STREQUAL "AUTO_OFF")
                PCL_SET_SUBSYS_HYPERSTATUS(${_subsys} ${dep} AUTO_ON)
                GET_IN_MAP(desc PCL_SUBSYS_DESC ${dep})
                set(BUILD_${dep} ON CACHE BOOL "${desc}" FORCE)
            endif("${dependee_status}" STREQUAL "AUTO_OFF")
        endforeach(dep)
    endif(NOT ("${${PCL_SUBSYS_DEPENDIES}}" STREQUAL ""))
endmacro(PCL_ENABLE_DEPENDIES subsys)

########################################################################################
# Macro to build subsystem centric documentation
# _subsys IN the name of the subsystem to generate documentation for
macro (PCL_ADD_DOC _subsys)
  string(TOUPPER "${_subsys}" SUBSYS)
  set(doc_subsys "doc_${_subsys}")
  GET_IN_MAP(dependencies PCL_SUBSYS_DEPS ${_subsys})
  if(DOXYGEN_FOUND)
    if(HTML_HELP_COMPILER)
      set(DOCUMENTATION_HTML_HELP YES)
    else(HTML_HELP_COMPILER)
      set(DOCUMENTATION_HTML_HELP NO)
    endif(HTML_HELP_COMPILER)
    if(DOXYGEN_DOT_EXECUTABLE)
      set(HAVE_DOT YES)
    else(DOXYGEN_DOT_EXECUTABLE)
      set(HAVE_DOT NO)
    endif(DOXYGEN_DOT_EXECUTABLE)
    if(NOT "${dependencies}" STREQUAL "")
      set(STRIPPED_HEADERS "${PCL_SOURCE_DIR}/${dependencies}/include")
      string(REPLACE ";" "/include \\\n\t\t\t\t\t\t\t\t\t\t\t\t ${PCL_SOURCE_DIR}/" 
             STRIPPED_HEADERS "${STRIPPED_HEADERS}")
    endif(NOT "${dependencies}" STREQUAL "")
    set(DOC_SOURCE_DIR "\"${CMAKE_CURRENT_SOURCE_DIR}\"\\")
    foreach(dep ${dependencies})
      set(DOC_SOURCE_DIR 
          "${DOC_SOURCE_DIR}\n\t\t\t\t\t\t\t\t\t\t\t\t \"${PCL_SOURCE_DIR}/${dep}\"\\")
    endforeach(dep)
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html")
    set(doxyfile "${CMAKE_CURRENT_BINARY_DIR}/doxyfile")
    configure_file("${PCL_SOURCE_DIR}/doc/doxygen/doxyfile.in" ${doxyfile})
    add_custom_target(${doc_subsys} ${DOXYGEN_EXECUTABLE} ${doxyfile})
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${doc_subsys} PROPERTIES FOLDER "Documentation")
    endif(USE_PROJECT_FOLDERS)
  endif(DOXYGEN_FOUND)
endmacro(PCL_ADD_DOC)

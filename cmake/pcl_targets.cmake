include(${PROJECT_SOURCE_DIR}/cmake/pcl_utils.cmake)

# Store location of current dir, because value of CMAKE_CURRENT_LIST_DIR is
# set to the directory where a function is used, not where a function is defined
set(_PCL_TARGET_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR})

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
      else()
        set(_reason "Disabled by default.")
      endif()
      PCL_SET_SUBSYS_STATUS(${_name} FALSE ${_reason})
      PCL_DISABLE_DEPENDIES(${_name})
    elseif(NOT ${_opt_name})
      set(${_var} FALSE)
      PCL_SET_SUBSYS_STATUS(${_name} FALSE "Disabled manually.")
      PCL_DISABLE_DEPENDIES(${_name})
    else()
      set(${_var} TRUE)
      PCL_SET_SUBSYS_STATUS(${_name} TRUE)
      PCL_ENABLE_DEPENDIES(${_name})
    endif()
  endif()
  PCL_ADD_SUBSYSTEM(${_name} ${_desc})
endmacro()

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
        else()
          set(_reason "Disabled by default.")
        endif()
        PCL_SET_SUBSYS_STATUS(${_parent}_${_name} FALSE ${_reason})
        PCL_DISABLE_DEPENDIES(${_parent}_${_name})
      elseif(NOT ${_opt_name})
        set(${_var} FALSE)
        PCL_SET_SUBSYS_STATUS(${_parent}_${_name} FALSE "Disabled manually.")
        PCL_DISABLE_DEPENDIES(${_parent}_${_name})
      else()
        set(${_var} TRUE)
        PCL_SET_SUBSYS_STATUS(${_parent}_${_name} TRUE)
        PCL_ENABLE_DEPENDIES(${_parent}_${_name})
      endif()
    endif()
  endif()
  PCL_ADD_SUBSUBSYSTEM(${_parent} ${_name} ${_desc})
endmacro()

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
  cmake_parse_arguments(SUBSYS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  if(SUBSYS_DEPS)
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_DEPS ${_name} "${SUBSYS_DEPS}")
  endif()
  if(SUBSYS_EXT_DEPS)
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_EXT_DEPS ${_name} "${SUBSYS_EXT_DEPS}")
  endif()
  if(SUBSYS_OPT_DEPS)
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_OPT_DEPS ${_name} "${SUBSYS_OPT_DEPS}")
  endif()
  GET_IN_MAP(subsys_status PCL_SUBSYS_HYPERSTATUS ${_name})
  if(${_var} AND (NOT ("${subsys_status}" STREQUAL "AUTO_OFF")))
    if(SUBSYS_DEPS)
      foreach(_dep ${SUBSYS_DEPS})
        PCL_GET_SUBSYS_STATUS(_status ${_dep})
        if(NOT _status)
          set(${_var} FALSE)
          PCL_SET_SUBSYS_STATUS(${_name} FALSE "Requires ${_dep}.")
        else()
          PCL_GET_SUBSYS_INCLUDE_DIR(_include_dir ${_dep})
          include_directories(${PROJECT_SOURCE_DIR}/${_include_dir}/include)
        endif()
      endforeach()
    endif()
    if(SUBSYS_EXT_DEPS)
      foreach(_dep ${SUBSYS_EXT_DEPS})
        string(TOUPPER "${_dep}_found" EXT_DEP_FOUND)
        if(NOT ${EXT_DEP_FOUND} AND (NOT (${EXT_DEP_FOUND} STREQUAL "TRUE")))
          set(${_var} FALSE)
          PCL_SET_SUBSYS_STATUS(${_name} FALSE "Requires external library ${_dep}.")
        endif()
      endforeach()
    endif()
    if(SUBSYS_OPT_DEPS)
      foreach(_dep ${SUBSYS_OPT_DEPS})
        PCL_GET_SUBSYS_INCLUDE_DIR(_include_dir ${_dep})
        include_directories(${PROJECT_SOURCE_DIR}/${_include_dir}/include)
      endforeach()
    endif()
  endif()
endmacro()

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
  cmake_parse_arguments(SUBSYS "${options}" "${parentArg}" "${nameArg}" "${multiValueArgs}" ${ARGN})
  if(SUBSUBSYS_DEPS)
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_DEPS ${_parent}_${_name} "${SUBSUBSYS_DEPS}")
  endif()
  if(SUBSUBSYS_EXT_DEPS)
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_EXT_DEPS ${_parent}_${_name} "${SUBSUBSYS_EXT_DEPS}")
  endif()
  if(SUBSUBSYS_OPT_DEPS)
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_OPT_DEPS ${_parent}_${_name} "${SUBSUBSYS_OPT_DEPS}")
  endif()
  GET_IN_MAP(subsys_status PCL_SUBSYS_HYPERSTATUS ${_parent}_${_name})
  if(${_var} AND (NOT ("${subsys_status}" STREQUAL "AUTO_OFF")))
    if(SUBSUBSYS_DEPS)
      foreach(_dep ${SUBSUBSYS_DEPS})
        PCL_GET_SUBSYS_STATUS(_status ${_dep})
        if(NOT _status)
          set(${_var} FALSE)
          PCL_SET_SUBSYS_STATUS(${_parent}_${_name} FALSE "Requires ${_dep}.")
        else()
          PCL_GET_SUBSYS_INCLUDE_DIR(_include_dir ${_dep})
          include_directories(${PROJECT_SOURCE_DIR}/${_include_dir}/include)
        endif()
      endforeach()
    endif()
    if(SUBSUBSYS_EXT_DEPS)
      foreach(_dep ${SUBSUBSYS_EXT_DEPS})
        string(TOUPPER "${_dep}_found" EXT_DEP_FOUND)
        if(NOT ${EXT_DEP_FOUND} AND (NOT ("${EXT_DEP_FOUND}" STREQUAL "TRUE")))
          set(${_var} FALSE)
          PCL_SET_SUBSYS_STATUS(${_parent}_${_name} FALSE "Requires external library ${_dep}.")
        endif()
      endforeach()
    endif()
  endif()
endmacro()

###############################################################################
# Adds version information to executable/library in form of a version.rc. This works only with MSVC.
#
# _name The library name.
##
function(PCL_ADD_VERSION_INFO _name)
  if(MSVC)
    string(REPLACE "." "," VERSION_INFO_VERSION_WITH_COMMA ${PCL_VERSION})
    if (SUBSUBSYS_DESC)
      set(VERSION_INFO_DISPLAY_NAME ${SUBSUBSYS_DESC})
    else()
      set(VERSION_INFO_DISPLAY_NAME ${SUBSYS_DESC})
    endif()
    set(VERSION_INFO_ICON_PATH "${_PCL_TARGET_CMAKE_DIR}/images/pcl.ico")
    configure_file(${_PCL_TARGET_CMAKE_DIR}/version.rc.in ${PROJECT_BINARY_DIR}/${_name}_version.rc @ONLY)
    target_sources(${_name} PRIVATE ${PROJECT_BINARY_DIR}/${_name}_version.rc)
  endif()
endfunction()

###############################################################################
# Add a set of include files to install.
# _component The part of PCL that the install files belong to.
# _subdir The sub-directory for these include files.
# ARGN The include files.
macro(PCL_ADD_INCLUDES _component _subdir)
  install(FILES ${ARGN}
          DESTINATION ${INCLUDE_INSTALL_DIR}/${_subdir}
          COMPONENT pcl_${_component})
endmacro()

###############################################################################
# Add a library target.
# _name The library name.
# COMPONENT The part of PCL that this library belongs to.
# SOURCES The source files for the library.
function(PCL_ADD_LIBRARY _name)
  set(options)
  set(oneValueArgs COMPONENT)
  set(multiValueArgs SOURCES)
  cmake_parse_arguments(ADD_LIBRARY_OPTION "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_library(${_name} ${PCL_LIB_TYPE} ${ADD_LIBRARY_OPTION_SOURCES})
  PCL_ADD_VERSION_INFO(${_name})
  target_compile_features(${_name} PUBLIC ${PCL_CXX_COMPILE_FEATURES})
  # must link explicitly against boost.
  target_link_libraries(${_name} ${Boost_LIBRARIES} Threads::Threads)
  if((UNIX AND NOT ANDROID) OR MINGW)
    target_link_libraries(${_name} m)
  endif()

  if(MINGW)
    target_link_libraries(${_name} gomp)
  endif()

  if(MSVC)
    target_link_libraries(${_name} delayimp.lib)  # because delay load is enabled for openmp.dll
  endif()

  set_target_properties(${_name} PROPERTIES
    VERSION ${PCL_VERSION}
    SOVERSION ${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
    DEFINE_SYMBOL "PCLAPI_EXPORTS")
  set_target_properties(${_name} PROPERTIES FOLDER "Libraries")

  install(TARGETS ${_name}
          RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT pcl_${ADD_LIBRARY_OPTION_COMPONENT}
          LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT pcl_${ADD_LIBRARY_OPTION_COMPONENT}
          ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT pcl_${ADD_LIBRARY_OPTION_COMPONENT})
endfunction()

###############################################################################
# Add a cuda library target.
# _name The library name.
# COMPONENT The part of PCL that this library belongs to.
# SOURCES The source files for the library.
function(PCL_CUDA_ADD_LIBRARY _name)
  set(options)
  set(oneValueArgs COMPONENT)
  set(multiValueArgs SOURCES)
  cmake_parse_arguments(ADD_LIBRARY_OPTION "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  REMOVE_VTK_DEFINITIONS()
  if(PCL_SHARED_LIBS)
    # to overcome a limitation in cuda_add_library, we add manually PCLAPI_EXPORTS macro
    cuda_add_library(${_name} ${PCL_LIB_TYPE} ${ADD_LIBRARY_OPTION_SOURCES} OPTIONS -DPCLAPI_EXPORTS)
  else()
    cuda_add_library(${_name} ${PCL_LIB_TYPE} ${ADD_LIBRARY_OPTION_SOURCES})
  endif()
  PCL_ADD_VERSION_INFO(${_name})

  # must link explicitly against boost.
  target_link_libraries(${_name} ${Boost_LIBRARIES})

  set_target_properties(${_name} PROPERTIES
    VERSION ${PCL_VERSION}
    SOVERSION ${PCL_VERSION_MAJOR}
    DEFINE_SYMBOL "PCLAPI_EXPORTS")
  set_target_properties(${_name} PROPERTIES FOLDER "Libraries")

  install(TARGETS ${_name}
          RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT pcl_${ADD_LIBRARY_OPTION_COMPONENT}
          LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT pcl_${ADD_LIBRARY_OPTION_COMPONENT}
          ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT pcl_${ADD_LIBRARY_OPTION_COMPONENT})
endfunction()

###############################################################################
# Add an executable target.
# _name The executable name.
# BUNDLE Target should be handled as bundle (APPLE and VTK_USE_COCOA only)
# COMPONENT The part of PCL that this library belongs to.
# SOURCES The source files for the library.
function(PCL_ADD_EXECUTABLE _name)
  set(options BUNDLE)
  set(oneValueArgs COMPONENT)
  set(multiValueArgs SOURCES)
  cmake_parse_arguments(ADD_LIBRARY_OPTION "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(ADD_LIBRARY_OPTION_BUNDLE AND APPLE AND VTK_USE_COCOA)
    add_executable(${_name} MACOSX_BUNDLE ${ADD_LIBRARY_OPTION_SOURCES})
  else()
    add_executable(${_name} ${ADD_LIBRARY_OPTION_SOURCES})
  endif()
  PCL_ADD_VERSION_INFO(${_name})
  # must link explicitly against boost.
  target_link_libraries(${_name} ${Boost_LIBRARIES} Threads::Threads)

  if(WIN32 AND MSVC)
    set_target_properties(${_name} PROPERTIES DEBUG_OUTPUT_NAME ${_name}${CMAKE_DEBUG_POSTFIX}
                                              RELEASE_OUTPUT_NAME ${_name}${CMAKE_RELEASE_POSTFIX})
  endif()

  # Some app targets report are defined with subsys other than apps
  # It's simpler check for tools and assume everythin else as an app
  if(${ADD_LIBRARY_OPTION_COMPONENT} STREQUAL "tools")
    set_target_properties(${_name} PROPERTIES FOLDER "Tools")
  else()
    set_target_properties(${_name} PROPERTIES FOLDER "Apps")
  endif()

  set(PCL_EXECUTABLES ${PCL_EXECUTABLES} ${_name})

  if(ADD_LIBRARY_OPTION_BUNDLE AND APPLE AND VTK_USE_COCOA)
    install(TARGETS ${_name} BUNDLE DESTINATION ${BIN_INSTALL_DIR} COMPONENT pcl_${ADD_LIBRARY_OPTION_COMPONENT})
  else()
    install(TARGETS ${_name} RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT pcl_${ADD_LIBRARY_OPTION_COMPONENT})
  endif()

  string(TOUPPER ${ADD_LIBRARY_OPTION_COMPONENT} _component_upper)
  set(PCL_${_component_upper}_ALL_TARGETS ${PCL_${_component_upper}_ALL_TARGETS} ${_name} PARENT_SCOPE)
endfunction()

###############################################################################
# Add an executable target.
# _name The executable name.
# COMPONENT The part of PCL that this library belongs to.
# SOURCES The source files for the library.
function(PCL_CUDA_ADD_EXECUTABLE _name)
  set(options)
  set(oneValueArgs COMPONENT)
  set(multiValueArgs SOURCES)
  cmake_parse_arguments(ADD_LIBRARY_OPTION "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  REMOVE_VTK_DEFINITIONS()
  cuda_add_executable(${_name} ${ADD_LIBRARY_OPTION_SOURCES})
  PCL_ADD_VERSION_INFO(${_name})

  # must link explicitly against boost.
  target_link_libraries(${_name} ${Boost_LIBRARIES})

  if(WIN32 AND MSVC)
    set_target_properties(${_name} PROPERTIES DEBUG_OUTPUT_NAME ${_name}${CMAKE_DEBUG_POSTFIX}
                                              RELEASE_OUTPUT_NAME ${_name}${CMAKE_RELEASE_POSTFIX})
  endif()

  # There's a single app.
  set_target_properties(${_name} PROPERTIES FOLDER "Apps")

  set(PCL_EXECUTABLES ${PCL_EXECUTABLES} ${_name})
  install(TARGETS ${_name} RUNTIME DESTINATION ${BIN_INSTALL_DIR}
          COMPONENT pcl_${ADD_LIBRARY_OPTION_COMPONENT})
endfunction()

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
  cmake_parse_arguments(PCL_ADD_TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  add_executable(${_exename} ${PCL_ADD_TEST_FILES})
  if(NOT WIN32)
    set_target_properties(${_exename} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  endif()
  #target_link_libraries(${_exename} ${GTEST_BOTH_LIBRARIES} ${PCL_ADD_TEST_LINK_WITH})
  target_link_libraries(${_exename} ${PCL_ADD_TEST_LINK_WITH} ${CLANG_LIBRARIES})

  target_link_libraries(${_exename} Threads::Threads)

  # must link explicitly against boost only on Windows
  target_link_libraries(${_exename} ${Boost_LIBRARIES})

  set_target_properties(${_exename} PROPERTIES FOLDER "Tests")
  add_test(NAME ${_name} COMMAND ${_exename} ${PCL_ADD_TEST_ARGUMENTS})

  add_dependencies(tests ${_exename})
endmacro()

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
  cmake_parse_arguments(PCL_ADD_EXAMPLE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  add_executable(${_name} ${PCL_ADD_EXAMPLE_FILES})
  target_link_libraries(${_name} ${PCL_ADD_EXAMPLE_LINK_WITH} ${CLANG_LIBRARIES})
  if(WIN32 AND MSVC)
    set_target_properties(${_name} PROPERTIES DEBUG_OUTPUT_NAME ${_name}${CMAKE_DEBUG_POSTFIX}
                                              RELEASE_OUTPUT_NAME ${_name}${CMAKE_RELEASE_POSTFIX})
  endif()
  set_target_properties(${_name} PROPERTIES FOLDER "Examples")

  # add target to list of example targets created at the parent scope
  list(APPEND PCL_EXAMPLES_ALL_TARGETS ${_name})
  set(PCL_EXAMPLES_ALL_TARGETS "${PCL_EXAMPLES_ALL_TARGETS}" PARENT_SCOPE)
endmacro()

###############################################################################
# Add compile flags to a target (because CMake doesn't provide something so
# common itself).
# _name The target name.
# _flags The new compile flags to be added, as a string.
macro(PCL_ADD_CFLAGS _name _flags)
  get_target_property(_current_flags ${_name} COMPILE_FLAGS)
  if(NOT _current_flags)
    set_target_properties(${_name} PROPERTIES COMPILE_FLAGS ${_flags})
  else()
    set_target_properties(${_name} PROPERTIES COMPILE_FLAGS "${_current_flags} ${_flags}")
  endif()
endmacro()

###############################################################################
# Add link flags to a target (because CMake doesn't provide something so
# common itself).
# _name The target name.
# _flags The new link flags to be added, as a string.
macro(PCL_ADD_LINKFLAGS _name _flags)
  get_target_property(_current_flags ${_name} LINK_FLAGS)
  if(NOT _current_flags)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS ${_flags})
  else()
      set_target_properties(${_name} PROPERTIES LINK_FLAGS "${_current_flags} ${_flags}")
  endif()
endmacro()

###############################################################################
# Make a pkg-config file for a library. Do not include general PCL stuff in the
# arguments; they will be added automatically.
# _name The library name. "pcl_" will be preprended to this.
# COMPONENT The part of PCL that this pkg-config file belongs to.
# DESC Description of the library.
# PCL_DEPS External dependencies to pcl libs, as a list. (will get mangled to external pkg-config name)
# EXT_DEPS External dependencies, as a list.
# INT_DEPS Internal dependencies, as a list.
# CFLAGS Compiler flags necessary to build with the library.
# LIB_FLAGS Linker flags necessary to link to the library.
# HEADER_ONLY Ensures that no -L or l flags will be created.
function(PCL_MAKE_PKGCONFIG _name)
  set(options HEADER_ONLY)
  set(oneValueArgs COMPONENT DESC CFLAGS LIB_FLAGS)
  set(multiValueArgs PCL_DEPS INT_DEPS EXT_DEPS)
  cmake_parse_arguments(PKGCONFIG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  
  set(PKG_NAME ${_name})
  set(PKG_DESC ${PKGCONFIG_DESC})
  set(PKG_CFLAGS ${PKGCONFIG_CFLAGS})
  set(PKG_LIBFLAGS ${PKGCONFIG_LIB_FLAGS})
  LIST_TO_STRING(PKG_EXTERNAL_DEPS "${PKGCONFIG_EXT_DEPS}")
  foreach(_dep ${PKGCONFIG_PCL_DEPS})
    set(PKG_EXTERNAL_DEPS "${PKG_EXTERNAL_DEPS} pcl_${_dep}-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}")
  endforeach()
  set(PKG_INTERNAL_DEPS "")
  foreach(_dep ${PKGCONFIG_INT_DEPS})
    set(PKG_INTERNAL_DEPS "${PKG_INTERNAL_DEPS} -l${_dep}")
  endforeach()

  set(_pc_file ${CMAKE_CURRENT_BINARY_DIR}/${_name}-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}.pc)
  if(PKGCONFIG_HEADER_ONLY)
    configure_file(${PROJECT_SOURCE_DIR}/cmake/pkgconfig-headeronly.cmake.in ${_pc_file} @ONLY)
  else()
    configure_file(${PROJECT_SOURCE_DIR}/cmake/pkgconfig.cmake.in ${_pc_file} @ONLY)
  endif()
  install(FILES ${_pc_file}
          DESTINATION ${PKGCFG_INSTALL_DIR}
          COMPONENT pcl_${PKGCONFIG_COMPONENT})
endfunction()

###############################################################################
# PRIVATE

###############################################################################
# Reset the subsystem status map.
macro(PCL_RESET_MAPS)
  foreach(_ss ${PCL_SUBSYSTEMS})
    string(TOUPPER "PCL_${_ss}_SUBSYS" PCL_SUBSYS_SUBSYS)
    if(${PCL_SUBSYS_SUBSYS})
      string(TOUPPER "PCL_${_ss}_SUBSYS_DESC" PCL_PARENT_SUBSYS_DESC)
      set(${PCL_SUBSYS_SUBSYS_DESC} "" CACHE INTERNAL "" FORCE)
      set(${PCL_SUBSYS_SUBSYS} "" CACHE INTERNAL "" FORCE)
    endif()
  endforeach()

  set(PCL_SUBSYS_HYPERSTATUS "" CACHE INTERNAL "To Build Or Not To Build, That Is The Question." FORCE)
  set(PCL_SUBSYS_STATUS "" CACHE INTERNAL "To build or not to build, that is the question." FORCE)
  set(PCL_SUBSYS_REASONS "" CACHE INTERNAL "But why?" FORCE)
  set(PCL_SUBSYS_DEPS "" CACHE INTERNAL "A depends on B and C." FORCE)
  set(PCL_SUBSYS_EXT_DEPS "" CACHE INTERNAL "A depends on B and C." FORCE)
  set(PCL_SUBSYS_OPT_DEPS "" CACHE INTERNAL "A depends on B and C." FORCE)
  set(PCL_SUBSYSTEMS "" CACHE INTERNAL "Internal list of subsystems" FORCE)
  set(PCL_SUBSYS_DESC "" CACHE INTERNAL "Subsystem descriptions" FORCE)
endmacro()

###############################################################################
# Register a subsystem.
# _name Subsystem name.
# _desc Description of the subsystem
macro(PCL_ADD_SUBSYSTEM _name _desc)
  set(_temp ${PCL_SUBSYSTEMS})
  list(APPEND _temp ${_name})
  set(PCL_SUBSYSTEMS ${_temp} CACHE INTERNAL "Internal list of subsystems" FORCE)
  SET_IN_GLOBAL_MAP(PCL_SUBSYS_DESC ${_name} ${_desc})
endmacro()

###############################################################################
# Register a subsubsystem.
# _name Subsystem name.
# _desc Description of the subsystem
macro(PCL_ADD_SUBSUBSYSTEM _parent _name _desc)
  string(TOUPPER "PCL_${_parent}_SUBSYS" PCL_PARENT_SUBSYS)
  string(TOUPPER "PCL_${_parent}_SUBSYS_DESC" PCL_PARENT_SUBSYS_DESC)
  set(_temp ${${PCL_PARENT_SUBSYS}})
  list(APPEND _temp ${_name})
  set(${PCL_PARENT_SUBSYS} ${_temp} CACHE INTERNAL "Internal list of ${_parenr} subsystems" FORCE)
  set_in_global_map(${PCL_PARENT_SUBSYS_DESC} ${_name} ${_desc})
endmacro()

###############################################################################
# Set the status of a subsystem.
# _name Subsystem name.
# _status TRUE if being built, FALSE otherwise.
# ARGN[0] Reason for not building.
macro(PCL_SET_SUBSYS_STATUS _name _status)
  if(${ARGC} EQUAL 3)
    set(_reason ${ARGV2})
  else()
    set(_reason "No reason")
  endif()
  SET_IN_GLOBAL_MAP(PCL_SUBSYS_STATUS ${_name} ${_status})
  SET_IN_GLOBAL_MAP(PCL_SUBSYS_REASONS ${_name} ${_reason})
endmacro()

###############################################################################
# Set the status of a subsystem.
# _name Subsystem name.
# _status TRUE if being built, FALSE otherwise.
# ARGN[0] Reason for not building.
macro(PCL_SET_SUBSUBSYS_STATUS _parent _name _status)
  if(${ARGC} EQUAL 4)
    set(_reason ${ARGV2})
  else()
    set(_reason "No reason")
  endif()
  SET_IN_GLOBAL_MAP(PCL_SUBSYS_STATUS ${_parent}_${_name} ${_status})
  SET_IN_GLOBAL_MAP(PCL_SUBSYS_REASONS ${_parent}_${_name} ${_reason})
endmacro()

###############################################################################
# Get the status of a subsystem
# _var Destination variable.
# _name Name of the subsystem.
macro(PCL_GET_SUBSYS_STATUS _var _name)
  GET_IN_MAP(${_var} PCL_SUBSYS_STATUS ${_name})
endmacro()

###############################################################################
# Get the status of a subsystem
# _var Destination variable.
# _name Name of the subsystem.
macro(PCL_GET_SUBSUBSYS_STATUS _var _parent _name)
    GET_IN_MAP(${_var} PCL_SUBSYS_STATUS ${_parent}_${_name})
endmacro()

###############################################################################
# Set the hyperstatus of a subsystem and its dependee
# _name Subsystem name.
# _dependee Dependent subsystem.
# _status AUTO_OFF to disable AUTO_ON to enable
# ARGN[0] Reason for not building.
macro(PCL_SET_SUBSYS_HYPERSTATUS _name _dependee _status)
  SET_IN_GLOBAL_MAP(PCL_SUBSYS_HYPERSTATUS ${_name}_${_dependee} ${_status})
  if(${ARGC} EQUAL 4)
    SET_IN_GLOBAL_MAP(PCL_SUBSYS_REASONS ${_dependee} ${ARGV3})
  endif()
endmacro()

###############################################################################
# Get the hyperstatus of a subsystem and its dependee
# _name IN subsystem name.
# _dependee IN dependent subsystem.
# _var OUT hyperstatus
# ARGN[0] Reason for not building.
macro(PCL_GET_SUBSYS_HYPERSTATUS _var _name)
  set(${_var} "AUTO_ON")
  if(${ARGC} EQUAL 3)
    GET_IN_MAP(${_var} PCL_SUBSYS_HYPERSTATUS ${_name}_${ARGV2})
  else()
    foreach(subsys ${PCL_SUBSYS_DEPS_${_name}})
      if("${PCL_SUBSYS_HYPERSTATUS_${subsys}_${_name}}" STREQUAL "AUTO_OFF")
        set(${_var} "AUTO_OFF")
        break()
      endif()
    endforeach()
  endif()
endmacro()

###############################################################################
# Set the hyperstatus of a subsystem and its dependee
macro(PCL_UNSET_SUBSYS_HYPERSTATUS _name _dependee)
  unset(PCL_SUBSYS_HYPERSTATUS_${_name}_${dependee})
endmacro()

###############################################################################
# Set the include directory name of a subsystem.
# _name Subsystem name.
# _includedir Name of subdirectory for includes
# ARGN[0] Reason for not building.
macro(PCL_SET_SUBSYS_INCLUDE_DIR _name _includedir)
  SET_IN_GLOBAL_MAP(PCL_SUBSYS_INCLUDE ${_name} ${_includedir})
endmacro()

###############################################################################
# Get the include directory name of a subsystem - return _name if not set
# _var Destination variable.
# _name Name of the subsystem.
macro(PCL_GET_SUBSYS_INCLUDE_DIR _var _name)
  GET_IN_MAP(${_var} PCL_SUBSYS_INCLUDE ${_name})
  if(NOT ${_var})
    set(${_var} ${_name})
  endif()
endmacro()

###############################################################################
# Write a report on the build/not-build status of the subsystems
macro(PCL_WRITE_STATUS_REPORT)
  message(STATUS "The following subsystems will be built:")
  foreach(_ss ${PCL_SUBSYSTEMS})
    PCL_GET_SUBSYS_STATUS(_status ${_ss})
    if(_status)
      set(message_text "  ${_ss}")
      string(TOUPPER "PCL_${_ss}_SUBSYS" PCL_SUBSYS_SUBSYS)
      if(${PCL_SUBSYS_SUBSYS})
        set(will_build)
        foreach(_sub ${${PCL_SUBSYS_SUBSYS}})
          PCL_GET_SUBSYS_STATUS(_sub_status ${_ss}_${_sub})
          if(_sub_status)
            set(will_build "${will_build}\n       |_ ${_sub}")
          endif()
        endforeach()
        if(NOT ("${will_build}" STREQUAL ""))
          set(message_text  "${message_text}\n       building: ${will_build}")
        endif()
        set(wont_build)
        foreach(_sub ${${PCL_SUBSYS_SUBSYS}})
          PCL_GET_SUBSYS_STATUS(_sub_status ${_ss}_${_sub})
          PCL_GET_SUBSYS_HYPERSTATUS(_sub_hyper_status ${_ss}_${sub})
          if(NOT _sub_status OR ("${_sub_hyper_status}" STREQUAL "AUTO_OFF"))
            GET_IN_MAP(_reason PCL_SUBSYS_REASONS ${_ss}_${_sub})
            set(wont_build "${wont_build}\n       |_ ${_sub}: ${_reason}")
          endif()
        endforeach()
        if(NOT ("${wont_build}" STREQUAL ""))
          set(message_text  "${message_text}\n       not building: ${wont_build}")
        endif()
      endif()
      message(STATUS "${message_text}")
    endif()
  endforeach()

  message(STATUS "The following subsystems will not be built:")
  foreach(_ss ${PCL_SUBSYSTEMS})
    PCL_GET_SUBSYS_STATUS(_status ${_ss})
    PCL_GET_SUBSYS_HYPERSTATUS(_hyper_status ${_ss})
    if(NOT _status OR ("${_hyper_status}" STREQUAL "AUTO_OFF"))
       GET_IN_MAP(_reason PCL_SUBSYS_REASONS ${_ss})
       message(STATUS "  ${_ss}: ${_reason}")
    endif()
  endforeach()
endmacro()

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
      endif()
    endforeach()
  else()
    foreach(file ${globbed})
      get_filename_component(dir ${file} PATH)
      set(${dirs} ${${dirs}} ${dir})
    endforeach()
  endif()
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
      endforeach()
    endif()
  endforeach()
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
    endforeach()
  endif()
endmacro()

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
      endif()
    endforeach()
  endif()
endmacro()

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
    else()
      set(DOCUMENTATION_HTML_HELP NO)
    endif()
    if(DOXYGEN_DOT_EXECUTABLE)
      set(HAVE_DOT YES)
    else()
      set(HAVE_DOT NO)
    endif()
    if(NOT "${dependencies}" STREQUAL "")
      set(STRIPPED_HEADERS "${PCL_SOURCE_DIR}/${dependencies}/include")
      string(REPLACE ";" "/include \\\n                         ${PCL_SOURCE_DIR}/"
             STRIPPED_HEADERS "${STRIPPED_HEADERS}")
    endif()
    set(DOC_SOURCE_DIR "\"${CMAKE_CURRENT_SOURCE_DIR}\"\\")
    foreach(dep ${dependencies})
      set(DOC_SOURCE_DIR
          "${DOC_SOURCE_DIR}\n\t\t\t\t\t\t\t\t\t\t\t\t \"${PCL_SOURCE_DIR}/${dep}\"\\")
    endforeach()
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html")
    set(doxyfile "${CMAKE_CURRENT_BINARY_DIR}/doxyfile")
    configure_file("${PCL_SOURCE_DIR}/doc/doxygen/doxyfile.in" ${doxyfile})
    add_custom_target(${doc_subsys} ${DOXYGEN_EXECUTABLE} ${doxyfile})
    set_target_properties(${doc_subsys} PROPERTIES FOLDER "Documentation")
  endif()
endmacro()

###############################################################################
# Add a dependency for a grabber
# _name The dependency name.
# _description The description text to display when dependency is not found.
# This macro adds on option named "WITH_NAME", where NAME is the capitalized
# dependency name. The user may use this option to control whether the
# corresponding grabber should be built or not. Also an attempt to find a
# package with the given name is made. If it is not successful, then the
# "WITH_NAME" option is coerced to FALSE.
macro(PCL_ADD_GRABBER_DEPENDENCY _name _description)
  string(TOUPPER ${_name} _name_capitalized)
  option(WITH_${_name_capitalized} "${_description}" TRUE)
  if(WITH_${_name_capitalized})
    find_package(${_name})
    if(NOT ${_name_capitalized}_FOUND)
      set(WITH_${_name_capitalized} FALSE CACHE BOOL "${_description}" FORCE)
      message(STATUS "${_description}: not building because ${_name} not found")
    else()
      set(HAVE_${_name_capitalized} TRUE)
      include_directories(SYSTEM "${${_name_capitalized}_INCLUDE_DIRS}")
    endif()
  endif()
endmacro()

###############################################################################
# Set the dependencies for a specific test module on the provided variable
# _var The variable to be filled with the dependencies
# _module The module name
macro(PCL_SET_TEST_DEPENDENCIES _var _module)
  set(${_var} global_tests ${_module} ${PCL_SUBSYS_DEPS_${_module}})
endmacro()

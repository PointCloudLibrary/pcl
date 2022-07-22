# .rst:
# UseCompilerCache
# --------
#
# This module provides a function to setup a compiler cache tool (default: ``ccache``)
# Main function of interest is ``UseCompilerCache``
#
# Needs CMake 3.4 at least
# Inspired from:
# * https://crascit.com/2016/04/09/using-ccache-with-cmake/
# * https://stackoverflow.com/a/36515503/
# * https://gitlab.kitware.com/henryiii/cmake/blob/cache/Modules/UseCompilerCache.cmake

# .rst
# pcl_ccache_compat_file_gen
# -- Generates a wrapper file which launches the compiler commands using ccache.
#    This allows support for XCode and CCache < 3.3
function(pcl_ccache_compat_file_gen FILE_NAME CCACHE_PROGRAM COMPILER)
  message(STATUS "${FILE_NAME} for ${CCACHE_PROGRAM} with ${COMPILER}")
  file(WRITE "${CMAKE_BINARY_DIR}/${FILE_NAME}" ""
       "#! /usr/bin/env sh\n"
       "\n"
       "# Xcode generator doesn't include the compiler as the\n"
       "# first argument, Ninja and Makefiles do. Handle both cases.\n"
       "if [ \"$1\" = \"${COMPILER}\" ] ; then\n"
       "  shift\n"
       "fi\n"
       "\n"
       "export CCACHE_CPP2=true\n"
       "exec \"${CCACHE_PROGRAM}\" \"${COMPILER}\" \"$@\"\n")
endfunction()

# .rst
# UseCompilerCache([PROGRAM <ccache_name>] [QUIET] [REQUIRED])
# -- Add the compiler cache tool (default to look for ccache on the path)
#    to your build through CMAKE_<LANG>_COMPILER_LAUNCHER variables. Also
#    supports XCode. Uses a wrapper for XCode and CCache < 3.3.
#    Sets the COMPILER_CACHE_VERSION variable.
function(UseCompilerCache)
  set(options QUIET REQUIRED)
  set(oneValueArgs CCACHE)
  set(multiValueArgs)

  cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  
  if(ARGS_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "Unknown arguments given to UseCompilerCache: ${ARGS_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARGS_CCACHE)
    set(ARGS_CCACHE ccache)
  endif()

  find_program(CCACHE_PROGRAM ${ARGS_CCACHE})

  # Quit if not found
  if(NOT CCACHE_PROGRAM)
    if(REQUIRED)
      message(FATAL_ERROR "Failed to find ${CCACHE_PROGRAM} (REQUIRED)")
    endif()
    return()
  endif()

  if(CMAKE_GENERATOR MATCHES "Visual")
    message(FATAL_ERROR "MSVC isn't compatible with current solutions. Please rename compiler cache to cl.exe and prepend its location in env PATH variable")
    return()
  endif()

  # Get version number
  execute_process(COMMAND "${CCACHE_PROGRAM}" --version OUTPUT_VARIABLE output)
  string(REPLACE "\n" ";" output "${output}")
  foreach(line ${output})
    string(TOLOWER ${line} line)
    string(REGEX REPLACE "^ccache version ([\\.0-9]+)$" "\\1" version "${line}")
    if(version)
      set(COMPILER_CACHE_VERSION ${version} PARENT_SCOPE)
      break()
    endif()
  endforeach()

  if(NOT ARGS_QUIET)
    message(STATUS "Using Compiler Cache (${CCACHE_PROGRAM}) v${version} in the C/C++ toolchain")
  endif()

  set(xcode_compat FALSE)
  if(CMAKE_GENERATOR STREQUAL Xcode)
    set(xcode_compat TRUE)
  endif()
  set(ccache_compat FALSE)
  if((ARGS_CCACHE STREQUAL ccache) AND (version VERSION_LESS 3.3.0))
    set(ccache_compat TRUE)
  endif()

  # Indirect wrapper is needed for CCache < 3.3 or XCode
  if(NOT (${xcode_compat} OR ${ccache_compat}))
    # Support Unix Makefiles and Ninja
    message(STATUS "Compiler cache via cmake launcher prefix")
    set(CMAKE_C_COMPILER_LAUNCHER    "${CCACHE_PROGRAM}" PARENT_SCOPE)
    set(CMAKE_CXX_COMPILER_LAUNCHER  "${CCACHE_PROGRAM}" PARENT_SCOPE)
    set(CMAKE_CUDA_COMPILER_LAUNCHER "${CCACHE_PROGRAM}" PARENT_SCOPE)
    return()
  endif()

  message(STATUS "Generating launch helpers for compiler cache")

  pcl_ccache_compat_file_gen("launch-c" ${CCACHE_PROGRAM} ${CMAKE_C_COMPILER})
  pcl_ccache_compat_file_gen("launch-cxx" ${CCACHE_PROGRAM} ${CMAKE_CXX_COMPILER})
  execute_process(COMMAND chmod a+rx
                  "${CMAKE_BINARY_DIR}/launch-c"
                  "${CMAKE_BINARY_DIR}/launch-cxx")

  if(CMAKE_CUDA_COMPILER)
    pcl_ccache_compat_file_gen("launch-cuda" ${CCACHE_PROGRAM} ${CMAKE_CUDA_COMPILER})
    execute_process(COMMAND chmod a+rx
                    "${CMAKE_BINARY_DIR}/launch-cuda")
  endif()

  if(${xcode_compat})
    # Set Xcode project attributes to route compilation and linking properly
    message(STATUS "Compiler cache via launch files to support XCode")
    set(CMAKE_XCODE_ATTRIBUTE_CC         "${CMAKE_BINARY_DIR}/launch-c" PARENT_SCOPE)
    set(CMAKE_XCODE_ATTRIBUTE_CXX        "${CMAKE_BINARY_DIR}/launch-cxx" PARENT_SCOPE)
    set(CMAKE_XCODE_ATTRIBUTE_LD         "${CMAKE_BINARY_DIR}/launch-c" PARENT_SCOPE)
    set(CMAKE_XCODE_ATTRIBUTE_LDPLUSPLUS "${CMAKE_BINARY_DIR}/launch-cxx" PARENT_SCOPE)
  else()
    message(STATUS "Compiler cache via launch files to support Unix Makefiles and Ninja")
    set(CMAKE_C_COMPILER_LAUNCHER    "${CMAKE_BINARY_DIR}/launch-c" PARENT_SCOPE)
    set(CMAKE_CXX_COMPILER_LAUNCHER  "${CMAKE_BINARY_DIR}/launch-cxx" PARENT_SCOPE)
    if (CMAKE_CUDA_COMPILER)
        set(CMAKE_CUDA_COMPILER_LAUNCHER "${CMAKE_BINARY_DIR}/launch-cuda" PARENT_SCOPE)
    endif()
  endif()
endfunction()

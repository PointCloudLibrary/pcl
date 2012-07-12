# This is FindGTSAM.cmake
# CMake module to locate the GTSAM package
#
# The following cache variables may be set before calling this script:
#
# GTSAM_DIR (or GTSAM_ROOT): (Optional) The install prefix OR source tree of gtsam (e.g. /usr/local or src/gtsam)
# GTSAM_BUILD_NAME:          (Optional) If compiling against a source tree, the name of the build directory
#                            within it (e.g build-debug).  Without this defined, this script tries to
#                            intelligently find the build directory based on the project's build directory name
#                            or based on the build type (Debug/Release/etc).
#
# The following variables will be defined:
#
# GTSAM_FOUND          : TRUE if the package has been successfully found
# GTSAM_INCLUDE_DIR    : paths to GTSAM's INCLUDE directories
# GTSAM_LIBS           : paths to GTSAM's libraries
#
# NOTES on compiling against an uninstalled GTSAM build tree:
# - A GTSAM source tree will be automatically searched for in the directory
#   'gtsam' next to your project directory, after searching
#   CMAKE_INSTALL_PREFIX and $HOME, but before searching /usr/local and /usr.
# - The build directory will be searched first with the same name as your
#   project's build directory, e.g. if you build from 'MyProject/build-optimized',
#   'gtsam/build-optimized' will be searched first.  Next, a build directory for
#   your project's build type, e.g. if CMAKE_BUILD_TYPE in your project is
#   'Release', then 'gtsam/build-release' will be searched next.  Finally, plain
#   'gtsam/build' will be searched.
# - You can control the gtsam build directory name directly by defining the CMake
#   cache variable 'GTSAM_BUILD_NAME', then only 'gtsam/${GTSAM_BUILD_NAME} will
#   be searched.
# - Use the standard CMAKE_PREFIX_PATH, or GTSAM_DIR, to find a specific gtsam
#   directory.

# Get path suffixes to help look for gtsam
if(GTSAM_BUILD_NAME)
  set(gtsam_build_names "${GTSAM_BUILD_NAME}/gtsam")
else()
  # lowercase build type
  string(TOLOWER "${CMAKE_BUILD_TYPE}" build_type_suffix)
  # build suffix of this project
  get_filename_component(my_build_name "${CMAKE_BINARY_DIR}" NAME)
  
  set(gtsam_build_names "${my_build_name}/gtsam" "build-${build_type_suffix}/gtsam" "build/gtsam")
endif()

# Use GTSAM_ROOT or GTSAM_DIR equivalently
if(GTSAM_ROOT AND NOT GTSAM_DIR)
  set(GTSAM_DIR "${GTSAM_ROOT}")
endif()

if(GTSAM_DIR)
  # Find include dirs
  find_path(GTSAM_INCLUDE_DIR gtsam/inference/FactorGraph.h
    PATHS "${GTSAM_DIR}/include" "${GTSAM_DIR}" NO_DEFAULT_PATH
    DOC "GTSAM include directories")

  # Find libraries
  find_library(GTSAM_LIBS NAMES gtsam
    HINTS "${GTSAM_DIR}/lib" "${GTSAM_DIR}" NO_DEFAULT_PATH
    PATH_SUFFIXES ${gtsam_build_names}
    DOC "GTSAM libraries")
else()
  # Find include dirs
  set(extra_include_paths ${CMAKE_INSTALL_PREFIX}/include "$ENV{HOME}/include" "${PROJECT_SOURCE_DIR}/../gtsam" /usr/local/include /usr/include)
  find_path(GTSAM_INCLUDE_DIR gtsam/inference/FactorGraph.h
    PATHS ${extra_include_paths}
    DOC "GTSAM include directories")
  if(NOT GTSAM_INCLUDE_DIR)
    message(STATUS "Searched for gtsam headers in default paths plus ${extra_include_paths}")
  endif()

  # Find libraries
  find_library(GTSAM_LIBS NAMES gtsam
    HINTS ${CMAKE_INSTALL_PREFIX}/lib "$ENV{HOME}/lib" "${PROJECT_SOURCE_DIR}/../gtsam" /usr/local/lib /usr/lib
    PATH_SUFFIXES ${gtsam_build_names}
    DOC "GTSAM libraries")
endif()

# handle the QUIETLY and REQUIRED arguments and set GTSAM_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GTSAM DEFAULT_MSG
                                  GTSAM_LIBS GTSAM_INCLUDE_DIR)
 




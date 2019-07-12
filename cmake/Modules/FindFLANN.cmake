#.rst:
# FindFLANN
# --------
#
# Try to find FLANN library and headers. This module supports both old released versions
# of FLANN ≤ 1.9.1 and newer development versions that ship with a modern config file.
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the :prop_tgt:`IMPORTED` targets:
#
# ``FLANN::FLANN``
#  Defined if the system has FLANN.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module sets the following variables:
#
# ::
#
#   FLANN_FOUND               True in case FLANN is found, otherwise false
#
# Example usage
# ^^^^^^^^^^^^^
#
# ::
#
#     find_package(FLANN REQUIRED)
#
#     add_executable(foo foo.cc)
#     target_link_libraries(foo FLANN::FLANN)
#

# First try to locate FLANN using modern config
find_package(flann NO_MODULE ${FLANN_FIND_VERSION} QUIET)
if(flann_FOUND)
  unset(flann_FOUND)
  set(FLANN_FOUND ON)
  # Create interface library that effectively becomes an alias for the appropriate (static/dynamic) imported FLANN target
  add_library(FLANN::FLANN INTERFACE IMPORTED)
  if(FLANN_USE_STATIC)
    set_property(TARGET FLANN::FLANN APPEND PROPERTY INTERFACE_LINK_LIBRARIES flann::flann_cpp_s)
  else()
    set_property(TARGET FLANN::FLANN APPEND PROPERTY INTERFACE_LINK_LIBRARIES flann::flann_cpp)
  endif()
  return()
endif()

# Second try to locate FLANN using pkgconfig
find_package(PkgConfig QUIET)
if(FLANN_FIND_VERSION)
  pkg_check_modules(PC_FLANN flann>=${FLANN_FIND_VERSION})
else()
  pkg_check_modules(PC_FLANN flann)
endif()

find_path(FLANN_INCLUDE_DIR
  NAMES
    flann/flann.hpp
  HINTS
    ${PC_FLANN_INCLUDE_DIRS}
    ${FLANN_ROOT}
    $ENV{FLANN_ROOT}
  PATHS
    $ENV{PROGRAMFILES}/Flann
    $ENV{PROGRAMW6432}/Flann
  PATH_SUFFIXES
    include
)

if(FLANN_USE_STATIC)
  set(FLANN_RELEASE_NAME flann_cpp_s)
  set(FLANN_DEBUG_NAME flann_cpp_s-gd)
  set(FLANN_LIBRARY_TYPE STATIC)
else()
  set(FLANN_RELEASE_NAME flann_cpp)
  set(FLANN_DEBUG_NAME flann_cpp-gd)
  set(FLANN_LIBRARY_TYPE SHARED)
endif()

find_library(FLANN_LIBRARY
  NAMES
    ${FLANN_RELEASE_NAME}
  HINTS
    ${PC_FLANN_LIBRARY_DIRS}
    ${FLANN_ROOT}
    $ENV{FLANN_ROOT}
  PATHS
    $ENV{PROGRAMFILES}/Flann
    $ENV{PROGRAMW6432}/Flann
  PATH_SUFFIXES
    lib
)

find_library(FLANN_LIBRARY_DEBUG
  NAMES
    ${FLANN_DEBUG_NAME}
  HINTS
    ${PC_FLANN_LIBRARY_DIRS}
    ${FLANN_ROOT}
    $ENV{FLANN_ROOT}
  PATHS
    $ENV{PROGRAMFILES}/Flann
    $ENV{PROGRAMW6432}/Flann
  PATH_SUFFIXES
    lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  FLANN DEFAULT_MSG
  FLANN_LIBRARY FLANN_INCLUDE_DIR
)

if(FLANN_FOUND)
  if(NOT TARGET FLANN::FLANN)
    add_library(FLANN::FLANN ${FLANN_LIBRARY_TYPE} IMPORTED)
    set_target_properties(FLANN::FLANN PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${FLANN_INCLUDE_DIR}")
    set_target_properties(FLANN::FLANN PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${PC_FLANN_CFLAGS_OTHER}")
    set_property(TARGET FLANN::FLANN APPEND PROPERTY IMPORTED_CONFIGURATIONS "RELEASE")
    set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX")
    if(WIN32 AND NOT FLANN_USE_STATIC)
      set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_IMPLIB_RELEASE "${FLANN_LIBRARY}")
    else()
      set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_LOCATION_RELEASE "${FLANN_LIBRARY}")
    endif()
    if(FLANN_LIBRARY_DEBUG)
      set_property(TARGET FLANN::FLANN APPEND PROPERTY IMPORTED_CONFIGURATIONS "DEBUG")
      set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX")
      if(WIN32 AND NOT FLANN_USE_STATIC)
        set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_IMPLIB_DEBUG "${FLANN_LIBRARY_DEBUG}")
      else()
        set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_LOCATION_DEBUG "${FLANN_LIBRARY_DEBUG}")
      endif()
    endif()
  endif()
endif()

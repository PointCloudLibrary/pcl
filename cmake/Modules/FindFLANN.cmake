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
#   FLANN_ROOT                Path to the root of found FLANN installation
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

# Early return if FLANN target is already defined. This makes it safe to run
# this script multiple times.
if(TARGET FLANN::FLANN)
  return()
endif()

# First try to locate FLANN using modern config
find_package(flann NO_MODULE ${FLANN_FIND_VERSION} QUIET)

if(flann_FOUND)
  unset(flann_FOUND)
  set(FLANN_FOUND ON)

  # Create interface library that effectively becomes an alias for the appropriate (static/dynamic) imported FLANN target
  add_library(FLANN::FLANN INTERFACE IMPORTED)

  if(TARGET flann::flann_cpp_s AND TARGET flann::flann_cpp)
    if(PCL_FLANN_REQUIRED_TYPE MATCHES "SHARED")
      set_property(TARGET FLANN::FLANN APPEND PROPERTY INTERFACE_LINK_LIBRARIES flann::flann_cpp)
      set(FLANN_LIBRARY_TYPE SHARED)
    elseif(PCL_FLANN_REQUIRED_TYPE MATCHES "STATIC")
      set_property(TARGET FLANN::FLANN APPEND PROPERTY INTERFACE_LINK_LIBRARIES flann::flann_cpp_s)
      set(FLANN_LIBRARY_TYPE STATIC)
    else()
      if(PCL_SHARED_LIBS)
        set_property(TARGET FLANN::FLANN APPEND PROPERTY INTERFACE_LINK_LIBRARIES flann::flann_cpp)
        set(FLANN_LIBRARY_TYPE SHARED)
      else()
        set_property(TARGET FLANN::FLANN APPEND PROPERTY INTERFACE_LINK_LIBRARIES flann::flann_cpp_s)
        set(FLANN_LIBRARY_TYPE STATIC)
      endif()
    endif()
  elseif(TARGET flann::flann_cpp_s)
    set_property(TARGET FLANN::FLANN APPEND PROPERTY INTERFACE_LINK_LIBRARIES flann::flann_cpp_s)
    set(FLANN_LIBRARY_TYPE STATIC)
  else()
    set_property(TARGET FLANN::FLANN APPEND PROPERTY INTERFACE_LINK_LIBRARIES flann::flann_cpp)
    set(FLANN_LIBRARY_TYPE SHARED)
  endif()

  # Determine FLANN installation root based on the path to the processed Config file
  get_filename_component(_config_dir "${flann_CONFIG}" DIRECTORY)
  get_filename_component(FLANN_ROOT "${_config_dir}/../../.." ABSOLUTE)
  unset(_config_dir)
  message(STATUS "Found flann version ${flann_VERSION}")
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

find_library(FLANN_LIBRARY_SHARED
  NAMES
    flann_cpp
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

find_library(FLANN_LIBRARY_DEBUG_SHARED
  NAMES
    flann_cpp-gd flann_cppd
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

find_library(FLANN_LIBRARY_STATIC
  NAMES
    flann_cpp_s
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

find_library(FLANN_LIBRARY_DEBUG_STATIC
  NAMES
    flann_cpp_s-gd flann_cpp_sd
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

if(FLANN_LIBRARY_SHARED AND FLANN_LIBRARY_STATIC)
  if(PCL_FLANN_REQUIRED_TYPE MATCHES "SHARED")
    set(FLANN_LIBRARY_TYPE SHARED)
    set(FLANN_LIBRARY ${FLANN_LIBRARY_SHARED})
  elseif(PCL_FLANN_REQUIRED_TYPE MATCHES "STATIC")
    set(FLANN_LIBRARY_TYPE STATIC)
    set(FLANN_LIBRARY ${FLANN_LIBRARY_STATIC})
  else()
    if(PCL_SHARED_LIBS)
      set(FLANN_LIBRARY_TYPE SHARED)
      set(FLANN_LIBRARY ${FLANN_LIBRARY_SHARED})
    else()
      set(FLANN_LIBRARY_TYPE STATIC)
      set(FLANN_LIBRARY ${FLANN_LIBRARY_STATIC})
    endif()
  endif()
elseif(FLANN_LIBRARY_STATIC)
  set(FLANN_LIBRARY_TYPE STATIC)
  set(FLANN_LIBRARY ${FLANN_LIBRARY_STATIC})
elseif(FLANN_LIBRARY_SHARED)
  set(FLANN_LIBRARY_TYPE SHARED)
  set(FLANN_LIBRARY ${FLANN_LIBRARY_SHARED})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  FLANN DEFAULT_MSG
  FLANN_LIBRARY FLANN_INCLUDE_DIR
)

if(FLANN_FOUND)
  add_library(FLANN::FLANN ${FLANN_LIBRARY_TYPE} IMPORTED)
  set_target_properties(FLANN::FLANN PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${FLANN_INCLUDE_DIR}")
  set_target_properties(FLANN::FLANN PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${PC_FLANN_CFLAGS_OTHER}")
  set_property(TARGET FLANN::FLANN APPEND PROPERTY IMPORTED_CONFIGURATIONS "RELEASE")
  set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX")
  if(WIN32 AND (NOT FLANN_LIBRARY_TYPE MATCHES "STATIC"))
    set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_IMPLIB_RELEASE "${FLANN_LIBRARY}")
  else()
    set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_LOCATION_RELEASE "${FLANN_LIBRARY}")
  endif()
  if(FLANN_LIBRARY_DEBUG)
    set_property(TARGET FLANN::FLANN APPEND PROPERTY IMPORTED_CONFIGURATIONS "DEBUG")
    set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX")
    if(WIN32 AND (NOT FLANN_LIBRARY_TYPE MATCHES "STATIC"))
      set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_IMPLIB_DEBUG "${FLANN_LIBRARY_DEBUG}")
    else()
      set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_LOCATION_DEBUG "${FLANN_LIBRARY_DEBUG}")
    endif()
  endif()
  # Pkgconfig may specify additional link libraries besides from FLANN itself
  # in PC_FLANN_LIBRARIES, add them to the target link interface.
  foreach(_library ${PC_FLANN_LIBRARIES})
    if(NOT _library MATCHES "flann")
      set_property(TARGET FLANN::FLANN APPEND PROPERTY INTERFACE_LINK_LIBRARIES "${_library}")
    endif()
  endforeach()
  get_filename_component(FLANN_ROOT "${FLANN_INCLUDE_DIR}" PATH)
  message(STATUS "FLANN found (include: ${FLANN_INCLUDE_DIR}, lib: ${FLANN_LIBRARY})")
endif()

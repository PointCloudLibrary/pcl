#.rst:
# FindFLANN
# --------
#
# Try to find FLANN library and include files.
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
#   FLANN_DEFINITIONS         Compiler flags for FLANN.
#   FLANN_INCLUDE_DIR         Location of FLANN header files
#   FLANN_INCLUDE_DIRS        Location of FLANN header files (including dependencies)
#   FLANN_LIBRARY             FLANN release library
#   FLANN_LIBRARY_DEBUG       FLANN debug library
#   FLANN_LIBRARIES           FLANN release and debug library
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

find_package(PkgConfig QUIET)
if(FLANN_FIND_VERSION)
  pkg_check_modules(PC_FLANN flann>=${FLANN_FIND_VERSION})
else()
  pkg_check_modules(PC_FLANN flann)
endif()

set(FLANN_DEFINITIONS ${PC_FLANN_CFLAGS_OTHER})

if(FLANN_USE_STATIC)
  set(FLANN_RELEASE_NAME flann_cpp_s)
  set(FLANN_DEBUG_NAME flann_cpp_s-gd)
  set(FLANN_DEFINITIONS ${FLANN_DEFINITIONS} "FLANN_STATIC")
else()
  set(FLANN_RELEASE_NAME flann_cpp)
  set(FLANN_DEBUG_NAME flann_cpp-gd)
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

if(FLANN_LIBRARY AND FLANN_LIBRARY_DEBUG)
  set(FLANN_LIBRARIES optimized ${FLANN_LIBRARY} debug ${FLANN_LIBRARY_DEBUG})
else()
  set(FLANN_LIBRARIES ${FLANN_LIBRARY})
endif()

set(FLANN_INCLUDE_DIRS
  ${FLANN_INCLUDE_DIR}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  FLANN DEFAULT_MSG
  FLANN_LIBRARIES FLANN_INCLUDE_DIR
)

if(FLANN_FOUND)
  if(NOT TARGET FLANN::FLANN)
    if (FLANN_USE_STATIC)
      add_library(FLANN::FLANN STATIC IMPORTED)
    else()
      add_library(FLANN::FLANN SHARED IMPORTED)
    endif()
    set_target_properties(FLANN::FLANN PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${FLANN_INCLUDE_DIRS}")
    set_target_properties(FLANN::FLANN PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${FLANN_DEFINITIONS}")
    if(FLANN_LIBRARY)
      set_property(TARGET FLANN::FLANN APPEND PROPERTY IMPORTED_CONFIGURATIONS "RELEASE")
      set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX")
      if(WIN32 AND NOT FLANN_USE_STATIC)
        set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_IMPLIB_RELEASE "${FLANN_LIBRARY}")
      else()
        set_target_properties(FLANN::FLANN PROPERTIES IMPORTED_LOCATION_RELEASE "${FLANN_LIBRARY}")
      endif()
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

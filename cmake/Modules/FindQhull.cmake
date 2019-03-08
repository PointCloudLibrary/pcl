###############################################################################
# Find QHULL
#
# This sets the following variables:
# QHULL_FOUND - True if QHULL was found.
# QHULL_INCLUDE_DIRS - Directories containing the QHULL include files.
# QHULL_LIBRARIES - Libraries needed to use QHULL.
# QHULL_DEFINITIONS - Compiler flags for QHULL.
# If QHULL_USE_STATIC is specified then look for static libraries ONLY else
# look for shared ones

set(QHULL_MAJOR_VERSION 6)

if(QHULL_USE_STATIC)
  set(QHULL_RELEASE_NAME qhullstatic)
  set(QHULL_DEBUG_NAME qhullstatic_d)
else()
  set(QHULL_RELEASE_NAME qhull_p qhull${QHULL_MAJOR_VERSION} qhull)
  set(QHULL_DEBUG_NAME qhull_p_d qhull${QHULL_MAJOR_VERSION}_d qhull_d${QHULL_MAJOR_VERSION} qhull_d)
endif()

find_file(QHULL_HEADER
          NAMES libqhull/libqhull.h qhull.h
          HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}" "${QHULL_INCLUDE_DIR}"
          PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
          PATH_SUFFIXES qhull src/libqhull libqhull include)

set(QHULL_HEADER "${QHULL_HEADER}" CACHE INTERNAL "QHull header" FORCE )

if(QHULL_HEADER)
  get_filename_component(qhull_header ${QHULL_HEADER} NAME_WE)
  if("${qhull_header}" STREQUAL "qhull")
    set(HAVE_QHULL_2011 OFF)
    get_filename_component(QHULL_INCLUDE_DIR ${QHULL_HEADER} PATH)
  elseif("${qhull_header}" STREQUAL "libqhull")
    set(HAVE_QHULL_2011 ON)
    get_filename_component(QHULL_INCLUDE_DIR ${QHULL_HEADER} PATH)
    get_filename_component(QHULL_INCLUDE_DIR ${QHULL_INCLUDE_DIR} PATH)
  endif()
else()
  set(QHULL_INCLUDE_DIR "QHULL_INCLUDE_DIR-NOTFOUND")
endif()

find_library(QHULL_LIBRARY
             NAMES ${QHULL_RELEASE_NAME}
             HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
             PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
             PATH_SUFFIXES project build bin lib)

get_filename_component(QHULL_LIBRARY_NAME "${QHULL_LIBRARY}" NAME)

find_library(QHULL_LIBRARY_DEBUG
             NAMES ${QHULL_DEBUG_NAME} ${QHULL_RELEASE_NAME}
             HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
             PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
             PATH_SUFFIXES project build bin lib debug/lib)

if(NOT QHULL_LIBRARY_DEBUG)
  set(QHULL_LIBRARY_DEBUG ${QHULL_LIBRARY})
endif()

get_filename_component(QHULL_LIBRARY_DEBUG_NAME "${QHULL_LIBRARY_DEBUG}" NAME)

if(QHULL_INCLUDE_DIR AND QHULL_LIBRARY)

  # Include directories
  set(QHULL_INCLUDE_DIRS ${QHULL_INCLUDE_DIR})
  unset(QHULL_INCLUDE_DIR)
  mark_as_advanced(QHULL_INCLUDE_DIRS)

  # Libraries
  set(QHULL_LIBRARIES optimized ${QHULL_LIBRARY} debug ${QHULL_LIBRARY_DEBUG})
  unset(QHULL_LIBRARY)
  unset(QHULL_LIBRARY_DEBUG)
  mark_as_advanced(QHULL_LIBRARIES)

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Qhull
  FOUND_VAR QHULL_FOUND
  REQUIRED_VARS QHULL_LIBRARIES QHULL_INCLUDE_DIRS
)

if(QHULL_FOUND)
  set(HAVE_QHULL ON)
  if(NOT QHULL_USE_STATIC)
    add_definitions("-Dqh_QHpointer")
    if(MSVC)
      add_definitions("-Dqh_QHpointer_dllimport")
    endif()
  endif()
  message(STATUS "QHULL found (include: ${QHULL_INCLUDE_DIRS}, lib: ${QHULL_LIBRARIES})")
endif()

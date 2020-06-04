###############################################################################
# Find QHULL
#
# This sets the following variables:
# Qhull_FOUND - True if QHULL was found.
# Qhull_INCLUDE_DIRS - Directories containing the QHULL include files.
# Qhull_LIBRARIES - Libraries needed to use QHULL.
# Qhull_DEFINITIONS - Compiler flags for QHULL.
# If Qhull_USE_STATIC is specified then look for static libraries ONLY else
# look for shared ones

set(Qhull_MAJOR_VERSION 6)

if(Qhull_USE_STATIC)
  set(Qhull_RELEASE_NAME qhullstatic)
  set(Qhull_DEBUG_NAME qhullstatic_d)
else()
  set(Qhull_RELEASE_NAME qhull_p qhull${Qhull_MAJOR_VERSION} qhull)
  set(Qhull_DEBUG_NAME qhull_p_d qhull${Qhull_MAJOR_VERSION}_d qhull_d${Qhull_MAJOR_VERSION} qhull_d)
endif()

find_file(Qhull_HEADER
          NAMES libqhull/libqhull.h qhull.h
          HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}" "${Qhull_INCLUDE_DIR}"
          PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
          PATH_SUFFIXES qhull src/libqhull libqhull include)

set(Qhull_HEADER "${Qhull_HEADER}" CACHE INTERNAL "QHull header" FORCE )

if(Qhull_HEADER)
  get_filename_component(qhull_header ${Qhull_HEADER} NAME_WE)
  if("${qhull_header}" STREQUAL "qhull")
    set(HAVE_QHULL_2011 OFF)
    get_filename_component(Qhull_INCLUDE_DIR ${Qhull_HEADER} PATH)
  elseif("${qhull_header}" STREQUAL "libqhull")
    set(HAVE_QHULL_2011 ON)
    get_filename_component(Qhull_INCLUDE_DIR ${Qhull_HEADER} PATH)
    get_filename_component(Qhull_INCLUDE_DIR ${Qhull_INCLUDE_DIR} PATH)
  endif()
else()
  set(Qhull_INCLUDE_DIR "QHULL_INCLUDE_DIR-NOTFOUND")
endif()

find_library(Qhull_LIBRARY
             NAMES ${Qhull_RELEASE_NAME}
             HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
             PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
             PATH_SUFFIXES project build bin lib)

get_filename_component(Qhull_LIBRARY_NAME "${Qhull_LIBRARY}" NAME)

find_library(Qhull_LIBRARY_DEBUG
             NAMES ${Qhull_DEBUG_NAME} ${Qhull_RELEASE_NAME}
             HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
             PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
             PATH_SUFFIXES project build bin lib debug/lib)

if(NOT Qhull_LIBRARY_DEBUG)
  set(Qhull_LIBRARY_DEBUG ${QHULL_LIBRARY})
endif()

get_filename_component(Qhull_LIBRARY_DEBUG_NAME "${Qhull_LIBRARY_DEBUG}" NAME)

if(Qhull_INCLUDE_DIR AND Qhull_LIBRARY)

  # Include directories
  set(Qhull_INCLUDE_DIRS ${Qhull_INCLUDE_DIR})
  unset(Qhull_INCLUDE_DIR)
  mark_as_advanced(Qhull_INCLUDE_DIRS)

  # Libraries
  set(Qhull_LIBRARIES optimized ${Qhull_LIBRARY} debug ${Qhull_LIBRARY_DEBUG})
  unset(Qhull_LIBRARY)
  unset(Qhull_LIBRARY_DEBUG)
  mark_as_advanced(Qhull_LIBRARIES)

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Qhull DEFAULT_MSG Qhull_LIBRARIES Qhull_INCLUDE_DIRS
)

if(Qhull_FOUND)
  set(HAVE_QHULL ON)
  if(NOT Qhull_USE_STATIC)
    add_definitions("-Dqh_QHpointer")
    if(MSVC)
      add_definitions("-Dqh_QHpointer_dllimport")
    endif()
  endif()
endif()

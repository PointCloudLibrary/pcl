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

if(QHULL_USE_STATIC)
  set(QHULL_RELEASE_NAME qhullstatic_r qhullstatic)
  set(QHULL_DEBUG_NAME qhullstatic_rd qhullstatic_d)
else(QHULL_USE_STATIC)
  set(QHULL_RELEASE_NAME qhull_r qhull_p qhull${QHULL_MAJOR_VERSION} qhull)
  set(QHULL_DEBUG_NAME qhull_rd qhull_pd qhull${QHULL_MAJOR_VERSION}_d qhull_d${QHULL_MAJOR_VERSION} qhull_d)
endif(QHULL_USE_STATIC)

find_file(QHULL_HEADER
          NAMES libqhull_r/libqhull_r.h libqhull/libqhull.h qhull.h
          HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}" "${QHULL_INCLUDE_DIR}"
          PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull" 
          PATH_SUFFIXES qhull src/libqhull libqhull include)

set(QHULL_HEADER "${QHULL_HEADER}" CACHE INTERNAL "QHull header" FORCE )

set(HAVE_QHULL_2011 OFF)
set(HAVE_QHULL_2015 OFF)

if(QHULL_HEADER)
  get_filename_component(qhull_header ${QHULL_HEADER} NAME_WE)
  if("${qhull_header}" STREQUAL "qhull")
    set(QHULL_MAJOR_VERSION 6)
    set(HAVE_QHULL_2011 OFF)
    set(HAVE_QHULL_2015 OFF)
    get_filename_component(QHULL_INCLUDE_DIR ${QHULL_HEADER} PATH)
  elseif("${qhull_header}" STREQUAL "libqhull")
    set(QHULL_MAJOR_VERSION 6)
    set(HAVE_QHULL_2011 ON)
    set(HAVE_QHULL_2015 OFF)
    get_filename_component(QHULL_INCLUDE_DIR ${QHULL_HEADER} PATH)
    get_filename_component(QHULL_INCLUDE_DIR ${QHULL_INCLUDE_DIR} PATH)
  elseif("${qhull_header}" STREQUAL "libqhull_r")
    set(QHULL_MAJOR_VERSION 7)
    set(HAVE_QHULL_2011 OFF)
    set(HAVE_QHULL_2015 ON)
    get_filename_component(QHULL_INCLUDE_DIR ${QHULL_HEADER} PATH)
    get_filename_component(QHULL_INCLUDE_DIR ${QHULL_INCLUDE_DIR} PATH)
  endif()
else(QHULL_HEADER)
  set(QHULL_INCLUDE_DIR "QHULL_INCLUDE_DIR-NOTFOUND")
endif(QHULL_HEADER)

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

if (QHULL_MAJOR_VERSION STREQUAL 7)
  string(REGEX MATCH ".*r[d]?\\.so$" QHULLR_LIB_FOUND "${QHULL_LIBRARY_DEBUG}")
  if("${QHULLR_LIB_FOUND}" STREQUAL "")
    # Reentrant libraries cannot be found although headers are available.
    # Use non-reentrant interface instead
    set(QHULL_MAJOR_VERSION 6)
    set(HAVE_QHULL_2011 ON)
    set(HAVE_QHULL_2015 OFF)
  endif()
endif()

if(NOT QHULL_LIBRARY_DEBUG)
  set(QHULL_LIBRARY_DEBUG ${QHULL_LIBRARY})
endif(NOT QHULL_LIBRARY_DEBUG)

get_filename_component(QHULL_LIBRARY_DEBUG_NAME "${QHULL_LIBRARY_DEBUG}" NAME)

set(QHULL_INCLUDE_DIRS ${QHULL_INCLUDE_DIR})
set(QHULL_LIBRARIES optimized ${QHULL_LIBRARY} debug ${QHULL_LIBRARY_DEBUG})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Qhull DEFAULT_MSG QHULL_LIBRARY QHULL_INCLUDE_DIR)

mark_as_advanced(QHULL_LIBRARY QHULL_LIBRARY_DEBUG QHULL_INCLUDE_DIR)

if(QHULL_FOUND)
  set(HAVE_QHULL ON)
  if(NOT QHULL_USE_STATIC)
    add_definitions("-Dqh_QHpointer")
    if(MSVC)
      add_definitions("-Dqh_QHpointer_dllimport")
    endif(MSVC)
  endif(NOT QHULL_USE_STATIC)
  message(STATUS "QHULL found (include: ${QHULL_INCLUDE_DIRS}, lib: ${QHULL_LIBRARIES})")
endif(QHULL_FOUND)

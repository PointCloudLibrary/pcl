###############################################################################
# Find QHULL
#
# This sets the following variables:
# QHULL_FOUND - True if QHULL was found.
# QHULL_INCLUDE_DIRS - Directories containing the QHULL include files.
# QHULL_LIBRARIES - Libraries needed to use QHULL.
# QHULL_DEFINITIONS - Compiler flags for QHULL.

set(QHULL_MAJOR_VERSION 6)

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
else(QHULL_HEADER)
    set(QHULL_INCLUDE_DIR "QHULL_INCLUDE_DIR-NOTFOUND")
endif(QHULL_HEADER)

set(QHULL_INCLUDE_DIR "${QHULL_INCLUDE_DIR}" CACHE PATH "QHull include dir." FORCE)

# Prefer static libraries in Windows over shared ones
if(WIN32)
  find_library(QHULL_LIBRARY 
               NAMES qhullstatic qhull qhull${QHULL_MAJOR_VERSION}
               HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
               PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull" 
               PATH_SUFFIXES project build bin lib)

  find_library(QHULL_LIBRARY_DEBUG 
               NAMES qhullstatic_d qhull_d qhull${QHULL_MAJOR_VERSION}_d qhull_d${QHULL_MAJOR_VERSION} qhull qhull${QHULL_MAJOR_VERSION}
               HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
               PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull" 
               PATH_SUFFIXES project build bin lib)
else(WIN32)
  find_library(QHULL_LIBRARY 
               NAMES qhull qhull${QHULL_MAJOR_VERSION}
               HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
               PATH_SUFFIXES project build bin lib)

  find_library(QHULL_LIBRARY_DEBUG 
               NAMES qhull_d qhull_d${QHULL_MAJOR_VERSION} qhull qhull${QHULL_MAJOR_VERSION}
               HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
               PATH_SUFFIXES project build bin lib)
endif(WIN32)

if(NOT QHULL_LIBRARY_DEBUG)
  set(QHULL_LIBRARY_DEBUG ${QHULL_LIBRARY})
endif(NOT QHULL_LIBRARY_DEBUG)

set(QHULL_INCLUDE_DIRS ${QHULL_INCLUDE_DIR})
set(QHULL_LIBRARIES optimized ${QHULL_LIBRARY} debug ${QHULL_LIBRARY_DEBUG})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Qhull DEFAULT_MSG QHULL_LIBRARY
    QHULL_INCLUDE_DIR)

mark_as_advanced(QHULL_LIBRARY QHULL_LIBRARY_DEBUG QHULL_INCLUDE_DIR)

if(QHULL_FOUND)
  set(HAVE_QHULL ON)
  get_filename_component(qhull_lib ${QHULL_LIBRARY} NAME_WE)
  if(NOT "${qhull_lib}" MATCHES "qhullstatic")
    add_definitions("-Dqh_QHpointer")
    if(MSVC)
      add_definitions("-Dqh_QHpointer_dllimport")
    endif(MSVC)
  endif(NOT "${qhull_lib}" MATCHES "qhullstatic")
  message(STATUS "QHULL found (include: ${QHULL_INCLUDE_DIRS}, lib: ${QHULL_LIBRARIES})")
endif(QHULL_FOUND)

###############################################################################
# Find QHULL
#
# This sets the following variables:
# QHULL_FOUND - True if QHULL was found.
# QHULL_INCLUDE_DIRS - Directories containing the QHULL include files.
# QHULL_LIBRARIES - Libraries needed to use QHULL.
# QHULL_DEFINITIONS - Compiler flags for QHULL.
# If QHULL_USE_SHARED is specified then look for shared libraries ONLY else
# look for static ones

set(QHULL_MAJOR_VERSION 7)

if(QHULL_USE_SHARED)
  set(QHULL_RELEASE_NAME qhull_r)
  set(QHULL_DEBUG_NAME qhull_rd)
else(QHULL_USE_SHARED)
  set(QHULL_RELEASE_NAME qhullstatic_r)
  set(QHULL_DEBUG_NAME qhullstatic_rd)
endif(QHULL_USE_SHARED)

find_file(QHULL_HEADER
          NAMES libqhull_r/libqhull_r.h
          HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}" "${QHULL_INCLUDE_DIR}"
          PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull" 
          PATH_SUFFIXES src/libqhull_r libqhull_r qhull src/libqhull libqhull include)

set(QHULL_HEADER "${QHULL_HEADER}" CACHE INTERNAL "QHull header" FORCE )

if(QHULL_HEADER)
  get_filename_component(qhull_header ${QHULL_HEADER} NAME_WE)
  get_filename_component(QHULL_INCLUDE_DIR ${QHULL_HEADER} PATH)
  get_filename_component(QHULL_INCLUDE_DIR ${QHULL_INCLUDE_DIR} PATH)
else(QHULL_HEADER)
  set(QHULL_INCLUDE_DIR "QHULL_INCLUDE_DIR-NOTFOUND")
endif(QHULL_HEADER)

set(QHULL_INCLUDE_DIR "${QHULL_INCLUDE_DIR}" CACHE PATH "QHull include dir." FORCE)

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

find_library(QHULL_CPP_LIBRARY
               NAMES qhullcpp
               HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
               PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
               PATH_SUFFIXES project build bin lib)

find_library(QHULL_CPP_LIBRARY_DEBUG
        NAMES qhullcpp_d qhullcpp
        HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
        PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
        PATH_SUFFIXES project build bin lib)

if(NOT QHULL_LIBRARY_DEBUG)
  set(QHULL_CPP_LIBRARY_DEBUG ${QHULL_CPP_LIBRARY})
  set(QHULL_LIBRARY_DEBUG ${QHULL_LIBRARY})
endif(NOT QHULL_LIBRARY_DEBUG)

get_filename_component(QHULL_LIBRARY_DEBUG_NAME "${QHULL_LIBRARY_DEBUG}" NAME)

set(QHULL_INCLUDE_DIRS ${QHULL_INCLUDE_DIR})
set(QHULL_LIBRARIES optimized ${QHULL_CPP_LIBRARY} ${QHULL_LIBRARY} debug ${QHULL_CPP_LIBRARY_DEBUG} ${QHULL_LIBRARY_DEBUG})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Qhull DEFAULT_MSG QHULL_LIBRARY QHULL_INCLUDE_DIR)

mark_as_advanced(QHULL_LIBRARY QHULL_LIBRARY_DEBUG QHULL_INCLUDE_DIR)

if(QHULL_FOUND)
  set(HAVE_QHULL ON)
  message(STATUS "QHULL found (include: ${QHULL_INCLUDE_DIRS}, lib: ${QHULL_LIBRARIES})")
endif(QHULL_FOUND)

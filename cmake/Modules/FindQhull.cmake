###############################################################################
# Find QHULL
#
# This sets the following variables:
# QHULL_FOUND - True if QHULL was found.
# QHULL_INCLUDE_DIRS - Directories containing the QHULL include files.
# QHULL_LIBRARIES - Libraries needed to use QHULL.
# QHULL_DEFINITIONS - Compiler flags for QHULL.

set(QHULL_MAJOR_VERSION 6)

find_path(QHULL_INCLUDE_DIR_PRE2011
          NAMES qhull.h
          HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
          PATHS "$ENV{PROGRAMFILES}/qhull 6.2.0.1373" "$ENV{PROGRAMW6432}/qhull 6.2.0.1373" 
          PATH_SUFFIXES qhull src/libqhull libqhull include)

find_path(QHULL_INCLUDE_DIR_2011
          NAMES libqhull/libqhull.h
          HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
          PATHS "$ENV{PROGRAMFILES}/qhull 6.2.0.1373" "$ENV{PROGRAMW6432}/qhull 6.2.0.1373" 
          PATH_SUFFIXES qhull src/libqhull libqhull include)

if(QHULL_INCLUDE_DIR_PRE2011)
   set(HAVE_QHULL_2011 OFF)
   set(QHULL_INCLUDE_DIR "${QHULL_INCLUDE_DIR_PRE2011}")
elseif(QHULL_INCLUDE_DIR_2011)
   set(HAVE_QHULL_2011 ON)
   set(QHULL_INCLUDE_DIR "${QHULL_INCLUDE_DIR_2011}")
endif()

# Prefer static libraries in Windows over shared ones
if(WIN32)
  find_library(QHULL_LIBRARY 
               NAMES qhullstatic qhull qhull${QHULL_MAJOR_VERSION}
               HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
               PATHS "$ENV{PROGRAMFILES}/qhull 6.2.0.1373" "$ENV{PROGRAMW6432}/qhull 6.2.0.1373" 
               PATH_SUFFIXES project build bin lib)

  find_library(QHULL_LIBRARY_DEBUG 
               NAMES qhullstatic_d qhull_d qhull_d${QHULL_MAJOR_VERSION} qhull qhull${QHULL_MAJOR_VERSION}
               HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
               PATHS "$ENV{PROGRAMFILES}/qhull 6.2.0.1373" "$ENV{PROGRAMW6432}/qhull 6.2.0.1373" 
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
  message(STATUS "QHULL found (include: ${QHULL_INCLUDE_DIRS}, lib: ${QHULL_LIBRARIES})")
endif(QHULL_FOUND)

###############################################################################
# Find QHULL
#
# This sets the following variables:
# QHULL_FOUND - True if QHULL was found.
# QHULL_INCLUDE_DIRS - Directories containing the QHULL include files.
# QHULL_LIBRARIES - Libraries needed to use QHULL.
# QHULL_DEFINITIONS - Compiler flags for QHULL.

#find_path(QHULL_INCLUDE_DIR qhull/qhull.h)
find_path(QHULL_INCLUDE_DIR qhull.h
          HINTS ${QHULL_ROOT}
          PATH_SUFFIXES qhull src)

#find_library(QHULL_LIBRARY qhull)
find_library(QHULL_LIBRARY qhull
             HINTS ${QHULL_ROOT}
             PATH_SUFFIXES project)

set(QHULL_INCLUDE_DIRS ${QHULL_INCLUDE_DIR}/qhull ${QHULL_INCLUDE_DIR})
set(QHULL_LIBRARIES ${QHULL_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Qhull DEFAULT_MSG QHULL_LIBRARY
    QHULL_INCLUDE_DIR)

mark_as_advanced(QHULL_LIBRARY QHULL_INCLUDE_DIR)

if(QHULL_FOUND)
  message(STATUS "QHULL found (include: ${QHULL_INCLUDE_DIRS}, lib: ${QHULL_LIBRARIES})")
endif(QHULL_FOUND)


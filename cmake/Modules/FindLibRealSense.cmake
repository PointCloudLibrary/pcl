###############################################################################
# Find Intel librealsense
#
#     find_package(LIBREALSENSE)
#
# Variables defined by this module:
#
#  LIBREALSENSE_FOUND                 True if librealsense was found
#  LIBREALSENSE_INCLUDE_DIRS          The location(s) of librealsense headers
#  LIBREALSENSE_LIBRARIES             Libraries needed to use librealsense

find_package(PkgConfig QUIET)

# Include directories
find_path(LIBREALSENSE_INCLUDE_DIR librealsense2/rs.h)
set(LIBREALSENSE_INCLUDE_DIRS ${LIBREALSENSE_INCLUDE_DIR})
mark_as_advanced(LIBREALSENSE_INCLUDE_DIRS)

# Libraries
find_library(LIBREALSENSE_LIBRARY NAMES librealsense.lib)
find_library(LIBREALSENSE_LIBRARY_DEBUG NAMES librealsense.lib)

if(NOT LIBREALSENSE_LIBRARY_DEBUG)
  set(LIBREALSENSE_LIBRARY_DEBUG ${LIBREALSENSE_LIBRARY})
endif()

set(LIBREALSENSE_LIBRARIES optimized ${LIBREALSENSE_LIBRARY} debug ${LIBREALSENSE_LIBRARY_DEBUG})
mark_as_advanced(LIBREALSENSE_LIBRARY LIBREALSENSE_LIBRARY_DEBUG)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBREALSENSE
  FOUND_VAR LIBREALSENSE_FOUND
  REQUIRED_VARS LIBREALSENSE_LIBRARIES LIBREALSENSE_INCLUDE_DIRS
)
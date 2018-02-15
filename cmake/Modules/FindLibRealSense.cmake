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
find_path(LIBREALSENSE_INCLUDE_DIR librealsense2/rs.h
          PATHS "$ENV{realsense2_DIR}"
                "$ENV{PROGRAMW6432}/librealsense2" # for self built library 
                "$ENV{PROGRAMFILES}/librealsense2" # for self built library
                "$ENV{PROGRAMFILES}/Intel RealSense SDK 2.0" # for pre built library
                "$ENV{LIBREALSENSE_ROOT}"
                # "Please specify search paths for Linux and MacOS"
          PATH_SUFFIXES include)

set(LIBREALSENSE_INCLUDE_DIRS ${LIBREALSENSE_INCLUDE_DIR})
mark_as_advanced(LIBREALSENSE_INCLUDE_DIRS)

# Libraries
set(LIBREALSENSE_RELEASE_NAME realsense2)
set(LIBREALSENSE_DEBUG_NAME realsense2_d)

set(LIBREALSENSE_SUFFIX x86)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(LIBREALSENSE_SUFFIX x64)
endif()

find_library(LIBREALSENSE_LIBRARY
             NAMES ${LIBREALSENSE_RELEASE_NAME}
             PATHS "$ENV{realsense2_DIR}"
                   "$ENV{PROGRAMW6432}/librealsense2"
                   "$ENV{PROGRAMFILES}/librealsense2"
                   "$ENV{PROGRAMFILES}/Intel RealSense SDK 2.0"
                   "$ENV{LIBREALSENSE_ROOT}"
                   # "Please specify search paths for Linux and MacOS"
             PATH_SUFFIXES lib lib/${LIBREALSENSE_SUFFIX})

find_library(LIBREALSENSE_LIBRARY_DEBUG
             NAMES ${LIBREALSENSE_DEBUG_NAME}
             PATHS "$ENV{realsense2_DIR}"
                   "$ENV{PROGRAMW6432}/librealsense2"
                   "$ENV{PROGRAMFILES}/librealsense2"
                   "$ENV{PROGRAMFILES}/Intel RealSense SDK 2.0"
                   "$ENV{LIBREALSENSE_ROOT}"
                   # "Please specify search paths for Linux and MacOS"
             PATH_SUFFIXES lib lib/${LIBREALSENSE_SUFFIX})

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
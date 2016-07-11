###############################################################################
# Find librealsense
#
#     find_package(librealsense)
#
# Variables defined by this module:
#
#  LIBREALSENSE_FOUND                 True if librealsense was found
#  LIBREALSENSE_INCLUDE_DIRS          The location(s) of librealsense headers
#  LIBREALSENSE_LIBRARIES             Libraries needed to use librealsense

if (LIBREALSENSE_LIBRARIES AND LIBREALSENSE_INCLUDE_DIRS)
  # in cache already
  set(LIBREALSENSE_FOUND TRUE)
else (LIBREALSENSE_LIBRARIES AND LIBREALSENSE_INCLUDE_DIRS)
find_path(LIBREALSENSE_INCLUDE_DIR
          NAMES librealsense
          PATHS /usr/local/include /usr/include /opt/local/include /sw/include)

find_library(LIBREALSENSE_LIBRARY
             NAMES librealsense.so
             PATHS /usr/local/lib /usr/lib /opt/local/lib /sw/lib)

set(LIBREALSENSE_INCLUDE_DIRS ${LIBREALSENSE_INCLUDE_DIR})
set(LIBREALSENSE_LIBRARIES ${LIBREALSENSE_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBREALSENSE LIBREALSENSE_LIBRARY LIBREALSENSE_INCLUDE_DIR)

mark_as_advanced(LIBREALSENSE_INCLUDE_DIRS LIBREALSENSE_LIBRARIES)
endif (LIBREALSENSE_LIBRARIES AND LIBREALSENSE_INCLUDE_DIRS)

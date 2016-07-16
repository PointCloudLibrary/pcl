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
  find_path(LIBREALSENSE_DIR include/librealsense
            PATHS $ENV{LIBREALSENSE_DIR} NO_DEFAULT_PATH)
  if (WIN32)
    set(LIBREALSENSE_RELEASE_NAME realsense.lib)
    set(LIBREALSENSE_DEBUG_NAME realsense-d.lib)
    set(LIBREALSENSE_SUFFIX Win32)
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(LIBREALSENSE_SUFFIX x64)
    endif ()
  else (WIN32)
    set(LIBREALSENSE_RELEASE_NAME librealsense.so)
    set(LIBREALSENSE_DEBUG_NAME librealsense.a)
  endif (WIN32)

find_path(LIBREALSENSE_INCLUDE_DIR
          NAMES librealsense
          PATHS /usr/local/include /usr/include /opt/local/include /sw/include ${LIBREALSENSE_DIR}/include NO_DEFAULT_PATH)

find_library(LIBREALSENSE_LIBRARY
             NAMES ${LIBREALSENSE_RELEASE_NAME}
             PATHS ${LIBREALSENSE_DIR}/bin/ /usr/local/lib /usr/lib /opt/local/lib /sw/lib ${LIBREALSENSE_DIR}/lib/ NO_DEFAULT_PATH
             PATH_SUFFIXES ${LIBREALSENSE_SUFFIX})
find_library(LIBREALSENSE_LIBRARY_DEBUG
             NAMES ${LIBREALSENSE_DEBUG_NAME}
             PATHS ${LIBREALSENSE_DIR}/bin/ ${LIBREALSENSE_DIR}/bin/debug/ ${LIBREALSENSE_DIR}/lib/ NO_DEFAULT_PATH
             PATH_SUFFIXES ${LIBREALSENSE_SUFFIX})
if (NOT LIBREALSENSE_LIBRARY_DEBUG)
  set(LIBREALSENSE_LIBRARY_DEBUG ${LIBREALSENSE_LIBRARY})
endif ()

set(LIBREALSENSE_LIBRARIES optimized ${LIBREALSENSE_LIBRARY} debug ${LIBREALSENSE_LIBRARY_DEBUG})
set(LIBREALSENSE_INCLUDE_DIRS ${LIBREALSENSE_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBREALSENSE LIBREALSENSE_LIBRARIES LIBREALSENSE_INCLUDE_DIRS)

endif (LIBREALSENSE_LIBRARIES AND LIBREALSENSE_INCLUDE_DIRS)
###############################################################################
# Find OpenNI2
#
#     find_package(OpenNI2)
#
# Variables defined by this module:
#
#  OpenNI2_FOUND               True if OpenNI2 was found
#  OpenNI2_INCLUDE_DIRS        The location(s) of OpenNI2 headers
#  OpenNI2_LIBRARIES           Libraries needed to use OpenNI2
#  OpenNI2_DEFINITIONS         Compiler flags for OpenNI2

find_package(PkgConfig QUIET)

#Search for libusb-1.0 beforehand
if(NOT libusb-1.0_FOUND)
  message(STATUS "OpenNI 2 disabled because libusb-1.0 not found.")
  return()
endif()

pkg_check_modules(PC_OpenNI2 QUIET libopenni2)

set(OpenNI2_DEFINITIONS ${PC_OpenNI2_CFLAGS_OTHER})

set(OpenNI2_SUFFIX)
if(WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(OpenNI2_SUFFIX 64)
endif()

find_path(OpenNI2_INCLUDE_DIR OpenNI.h
          PATHS "$ENV{OPENNI2_INCLUDE${OPENNI2_SUFFIX}}"  # Win64 needs '64' suffix
                "/usr/include/openni2"                    # common path for deb packages
          PATH_SUFFIXES include/openni2
)

find_library(OpenNI2_LIBRARY
             NAMES OpenNI2      # No suffix needed on Win64
                   libOpenNI2   # Linux
             PATHS "$ENV{OPENNI2_LIB${OPENNI2_SUFFIX}}"   # Windows default path, Win64 needs '64' suffix
                   "$ENV{OPENNI2_REDIST}"                 # Linux install does not use a separate 'lib' directory
)

if(OpenNI2_INCLUDE_DIR AND OpenNI2_LIBRARY)

  # Include directories
  set(OpenNI2_INCLUDE_DIRS ${OpenNI2_INCLUDE_DIR})
  unset(OpenNI2_INCLUDE_DIR)
  mark_as_advanced(OpenNI2_INCLUDE_DIRS)

  # Libraries
  if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(OpenNI2_LIBRARIES ${OpenNI2_LIBRARY} ${libusb-1.0_LIBRARIES})
  else()
    set(OpenNI2_LIBRARIES ${OpenNI2_LIBRARY})
  endif()
  unset(OpenNI2_LIBRARY)
  mark_as_advanced(OpenNI2_LIBRARIES)

  set(OpenNI2_REDIST_DIR $ENV{OpenNI2_REDIST${OpenNI2_SUFFIX}})
  mark_as_advanced(OpenNI2_REDIST_DIR)

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI2
  FOUND_VAR OpenNI2_FOUND
  REQUIRED_VARS OpenNI2_LIBRARIES OpenNI2_INCLUDE_DIRS
)

if(OpenNI2_FOUND)
  message(STATUS "OpenNI2 found (include: ${OpenNI2_INCLUDE_DIRS}, lib: ${OpenNI2_LIBRARIES})")
endif()

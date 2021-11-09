###############################################################################
# Find OpenNI2
#
#     find_package(OpenNI2)
#
# Variables defined by this module:
#
#  OPENNI2_FOUND               True if OpenNI2 was found
#  OPENNI2_INCLUDE_DIRS        The location(s) of OpenNI2 headers
#  OPENNI2_LIBRARIES           Libraries needed to use OpenNI2
#  OPENNI2_DEFINITIONS         Compiler flags for OpenNI2

find_package(PkgConfig QUIET)
pkg_check_modules(PC_OPENNI2 QUIET libopenni2)

set(OPENNI2_DEFINITIONS ${PC_OPENNI_CFLAGS_OTHER})

set(OPENNI2_SUFFIX)
if(WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(OPENNI2_SUFFIX 64)
endif()

find_path(OPENNI2_INCLUDE_DIR OpenNI.h
          PATHS "$ENV{OPENNI2_INCLUDE${OPENNI2_SUFFIX}}"  # Win64 needs '64' suffix
                "/usr/include/openni2"                    # common path for deb packages
          PATH_SUFFIXES include/openni2
)

find_library(OPENNI2_LIBRARY
             NAMES OpenNI2      # No suffix needed on Win64
                   libOpenNI2   # Linux
             PATHS "$ENV{OPENNI2_LIB${OPENNI2_SUFFIX}}"   # Windows default path, Win64 needs '64' suffix
                   "$ENV{OPENNI2_REDIST}"                 # Linux install does not use a separate 'lib' directory
)

if(OPENNI2_INCLUDE_DIR AND OPENNI2_LIBRARY)

  # Include directories
  set(OPENNI2_INCLUDE_DIRS ${OPENNI2_INCLUDE_DIR})
  unset(OPENNI2_INCLUDE_DIR)
  mark_as_advanced(OPENNI2_INCLUDE_DIRS)

  # Libraries
  if(NOT WIN32)
    find_package(libusb REQUIRED)
    set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY} libusb::libusb)
  else()
    set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY})
  endif()
  unset(OPENNI2_LIBRARY)
  mark_as_advanced(OPENNI2_LIBRARIES)

  set(OPENNI2_REDIST_DIR $ENV{OPENNI2_REDIST${OPENNI2_SUFFIX}})
  mark_as_advanced(OPENNI2_REDIST_DIR)

endif()

if(EXISTS "${OPENNI2_INCLUDE_DIR}/OniVersion.h")
  file(STRINGS "${OPENNI2_INCLUDE_DIR}/OniVersion.h" _contents REGEX "^#define[ \t]+ONI_VERSION_[A-Z]+[ \t]+[0-9]+")
  if(_contents)
    string(REGEX REPLACE ".*#define[ \t]+ONI_VERSION_MAJOR[ \t]+([0-9]+).*" "\\1" OPENNI2_VERSION_MAJOR "${_contents}")
    string(REGEX REPLACE ".*#define[ \t]+ONI_VERSION_MINOR[ \t]+([0-9]+).*" "\\1" OPENNI2_VERSION_MINOR "${_contents}")
    string(REGEX REPLACE ".*#define[ \t]+ONI_VERSION_MAINTENANCE[ \t]+([0-9]+).*" "\\1" OPENNI2_VERSION_PATCH "${_contents}")
    string(REGEX REPLACE ".*#define[ \t]+ONI_VERSION_BUILD[ \t]+([0-9]+).*" "\\1" OPENNI2_VERSION_BUILD "${_contents}")
    set(OPENNI2_VERSION "${OPENNI2_VERSION_MAJOR}.${OPENNI2_VERSION_MINOR}.${OPENNI2_VERSION_PATCH}.${OPENNI2_VERSION_BUILD}")
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI2
  FOUND_VAR OPENNI2_FOUND
  REQUIRED_VARS OPENNI2_LIBRARIES OPENNI2_INCLUDE_DIRS
  VERSION_VAR OPENNI2_VERSION
)

if(OPENNI2_FOUND)
  message(STATUS "OpenNI2 found (version: ${OPENNI2_VERSION}, include: ${OPENNI2_INCLUDE_DIRS}, lib: ${OPENNI2_LIBRARIES})")
endif()

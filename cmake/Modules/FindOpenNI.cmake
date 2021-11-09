###############################################################################
# Find OpenNI
#
#     find_package(OpenNI)
#
# Variables defined by this module:
#
#  OPENNI_FOUND                True if OpenNI was found
#  OPENNI_INCLUDE_DIRS         The location(s) of OpenNI headers
#  OPENNI_LIBRARIES            Libraries needed to use OpenNI
#  OPENNI_DEFINITIONS          Compiler flags for OpenNI

find_package(PkgConfig QUIET)
pkg_check_modules(PC_OPENNI QUIET libopenni)

set(OPENNI_DEFINITIONS ${PC_OPENNI_CFLAGS_OTHER})

set(OPENNI_SUFFIX)
if(WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(OPENNI_SUFFIX 64)
endif()

# Add a hint so that it can find it without the pkg-config
find_path(OPENNI_INCLUDE_DIR XnStatus.h
          HINTS ${PC_OPENNI_INCLUDEDIR}
                ${PC_OPENNI_INCLUDE_DIRS}
                /usr/include/openni
                /usr/include/ni
                /opt/local/include/ni
                "${OPENNI_ROOT}"
                "$ENV{OPENNI_ROOT}"
          PATHS "$ENV{OPEN_NI_INSTALL_PATH${OPENNI_SUFFIX}}/Include"
          PATH_SUFFIXES openni include Include)

# Add a hint so that it can find it without the pkg-config
find_library(OPENNI_LIBRARY
             NAMES OpenNI${OPENNI_SUFFIX}
             HINTS ${PC_OPENNI_LIBDIR}
                   ${PC_OPENNI_LIBRARY_DIRS}
                   /usr/lib
                   "${OPENNI_ROOT}"
                   "$ENV{OPENNI_ROOT}"
             PATHS "$ENV{OPEN_NI_LIB${OPENNI_SUFFIX}}"
             PATH_SUFFIXES lib Lib Lib64)

if(OPENNI_INCLUDE_DIR AND OPENNI_LIBRARY)

  # Include directories
  set(OPENNI_INCLUDE_DIRS ${OPENNI_INCLUDE_DIR})
  unset(OPENNI_INCLUDE_DIR)
  mark_as_advanced(OPENNI_INCLUDE_DIRS)

  # Libraries
  if(NOT WIN32)
    find_package(libusb REQUIRED)
    set(OPENNI_LIBRARIES ${OPENNI_LIBRARY} libusb::libusb)
  else()
    set(OPENNI_LIBRARIES ${OPENNI_LIBRARY})
  endif()
  unset(OPENNI_LIBRARY)
  mark_as_advanced(OPENNI_LIBRARIES)

endif()

if(EXISTS "${OPENNI_INCLUDE_DIR}/XnVersion.h")
  file(STRINGS "${OPENNI_INCLUDE_DIR}/XnVersion.h" _contents REGEX "^#define[ \t]+XN_[A-Z]+_VERSION[ \t]+[0-9]+")
  if(_contents)
    string(REGEX REPLACE ".*#define[ \t]+XN_MAJOR_VERSION[ \t]+([0-9]+).*" "\\1" OPENNI_VERSION_MAJOR "${_contents}")
    string(REGEX REPLACE ".*#define[ \t]+XN_MINOR_VERSION[ \t]+([0-9]+).*" "\\1" OPENNI_VERSION_MINOR "${_contents}")
    string(REGEX REPLACE ".*#define[ \t]+XN_MAINTENANCE_VERSION[ \t]+([0-9]+).*" "\\1" OPENNI_VERSION_PATCH "${_contents}")
    string(REGEX REPLACE ".*#define[ \t]+XN_BUILD_VERSION[ \t]+([0-9]+).*" "\\1" OPENNI_VERSION_BUILD "${_contents}")
    set(OPENNI_VERSION "${OPENNI_VERSION_MAJOR}.${OPENNI_VERSION_MINOR}.${OPENNI_VERSION_PATCH}.${OPENNI_VERSION_BUILD}")
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI
  FOUND_VAR OPENNI_FOUND
  REQUIRED_VARS OPENNI_LIBRARIES OPENNI_INCLUDE_DIRS
  VERSION_VAR OPENNI_VERSION
)

if(OPENNI_FOUND)
  message(STATUS "OpenNI found (version: ${OPENNI_VERSION}, include: ${OPENNI_INCLUDE_DIRS}, lib: ${OPENNI_LIBRARIES})")
endif()

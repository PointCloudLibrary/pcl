###############################################################################
# Find OpenNI
#
#     find_package(OpenNI)
#
# Variables defined by this module:
#
#  OpenNI_FOUND                True if OpenNI was found
#  OpenNI_INCLUDE_DIRS         The location(s) of OpenNI headers
#  OpenNI_LIBRARIES            Libraries needed to use OpenNI
#  OpenNI_DEFINITIONS          Compiler flags for OpenNI

find_package(PkgConfig QUIET)

#Search for libusb-1.0 beforehand
if(NOT libusb-1.0_FOUND)
  message(STATUS "OpenNI 2 disabled because libusb-1.0 not found.")
  return()
endif()

pkg_check_modules(PC_OpenNI QUIET libopenni)

set(OpenNI_DEFINITIONS ${PC_OpenNI_CFLAGS_OTHER})

set(OpenNI_SUFFIX)
if(WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(OpenNI_SUFFIX 64)
endif()

# Add a hint so that it can find it without the pkg-config
find_path(OpenNI_INCLUDE_DIR XnStatus.h
          HINTS ${PC_OpenNI_INCLUDEDIR}
                ${PC_OpenNI_INCLUDE_DIRS}
                /usr/include/openni
                /usr/include/ni
                /opt/local/include/ni
                "${OPENNI_ROOT}"
                "$ENV{OPENNI_ROOT}"
          PATHS "$ENV{OPEN_NI_INSTALL_PATH${OpenNI_SUFFIX}}/Include"
          PATH_SUFFIXES openni include Include)

# Add a hint so that it can find it without the pkg-config
find_library(OpenNI_LIBRARY
             NAMES OpenNI${OpenNI_SUFFIX}
             HINTS ${PC_OpenNI_LIBDIR}
                   ${PC_OpenNI_LIBRARY_DIRS}
                   /usr/lib
                   "${OPENNI_ROOT}"
                   "$ENV{OPENNI_ROOT}"
             PATHS "$ENV{OPEN_NI_LIB${OpenNI_SUFFIX}}"
             PATH_SUFFIXES lib Lib Lib64)

if(OpenNI_INCLUDE_DIR AND OpenNI_LIBRARY)

  # Include directories
  set(OpenNI_INCLUDE_DIRS ${OpenNI_INCLUDE_DIR})
  unset(OpenNI_INCLUDE_DIR)
  mark_as_advanced(OpenNI_INCLUDE_DIRS)

  # Libraries
  if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(OpenNI_LIBRARIES ${OpenNI_LIBRARY} ${libusb-1.0_LIBRARIES})
  else()
    set(OpenNI_LIBRARIES ${OpenNI_LIBRARY})
  endif()
  unset(OpenNI_LIBRARY)
  mark_as_advanced(OpenNI_LIBRARIES)

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI DEFAULT_MSG OpenNI_LIBRARIES OpenNI_INCLUDE_DIRS)

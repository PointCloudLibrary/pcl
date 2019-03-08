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

# Find LibUSB
if(NOT WIN32)
  pkg_check_modules(PC_USB_10 libusb-1.0)
  find_path(USB_10_INCLUDE_DIR libusb-1.0/libusb.h
            HINTS ${PC_USB_10_INCLUDEDIR} ${PC_USB_10_INCLUDE_DIRS} "${USB_10_ROOT}" "$ENV{USB_10_ROOT}"
            PATH_SUFFIXES libusb-1.0)

  find_library(USB_10_LIBRARY
               NAMES usb-1.0
               HINTS ${PC_USB_10_LIBDIR} ${PC_USB_10_LIBRARY_DIRS} "${USB_10_ROOT}" "$ENV{USB_10_ROOT}"
               PATH_SUFFIXES lib)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(USB_10 DEFAULT_MSG USB_10_LIBRARY USB_10_INCLUDE_DIR)

  if(NOT USB_10_FOUND)
    message(STATUS "OpenNI 2 disabled because libusb-1.0 not found.")
    return()
  else()
    include_directories(SYSTEM ${USB_10_INCLUDE_DIR})
  endif()
endif()

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
  if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY} ${LIBUSB_1_LIBRARIES})
  else()
    set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY})
  endif()
  unset(OPENNI2_LIBRARY)
  mark_as_advanced(OPENNI2_LIBRARIES)

  set(OPENNI2_REDIST_DIR $ENV{OPENNI2_REDIST${OPENNI2_SUFFIX}})
  mark_as_advanced(OPENNI2_REDIST_DIR)

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI2
  FOUND_VAR OPENNI2_FOUND
  REQUIRED_VARS OPENNI2_LIBRARIES OPENNI2_INCLUDE_DIRS
)

if(OPENNI2_FOUND)
  message(STATUS "OpenNI2 found (include: ${OPENNI2_INCLUDE_DIRS}, lib: ${OPENNI2_LIBRARIES})")
endif()

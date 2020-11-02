#.rst:
# Findlibusb
# --------
#
# Try to find libusb library and headers.
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the :prop_tgt:`IMPORTED` targets:
#
# ``libusb::libusb``
#  Defined if the system has libusb.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module sets the following variables:
#
# ::
#
#   LIBUSB_FOUND               True in case libusb is found, otherwise false
#   LIBUSB_ROOT                Path to the root of found libusb installation
#
# Example usage
# ^^^^^^^^^^^^^
#
# ::
#
#     find_package(libusb REQUIRED)
#
#     add_executable(foo foo.cc)
#     target_link_libraries(foo libusb::libusb)
#

# Early return if libusb target is already defined. This makes it safe to run
# this script multiple times.
if(TARGET libusb::libusb)
  return()
endif()

find_package(PkgConfig QUIET)
if(libusb_FIND_VERSION)
  pkg_check_modules(PC_LIBUSB libusb-1.0>=${libusb_FIND_VERSION})
else()
  pkg_check_modules(PC_LIBUSB libusb-1.0)
endif()

if(PC_LIBUSB_FOUND)
  set(LIBUSB_INCLUDE_DIR ${PC_LIBUSB_INCLUDEDIR})
  set(LIBUSB_LIBRARIES ${PC_LIBUSB_LIBRARIES})
else()
  find_path(LIBUSB_INCLUDE_DIR
    NAMES
      libusb-1.0/libusb.h
  )
  find_library(LIBUSB_LIBRARIES
    NAMES
      usb-1.0
      libusb
  )
endif()
  
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBUSB DEFAULT_MSG LIBUSB_LIBRARIES LIBUSB_INCLUDE_DIR)

mark_as_advanced(LIBUSB_INCLUDE_DIRS LIBUSB_LIBRARIES)

if(LIBUSB_FOUND)
  add_library(libusb::libusb INTERFACE IMPORTED)
  set_target_properties(libusb::libusb PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${LIBUSB_INCLUDE_DIR}")
  set_target_properties(libusb::libusb PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${PC_LIBUSB_CFLAGS_OTHER}")
  set_target_properties(libusb::libusb PROPERTIES INTERFACE_LINK_OPTIONS "${PC_LIBUSB_LDFLAGS_OTHER}")
  set_target_properties(libusb::libusb PROPERTIES INTERFACE_LINK_LIBRARIES "${LIBUSB_LIBRARIES}")
endif()

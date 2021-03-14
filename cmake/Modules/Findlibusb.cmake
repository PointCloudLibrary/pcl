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
  pkg_check_modules(PC_libusb libusb-1.0>=${libusb_FIND_VERSION})
else()
  pkg_check_modules(PC_libusb libusb-1.0)
endif()

find_path(libusb_INCLUDE_DIR
  NAMES
    libusb-1.0/libusb.h
  PATHS
    ${PC_libusb_INCLUDEDIR}
)

find_library(libusb_LIBRARIES
  NAMES
    usb-1.0
    libusb
  PATHS
    ${PC_libusb_LIBRARY_DIRS}
)
  
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libusb DEFAULT_MSG libusb_LIBRARIES libusb_INCLUDE_DIR)

mark_as_advanced(libusb_INCLUDE_DIRS libusb_LIBRARIES)

if(libusb_FOUND)
  add_library(libusb::libusb UNKNOWN IMPORTED)
  set_target_properties(libusb::libusb PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${libusb_INCLUDE_DIR}")
  set_target_properties(libusb::libusb PROPERTIES IMPORTED_LOCATION "${libusb_LIBRARIES}")
endif()

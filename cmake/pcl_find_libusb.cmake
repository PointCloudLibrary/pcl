#
#pcl_find_libusb is used due to VCPKG making impossible to use local findXX modules.
# and VCPKG findlibusb doesn't create the libusb targets.

# Find and set libusb
find_package(libusb)

if(TARGET libusb::libusb)
  #libusb target is found by the find_package script. 
  #VCPKG skip PCLs findlibusb and sets its own variables which is handled below.
  return()
endif()

#Handle VCPKG definitions
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libusb DEFAULT_MSG LIBUSB_LIBRARIES LIBUSB_INCLUDE_DIRS)

mark_as_advanced(LIBUSB_INCLUDE_DIRS LIBUSB_LIBRARIES)

if(libusb_FOUND)
  add_library(libusb::libusb UNKNOWN IMPORTED)
  if(${LIBUSB_INCLUDE_DIRS})
    set_target_properties(libusb::libusb PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${LIBUSB_INCLUDE_DIRS}")
  endif()
  if(EXISTS "${LIBUSB_LIBRARY}")
    set_target_properties(libusb::libusb PROPERTIES IMPORTED_LOCATION ${LIBUSB_LIBRARY})
  endif()
  if(EXISTS "${LIBUSB_LIBRARY_DEBUG}")
    set_target_properties(libusb::libusb PROPERTIES IMPORTED_LOCATION_DEBUG ${LIBUSB_LIBRARY_DEBUG})
  endif()
  if(EXISTS "${LIBUSB_LIBRARY_RELEASE}")
    set_target_properties(libusb::libusb PROPERTIES IMPORTED_LOCATION_RELEASE ${LIBUSB_LIBRARY_RELEASE})
  endif()
endif()

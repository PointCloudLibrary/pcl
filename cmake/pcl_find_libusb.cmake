# Find and set libusb
find_package(libusb)

if(libusb_FOUND)
  return()
endif()

#Handle VCPKG definitions
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libusb DEFAULT_MSG LIBUSB_LIBRARIES LIBUSB_INCLUDE_DIRS)

mark_as_advanced(LIBUSB_INCLUDE_DIRS LIBUSB_LIBRARIES)

if(LIBUSB_FOUND)
  add_library(libusb::libusb UNKNOWN IMPORTED)
  set_target_properties(libusb::libusb PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${LIBUSB_INCLUDE_DIRS}")
  
  foreach(LIBRARY_CONFIG ${LIBUSB_LIBRARIES})
   if("${LIBRARY_CONFIG}" MATCHES "^(debug|optimized)$")
     set(_CONF "${LIBRARY_CONFIG}")
   elseif("${_CONF}" MATCHES "^debug$")
     set_target_properties(libusb::libusb PROPERTIES IMPORTED_LOCATION_DEBUG "${LIBRARY_CONFIG}")
   else()
     set_target_properties(libusb::libusb PROPERTIES IMPORTED_LOCATION "${LIBRARY_CONFIG}")
   endif()
 endforeach()
endif()

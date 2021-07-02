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
  set_target_properties(libusb::libusb PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${LIBUSB_INCLUDE_DIRS}")
  
  if(LINUX)
    set_target_properties(libusb::libusb PROPERTIES IMPORTED_LOCATION "${libusb_LIBRARIES}")
  endif()
  
  if(WIN32)
    foreach(LIBRARY_CONFIG ${LIBUSB_LIBRARIES})
     if(LIBRARY_CONFIG MATCHES "^(debug|optimized)$")
       set(_CONF ${LIBRARY_CONFIG})
     elseif(_CONF MATCHES "^debug$")
       set_target_properties(libusb::libusb PROPERTIES IMPORTED_LOCATION_DEBUG "${LIBRARY_CONFIG}")
     else()
       set_target_properties(libusb::libusb PROPERTIES IMPORTED_LOCATION "${LIBRARY_CONFIG}")
     endif()
    endforeach()
  endif()
endif()

###############################################################################
# Find Intel Perceptual Computing SDK PXCAPI
#
# This sets the following variables:
# PXCAPI_FOUND - True if PXCAPI was found.
# PXCAPI_INCLUDE_DIRS - Directories containing the PXCAPI include files.
# PXCAPI_LIBRARIES - Libraries needed to use PXCAPI.
 
#MESSAGE("Searching for PXCAPI in: ${PXCAPI_DIR}")

if(PXCAPI_DIR)
  set(PXCAPI_INCLUDE_DIRS ${PXCAPI_DIR}/include ${PXCAPI_DIR}/sample/common/include)
  set(PXCAPI_LIB_DIRS ${PXCAPI_DIR}/lib/x64 ${PXCAPI_DIR}/sample/common/lib/x64/v100)
  set(PXCAPI_LIBS ${PXCAPI_DIR}/lib/x64/libpxc.lib ${PXCAPI_DIR}/sample/common/lib/x64/v100/libpxcutils.lib)
else()
  MESSAGE("Searching PXCAPI includes: ${PXCAPI_DIR}")
  find_path(PXCAPI_DIR include/pxcimage.h
    PATHS "${PXCAPI_DIR}" "C:/Program Files/Intel/PCSDK" "C:/Program Files (x86)/Intel/PCSDK"
    DOC "PXCAPI include directories")

  if(PXCAPI_DIR)
	set(PXCAPI_INCLUDE_DIRS ${PXCAPI_DIR}/include ${PXCAPI_DIR}/sample/common/include)
	set(PXCAPI_LIB_DIRS ${PXCAPI_DIR}/lib/x64 ${PXCAPI_DIR}/sample/common/lib/x64/v100)
	set(PXCAPI_LIBS ${PXCAPI_DIR}/lib/x64/libpxc.lib ${PXCAPI_DIR}/sample/common/lib/x64/v100/libpxcutils.lib)
  else()
	set(PXCAPI_DIR "directory not found (please enter)" CACHE FILEPATH "directory of PXCAPI")
  endif()	
endif()

MESSAGE("PXCAPI_INCLUDE_DIR: ${PXCAPI_INCLUDE_DIRS}")
MESSAGE("PXCAPI_LIBS: ${PXCAPI_LIBS}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PXCAPI DEFAULT_MSG
                                  PXCAPI_LIBS PXCAPI_INCLUDE_DIRS PXCAPI_LIB_DIRS)
							
if(MSVC)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /NODEFAULTLIB:LIBCMT")
endif()
							
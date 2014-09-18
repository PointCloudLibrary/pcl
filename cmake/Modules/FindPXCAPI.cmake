###############################################################################
# Find Intel Perceptual Computing SDK PXCAPI
#
# This sets the following variables:
# PXCAPI_FOUND - True if PXCAPI was found.
# PXCAPI_INCLUDE_DIRS - Directories containing the PXCAPI include files.
# PXCAPI_LIBRARIES - Libraries needed to use PXCAPI.

find_path(PXCAPI_DIR include/pxcimage.h
          PATHS "${PXCAPI_DIR}" "C:/Program Files/Intel/PCSDK" "C:/Program Files (x86)/Intel/PCSDK"
          DOC "PXCAPI include directories")

if(PXCAPI_DIR)
  set(PXCAPI_INCLUDE_DIRS ${PXCAPI_DIR}/include ${PXCAPI_DIR}/sample/common/include)
  if(MSVC10)
   set(msvc_version "v100")
   elseif(MSVC11)
    set(msvc_version "v110")
    elseif(MSVC12)
      set(msvc_version "v120")
    endif()
   endif()
  endif()
  find_library(PXCAPI_LIB libpxc.lib
               PATHS "${PXCAPI_DIR}/lib/" NO_DEFAULT_PATH
               PATH_SUFFIXES x64 Win32)
  find_library(PXCAPI_SAMPLE_LIB_RELEASE libpxcutils.lib
               PATHS "${PXCAPI_DIR}/sample/common/lib" NO_DEFAULT_PATH
               PATH_SUFFIXES "x64/${msvc_version}" "Win32/${msvc_version}")
               
  find_library(PXCAPI_SAMPLE_LIB_DEBUG libpxcutils_d.lib
               PATHS "${PXCAPI_DIR}/sample/common/lib" NO_DEFAULT_PATH
               PATH_SUFFIXES "x64/${msvc_version}" "Win32/${msvc_version}")
               
  set(PXCAPI_LIBS_RELEASE ${PXCAPI_LIB} ${PXCAPI_SAMPLE_LIB_RELEASE})
  set(PXCAPI_LIBS_DEBUG ${PXCAPI_LIB} ${PXCAPI_SAMPLE_LIB_DEBUG})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PXCAPI DEFAULT_MSG
                                  PXCAPI_LIBS_RELEASE PXCAPI_INCLUDE_DIRS)

mark_as_advanced(PXCAPI_LIB PXCAPI_SAMPLE_LIB_DEBUG PXCAPI_SAMPLE_LIB_RELEASE)

if(MSVC)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /NODEFAULTLIB:LIBCMT")
endif()

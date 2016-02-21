###############################################################################
# Find SoftKinetic DepthSense SDK
#
#     find_package(DSSDK)
#
# Variables defined by this module:
#
#  DSSDK_FOUND                 True if DepthSense SDK was found
#  DSSDK_VERSION               The version of DepthSense SDK
#  DSSDK_INCLUDE_DIRS          The location(s) of DepthSense SDK headers
#  DSSDK_LIBRARIES             Libraries needed to use DepthSense SDK

find_path(DSSDK_DIR include/DepthSenseVersion.hxx
          HINTS "$ENV{DEPTHSENSESDK32}"
                "$ENV{DEPTHSENSESDK64}"
          PATHS "$ENV{PROGRAMFILES}/SoftKinetic/DepthSenseSDK"
                "$ENV{PROGRAMW6432}/SoftKinetic/DepthSenseSDK"
                "C:/Program Files (x86)/SoftKinetic/DepthSenseSDK"
                "C:/Program Files/SoftKinetic/DepthSenseSDK"
                "/opt/softkinetic/DepthSenseSDK"
          DOC "DepthSense SDK directory")

if(DSSDK_DIR)

  # Include directories
  set(DSSDK_INCLUDE_DIRS ${DSSDK_DIR}/include)
  mark_as_advanced(DSSDK_INCLUDE_DIRS)

  # Libraries
  if(MSVC)
    set(DSSDK_LIBRARIES_NAMES DepthSense)
  else()
    set(DSSDK_LIBRARIES_NAMES DepthSense DepthSensePlugins turbojpeg)
  endif()
  foreach(LIB ${DSSDK_LIBRARIES_NAMES})
    find_library(DSSDK_LIBRARY_${LIB}
                 NAMES "${LIB}"
                 PATHS "${DSSDK_DIR}/lib/" NO_DEFAULT_PATH)
    list(APPEND DSSDK_LIBRARIES ${DSSDK_LIBRARY_${LIB}})
    mark_as_advanced(DSSDK_LIBRARY_${LIB})
  endforeach()

  # Version
  set(DSSDK_VERSION 0)
  file(STRINGS "${DSSDK_INCLUDE_DIRS}/DepthSenseVersion.hxx" _dsversion_H_CONTENTS REGEX "#define DEPTHSENSE_FILE_VERSION_STRING.*")
  set(_DSSDK_VERSION_REGEX "([0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+)")
  if("${_dsversion_H_CONTENTS}" MATCHES ".*#define DEPTHSENSE_FILE_VERSION_STRING .*${_DSSDK_VERSION_REGEX}.*")
    set(DSSDK_VERSION "${CMAKE_MATCH_1}")
  endif()
  unset(_dsversion_H_CONTENTS)

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DSSDK
  FOUND_VAR DSSDK_FOUND
  REQUIRED_VARS DSSDK_LIBRARIES DSSDK_INCLUDE_DIRS
  VERSION_VAR DSSDK_VERSION
)

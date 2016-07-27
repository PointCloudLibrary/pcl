###############################################################################
# Find Intel RealSense SDK
#
#     find_package(RSSDK)
#
# Variables defined by this module:
#
#  RSSDK_FOUND                 True if RealSense SDK was found
#  RSSDK_VERSION               The version of RealSense SDK
#  RSSDK_INCLUDE_DIRS          The location(s) of RealSense SDK headers
#  RSSDK_LIBRARIES             Libraries needed to use RealSense SDK

find_path(RSSDK_DIR include/pxcversion.h
          PATHS "$ENV{RSSDK_DIR}"
                "$ENV{PROGRAMFILES}/Intel/RSSDK"
                "$ENV{PROGRAMW6432}/Intel/RSSDK"
                "C:/Program Files (x86)/Intel/RSSDK"
                "C:/Program Files/Intel/RSSDK"
          DOC "RealSense SDK directory")

if(RSSDK_DIR)

  # Include directories
  set(RSSDK_INCLUDE_DIRS ${RSSDK_DIR}/include)
  mark_as_advanced(RSSDK_INCLUDE_DIRS)

  # Libraries
  set(RSSDK_RELEASE_NAME libpxc.lib)
  set(RSSDK_DEBUG_NAME libpxc_d.lib)
  set(RSSDK_SUFFIX Win32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(RSSDK_SUFFIX x64)
  endif()
  find_library(RSSDK_LIBRARY
               NAMES ${RSSDK_RELEASE_NAME}
               PATHS "${RSSDK_DIR}/lib/" NO_DEFAULT_PATH
               PATH_SUFFIXES ${RSSDK_SUFFIX})
  find_library(RSSDK_LIBRARY_DEBUG
               NAMES ${RSSDK_DEBUG_NAME} ${RSSDK_RELEASE_NAME}
               PATHS "${RSSDK_DIR}/lib/" NO_DEFAULT_PATH
               PATH_SUFFIXES ${RSSDK_SUFFIX})
  if(NOT RSSDK_LIBRARY_DEBUG)
    set(RSSDK_LIBRARY_DEBUG ${RSSDK_LIBRARY})
  endif()
  set(RSSDK_LIBRARIES optimized ${RSSDK_LIBRARY} debug ${RSSDK_LIBRARY_DEBUG})
  mark_as_advanced(RSSDK_LIBRARY RSSDK_LIBRARY_DEBUG)

  # Version
  set(RSSDK_VERSION 0)
  file(STRINGS "${RSSDK_INCLUDE_DIRS}/pxcversion.h" _pxcversion_H_CONTENTS REGEX "#define PXC_VERSION_.*")
  set(_RSSDK_VERSION_REGEX "([0-9]+)")
  foreach(v MAJOR MINOR BUILD REVISION)
    if("${_pxcversion_H_CONTENTS}" MATCHES ".*#define PXC_VERSION_${v} *${_RSSDK_VERSION_REGEX}.*")
      set(RSSDK_VERSION_${v} "${CMAKE_MATCH_1}")
    endif()
  endforeach()
  unset(_pxcversion_H_CONTENTS)
  set(RSSDK_VERSION "${RSSDK_VERSION_MAJOR}.${RSSDK_VERSION_MINOR}.${RSSDK_VERSION_BUILD}.${RSSDK_VERSION_REVISION}")

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RSSDK
  FOUND_VAR RSSDK_FOUND
  REQUIRED_VARS RSSDK_LIBRARIES RSSDK_INCLUDE_DIRS
  VERSION_VAR RSSDK_VERSION
)

if(MSVC)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /NODEFAULTLIB:LIBCMTD")
endif()

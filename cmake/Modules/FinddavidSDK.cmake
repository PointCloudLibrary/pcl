###############################################################################
# - Try to find davidSDK (David Vision Systems)
# Once done this will define
#  DAVIDSDK_FOUND - System has davidSDK
#  DAVIDSDK_INCLUDE_DIRS - The davidSDK include directories
#  DAVIDSDK_LIBRARIES - The libraries needed to use davidSDK
#  DAVIDSDK_DEFINITIONS - Compiler switches required for using davidSDK
# -----------------------

find_path(DAVIDSDK_INCLUDE_DIR david.h
          HINTS ${DAVIDSDK_ABI_HINT}
          /usr/local/include/davidSDK
          "$ENV{PROGRAMFILES}/davidSDK" "$ENV{PROGRAMW6432}/davidSDK"
          PATH_SUFFIXES include/)

find_library(DAVIDSDK_LIBRARY QUIET NAMES davidSDK
             HINTS ${DAVIDSDK_ABI_HINT}
             "$ENV{PROGRAMFILES}/davidSDK" "$ENV{PROGRAMW6432}/davidSDK"
             PATH_SUFFIXES lib/)

set(DAVIDSDK_LIBRARIES ${DAVIDSDK_LIBRARY})
set(DAVIDSDK_INCLUDE_DIRS ${DAVIDSDK_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set DAVIDSDK_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(davidSDK DEFAULT_MSG
                                  DAVIDSDK_LIBRARY DAVIDSDK_INCLUDE_DIR)

mark_as_advanced(DAVIDSDK_INCLUDE_DIR DAVIDSDK_LIBRARY)

if(DAVIDSDK_FOUND)
  message(STATUS "davidSDK found")
endif()


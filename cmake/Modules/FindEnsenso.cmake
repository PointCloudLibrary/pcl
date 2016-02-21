###############################################################################
# - Try to find Ensenso SDK (IDS-Imaging)
# Once done this will define
#  ENSENSO_FOUND - System has Ensenso SDK
#  ENSENSO_INCLUDE_DIRS - The Ensenso SDK include directories
#  ENSENSO_LIBRARIES - The libraries needed to use Ensenso SDK
#  ENSENSO_DEFINITIONS - Compiler switches required for using Ensenso SDK
# -----------------------

find_path(ENSENSO_INCLUDE_DIR nxLib.h
          HINTS ${ENSENSO_ABI_HINT}
          /opt/ensenso/development/c
          "$ENV{PROGRAMFILES}/Ensenso/development/c" "$ENV{PROGRAMW6432}/Ensenso/development/c"
          PATH_SUFFIXES include/)

find_library(ENSENSO_LIBRARY QUIET NAMES NxLib64 NxLib32 nxLib64 nxLib32
             HINTS ${ENSENSO_ABI_HINT}
             "$ENV{PROGRAMFILES}/Ensenso/development/c" "$ENV{PROGRAMW6432}/Ensenso/development/c"
             PATH_SUFFIXES lib/)

set(ENSENSO_LIBRARIES ${ENSENSO_LIBRARY})
set(ENSENSO_INCLUDE_DIRS ${ENSENSO_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ENSENSO_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(ensenso DEFAULT_MSG
                                  ENSENSO_LIBRARY ENSENSO_INCLUDE_DIR)

mark_as_advanced(ENSENSO_INCLUDE_DIR ENSENSO_LIBRARY)

if(ENSENSO_FOUND)
  message(STATUS "Ensenso SDK found")
endif(ENSENSO_FOUND)


###############################################################################
# Find Ensenso SDK (IDS-Imaging)
#
#     find_package(Ensenso)
#
# Variables defined by this module:
#
#  ENSENSO_FOUND               True if Ensenso SDK was found
#  ENSENSO_INCLUDE_DIRS        The location(s) of Ensenso SDK headers
#  ENSENSO_LIBRARIES           Libraries needed to use Ensenso SDK

find_path(ENSENSO_INCLUDE_DIR nxLib.h
          HINTS "${ENSENSO_ABI_HINT}"
                "/opt/ensenso/development/c"
                "$ENV{PROGRAMFILES}/Ensenso/development/c"
                "$ENV{PROGRAMW6432}/Ensenso/development/c"
          PATH_SUFFIXES include/)

find_library(ENSENSO_LIBRARY QUIET
             NAMES NxLib64 NxLib32 nxLib64 nxLib32
             HINTS "${ENSENSO_ABI_HINT}"
                   "$ENV{PROGRAMFILES}/Ensenso/development/c"
                   "$ENV{PROGRAMW6432}/Ensenso/development/c"
             PATH_SUFFIXES lib/)

if(ENSENSO_INCLUDE_DIR AND ENSENSO_LIBRARY)

  # Include directories
  set(ENSENSO_INCLUDE_DIRS ${ENSENSO_INCLUDE_DIR})
  unset(ENSENSO_INCLUDE_DIR)
  mark_as_advanced(ENSENSO_INCLUDE_DIRS)

  # Libraries
  set(ENSENSO_LIBRARIES ${ENSENSO_LIBRARY})
  unset(ENSENSO_LIBRARY)
  mark_as_advanced(ENSENSO_LIBRARIES)

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ensenso
  FOUND_VAR ENSENSO_FOUND
  REQUIRED_VARS ENSENSO_LIBRARIES ENSENSO_INCLUDE_DIRS
)

if(ENSENSO_FOUND)
  message(STATUS "Ensenso found (include: ${ENSENSO_INCLUDE_DIRS}, lib: ${ENSENSO_LIBRARIES})")
endif()

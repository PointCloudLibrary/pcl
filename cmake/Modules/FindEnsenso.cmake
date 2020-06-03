###############################################################################
# Find Ensenso SDK (IDS-Imaging)
#
#     find_package(Ensenso)
#
# Variables defined by this module:
#
#  Ensenso_FOUND               True if Ensenso SDK was found
#  Ensenso_INCLUDE_DIRS        The location(s) of Ensenso SDK headers
#  Ensenso_LIBRARIES           Libraries needed to use Ensenso SDK

find_path(Ensenso_INCLUDE_DIR nxLib.h
          HINTS "${ENSENSO_ABI_HINT}"
                "/opt/ensenso/development/c"
                "$ENV{PROGRAMFILES}/Ensenso/development/c"
                "$ENV{PROGRAMW6432}/Ensenso/development/c"
          PATH_SUFFIXES include/)

find_library(Ensenso_LIBRARY QUIET
             NAMES NxLib64 NxLib32 nxLib64 nxLib32
             HINTS "${ENSENSO_ABI_HINT}"
                   "$ENV{PROGRAMFILES}/Ensenso/development/c"
                   "$ENV{PROGRAMW6432}/Ensenso/development/c"
             PATH_SUFFIXES lib/)

if(Ensenso_INCLUDE_DIR AND Ensenso_LIBRARY)

  # Include directories
  set(Ensenso_INCLUDE_DIRS ${Ensenso_INCLUDE_DIR})
  unset(Ensenso_INCLUDE_DIR)
  mark_as_advanced(Ensenso_INCLUDE_DIRS)

  # Libraries
  set(Ensenso_LIBRARIES ${Ensenso_LIBRARY})
  unset(Ensenso_LIBRARY)
  mark_as_advanced(Ensenso_LIBRARIES)

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ensenso
  FOUND_VAR Ensenso_FOUND
  REQUIRED_VARS Ensenso_LIBRARIES Ensenso_INCLUDE_DIRS
)

if(Ensenso_FOUND)
  message(STATUS "Ensenso found (include: ${Ensenso_INCLUDE_DIRS}, lib: ${Ensenso_LIBRARIES})")
endif()

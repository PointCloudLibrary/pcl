###############################################################################
# Find OpenGM
#
# This sets the following variables:
#  OPENGM_FOUND - True if OpenGM was found.
#  OPENGM_INCLUDE_DIRS - Directories containing the OpenGM include files.

find_path(OPENGM_INCLUDE_DIR
          NAMES opengm/opengm.hxx
          HINTS "${OPENGM_ROOT}" "$ENV{OPENGM_ROOT}" "${OPENGM_INCLUDE_DIR}"
          PATHS "$ENV{PROGRAMFILES}/OpenGM" "$ENV{PROGRAMW6432}/OpenGM" 
          PATH_SUFFIXES opengm include )

set(OPENGM_INCLUDE_DIRS ${OPENGM_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenGM DEFAULT_MSG OPENGM_INCLUDE_DIR)


mark_as_advanced(OPENGM_INCLUDE_DIR OPENGM_ROOT_DIR)

if(OPENGM_FOUND)
  message(STATUS "OpenGM found (include: ${OPENGM_INCLUDE_DIRS})")
endif(OPENGM_FOUND)
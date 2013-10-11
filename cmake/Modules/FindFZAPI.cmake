###############################################################################
# Find Fotonic FZAPI
#
# This sets the following variables:
# FZAPI_FOUND - True if FZAPI was found.
# FZAPI_INCLUDE_DIRS - Directories containing the FZAPI include files.
# FZAPI_LIBRARIES - Libraries needed to use FZAPI.
 
if(FZ_API_DIR)
  # Find include dirs
  find_path(FZAPI_INCLUDE_DIR fz_api.h
    PATHS "${FZ_API_DIR}" NO_DEFAULT_PATH
    DOC "Fotonic include directories")
	
  # Find libraries
  find_library(FZAPI_LIBS fotonic_fz_api
    HINTS "${FZ_API_DIR}/Release" NO_DEFAULT_PATH
    DOC "Fotonic libraries")
else()
  set(FZ_API_DIR "default value" CACHE FILEPATH "directory of Fotonic API")	
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FZAPI DEFAULT_MSG
                                  FZAPI_LIBS FZAPI_INCLUDE_DIR)
								  
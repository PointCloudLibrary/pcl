###############################################################################
# Find Flann
#
# This sets the following variables:
# FLANN_FOUND - True if FLANN was found.
# FLANN_INCLUDE_DIRS - Directories containing the FLANN include files.
# FLANN_LIBRARIES - Libraries needed to use FLANN.
# FLANN_DEFINITIONS - Compiler flags for FLANN.
# If FLANN_USE_STATIC is specified and then look for static libraries ONLY else
# look for shared ones

if(FLANN_USE_STATIC)
  set(FLANN_RELEASE_NAME flann_cpp_s)
  set(FLANN_DEBUG_NAME flann_cpp_s-gd)
else(FLANN_USE_STATIC)
  set(FLANN_RELEASE_NAME flann_cpp)
  set(FLANN_DEBUG_NAME flann_cpp-gd)
endif(FLANN_USE_STATIC)

find_package(PkgConfig)
pkg_check_modules(PC_FLANN flann)
set(FLANN_DEFINITIONS ${PC_FLANN_CFLAGS_OTHER})

find_path(FLANN_INCLUDE_DIR flann/flann.hpp
          HINTS ${PC_FLANN_INCLUDEDIR} ${PC_FLANN_INCLUDE_DIRS} "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
          PATHS "$ENV{PROGRAMFILES}/Flann" "$ENV{PROGRAMW6432}/Flann" 
                "$ENV{PROGRAMFILES}/flann 1.6.9" "$ENV{PROGRAMW6432}/flann 1.6.9" 
          PATH_SUFFIXES include)

find_library(FLANN_LIBRARY
             NAMES ${FLANN_RELEASE_NAME}
             HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS} "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
             PATHS "$ENV{PROGRAMFILES}/Flann" "$ENV{PROGRAMW6432}/Flann" 
	           "$ENV{PROGRAMFILES}/flann 1.6.9" "$ENV{PROGRAMW6432}/flann 1.6.9" 
	     PATH_SUFFIXES lib)

find_library(FLANN_LIBRARY_DEBUG 
             NAMES ${FLANN_DEBUG_NAME} ${FLANN_RELEASE_NAME}
	     HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS} "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
	     PATHS "$ENV{PROGRAMFILES}/Flann" "$ENV{PROGRAMW6432}/Flann" 
	           "$ENV{PROGRAMFILES}/flann 1.6.9" "$ENV{PROGRAMW6432}/flann 1.6.9" 
	     PATH_SUFFIXES lib)

if(NOT FLANN_LIBRARY_DEBUG)
  set(FLANN_LIBRARY_DEBUG ${FLANN_LIBRARY})
endif(NOT FLANN_LIBRARY_DEBUG)

set(FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})
set(FLANN_LIBRARIES optimized ${FLANN_LIBRARY} debug ${FLANN_LIBRARY_DEBUG})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Flann DEFAULT_MSG FLANN_LIBRARY FLANN_INCLUDE_DIR)

mark_as_advanced(FLANN_LIBRARY FLANN_LIBRARY_DEBUG FLANN_INCLUDE_DIR)

if(FLANN_FOUND)
  message(STATUS "FLANN found (include: ${FLANN_INCLUDE_DIRS}, lib: ${FLANN_LIBRARIES})")
  if(FLANN_USE_STATIC)
    add_definitions(-DFLANN_STATIC)
  endif(FLANN_USE_STATIC)
endif(FLANN_FOUND)

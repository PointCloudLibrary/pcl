###############################################################################
# Find Flann
#
# This sets the following variables:
# FLANN_FOUND - True if FLANN was found.
# FLANN_INCLUDE_DIRS - Directories containing the FLANN include files.
# FLANN_LIBRARIES - Libraries needed to use FLANN.
# FLANN_DEFINITIONS - Compiler flags for FLANN.

find_package(PkgConfig)
pkg_check_modules(PC_FLANN flann)
set(FLANN_DEFINITIONS ${PC_FLANN_CFLAGS_OTHER})

find_path(FLANN_INCLUDE_DIR flann/flann.hpp
          HINTS ${PC_FLANN_INCLUDEDIR} ${PC_FLANN_INCLUDE_DIRS} "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
          PATHS "$ENV{PROGRAMFILES}/flann 1.6.8" "$ENV{PROGRAMW6432}/flann 1.6.8" 
          PATH_SUFFIXES include)

# Prefer static libraries in Windows over shared ones
if(WIN32)
  find_library(FLANN_LIBRARY
               NAMES flann_cpp_s flann_cpp
               HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS} "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
               PATHS "$ENV{PROGRAMFILES}/flann 1.6.8" "$ENV{PROGRAMW6432}/flann 1.6.8" 
               PATH_SUFFIXES lib)

  find_library(FLANN_LIBRARY_DEBUG 
               NAMES flann_cpp_s-gd flann_cpp-gd flann_cpp_s flann_cpp
               HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS} "${FLANN_ROOT} $ENV{FLANN_ROOT}"
               PATHS "$ENV{PROGRAMFILES}/flann 1.6.8" "$ENV{PROGRAMW6432}/flann 1.6.8" 
               PATH_SUFFIXES lib)
else(WIN32)
  find_library(FLANN_LIBRARY
               NAMES flann_cpp
               HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS} "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
               PATH_SUFFIXES lib)

  find_library(FLANN_LIBRARY_DEBUG 
               NAMES flann_cpp-gd flann_cpp
               HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS} "${FLANN_ROOT} $ENV{FLANN_ROOT}"
               PATH_SUFFIXES lib)
endif(WIN32)

if(NOT FLANN_LIBRARY_DEBUG)
    set(FLANN_LIBRARY_DEBUG ${FLANN_LIBRARY})
endif(NOT FLANN_LIBRARY_DEBUG)

set(FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})
set(FLANN_LIBRARIES optimized ${FLANN_LIBRARY} debug ${FLANN_LIBRARY_DEBUG})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Flann DEFAULT_MSG
    FLANN_LIBRARY FLANN_INCLUDE_DIR)

mark_as_advanced(FLANN_LIBRARY FLANN_LIBRARY_DEBUG FLANN_INCLUDE_DIR)

if(FLANN_FOUND)
    message(STATUS "FLANN found (include: ${FLANN_INCLUDE_DIRS}, lib: ${FLANN_LIBRARIES})")
endif(FLANN_FOUND)


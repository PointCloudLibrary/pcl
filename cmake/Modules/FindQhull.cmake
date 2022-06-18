#.rst:
# FindQhull
# --------
#
# Try to find QHULL library and headers. This module supports both old released versions
# of QHULL ≤ 7.3.2 and newer development versions that ship with a modern config file,
# but its limited to only the reentrant version of Qhull.
#
# PCL_QHULL_REQUIRED_TYPE can be used to select if you want static or shared libraries, but it defaults to "don't care".
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the :prop_tgt:`IMPORTED` targets:
#
# ``QHULL::QHULL``
#  Defined if the system has QHULL.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module sets the following variables:
#
# ::
#
#   QHULL_FOUND               True in case QHULL is found, otherwise false
#
# Example usage
# ^^^^^^^^^^^^^
#
# ::
#
#     find_package(QHULL REQUIRED)
#
#     add_executable(foo foo.cc)
#     target_link_libraries(foo QHULL::QHULL)
#

# Skip if QHULL::QHULL is already defined
if(TARGET QHULL::QHULL)
  return()
endif()

# Try to locate QHull using modern cmake config (available on latest Qhull version).
find_package(Qhull CONFIG QUIET)

if(Qhull_FOUND)
  unset(Qhull_FOUND)
  set(QHULL_FOUND ON)
  set(HAVE_QHULL ON)
  
  message(STATUS "Found Qhull version ${Qhull_VERSION}")
  
  # Create interface library that effectively becomes an alias for the appropriate (static/dynamic) imported QHULL target
  add_library(QHULL::QHULL INTERFACE IMPORTED)
  
  if(TARGET Qhull::qhull_r AND TARGET Qhull::qhullstatic_r)
    if(PCL_QHULL_REQUIRED_TYPE MATCHES "SHARED")
      set_property(TARGET QHULL::QHULL APPEND PROPERTY INTERFACE_LINK_LIBRARIES Qhull::qhull_r)
      set(QHULL_LIBRARY_TYPE SHARED)
    elseif(PCL_QHULL_REQUIRED_TYPE MATCHES "STATIC")
      set_property(TARGET QHULL::QHULL APPEND PROPERTY INTERFACE_LINK_LIBRARIES Qhull::qhullstatic_r)
      set(QHULL_LIBRARY_TYPE STATIC)
    else()
      if(PCL_SHARED_LIBS)
        set_property(TARGET QHULL::QHULL APPEND PROPERTY INTERFACE_LINK_LIBRARIES Qhull::qhull_r)
        set(QHULL_LIBRARY_TYPE SHARED)
      else()
        set_property(TARGET QHULL::QHULL APPEND PROPERTY INTERFACE_LINK_LIBRARIES Qhull::qhullstatic_r)
        set(QHULL_LIBRARY_TYPE STATIC)
      endif()
    endif()
  elseif(TARGET Qhull::qhullstatic_r)
    set_property(TARGET QHULL::QHULL APPEND PROPERTY INTERFACE_LINK_LIBRARIES Qhull::qhullstatic_r)
    set(QHULL_LIBRARY_TYPE STATIC)
  else()
    set_property(TARGET QHULL::QHULL APPEND PROPERTY INTERFACE_LINK_LIBRARIES Qhull::qhull_r)
    set(QHULL_LIBRARY_TYPE SHARED)  
  endif()
  
  return()
endif()

find_file(QHULL_HEADER
          NAMES libqhull_r.h
          HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}" "${QHULL_INCLUDE_DIR}"
          PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
          PATH_SUFFIXES qhull_r src/libqhull_r libqhull_r include)

set(QHULL_HEADER "${QHULL_HEADER}" CACHE INTERNAL "QHull header" FORCE )

if(QHULL_HEADER)
  get_filename_component(qhull_header ${QHULL_HEADER} NAME_WE)
  if("${qhull_header}" STREQUAL "qhull_r")
    get_filename_component(QHULL_INCLUDE_DIR ${QHULL_HEADER} PATH)
  elseif("${qhull_header}" STREQUAL "libqhull_r")
    get_filename_component(QHULL_INCLUDE_DIR ${QHULL_HEADER} PATH)
    get_filename_component(QHULL_INCLUDE_DIR ${QHULL_INCLUDE_DIR} PATH)
  endif()
else()
  set(QHULL_INCLUDE_DIR "QHULL_INCLUDE_DIR-NOTFOUND")
endif()

find_library(QHULL_LIBRARY_SHARED
             NAMES qhull_r qhull
             HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
             PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
             PATH_SUFFIXES project build bin lib)

find_library(QHULL_LIBRARY_DEBUG
             NAMES qhull_rd qhull_d
             HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
             PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
             PATH_SUFFIXES project build bin lib debug/lib)

find_library(QHULL_LIBRARY_STATIC
             NAMES qhullstatic_r
             HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
             PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
             PATH_SUFFIXES project build bin lib)

find_library(QHULL_LIBRARY_DEBUG_STATIC
             NAMES qhullstatic_rd
             HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
             PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
             PATH_SUFFIXES project build bin lib debug/lib)

if(QHULL_LIBRARY_SHARED AND QHULL_LIBRARY_STATIC)
  if(PCL_QHULL_REQUIRED_TYPE MATCHES "SHARED")
    set(QHULL_LIBRARY_TYPE SHARED)
    set(QHULL_LIBRARY ${QHULL_LIBRARY_SHARED})
  elseif(PCL_QHULL_REQUIRED_TYPE MATCHES "STATIC")
    set(QHULL_LIBRARY_TYPE STATIC)
    set(QHULL_LIBRARY ${QHULL_LIBRARY_STATIC})
  else()
    if(PCL_SHARED_LIBS)
      set(QHULL_LIBRARY_TYPE SHARED)
      set(QHULL_LIBRARY ${QHULL_LIBRARY_SHARED})
    else()
      set(QHULL_LIBRARY_TYPE STATIC)
      set(QHULL_LIBRARY ${QHULL_LIBRARY_STATIC})
    endif()
  endif()
elseif(QHULL_LIBRARY_STATIC)
  set(QHULL_LIBRARY_TYPE STATIC)
  set(QHULL_LIBRARY ${QHULL_LIBRARY_STATIC})
elseif(QHULL_LIBRARY_SHARED)
  set(QHULL_LIBRARY_TYPE SHARED)
  set(QHULL_LIBRARY ${QHULL_LIBRARY_SHARED})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Qhull
  FOUND_VAR QHULL_FOUND
  REQUIRED_VARS QHULL_LIBRARY QHULL_INCLUDE_DIR
)

if(QHULL_FOUND)
  set(HAVE_QHULL ON)
  add_library(QHULL::QHULL ${QHULL_LIBRARY_TYPE} IMPORTED)
  set_target_properties(QHULL::QHULL PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${QHULL_INCLUDE_DIR}")
  set_property(TARGET QHULL::QHULL APPEND PROPERTY IMPORTED_CONFIGURATIONS "RELEASE")
  set_target_properties(QHULL::QHULL PROPERTIES IMPORTED_LINK_INTERFACE_LANGUAGES "CXX")
  set_target_properties(QHULL::QHULL PROPERTIES INTERFACE_COMPILE_DEFINITIONS "qh_QHpointer")
  if(MSVC)
    set_target_properties(QHULL::QHULL PROPERTIES INTERFACE_COMPILE_DEFINITIONS "qh_QHpointer_dllimport")
  endif()
  if(WIN32 AND NOT (PCL_QHULL_REQUIRED_TYPE MATCHES "STATIC"))
    set_target_properties(QHULL::QHULL PROPERTIES IMPORTED_IMPLIB_RELEASE "${QHULL_LIBRARY}")
  else()
    set_target_properties(QHULL::QHULL PROPERTIES IMPORTED_LOCATION_RELEASE "${QHULL_LIBRARY}")
  endif()
  if(QHULL_LIBRARY_DEBUG)
    set_property(TARGET QHULL::QHULL APPEND PROPERTY IMPORTED_CONFIGURATIONS "DEBUG")
    if(WIN32 AND NOT (PCL_QHULL_REQUIRED_TYPE MATCHES "STATIC"))
      set_target_properties(QHULL::QHULL PROPERTIES IMPORTED_IMPLIB_DEBUG "${QHULL_LIBRARY_DEBUG}")
    else()
      set_target_properties(QHULL::QHULL PROPERTIES IMPORTED_LOCATION_DEBUG "${QHULL_LIBRARY_DEBUG}")
    endif()
  endif()
  message(STATUS "QHULL found (include: ${QHULL_INCLUDE_DIR}, lib: ${QHULL_LIBRARY})")
endif()

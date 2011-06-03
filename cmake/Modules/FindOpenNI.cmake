###############################################################################
# Find OpenNI
#
# This sets the following variables:
# OPENNI_FOUND - True if OPENNI was found.
# OPENNI_INCLUDE_DIRS - Directories containing the OPENNI include files.
# OPENNI_LIBRARIES - Libraries needed to use OPENNI.
# OPENNI_DEFINITIONS - Compiler flags for OPENNI.

find_package(PkgConfig)
if(${CMAKE_VERSION} VERSION_LESS 2.8.2)
  pkg_check_modules(PC_OPENNI openni-dev)
else()
  pkg_check_modules(PC_OPENNI QUIET openni-dev)
endif()

set(OPENNI_DEFINITIONS ${PC_OPENNI_CFLAGS_OTHER})

#add a hint so that it can find it without the pkg-config
find_path(OPENNI_INCLUDE_DIR XnStatus.h
          HINTS ${PC_OPENNI_INCLUDEDIR} ${PC_OPENNI_INCLUDE_DIRS} /usr/include/openni /usr/include/ni "${OPENNI_ROOT}" "$ENV{OPENNI_ROOT}"
          PATHS "$ENV{PROGRAMFILES}/OpenNI/Include" "$ENV{PROGRAMW6432}/OpenNI/Include"
          PATH_SUFFIXES openni include Include)
#add a hint so that it can find it without the pkg-config
find_library(OPENNI_LIBRARY 
             NAMES OpenNI64 OpenNI
             HINTS ${PC_OPENNI_LIBDIR} ${PC_OPENNI_LIBRARY_DIRS} /usr/lib "${OPENNI_ROOT}" "$ENV{OPENNI_ROOT}"
             PATHS "$ENV{PROGRAMFILES}/OpenNI/Lib" "$ENV{PROGRAMW6432}/OpenNI/Lib64"
			 PATH_SUFFIXES lib Lib Lib64)

set(OPENNI_INCLUDE_DIRS ${OPENNI_INCLUDE_DIR})
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  set(OPENNI_LIBRARIES ${OPENNI_LIBRARY} usb-1.0)
else()
  set(OPENNI_LIBRARIES ${OPENNI_LIBRARY})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI DEFAULT_MSG
    OPENNI_LIBRARY OPENNI_INCLUDE_DIR)
    
mark_as_advanced(OPENNI_LIBRARY OPENNI_INCLUDE_DIR)
if(OPENNI_FOUND)
  set(HAVE_OPENNI ON)
  include_directories(${OPENNI_INCLUDE_DIRS})
  message(STATUS "OpenNI found (include: ${OPENNI_INCLUDE_DIR}, lib: ${OPENNI_LIBRARY})")
endif(OPENNI_FOUND)


###############################################################################
# Find OpenNI
#
# This sets the following variables:
# OPENNI_FOUND - True if OPENNI was found.
# OPENNI_INCLUDE_DIRS - Directories containing the OPENNI include files.
# OPENNI_LIBRARIES - Libraries needed to use OPENNI.
# OPENNI_DEFINITIONS - Compiler flags for OPENNI.

find_package(PkgConfig)
#don't use QUIET
pkg_check_modules(PC_OPENNI openni-dev)
set(OPENNI_DEFINITIONS ${PC_OPENNI_CFLAGS_OTHER})

find_path(OPENNI_INCLUDE_DIR XnStatus.h
    HINTS ${PC_OPENNI_INCLUDEDIR} ${PC_OPENNI_INCLUDE_DIRS} 
    PATH_SUFFIXES openni)

find_library(OPENNI_LIBRARY OpenNI
    HINTS ${PC_OPENNI_LIBDIR} ${PC_OPENNI_LIBRARY_DIRS})

set(OPENNI_INCLUDE_DIRS ${OPENNI_INCLUDE_DIR})
set(OPENNI_LIBRARIES ${OPENNI_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI DEFAULT_MSG
    OPENNI_LIBRARY OPENNI_INCLUDE_DIR)

mark_as_advanced(OPENNI_LIBRARY OPENNI_INCLUDE_DIR)


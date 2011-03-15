###############################################################################
# Find CMinpack
#
# This sets the following variables:
# CMINPACK_FOUND - True if CMinpack was found.
# CMINPACK_INCLUDE_DIRS - Directories containing the CMinpack include files.
# CMINPACK_LIBRARIES - Libraries needed to use CMinpack.
# CMINPACK_DEFINITIONS - Compiler flags for CMinpack.

find_package(PkgConfig)
pkg_check_modules(PC_CMINPACK cminpack)
set(CMINPACK_DEFINITIONS ${PC_CMINPACK_CFLAGS_OTHER})

find_path(CMINPACK_INCLUDE_DIR cminpack.h
    HINTS ${PC_CMINPACK_INCLUDEDIR} ${PC_CMINPACK_INCLUDE_DIRS})

find_library(CMINPACK_LIBRARY cminpack
    HINTS ${PC_CMINPACK_LIBDIR} ${PC_CMINPACK_LIBRARY_DIRS})

set(CMINPACK_INCLUDE_DIRS ${CMINPACK_INCLUDE_DIR})
set(CMINPACK_LIBRARIES ${CMINPACK_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CMinpack DEFAULT_MSG
    CMINPACK_LIBRARY CMINPACK_INCLUDE_DIR)

mark_as_advanced(CMINPACK_LIBRARY CMINPACK_INCLUDE_DIR)


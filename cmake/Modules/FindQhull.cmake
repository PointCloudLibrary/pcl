###############################################################################
# Find Qhull
#
# This sets the following variables:
# Qhull_FOUND - True if Qhull was found.
# Qhull_INCLUDE_DIRS - Directories containing the Qhull include files.
# Qhull_LIBRARIES - Libraries needed to use Qhull.
# Qhull_DEFINITIONS - Compiler flags for Qhull.

find_path(Qhull_INCLUDE_DIR qhull/qhull.h)

find_library(Qhull_LIBRARY qhull)

set(Qhull_INCLUDE_DIRS ${Qhull_INCLUDE_DIR})
set(Qhull_LIBRARIES ${Qhull_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Qhull DEFAULT_MSG
    Qhull_LIBRARY Qhull_INCLUDE_DIR)

mark_as_advanced(Qhull_LIBRARY Qhull_INCLUDE_DIR)


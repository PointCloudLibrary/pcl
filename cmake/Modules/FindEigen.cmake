###############################################################################
# Find Eigen3
#
# This sets the following variables:
# EIGEN_FOUND - True if Eigen was found.
# EIGEN_INCLUDE_DIRS - Directories containing the Eigen include files.
# EIGEN_DEFINITIONS - Compiler flags for Eigen.

find_package(PkgConfig)
pkg_check_modules(PC_EIGEN eigen3)
set(EIGEN_DEFINITIONS ${PC_EIGEN_CFLAGS_OTHER})

find_path(EIGEN_INCLUDE_DIR Eigen/Core
    HINTS ${PC_EIGEN_INCLUDEDIR} ${PC_EIGEN_INCLUDE_DIRS}
    PATH_PREFIX eigen3)

set(EIGEN_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen DEFAULT_MSG EIGEN_INCLUDE_DIR)

mark_as_advanced(EIGEN_INCLUDE_DIR)


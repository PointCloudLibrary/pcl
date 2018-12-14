###############################################################################
# Find Sphinx
#
# This sets the following variables:
# SPHINX_FOUND - True if Sphinx was found.
# SPHINX_EXECUTABLE - Sphinx-build executable

find_package(PkgConfig QUIET)
pkg_check_modules(PC_SPHINX sphinx-build)

find_package(PythonInterp)

if(PYTHONINTERP_FOUND)
  get_filename_component(PYTHON_DIR "${PYTHON_EXECUTABLE}" PATH)
endif()

find_program(SPHINX_EXECUTABLE NAMES sphinx-build
             HINTS ${PC_SPHINX_EXECUTABLE} $ENV{SPHINX_DIR} ${PYTHON_DIR}/Scripts
             PATH_SUFFIXES bin
             DOC "Sphinx documentation generator"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Sphinx DEFAULT_MSG SPHINX_EXECUTABLE)

mark_as_advanced(SPHINX_EXECUTABLE)

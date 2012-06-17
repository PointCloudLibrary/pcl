###############################################################################
# Find QVTK
#
# This sets the following variables:
# QVTK_FOUND - True if QVTK was found.
# QVTK_INCLUDE_DIR - Directory containing the QVTK include files.
# QVTK_LIBRARY - QVTK library.
# if QVTK_FOUND then QVTK_INCLUDE_DIR is appended to VTK_INCLUDE_DIRS and 
# QVTK_LIBRARY is appended to QVTK_LIBRARY_DIR

find_library (QVTK_LIBRARY QVTK HINTS ${VTK_DIR} ${VTK_DIR}/bin)
find_path (QVTK_INCLUDE_DIR QVTKWidget.h HINT ${VTK_INCLUDE_DIRS})
find_package_handle_standard_args(QVTK DEFAULT_MSG
  QVTK_LIBRARY QVTK_INCLUDE_DIR)

if(NOT QVTK_FOUND)
  set (VTK_USE_QVTK OFF)
else(NOT QVTK_FOUND)
  get_filename_component (QVTK_LIBRARY_DIR ${QVTK_LIBRARY} PATH)
  set (VTK_LIBRARY_DIRS ${VTK_LIBRARY_DIRS} ${QVTK_LIBRARY_DIR})
  set (VTK_INCLUDE_DIRS ${VTK_INCLUDE_DIRS} ${QVTK_INCLUDE_DIR})
  set (VTK_USE_QVTK ON)
endif(NOT QVTK_FOUND)

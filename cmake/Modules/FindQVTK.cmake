###############################################################################
# Find QVTK
#
# This sets the following variables:
# QVTK_FOUND - True if QVTK was found.
# QVTK_INCLUDE_DIR - Directory containing the QVTK include files.
# QVTK_LIBRARY - QVTK library.
# if QVTK_FOUND then QVTK_INCLUDE_DIR is appended to VTK_INCLUDE_DIRS and
# QVTK_LIBRARY is appended to QVTK_LIBRARY_DIR
if(";${VTK_MODULES_ENABLED};" MATCHES ";vtkGUISupportQt;" AND ";${VTK_MODULES_ENABLED};" MATCHES ";vtkRenderingQt;")
  set(VTK_USE_QVTK ON)
  set(QVTK_LIBRARY vtkRenderingQt vtkGUISupportQt)
else()
  unset(QVTK_FOUND)
endif()

set(SUBSUBSYS_NAME optronic_viewer)
set(SUBSUBSYS_DESC "PCL Optronic Viewer")
set(SUBSUBSYS_DEPS common geometry io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search apps)

# Find VTK and QVTK
if(VTK_FOUND AND VTK_USE_QVTK)
  set(DEFAULT TRUE)
  set(REASON)
  set(VTK_USE_FILE "${VTK_USE_FILE}" CACHE INTERNAL "VTK_USE_FILE")
  include("${VTK_USE_FILE}")
elseif(NOT VTK_FOUND)
  set(DEFAULT AUTO_OFF)
  set(REASON "VTK was not found.")
elseif(NOT VTK_USE_QVTK)
  set(DEFAULT AUTO_OFF)
  set(REASON "VTK was not built with Qt support.")
endif()

# QT5 Found?
if(NOT Qt5_FOUND)
  set(DEFAULT AUTO_OFF)
  set(REASON "Qt5 was not found.")
elseif(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
  set(DEFAULT TRUE)
  set(REASON)
endif()

# FZAPI
if(FZAPI_FOUND)
  set(DEFAULT TRUE)
  set(REASON)
elseif(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
  set(DEFAULT AUTO_OFF)
  set(REASON "FZAPI was not found.")
endif()

# Default to not building for now
if(${DEFAULT} STREQUAL "TRUE")
  set(DEFAULT FALSE)
endif()

PCL_SUBSUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSUBSYS_DEPEND(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" DEPS ${SUBSUBSYS_DEPS} EXT_DEPS vtk)

PCL_ADD_DOC("${SUBSUBSYS_NAME}")

if(NOT build)
  return()
endif()

include_directories("${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/include")

# Set Qt files and resources here
set(moc_incs
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/main_window.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/filter_window.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/openni_grabber.h"
)

set(incs
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/qt.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/openni_grabber.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/cloud_filter.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/main_window.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/filter_window.h"
)

set(srcs
  src/main.cpp
  src/cloud_filter.cpp
  src/openni_grabber.cpp
  src/filter_window.cpp
  src/main_window.cpp
)

set(impl_incs)

# Qt stuff
QT5_WRAP_CPP(moc_srcs ${moc_incs} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

# Generate executable
set(EXE_NAME "pcl_${SUBSUBSYS_NAME}")
PCL_ADD_EXECUTABLE("${EXE_NAME}" "${SUBSUBSYS_NAME}" ${moc_srcs} ${srcs} ${incs} ${impl_incs})
target_link_libraries("${EXE_NAME}" pcl_common pcl_io pcl_kdtree pcl_filters pcl_visualization pcl_segmentation pcl_surface pcl_features pcl_sample_consensus pcl_search QVTK Qt5::Widgets)

# Put the ui in the windows project file
if(("${CMAKE_BUILD_TOOL}" MATCHES "msdev") OR("${CMAKE_BUILD_TOOL}" MATCHES "devenv"))
  list(APPEND srcs ${uis})
endif()

# Install include files
PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSUBSYS_NAME}/impl" ${impl_incs})

PCL_MAKE_PKGCONFIG("${EXE_NAME}" "${SUBSUBSYS_NAME}" "${SUBSUBSYS_DESC}" "" "" "" "" "")

# Add to the compound apps target
list(APPEND PCL_APPS_ALL_TARGETS ${EXE_NAME})
set(PCL_APPS_ALL_TARGETS ${PCL_APPS_ALL_TARGETS} PARENT_SCOPE)

set(SUBSUBSYS_NAME modeler)
set(SUBSUBSYS_DESC "PCLModeler: PCL based reconstruction platform")
set(SUBSUBSYS_DEPS common geometry io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search apps)
set(REASON "")

# Find VTK
if(NOT VTK_FOUND)
  set(DEFAULT AUTO_OFF)
  set(REASON "VTK was not found.")
else()
  set(DEFAULT TRUE)
  set(REASON)
  set(VTK_USE_FILE "${VTK_USE_FILE}" CACHE INTERNAL "VTK_USE_FILE")
  include("${VTK_USE_FILE}")
endif()

# QT5 Found?
if(NOT Qt5_FOUND)
  set(DEFAULT AUTO_OFF)
  set(REASON "Qt5 was not found.")
elseif(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
  set(DEFAULT TRUE)
  set(REASON)
endif()

# QVTK?
if(NOT QVTK_FOUND)
  set(DEFAULT AUTO_OFF)
  set(REASON "VTK was not built with Qt support.")
elseif(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
  set(DEFAULT TRUE)
  set(REASON)
endif()

# Default to not building for now
if(${DEFAULT} STREQUAL "TRUE")
  set(DEFAULT FALSE)
endif()

PCL_SUBSUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSUBSYS_DEPEND(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" DEPS ${SUBSYS_DEPS} EXT_DEPS vtk)

PCL_ADD_DOC("${SUBSUBSYS_NAME}")

if(NOT build)
  return()
endif()

include_directories("${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/include")

# Set Qt files and resources here
set(uis
  main_window.ui
)

set(moc_incs
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/main_window.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/scene_tree.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/parameter_dialog.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/thread_controller.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/abstract_worker.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/cloud_mesh_item_updater.h"
)

set(resources
  resources/resources.qrc
)

set(incs
  ${moc_incs}

  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/dock_widget.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/abstract_item.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/render_window.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/render_window_item.h"

  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/parameter.h"

  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/cloud_mesh.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/cloud_mesh_item.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/channel_actor_item.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/points_actor_item.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/normals_actor_item.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/surface_actor_item.h"

  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/icp_registration_worker.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/voxel_grid_downsample_worker.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/statistical_outlier_removal_worker.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/normal_estimation_worker.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/poisson_worker.h"
)

set(srcs
  src/main.cpp

  src/main_window.cpp
  src/dock_widget.cpp
  src/abstract_item.cpp
  src/render_window.cpp
  src/render_window_item.cpp

  src/parameter.cpp
  src/parameter_dialog.cpp

  src/scene_tree.cpp

  src/cloud_mesh.cpp
  src/cloud_mesh_item.cpp
  src/cloud_mesh_item_updater.cpp
  src/channel_actor_item.cpp
  src/points_actor_item.cpp
  src/normals_actor_item.cpp
  src/surface_actor_item.cpp

  src/thread_controller.cpp
  src/abstract_worker.cpp
  src/icp_registration_worker.cpp
  src/voxel_grid_downsample_worker.cpp
  src/statistical_outlier_removal_worker.cpp
  src/normal_estimation_worker.cpp
  src/poisson_worker.cpp
)

set(impl_incs
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/impl/parameter.hpp"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/impl/scene_tree.hpp"
)

# Qt stuff
QT5_WRAP_UI(ui_srcs ${uis})
QT5_WRAP_CPP(moc_srcs ${moc_incs} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
QT5_ADD_RESOURCES(resource_srcs ${resources})

# Organize files
source_group("Resources" FILES ${uis} ${resources} ${EXE_ICON})
source_group("Generated" FILES ${ui_srcs} ${moc_srcs} ${resource_srcs} ${RCS_SOURCES})
set_source_files_properties(${srcs} PROPERTIES OBJECT_DEPENDS "${ui_srcs}")

# Generate executable
set(EXE_NAME "pcl_${SUBSUBSYS_NAME}")
PCL_ADD_EXECUTABLE(${EXE_NAME} COMPONENT ${SUBSUBSYS_NAME} SOURCES ${ui_srcs} ${moc_srcs} ${resource_srcs} ${srcs} ${incs} ${impl_incs})
target_link_libraries("${EXE_NAME}" pcl_common pcl_io pcl_kdtree pcl_filters pcl_visualization pcl_segmentation pcl_surface pcl_features pcl_sample_consensus pcl_search Qt5::Widgets)

# Put the ui in the windows project file
if(("${CMAKE_BUILD_TOOL}" MATCHES "msdev") OR("${CMAKE_BUILD_TOOL}" MATCHES "devenv"))
  list(APPEND srcs ${uis})
endif()

# Install include files
PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSUBSYS_NAME}/impl" ${impl_incs})

PCL_MAKE_PKGCONFIG(${EXE_NAME} COMPONENT ${SUBSUBSYS_NAME} DESC ${SUBSUBSYS_DESC})

# Add to the compound apps target
list(APPEND PCL_APPS_ALL_TARGETS ${EXE_NAME})
set(PCL_APPS_ALL_TARGETS ${PCL_APPS_ALL_TARGETS} PARENT_SCOPE)

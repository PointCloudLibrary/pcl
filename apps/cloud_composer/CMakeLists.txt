#Disable some warnings which are all over Qt headers and make finding error messages hell
#if(${CMAKE_CXX_FLAGS} MATCHES "-Wconversion")
#    string(REGEX REPLACE "-Wconversion(.+)" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
#endif()

set(SUBSUBSYS_NAME cloud_composer)
set(SUBSUBSYS_DESC "Cloud Composer - Application for Manipulating Point Clouds")
set(SUBSUBSYS_DEPS common io visualization filters apps)
set(SUBSUBSYS_EXT_DEPS vtk ${QTX})

# QVTK?
if(NOT HAVE_QVTK)
  set(DEFAULT AUTO_OFF)
  set(REASON "Cloud composer requires QVTK")
elseif(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
  set(DEFAULT TRUE)
  set(REASON)
endif()

#Default to not building for now
if("${DEFAULT}" STREQUAL "TRUE")
  set(DEFAULT FALSE)
endif()

PCL_SUBSUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSUBSYS_NAME} PARENT_NAME ${SUBSYS_NAME} DEPS ${SUBSUBSYS_DEPS} EXT_DEPS ${SUBSUBSYS_EXT_DEPS})

PCL_ADD_DOC(${SUBSUBSYS_NAME})

if(NOT build)
  return()
endif()

include_directories("${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/include")

#Create subdirectory for plugin libs
set(CLOUD_COMPOSER_PLUGIN_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cloud_composer_plugins")
make_directory("${CLOUD_COMPOSER_PLUGIN_DIR}")

set(INTERFACE_HEADERS
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/commands.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/properties_model.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/items/cloud_composer_item.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/items/cloud_item.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/items/fpfh_item.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/items/normals_item.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/point_selectors/click_trackball_interactor_style.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/point_selectors/interactor_style_switch.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/point_selectors/manipulation_event.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/point_selectors/rectangular_frustum_selector.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/point_selectors/selected_trackball_interactor_style.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/point_selectors/selection_event.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tool_interface/abstract_tool.h"
)

set(INTERFACE_SOURCES
  src/commands.cpp
  src/properties_model.cpp
  src/items/cloud_composer_item.cpp
  src/items/cloud_item.cpp
  src/items/fpfh_item.cpp
  src/items/normals_item.cpp
  src/point_selectors/click_trackball_interactor_style.cpp
  src/point_selectors/interactor_style_switch.cpp
  src/point_selectors/manipulation_event.cpp
  src/point_selectors/rectangular_frustum_selector.cpp
  src/point_selectors/selected_trackball_interactor_style.cpp
  src/point_selectors/selection_event.cpp
  src/tool_interface/abstract_tool.cpp
)

# Build pcl_cc_tool_interface as static library, to fix issue mentioned in #2708
set(PCL_LIB_TYPE_ORIGIN ${PCL_LIB_TYPE})
set(PCL_LIB_TYPE STATIC)

PCL_ADD_LIBRARY(pcl_cc_tool_interface COMPONENT ${SUBSUBSYS_NAME} SOURCES ${INTERFACE_HEADERS} ${INTERFACE_SOURCES})


target_link_libraries(pcl_cc_tool_interface pcl_common pcl_filters pcl_search pcl_visualization ${QTX}::Widgets)

set(PCL_LIB_TYPE ${PCL_LIB_TYPE_ORIGIN})

if(APPLE)
  set_target_properties(pcl_cc_tool_interface PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif()

#Sources & Headers for main application
set(incs
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/cloud_browser.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/cloud_composer.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/cloud_view.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/cloud_viewer.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/item_inspector.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/merge_selection.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/project_model.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/signal_multiplexer.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/toolbox_model.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/transform_clouds.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/work_queue.h"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tool_interface/tool_factory.h"
)

set(srcs
  src/cloud_browser.cpp
  src/cloud_composer.cpp
  src/cloud_view.cpp
  src/cloud_viewer.cpp
  src/item_inspector.cpp
  src/main.cpp
  src/merge_selection.cpp
  src/project_model.cpp
  src/properties_model.cpp
  src/signal_multiplexer.cpp  
  src/toolbox_model.cpp
  src/transform_clouds.cpp
  src/work_queue.cpp
)

set(impl_incs
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/impl/cloud_item.hpp"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/impl/merge_selection.hpp"
  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/impl/transform_clouds.hpp"
)

set(uis src/cloud_composer_main_window.ui)
set(resources resources/resources.qrc)

list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS "src")

set(EXE_NAME "pcl_${SUBSUBSYS_NAME}")
PCL_ADD_EXECUTABLE(${EXE_NAME} COMPONENT ${SUBSUBSYS_NAME} SOURCES ${uis} ${incs} ${srcs} ${resources} ${impl_incs})
target_link_libraries("${EXE_NAME}" pcl_cc_tool_interface pcl_common pcl_io pcl_visualization pcl_filters ${QTX}::Widgets)



# Install include files
PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSUBSYS_NAME}" ${incs} ${item_incs} ${selector_incs})
PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSUBSYS_NAME}/impl" ${impl_incs})

PCL_MAKE_PKGCONFIG(${EXE_NAME} COMPONENT ${SUBSUBSYS_NAME} DESC ${SUBSYS_DESC})


#TOOL buildING SCRIPTS
include(ComposerTool.cmake REQUIRED)

#FPFH Tool
set(FPFH_DEPS pcl_features pcl_kdtree pcl_filters)
set(FPFH_SOURCES tools/fpfh_estimation.cpp)
set(FPFH_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/fpfh_estimation.h")
define_composer_tool (fpfh_estimation "${FPFH_SOURCES}" "${FPFH_HEADERS}" "${FPFH_DEPS}")

#Normals Tool
set(NORMALS_DEPS pcl_features pcl_kdtree)
set(NORMALS_SOURCES tools/normal_estimation.cpp)
set(NORMALS_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/normal_estimation.h")
define_composer_tool (normal_estimation "${NORMALS_SOURCES}" "${NORMALS_HEADERS}" "${NORMALS_DEPS}")

#Euclidean Clustering Tool
set(EC_DEPS pcl_segmentation pcl_kdtree)
set(EC_SOURCES tools/euclidean_clustering.cpp)
set(EC_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/euclidean_clustering.h")
define_composer_tool (euclidean_clustering "${EC_SOURCES}" "${EC_HEADERS}" "${EC_DEPS}")

#Statistical Outlier Removal Tool
set(SOR_DEPS pcl_filters)
set(SOR_SOURCES tools/statistical_outlier_removal.cpp)
set(SOR_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/statistical_outlier_removal.h")
define_composer_tool (statistical_outlier_removal "${SOR_SOURCES}" "${SOR_HEADERS}" "${SOR_DEPS}")

#Vox Grid Downsample Tool
set(VOXDS_DEPS pcl_filters)
set(VOXDS_SOURCES tools/voxel_grid_downsample.cpp)
set(VOXDS_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/voxel_grid_downsample.h")
define_composer_tool (voxel_grid_downsample "${VOXDS_SOURCES}" "${VOXDS_HEADERS}" "${VOXDS_DEPS}")

#Organized Segmentation
set(OSEG_DEPS pcl_segmentation pcl_kdtree)
set(OSEG_SOURCES tools/organized_segmentation.cpp "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/impl/organized_segmentation.hpp")
set(OSEG_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/organized_segmentation.h")
define_composer_tool (organized_segmentation "${OSEG_SOURCES}" "${OSEG_HEADERS}" "${OSEG_DEPS}")

#Sanitize Cloud Tool
set(SAN_DEPS pcl_filters)
set(SAN_SOURCES tools/sanitize_cloud.cpp)
set(SAN_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/sanitize_cloud.h")
define_composer_tool (sanitize_cloud "${SAN_SOURCES}" "${SAN_HEADERS}" "${SAN_DEPS}")

#Supervoxels
set(VSP_DEPS pcl_octree pcl_segmentation)
set(VSP_SOURCES tools/supervoxels.cpp "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/impl/supervoxels.hpp")
set(VSP_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/supervoxels.h")
define_composer_tool (supervoxels "${VSP_SOURCES}" "${VSP_HEADERS}" "${VSP_DEPS}")

# Add to the compound apps target
list(APPEND PCL_APPS_ALL_TARGETS ${EXE_NAME})
set(PCL_APPS_ALL_TARGETS ${PCL_APPS_ALL_TARGETS} PARENT_SCOPE)

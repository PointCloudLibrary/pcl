#Disable some warnings which are all over Qt headers and make finding error messages hell
#if(${CMAKE_CXX_FLAGS} MATCHES "-Wconversion") 
#  string(REGEX REPLACE "-Wconversion(.+)" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
#endif()

SET(SUBSUBSYS_NAME cloud_composer)
SET(SUBSUBSYS_DESC "Cloud Composer - Application for Manipulating Point Clouds")
SET(SUBSUBSYS_DEPS common io visualization filters apps)

# Find VTK 
if(NOT VTK_FOUND)
  set(DEFAULT AUTO_OFF)
  set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
  set(DEFAULT TRUE)
  set(REASON)
  include("${VTK_USE_FILE}")
endif(NOT VTK_FOUND)

# QT4 Found?
if(NOT QT4_FOUND)
  set(DEFAULT AUTO_OFF)
  set(REASON "Qt4 was not found.")
elseif(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
  set(DEFAULT TRUE)
  set(REASON)
endif(NOT QT4_FOUND)

# QVTK?
if(NOT VTK_USE_QVTK)
  set(DEFAULT AUTO_OFF)
  set(REASON "Cloud composer requires QVTK")
elseif(NOT ${DEFAULT} STREQUAL "AUTO_OFF")
  set(DEFAULT TRUE)
  set(REASON)
endif(NOT VTK_USE_QVTK)

#Default to not building for now
if ("${DEFAULT}" STREQUAL "TRUE")
  set(DEFAULT FALSE)
endif ("${DEFAULT}" STREQUAL "TRUE")

PCL_SUBSUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSUBSYS_DEPEND(build "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" DEPS ${SUBSUBSYS_DEPS})

PCL_ADD_DOC(${SUBSUBSYS_NAME})

if(build)

    include_directories("${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/include")
  
    #Sources & Headers for main application
    set(incs "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/qt.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/cloud_composer.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/project_model.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/cloud_viewer.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/cloud_view.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/cloud_browser.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/item_inspector.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tool_interface/abstract_tool.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tool_interface/tool_factory.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/commands.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/work_queue.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/toolbox_model.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/properties_model.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/signal_multiplexer.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/merge_selection.h"
             "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/transform_clouds.h")
          
    set(srcs src/main.cpp 
             src/cloud_composer.cpp
             src/project_model.cpp
             src/cloud_viewer.cpp
             src/cloud_view.cpp
             src/item_inspector.cpp
             src/cloud_browser.cpp
             src/commands.cpp
             src/work_queue.cpp
             src/toolbox_model.cpp
             src/properties_model.cpp
             src/signal_multiplexer.cpp
             src/merge_selection.cpp
             src/tool_interface/abstract_tool.cpp
             src/transform_clouds.cpp)
          
    set(impl_incs "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/impl/cloud_item.hpp"
                  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/impl/merge_selection.hpp"
                  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/impl/transform_clouds.hpp")
              
    #Sources and headers for item types      
    set(item_incs "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/items/cloud_composer_item.h"
                  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/items/cloud_item.h"
                  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/items/normals_item.h"
                  "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/items/fpfh_item.h")
           
    set(item_srcs src/items/cloud_composer_item.cpp
                  src/items/cloud_item.cpp
                  src/items/normals_item.cpp
                  src/items/fpfh_item.cpp)
                  
    #Sources and headers for point selectors
    set (selector_incs "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/point_selectors/interactor_style_switch.h"
                       "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/point_selectors/rectangular_frustum_selector.h"
                       "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/point_selectors/selected_trackball_interactor_style.h"
                       "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/point_selectors/click_trackball_interactor_style.h")
                       
    set (selector_srcs src/point_selectors/interactor_style_switch.cpp
                       src/point_selectors/selection_event.cpp
                       src/point_selectors/rectangular_frustum_selector.cpp
                       src/point_selectors/selected_trackball_interactor_style.cpp
                       src/point_selectors/click_trackball_interactor_style.cpp
                       src/point_selectors/manipulation_event.cpp)
                       
    set(uis src/cloud_composer_main_window.ui)
    set(resources resources/resources.qrc)

    QT4_WRAP_UI(cloud_composer_ui ${uis})
    QT4_WRAP_CPP(cloud_composer_moc ${incs} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
    QT4_ADD_RESOURCES(resource_srcs ${resources})
    
    set(EXE_NAME "pcl_${SUBSUBSYS_NAME}")
    PCL_ADD_EXECUTABLE("${EXE_NAME}" "${SUBSUBSYS_NAME}" ${cloud_composer_ui} ${cloud_composer_moc} ${srcs} ${resource_srcs} ${item_srcs} ${selector_srcs} ${impl_incs})
    target_link_libraries("${EXE_NAME}" pcl_common pcl_io pcl_visualization pcl_filters ${QVTK_LIBRARY} ${QT_LIBRARIES})

    # Install include files
    PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSUBSYS_NAME}" ${incs} ${item_incs} ${selector_incs})
    PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSUBSYS_NAME}/impl" ${impl_incs})
    
    PCL_MAKE_PKGCONFIG("${EXE_NAME}" "${SUBSUBSYS_NAME}" "${SUBSYS_DESC}" "" "" "" "" "")
    

    #TOOL buildING SCRIPTS
    #Create subdirectory for plugin libs
    set (CLOUD_COMPOSER_PLUGIN_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cloud_composer_plugins")
    make_directory("${CLOUD_COMPOSER_PLUGIN_DIR}")

    set(INTERFACE_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tool_interface/abstract_tool.h")
    set(INTERFACE_SOURCES src/tool_interface/abstract_tool.cpp)
    
    QT4_WRAP_CPP(INTERFACE_HEADERS_MOC ${INTERFACE_HEADERS} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
    PCL_ADD_LIBRARY(pcl_cc_tool_interface "${SUBSUBSYS_NAME}" ${INTERFACE_SOURCES} ${INTERFACE_HEADERS_MOC})
    target_link_libraries(pcl_cc_tool_interface pcl_common ${QT_LIBRARIES})

    IF(APPLE)
       SET_TARGET_PROPERTIES(pcl_cc_tool_interface PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
    ENDIF()
    
    include(ComposerTool.cmake REQUIRED)
    
    #FPFH Tool
    set (FPFH_DEPS pcl_features pcl_kdtree pcl_filters)
    set (FPFH_SOURCES tools/fpfh_estimation.cpp)
    set (FPFH_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/fpfh_estimation.h")
    define_composer_tool (fpfh_estimation "${FPFH_SOURCES}" "${FPFH_HEADERS}" "${FPFH_DEPS}")
    
    #Normals Tool
    set (NORMALS_DEPS pcl_features pcl_kdtree)
    set (NORMALS_SOURCES tools/normal_estimation.cpp)
    set (NORMALS_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/normal_estimation.h")
    define_composer_tool (normal_estimation "${NORMALS_SOURCES}" "${NORMALS_HEADERS}" "${NORMALS_DEPS}")

    #Euclidean Clustering Tool
    set (EC_DEPS pcl_segmentation pcl_kdtree)
    set (EC_SOURCES tools/euclidean_clustering.cpp)
    set (EC_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/euclidean_clustering.h")
    define_composer_tool (euclidean_clustering "${EC_SOURCES}" "${EC_HEADERS}" "${EC_DEPS}")
    
    #Statistical Outlier Removal Tool
    set (SOR_DEPS pcl_filters)
    set (SOR_SOURCES tools/statistical_outlier_removal.cpp)
    set (SOR_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/statistical_outlier_removal.h")
    define_composer_tool (statistical_outlier_removal "${SOR_SOURCES}" "${SOR_HEADERS}" "${SOR_DEPS}")
    
    #Vox Grid Downsample Tool
    set (VOXDS_DEPS pcl_filters)
    set (VOXDS_SOURCES tools/voxel_grid_downsample.cpp)
    set (VOXDS_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/voxel_grid_downsample.h")
    define_composer_tool (voxel_grid_downsample "${VOXDS_SOURCES}" "${VOXDS_HEADERS}" "${VOXDS_DEPS}")
    
     #Organized Segmentation
    set (OSEG_DEPS pcl_segmentation pcl_kdtree)
    set (OSEG_SOURCES tools/organized_segmentation.cpp "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/impl/organized_segmentation.hpp")
    set (OSEG_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/organized_segmentation.h")
    define_composer_tool (organized_segmentation "${OSEG_SOURCES}" "${OSEG_HEADERS}" "${OSEG_DEPS}")
    
        #Sanitize Cloud Tool
    set (SAN_DEPS pcl_filters)
    set (SAN_SOURCES tools/sanitize_cloud.cpp)
    set (SAN_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/sanitize_cloud.h")
    define_composer_tool (sanitize_cloud "${SAN_SOURCES}" "${SAN_HEADERS}" "${SAN_DEPS}")
    
     #Supervoxels
    set (VSP_DEPS pcl_octree pcl_segmentation)
    set (VSP_SOURCES tools/supervoxels.cpp "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/impl/supervoxels.hpp")
    set (VSP_HEADERS "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/tools/supervoxels.h")
    define_composer_tool (supervoxels "${VSP_SOURCES}" "${VSP_HEADERS}" "${VSP_DEPS}")

endif(build)

cmake_minimum_required (VERSION 2.4.6)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})
include_directories (src)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories(${wxWidgets_INCLUDE_DIRS})
find_package (VTK REQUIRED)
include (${VTK_USE_FILE})

## ---[ Point Cloud Library - Visualization
rosbuild_add_library (pcl_visualization
                      src/libpcl_visualization/interactor.cpp
                      src/libpcl_visualization/interactor_style.cpp
                      src/libpcl_visualization/point_cloud_handlers.cpp
                      src/libpcl_visualization/pcl_visualizer.cpp
                      src/libpcl_visualization/histogram_visualizer.cpp
                      src/libpcl_visualization/common/common.cpp
                      src/libpcl_visualization/common/io.cpp
                      src/libpcl_visualization/common/shapes.cpp
                      src/libpcl_visualization/common/float_image_utils.cpp
                      src/libpcl_visualization/common/image_widget_wx.cpp
                      src/libpcl_visualization/range_image_visualizer.cpp
                      src/libpcl_visualization/cloud_viewer.cpp
                     )
target_link_libraries (pcl_visualization vtkCommon vtkWidgets vtkHybrid ${wxWidgets_LIBRARIES})

rosbuild_add_executable (test_pcl_vis src/test.cpp)
target_link_libraries (test_pcl_vis pcl_visualization)

rosbuild_add_executable (test_geometry src/test_geometry.cpp)
target_link_libraries (test_geometry pcl_visualization)

rosbuild_add_executable (test_shapes src/test_shapes.cpp)
target_link_libraries (test_shapes pcl_visualization)

rosbuild_download_data (http://pr.willowgarage.com/data/pcl/partial_cup_model_new.pcd data/partial_cup_model.pcd a9acd00d71cfdd3c176053f1fd1497c8)

add_subdirectory (src/examples)
add_subdirectory (src/tools)
# Option to generate a standalone tarball
add_custom_target (tarball ${PROJECT_SOURCE_DIR}/make_tarball WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} VERBATIM)



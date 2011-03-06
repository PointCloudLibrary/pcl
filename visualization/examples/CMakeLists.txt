PCL_ADD_EXECUTABLE(tutorial_range_image_visualization ${VIS_COMPONENT}
    range_image_visualization.cpp)
target_link_libraries(tutorial_range_image_visualization pcl_visualization
    pcl_range_image)


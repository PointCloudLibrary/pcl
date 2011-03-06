PCL_ADD_EXECUTABLE(test_pcl_vis ${VIS_COMPONENT} test.cpp)
target_link_libraries(test_pcl_vis pcl_visualization)

PCL_ADD_EXECUTABLE(test_geometry ${VIS_COMPONENT} test_geometry.cpp)
target_link_libraries(test_geometry pcl_visualization)

PCL_ADD_EXECUTABLE(test_shapes ${VIS_COMPONENT} test_shapes.cpp)
target_link_libraries(test_shapes pcl_visualization)


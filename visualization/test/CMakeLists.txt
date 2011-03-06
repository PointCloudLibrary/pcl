PCL_ADD_EXECUTABLE(test_pcl_vis ${SUBSYS_NAME} test.cpp)
target_link_libraries(test_pcl_vis pcl_visualization)

PCL_ADD_EXECUTABLE(test_geometry ${SUBSYS_NAME} test_geometry.cpp)
target_link_libraries(test_geometry pcl_visualization)

PCL_ADD_EXECUTABLE(test_shapes ${SUBSYS_NAME} test_shapes.cpp)
target_link_libraries(test_shapes pcl_visualization)


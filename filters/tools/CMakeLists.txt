PCL_ADD_EXECUTABLE(voxel_grid ${SUBSYS_NAME} voxel_grid.cpp)
target_link_libraries(voxel_grid pcl_common pcl_io pcl_filters)

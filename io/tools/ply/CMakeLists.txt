PCL_ADD_EXECUTABLE(pcl_ply2obj ${SUBSYS_NAME} ply2obj.cpp)
target_link_libraries(pcl_ply2obj pcl_io_ply)

PCL_ADD_EXECUTABLE(pcl_ply2ply ${SUBSYS_NAME} ply2ply.cpp)
target_link_libraries(pcl_ply2ply pcl_io_ply)

PCL_ADD_EXECUTABLE(pcl_ply2raw ${SUBSYS_NAME} ply2raw.cpp)
target_link_libraries(pcl_ply2raw pcl_io_ply)

PCL_ADD_EXECUTABLE(pcl_plyheader ${SUBSYS_NAME} plyheader.cpp)
target_link_libraries(pcl_plyheader pcl_io_ply)

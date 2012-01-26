PCL_ADD_EXECUTABLE(ply2obj ${SUBSYS_NAME} ply2obj.cpp)
target_link_libraries(ply2obj pcl_io_ply)

PCL_ADD_EXECUTABLE(ply2ply ${SUBSYS_NAME} ply2ply.cpp)
target_link_libraries(ply2ply pcl_io_ply)

PCL_ADD_EXECUTABLE(ply2raw ${SUBSYS_NAME} ply2raw.cpp)
target_link_libraries(ply2raw pcl_io_ply)

PCL_ADD_EXECUTABLE(plyheader ${SUBSYS_NAME} plyheader.cpp)
target_link_libraries(plyheader pcl_io_ply)

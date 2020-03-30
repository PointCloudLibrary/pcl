PCL_ADD_EXECUTABLE(pcl_ply2obj COMPONENT ${SUBSYS_NAME} SOURCES ply2obj.cpp)
target_link_libraries(pcl_ply2obj pcl_io_ply)

PCL_ADD_EXECUTABLE(pcl_ply2ply COMPONENT ${SUBSYS_NAME} SOURCES ply2ply.cpp)
target_link_libraries(pcl_ply2ply pcl_io_ply)

PCL_ADD_EXECUTABLE(pcl_ply2raw COMPONENT ${SUBSYS_NAME} SOURCES ply2raw.cpp)
target_link_libraries(pcl_ply2raw pcl_io_ply)

PCL_ADD_EXECUTABLE(pcl_plyheader COMPONENT ${SUBSYS_NAME} SOURCES plyheader.cpp)
target_link_libraries(pcl_plyheader pcl_io_ply)

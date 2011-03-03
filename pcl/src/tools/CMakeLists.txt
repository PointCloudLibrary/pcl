# ---[ Tools
rosbuild_add_executable (convert_pcd_ascii_binary convert_pcd_ascii_binary.cpp)
target_link_libraries (convert_pcd_ascii_binary pcl_io)

rosbuild_add_executable (concatenate_points_pcd concatenate_points_pcd.cpp)
target_link_libraries (concatenate_points_pcd pcl_io)

rosbuild_add_executable (alu_pointcloud_to_pcd alu_pointcloud_to_pcd.cpp)
target_link_libraries (alu_pointcloud_to_pcd pcl_io)


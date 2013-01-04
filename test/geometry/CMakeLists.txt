PCL_ADD_TEST(geometry_iterator test_iterator
             FILES test_iterator.cpp
             LINK_WITH pcl_gtest pcl_common)

PCL_ADD_TEST(geometry_mesh_circulators test_mesh_circulators
             FILES test_mesh_circulators.cpp
             LINK_WITH pcl_gtest)

PCL_ADD_TEST(geometry_mesh_indices test_mesh_indices
             FILES test_mesh_indices.cpp
             LINK_WITH pcl_gtest)

add_definitions(-DPCL_TEST_GEOMETRY_BINARY_DIR="${CMAKE_CURRENT_BINARY_DIR}")
PCL_ADD_TEST(geometry_mesh_io test_mesh_io
             FILES test_mesh_io.cpp
             LINK_WITH pcl_gtest)

PCL_ADD_TEST(geometry_mesh test_mesh
             FILES test_mesh.cpp
             LINK_WITH pcl_gtest)

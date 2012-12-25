PCL_ADD_TEST(geometry_iterator test_iterator
             FILES test_iterator.cpp
             LINK_WITH pcl_gtest pcl_common)

PCL_ADD_TEST(geometry_mesh test_mesh
             FILES test_mesh.cpp
             LINK_WITH pcl_gtest pcl_geometry)

PCL_ADD_TEST(geometry_mesh_circulators test_mesh_circulators
             FILES test_mesh_circulators.cpp
             LINK_WITH pcl_gtest pcl_geometry)

PCL_ADD_TEST(geometry_mesh_indices test_mesh_indices
             FILES test_mesh_indices.cpp
             LINK_WITH pcl_gtest pcl_geometry)

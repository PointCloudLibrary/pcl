PCL_ADD_TEST(geometry_iterator test_iterator
             FILES test_iterator.cpp
             LINK_WITH pcl_gtest pcl_common)

PCL_ADD_TEST(geometry_mesh_circulators test_mesh_circulators
             FILES test_mesh_circulators.cpp
             LINK_WITH pcl_gtest)

PCL_ADD_TEST(geometry_mesh_conversion test_mesh_conversion
             FILES test_mesh_conversion.cpp
             LINK_WITH pcl_gtest pcl_common)

PCL_ADD_TEST(geometry_mesh_data test_mesh_data
             FILES test_mesh_data.cpp
             LINK_WITH pcl_gtest)

PCL_ADD_TEST(geometry_mesh_get_boundary test_mesh_get_boundary
             FILES test_mesh_get_boundary.cpp
             LINK_WITH pcl_gtest)

PCL_ADD_TEST(geometry_mesh_indices test_mesh_indices
             FILES test_mesh_indices.cpp
             LINK_WITH pcl_gtest)

add_definitions(-DPCL_TEST_GEOMETRY_BINARY_DIR="${CMAKE_CURRENT_BINARY_DIR}")
PCL_ADD_TEST(geometry_mesh_io test_mesh_io
             FILES test_mesh_io.cpp
             LINK_WITH pcl_gtest)

PCL_ADD_TEST(geometry_mesh test_mesh
             FILES test_mesh.cpp test_mesh_common_functions.h
             LINK_WITH pcl_gtest)

PCL_ADD_TEST(geometry_polygon_mesh test_polygon_mesh
             FILES test_polygon_mesh.cpp
             LINK_WITH pcl_gtest)

PCL_ADD_TEST(geometry_quad_mesh test_quad_mesh
             FILES test_quad_mesh.cpp
             LINK_WITH pcl_gtest)

PCL_ADD_TEST(geometry_triangle_mesh test_triangle_mesh
             FILES test_triangle_mesh.cpp
             LINK_WITH pcl_gtest)

set(SUBSYS_NAME tests_geometry)
set(SUBSYS_DESC "Point cloud library geometry module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS geometry)

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(geometry_iterator test_iterator
             FILES test_iterator.cpp
             LINK_WITH pcl_gtest pcl_geometry)

PCL_ADD_TEST(geometry_mesh_circulators test_mesh_circulators
             FILES test_mesh_circulators.cpp
             LINK_WITH pcl_gtest pcl_geometry)

PCL_ADD_TEST(geometry_mesh_conversion test_mesh_conversion
             FILES test_mesh_conversion.cpp
             LINK_WITH pcl_gtest pcl_geometry)

PCL_ADD_TEST(geometry_mesh_data test_mesh_data
             FILES test_mesh_data.cpp
             LINK_WITH pcl_gtest pcl_geometry)

PCL_ADD_TEST(geometry_mesh_get_boundary test_mesh_get_boundary
             FILES test_mesh_get_boundary.cpp
             LINK_WITH pcl_gtest pcl_geometry)

PCL_ADD_TEST(geometry_mesh_indices test_mesh_indices
             FILES test_mesh_indices.cpp
             LINK_WITH pcl_gtest pcl_geometry)

add_definitions(-DPCL_TEST_GEOMETRY_BINARY_DIR="${CMAKE_CURRENT_BINARY_DIR}")
PCL_ADD_TEST(geometry_mesh_io test_mesh_io
             FILES test_mesh_io.cpp
             LINK_WITH pcl_gtest pcl_geometry)

PCL_ADD_TEST(geometry_mesh test_mesh
             FILES test_mesh.cpp test_mesh_common_functions.h
             LINK_WITH pcl_gtest pcl_geometry)

PCL_ADD_TEST(geometry_polygon_mesh test_polygon_mesh
             FILES test_polygon_mesh.cpp
             LINK_WITH pcl_gtest pcl_geometry)

PCL_ADD_TEST(geometry_quad_mesh test_quad_mesh
             FILES test_quad_mesh.cpp
             LINK_WITH pcl_gtest pcl_geometry)

PCL_ADD_TEST(geometry_triangle_mesh test_triangle_mesh
             FILES test_triangle_mesh.cpp
             LINK_WITH pcl_gtest pcl_geometry)

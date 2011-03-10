cmake_minimum_required (VERSION 2.8)

include (cmake/rosbuild.cmake)
#set (ROS_BUILD_TYPE Release)
set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories ()
add_definitions (-W0 -DEIGEN_USE_NEW_STDVECTOR -DEIGEN2_SUPPORT)
rosbuild_check_for_sse ()

rosbuild_genmsg ()
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})
#include_directories (src include include/roslib include/sensor_msgs include/pcl)
include_directories (src)
include_directories (include)
include_directories (include/roslib)
include_directories (include/sensor_msgs)
include_directories (include/pcl)

include(cmake/find_boost.cmake)
#link_directories(${BOOST_LIB_DIR})
include(cmake/find_cminpack.cmake)
include_directories(${CMINPACK_INCLUDE_DIR})
include(cmake/find_qhull.cmake)
include_directories(${QHULL_INCLUDE_DIR})
include(cmake/find_flann.cmake)
include_directories(${flann_INCLUDE_DIR})
include(cmake/find_eigen3.cmake)
include_directories(${eigen3_INCLUDE_DIR})
#include(cmake/find_tbb.cmake)
#include_directories(${TBB_INCLUDE_DIR})
#link_directories(${TBB_LIB_DIR})

# ---[ Point Cloud Library
  rosbuild_add_library (pcl_base
                        src/pcl/pcl_base.cpp
                        src/pcl/common/distances.cpp
                        src/pcl/common/intersections.cpp
                       )
  rosbuild_add_compile_flags (pcl_base ${SSE_FLAGS})

# ---[ Point Cloud Library - Features
  rosbuild_add_library (pcl_features
                        src/pcl/features/feature.cpp
                        src/pcl/features/boundary.cpp
                        src/pcl/features/intensity_gradient.cpp
                        src/pcl/features/intensity_spin.cpp
                        src/pcl/features/moment_invariants.cpp
                        src/pcl/features/normal_3d.cpp
                        src/pcl/features/normal_3d_omp.cpp
                        #src/pcl/features/normal_3d_tbb.cpp
                        src/pcl/features/principal_curvatures.cpp
                        src/pcl/features/fpfh.cpp
                        src/pcl/features/fpfh_omp.cpp
                        src/pcl/features/pfh.cpp
                        src/pcl/features/rift.cpp
                        src/pcl/features/rsd.cpp
                        src/pcl/features/vfh.cpp
                        src/pcl/features/narf.cpp
                        src/pcl/features/narf_descriptor.cpp
                        src/pcl/features/integral_image_normal.cpp
                       )
  rosbuild_add_compile_flags (pcl_features ${SSE_FLAGS})
  rosbuild_add_openmp_flags (pcl_features)
  rosbuild_link_boost (pcl_features system filesystem)
  #  target_link_libraries (pcl_features tbb)
  target_link_libraries (pcl_features pcl_range_image)

# ---[ Point Cloud Library - Features
  rosbuild_add_library (pcl_keypoints
                        src/pcl/keypoints/narf_keypoint.cpp
                       )
  target_link_libraries(pcl_keypoints pcl_range_image_border_extractor)
  rosbuild_add_compile_flags (pcl_keypoints ${SSE_FLAGS})
  rosbuild_add_openmp_flags (pcl_keypoints)

# ---[ Point Cloud Library - IO
  rosbuild_add_library (pcl_io
                      src/pcl/io/io.cpp
                      src/pcl/io/pcd_io.cpp
                      src/pcl/io/vtk_io.cpp
                     )
  rosbuild_add_compile_flags (pcl_io ${SSE_FLAGS})
  rosbuild_link_boost (pcl_io system filesystem)
  #  target_link_libraries (pcl_io tbb)

# ---[ Point Cloud Library - Surface
  rosbuild_add_library (pcl_surface
                        src/pcl/surface/mls.cpp
                        src/pcl/surface/grid_projection.cpp
                        src/pcl/surface/convex_hull.cpp
                        src/pcl/surface/concave_hull.cpp
                        src/pcl/surface/gp3.cpp
                       )
  rosbuild_add_compile_flags (pcl_surface ${SSE_FLAGS})
  rosbuild_link_boost (pcl_surface system filesystem)
# ---[ Point Cloud Library - KdTree
rosbuild_add_library (pcl_kdtree
                      src/pcl/kdtree/kdtree_flann.cpp
                      src/pcl/kdtree/organized_data.cpp
                      src/pcl/kdtree/tree_types.cpp
                     )
rosbuild_add_compile_flags (pcl_kdtree ${SSE_FLAGS})
 # ---[ Point Cloud Library - Sample Consensus
rosbuild_add_library (pcl_sample_consensus
                      src/pcl/sample_consensus/lmeds.cpp
                      src/pcl/sample_consensus/ransac.cpp
                      src/pcl/sample_consensus/mlesac.cpp
                      src/pcl/sample_consensus/msac.cpp
                      src/pcl/sample_consensus/prosac.cpp
                      src/pcl/sample_consensus/rmsac.cpp
                      src/pcl/sample_consensus/rransac.cpp
                      src/pcl/sample_consensus/sac_model_circle.cpp
                      src/pcl/sample_consensus/sac_model_cylinder.cpp
                      src/pcl/sample_consensus/sac_model_normal_parallel_plane.cpp
                      src/pcl/sample_consensus/sac_model_normal_plane.cpp
                      src/pcl/sample_consensus/sac_model_parallel_plane.cpp
                      src/pcl/sample_consensus/sac_model_perpendicular_plane.cpp
                      src/pcl/sample_consensus/sac_model_plane.cpp
                      src/pcl/sample_consensus/sac_model_line.cpp
                      src/pcl/sample_consensus/sac_model_parallel_line.cpp
                      src/pcl/sample_consensus/sac_model_sphere.cpp
                      src/pcl/sample_consensus/sac_model_registration.cpp
                     )
rosbuild_add_compile_flags (pcl_sample_consensus ${SSE_FLAGS})

# ---[ Point Cloud Library - Filters
  rosbuild_add_library (pcl_filters
                        src/pcl/filters/filter.cpp
                        src/pcl/filters/conditional_removal.cpp
                        src/pcl/filters/passthrough.cpp
                        src/pcl/filters/voxel_grid.cpp
                        src/pcl/filters/extract_indices.cpp
                        src/pcl/filters/project_inliers.cpp
                        src/pcl/filters/radius_outlier_removal.cpp
                        src/pcl/filters/statistical_outlier_removal.cpp
                       )
  rosbuild_add_compile_flags (pcl_filters ${SSE_FLAGS})
  rosbuild_link_boost (pcl_filters system filesystem)
  target_link_libraries (pcl_filters pcl_base pcl_sample_consensus)
# ---[ Point Cloud Library - Segmentation
rosbuild_add_library (pcl_segmentation
                      src/pcl/segmentation/segment_differences.cpp
                      src/pcl/segmentation/sac_segmentation.cpp
                      src/pcl/segmentation/extract_clusters.cpp
                      src/pcl/segmentation/extract_polygonal_prism_data.cpp
                     )
rosbuild_add_compile_flags (pcl_segmentation ${SSE_FLAGS})
target_link_libraries (pcl_segmentation pcl_sample_consensus pcl_kdtree)
rosbuild_link_boost (pcl_segmentation system filesystem)
  # ---[ Point Cloud Library - Registration
  rosbuild_add_library (pcl_registration
                        src/pcl/registration/registration.cpp
                       )
  rosbuild_add_compile_flags (pcl_registration ${SSE_FLAGS})
  rosbuild_link_boost (pcl_registration system filesystem)
target_link_libraries (pcl_registration pcl_sample_consensus pcl_kdtree)

  # ---[ Point Cloud Library - RangeImage
  rosbuild_add_library (pcl_range_image
                        src/pcl/range_image/range_image.cpp
                        src/pcl/range_image/range_image_planar.cpp
                       )
  rosbuild_add_compile_flags (pcl_range_image ${SSE_FLAGS})
  rosbuild_add_openmp_flags(pcl_range_image)

  rosbuild_add_library (pcl_range_image_border_extractor
                        src/pcl/features/range_image_border_extractor.cpp
                        src/pcl/features/boundary.cpp
                       )
  target_link_libraries (pcl_range_image_border_extractor pcl_range_image)
  rosbuild_add_openmp_flags (pcl_range_image_border_extractor)

add_subdirectory (src/tools)


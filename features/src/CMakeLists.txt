# ---[ Point Cloud Library - Features
rosbuild_add_library (pcl_features
                      feature.cpp
                      boundary.cpp
                      intensity_gradient.cpp
                      intensity_spin.cpp
                      moment_invariants.cpp
                      normal_3d.cpp
                      normal_3d_omp.cpp
#                      normal_3d_tbb.cpp
                      principal_curvatures.cpp
                      fpfh.cpp
                      fpfh_omp.cpp
                      pfh.cpp
                      rift.cpp
                      rsd.cpp
                      vfh.cpp
                      narf.cpp
                      narf_descriptor.cpp
                      integral_image_normal.cpp
                     )
rosbuild_add_compile_flags (pcl_features "${SSE_FLAGS}")
rosbuild_add_openmp_flags (pcl_features)
rosbuild_link_boost (pcl_features system filesystem)
#  if (HAVE_TBB)
#    target_link_libraries (pcl_features tbb)
#  endif (HAVE_TBB)
target_link_libraries (pcl_features pcl_range_image)


rosbuild_add_library (pcl_range_image_border_extractor
                      range_image_border_extractor.cpp
                      boundary.cpp
                     )
target_link_libraries (pcl_range_image_border_extractor pcl_range_image)
rosbuild_add_openmp_flags (pcl_range_image_border_extractor)



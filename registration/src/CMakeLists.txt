# ---[ Point Cloud Library - Registration
rosbuild_add_library (pcl_registration
                      registration.cpp
                     )
rosbuild_add_compile_flags (pcl_registration ${SSE_FLAGS})
rosbuild_link_boost (pcl_registration system filesystem)
target_link_libraries (pcl_registration pcl_sample_consensus pcl_kdtree)



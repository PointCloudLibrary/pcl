# ---[ Point Cloud Library - Common
rosbuild_add_library (pcl_base
                      ../pcl_base.cpp
                      distances.cpp
                      intersections.cpp
                     )
rosbuild_add_compile_flags (pcl_base ${SSE_FLAGS})

rosbuild_add_library (pcl_common poses_from_matches.cpp)
rosbuild_add_compile_flags (pcl_common ${SSE_FLAGS})

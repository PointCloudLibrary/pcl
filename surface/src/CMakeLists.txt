# ---[ Point Cloud Library - Surface
rosbuild_add_library (pcl_surface
                      mls.cpp
                      grid_projection.cpp
                      convex_hull.cpp
                      concave_hull.cpp
                      gp3.cpp
                     )
                     
target_link_libraries (pcl_surface qhull)
rosbuild_add_compile_flags (pcl_surface ${SSE_FLAGS})
rosbuild_link_boost (pcl_surface system filesystem)


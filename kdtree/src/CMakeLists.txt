# ---[ Point Cloud Library - KdTree
rosbuild_add_library (pcl_kdtree
                      kdtree_flann.cpp
                      organized_data.cpp
                      tree_types.cpp
                     )
rosbuild_add_compile_flags (pcl_kdtree ${SSE_FLAGS})


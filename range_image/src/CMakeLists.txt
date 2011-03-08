# ---[ Point Cloud Library - RangeImage
rosbuild_add_library (pcl_range_image
                      range_image.cpp
                      range_image_planar.cpp
                     )
rosbuild_add_compile_flags (pcl_range_image ${SSE_FLAGS})
rosbuild_add_openmp_flags (pcl_range_image)



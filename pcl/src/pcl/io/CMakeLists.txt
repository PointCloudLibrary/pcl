# ---[ Point Cloud Library - IO
rosbuild_add_library (pcl_io
                      io.cpp
                      pcd_io.cpp
                      vtk_io.cpp
                     )
rosbuild_add_compile_flags (pcl_io ${SSE_FLAGS})
rosbuild_link_boost (pcl_io system filesystem)
# if (HAVE_TBB)
#  target_link_libraries (pcl_io tbb)
# endif (HAVE_TBB)

# ---[ Point Cloud Library - Sample Consensus
rosbuild_add_library (pcl_sample_consensus
                      lmeds.cpp
                      ransac.cpp
                      mlesac.cpp
                      msac.cpp
                      prosac.cpp
                      rmsac.cpp
                      rransac.cpp
                      sac_model_circle.cpp
                      sac_model_cylinder.cpp
                      sac_model_normal_parallel_plane.cpp
                      sac_model_normal_plane.cpp
                      sac_model_parallel_plane.cpp
                      sac_model_perpendicular_plane.cpp
                      sac_model_plane.cpp
                      sac_model_line.cpp
                      sac_model_parallel_line.cpp
                      sac_model_sphere.cpp
                      sac_model_registration.cpp
                     )
rosbuild_add_compile_flags (pcl_sample_consensus ${SSE_FLAGS})



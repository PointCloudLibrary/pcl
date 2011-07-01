cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("PCL Tutorials")

# uncomment the next line in case you didn't build PCL out of sources or to use FindPCL.cmake 
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

foreach(subdir 
          cloud_viewer
          cluster_extraction
          concatenate_fields
          concatenate_points
          convex_hull_2d
          cylinder_segmentation
          extract_indices
          greedy_projection
          narf_descriptor_visualization
          narf_feature_extraction
          narf_keypoint_extraction
          octree_change_detection
          octree_search
          openni_grabber
          passthrough
          pcd_read 
          pcd_write
          pcl_visualizer
          planar_segmentation
          point_cloud_compression
          project_inliers
          range_image_border_extraction
          range_image_creation
          range_image_visualization
          resampling
          statistical_removal
          template_alignment
          vfh_recognition
          voxel_grid
       )

  add_subdirectory(${subdir})

endforeach()

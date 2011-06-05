cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("PCL Tutorials")

foreach(subdir 
          cloud_viewer
          cluster_extraction
          convex_hull_2d
          concatenate_fields
          concatenate_points
          cylinder_segmentation
          extract_indices
          greedy_projection
          octree_change_detection
          octree_search
          openni_grabber
          passthrough
          pcd_read 
          pcd_write
          planar_segmentation
          point_cloud_compression
          project_inliers
          range_image_creation
          #range_image_visualization
          resampling
          statistical_removal
          template_alignment
          vfh_recognition
          voxel_grid
       )

  add_subdirectory(${subdir})

endforeach()

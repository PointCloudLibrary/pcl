cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("PCL Tutorials")

foreach(subdir 
          cloud_viewer
          cluster_extraction
          concatenate_fields
          concatenate_points
          convex_hull_2d
          concave_hull_2d
          cylinder_segmentation
          extract_indices
          greedy_projection
          #narf_descriptor_visualization
          narf_feature_extraction
          narf_keypoint_extraction
          octree_change_detection
          octree_search
          openni_grabber
          openni_narf_keypoint_extraction
          openni_range_image_visualization
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
          registration_api
          resampling
          statistical_removal
          template_alignment
          vfh_recognition
          voxel_grid
       )

      add_subdirectory(${subdir})

endforeach()

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("PCL_Tutorials")

foreach(subdir 
          cloud_viewer
          cluster_extraction
          concatenate_clouds
          concatenate_fields
          concatenate_points
          concave_hull_2d
          conditional_removal
          convex_hull_2d
          correspondence_grouping
          cylinder_segmentation
          extract_indices
          #feature_evaluation
          greedy_projection
		  implicit_shape_model
          iterative_closest_point
          kdtree_search
          min_cut_segmentation
          moment_of_inertia
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
          radius_outlier_removal
          random_sample_consensus
          range_image_border_extraction
          range_image_creation
          range_image_visualization
          region_growing_rgb_segmentation
          region_growing_segmentation
          registration_api
          remove_outliers
          resampling
          rops_feature
          statistical_removal
          stick_segmentation
          supervoxel_clustering
          template_alignment
	  tracking
          vfh_recognition
          voxel_grid
          normal_distributions_transform
          # Additional/Complex tutorials presented at PCL events
          iros2011
          iccv2011
		  normal_estimation_using_integral_images
       )

      add_subdirectory(${subdir})

endforeach()

#pragma once

#include "typedefs.h"

#include "solution/filters.h"
#include "solution/segmentation.h"
#include "solution/feature_estimation.h"
#include "solution/registration.h"

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

struct ObjectRecognitionParameters
{
  // Filter parameters
  float min_depth;
  float max_depth;
  float downsample_leaf_size;
  float outlier_rejection_radius;
  int outlier_rejection_min_neighbors;

  // Segmentation parameters
  float plane_inlier_distance_threshold;
  int max_ransac_iterations;
  float cluster_tolerance;
  int min_cluster_size;
  int max_cluster_size;

  // Feature estimation parameters
  float surface_normal_radius;
  float keypoints_min_scale;
  float keypoints_nr_octaves;
  float keypoints_nr_scales_per_octave;
  float keypoints_min_contrast;
  float local_descriptor_radius;

  // Registration parameters
  float initial_alignment_min_sample_distance;
  float initial_alignment_max_correspondence_distance;
  int initial_alignment_nr_iterations;
  float icp_max_correspondence_distance;
  float icp_outlier_rejection_threshold;
  float icp_transformation_epsilon;
  int icp_max_iterations;
};

struct ObjectModel
{
  PointCloudPtr points;
  PointCloudPtr keypoints;
  LocalDescriptorsPtr local_descriptors;
  GlobalDescriptorsPtr global_descriptor;
};

class ObjectRecognition
{
public:
  ObjectRecognition (const ObjectRecognitionParameters & params) : params_ (params)
  {}

  void 
  populateDatabase (const std::vector<std::string> & filenames)
  {
  } 

  const ObjectModel & 
  recognizeObject (const PointCloudPtr & query_cloud)
  {
    int best_match = 0;
    return (models_[best_match]);
  }

  PointCloudPtr
  recognizeAndAlignPoints (const PointCloudPtr & query_cloud)
  {
    PointCloudPtr output;
    return (output);
  }

  /* Construct an object model by filtering, segmenting, and estimating feature descriptors */
  void
  constructObjectModel (const PointCloudPtr & points, ObjectModel & output) const
  {
    output.points = applyFiltersAndSegment (points, params_);

    SurfaceNormalsPtr normals;
    estimateFeatures (output.points, params_, normals, output.keypoints, 
                      output.local_descriptors, output.global_descriptor);
  }

protected: 
  /* Apply a series of filters (threshold depth, downsample, and remove outliers) */
  PointCloudPtr
  applyFiltersAndSegment (const PointCloudPtr & input, const ObjectRecognitionParameters & params) const
  {
    PointCloudPtr cloud;
    cloud = thresholdDepth (input, params.min_depth, params.max_depth);
    cloud = downsample (cloud, params.downsample_leaf_size);
    cloud = removeOutliers (cloud, params.outlier_rejection_radius, params.outlier_rejection_min_neighbors);

    cloud = findAndSubtractPlane (cloud, params.plane_inlier_distance_threshold, params.max_ransac_iterations);
    std::vector<pcl::PointIndices> cluster_indices;
    clusterObjects (cloud, params.cluster_tolerance, params.min_cluster_size, 
                    params.max_cluster_size, cluster_indices);

    PointCloudPtr largest_cluster (new PointCloud);
    pcl::copyPointCloud (*cloud, cluster_indices[0], *largest_cluster);

    return (largest_cluster);
  }

  /* Estimate surface normals, keypoints, and local/global feature descriptors */
  void
  estimateFeatures (const PointCloudPtr & points, const ObjectRecognitionParameters & params,
                    SurfaceNormalsPtr & normals_out, PointCloudPtr & keypoints_out, 
                    LocalDescriptorsPtr & local_descriptors_out, GlobalDescriptorsPtr & global_descriptor_out) const
  {
    normals_out = estimateSurfaceNormals (points, params.surface_normal_radius);
    
    keypoints_out = detectKeypoints (points, normals_out, params.keypoints_min_scale, params.keypoints_nr_octaves,
                                     params.keypoints_nr_scales_per_octave, params.keypoints_min_contrast);
    
    local_descriptors_out = computeLocalDescriptors (points, normals_out, keypoints_out, 
                                                     params.local_descriptor_radius);
    
    global_descriptor_out = computeGlobalDescriptor (points, normals_out);
  }

  /* Align the points in the source model to the points in the target model */
  PointCloudPtr
  alignModelPoints (const ObjectModel & source, const ObjectModel & target, 
                    const ObjectRecognitionParameters & params) const
  {
    Eigen::Matrix4f tform; 
    tform = computeInitialAlignment (source.keypoints, source.local_descriptors,
                                     target.keypoints, target.local_descriptors,
                                     params.initial_alignment_min_sample_distance,
                                     params.initial_alignment_max_correspondence_distance, 
                                     params.initial_alignment_nr_iterations);

    tform = refineAlignment (source.points, target.points, tform, 
                             params.icp_max_correspondence_distance, params.icp_outlier_rejection_threshold, 
                             params.icp_transformation_epsilon, params.icp_max_iterations);

    PointCloudPtr output (new PointCloud);
    pcl::transformPointCloud (*(source.points), *output, tform);

    return (output);
  }  

  ObjectRecognitionParameters params_;
  std::vector<ObjectModel> models_;
  GlobalDescriptorsPtr descriptors_;
  pcl::KdTreeFLANN<GlobalDescriptorT>::Ptr kdtree_;
};

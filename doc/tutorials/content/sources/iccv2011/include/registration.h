#pragma once

#include "typedefs.h"

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>

/* Use SampleConsensusInitialAlignment to find a rough alignment from the source cloud to the target cloud by finding
 * correspondences between two sets of local features
 * Inputs:
 *   source_points
 *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
 *   source_descriptors
 *     The local descriptors for each source point
 *   target_points
 *     The "target" points, i.e., the points to which the source point cloud will be aligned
 *   target_descriptors
 *     The local descriptors for each target point
 *   min_sample_distance
 *     The minimum distance between any two random samples
 *   max_correspondence_distance
 *     The 
 *   nr_iterations
 *     The number of RANSAC iterations to perform
 * Return: A transformation matrix that will roughly align the points in source to the points in target
 */
Eigen::Matrix4f
computeInitialAlignment (const PointCloudPtr & source_points, const LocalDescriptorsPtr & source_descriptors,
                         const PointCloudPtr & target_points, const LocalDescriptorsPtr & target_descriptors,
                         float min_sample_distance, float max_correspondence_distance, int nr_iterations)
{
  pcl::SampleConsensusInitialAlignment<PointT, PointT, LocalDescriptorT> sac_ia;
  sac_ia.setMinSampleDistance (min_sample_distance);
  sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance);
  sac_ia.setMaximumIterations (nr_iterations);
  
  sac_ia.setInputSource (source_points);
  sac_ia.setSourceFeatures (source_descriptors);

  sac_ia.setInputTarget (target_points);
  sac_ia.setTargetFeatures (target_descriptors);

  PointCloud registration_output;
  sac_ia.align (registration_output);

  return (sac_ia.getFinalTransformation ());
}

/* Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud, 
 * starting with an initial guess
 * Inputs:
 *   source_points
 *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
 *   target_points
 *     The "target" points, i.e., the points to which the source point cloud will be aligned
 *   initial_alignment
 *     An initial estimate of the transformation matrix that aligns the source points to the target points
 *   max_correspondence_distance
 *     A threshold on the distance between any two corresponding points.  Any corresponding points that are further 
 *     apart than this threshold will be ignored when computing the source-to-target transformation
 *   outlier_rejection_threshold
 *     A threshold used to define outliers during RANSAC outlier rejection
 *   transformation_epsilon
 *     The smallest iterative transformation allowed before the algorithm is considered to have converged
 *   max_iterations
 *     The maximum number of ICP iterations to perform
 * Return: A transformation matrix that will precisely align the points in source to the points in target
 */
Eigen::Matrix4f
refineAlignment (const PointCloudPtr & source_points, const PointCloudPtr & target_points, 
                 const Eigen::Matrix4f &initial_alignment, float max_correspondence_distance,
                 float outlier_rejection_threshold, float transformation_epsilon, float max_iterations)
{

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance (max_correspondence_distance);
  icp.setRANSACOutlierRejectionThreshold (outlier_rejection_threshold);
  icp.setTransformationEpsilon (transformation_epsilon);
  icp.setMaximumIterations (max_iterations);

  PointCloudPtr source_points_transformed (new PointCloud);
  pcl::transformPointCloud (*source_points, *source_points_transformed, initial_alignment);

  icp.setInputSource (source_points_transformed);
  icp.setInputTarget (target_points);

  PointCloud registration_output;
  icp.align (registration_output);

  return (icp.getFinalTransformation () * initial_alignment);
}

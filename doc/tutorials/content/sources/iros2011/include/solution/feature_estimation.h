#ifndef FEATURE_ESTIMATION_H
#define FEATURE_ESTIMATION_H

#include "typedefs.h"

#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/search/kdtree.h>

/* Use NormalEstimation to estimate a cloud's surface normals 
 * Inputs:
 *   input
 *     The input point cloud
 *   radius
 *     The size of the local neighborhood used to estimate the surface
 * Return: A pointer to a SurfaceNormals point cloud
 */
SurfaceNormalsPtr
estimateSurfaceNormals (const PointCloudPtr & input, float radius)
{
  pcl::NormalEstimation<PointT, NormalT> normal_estimation;
  normal_estimation.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  normal_estimation.setRadiusSearch (radius);
  normal_estimation.setInputCloud (input);
  SurfaceNormalsPtr normals (new SurfaceNormals);
  normal_estimation.compute (*normals);

  return (normals);
}

/* Use SIFTKeypoint to detect a set of keypoints
 * Inputs:
 *   points
 *     The input point cloud
 *   normals
 *     The input surface normals
 *   min_scale
 *     The smallest scale in the difference-of-Gaussians (DoG) scale-space
 *   nr_octaves
 *     The number of times the scale doubles in the DoG scale-space
 *   nr_scales_per_octave
 *     The number of scales computed for each doubling
 *   min_contrast
 *     The minimum local contrast that must be present for a keypoint to be detected
 * Return: A pointer to a point cloud of keypoints
 */
PointCloudPtr
detectKeypoints (const PointCloudPtr & points, const SurfaceNormalsPtr & normals,
                 float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast)
{
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
  sift_detect.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
  sift_detect.setMinimumContrast (min_contrast);
  sift_detect.setInputCloud (points);
  pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
  sift_detect.compute (keypoints_temp);
  PointCloudPtr keypoints (new PointCloud);
  pcl::copyPointCloud (keypoints_temp, *keypoints);

  return (keypoints);
}

/* Use FPFHEstimation to compute local feature descriptors around each keypoint
 * Inputs:
 *   points
 *     The input point cloud
 *   normals
 *     The input surface normals
 *   keypoints
 *     A cloud of keypoints specifying the positions at which the descriptors should be computed
 *   feature_radius
 *     The size of the neighborhood from which the local descriptors will be computed 
 * Return: A pointer to a LocalDescriptors (a cloud of LocalDescriptorT points)
 */
LocalDescriptorsPtr
computeLocalDescriptors (const PointCloudPtr & points, const SurfaceNormalsPtr & normals, 
                         const PointCloudPtr & keypoints, float feature_radius)
{
  pcl::FPFHEstimation<PointT, NormalT, LocalDescriptorT> fpfh_estimation;
  fpfh_estimation.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  fpfh_estimation.setRadiusSearch (feature_radius);
  fpfh_estimation.setSearchSurface (points);  
  fpfh_estimation.setInputNormals (normals);
  fpfh_estimation.setInputCloud (keypoints);
  LocalDescriptorsPtr local_descriptors (new LocalDescriptors);
  fpfh_estimation.compute (*local_descriptors);

  return (local_descriptors);
}

/* Use VFHEstimation to compute a single global descriptor for the entire input cloud
 * Inputs:
 *   points
 *     The input point cloud
 *   normals
 *     The input surface normals
 * Return: A pointer to a GlobalDescriptors point cloud (a cloud containing a single GlobalDescriptorT point)
 */
GlobalDescriptorsPtr
computeGlobalDescriptor (const PointCloudPtr & points, const SurfaceNormalsPtr & normals)
{
  pcl::VFHEstimation<PointT, NormalT, GlobalDescriptorT> vfh_estimation;
  vfh_estimation.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  vfh_estimation.setInputCloud (points);
  vfh_estimation.setInputNormals (normals);
  GlobalDescriptorsPtr global_descriptor (new GlobalDescriptors);
  vfh_estimation.compute (*global_descriptor);

  return (global_descriptor);
}

/* A simple structure for storing all of a cloud's features */
struct ObjectFeatures
{
  PointCloudPtr points;
  SurfaceNormalsPtr normals;
  PointCloudPtr keypoints;
  LocalDescriptorsPtr local_descriptors;
  GlobalDescriptorsPtr global_descriptor;
};

/* Estimate normals, detect keypoints, and compute local and global descriptors 
 * Return: An ObjectFeatures struct containing all the features
 */
ObjectFeatures
computeFeatures (const PointCloudPtr & input)
{
  ObjectFeatures features;
  features.points = input;
  features.normals = estimateSurfaceNormals (input, 0.05);
  features.keypoints = detectKeypoints (input, features.normals, 0.005, 10, 8, 1.5);
  features.local_descriptors = computeLocalDescriptors (input, features.normals, features.keypoints, 0.1);
  features.global_descriptor = computeGlobalDescriptor (input, features.normals);

  return (features);
}

#endif

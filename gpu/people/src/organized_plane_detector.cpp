/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @author: Koen Buys
 */

#include <pcl/gpu/people/organized_plane_detector.h>
#include <pcl/console/print.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>


pcl::gpu::people::OrganizedPlaneDetector::OrganizedPlaneDetector()
{
  PCL_DEBUG("[pcl::gpu::people::OrganizedPlaneDetector::OrganizedPlaneDetector] : (D) : Constructor called\n");

  // Set NE defaults
  ne_.setNormalEstimationMethod (ne_.COVARIANCE_MATRIX);
  ne_.setMaxDepthChangeFactor (0.02f);
  ne_.setNormalSmoothingSize (20.0f);

  // Set MPS defaults
  mps_MinInliers_ = 10000;
  mps_AngularThreshold_ = pcl::deg2rad (3.0);   //3 degrees
  mps_DistanceThreshold_ = 0.02;                //2cm
  mps_use_planar_refinement_ = true;

  mps_.setMinInliers (mps_MinInliers_);
  mps_.setAngularThreshold (mps_AngularThreshold_);
  mps_.setDistanceThreshold (mps_DistanceThreshold_);

}

void
pcl::gpu::people::OrganizedPlaneDetector::process(const PointCloud<PointXYZRGB>::ConstPtr &cloud)
{
  PCL_DEBUG("[pcl::gpu::people::OrganizedPlaneDetector::process] : (D) : Called\n");

  // Estimate Normals
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
  ne_.setInputCloud (cloud);
  ne_.compute (*normal_cloud);

  /*
  float* distance_map = ne.getDistanceMap ();
  boost::shared_ptr<pcl::EdgeAwarePlaneComparator<PointT,pcl::Normal> > eapc = boost::dynamic_pointer_cast<pcl::EdgeAwarePlaneComparator<PointT,pcl::Normal> >(edge_aware_comparator_);
  eapc->setDistanceMap (distance_map);
  eapc->setDistanceThreshold (0.01f, false);
  */

  // Segment Planes
  std::vector<pcl::PlanarRegion<PointXYZRGB>, Eigen::aligned_allocator<pcl::PlanarRegion<PointXYZRGB> > > regions;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);

  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;

  mps_.setInputNormals (normal_cloud);
  mps_.setInputCloud (cloud);
  if (mps_use_planar_refinement_)
  {
    //mps_.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
  }
  else
  {
    //mps_.segment (regions);
  }

}

void
pcl::gpu::people::OrganizedPlaneDetector::allocate_buffers(int rows, int cols)
{
  PCL_DEBUG("[pcl::gpu::people::OrganizedPlaneDetector::allocate_buffers] : (D) : Called\n");
}

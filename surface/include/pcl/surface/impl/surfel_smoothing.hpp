/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *                      Willow Garage, Inc
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
 * $Id$
 */

#ifndef PCL_SURFACE_IMPL_SURFEL_SMOOTHING_H_
#define PCL_SURFACE_IMPL_SURFEL_SMOOTHING_H_

#include "pcl/surface/surfel_smoothing.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SurfelSmoothing<PointT, PointNT>::initCompute ()
{
  if (!PCLBase<PointT>::initCompute ())
    return false;

  if (!normals_)
  {
    PCL_ERROR ("SurfelSmoothing: normal cloud not set\n");
    return false;
  }

  if (input_->points.size () != normals_->points.size ())
  {
    PCL_ERROR ("SurfelSmoothing: number of input points different from the number of given normals\n");
    return false;
  }

  if (!tree_)
  {
    PCL_ERROR ("SurfelSmoothing: kd-tree not set\n");
    return false;
  }

  // create internal copies of the input - these will be modified
  interm_cloud_ = PointCloudInPtr (new PointCloudIn (*input_));
  interm_normals_ = NormalCloudPtr (new NormalCloud (*normals_));

  return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SurfelSmoothing<PointT, PointNT>::smoothCloudIteration (PointCloudInPtr &output_positions,
                                                             NormalCloudPtr &output_normals)
{
  PCL_INFO ("SurfelSmoothing: cloud smoothing iteration starting ...\n");
  float scale_squared = scale_ * scale_;
  tree_->setInputCloud (interm_cloud_);

  output_positions = PointCloudInPtr (new PointCloudIn ());
  output_normals = NormalCloudPtr (new NormalCloud ());

  std::vector<float> diffs;
  Eigen::Vector3f total_residual (0.0, 0.0, 0.0);
  for (size_t point_i = 0; point_i < interm_cloud_->points.size (); ++point_i)
  {
    Eigen::Vector3f smoothed_point (0.0, 0.0, 0.0);
    Eigen::Vector3f smoothed_normal (0.0, 0.0, 0.0);

    // get neighbors
    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    tree_->radiusSearch (point_i, scale_, nn_indices, nn_distances);

//    std::cerr << "# nn: " << nn_indices.size () << std::endl;

    float theta_normalization_factor = 0.0;
    Eigen::Vector3f e_residual (0.0, 0.0, 0.0);
    for (std::vector<int>::iterator nn_index_it = nn_indices.begin (); nn_index_it != nn_indices.end (); ++nn_index_it)
    {
      float dist = pcl::squaredEuclideanDistance (interm_cloud_->points[point_i], interm_cloud_->points[*nn_index_it]);
      float theta_i = dist; //exp ( (-1) * dist / scale_squared);
      theta_normalization_factor += theta_i;

      smoothed_normal += theta_i * interm_normals_->points[*nn_index_it].getNormalVector3fMap ();
      e_residual += theta_i * (interm_cloud_->points[point_i].getVector3fMap () - interm_cloud_->points[*nn_index_it].getVector3fMap ());
    }
    smoothed_normal /= theta_normalization_factor;
    e_residual /= theta_normalization_factor;
    smoothed_point = interm_cloud_->points[point_i].getVector3fMap () - e_residual.dot (smoothed_normal) * smoothed_normal;
///    smoothed_point = interm_cloud_->points[point_i].getVector3fMap () - e_residual;

    total_residual += e_residual;

//    std::cerr << "---" << std::endl;
//    std::cerr << smoothed_normal << std::endl << std::endl;
//    std::cerr << "theta_norm_factor: " << theta_normalization_factor << std::endl;
//
//    std::cerr << "initial: " << interm_cloud_->points[point_i].getVector3fMap () << " smoothed: " << smoothed_point << std::endl;

    PointT new_point;
    new_point.x = smoothed_point.x (); new_point.y = smoothed_point.y (); new_point.z = smoothed_point.z ();
    output_positions->points.push_back (new_point);
    PointNT new_normal;
    new_normal.normal_x = smoothed_normal.x (); new_normal.normal_y = smoothed_normal.y (); new_normal.normal_z = smoothed_normal.z ();
    output_normals->points.push_back (new_normal);

    // calculate difference
    float diff = smoothed_normal.dot (smoothed_point - interm_cloud_->points[point_i].getVector3fMap ());
    diffs.push_back (diff);
  }

  std::cerr << "Total residual after an iteration: " << total_residual << std::endl;
  PCL_INFO("SurfelSmoothing done iteration\n");
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SurfelSmoothing<PointT, PointNT>::computeSmoothedCloud (PointCloudInPtr &output_positions,
                                                             NormalCloudPtr &output_normals)
{
  if (!initCompute ())
  {
    PCL_ERROR ("SurfelSmoothing: not initialized properly, skipping computeSmoothedCloud ()\n");
    return;
  }

  for (size_t iteration = 0; iteration < 500; ++iteration)
  {
    smoothCloudIteration (output_positions, output_normals);
    interm_cloud_ = output_positions;
    interm_normals_ = output_normals;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SurfelSmoothing<PointT, PointNT>::extractSalientFeaturesBetweenScales (PointCloudInPtr &cloud2,
                                                                            NormalCloudPtr &cloud2_normals,
                                                                            boost::shared_ptr<std::vector<int> > &output_features)
{
  if (interm_cloud_->points.size () != cloud2->points.size () || cloud2->points.size () != cloud2_normals->points.size ())
  {
    PCL_ERROR ("SurfelSmoothing: number of points in the clouds do not match\n");
    return;
  }

  output_features->clear ();

  std::vector<float> diffs;
  for (size_t point_i = 0; point_i < cloud2->points.size (); ++point_i)
  {
    float diff = cloud2_normals->points[point_i].getNormalVector3fMap ().dot (cloud2->points[point_i].getVector3fMap () - interm_cloud_->points[point_i].getVector3fMap ());
    diffs.push_back (diff);
  }

  for (size_t point_i = 0; point_i < cloud2->points.size (); ++point_i)
  {
    // get neighbors
    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    tree_->radiusSearch (point_i, scale_, nn_indices, nn_distances);

    bool largest = true;
    bool smallest = true;
    for (std::vector<int>::iterator nn_index_it = nn_indices.begin (); nn_index_it != nn_indices.end (); ++nn_index_it)
    {
      if (diffs[point_i] < diffs[*nn_index_it])
        largest = false;
      else smallest = false;
    }

    if (largest == true || smallest == true)
      output_features->push_back (point_i);
  }
}



#define PCL_INSTANTIATE_SurfelSmoothing(PointT,PointNT) template class PCL_EXPORTS pcl::SurfelSmoothing<PointT, PointNT>;

#endif /* PCL_SURFACE_IMPL_SURFEL_SMOOTHING_H_ */

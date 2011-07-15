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
  tree_->setInputCloud (interm_cloud_);

  output_positions = PointCloudInPtr (new PointCloudIn);
  output_positions->points.resize (interm_cloud_->points.size ());
  output_normals = NormalCloudPtr (new NormalCloud);
  output_normals->points.resize (interm_cloud_->points.size ());

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;

  std::vector<float> diffs (interm_cloud_->points.size ());
  Eigen::Vector4f total_residual = Eigen::Vector4f::Zero ();

  for (size_t i = 0; i < interm_cloud_->points.size (); ++i)
  {
    Eigen::Vector4f smoothed_point  = Eigen::Vector4f::Zero ();
    Eigen::Vector4f smoothed_normal = Eigen::Vector4f::Zero (); 

    // get neighbors
    tree_->radiusSearch (i, scale_, nn_indices, nn_distances);

    float theta_normalization_factor = 0.0;
    Eigen::Vector4f e_residual = Eigen::Vector4f::Zero ();
    for (std::vector<int>::iterator nn_index_it = nn_indices.begin (); nn_index_it != nn_indices.end (); ++nn_index_it)
    {
      float dist = pcl::squaredEuclideanDistance (interm_cloud_->points[i], interm_cloud_->points[*nn_index_it]);
      float theta_i = exp ( (-1) * dist / scale_squared_);
      theta_normalization_factor += theta_i;

      smoothed_normal += theta_i * interm_normals_->points[*nn_index_it].getNormalVector4fMap ();
      e_residual += theta_i * (interm_cloud_->points[i].getVector4fMap () - interm_cloud_->points[*nn_index_it].getVector4fMap ());
    }
    smoothed_normal /= theta_normalization_factor;
    e_residual /= theta_normalization_factor;
    smoothed_point = interm_cloud_->points[i].getVector4fMap () - e_residual.dot (smoothed_normal) * smoothed_normal;
///    smoothed_point = interm_cloud_->points[point_i].getVector3fMap () - e_residual;

    total_residual += e_residual;

    output_positions->points[i].getVector4fMap () = smoothed_point;
    output_normals->points[i].getNormalVector4fMap () = smoothed_normal;

    // Calculate difference
    diffs[i] = smoothed_normal.dot (smoothed_point - interm_cloud_->points[i].getVector4fMap ());
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
    PCL_ERROR ("[pcl::SurfelSmoothing::computeSmoothedCloud]: SurfelSmoothing not initialized properly, skipping computeSmoothedCloud ().\n");
    return;
  }

  // 250?
  for (size_t iteration = 0; iteration < 250; ++iteration)
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
  if (interm_cloud_->points.size () != cloud2->points.size () || 
      cloud2->points.size () != cloud2_normals->points.size ())
  {
    PCL_ERROR ("[pcl::SurfelSmoothing::extractSalientFeaturesBetweenScales]: Number of points in the clouds does not match.\n");
    return;
  }

  std::vector<float> diffs (cloud2->points.size ());
  for (size_t i = 0; i < cloud2->points.size (); ++i)
    diffs[i] = cloud2_normals->points[i].getNormalVector4fMap ().dot (cloud2->points[i].getVector4fMap () - 
                                                                      interm_cloud_->points[i].getVector4fMap ());

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;

  output_features->resize (cloud2->points.size ());
  for (size_t point_i = 0; point_i < cloud2->points.size (); ++point_i)
  {
    // Get neighbors
    tree_->radiusSearch (point_i, scale_, nn_indices, nn_distances);

    bool largest = true;
    bool smallest = true;
    for (std::vector<int>::iterator nn_index_it = nn_indices.begin (); nn_index_it != nn_indices.end (); ++nn_index_it)
    {
      if (diffs[point_i] < diffs[*nn_index_it])
        largest = false;
      else 
        smallest = false;
    }

    if (largest == true || smallest == true)
      (*output_features)[point_i] = point_i;
  }
}



#define PCL_INSTANTIATE_SurfelSmoothing(PointT,PointNT) template class PCL_EXPORTS pcl::SurfelSmoothing<PointT, PointNT>;

#endif /* PCL_SURFACE_IMPL_SURFEL_SMOOTHING_H_ */

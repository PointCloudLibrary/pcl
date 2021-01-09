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

#include <pcl/search/organized.h> // for OrganizedNeighbor
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/surface/surfel_smoothing.h>
#include <pcl/common/distances.h>
#include <pcl/console/print.h> // for PCL_ERROR, PCL_DEBUG

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

  if (input_->size () != normals_->size ())
  {
    PCL_ERROR ("SurfelSmoothing: number of input points different from the number of given normals\n");
    return false;
  }

  // Initialize the spatial locator
  if (!tree_)
  {
    if (input_->isOrganized ())
      tree_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
      tree_.reset (new pcl::search::KdTree<PointT> (false));
  }

  // create internal copies of the input - these will be modified
  interm_cloud_ = PointCloudInPtr (new PointCloudIn (*input_));
  interm_normals_ = NormalCloudPtr (new NormalCloud (*normals_));

  return (true);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> float
pcl::SurfelSmoothing<PointT, PointNT>::smoothCloudIteration (PointCloudInPtr &output_positions,
                                                             NormalCloudPtr &output_normals)
{
//  PCL_INFO ("SurfelSmoothing: cloud smoothing iteration starting ...\n");

  output_positions = PointCloudInPtr (new PointCloudIn);
  output_positions->points.resize (interm_cloud_->size ());
  output_normals = NormalCloudPtr (new NormalCloud);
  output_normals->points.resize (interm_cloud_->size ());

  pcl::Indices nn_indices;
  std::vector<float> nn_distances;

  std::vector<float> diffs (interm_cloud_->size ());
  float total_residual = 0.0f;

  for (std::size_t i = 0; i < interm_cloud_->size (); ++i)
  {
    Eigen::Vector4f smoothed_point  = Eigen::Vector4f::Zero ();
    Eigen::Vector4f smoothed_normal = Eigen::Vector4f::Zero (); 

    // get neighbors
    // @todo using 5x the scale for searching instead of all the points to avoid O(N^2)
    tree_->radiusSearch ((*interm_cloud_)[i], 5*scale_, nn_indices, nn_distances);

    float theta_normalization_factor = 0.0;
    std::vector<float> theta (nn_indices.size ());
    for (std::size_t nn_index_i = 0; nn_index_i < nn_indices.size (); ++nn_index_i)
    {
      float dist = pcl::squaredEuclideanDistance ((*interm_cloud_)[i], (*input_)[nn_indices[nn_index_i]]);//(*interm_cloud_)[nn_indices[nn_index_i]]);
      float theta_i = std::exp ( (-1) * dist / scale_squared_);
      theta_normalization_factor += theta_i;

      smoothed_normal += theta_i * (*interm_normals_)[nn_indices[nn_index_i]].getNormalVector4fMap ();

      theta[nn_index_i] = theta_i;
    }

    smoothed_normal /= theta_normalization_factor;
    smoothed_normal(3) = 0.0f;
    smoothed_normal.normalize ();


    // find minimum along the normal
    float e_residual;
    smoothed_point = (*interm_cloud_)[i].getVector4fMap ();
    while (true)
    {
      e_residual = 0.0f;
      smoothed_point(3) = 0.0f;
      for (std::size_t nn_index_i = 0; nn_index_i < nn_indices.size (); ++nn_index_i)
      {
        Eigen::Vector4f neighbor = (*input_)[nn_indices[nn_index_i]].getVector4fMap ();//(*interm_cloud_)[nn_indices[nn_index_i]].getVector4fMap ();
        neighbor(3) = 0.0f;
        float dot_product = smoothed_normal.dot (neighbor - smoothed_point);
        e_residual += theta[nn_index_i] * dot_product;// * dot_product;
      }
      e_residual /= theta_normalization_factor;
      if (e_residual < 1e-5) break;

      smoothed_point += e_residual * smoothed_normal;
    }

    total_residual += e_residual;

    (*output_positions)[i].getVector4fMap () = smoothed_point;
    (*output_normals)[i].getNormalVector4fMap () = (*normals_)[i].getNormalVector4fMap ();//smoothed_normal;
  }

//  std::cerr << "Total residual after iteration: " << total_residual << std::endl;
//  PCL_INFO("SurfelSmoothing done iteration\n");
  return total_residual;
}


template <typename PointT, typename PointNT> void
pcl::SurfelSmoothing<PointT, PointNT>::smoothPoint (std::size_t &point_index,
                                                    PointT &output_point,
                                                    PointNT &output_normal)
{
  Eigen::Vector4f average_normal = Eigen::Vector4f::Zero ();
  Eigen::Vector4f result_point = (*input_)[point_index].getVector4fMap ();
  result_point(3) = 0.0f;

  // @todo parameter
  float error_residual_threshold_ = 1e-3f;
  float error_residual = error_residual_threshold_ + 1;
  float last_error_residual = error_residual + 100.0f;

  pcl::Indices nn_indices;
  std::vector<float> nn_distances;

  int big_iterations = 0;
  int max_big_iterations = 500;

  while (std::fabs (error_residual) < std::fabs (last_error_residual) -error_residual_threshold_ &&
         big_iterations < max_big_iterations)
  {
    average_normal = Eigen::Vector4f::Zero ();
    big_iterations ++;
    PointT aux_point; aux_point.x = result_point(0); aux_point.y = result_point(1); aux_point.z = result_point(2);
    tree_->radiusSearch (aux_point, 5*scale_, nn_indices, nn_distances);

    float theta_normalization_factor = 0.0;
    std::vector<float> theta (nn_indices.size ());
    for (std::size_t nn_index_i = 0; nn_index_i < nn_indices.size (); ++nn_index_i)
    {
      float dist = nn_distances[nn_index_i];
      float theta_i = std::exp ( (-1) * dist / scale_squared_);
      theta_normalization_factor += theta_i;

      average_normal += theta_i * (*normals_)[nn_indices[nn_index_i]].getNormalVector4fMap ();
      theta[nn_index_i] = theta_i;
    }
    average_normal /= theta_normalization_factor;
    average_normal(3) = 0.0f;
    average_normal.normalize ();

    // find minimum along the normal
    float e_residual_along_normal = 2, last_e_residual_along_normal = 3;
    int small_iterations = 0;
    int max_small_iterations = 10;
    while ( std::fabs (e_residual_along_normal) < std::fabs (last_e_residual_along_normal) &&
        small_iterations < max_small_iterations)
    {
      small_iterations ++;

      e_residual_along_normal = 0.0f;
      for (std::size_t nn_index_i = 0; nn_index_i < nn_indices.size (); ++nn_index_i)
      {
        Eigen::Vector4f neighbor = (*input_)[nn_indices[nn_index_i]].getVector4fMap ();
        neighbor(3) = 0.0f;
        float dot_product = average_normal.dot (neighbor - result_point);
        e_residual_along_normal += theta[nn_index_i] * dot_product;
      }
      e_residual_along_normal /= theta_normalization_factor;
      if (e_residual_along_normal < 1e-3) break;

      result_point += e_residual_along_normal * average_normal;
    }

//    if (small_iterations == max_small_iterations)
//      PCL_INFO ("passed the number of small iterations %d\n", small_iterations);

    last_error_residual = error_residual;
    error_residual = e_residual_along_normal;

//    PCL_INFO ("last %f    current %f\n", last_error_residual, error_residual);
  }

  output_point.x = result_point(0);
  output_point.y = result_point(1);
  output_point.z = result_point(2);
  output_normal = (*normals_)[point_index];

  if (big_iterations == max_big_iterations)
    PCL_DEBUG ("[pcl::SurfelSmoothing::smoothPoint] Passed the number of BIG iterations: %d\n", big_iterations);
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

  tree_->setInputCloud (input_);

  output_positions->header = input_->header;
  output_positions->height = input_->height;
  output_positions->width = input_->width;

  output_normals->header = input_->header;
  output_normals->height = input_->height;
  output_normals->width = input_->width;

  output_positions->points.resize (input_->size ());
  output_normals->points.resize (input_->size ());
  for (std::size_t i = 0; i < input_->size (); ++i)
  {
    smoothPoint (i, (*output_positions)[i], (*output_normals)[i]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SurfelSmoothing<PointT, PointNT>::extractSalientFeaturesBetweenScales (PointCloudInPtr &cloud2,
                                                                            NormalCloudPtr &cloud2_normals,
                                                                            pcl::IndicesPtr &output_features)
{
  if (interm_cloud_->size () != cloud2->size () || 
      cloud2->size () != cloud2_normals->size ())
  {
    PCL_ERROR ("[pcl::SurfelSmoothing::extractSalientFeaturesBetweenScales]: Number of points in the clouds does not match.\n");
    return;
  }

  std::vector<float> diffs (cloud2->size ());
  for (std::size_t i = 0; i < cloud2->size (); ++i)
    diffs[i] = (*cloud2_normals)[i].getNormalVector4fMap ().dot ((*cloud2)[i].getVector4fMap () - 
                                                                      (*interm_cloud_)[i].getVector4fMap ());

  pcl::Indices nn_indices;
  std::vector<float> nn_distances;

  output_features->resize (cloud2->size ());
  for (int point_i = 0; point_i < static_cast<int> (cloud2->size ()); ++point_i)
  {
    // Get neighbors
    tree_->radiusSearch (point_i, scale_, nn_indices, nn_distances);

    bool largest = true;
    bool smallest = true;
    for (const auto &nn_index : nn_indices)
    {
      if (diffs[point_i] < diffs[nn_index])
        largest = false;
      else 
        smallest = false;
    }

    if (largest || smallest)
      (*output_features)[point_i] = point_i;
  }
}



#define PCL_INSTANTIATE_SurfelSmoothing(PointT,PointNT) template class PCL_EXPORTS pcl::SurfelSmoothing<PointT, PointNT>;

#endif /* PCL_SURFACE_IMPL_SURFEL_SMOOTHING_H_ */

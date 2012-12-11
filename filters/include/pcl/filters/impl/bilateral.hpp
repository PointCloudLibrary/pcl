/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 */

#ifndef PCL_FILTERS_BILATERAL_IMPL_H_
#define PCL_FILTERS_BILATERAL_IMPL_H_

#include <pcl/filters/bilateral.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::BilateralFilter<PointT>::computePointWeight (const int pid, 
                                                  const std::vector<int> &indices,
                                                  const std::vector<float> &distances)
{
  double BF = 0, W = 0;

  // For each neighbor
  for (size_t n_id = 0; n_id < indices.size (); ++n_id)
  {
    int id = indices[n_id];
    // Compute the difference in intensity
    double intensity_dist = fabs (input_->points[pid].intensity - input_->points[id].intensity);

    // Compute the Gaussian intensity weights both in Euclidean and in intensity space
    double dist = std::sqrt (distances[n_id]);
    double weight = kernel (dist, sigma_s_) * kernel (intensity_dist, sigma_r_);

    // Calculate the bilateral filter response
    BF += weight * input_->points[id].intensity;
    W += weight;
  }
  return (BF / W);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::BilateralFilter<PointT>::applyFilter (PointCloud &output)
{
  // Check if sigma_s has been given by the user
  if (sigma_s_ == 0)
  {
    PCL_ERROR ("[pcl::BilateralFilter::applyFilter] Need a sigma_s value given before continuing.\n");
    return;
  }
  // In case a search method has not been given, initialize it using some defaults
  if (!tree_)
  {
    // For organized datasets, use an OrganizedDataIndex
    if (input_->isOrganized ())
      tree_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    // For unorganized data, use a FLANN kdtree
    else
      tree_.reset (new pcl::search::KdTree<PointT> (false));
  }
  tree_->setInputCloud (input_);

  std::vector<int> k_indices;
  std::vector<float> k_distances;

  // Copy the input data into the output
  output = *input_;

  // For all the indices given (equal to the entire cloud if none given)
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Perform a radius search to find the nearest neighbors
    tree_->radiusSearch ((*indices_)[i], sigma_s_ * 2, k_indices, k_distances);

    // Overwrite the intensity value with the computed average
    output.points[(*indices_)[i]].intensity = static_cast<float> (computePointWeight ((*indices_)[i], k_indices, k_distances));
  }
}
 
#define PCL_INSTANTIATE_BilateralFilter(T) template class PCL_EXPORTS pcl::BilateralFilter<T>;

#endif // PCL_FILTERS_BILATERAL_H_


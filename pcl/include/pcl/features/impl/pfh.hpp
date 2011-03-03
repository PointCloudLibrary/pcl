/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: pfh.hpp 35826 2011-02-08 00:59:32Z rusu $
 *
 */

#ifndef PCL_FEATURES_IMPL_PFH_H_
#define PCL_FEATURES_IMPL_PFH_H_

#include "pcl/features/pfh.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
pcl::PFHEstimation<PointInT, PointNT, PointOutT>::computePairFeatures (
      const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
      int p_idx, int q_idx, float &f1, float &f2, float &f3, float &f4)
{
  // Compute the Cartesian difference between the two points
  Eigen::Vector4f delta = cloud.points[q_idx].getVector4fMap () - cloud.points[p_idx].getVector4fMap ();
  delta[3] = 0;

  // Compute the Euclidean norm = || p_idx - q_idx ||
  float distance_sqr = delta.squaredNorm ();

  if (distance_sqr == 0)
  {
    ROS_ERROR ("Euclidean distance between points %d and %d is 0!", p_idx, q_idx);
    f1 = f2 = f3 = f4 = 0;
    return (false);
  }

  // Estimate f4 = || delta ||
  f4 = sqrt (distance_sqr);

  // Create a Darboux frame coordinate system u-v-w
  // u = n1; v = (p_idx - q_idx) x u / || (p_idx - q_idx) x u ||; w = u x v
  pcl::Vector4fMapConst u = normals.points[p_idx].getNormalVector4fMap ();

  // Estimate f3 = u * delta / || delta ||
  // delta[3] = 0 (line 59)
  f3 = u.dot (delta) / f4;

  // v = delta * u
  Eigen::Vector4f v = Eigen::Vector4f::Zero ();
  v = delta.cross3 (u);

  distance_sqr = v.squaredNorm ();
  if (distance_sqr == 0)
  {
    ROS_ERROR ("Norm of Delta x U is 0 for point %d and %d!", p_idx, q_idx);
    f1 = f2 = f3 = f4 = 0;
    return (false);
  }

  // Copy the q_idx normal
  Eigen::Vector4f nq (normals.points[q_idx].normal_x, 
                      normals.points[q_idx].normal_y,
                      normals.points[q_idx].normal_z,
                      0);

  // Normalize the vector
  v /= sqrt (distance_sqr);

  // Compute delta (w) = u x v
  delta = u.cross3 (v);

  // Compute f2 = v * n2;
  // v[3] = 0 (line 82)
  f2 = v.dot (nq);

  // Compute f1 = arctan (w * n2, u * n2) i.e. angle of n2 in the x=u, y=w coordinate system
  // delta[3] = 0 (line 59), nq[3] = 0 (line 97)
  f1 = atan2f (delta.dot (nq), u.dot (nq));       // @todo: optimize this

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PFHEstimation<PointInT, PointNT, PointOutT>::computePointPFHSignature (
      const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
      const std::vector<int> &indices, int nr_split, Eigen::VectorXf &pfh_histogram)
{
  int h_index, h_p;

  // Clear the resultant point histogram
  pfh_histogram.setZero ();

  // Factorization constant
  float hist_incr = 100.0 / (indices.size () * indices.size () - 1);

  // Iterate over all the points in the neighborhood
  for (size_t i_idx = 0; i_idx < indices.size (); ++i_idx)
  {
    for (size_t j_idx = 0; j_idx < indices.size (); ++j_idx)
    {
      // Avoid unnecessary returns
      if (i_idx == j_idx)
        continue;

      // Compute the pair NNi to NNj
      if (!computePairFeatures (cloud, normals, indices[i_idx], indices[j_idx],
                                pfh_tuple_[0], pfh_tuple_[1], pfh_tuple_[2], pfh_tuple_[3]))
        continue;

      // Normalize the f1, f2, f3 features and push them in the histogram
      f_index_[0] = floor (nr_split * ((pfh_tuple_[0] + M_PI) * d_pi_));
      if (f_index_[0] < 0)         f_index_[0] = 0;
      if (f_index_[0] >= nr_split) f_index_[0] = nr_split - 1;

      f_index_[1] = floor (nr_split * ((pfh_tuple_[1] + 1.0) * 0.5));
      if (f_index_[1] < 0)         f_index_[1] = 0;
      if (f_index_[1] >= nr_split) f_index_[1] = nr_split - 1;

      f_index_[2] = floor (nr_split * ((pfh_tuple_[2] + 1.0) * 0.5));
      if (f_index_[2] < 0)         f_index_[2] = 0;
      if (f_index_[2] >= nr_split) f_index_[2] = nr_split - 1;

      // Copy into the histogram
      h_index = 0;
      h_p     = 1;
      for (int d = 0; d < 3; ++d)
      {
        h_index += h_p * f_index_[d];
        h_p     *= nr_split;
      }
      pfh_histogram[h_index] += hist_incr;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PFHEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Check if input was set
  if (!normals_)
  {
    ROS_ERROR ("[pcl::%s::computeFeature] No input dataset containing normals was given!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  if (normals_->points.size () != surface_->points.size ())
  {
    ROS_ERROR ("[pcl::%s::computeFeature] The number of points in the input dataset differs from the number of points in the dataset containing the normals!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  pfh_histogram_.setZero (nr_subdiv_ * nr_subdiv_ * nr_subdiv_);

  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
  {
    searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    // Estimate the PFH signature at each patch
    computePointPFHSignature (*surface_, *normals_, nn_indices, nr_subdiv_, pfh_histogram_);

    // Copy into the resultant cloud
    for (int d = 0; d < pfh_histogram_.size (); ++d)
      output.points[idx].histogram[d] = pfh_histogram_[d];
  }
}

#define PCL_INSTANTIATE_PFHEstimation(T,NT,OutT) template class pcl::PFHEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_PFH_H_ 


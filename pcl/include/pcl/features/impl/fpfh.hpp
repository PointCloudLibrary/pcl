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
 * $Id: fpfh.hpp 35826 2011-02-08 00:59:32Z rusu $
 *
 */

#ifndef PCL_FEATURES_IMPL_FPFH_H_
#define PCL_FEATURES_IMPL_FPFH_H_

#include "pcl/features/fpfh.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
pcl::FPFHEstimation<PointInT, PointNT, PointOutT>::computePairFeatures (
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
pcl::FPFHEstimation<PointInT, PointNT, PointOutT>::computePointSPFHSignature (
      const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
      int p_idx, const std::vector<int> &indices,
      Eigen::MatrixXf &hist_f1, Eigen::MatrixXf &hist_f2, Eigen::MatrixXf &hist_f3)
{
  Eigen::Vector4f pfh_tuple;
  // Get the number of bins from the histograms size
  int nr_bins_f1 = hist_f1.cols ();
  int nr_bins_f2 = hist_f2.cols ();
  int nr_bins_f3 = hist_f3.cols ();

  // Factorization constant
  float hist_incr = 100.0 / (float)(indices.size () - 1);

  // Iterate over all the points in the neighborhood
  for (size_t idx = 0; idx < indices.size (); ++idx)
  {
    // Avoid unnecessary returns
    if (p_idx == indices[idx])
      continue;

    // Compute the pair P to NNi
    if (!computePairFeatures (cloud, normals, p_idx, indices[idx], pfh_tuple[0], pfh_tuple[1], pfh_tuple[2], pfh_tuple[3]))
      continue;

    // Normalize the f1, f2, f3 features and push them in the histogram
    int h_index = floor (nr_bins_f1 * ((pfh_tuple[0] + M_PI) * d_pi_));
    if (h_index < 0)           h_index = 0;
    if (h_index >= nr_bins_f1) h_index = nr_bins_f1 - 1;
    hist_f1 (p_idx, h_index) += hist_incr;

    h_index = floor (nr_bins_f2 * ((pfh_tuple[1] + 1.0) * 0.5));
    if (h_index < 0)           h_index = 0;
    if (h_index >= nr_bins_f2) h_index = nr_bins_f2 - 1;
    hist_f2 (p_idx, h_index) += hist_incr;

    h_index = floor (nr_bins_f3 * ((pfh_tuple[2] + 1.0) * 0.5));
    if (h_index < 0)           h_index = 0;
    if (h_index >= nr_bins_f3) h_index = nr_bins_f3 - 1;
    hist_f3 (p_idx, h_index) += hist_incr;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::FPFHEstimation<PointInT, PointNT, PointOutT>::weightPointSPFHSignature (
      const Eigen::MatrixXf &hist_f1, const Eigen::MatrixXf &hist_f2, const Eigen::MatrixXf &hist_f3,
      const std::vector<int> &indices, const std::vector<float> &dists, Eigen::VectorXf &fpfh_histogram)
{
  assert (indices.size () == dists.size ());
  double sum_f1 = 0.0, sum_f2 = 0.0, sum_f3 = 0.0;
  float weight = 0.0, val_f1, val_f2, val_f3;

  // Get the number of bins from the histograms size
  int nr_bins_f1 = hist_f1.cols ();
  int nr_bins_f2 = hist_f2.cols ();
  int nr_bins_f3 = hist_f3.cols ();
  int nr_bins_f12 = nr_bins_f1 + nr_bins_f2;

  // Use the entire patch
  for (size_t idx = 0, data_size = indices.size (); idx < data_size; ++idx)
  {
    // Minus the query point itself
    if (dists[idx] == 0)
      continue;

    // Standard weighting function used
    weight = 1.0 / dists[idx];

    // Weight the SPFH of the query point with the SPFH of its neighbors
    for (int f1_i = 0; f1_i < nr_bins_f1; ++f1_i)
    {
      val_f1 = hist_f1 (indices[idx], f1_i) * weight;
      sum_f1 += val_f1;
      fpfh_histogram[f1_i] += val_f1;
    }

    for (int f2_i = 0; f2_i < nr_bins_f2; ++f2_i)
    {
      val_f2 = hist_f2 (indices[idx], f2_i) * weight;
      sum_f2 += val_f2;
      fpfh_histogram[f2_i + nr_bins_f1] += val_f2;
    }

    for (int f3_i = 0; f3_i < nr_bins_f3; ++f3_i)
    {
      val_f3 = hist_f3 (indices[idx], f3_i) * weight;
      sum_f3 += val_f3;
      fpfh_histogram[f3_i + nr_bins_f12] += val_f3;
    }
  }

  if (sum_f1 != 0)
    sum_f1 = 100.0 / sum_f1;           // histogram values sum up to 100
  if (sum_f2 != 0)
    sum_f2 = 100.0 / sum_f2;           // histogram values sum up to 100
  if (sum_f3 != 0)
    sum_f3 = 100.0 / sum_f3;           // histogram values sum up to 100

  // Adjust final FPFH values
  for (int f1_i = 0; f1_i < nr_bins_f1; ++f1_i)
    fpfh_histogram[f1_i] *= sum_f1;
  for (int f2_i = 0; f2_i < nr_bins_f2; ++f2_i)
    fpfh_histogram[f2_i + nr_bins_f1] *= sum_f2;
  for (int f3_i = 0; f3_i < nr_bins_f3; ++f3_i)
    fpfh_histogram[f3_i + nr_bins_f12] *= sum_f3;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::FPFHEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
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

  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  size_t data_size = indices_->size ();
  // Reset the whole thing
  hist_f1_.setZero (data_size, nr_bins_f1_);
  hist_f2_.setZero (data_size, nr_bins_f2_);
  hist_f3_.setZero (data_size, nr_bins_f3_);

  int nr_bins = nr_bins_f1_ + nr_bins_f2_ + nr_bins_f3_;

  // Iterating over the entire index vector
  for (size_t idx = 0; idx < data_size; ++idx)
  {
    int p_idx = (*indices_)[idx];
    searchForNeighbors (p_idx, search_parameter_, nn_indices, nn_dists);

    // Estimate the FPFH signature at each patch
    computePointSPFHSignature (*surface_, *normals_, p_idx, nn_indices,
                               hist_f1_, hist_f2_, hist_f3_);
  }

  fpfh_histogram_.setZero (nr_bins);
  // Iterating over the entire index vector
  for (size_t idx = 0; idx < data_size; ++idx)
  {
    searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    weightPointSPFHSignature (hist_f1_, hist_f2_, hist_f3_, nn_indices, nn_dists, fpfh_histogram_);

    // Copy into the resultant cloud
    for (int d = 0; d < fpfh_histogram_.size (); ++d)
      output.points[idx].histogram[d] = fpfh_histogram_[d];
    fpfh_histogram_.setZero ();
  }

}

#define PCL_INSTANTIATE_FPFHEstimation(T,NT,OutT) template class pcl::FPFHEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_FPFH_H_ 


/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *  $Id$
 */

#ifndef PCL_FEATURES_IMPL_PFHRGB_H_
#define PCL_FEATURES_IMPL_PFHRGB_H_

#include <pcl/features/pfhrgb.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
pcl::PFHRGBEstimation<PointInT, PointNT, PointOutT>::computeRGBPairFeatures (
    const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
    int p_idx, int q_idx,
    float &f1, float &f2, float &f3, float &f4, float &f5, float &f6, float &f7)
{
  Eigen::Vector4i colors1 (cloud[p_idx].r, cloud[p_idx].g, cloud[p_idx].b, 0),
      colors2 (cloud[q_idx].r, cloud[q_idx].g, cloud[q_idx].b, 0);
  pcl::computeRGBPairFeatures (cloud[p_idx].getVector4fMap (), normals[p_idx].getNormalVector4fMap (),
                               colors1,
                               cloud[q_idx].getVector4fMap (), normals[q_idx].getNormalVector4fMap (),
                               colors2,
                               f1, f2, f3, f4, f5, f6, f7);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PFHRGBEstimation<PointInT, PointNT, PointOutT>::computePointPFHRGBSignature (
    const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
    const pcl::Indices &indices, int nr_split, Eigen::VectorXf &pfhrgb_histogram)
{
  int h_index, h_p;

  // Clear the resultant point histogram
  pfhrgb_histogram.setZero ();

  // Factorization constant
  float hist_incr = 100.0f / static_cast<float> (indices.size () * (indices.size () - 1) / 2);

  // Iterate over all the points in the neighborhood
  for (const auto& index_i: indices)
  {
    for (const auto& index_j: indices)
    {
      // Avoid unnecessary returns
      if (index_i == index_j)
        continue;

      // Compute the pair NNi to NNj
      if (!computeRGBPairFeatures (cloud, normals, index_i, index_j,
                                   pfhrgb_tuple_[0], pfhrgb_tuple_[1], pfhrgb_tuple_[2], pfhrgb_tuple_[3],
                                   pfhrgb_tuple_[4], pfhrgb_tuple_[5], pfhrgb_tuple_[6]))
        continue;

      // Normalize the f1, f2, f3, f5, f6, f7 features and push them in the histogram
      f_index_[0] = static_cast<int> (std::floor (nr_split * ((pfhrgb_tuple_[0] + M_PI) * d_pi_)));
      // @TODO: confirm "not to do for i == 3"
      for (int i = 1; i < 3; ++i)
      {
        const float feature_value = nr_split * ((pfhrgb_tuple_[i] + 1.0) * 0.5);
        f_index_[i] = static_cast<int> (std::floor (feature_value));
      }
      // color ratios are in [-1, 1]
      for (int i = 4; i < 7; ++i)
      {
        const float feature_value = nr_split * ((pfhrgb_tuple_[i] + 1.0) * 0.5);
        f_index_[i] = static_cast<int> (std::floor (feature_value));
      }
      for (auto& feature: f_index_)
      {
        feature = std::min(nr_split - 1, std::max(0, feature));
      }

      // Copy into the histogram
      h_index = 0;
      h_p     = 1;
      for (int d = 0; d < 3; ++d)
      {
        h_index += h_p * f_index_[d];
        h_p     *= nr_split;
      }
      pfhrgb_histogram[h_index] += hist_incr;

      // and the colors
      h_index = 125;
      h_p     = 1;
      for (int d = 4; d < 7; ++d)
      {
        h_index += h_p * f_index_[d];
        h_p     *= nr_split;
      }
      pfhrgb_histogram[h_index] += hist_incr;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PFHRGBEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  /// nr_subdiv^3 for RGB and nr_subdiv^3 for the angular features
  pfhrgb_histogram_.setZero (2 * nr_subdiv_ * nr_subdiv_ * nr_subdiv_);
  pfhrgb_tuple_.setZero (7);

  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  pcl::Indices nn_indices (k_);
  std::vector<float> nn_dists (k_);

  // Iterating over the entire index vector
  for (std::size_t idx = 0; idx < indices_->size (); ++idx)
  {
    this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    // Estimate the PFH signature at each patch
    computePointPFHRGBSignature (*surface_, *normals_, nn_indices, nr_subdiv_, pfhrgb_histogram_);

    std::copy (pfhrgb_histogram_.data (), pfhrgb_histogram_.data () + pfhrgb_histogram_.size (),
                 output[idx].histogram);
  }
}

#define PCL_INSTANTIATE_PFHRGBEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::PFHRGBEstimation<T,NT,OutT>;

#endif /* PCL_FEATURES_IMPL_PFHRGB_H_ */

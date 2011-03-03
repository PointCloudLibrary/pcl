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
 * $Id: normal_3d_tbb.hpp 35810 2011-02-08 00:03:46Z rusu $
 *
 */

#ifndef PCL_FEATURES_IMPL_NORMAL_3D_TBB_H_
#define PCL_FEATURES_IMPL_NORMAL_3D_TBB_H_

#include "pcl/features/normal_3d_tbb.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::TBB_NormalEstimationTBB<PointInT, PointOutT>::operator () (const tbb::blocked_range <size_t> &r) const
{
  float vpx, vpy, vpz;
  feature_->getViewPoint (vpx, vpy, vpz);
  // Iterating over the entire index vector
  for (size_t idx = r.begin (); idx != r.end (); ++idx)
  {
    std::vector<int> nn_indices (feature_->getKSearch ());
    std::vector<float> nn_dists (feature_->getKSearch ());

    feature_->searchForNeighbors ((*feature_->getIndices ())[idx], feature_->getSearchParameter (), nn_indices, nn_dists);

    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;
    // Estimate the XYZ centroid
    compute3DCentroid (*feature_->getSearchSurface (), nn_indices, xyz_centroid);

    // Placeholder for the 3x3 covariance matrix at each surface patch
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    // Compute the 3x3 covariance matrix
    computeCovarianceMatrix (*feature_->getSearchSurface (), nn_indices, xyz_centroid, covariance_matrix);

    // Get the plane normal and surface curvature
    solvePlaneParameters (covariance_matrix,
                          output_.points[idx].normal[0], output_.points[idx].normal[1], output_.points[idx].normal[2], output_.points[idx].curvature);

    flipNormalTowardsViewpoint<PointInT> (feature_->getSearchSurface ()->points[idx], vpx, vpy, vpz,
                                          output_.points[idx].normal[0], output_.points[idx].normal[1], output_.points[idx].normal[2]);
  }
}

#define PCL_INSTANTIATE_TBB_NormalEstimationTBB(T,NT) template class pcl::TBB_NormalEstimationTBB<T,NT>;
#define PCL_INSTANTIATE_NormalEstimationTBB(T,NT) template class pcl::NormalEstimationTBB<T,NT>;

#endif    // PCL_FEATURES_IMPL_NORMAL_3D_TBB_H_


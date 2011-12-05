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
 *
 */

#ifndef PCL_SURFACE_IMPL_MLS_OMP_H_
#define PCL_SURFACE_IMPL_MLS_OMP_H_

#include <pcl/surface/mls_omp.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename NormalOutT> void
pcl::MovingLeastSquaresOMP<PointInT, NormalOutT>::performReconstruction (PointCloudIn &output)
{
  // Compute the number of coefficients
  nr_coeff_ = (order_ + 1) * (order_ + 2) / 2;

#pragma omp parallel for schedule (dynamic, threads_)
  // For all points
  for (size_t cp = 0; cp < indices_->size (); ++cp)
  {
    // Allocate enough space to hold the results of nearest neighbor searches
    // \note resize is irrelevant for a radiusSearch ().
    std::vector<int> nn_indices;
    std::vector<float> nn_sqr_dists;

    // Get the initial estimates of point positions and their neighborhoods
    if (!searchForNeighbors ((*indices_)[cp], nn_indices, nn_sqr_dists))
    {
      if (normals_)
        normals_->points[cp].normal[0] = normals_->points[cp].normal[1] = normals_->points[cp].normal[2] = normals_->points[cp].curvature = std::numeric_limits<float>::quiet_NaN ();
      continue;
    }

    // Check the number of nearest neighbors for normal estimation (and later
    // for polynomial fit as well)
    if (nn_indices.size () < 3)
      continue;

    Eigen::Vector4f model_coefficients;
    // Get a plane approximating the local surface's tangent and project point onto it
    computeMLSPointNormal (output.points[cp], *input_, nn_indices, nn_sqr_dists,
                           model_coefficients); 

    // Save results to output cloud
    if (normals_)
    {
      normals_->points[cp].normal[0] = model_coefficients[0];
      normals_->points[cp].normal[1] = model_coefficients[1];
      normals_->points[cp].normal[2] = model_coefficients[2];
      normals_->points[cp].curvature = model_coefficients[3];
    }
  }
}

#define PCL_INSTANTIATE_MovingLeastSquaresOMP(T,OutT) template class PCL_EXPORTS pcl::MovingLeastSquaresOMP<T,OutT>;

#endif    // PCL_SURFACE_IMPL_MLS_OMP_H_


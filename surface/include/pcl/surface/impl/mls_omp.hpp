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
 * $Id: mls_omp.hpp 5835 2012-06-04 05:27:21Z holzers $
 *
 */

#ifndef PCL_SURFACE_IMPL_MLS_OMP_H_
#define PCL_SURFACE_IMPL_MLS_OMP_H_

#include <cstddef>
#include <pcl/surface/mls_omp.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquaresOMP<PointInT, PointOutT>::performProcessing (PointCloudOut &output)
{
  typedef std::size_t size_t;
  // Compute the number of coefficients
  nr_coeff_ = (order_ + 1) * (order_ + 2) / 2;

#pragma omp parallel for schedule (dynamic, threads_)
  // For all points
  for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
  {
    // Allocate enough space to hold the results of nearest neighbor searches
    // \note resize is irrelevant for a radiusSearch ().
    std::vector<int> nn_indices;
    std::vector<float> nn_sqr_dists;

    // Get the initial estimates of point positions and their neighborhoods
    if (!searchForNeighbors (cp, nn_indices, nn_sqr_dists))
      continue;


    // Check the number of nearest neighbors for normal estimation (and later
    // for polynomial fit as well)
    if (nn_indices.size () < 3)
      continue;


    PointCloudOut projected_points;
    NormalCloud projected_points_normals;

    // Get a plane approximating the local surface's tangent and project point onto it
    this->computeMLSPointNormal (cp, *input_, nn_indices, nn_sqr_dists, projected_points, projected_points_normals);

#pragma omp critical
    {
      // Append projected points to output
      output.insert (output.end (), projected_points.begin (), projected_points.end ());
      if (compute_normals_)
        normals_->insert (normals_->end (), projected_points_normals.begin (), projected_points_normals.end ());
    }
  }


  // For the voxel grid upsampling method, generate the voxel grid and dilate it
  // Then, project the newly obtained points to the MLS surface
  if (upsample_method_ == MovingLeastSquares<PointInT, PointOutT>::VOXEL_GRID_DILATION)
  {
    MLSVoxelGrid voxel_grid (input_, indices_, voxel_size_);

    for (int iteration = 0; iteration < dilation_iteration_num_; ++iteration)
      voxel_grid.dilate ();

#if /*defined(_WIN32) ||*/ ((__GNUC__ > 4) && (__GNUC_MINOR__ > 2))
#pragma omp parallel for schedule (dynamic, threads_)
#endif
    for (typename MLSVoxelGrid::HashMap::iterator h_it = voxel_grid.voxel_grid_.begin (); h_it != voxel_grid.voxel_grid_.end (); ++h_it)
    {
      typename MLSVoxelGrid::HashMap::value_type voxel = *h_it;

      // Get 3D position of point
      Eigen::Vector3f pos;
      voxel_grid.getPosition (voxel.first, pos);

      PointInT p;
      p.x = pos[0];
      p.y = pos[1];
      p.z = pos[2];

      std::vector<int> nn_indices;
      std::vector<float> nn_dists;
      tree_->nearestKSearch (p, 1, nn_indices, nn_dists);
      int input_index = nn_indices.front ();

      // If the closest point did not have a valid MLS fitting result
      // OR if it is too far away from the sampled point
      if (mls_results_[input_index].valid == false)
        continue;

      Eigen::Vector3f add_point = p.getVector3fMap (),
                      input_point = input_->points[input_index].getVector3fMap ();

      Eigen::Vector3d aux = mls_results_[input_index].u;
      Eigen::Vector3f u = aux.cast<float> ();
      aux = mls_results_[input_index].v;
      Eigen::Vector3f v = aux.cast<float> ();

      float u_disp = (add_point - input_point).dot (u),
            v_disp = (add_point - input_point).dot (v);

      PointOutT result_point;
      pcl::Normal result_normal;
      this->projectPointToMLSSurface (u_disp, v_disp,
                                      mls_results_[input_index].u, mls_results_[input_index].v,
                                      mls_results_[input_index].plane_normal,
                                      mls_results_[input_index].curvature,
                                      input_point,
                                      mls_results_[input_index].c_vec,
                                      mls_results_[input_index].num_neighbors,
                                      result_point, result_normal);

      float d_before = (pos - input_point).norm (),
            d_after = (result_point.getVector3fMap () - input_point). norm();
      if (d_after > d_before)
        continue;

#pragma critical
      {
        output.push_back (result_point);
        if (compute_normals_)
          normals_->push_back (result_normal);
      }
    }
  }
}

#define PCL_INSTANTIATE_MovingLeastSquaresOMP(T,OutT) template class PCL_EXPORTS pcl::MovingLeastSquaresOMP<T,OutT>;

#endif    // PCL_SURFACE_IMPL_MLS_OMP_H_

